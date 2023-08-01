#ifndef VICON_BRIDGE_HPP
#define VICON_BRIDGE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

// Vicon
#include <ViconDataStreamSDK_CPP/DataStreamClient.h>

namespace vicon_bridge {

	using namespace ViconDataStreamSDK::CPP;

        class SegmentPublisher {
        public:
          rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr
              pub;
          bool is_ready = false;

        }; // class Segment Publisher;

        typedef std::map<std::string, SegmentPublisher> SegmentMap;

        class ViconBridge : public rclcpp::Node {

	public:
		ViconBridge();

	private:

		// functions
		bool init_vicon();
		void get_parameters();
                void timer_callback();
                bool get_transform(geometry_msgs::msg::TransformStamped &msg,
                                   const rclcpp::Time &now,
                                   std::string subjectName,
                                   std::string segmentName);
                void print_drop_rate();
                void create_segment_thread(const std::string subject,
                                           const std::string segment);
                void create_segment(const std::string subject,
                                    const std::string segment);
                void process_frame(rclcpp::Time &grab_time);
                void process_specific_segment(const rclcpp::Time &frame_time);
                void process_all_segments(const rclcpp::Time &frame_time);

                // parameters
		std::string host_name_ = "192.168.2.136:801";
		std::string stream_mode_ = "ClientPull";
                double update_rate_hz_ = 250.0;
                bool publish_specific_segment_ = false;
		std::string target_subject_name_ = "px4_1";
		std::string target_segment_name_ = "px4_1";
                std::string world_frame_id_ = "world";
                std::string tf_namespace_ = "vicon";

                // vars
                Client client_;
                rclcpp::TimerBase::SharedPtr timer_;
		std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
		const rclcpp::Time start_time_;
		std::size_t drop_count_ = 0;
                std::size_t frame_count_ = 0;
                std::size_t last_frame_number_ = 0;

                SegmentMap segment_publishers_;
                boost::mutex segments_mutex_;

}; // class ViconBridge


} // namespace vicon_bridge



#endif
