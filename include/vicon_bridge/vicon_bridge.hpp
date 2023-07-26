#ifndef VICON_BRIDGE_HPP
#define VICON_BRIDGE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
//#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


// Vicon
#include <ViconDataStreamSDK_CPP/DataStreamClient.h>
#include <ViconDataStreamSDK_CPP/DataStreamRetimingClient.h>

namespace vicon_bridge {

	using namespace ViconDataStreamSDK::CPP;

class ViconBridge : public rclcpp::Node
{

	public:
		ViconBridge();

	private:

		// functions
		bool init_vicon();
		void get_parameters();
		void timer_callback_all();
		void timer_callback_specific();
		void publish_pose(const rclcpp::Time & now, const String& subjectName, const String& segmentName, double* quaternion, double* translation_mm);
		void print_drop_rate();

		// parameters
		std::string host_name_ = "192.168.2.136:801";
		std::string stream_mode_ = "ClientPull";
		double max_prediction_ms_ = 100.0;
		double update_rate_hz_ = 100.0; 
		bool publish_specific_segment_ = false;
		std::string target_subject_name_ = "px4_1";
		std::string target_segment_name_ = "px4_1";
		std::string world_frame_id_ = "vicon/world";

		// vars
		RetimingClient client_;
		rclcpp::TimerBase::SharedPtr timer_;
		std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
		const rclcpp::Time start_time_;
		std::size_t drop_count_ = 0;




}; // class ViconBridge


} // namespace vicon_bridge



#endif