#ifndef VICON_BRIDGE_HPP
#define VICON_BRIDGE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS2
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

// Vicon
#include <ViconDataStreamSDK_CPP/DataStreamClient.h>

namespace vicon_bridge {

using namespace ViconDataStreamSDK::CPP;

class SegmentPublisher {
public:
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_poseMsg;
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
  bool get_transform_msg(geometry_msgs::msg::TransformStamped &msg,
                         const rclcpp::Time &now, std::string subjectName,
                         std::string segmentName);
  void print_drop_rate();
  void create_segment_thread(const std::string subject,
                             const std::string segment);
  void create_segment(const std::string subject, const std::string segment);
  void process_frame(rclcpp::Time &grab_time);
  void process_specific_segment(const rclcpp::Time &frame_time);
  void process_all_segments(const rclcpp::Time &frame_time);
  geometry_msgs::msg::PoseStamped
  transform2pose(geometry_msgs::msg::TransformStamped &transformMsg);

  // parameters
  std::string host_name_ = "192.168.1.164:801";
  std::string stream_mode_ = "ServerPush";
  double update_rate_hz_ = 250.0;
  double expected_rate_hz_ = 100.0;
  bool publish_specific_segment_ = false;
  std::string target_subject_name_ = "";
  std::string target_segment_name_ = "";
  std::string world_frame_id_ = "world";
  std::string tf_namespace_ = "vicon";

  // vars
  Client client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  const rclcpp::Time start_time_;
  std::size_t drop_count_ = 0;
  std::size_t frame_count_ = 0;
  std::size_t first_frame_number_ = 0;
  std::size_t last_frame_number_ = 0;
  bool first_frame_ = true;
  std::shared_ptr<diagnostic_updater::Updater> updater_ptr_;
  std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> pub_freq_ptr_;

  SegmentMap segment_publishers_;
  boost::mutex segments_mutex_;

  double tolerance_ = 0.1;
  int window_ = 100;

}; // class ViconBridge

} // namespace vicon_bridge

#endif
