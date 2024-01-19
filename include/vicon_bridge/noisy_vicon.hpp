#ifndef NOISY_VICON_HPP
#define NOISY_VICON_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

// Perturb
#include "vicon_bridge/perturb_transform.hpp"

// EIGEN
#include <Eigen/Geometry>

// ROS2
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace noisy_vicon {

typedef Eigen::Transform<double, 3, Eigen::Isometry> ETransform;

class NoisyVicon : public rclcpp::Node {

public:
  NoisyVicon();

private:
  // functions
  void get_parameters();
  void timer_callback();
  void transform_callback(geometry_msgs::msg::TransformStamped::SharedPtr msg);
  void pearls_callback(sensor_msgs::msg::PointCloud::SharedPtr msg);
  void publish_transform();
  void publish_pearls();

  // parameters
  double update_rate_hz_ = 30.0;
  double epsilon_R_ = 0;
  double epsilon_t_ = 0;

  // vars
  bool is_first_msg_ = true;
  rclcpp::TimerBase::SharedPtr timer_;
  const rclcpp::Time start_time_;
  rclcpp::Time current_transform_time_;
  rclcpp::Time last_transform_time_;

  std::string world_frame_id_;

  ETransform hat_transform_;     // this is the estimated/noisy transform
  ETransform last_transform_;    // last published transform
  ETransform current_transform_; // last transform received

  sensor_msgs::msg::PointCloud::SharedPtr current_pearls_pointcloud_;

  // subs
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_pearls_;

  // pubs
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pub_pose_with_cov_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_noisy_pearls_;

}; // class ViconBridge

} // namespace noisy_vicon

#endif // NOISY_VICON
