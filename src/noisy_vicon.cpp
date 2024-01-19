#include "vicon_bridge/noisy_vicon.hpp"
#include <iostream>

namespace noisy_vicon {

using namespace std::chrono_literals;
using namespace geometry_msgs::msg;
using std::placeholders::_1;

NoisyVicon::NoisyVicon()
    : Node("noisy_vicon"), start_time_(this->get_clock()->now()) {

  get_parameters();

  // initialize the publishers
  pub_pose_ = this->create_publisher<PoseStamped>("pose", 10);
  pub_pose_with_cov_ = this->create_publisher<PoseWithCovarianceStamped>(
      "pose_with_covariance", 10);
  pub_noisy_pearls_ =
      this->create_publisher<sensor_msgs::msg::PointCloud>("noisy_pearls", 2);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // initialize the subscribers
  sub_ = this->create_subscription<TransformStamped>(
      "transform", 10, std::bind(&NoisyVicon::transform_callback, this, _1));

  sub_pearls_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
      "/vicon/unlabeledMarkers", 10,
      std::bind(&NoisyVicon::pearls_callback, this, _1));

  // start timer
  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_hz_),
      std::bind(&NoisyVicon::timer_callback, this));
}

void NoisyVicon::get_parameters() {

  update_rate_hz_ =
      this->declare_parameter<double>("update_rate_hz", update_rate_hz_);
  epsilon_R_ =
      this->declare_parameter<double>("epsilon_R_per_frame", epsilon_R_);
  epsilon_t_ =
      this->declare_parameter<double>("epsilon_t_per_frame", epsilon_t_);
}

void NoisyVicon::transform_callback(TransformStamped::SharedPtr msg) {

  // extract the transform to eigen
  Eigen::Quaterniond q(msg->transform.rotation.w, msg->transform.rotation.x,
                       msg->transform.rotation.y, msg->transform.rotation.z);

  Eigen::Vector3d t(msg->transform.translation.x, msg->transform.translation.y,
                    msg->transform.translation.z);

  // save it as the current transform
  current_transform_ =
      perturb::fromRotationTranslation<double>(q.inverse(), -t).inverse();
  current_transform_time_ = msg->header.stamp;
  world_frame_id_ = msg->header.frame_id;

  if (is_first_msg_) {
    last_transform_ = current_transform_;
    last_transform_time_ = current_transform_time_;
    hat_transform_ = last_transform_; // set up the initial transform too
    is_first_msg_ = false;
    // std::cout << hat_transform_.matrix() << std::endl;
    // exit(0);
  }
}

void NoisyVicon::pearls_callback(sensor_msgs::msg::PointCloud::SharedPtr msg) {

  current_pearls_pointcloud_ = msg;
}

void NoisyVicon::timer_callback() {
  // exit early if we havent received any msgs yet
  if (is_first_msg_)
    return;

  if (epsilon_R_ == 0 && epsilon_t_ == 0) {
    hat_transform_ = current_transform_;
  } else {
    // compute the true relative transform
    ETransform R_kkm1 = current_transform_ * (last_transform_.inverse());

    // get perturbed relative transform
    ETransform hat_Rkkm1 = perturb::perturb(R_kkm1, epsilon_R_, epsilon_t_);

    // multiply it onto the last published transform
    hat_transform_ = hat_Rkkm1 * hat_transform_;
  }

  // publish
  publish_transform();
  publish_pearls();

  // update the last_transforms
  last_transform_ = current_transform_;
  last_transform_time_ = current_transform_time_;
}

void NoisyVicon::publish_pearls() {

  if (!current_pearls_pointcloud_) {
    return;
  }

  // compute the total drift (drifted_pose = drift * true_pose)
  auto total_drift = hat_transform_ * current_transform_.inverse();

  // apply this transform to all the pearls
  sensor_msgs::msg::PointCloud msg;
  msg.header.frame_id = "mapping";
  msg.header.stamp = current_pearls_pointcloud_->header.stamp;

  for (auto &pt : current_pearls_pointcloud_->points) {

    Eigen::Vector4d v(pt.x, pt.y, pt.z, 1.0);
    Eigen::Vector4d new_v = total_drift * v; // apply the drift

    geometry_msgs::msg::Point32 new_pt;
    new_pt.x = new_v(0);
    new_pt.y = new_v(1);
    new_pt.z = new_v(2);

    msg.points.push_back(new_pt);
  }

  // publish
  pub_noisy_pearls_->publish(msg);
}

void NoisyVicon::publish_transform() {

  auto tf_ = hat_transform_;

  // create the pose msg
  PoseStamped poseMsg;
  poseMsg.header.stamp = current_transform_time_;
  poseMsg.header.frame_id = "mapping"; // world_frame_id_;
  poseMsg.pose.position.x = tf_.translation()[0];
  poseMsg.pose.position.y = tf_.translation()[1];
  poseMsg.pose.position.z = tf_.translation()[2];

  Eigen::Quaterniond q(tf_.rotation());
  poseMsg.pose.orientation.x = q.x();
  poseMsg.pose.orientation.y = q.y();
  poseMsg.pose.orientation.z = q.z();
  poseMsg.pose.orientation.w = q.w();

  pub_pose_->publish(poseMsg);

  // create the pose with cov msg
  PoseWithCovarianceStamped poseCovMsg;
  poseCovMsg.header = poseMsg.header;
  poseCovMsg.pose.pose = poseMsg.pose;
  poseCovMsg.pose.covariance[0] = epsilon_R_;
  poseCovMsg.pose.covariance[1] = epsilon_t_;

  pub_pose_with_cov_->publish(poseCovMsg);

  // publish the tf object
  TransformStamped tfMsg;
  tfMsg.header = poseMsg.header;
  tfMsg.child_frame_id =
      "vicon/laptop_realsense/laptop_realsense"; // TODO(dev): make this a
                                                 // parameter
  tfMsg.transform.translation.x = poseMsg.pose.position.x;
  tfMsg.transform.translation.y = poseMsg.pose.position.y;
  tfMsg.transform.translation.z = poseMsg.pose.position.z;
  tfMsg.transform.rotation.x = poseMsg.pose.orientation.x;
  tfMsg.transform.rotation.y = poseMsg.pose.orientation.y;
  tfMsg.transform.rotation.z = poseMsg.pose.orientation.z;
  tfMsg.transform.rotation.w = poseMsg.pose.orientation.w;

  tf_broadcaster_->sendTransform(tfMsg);
}

} // namespace noisy_vicon

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<noisy_vicon::NoisyVicon>());
  rclcpp::shutdown();
  return 0;
}
