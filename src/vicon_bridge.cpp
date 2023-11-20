#include "vicon_bridge/vicon_bridge.hpp"
#include "vicon_bridge/adapt.hpp"
#include <unistd.h>

namespace vicon_bridge {

using namespace std::chrono_literals;
using namespace ViconDataStreamSDK::CPP;

ViconBridge::ViconBridge()
    : Node("vicon_bridge"), start_time_(this->get_clock()->now()) {

  get_parameters();

  // initialize the tf broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // initialize the publishers

  // start vicon
  if (!init_vicon()) {
    RCLCPP_WARN(this->get_logger(), "INITIALIZATION FAILED.. EXITING");
    rclcpp::shutdown();
  }

  // setup diagnostics
  updater_ptr_ = std::make_shared<diagnostic_updater::Updater>(this);
  updater_ptr_->setHardwareID("vicon");
  // auto diagnostics_param =
  // diagnostic_updater::FrequencyStatusParam(&update_rate_hz_, &update_rate_hz_,
  // tolerance_, window_);
  auto diagnostics_param = diagnostic_updater::FrequencyStatusParam(
      &expected_rate_hz_, &expected_rate_hz_);
  pub_freq_ptr_ =
      std::make_shared<diagnostic_updater::HeaderlessTopicDiagnostic>(
          "vicon", *updater_ptr_, diagnostics_param);

  // start timer
  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_hz_),
      std::bind(&ViconBridge::timer_callback, this));
}

void ViconBridge::get_parameters() {

  host_name_ = this->declare_parameter<std::string>("host_name", host_name_);
  update_rate_hz_ =
      this->declare_parameter<double>("update_rate_hz", update_rate_hz_);
  expected_rate_hz_ =
      this->declare_parameter<double>("expected_rate_hz", expected_rate_hz_);
  publish_specific_segment_ = this->declare_parameter<bool>(
      "publish_specific_segment", publish_specific_segment_);
  target_subject_name_ = this->declare_parameter<std::string>(
      "target_subject_name", target_subject_name_);
  target_segment_name_ = this->declare_parameter<std::string>(
      "target_segment_name", target_segment_name_);
  world_frame_id_ =
      this->declare_parameter<std::string>("world_frame_id", world_frame_id_);
  tf_namespace_ =
      this->declare_parameter<std::string>("tf_namespace", tf_namespace_);
  stream_mode_ =
      this->declare_parameter<std::string>("stream_mode", stream_mode_);
}

bool ViconBridge::init_vicon() {

  Result::Enum result(Result::Unknown);

  // connect
  RCLCPP_INFO(get_logger(), "Connecting to Vicon Datastream SDK at %s",
              host_name_.c_str());
  while (!client_.IsConnected().Connected) {
    client_.Connect(host_name_);
    RCLCPP_INFO(get_logger(), ".");
    usleep(1000);
  }
  assert((client_.IsConnected().Connected));
  RCLCPP_INFO(get_logger(), "...connected!");

  // set stream mode
  // ClientPullPrefetch doesn't make much sense here, since we're only
  // forwarding the data
  if (stream_mode_ == "ServerPush") {
    result = client_.SetStreamMode(StreamMode::ServerPush).Result;
  } else if (stream_mode_ == "ClientPull") {
    result = client_.SetStreamMode(StreamMode::ClientPull).Result;
  } else {
    RCLCPP_FATAL(get_logger(),
                 "Unknown stream mode -- options are ServerPush, ClientPull");
    rclcpp::shutdown();
  }

  RCLCPP_INFO_STREAM(get_logger(), "Setting Stream Mode to " << stream_mode_
                                                             << ": "
                                                             << Adapt(result));

  // setup axis mode
  client_.SetAxisMapping(Direction::Forward, Direction::Left,
                         Direction::Up); // 'Z-up'
  Output_GetAxisMapping _Output_GetAxisMapping = client_.GetAxisMapping();

  RCLCPP_INFO_STREAM(get_logger(),
                     "Axis Mapping: X-"
                         << Adapt(_Output_GetAxisMapping.XAxis) << " Y-"
                         << Adapt(_Output_GetAxisMapping.YAxis) << " Z-"
                         << Adapt(_Output_GetAxisMapping.ZAxis));

  // reduce network bandwidth
  Output_EnableLightweightSegmentData _Output_lightweightSegmentData =
      client_.EnableLightweightSegmentData();
  if (_Output_lightweightSegmentData.Result == Result::Success) {
    RCLCPP_INFO_STREAM(get_logger(), "Setting lightweight mode: Success");
  } else {
    RCLCPP_WARN_STREAM(get_logger(), "Setting lightweight mode: Failure");
  }

  // setup segments
  client_.EnableSegmentData();
  assert(client_.IsSegmentDataEnabled().Enabled);

  // version
  Output_GetVersion _Output_GetVersion = client_.GetVersion();
  RCLCPP_INFO_STREAM(get_logger(),
                     "Version: " << _Output_GetVersion.Major << "."
                                 << _Output_GetVersion.Minor << "."
                                 << _Output_GetVersion.Point);
  return true;
}

void ViconBridge::create_segment_thread(const std::string subject_name,
                                        const std::string segment_name) {

  RCLCPP_INFO(get_logger(), "Creating new object %s/%s", subject_name.c_str(),
              segment_name.c_str());

  boost::mutex::scoped_lock lock(segments_mutex_);
  SegmentPublisher &spub =
      segment_publishers_[subject_name + "/" + segment_name];

  lock.unlock(); // we dont need the lock anymore

  // register the publisher
  spub.pub = this->create_publisher<geometry_msgs::msg::TransformStamped>(
      tf_namespace_ + "/" + subject_name + "/" + segment_name, 10);

  // register the pose publisher
  spub.pub_poseMsg = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      tf_namespace_ + "/" + subject_name + "/" + segment_name + "/pose", 10);

  spub.is_ready = true;
}

void ViconBridge::create_segment(const std::string subject,
                                 const std::string segment) {

  boost::thread(&ViconBridge::create_segment_thread, this, subject, segment);
}

void ViconBridge::timer_callback() {

  rclcpp::Rate rate(update_rate_hz_);

  while (client_.GetFrame().Result != Result::Success && rclcpp::ok()) {

    RCLCPP_INFO(get_logger(), "GetFrame returned false");
    rate.sleep();
  }

  auto now = this->get_clock()->now();

  process_frame(now);
}

void ViconBridge::process_frame(rclcpp::Time &grab_time) {

  if (first_frame_) {
    first_frame_number_ = client_.GetFrameNumber().FrameNumber;
  }

  std::size_t frame_number = client_.GetFrameNumber().FrameNumber;

  // RCLCPP_INFO(get_logger(), "FIRST: %zu, LAST: %zu, CURR: %zu, DROP: %zu,
  // TOTAL: %zu", first_frame_number_, last_frame_number_, frame_number,
  // drop_count_, frame_count_);

  // check that indeed we have a new frame
  int frame_diff = frame_number - last_frame_number_;
  if (frame_diff <= 0) {
    RCLCPP_WARN(get_logger(), "FRAME_DIFF <= 0");
    return;
  }

  // check if we have dropped frames
  if (!first_frame_) {
    frame_count_ += frame_diff;
    if (frame_diff > 1) {
      drop_count_ += frame_diff - 1;
      double droppedFramePct = (double)drop_count_ / frame_count_ * 100.0;
      RCLCPP_WARN_STREAM(get_logger(), frame_diff - 1
                                           << " more frame(s) dropped. Total "
                                           << drop_count_ << "/" << frame_count_
                                           << ", " << droppedFramePct
                                           << "%. Consider adjusting rates.");
    }
  }

  // now do the actual processing
  double latency_s = client_.GetLatencyTotal().Total;
  rclcpp::Duration latency{std::chrono::duration<double>(latency_s)};
  RCLCPP_INFO_THROTTLE(get_logger(), *(this->get_clock()),
                       1000, // 1000 ms = 1 second
                       "latency of last frame: %f s  [frame_count: %zu]",
                       latency_s, frame_count_);

  if (publish_specific_segment_) {
    process_specific_segment(grab_time - latency);
  } else {
    process_all_segments(grab_time - latency);
  }

  pub_freq_ptr_->tick();

  last_frame_number_ = frame_number;
  return;
}

void ViconBridge::process_specific_segment(const rclcpp::Time &frame_time) {

  geometry_msgs::msg::TransformStamped msg;
  bool success = get_transform_msg(msg, frame_time, target_subject_name_,
                                   target_segment_name_);

  if (success) {
    tf_broadcaster_->sendTransform(msg);
    first_frame_ = false; // got a frame!
  }

  return;
}

void ViconBridge::process_all_segments(const rclcpp::Time &frame_time) {

  std::string subject_name, segment_name;

  std::size_t n_subjects = client_.GetSubjectCount().SubjectCount;

  SegmentMap::iterator pub_it;

  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  for (std::size_t i_subject = 0; i_subject < n_subjects; ++i_subject) {

    subject_name = client_.GetSubjectName(i_subject).SubjectName;

    std::size_t n_segments = client_.GetSegmentCount(subject_name).SegmentCount;

    for (std::size_t i_segment = 0; i_segment < n_segments; ++i_segment) {

      segment_name =
          client_.GetSegmentName(subject_name, i_segment).SegmentName;

      geometry_msgs::msg::TransformStamped transform;
      bool success =
          get_transform_msg(transform, frame_time, subject_name, segment_name);

      if (success) {
        transforms.push_back(transform);

        // now find the specific publisher
        boost::mutex::scoped_try_lock lock(segments_mutex_);
        if (lock.owns_lock()) {
          pub_it = segment_publishers_.find(subject_name + "/" + segment_name);
          if (pub_it != segment_publishers_.end()) {
            SegmentPublisher &spub = pub_it->second;
            if (spub.is_ready) {
              // publish the tranform msg
              spub.pub->publish(transform);
              // also publish as a pose msg
              spub.pub_poseMsg->publish(transform2pose(transform));
            }
          } else {
            // need to create a new publisher
            lock.unlock();
            create_segment(subject_name, segment_name);
          }
        }
      }
    }
  }

  if (transforms.size() > 0) {
    // now publish all of the transforms as a single tf broadcaster message
    tf_broadcaster_->sendTransform(transforms);
    first_frame_ = false;
  }
}

geometry_msgs::msg::PoseStamped ViconBridge::transform2pose(
    geometry_msgs::msg::TransformStamped &transformMsg) {

  geometry_msgs::msg::PoseStamped poseMsg;
  poseMsg.header = transformMsg.header;
  poseMsg.pose.position.x = transformMsg.transform.translation.x;
  poseMsg.pose.position.y = transformMsg.transform.translation.y;
  poseMsg.pose.position.z = transformMsg.transform.translation.z;
  poseMsg.pose.orientation = transformMsg.transform.rotation;

  return poseMsg;
}

bool ViconBridge::get_transform_msg(geometry_msgs::msg::TransformStamped &msg,
                                    const rclcpp::Time &frame_time,
                                    std::string subject_name,
                                    std::string segment_name) {

  auto res_quat =
      client_.GetSegmentGlobalRotationQuaternion(subject_name, segment_name);
  auto res_trans =
      client_.GetSegmentGlobalTranslation(subject_name, segment_name);

  if (res_quat.Result != Result::Success ||
      res_trans.Result != Result::Success) {
    RCLCPP_WARN(get_logger(), "getTransform failed for %s/%s",
                subject_name.c_str(), segment_name.c_str());
    return false;
  }

  if (res_quat.Occluded || res_trans.Occluded) {
    RCLCPP_WARN(get_logger(), "Subject %s/%s is Occluded", subject_name.c_str(),
                segment_name.c_str());
    return false;
  }

  msg.header.frame_id = (tf_namespace_ + "/" + world_frame_id_).c_str();
  msg.header.stamp = frame_time;
  msg.child_frame_id =
      (tf_namespace_ + "/" + subject_name + "/" + segment_name).c_str();

  auto trans = res_trans.Translation;
  msg.transform.translation.x = trans[0] / 1000;
  msg.transform.translation.y = trans[1] / 1000;
  msg.transform.translation.z = trans[2] / 1000;

  auto quat = res_quat.Rotation;
  msg.transform.rotation.x = quat[0];
  msg.transform.rotation.y = quat[1];
  msg.transform.rotation.z = quat[2];
  msg.transform.rotation.w = quat[3];

  return true;
}

} // namespace vicon_bridge

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vicon_bridge::ViconBridge>());
  rclcpp::shutdown();
  return 0;
}
