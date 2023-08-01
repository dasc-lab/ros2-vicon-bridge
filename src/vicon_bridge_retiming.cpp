#include "vicon_bridge/vicon_bridge_retiming.hpp"
#include "vicon_bridge/adapt.hpp"
#include <unistd.h>

namespace vicon_bridge {

using namespace std::chrono_literals;
using namespace ViconDataStreamSDK::CPP;

ViconBridgeRetiming::ViconBridgeRetiming()
    : Node("vicon_bridge_retiming"), start_time_(this->get_clock()->now()) {

  get_parameters();

  // initialize the tf broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // start vicon
  if (!init_vicon()) {
    RCLCPP_WARN(this->get_logger(), "INITIALIZATION FAILED.. EXITING");
    rclcpp::shutdown();
  }

  // start timer
  // start a different timer based on whether we want to publish a specific
  // segment or all segments
  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_hz_),
      std::bind((publish_specific_segment_
                     ? &ViconBridgeRetiming::timer_callback_specific
                     : &ViconBridgeRetiming::timer_callback_all),
                this));
}

void ViconBridgeRetiming::get_parameters() {

  host_name_ = this->declare_parameter<std::string>("host_name", host_name_);
  max_prediction_ms_ =
      this->declare_parameter<double>("max_prediction_ms", max_prediction_ms_);
  update_rate_hz_ =
      this->declare_parameter<double>("update_rate_hz", update_rate_hz_);
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
}

bool ViconBridgeRetiming::init_vicon() {

  // connect
  RCLCPP_INFO(get_logger(), "Connecting to Vicon Datastream SDK at %s",
              host_name_.c_str());
  while (!client_.IsConnected().Connected) {
    client_.Connect(host_name_);
    RCLCPP_INFO(get_logger(), ".");
    usleep(1000);
  }
  assert((client_.IsConnected().Connected));
  RCLCPP_INFO(get_logger(), "Connected!");

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

  // set Maximum prediction allowed
  client_.SetMaximumPrediction(max_prediction_ms_);

  Output_GetVersion _Output_GetVersion = client_.GetVersion();
  RCLCPP_INFO_STREAM(get_logger(),
                     "Version: " << _Output_GetVersion.Major << "."
                                 << _Output_GetVersion.Minor << "."
                                 << _Output_GetVersion.Point);
  return true;
}

void ViconBridgeRetiming::timer_callback_specific() {

  // print drop_rate
  print_drop_rate();

  // increment the drop counter, and then reduce it when we publish a pose
  drop_count_++;

  double offset = 0.0;
  Output_UpdateFrame res_updateFrame = client_.UpdateFrame(offset);
  const rclcpp::Time now = this->get_clock()->now();

  if (res_updateFrame.Result != Result::Success) {
    RCLCPP_WARN_STREAM(get_logger(), "UpdateFrame failed! DropCount: "
                                         << drop_count_ << " Status: "
                                         << Adapt(res_updateFrame.Result));
    return;
  }

  String subjectName = target_subject_name_;
  String segmentName = target_segment_name_;

  // GET THE SEGMENT ROTATION
  Output_GetSegmentGlobalRotationQuaternion res_quat =
      client_.GetSegmentGlobalRotationQuaternion(subjectName, segmentName);

  if (res_quat.Result != Result::Success) {
    RCLCPP_WARN_STREAM(get_logger(),
                       "Could not get the rotation for "
                           << subjectName << "/" << segmentName
                           << ". Status: " << Adapt(res_quat.Result) << " ["
                           << res_quat.Result << "]");
    return;
  }

  // GET THE SEGMENT TRANSLATION
  Output_GetSegmentGlobalTranslation res_trans =
      client_.GetSegmentGlobalTranslation(subjectName, segmentName);
  if (res_trans.Result != Result::Success) {
    RCLCPP_INFO_STREAM(get_logger(),
                       "Could not get translation for "
                           << subjectName << "/" << segmentName
                           << ". Status : " << Adapt(res_trans.Result));
    return;
  }

  // check for occlusion
  if (res_quat.Occluded || res_trans.Occluded) {
    RCLCPP_WARN_STREAM(get_logger(),
                       "OCCULDED: " << subjectName << "/" << segmentName);
    return;
  }

  // Now publish it
  drop_count_--;
  publish_pose(now, subjectName, segmentName, res_quat.Rotation,
               res_trans.Translation);
}

void ViconBridgeRetiming::print_drop_rate() {

  double t = (get_clock()->now() - start_time_).seconds();
  double drop_rate = drop_count_ / t;
  RCLCPP_INFO_THROTTLE(
      get_logger(), *(this->get_clock()), 1000,
      "Missed %zu callbacks in %f seconds. Drop Rate: %f cbs/s", drop_count_, t,
      drop_rate);
}

void ViconBridgeRetiming::timer_callback_all() {

  // print drop_rate
  print_drop_rate();

  // increment the drop counter, and then reduce it when we publish a pose
  drop_count_++;

  double offset = 0.0;
  Output_UpdateFrame res_updateFrame = client_.UpdateFrame(offset);

  if (res_updateFrame.Result != Result::Success) {
    RCLCPP_WARN_STREAM(get_logger(), "UpdateFrame failed! DropCount: "
                                         << drop_count_ << " Status: "
                                         << Adapt(res_updateFrame.Result));
    return;
  }

  const rclcpp::Time now = this->get_clock()->now();

  // GET SUBJECT COUNT
  Output_GetSubjectCount res_subjectCount = client_.GetSubjectCount();
  if (res_subjectCount.Result != Result::Success) {
    RCLCPP_WARN(get_logger(), "Could not get subject count");
    return;
  }
  std::size_t N_subject = res_subjectCount.SubjectCount;
  // RCLCPP_INFO(get_logger(), "Got Frame: Subject Count: %zu", N_subject);

  // LOOP OVER SUBJECTS
  for (std::size_t i_subject = 0; i_subject < N_subject; ++i_subject) {

    // GET SUBJECT NAME
    Output_GetSubjectName res_subjectName = client_.GetSubjectName(i_subject);
    if (res_subjectName.Result != Result::Success) {
      RCLCPP_WARN(get_logger(), "Could not get subject %li name", i_subject);
      continue;
    }
    String subjectName = res_subjectName.SubjectName;

    // GET SEGMENT COUNT
    Output_GetSegmentCount res_segmentCount =
        client_.GetSegmentCount(res_subjectName.SubjectName);
    if (res_segmentCount.Result != Result::Success) {
      RCLCPP_WARN_STREAM(get_logger(),
                         "Count not get segment count for subject "
                             << subjectName);
      continue;
    }

    // LOOP OVER SEGMENTS
    std::size_t N_segment = res_segmentCount.SegmentCount;
    for (std::size_t i_segment = 0; i_segment < N_segment; ++i_segment) {

      // GET SEGMENT NAME
      Output_GetSegmentName res_segmentName =
          client_.GetSegmentName(subjectName, i_segment);
      if (res_segmentName.Result != Result::Success) {
        RCLCPP_WARN_STREAM(get_logger(),
                           "Could not get segment name for segment "
                               << i_segment << "of subject " << subjectName);
        continue;
      }
      String segmentName = res_segmentName.SegmentName;

      // GET THE SEGMENT ROTATION
      Output_GetSegmentGlobalRotationQuaternion res_quat =
          client_.GetSegmentGlobalRotationQuaternion(subjectName, segmentName);

      if (res_quat.Result != Result::Success) {
        RCLCPP_WARN_STREAM(get_logger(),
                           "Could not get the rotation for "
                               << subjectName << "/" << segmentName
                               << ". Status: " << Adapt(res_quat.Result) << " ["
                               << res_quat.Result << "]");
        continue;
      }

      // GET THE SEGMENT TRANSLATION
      Output_GetSegmentGlobalTranslation res_trans =
          client_.GetSegmentGlobalTranslation(subjectName, segmentName);
      if (res_trans.Result != Result::Success) {
        RCLCPP_INFO_STREAM(get_logger(),
                           "Could not get translation for "
                               << subjectName << "/" << segmentName
                               << ". Status : " << Adapt(res_trans.Result));
        continue;
      }

      // check for occlusion
      if (res_quat.Occluded || res_trans.Occluded) {
        RCLCPP_WARN_STREAM(get_logger(),
                           "OCCULDED: " << subjectName << "/" << segmentName);
        continue;
      }

      // Now publish it
      drop_count_--;
      publish_pose(now, subjectName, segmentName, res_quat.Rotation,
                   res_trans.Translation);
    }
  }
}

void ViconBridgeRetiming::publish_pose(const rclcpp::Time &now,
                                       const String &subjectName,
                                       const String &segmentName, double *quat,
                                       double *trans) {

  geometry_msgs::msg::TransformStamped msg;

  msg.header.frame_id = (tf_namespace_ + "/" + world_frame_id_).c_str();
  msg.header.stamp = now;
  msg.child_frame_id = (tf_namespace_ + "/" + std::string(subjectName) + "/" +
                        std::string(segmentName))
                           .c_str();

  msg.transform.translation.x = trans[0] / 1000;
  msg.transform.translation.y = trans[1] / 1000;
  msg.transform.translation.z = trans[2] / 1000;

  msg.transform.rotation.x = quat[0];
  msg.transform.rotation.y = quat[1];
  msg.transform.rotation.z = quat[2];
  msg.transform.rotation.w = quat[3];

  //  send
  tf_broadcaster_->sendTransform(msg);
}

} // namespace vicon_bridge

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vicon_bridge::ViconBridgeRetiming>());
  rclcpp::shutdown();
  return 0;
}
