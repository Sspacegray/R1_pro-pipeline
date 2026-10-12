#include "localization_state/localization_state.h"

#include <tf2/LinearMath/Quaternion.h>

namespace indoor_location {

const char* ToString(const LocalizationState state) {
  switch (state) {
    case LocalizationState::kUninitialized:
      return "UNINITIALIZED";
    case LocalizationState::kRestoredPosePending:
      return "RESTORED_POSE_PENDING";
    case LocalizationState::kGlobalRelocalizing:
      return "GLOBAL_RELOCALIZING";
    case LocalizationState::kTrackingOk:
      return "TRACKING_OK";
    case LocalizationState::kTrackingWeak:
      return "TRACKING_WEAK";
    case LocalizationState::kLost:
      return "LOST";
    default:
      return "UNINITIALIZED";
  }
}

LocalizationState LocalizationStateFromString(const std::string& state) {
  if (state == "RESTORED_POSE_PENDING") {
    return LocalizationState::kRestoredPosePending;
  }
  if (state == "GLOBAL_RELOCALIZING") {
    return LocalizationState::kGlobalRelocalizing;
  }
  if (state == "TRACKING_OK") {
    return LocalizationState::kTrackingOk;
  }
  if (state == "TRACKING_WEAK") {
    return LocalizationState::kTrackingWeak;
  }
  if (state == "LOST") {
    return LocalizationState::kLost;
  }
  return LocalizationState::kUninitialized;
}

geometry_msgs::msg::Pose PoseToRosPose(const pose_type& pose) {
  geometry_msgs::msg::Pose ros_pose;
  ros_pose.position.x = pose.pos.x();
  ros_pose.position.y = pose.pos.y();
  ros_pose.position.z = pose.pos.z();

  tf2::Quaternion q;
  q.setRPY(pose.orient.x(), pose.orient.y(), pose.orient.z());
  ros_pose.orientation.x = q.x();
  ros_pose.orientation.y = q.y();
  ros_pose.orientation.z = q.z();
  ros_pose.orientation.w = q.w();
  return ros_pose;
}

indoor_location::msg::LocalizationStatus BuildLocalizationStatusMessage(
    const LocalizationStatusSnapshot& snapshot,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frame_id) {
  indoor_location::msg::LocalizationStatus message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  message.state = ToString(snapshot.state);
  message.in_relocalization = snapshot.in_relocalization;
  message.runtime_reloc_locked = snapshot.runtime_reloc_locked;
  message.map_to_odom_frozen = snapshot.map_to_odom_frozen;
  message.pose_restored_from_disk = snapshot.pose_restored_from_disk;
  message.wheel_odom_available = snapshot.wheel_odom_available;
  message.fitness = snapshot.fitness;
  message.raw_fitness = snapshot.raw_fitness;
  message.reloc_success_count = snapshot.reloc_success_count;
  message.tracking_loss_count = snapshot.tracking_loss_count;
  message.pose = PoseToRosPose(snapshot.pose);
  return message;
}

}  // namespace indoor_location
