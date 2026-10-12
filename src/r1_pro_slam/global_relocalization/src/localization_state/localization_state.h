#ifndef INDOOR_LOCATION_LOCALIZATION_STATE_H_
#define INDOOR_LOCATION_LOCALIZATION_STATE_H_

#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "indoor_location/msg/localization_status.hpp"
#include "utility.h"

namespace indoor_location {

enum class LocalizationState {
  kUninitialized,
  kRestoredPosePending,
  kGlobalRelocalizing,
  kTrackingOk,
  kTrackingWeak,
  kLost,
};

struct LocalizationStatusSnapshot {
  LocalizationState state = LocalizationState::kUninitialized;
  bool in_relocalization = false;
  bool runtime_reloc_locked = false;
  bool map_to_odom_frozen = false;
  bool pose_restored_from_disk = false;
  bool wheel_odom_available = false;
  float fitness = 0.0f;
  float raw_fitness = 0.0f;
  int32_t reloc_success_count = 0;
  int32_t tracking_loss_count = 0;
  pose_type pose;
};

const char* ToString(LocalizationState state);
LocalizationState LocalizationStateFromString(const std::string& state);
geometry_msgs::msg::Pose PoseToRosPose(const pose_type& pose);
indoor_location::msg::LocalizationStatus BuildLocalizationStatusMessage(
    const LocalizationStatusSnapshot& snapshot,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frame_id);

}  // namespace indoor_location

#endif  // INDOOR_LOCATION_LOCALIZATION_STATE_H_
