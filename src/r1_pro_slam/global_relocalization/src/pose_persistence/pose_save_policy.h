#ifndef INDOOR_LOCATION_POSE_SAVE_POLICY_H_
#define INDOOR_LOCATION_POSE_SAVE_POLICY_H_

#include "utility.h"

class PoseSavePolicy {
 public:
  struct Config {
    double interval_sec = 1.0;
    double max_interval_sec = 10.0;
    double min_translation_m = 0.3;
    double min_yaw_deg = 10.0;
  };

  static bool ShouldPersist(
      const Config& config,
      const pose_type& current_pose,
      double current_time,
      const pose_type& last_saved_pose,
      double last_save_time,
      bool has_last_saved_pose);
};

#endif  // INDOOR_LOCATION_POSE_SAVE_POLICY_H_
