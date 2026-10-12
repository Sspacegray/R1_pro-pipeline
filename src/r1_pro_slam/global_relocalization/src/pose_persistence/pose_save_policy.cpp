#include "pose_persistence/pose_save_policy.h"

#include <algorithm>
#include <cmath>

namespace {

double TranslationDelta(const pose_type& lhs, const pose_type& rhs) {
  return (lhs.pos.cast<double>() - rhs.pos.cast<double>()).norm();
}

double YawDeltaDeg(const pose_type& lhs, const pose_type& rhs) {
  const double delta_rad = std::remainder(
      static_cast<double>(lhs.orient.z() - rhs.orient.z()),
      2.0 * M_PI);
  return std::abs(delta_rad) * 180.0 / M_PI;
}

}  // namespace

bool PoseSavePolicy::ShouldPersist(
    const Config& config,
    const pose_type& current_pose,
    const double current_time,
    const pose_type& last_saved_pose,
    const double last_save_time,
    const bool has_last_saved_pose) {
  if (!has_last_saved_pose) {
    return true;
  }

  const double elapsed_sec = current_time - last_save_time;
  if (!std::isfinite(elapsed_sec) || elapsed_sec < 0.0) {
    return true;
  }

  if (config.max_interval_sec > 0.0 && elapsed_sec >= config.max_interval_sec) {
    return true;
  }

  if (config.interval_sec > 0.0 && elapsed_sec < config.interval_sec) {
    return false;
  }

  const double translation_delta = TranslationDelta(current_pose, last_saved_pose);
  if (translation_delta >= std::max(0.0, config.min_translation_m)) {
    return true;
  }

  const double yaw_delta_deg = YawDeltaDeg(current_pose, last_saved_pose);
  if (yaw_delta_deg >= std::max(0.0, config.min_yaw_deg)) {
    return true;
  }

  return config.interval_sec <= 0.0 && config.max_interval_sec <= 0.0;
}
