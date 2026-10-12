#include <gtest/gtest.h>

#include <cmath>

#include "pose_persistence/pose_save_policy.h"

namespace {

pose_type makePose(const float x, const float y, const float z, const float yaw) {
  pose_type pose;
  pose.pos = Eigen::Vector3f(x, y, z);
  pose.orient = Eigen::Vector3f(0.0f, 0.0f, yaw);
  pose.vel.setZero();
  update_q(pose);
  return pose;
}

}  // namespace

TEST(PoseSavePolicy, FirstStablePoseIsPersisted) {
  PoseSavePolicy::Config config;
  config.interval_sec = 1.0;
  config.max_interval_sec = 10.0;
  config.min_translation_m = 0.3;
  config.min_yaw_deg = 10.0;

  EXPECT_TRUE(PoseSavePolicy::ShouldPersist(
      config,
      makePose(0.0f, 0.0f, 0.0f, 0.0f),
      5.0,
      makePose(0.0f, 0.0f, 0.0f, 0.0f),
      0.0,
      false));
}

TEST(PoseSavePolicy, IntervalGateBlocksFrequentWrites) {
  PoseSavePolicy::Config config;
  config.interval_sec = 1.0;
  config.max_interval_sec = 10.0;
  config.min_translation_m = 0.3;
  config.min_yaw_deg = 10.0;

  EXPECT_FALSE(PoseSavePolicy::ShouldPersist(
      config,
      makePose(1.0f, 0.0f, 0.0f, 0.0f),
      5.5,
      makePose(0.0f, 0.0f, 0.0f, 0.0f),
      5.0,
      true));
}

TEST(PoseSavePolicy, RequiresMotionAfterInterval) {
  PoseSavePolicy::Config config;
  config.interval_sec = 1.0;
  config.max_interval_sec = 10.0;
  config.min_translation_m = 0.3;
  config.min_yaw_deg = 10.0;

  EXPECT_FALSE(PoseSavePolicy::ShouldPersist(
      config,
      makePose(0.1f, 0.0f, 0.0f, 0.05f),
      7.0,
      makePose(0.0f, 0.0f, 0.0f, 0.0f),
      5.0,
      true));
}

TEST(PoseSavePolicy, PersistsAfterTranslationThreshold) {
  PoseSavePolicy::Config config;
  config.interval_sec = 1.0;
  config.max_interval_sec = 10.0;
  config.min_translation_m = 0.3;
  config.min_yaw_deg = 10.0;

  EXPECT_TRUE(PoseSavePolicy::ShouldPersist(
      config,
      makePose(0.35f, 0.0f, 0.0f, 0.0f),
      7.0,
      makePose(0.0f, 0.0f, 0.0f, 0.0f),
      5.0,
      true));
}

TEST(PoseSavePolicy, PersistsAfterYawThreshold) {
  PoseSavePolicy::Config config;
  config.interval_sec = 1.0;
  config.max_interval_sec = 10.0;
  config.min_translation_m = 0.3;
  config.min_yaw_deg = 10.0;

  EXPECT_TRUE(PoseSavePolicy::ShouldPersist(
      config,
      makePose(0.0f, 0.0f, 0.0f, static_cast<float>(12.0 * M_PI / 180.0)),
      7.0,
      makePose(0.0f, 0.0f, 0.0f, 0.0f),
      5.0,
      true));
}

TEST(PoseSavePolicy, MaxIntervalForcesStaticPoseRefresh) {
  PoseSavePolicy::Config config;
  config.interval_sec = 1.0;
  config.max_interval_sec = 5.0;
  config.min_translation_m = 0.3;
  config.min_yaw_deg = 10.0;

  EXPECT_TRUE(PoseSavePolicy::ShouldPersist(
      config,
      makePose(0.0f, 0.0f, 0.0f, 0.0f),
      10.2,
      makePose(0.0f, 0.0f, 0.0f, 0.0f),
      5.0,
      true));
}
