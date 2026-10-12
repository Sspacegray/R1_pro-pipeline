#include <gtest/gtest.h>

#include <cmath>
#include <Eigen/Geometry>

#include "global_relocalization/global_relocalization.h"

namespace {

std::vector<Eigen::Vector2f> buildLocalPattern() {
  return {
      {0.0f, 0.0f}, {1.0f, 0.0f}, {2.0f, 0.0f}, {3.0f, 0.0f},
      {0.0f, 1.0f}, {0.0f, 2.0f}, {0.0f, 3.0f}, {1.0f, 3.0f},
      {2.0f, 3.0f}, {3.0f, 3.0f}, {3.0f, 2.0f}, {3.0f, 1.0f},
      {1.5f, 1.0f}, {1.5f, 2.0f}, {2.2f, 1.5f}, {2.6f, 2.4f},
      {0.8f, 2.2f}, {2.8f, 0.4f}, {0.3f, 2.8f}, {2.2f, 2.8f},
  };
}

pcl::PointCloud<PointType>::Ptr buildMap(const Eigen::Vector3f& pose_xy_yaw) {
  auto cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
  const float yaw = pose_xy_yaw.z();
  const Eigen::Matrix2f rotation = Eigen::Rotation2Df(yaw).toRotationMatrix();
  for (const auto& local : buildLocalPattern()) {
    const Eigen::Vector2f world = rotation * local + pose_xy_yaw.head<2>();
    PointType point;
    point.x = world.x();
    point.y = world.y();
    point.z = 0.0f;
    point.intensity = 1.0f;
    cloud->push_back(point);
  }

  for (int idx = 0; idx < 10; ++idx) {
    PointType point;
    point.x = -20.0f + static_cast<float>(idx);
    point.y = 15.0f - static_cast<float>(idx % 3);
    point.z = 0.0f;
    point.intensity = 0.5f;
    cloud->push_back(point);
  }
  return cloud;
}

pcl::PointCloud<PointType>::Ptr buildLocalScan() {
  auto scan = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
  for (const auto& local : buildLocalPattern()) {
    PointType point;
    point.x = local.x();
    point.y = local.y();
    point.z = 0.0f;
    point.intensity = 1.0f;
    scan->push_back(point);
  }
  return scan;
}

}  // namespace

TEST(GlobalRelocalization, FindsKnownPlanarPose) {
  GlobalRelocalization::Config config;
  config.max_range = 20.0;
  config.min_tx = -15.0;
  config.max_tx = 15.0;
  config.min_ty = -15.0;
  config.max_ty = 15.0;
  config.min_theta = -M_PI;
  config.max_theta = M_PI;
  config.map_resolution = 1.0;
  config.map_width = 64;
  config.map_height = 64;
  config.map_pyramid_level = 4;
  config.max_points_per_cell = 3;
  config.map_min_z = -0.5;
  config.map_max_z = 0.5;
  config.scan_min_z = -0.5;
  config.scan_max_z = 0.5;

  GlobalRelocalization relocalization(config);
  const Eigen::Vector3f expected_pose(4.0f, -3.0f, 0.45f);
  relocalization.setGlobalMap(buildMap(expected_pose));
  const auto scan = buildLocalScan();

  GlobalRelocalization::Result result;
  ASSERT_TRUE(relocalization.localize(scan, result));
  EXPECT_NEAR(result.pose.pos.x(), expected_pose.x(), 1.0f);
  EXPECT_NEAR(result.pose.pos.y(), expected_pose.y(), 1.0f);
  EXPECT_NEAR(result.pose.orient.z(), expected_pose.z(), 0.2f);
}
