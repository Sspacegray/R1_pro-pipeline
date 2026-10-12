#include "global_relocalization/global_relocalization.h"

#include <algorithm>
#include <cmath>

GlobalRelocalization::GlobalRelocalization() = default;

GlobalRelocalization::GlobalRelocalization(const Config& config) : config_(config) {}

void GlobalRelocalization::setGlobalMap(const pcl::PointCloud<PointType>::ConstPtr& cloud) {
  map_cloud_ = cloud;
  map_points_2d_ = cloud ? slice(*cloud, config_.map_min_z, config_.map_max_z) : Points2D();
}

bool GlobalRelocalization::hasMap() const {
  return map_cloud_ != nullptr && !map_points_2d_.empty();
}

bool GlobalRelocalization::localize(
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    Result& result) const {
  return runLocalization(cloud, fullSearchWindow(), false, result);
}

bool GlobalRelocalization::localizeNearPose(
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    const pose_type& prior_pose,
    Result& result) const {
  return runLocalization(cloud, boundedSearchWindow(prior_pose), true, result);
}

GlobalRelocalization::Points2D GlobalRelocalization::slice(
    const pcl::PointCloud<PointType>& cloud,
    const double min_z,
    const double max_z) const {
  Points2D points;
  points.reserve(cloud.size());
  for (const auto& point : cloud) {
    if (point.z >= min_z && point.z <= max_z) {
      points.emplace_back(point.x, point.y);
    }
  }
  return points;
}

BBSSearchWindow GlobalRelocalization::fullSearchWindow() const {
  BBSSearchWindow window;
  window.min_tx = config_.min_tx;
  window.max_tx = config_.max_tx;
  window.min_ty = config_.min_ty;
  window.max_ty = config_.max_ty;
  window.min_theta = config_.min_theta;
  window.max_theta = config_.max_theta;
  return window;
}

BBSSearchWindow GlobalRelocalization::boundedSearchWindow(const pose_type& prior_pose) const {
  BBSSearchWindow window = fullSearchWindow();
  const double half_translation = std::max(0.1, config_.bounded_translation_window);
  const double half_yaw = std::max(1.0, config_.bounded_yaw_window_deg) * Deg2Rad;
  window.min_tx = std::max(config_.min_tx, static_cast<double>(prior_pose.pos.x()) - half_translation);
  window.max_tx = std::min(config_.max_tx, static_cast<double>(prior_pose.pos.x()) + half_translation);
  window.min_ty = std::max(config_.min_ty, static_cast<double>(prior_pose.pos.y()) - half_translation);
  window.max_ty = std::min(config_.max_ty, static_cast<double>(prior_pose.pos.y()) + half_translation);
  window.min_theta = std::max(config_.min_theta, static_cast<double>(prior_pose.orient.z()) - half_yaw);
  window.max_theta = std::min(config_.max_theta, static_cast<double>(prior_pose.orient.z()) + half_yaw);
  return window;
}

bool GlobalRelocalization::runLocalization(
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    const BBSSearchWindow& search_window,
    const bool bounded_window,
    Result& result) const {
  if (!hasMap() || !cloud) {
    return false;
  }

  const Points2D scan_points = slice(*cloud, config_.scan_min_z, config_.scan_max_z);
  if (scan_points.size() < 16) {
    return false;
  }

  BBSParams params;
  params.max_range = config_.max_range;
  BBSLocalization bbs(params);
  bbs.setMap(
      map_points_2d_,
      config_.map_resolution,
      config_.map_width,
      config_.map_height,
      config_.map_pyramid_level,
      config_.max_points_per_cell);

  double best_score = 0.0;
  const auto transform = bbs.localize(scan_points, search_window, 0.0, &best_score);
  if (transform == boost::none) {
    return false;
  }

  result.score = best_score;
  result.used_bounded_window = bounded_window;
  result.pose = pose_type();
  result.pose.timestamp = 0.0;
  result.pose.status = bounded_window ? "bounded_bbs" : "global_bbs";
  result.pose.pos = Eigen::Vector3f(transform->translation().x(), transform->translation().y(), 0.0f);
  result.pose.orient = Eigen::Vector3f(
      0.0f,
      0.0f,
      std::atan2(transform->linear()(1, 0), transform->linear()(0, 0)));
  result.pose.vel.setZero();
  update_q(result.pose);
  return true;
}
