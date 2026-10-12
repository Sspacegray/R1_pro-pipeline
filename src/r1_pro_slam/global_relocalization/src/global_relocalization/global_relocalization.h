#ifndef INDOOR_LOCATION_GLOBAL_RELOCALIZATION_H_
#define INDOOR_LOCATION_GLOBAL_RELOCALIZATION_H_

#include <memory>

#include "global_relocalization/bbs_localization.h"
#include "utility.h"

class GlobalRelocalization {
 public:
  struct Config {
    double max_range = 15.0;
    double min_tx = -50.0;
    double max_tx = 50.0;
    double min_ty = -50.0;
    double max_ty = 50.0;
    double min_theta = -M_PI;
    double max_theta = M_PI;
    double map_min_z = -0.2;
    double map_max_z = 1.2;
    double scan_min_z = -0.2;
    double scan_max_z = 1.2;
    double map_resolution = 0.5;
    int map_width = 512;
    int map_height = 512;
    int map_pyramid_level = 6;
    int max_points_per_cell = 5;
    bool enable_bounded_window = true;
    double bounded_translation_window = 8.0;
    double bounded_yaw_window_deg = 45.0;
  };

  struct Result {
    pose_type pose;
    double score = 0.0;
    bool used_bounded_window = false;
  };

  using Points2D = OccupancyGridMap::Points2D;

  GlobalRelocalization();
  explicit GlobalRelocalization(const Config& config);

  void setGlobalMap(const pcl::PointCloud<PointType>::ConstPtr& cloud);
  bool hasMap() const;

  bool localize(const pcl::PointCloud<PointType>::ConstPtr& cloud, Result& result) const;
  bool localizeNearPose(
      const pcl::PointCloud<PointType>::ConstPtr& cloud,
      const pose_type& prior_pose,
      Result& result) const;

 private:
  Points2D slice(const pcl::PointCloud<PointType>& cloud, double min_z, double max_z) const;
  BBSSearchWindow fullSearchWindow() const;
  BBSSearchWindow boundedSearchWindow(const pose_type& prior_pose) const;
  bool runLocalization(
      const pcl::PointCloud<PointType>::ConstPtr& cloud,
      const BBSSearchWindow& search_window,
      bool bounded_window,
      Result& result) const;

  Config config_;
  Points2D map_points_2d_;
  pcl::PointCloud<PointType>::ConstPtr map_cloud_;
};

#endif  // INDOOR_LOCATION_GLOBAL_RELOCALIZATION_H_
