#ifndef INDOOR_LOCATION_BBS_LOCALIZATION_H_
#define INDOOR_LOCATION_BBS_LOCALIZATION_H_

#include <memory>
#include <queue>
#include <vector>

#include <boost/optional.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "global_relocalization/occupancy_gridmap.h"

struct BBSSearchWindow {
  double min_tx = -50.0;
  double max_tx = 50.0;
  double min_ty = -50.0;
  double max_ty = 50.0;
  double min_theta = -M_PI;
  double max_theta = M_PI;
};

struct BBSParams {
  double max_range = 15.0;
};

struct DiscreteTransformation {
  using Points2D = OccupancyGridMap::Points2D;

  DiscreteTransformation();
  DiscreteTransformation(int level, int x, int y, int theta);

  bool operator<(const DiscreteTransformation& rhs) const;
  bool isLeaf() const;
  Eigen::Isometry2f transformation(
      double theta_resolution,
      const std::vector<std::shared_ptr<OccupancyGridMap>>& gridmap_pyramid) const;
  double calcScore(
      const Points2D& points,
      double theta_resolution,
      const std::vector<std::shared_ptr<OccupancyGridMap>>& gridmap_pyramid);
  std::vector<DiscreteTransformation> branch() const;

  double score = 0.0;
  int level = -1;
  int x = 0;
  int y = 0;
  int theta = 0;
};

class BBSLocalization {
 public:
  using Points2D = OccupancyGridMap::Points2D;

  explicit BBSLocalization(BBSParams params = {});

  void setMap(
      const Points2D& map_points,
      double resolution,
      int width,
      int height,
      int pyramid_levels,
      int max_points_per_cell);

  boost::optional<Eigen::Isometry2f> localize(
      const Points2D& scan_points,
      const BBSSearchWindow& search_window,
      double min_score,
      double* best_score = nullptr) const;

 private:
  std::priority_queue<DiscreteTransformation> createInitialSet(
      const Points2D& scan_points,
      const BBSSearchWindow& search_window) const;

  BBSParams params_;
  mutable double theta_resolution_ = 0.0;
  std::vector<std::shared_ptr<OccupancyGridMap>> gridmap_pyramid_;
};

#endif  // INDOOR_LOCATION_BBS_LOCALIZATION_H_
