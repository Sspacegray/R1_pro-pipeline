#include "global_relocalization/bbs_localization.h"

#include <algorithm>
#include <cmath>

DiscreteTransformation::DiscreteTransformation() = default;

DiscreteTransformation::DiscreteTransformation(const int level_in, const int x_in, const int y_in, const int theta_in)
    : level(level_in), x(x_in), y(y_in), theta(theta_in) {}

bool DiscreteTransformation::operator<(const DiscreteTransformation& rhs) const {
  return score < rhs.score;
}

bool DiscreteTransformation::isLeaf() const {
  return level == 0;
}

Eigen::Isometry2f DiscreteTransformation::transformation(
    const double theta_resolution,
    const std::vector<std::shared_ptr<OccupancyGridMap>>& gridmap_pyramid) const {
  const double trans_resolution = gridmap_pyramid[level]->gridResolution();
  Eigen::Isometry2f transform = Eigen::Isometry2f::Identity();
  transform.linear() = Eigen::Rotation2Df(static_cast<float>(theta_resolution * theta)).toRotationMatrix();
  transform.translation() = Eigen::Vector2f(
      static_cast<float>(trans_resolution * x),
      static_cast<float>(trans_resolution * y));
  return transform;
}

double DiscreteTransformation::calcScore(
    const Points2D& points,
    const double theta_resolution,
    const std::vector<std::shared_ptr<OccupancyGridMap>>& gridmap_pyramid) {
  const auto& gridmap = gridmap_pyramid[level];
  const double trans_resolution = gridmap->gridResolution();
  const Eigen::Matrix2f rotation =
      Eigen::Rotation2Df(static_cast<float>(theta_resolution * theta)).toRotationMatrix();
  const Eigen::Vector2f translation(
      static_cast<float>(trans_resolution * x),
      static_cast<float>(trans_resolution * y));

  Points2D transformed;
  transformed.reserve(points.size());
  for (const auto& point : points) {
    transformed.emplace_back(rotation * point + translation);
  }

  score = gridmap->calcScore(transformed);
  return score;
}

std::vector<DiscreteTransformation> DiscreteTransformation::branch() const {
  return {
      DiscreteTransformation(level - 1, x * 2, y * 2, theta),
      DiscreteTransformation(level - 1, x * 2 + 1, y * 2, theta),
      DiscreteTransformation(level - 1, x * 2, y * 2 + 1, theta),
      DiscreteTransformation(level - 1, x * 2 + 1, y * 2 + 1, theta),
  };
}

BBSLocalization::BBSLocalization(BBSParams params) : params_(std::move(params)) {}

void BBSLocalization::setMap(
    const Points2D& map_points,
    const double resolution,
    const int width,
    const int height,
    const int pyramid_levels,
    const int max_points_per_cell) {
  gridmap_pyramid_.clear();
  gridmap_pyramid_.resize(std::max(1, pyramid_levels));
  gridmap_pyramid_[0] = std::make_shared<OccupancyGridMap>(resolution, width, height);
  gridmap_pyramid_[0]->insertPoints(map_points, max_points_per_cell);
  for (size_t level = 1; level < gridmap_pyramid_.size(); ++level) {
    gridmap_pyramid_[level] = gridmap_pyramid_[level - 1]->pyramidUp();
  }
}

boost::optional<Eigen::Isometry2f> BBSLocalization::localize(
    const Points2D& scan_points,
    const BBSSearchWindow& search_window,
    const double min_score,
    double* best_score) const {
  if (gridmap_pyramid_.empty() || scan_points.empty()) {
    return boost::none;
  }

  const double base_resolution = gridmap_pyramid_.front()->gridResolution();
  const double cos_argument = std::clamp(
      1.0 - (base_resolution * base_resolution) / (2.0 * params_.max_range * params_.max_range),
      -1.0,
      1.0);
  theta_resolution_ = std::acos(cos_argument);

  double score_storage = min_score;
  double* resolved_best_score = best_score ? best_score : &score_storage;
  *resolved_best_score = min_score;
  boost::optional<DiscreteTransformation> best_transform;

  auto queue = createInitialSet(scan_points, search_window);
  while (!queue.empty()) {
    auto transform = queue.top();
    queue.pop();

    if (transform.score < *resolved_best_score) {
      continue;
    }

    if (transform.isLeaf()) {
      best_transform = transform;
      *resolved_best_score = transform.score;
      continue;
    }

    for (auto child : transform.branch()) {
      child.calcScore(scan_points, theta_resolution_, gridmap_pyramid_);
      queue.push(child);
    }
  }

  if (best_transform == boost::none) {
    return boost::none;
  }
  return best_transform->transformation(theta_resolution_, gridmap_pyramid_);
}

std::priority_queue<DiscreteTransformation> BBSLocalization::createInitialSet(
    const Points2D& scan_points,
    const BBSSearchWindow& search_window) const {
  const double trans_resolution = gridmap_pyramid_.back()->gridResolution();
  const auto tx_range = std::pair<int, int>(
      static_cast<int>(std::floor(search_window.min_tx / trans_resolution)),
      static_cast<int>(std::ceil(search_window.max_tx / trans_resolution)));
  const auto ty_range = std::pair<int, int>(
      static_cast<int>(std::floor(search_window.min_ty / trans_resolution)),
      static_cast<int>(std::ceil(search_window.max_ty / trans_resolution)));
  const auto theta_range = std::pair<int, int>(
      static_cast<int>(std::floor(search_window.min_theta / theta_resolution_)),
      static_cast<int>(std::ceil(search_window.max_theta / theta_resolution_)));

  std::vector<DiscreteTransformation> transforms;
  const int top_level = static_cast<int>(gridmap_pyramid_.size()) - 1;
  for (int tx = tx_range.first; tx <= tx_range.second; ++tx) {
    for (int ty = ty_range.first; ty <= ty_range.second; ++ty) {
      for (int theta = theta_range.first; theta <= theta_range.second; ++theta) {
        transforms.emplace_back(top_level, tx, ty, theta);
      }
    }
  }

#pragma omp parallel for
  for (int index = 0; index < static_cast<int>(transforms.size()); ++index) {
    transforms[index].calcScore(scan_points, theta_resolution_, gridmap_pyramid_);
  }

  return std::priority_queue<DiscreteTransformation>(transforms.begin(), transforms.end());
}
