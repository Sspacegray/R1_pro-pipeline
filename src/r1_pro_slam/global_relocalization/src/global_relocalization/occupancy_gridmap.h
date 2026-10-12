#ifndef INDOOR_LOCATION_OCCUPANCY_GRIDMAP_H_
#define INDOOR_LOCATION_OCCUPANCY_GRIDMAP_H_

#include <algorithm>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

class OccupancyGridMap {
 public:
  using Points2D = std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;
  using Ptr = std::shared_ptr<OccupancyGridMap>;

  OccupancyGridMap(const double resolution, const cv::Mat& values)
      : resolution_(resolution), values_(values) {}

  OccupancyGridMap(const double resolution, const int width, const int height)
      : resolution_(resolution), values_(cv::Mat1f(height, width, 0.0f)) {}

  double gridResolution() const { return resolution_; }

  void insertPoints(const Points2D& points, const int max_points_per_cell) {
    for (const auto& point : points) {
      const Eigen::Vector2i loc = gridLocation(point);
      if (inMap(loc)) {
        value(loc) += 1.0f;
      }
    }

    values_ /= std::max(1, max_points_per_cell);
    for (auto& cell : values_) {
      cell = std::min(cell, 1.0f);
    }
  }

  double calcScore(const Points2D& points) const {
    double score = 0.0;
    for (const auto& point : points) {
      Eigen::Vector2i loc = gridLocation(point);
      loc.x() = std::max(0, std::min(values_.cols - 1, loc.x()));
      loc.y() = std::max(0, std::min(values_.rows - 1, loc.y()));
      score += value(loc);
    }
    return score;
  }

  Ptr pyramidUp() const {
    cv::Mat1f up(values_.rows / 2, values_.cols / 2);
    for (int row = 0; row < up.rows; ++row) {
      for (int col = 0; col < up.cols; ++col) {
        float cell = values_.at<float>(row * 2, col * 2);
        cell = std::max(cell, values_.at<float>(row * 2 + 1, col * 2));
        cell = std::max(cell, values_.at<float>(row * 2, col * 2 + 1));
        cell = std::max(cell, values_.at<float>(row * 2 + 1, col * 2 + 1));
        up.at<float>(row, col) = cell;
      }
    }
    return std::make_shared<OccupancyGridMap>(resolution_ * 2.0, up);
  }

 private:
  bool inMap(const Eigen::Vector2i& pixel) const {
    return (pixel.array() >= Eigen::Array2i::Zero()).all() &&
           (pixel.array() < Eigen::Array2i(values_.cols, values_.rows)).all();
  }

  Eigen::Vector2i gridLocation(const Eigen::Vector2f& point) const {
    const Eigen::Vector2i raw = (point / resolution_).array().floor().cast<int>();
    return raw + Eigen::Vector2i(values_.cols / 2, values_.rows / 2);
  }

  float value(const Eigen::Vector2i& location) const {
    return values_.at<float>(location.y(), location.x());
  }

  float& value(const Eigen::Vector2i& location) {
    return values_.at<float>(location.y(), location.x());
  }

  double resolution_;
  cv::Mat1f values_;
};

#endif  // INDOOR_LOCATION_OCCUPANCY_GRIDMAP_H_
