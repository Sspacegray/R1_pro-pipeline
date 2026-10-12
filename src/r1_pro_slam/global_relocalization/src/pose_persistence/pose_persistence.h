#ifndef INDOOR_LOCATION_POSE_PERSISTENCE_H_
#define INDOOR_LOCATION_POSE_PERSISTENCE_H_

#include <string>

#include "utility.h"

class PosePersistence {
 public:
  struct LoadResult {
    pose_type pose;
    double timestamp = 0.0;
    float fitness = 0.0f;
    std::string state;
    std::string source;
  };

  explicit PosePersistence(std::string path = "");

  void setPath(const std::string& path);
  const std::string& path() const;
  bool exists() const;

  bool save(const pose_type& pose, const std::string& state, float fitness, const std::string& source) const;
  bool load(LoadResult& result) const;

  static bool IsFinite(const pose_type& pose);
  static std::string ResolvePath(const std::string& path);

 private:
  std::string path_;
};

#endif  // INDOOR_LOCATION_POSE_PERSISTENCE_H_
