#include "pose_persistence/pose_persistence.h"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <system_error>

#include <fcntl.h>
#include <unistd.h>

namespace {

bool WriteAll(const int fd, const std::string& content) {
  std::size_t written = 0;
  while (written < content.size()) {
    const ssize_t rc = ::write(fd, content.data() + written, content.size() - written);
    if (rc < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    written += static_cast<std::size_t>(rc);
  }
  return true;
}

bool SyncDirectory(const std::filesystem::path& directory) {
  const auto dir_path = directory.empty() ? std::filesystem::path(".") : directory;
  const int dir_fd = ::open(dir_path.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (dir_fd < 0) {
    return false;
  }

  const bool ok = (::fsync(dir_fd) == 0);
  ::close(dir_fd);
  return ok;
}

std::string HomeDirectory() {
  const char* home = std::getenv("HOME");
  return home ? std::string(home) : std::string();
}

}  // namespace

PosePersistence::PosePersistence(std::string path) : path_(ResolvePath(path)) {}

void PosePersistence::setPath(const std::string& path) {
  path_ = ResolvePath(path);
}

const std::string& PosePersistence::path() const {
  return path_;
}

bool PosePersistence::exists() const {
  return !path_.empty() && std::filesystem::exists(path_);
}

bool PosePersistence::save(
    const pose_type& pose,
    const std::string& state,
    const float fitness,
    const std::string& source) const {
  if (path_.empty() || !IsFinite(pose)) {
    return false;
  }

  const std::filesystem::path output_path(path_);
  if (output_path.has_parent_path()) {
    std::filesystem::create_directories(output_path.parent_path());
  }

  std::ostringstream content;
  content << std::setprecision(std::numeric_limits<double>::max_digits10)
          << pose.timestamp << ' '
          << pose.pos.x() << ' ' << pose.pos.y() << ' ' << pose.pos.z() << ' '
          << pose.orient.x() << ' ' << pose.orient.y() << ' ' << pose.orient.z() << ' '
          << fitness << ' ' << state << ' ' << source << '\n';

  const std::filesystem::path tmp_path = output_path.string() + ".tmp";
  const int fd = ::open(tmp_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (fd < 0) {
    return false;
  }

  bool ok = WriteAll(fd, content.str());
  if (ok) {
    ok = (::fsync(fd) == 0);
  }
  ::close(fd);

  if (!ok) {
    std::error_code remove_ec;
    std::filesystem::remove(tmp_path, remove_ec);
    return false;
  }

  std::error_code rename_ec;
  std::filesystem::rename(tmp_path, output_path, rename_ec);
  if (rename_ec) {
    std::error_code remove_ec;
    std::filesystem::remove(tmp_path, remove_ec);
    return false;
  }

  if (!SyncDirectory(output_path.parent_path())) {
    return false;
  }

  return true;
}

bool PosePersistence::load(LoadResult& result) const {
  if (!exists()) {
    return false;
  }

  std::ifstream in(path_);
  if (!in.is_open()) {
    return false;
  }

  LoadResult parsed;
  if (!(in >> parsed.timestamp
        >> parsed.pose.pos.x() >> parsed.pose.pos.y() >> parsed.pose.pos.z()
        >> parsed.pose.orient.x() >> parsed.pose.orient.y() >> parsed.pose.orient.z()
        >> parsed.fitness >> parsed.state >> parsed.source)) {
    return false;
  }

  parsed.pose.timestamp = parsed.timestamp;
  parsed.pose.status = parsed.state;
  parsed.pose.vel.setZero();
  update_q(parsed.pose);

  if (!std::isfinite(parsed.timestamp) || !std::isfinite(parsed.fitness) || !IsFinite(parsed.pose)) {
    return false;
  }

  result = parsed;
  return true;
}

bool PosePersistence::IsFinite(const pose_type& pose) {
  return std::isfinite(pose.pos.x()) &&
         std::isfinite(pose.pos.y()) &&
         std::isfinite(pose.pos.z()) &&
         std::isfinite(pose.orient.x()) &&
         std::isfinite(pose.orient.y()) &&
         std::isfinite(pose.orient.z());
}

std::string PosePersistence::ResolvePath(const std::string& path) {
  if (path.empty()) {
    return path;
  }

  const std::string home = HomeDirectory();
  if (home.empty()) {
    return path;
  }

  if (path.rfind("~/", 0) == 0) {
    return (std::filesystem::path(home) / path.substr(2)).string();
  }
  if (path == "~") {
    return home;
  }
  if (path.rfind("$HOME/", 0) == 0) {
    return (std::filesystem::path(home) / path.substr(6)).string();
  }
  if (path == "$HOME") {
    return home;
  }
  if (path.rfind("${HOME}/", 0) == 0) {
    return (std::filesystem::path(home) / path.substr(8)).string();
  }
  if (path == "${HOME}") {
    return home;
  }

  return path;
}
