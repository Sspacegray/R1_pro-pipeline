#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <cstdlib>

#include "pose_persistence/pose_persistence.h"

namespace {

pose_type makePose() {
  pose_type pose;
  pose.timestamp = 123.456;
  pose.status = "tracking";
  pose.pos = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
  pose.orient = Eigen::Vector3f(0.1f, -0.2f, 0.3f);
  pose.vel = Eigen::Vector3f(0.4f, 0.5f, 0.6f);
  return pose;
}

}  // namespace

TEST(PosePersistence, SaveAndLoadRoundTrip) {
  const auto test_dir = std::filesystem::temp_directory_path() / "indoor_location_pose_persistence";
  std::filesystem::create_directories(test_dir);
  const auto pose_file = test_dir / "pose.txt";

  PosePersistence persistence(pose_file.string());
  const pose_type input = makePose();

  ASSERT_TRUE(persistence.save(input, "TRACKING_OK", 0.88f, "tracking"));

  PosePersistence::LoadResult loaded;
  ASSERT_TRUE(persistence.load(loaded));
  EXPECT_NEAR(loaded.pose.pos.x(), input.pos.x(), 1e-5);
  EXPECT_NEAR(loaded.pose.pos.y(), input.pos.y(), 1e-5);
  EXPECT_NEAR(loaded.pose.pos.z(), input.pos.z(), 1e-5);
  EXPECT_NEAR(loaded.pose.orient.x(), input.orient.x(), 1e-5);
  EXPECT_NEAR(loaded.pose.orient.y(), input.orient.y(), 1e-5);
  EXPECT_NEAR(loaded.pose.orient.z(), input.orient.z(), 1e-5);
  EXPECT_EQ(loaded.state, "TRACKING_OK");
  EXPECT_NEAR(loaded.fitness, 0.88f, 1e-5);
  EXPECT_EQ(loaded.source, "tracking");
}

TEST(PosePersistence, RejectsInvalidFileContent) {
  const auto test_dir = std::filesystem::temp_directory_path() / "indoor_location_pose_persistence";
  std::filesystem::create_directories(test_dir);
  const auto pose_file = test_dir / "invalid_pose.txt";

  std::ofstream out(pose_file);
  out << "not-a-pose\n";
  out.close();

  PosePersistence persistence(pose_file.string());
  PosePersistence::LoadResult loaded;
  EXPECT_FALSE(persistence.load(loaded));
}

TEST(PosePersistence, RejectsNanValues) {
  const auto test_dir = std::filesystem::temp_directory_path() / "indoor_location_pose_persistence";
  std::filesystem::create_directories(test_dir);
  const auto pose_file = test_dir / "nan_pose.txt";

  std::ofstream out(pose_file);
  out << "1.0 nan 0 0 0 0 0 0 0.5 TRACKING_OK tracking\n";
  out.close();

  PosePersistence persistence(pose_file.string());
  PosePersistence::LoadResult loaded;
  EXPECT_FALSE(persistence.load(loaded));
}

TEST(PosePersistence, ResolvesHomePathTokens) {
  const char* home = std::getenv("HOME");
  ASSERT_NE(home, nullptr);

  const std::string tilde_path = "~/indoor_location_pose_test/last_pose.txt";
  const std::string env_path = "$HOME/indoor_location_pose_test/last_pose.txt";

  EXPECT_EQ(
      PosePersistence::ResolvePath(tilde_path),
      (std::filesystem::path(home) / "indoor_location_pose_test" / "last_pose.txt").string());
  EXPECT_EQ(
      PosePersistence::ResolvePath(env_path),
      (std::filesystem::path(home) / "indoor_location_pose_test" / "last_pose.txt").string());
}

TEST(PosePersistence, SaveOverwritesCorruptFileWithValidSnapshot) {
  const auto test_dir = std::filesystem::temp_directory_path() / "indoor_location_pose_persistence";
  std::filesystem::create_directories(test_dir);
  const auto pose_file = test_dir / "rewrite_pose.txt";

  {
    std::ofstream out(pose_file);
    out << "corrupt-payload";
  }

  PosePersistence persistence(pose_file.string());
  const pose_type input = makePose();
  ASSERT_TRUE(persistence.save(input, "TRACKING_OK", 0.42f, "rewrite"));

  PosePersistence::LoadResult loaded;
  ASSERT_TRUE(persistence.load(loaded));
  EXPECT_NEAR(loaded.pose.pos.x(), input.pos.x(), 1e-5);
  EXPECT_NEAR(loaded.pose.pos.y(), input.pos.y(), 1e-5);
  EXPECT_NEAR(loaded.pose.pos.z(), input.pos.z(), 1e-5);
  EXPECT_NEAR(loaded.pose.orient.z(), input.orient.z(), 1e-5);
  EXPECT_NEAR(loaded.fitness, 0.42f, 1e-5);
  EXPECT_EQ(loaded.state, "TRACKING_OK");
  EXPECT_EQ(loaded.source, "rewrite");
}
