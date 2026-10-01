#include "irc_step_motion_executor/startup_pose_catalog.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace
{

std::string angles_json(int missing_id = -1, int changed_id = -1)
{
  std::string output = "{";
  bool first = true;
  for (int id = 0; id <= 22; ++id) {
    if (id == missing_id) {continue;}
    if (!first) {output += ",";}
    first = false;
    output += "\"" + std::to_string(id) + "\":" +
      std::to_string(id == changed_id ? 999.0 : static_cast<double>(id));
  }
  return output + "}";
}

std::filesystem::path write_catalog(
  const std::string & first_angles, const std::string & second_angles)
{
  const auto path = std::filesystem::temp_directory_path() /
    "irc_step_startup_pose_catalog_test.json";
  std::ofstream stream(path);
  stream << "{\"motions\":[{\"frames\":["
    << "{\"name\":\"오뒤307\",\"frame_id\":\"same\",\"angles\":"
    << first_angles << "},"
    << "{\"name\":\"오뒤307\",\"frame_id\":\"same\",\"angles\":"
    << second_angles << "}]}]}";
  return path;
}

TEST(StartupPoseCatalog, AcceptsRepeatedNameWithIdenticalAngles)
{
  const auto path = write_catalog(angles_json(), angles_json());
  std::vector<double> angles;
  std::string error;
  EXPECT_TRUE(irc_step_motion_executor::load_startup_pose_angles(
      path.string(), "오뒤307", angles, error)) << error;
  ASSERT_EQ(angles.size(), 23U);
  EXPECT_DOUBLE_EQ(angles[22], 22.0);
  std::filesystem::remove(path);
}

TEST(StartupPoseCatalog, RejectsRepeatedNameWithDifferentAngles)
{
  const auto path = write_catalog(angles_json(), angles_json(-1, 7));
  std::vector<double> angles;
  std::string error;
  EXPECT_FALSE(irc_step_motion_executor::load_startup_pose_angles(
      path.string(), "오뒤307", angles, error));
  EXPECT_EQ(error, "startup pose name is ambiguous: 오뒤307");
  std::filesystem::remove(path);
}

TEST(StartupPoseCatalog, RejectsRepeatedNameWithMissingMotor)
{
  const auto path = write_catalog(angles_json(), angles_json(12));
  std::vector<double> angles;
  std::string error;
  EXPECT_FALSE(irc_step_motion_executor::load_startup_pose_angles(
      path.string(), "오뒤307", angles, error));
  EXPECT_EQ(error, "startup pose name is ambiguous: 오뒤307");
  std::filesystem::remove(path);
}

}  // namespace
