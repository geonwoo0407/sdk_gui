#include "irc_step_motion_executor/robot_motion_runtime_config.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <algorithm>
#include <map>
#include <string>
#include <vector>

namespace
{

irc_step_motion_executor::RobotMotionRuntimeConfig valid_hardware_policy()
{
  irc_step_motion_executor::RobotMotionRuntimeConfig config;
  config.motion_json_path = TEST_EXISTING_RUNTIME_FILE;
  config.enable_robot_hardware = true;
  config.device_path = "/dev/ttyUSB0";
  config.baud_rate = 4000000;
  for (std::int64_t motor_id = 0; motor_id <= 22; ++motor_id) {
    config.motor_ids.push_back(motor_id);
  }
  config.explicit_torque_approval = true;
  return config;
}

TEST(RobotMotionRuntimeConfig, RejectsMissingPath)
{
  const auto result =
    irc_step_motion_executor::validate_robot_motion_runtime_config({});

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "MOTION_JSON_PATH_REQUIRED");
}

TEST(RobotMotionRuntimeConfig, RejectsMissingFile)
{
  irc_step_motion_executor::RobotMotionRuntimeConfig config;
  config.motion_json_path =
    "/definitely/not/a/robot_motion_runtime_config.json";

  const auto result =
    irc_step_motion_executor::validate_robot_motion_runtime_config(config);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "MOTION_JSON_FILE_NOT_FOUND");
}

TEST(RobotMotionRuntimeConfig, RejectsUnknownSetting)
{
  const auto result =
    irc_step_motion_executor::parse_robot_motion_runtime_config(
    {{"device_path", "/dev/ttyUSB0"}});

  EXPECT_FALSE(result);
  EXPECT_EQ(
    result.error_code, "UNKNOWN_ROBOT_MOTION_RUNTIME_SETTING");
}

TEST(RobotMotionRuntimeConfig, CopiesTypedRosParameterValues)
{
  const auto config =
    irc_step_motion_executor::make_robot_motion_runtime_config(
    "/tmp/motions.json", true, "/dev/ttyUSB9", 123456,
    {2, 4, 6}, true);

  EXPECT_EQ(config.motion_json_path, "/tmp/motions.json");
  EXPECT_TRUE(config.enable_robot_hardware);
  EXPECT_EQ(config.device_path, "/dev/ttyUSB9");
  EXPECT_EQ(config.baud_rate, 123456);
  EXPECT_EQ(config.motor_ids, (std::vector<std::int64_t>{2, 4, 6}));
  EXPECT_TRUE(config.explicit_torque_approval);
}

TEST(RobotHardwareInitializationPolicy, DefaultConfigDisablesHardware)
{
  const irc_step_motion_executor::RobotMotionRuntimeConfig config;
  EXPECT_FALSE(config.enable_robot_hardware);
  EXPECT_TRUE(config.device_path.empty());
  EXPECT_EQ(config.baud_rate, 0);
  EXPECT_TRUE(config.motor_ids.empty());
  EXPECT_FALSE(config.explicit_torque_approval);

  const auto result =
    irc_step_motion_executor::validate_robot_hardware_initialization_policy(
    config);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "ROBOT_HARDWARE_NOT_ENABLED");
}

TEST(RobotHardwareInitializationPolicy, RequiresDevicePath)
{
  auto config = valid_hardware_policy();
  config.device_path.clear();

  const auto result =
    irc_step_motion_executor::validate_robot_hardware_initialization_policy(config);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "ROBOT_DEVICE_PATH_REQUIRED");
}

TEST(RobotHardwareInitializationPolicy, RejectsZeroBaudRate)
{
  auto config = valid_hardware_policy();
  config.baud_rate = 0;

  const auto result =
    irc_step_motion_executor::validate_robot_hardware_initialization_policy(config);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "ROBOT_BAUD_RATE_INVALID");
}

TEST(RobotHardwareInitializationPolicy, RejectsNegativeBaudRate)
{
  auto config = valid_hardware_policy();
  config.baud_rate = -1;

  const auto result =
    irc_step_motion_executor::validate_robot_hardware_initialization_policy(config);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "ROBOT_BAUD_RATE_INVALID");
}

TEST(RobotHardwareInitializationPolicy, RequiresMotorIds)
{
  auto config = valid_hardware_policy();
  config.motor_ids.clear();

  const auto result =
    irc_step_motion_executor::validate_robot_hardware_initialization_policy(config);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "ROBOT_MOTOR_IDS_REQUIRED");
}

TEST(RobotHardwareInitializationPolicy, RejectsDuplicateMotorIds)
{
  auto config = valid_hardware_policy();
  config.motor_ids = {0, 1, 1};

  const auto result =
    irc_step_motion_executor::validate_robot_hardware_initialization_policy(config);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "ROBOT_MOTOR_ID_DUPLICATED");
}

TEST(RobotHardwareInitializationPolicy, RejectsIdsOutsideCurrentSdkRange)
{
  auto config = valid_hardware_policy();
  config.motor_ids = {0, 23};

  const auto result =
    irc_step_motion_executor::validate_robot_hardware_initialization_policy(config);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "ROBOT_MOTOR_ID_OUT_OF_RANGE");
}

TEST(RobotHardwareInitializationPolicy, RequiresTorqueApproval)
{
  auto config = valid_hardware_policy();
  config.explicit_torque_approval = false;

  const auto result =
    irc_step_motion_executor::validate_robot_hardware_initialization_policy(config);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "ROBOT_TORQUE_APPROVAL_REQUIRED");
}

TEST(RobotHardwareInitializationPolicy, RejectsDevicePathMismatch)
{
  auto config = valid_hardware_policy();
  config.device_path = "/dev/ttyUSB2";

  const auto result =
    irc_step_motion_executor::validate_robot_hardware_initialization_policy(config);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "ROBOT_DEVICE_PATH_MISMATCH");
}

TEST(RobotHardwareInitializationPolicy, AcceptsAlternateUsbEnumeration)
{
  auto config = valid_hardware_policy();
  config.device_path = "/dev/ttyUSB1";

  const auto result =
    irc_step_motion_executor::validate_robot_hardware_initialization_policy(config);

  EXPECT_TRUE(result);
  EXPECT_EQ(result.config.device_path, "/dev/ttyUSB1");
}

TEST(RobotHardwareInitializationPolicy, RejectsBaudRateMismatch)
{
  auto config = valid_hardware_policy();
  config.baud_rate = 1000000;

  const auto result =
    irc_step_motion_executor::validate_robot_hardware_initialization_policy(config);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "ROBOT_BAUD_RATE_MISMATCH");
}

TEST(RobotHardwareInitializationPolicy, RejectsIncompleteMotorIdSet)
{
  auto config = valid_hardware_policy();
  config.motor_ids.pop_back();

  const auto result =
    irc_step_motion_executor::validate_robot_hardware_initialization_policy(config);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "ROBOT_MOTOR_IDS_MISMATCH");
}

TEST(RobotHardwareInitializationPolicy, AcceptsReversedCompleteMotorIdSet)
{
  auto config = valid_hardware_policy();
  std::reverse(config.motor_ids.begin(), config.motor_ids.end());

  const auto result =
    irc_step_motion_executor::validate_robot_hardware_initialization_policy(config);

  EXPECT_TRUE(result);
}

TEST(RobotHardwareInitializationPolicy, AcceptsAllPrerequisites)
{
  const auto result =
    irc_step_motion_executor::validate_robot_hardware_initialization_policy(
    valid_hardware_policy());

  EXPECT_TRUE(result);
  EXPECT_TRUE(result.config.enable_robot_hardware);
}

TEST(RobotHardwarePreflightPolicy, DoesNotRequireMotionJsonOrTorqueApproval)
{
  auto config = valid_hardware_policy();
  config.motion_json_path.clear();
  config.explicit_torque_approval = false;

  const auto result =
    irc_step_motion_executor::validate_robot_hardware_preflight_policy(config);

  EXPECT_TRUE(result);
  EXPECT_TRUE(result.config.motion_json_path.empty());
  EXPECT_FALSE(result.config.explicit_torque_approval);
}

TEST(RobotHardwareInitializationPolicy, RequiresMotionJsonPath)
{
  auto config = valid_hardware_policy();
  config.motion_json_path.clear();

  const auto result =
    irc_step_motion_executor::validate_robot_hardware_initialization_policy(config);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "MOTION_JSON_PATH_REQUIRED");
}

TEST(RobotHardwarePreflightPolicy, EnforcesFixedHardwareProfile)
{
  auto config = valid_hardware_policy();
  config.motion_json_path.clear();
  config.explicit_torque_approval = false;
  config.baud_rate = 1000000;

  const auto result =
    irc_step_motion_executor::validate_robot_hardware_preflight_policy(config);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "ROBOT_BAUD_RATE_MISMATCH");
}

}  // namespace
