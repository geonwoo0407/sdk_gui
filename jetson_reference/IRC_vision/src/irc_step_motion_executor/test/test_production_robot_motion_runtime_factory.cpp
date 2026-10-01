#include "irc_step_motion_executor/robot_motion_runtime_factory.hpp"

#include "fake_robot_motion_sdk_test_support.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace
{

irc_step_motion_executor::RobotMotionRuntimeConfig complete_runtime_config()
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

TEST(ProductionRobotMotionRuntimeFactory, CreatesInitializedOwnedRuntime)
{
  irc_step::fake_sdk::reset_tracking();
  irc_step_motion_executor::ProductionRobotMotionRuntimeFactory factory;
  const auto config = complete_runtime_config();

  auto result = factory.create(config);

  ASSERT_TRUE(result);
  ASSERT_NE(result.runtime.backend, nullptr);
  ASSERT_NE(result.runtime.runtime_owner, nullptr);
  EXPECT_EQ(irc_step::fake_sdk::hardware_construction_count(), 1);
  EXPECT_EQ(irc_step::fake_sdk::player_construction_count(), 1);
  EXPECT_EQ(irc_step::fake_sdk::hardware_device_path(), "/dev/ttyUSB0");
  EXPECT_EQ(irc_step::fake_sdk::hardware_baud_rate(), 4000000);
  std::vector<int> expected_motor_ids;
  for (int motor_id = 0; motor_id <= 22; ++motor_id) {
    expected_motor_ids.push_back(motor_id);
  }
  EXPECT_EQ(irc_step::fake_sdk::hardware_motor_ids(), expected_motor_ids);
  EXPECT_EQ(irc_step::fake_sdk::hardware_initialize_count(), 1);
  EXPECT_EQ(irc_step::fake_sdk::hardware_preflight_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::player_initialize_count(), 1);

  const auto start_result = result.runtime.backend->start_motion("test_motion");
  EXPECT_TRUE(start_result.accepted);
  EXPECT_EQ(irc_step::fake_sdk::hardware_initialize_count(), 1);
  EXPECT_EQ(irc_step::fake_sdk::player_initialize_count(), 1);

  result.runtime.backend.reset();
  EXPECT_TRUE(irc_step::fake_sdk::destruction_order().empty());
  result.runtime.runtime_owner.reset();
  EXPECT_EQ(
    irc_step::fake_sdk::destruction_order(),
    (std::vector<std::string>{"player", "hardware"}));
}

TEST(ProductionRobotMotionRuntimeFactory, PreflightCreatesOwnedDiagnosticsRuntime)
{
  irc_step::fake_sdk::reset_tracking();
  irc_step_motion_executor::ProductionRobotMotionRuntimeFactory factory;
  auto config = complete_runtime_config();
  config.motion_json_path.clear();
  config.explicit_torque_approval = false;

  auto result = factory.preflight(config);

  ASSERT_TRUE(result);
  ASSERT_NE(result.runtime_owner, nullptr);
  EXPECT_TRUE(result.error_code.empty());
  EXPECT_EQ(irc_step::fake_sdk::hardware_construction_count(), 1);
  EXPECT_EQ(irc_step::fake_sdk::player_construction_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::hardware_preflight_count(), 1);
  EXPECT_EQ(irc_step::fake_sdk::hardware_initialize_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::player_initialize_count(), 0);
  EXPECT_TRUE(irc_step::fake_sdk::destruction_order().empty());

  result.runtime_owner.reset();
  EXPECT_EQ(
    irc_step::fake_sdk::destruction_order(),
    (std::vector<std::string>{"hardware"}));
}

TEST(ProductionRobotMotionRuntimeFactory, PreflightPolicyFailureCreatesNoSdkObjects)
{
  irc_step::fake_sdk::reset_tracking();
  irc_step_motion_executor::ProductionRobotMotionRuntimeFactory factory;
  auto config = complete_runtime_config();
  config.motion_json_path.clear();
  config.explicit_torque_approval = false;
  config.device_path = "/dev/ttyUSB2";

  const auto result = factory.preflight(config);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "ROBOT_DEVICE_PATH_MISMATCH");
  EXPECT_EQ(result.runtime_owner, nullptr);
  EXPECT_EQ(irc_step::fake_sdk::hardware_construction_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::player_construction_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::hardware_preflight_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::hardware_initialize_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::player_initialize_count(), 0);
}

TEST(ProductionRobotMotionRuntimeFactory, PreflightFailureReturnsHardwareError)
{
  irc_step::fake_sdk::reset_tracking();
  irc_step::fake_sdk::set_hardware_preflight_result(
    false, "fake preflight communication detail");
  irc_step_motion_executor::ProductionRobotMotionRuntimeFactory factory;
  auto config = complete_runtime_config();
  config.motion_json_path.clear();
  config.explicit_torque_approval = false;

  const auto result = factory.preflight(config);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "ROBOT_MOTION_RUNTIME_PREFLIGHT_FAILED");
  EXPECT_NE(
    result.message.find("fake preflight communication detail"),
    std::string::npos);
  EXPECT_EQ(result.runtime_owner, nullptr);
  EXPECT_EQ(irc_step::fake_sdk::hardware_construction_count(), 1);
  EXPECT_EQ(irc_step::fake_sdk::player_construction_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::hardware_preflight_count(), 1);
  EXPECT_EQ(irc_step::fake_sdk::hardware_initialize_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::player_initialize_count(), 0);
  EXPECT_EQ(
    irc_step::fake_sdk::destruction_order(),
    (std::vector<std::string>{"hardware"}));
}

TEST(ProductionRobotMotionRuntimeFactory, PreflightDoesNotConstructPlayer)
{
  irc_step::fake_sdk::reset_tracking();
  irc_step::fake_sdk::set_player_constructor_throws(true);
  irc_step_motion_executor::ProductionRobotMotionRuntimeFactory factory;
  auto config = complete_runtime_config();
  config.motion_json_path.clear();
  config.explicit_torque_approval = false;

  auto result = factory.preflight(config);

  EXPECT_TRUE(result);
  EXPECT_NE(result.runtime_owner, nullptr);
  EXPECT_TRUE(result.error_code.empty());
  EXPECT_EQ(irc_step::fake_sdk::hardware_construction_count(), 1);
  EXPECT_EQ(irc_step::fake_sdk::player_construction_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::hardware_preflight_count(), 1);
  EXPECT_EQ(irc_step::fake_sdk::hardware_initialize_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::player_initialize_count(), 0);
  EXPECT_TRUE(irc_step::fake_sdk::destruction_order().empty());

  result.runtime_owner.reset();
  EXPECT_EQ(
    irc_step::fake_sdk::destruction_order(),
    (std::vector<std::string>{"hardware"}));
}

TEST(ProductionRobotMotionRuntimeFactory, InvalidConfigErrorTakesPrecedence)
{
  irc_step::fake_sdk::reset_tracking();
  irc_step_motion_executor::ProductionRobotMotionRuntimeFactory factory;

  const auto result = factory.create({});

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "ROBOT_HARDWARE_NOT_ENABLED");
  EXPECT_EQ(result.runtime.backend, nullptr);
  EXPECT_EQ(result.runtime.runtime_owner, nullptr);
  EXPECT_EQ(irc_step::fake_sdk::hardware_construction_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::player_construction_count(), 0);
}

TEST(ProductionRobotMotionRuntimeFactory, ConvertsConstructionException)
{
  irc_step::fake_sdk::reset_tracking();
  irc_step::fake_sdk::set_player_constructor_throws(true);
  irc_step_motion_executor::ProductionRobotMotionRuntimeFactory factory;
  const auto config = complete_runtime_config();

  const auto result = factory.create(config);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "ROBOT_MOTION_RUNTIME_CREATION_FAILED");
  EXPECT_NE(result.message.find("fake player construction failed"), std::string::npos);
  EXPECT_EQ(result.runtime.backend, nullptr);
  EXPECT_EQ(result.runtime.runtime_owner, nullptr);
  EXPECT_EQ(irc_step::fake_sdk::hardware_initialize_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::player_initialize_count(), 0);
  EXPECT_EQ(
    irc_step::fake_sdk::destruction_order(),
    (std::vector<std::string>{"hardware"}));
}

TEST(ProductionRobotMotionRuntimeFactory, RejectsOutOfRangeMotorIdBeforeSdkCreation)
{
  irc_step::fake_sdk::reset_tracking();
  irc_step_motion_executor::ProductionRobotMotionRuntimeFactory factory;
  auto config = complete_runtime_config();
  config.motor_ids.front() =
    static_cast<std::int64_t>(std::numeric_limits<int>::max()) + 1;

  const auto result = factory.create(config);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "ROBOT_MOTOR_ID_OUT_OF_RANGE");
  EXPECT_EQ(irc_step::fake_sdk::hardware_construction_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::player_construction_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::hardware_initialize_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::player_initialize_count(), 0);
}

TEST(ProductionRobotMotionRuntimeFactory, InitializationFailureDestroysOwner)
{
  irc_step::fake_sdk::reset_tracking();
  irc_step::fake_sdk::set_player_initialize_result(
    false, "fake SDK initialization detail");
  irc_step_motion_executor::ProductionRobotMotionRuntimeFactory factory;

  const auto result = factory.create(complete_runtime_config());

  EXPECT_FALSE(result);
  EXPECT_EQ(
    result.error_code, "ROBOT_MOTION_RUNTIME_INITIALIZATION_FAILED");
  EXPECT_NE(
    result.message.find("fake SDK initialization detail"), std::string::npos);
  EXPECT_EQ(result.runtime.backend, nullptr);
  EXPECT_EQ(result.runtime.runtime_owner, nullptr);
  EXPECT_EQ(irc_step::fake_sdk::player_initialize_count(), 1);
  EXPECT_EQ(irc_step::fake_sdk::hardware_initialize_count(), 1);
  EXPECT_EQ(
    irc_step::fake_sdk::destruction_order(),
    (std::vector<std::string>{"player", "hardware"}));
}

TEST(ProductionRobotMotionRuntimeFactory, PolicyFailuresCreateNoSdkObjects)
{
  irc_step_motion_executor::ProductionRobotMotionRuntimeFactory factory;

  for (const auto & expected_and_config : {
      std::pair<std::string, irc_step_motion_executor::RobotMotionRuntimeConfig>{
        "ROBOT_HARDWARE_NOT_ENABLED", [] {
          auto config = complete_runtime_config();
          config.enable_robot_hardware = false;
          return config;
        }()},
      {"ROBOT_TORQUE_APPROVAL_REQUIRED", [] {
          auto config = complete_runtime_config();
          config.explicit_torque_approval = false;
          return config;
        }()},
      {"ROBOT_DEVICE_PATH_MISMATCH", [] {
          auto config = complete_runtime_config();
          config.device_path = "/dev/ttyUSB2";
          return config;
        }()}})
  {
    irc_step::fake_sdk::reset_tracking();
    const auto result = factory.create(expected_and_config.second);
    EXPECT_FALSE(result);
    EXPECT_EQ(result.error_code, expected_and_config.first);
    EXPECT_EQ(irc_step::fake_sdk::hardware_construction_count(), 0);
    EXPECT_EQ(irc_step::fake_sdk::player_construction_count(), 0);
    EXPECT_EQ(irc_step::fake_sdk::player_initialize_count(), 0);
    EXPECT_EQ(irc_step::fake_sdk::hardware_initialize_count(), 0);
  }
}

TEST(ProductionRobotMotionRuntimeFactory, PolicyValidationCreatesNoSdkObjects)
{
  irc_step::fake_sdk::reset_tracking();
  irc_step_motion_executor::RobotMotionRuntimeConfig config;
  config.motion_json_path = TEST_EXISTING_RUNTIME_FILE;
  config.enable_robot_hardware = true;
  config.device_path = "/dev/ttyUSB0";
  config.baud_rate = 4000000;
  for (std::int64_t motor_id = 0; motor_id <= 22; ++motor_id) {
    config.motor_ids.push_back(motor_id);
  }
  config.explicit_torque_approval = true;

  const auto result =
    irc_step_motion_executor::validate_robot_hardware_initialization_policy(config);

  EXPECT_TRUE(result);
  EXPECT_EQ(irc_step::fake_sdk::hardware_construction_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::player_construction_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::hardware_initialize_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::player_initialize_count(), 0);
}

}  // namespace
