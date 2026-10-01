#include "irc_step_motion_executor/motion_backend_factory.hpp"
#include "irc_step_motion_executor/simulated_motion_backend.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#ifndef EXPECT_ROBOT_MOTION_PLAYER_BACKEND_BUILT
#define EXPECT_ROBOT_MOTION_PLAYER_BACKEND_BUILT 0
#endif

namespace
{

class TestBackend final : public irc_step_motion_executor::MotionBackend
{
public:
  explicit TestBackend(
    std::weak_ptr<int> owner, bool & destroyed_while_owner_alive)
  : owner_(std::move(owner)),
    destroyed_while_owner_alive_(destroyed_while_owner_alive)
  {
  }

  ~TestBackend() override
  {
    destroyed_while_owner_alive_ = !owner_.expired();
  }

  irc_step_motion_executor::BackendStartResult start_motion(
    const std::string &) override
  {
    return {true, "", ""};
  }

  irc_step_motion_executor::BackendCancelResult cancel_motion() override
  {
    return {true, "", ""};
  }

  irc_step_motion_executor::BackendStatus poll_status() override
  {
    return {irc_step_motion_executor::BackendState::IDLE, "", ""};
  }

private:
  std::weak_ptr<int> owner_;
  bool & destroyed_while_owner_alive_;
};

class FakeRuntimeFactory final
  : public irc_step_motion_executor::RobotMotionRuntimeFactory
{
public:
  irc_step_motion_executor::RobotMotionRuntimeFactoryResult create(
    const irc_step_motion_executor::RobotMotionRuntimeConfig & config) override
  {
    called = true;
    received_config = config;
    if (fail) {
      return {{}, "FAKE_RUNTIME_FAILURE", "fake runtime creation failed"};
    }
    auto owner = std::make_shared<int>(42);
    irc_step_motion_executor::RobotMotionRuntime runtime;
    runtime.runtime_owner = owner;
    runtime.backend =
      std::make_unique<TestBackend>(owner, backend_destroyed_safely);
    return {std::move(runtime), "", ""};
  }

  bool called{false};
  bool fail{false};
  bool backend_destroyed_safely{false};
  irc_step_motion_executor::RobotMotionRuntimeConfig received_config;
};

void configure_valid_hardware_policy(
  irc_step_motion_executor::MotionBackendFactoryOptions & options)
{
  options.enable_robot_hardware = true;
  options.robot_motion_player.enable_robot_hardware = true;
  options.robot_motion_player.motion_json_path = TEST_EXISTING_RUNTIME_FILE;
  options.robot_motion_player.device_path = "/dev/ttyUSB0";
  options.robot_motion_player.baud_rate = 4000000;
  for (std::int64_t motor_id = 0; motor_id <= 22; ++motor_id) {
    options.robot_motion_player.motor_ids.push_back(motor_id);
  }
  options.robot_motion_player.explicit_torque_approval = true;
}

TEST(MotionBackendFactory, DefaultAndExplicitSimulatedCreateBackend)
{
  irc_step_motion_executor::MotionBackendFactoryOptions defaults;
  auto default_result =
    irc_step_motion_executor::create_motion_backend(defaults);
  ASSERT_TRUE(default_result);
  EXPECT_NE(
    dynamic_cast<irc_step_motion_executor::SimulatedMotionBackend *>(
      default_result.backend.get()),
    nullptr);

  irc_step_motion_executor::MotionBackendFactoryOptions explicit_options;
  explicit_options.backend_type = "simulated";
  auto explicit_result =
    irc_step_motion_executor::create_motion_backend(explicit_options);
  ASSERT_TRUE(explicit_result);
  EXPECT_NE(
    dynamic_cast<irc_step_motion_executor::SimulatedMotionBackend *>(
      explicit_result.backend.get()),
    nullptr);
}

TEST(MotionBackendFactory, PassesSimulatedConfiguration)
{
  irc_step_motion_executor::MotionBackendFactoryOptions options;
  options.simulated.running_polls = 1;
  options.simulated.settling_polls = 2;
  options.simulated.force_start_failure = false;
  options.simulated.force_backend_failure = false;

  auto result = irc_step_motion_executor::create_motion_backend(options);
  ASSERT_TRUE(result);
  ASSERT_TRUE(result.backend->start_motion("resolved alias").accepted);
  EXPECT_EQ(
    result.backend->poll_status().state,
    irc_step_motion_executor::BackendState::RUNNING);
  EXPECT_EQ(
    result.backend->poll_status().state,
    irc_step_motion_executor::BackendState::SETTLING);
  EXPECT_EQ(
    result.backend->poll_status().state,
    irc_step_motion_executor::BackendState::SETTLING);
  EXPECT_EQ(
    result.backend->poll_status().state,
    irc_step_motion_executor::BackendState::SUCCEEDED);
}

TEST(MotionBackendFactory, RejectsUnknownTypeWithoutFallback)
{
  irc_step_motion_executor::MotionBackendFactoryOptions options;
  options.backend_type = "unknown";

  auto result = irc_step_motion_executor::create_motion_backend(options);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.backend, nullptr);
  EXPECT_EQ(result.error_code, "UNSUPPORTED_BACKEND_TYPE");
  EXPECT_NE(result.message.find("simulated"), std::string::npos);
  EXPECT_NE(result.message.find("robot_motion_player"), std::string::npos);
}

TEST(MotionBackendFactory, RealBackendRequestFailsWithoutSimulatedFallback)
{
  irc_step_motion_executor::MotionBackendFactoryOptions options;
  options.backend_type = "robot_motion_player";

  auto result = irc_step_motion_executor::create_motion_backend(options);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.backend, nullptr);
  EXPECT_EQ(result.error_code, "ROBOT_HARDWARE_NOT_ENABLED");
}

TEST(MotionBackendFactory, EnabledRealBackendRequiresBuiltRuntime)
{
  irc_step_motion_executor::MotionBackendFactoryOptions options;
  options.backend_type = "robot_motion_player";
  configure_valid_hardware_policy(options);

  auto result = irc_step_motion_executor::create_motion_backend(options);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.backend, nullptr);
#if EXPECT_ROBOT_MOTION_PLAYER_BACKEND_BUILT
  EXPECT_EQ(
    result.error_code,
    "ROBOT_MOTION_PLAYER_RUNTIME_NOT_CONFIGURED");
#else
  EXPECT_EQ(
    result.error_code,
    "ROBOT_MOTION_PLAYER_BACKEND_NOT_BUILT");
#endif
}

TEST(MotionBackendFactory, ProductionPolicyFailureNeverFallsBack)
{
  FakeRuntimeFactory runtime_factory;
  irc_step_motion_executor::MotionBackendFactoryOptions options;
  options.backend_type = "robot_motion_player";
  configure_valid_hardware_policy(options);
  options.robot_motion_player.device_path.clear();
  options.robot_motion_runtime_factory = &runtime_factory;

  const auto result =
    irc_step_motion_executor::create_motion_backend(options);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.backend, nullptr);
#if EXPECT_ROBOT_MOTION_PLAYER_BACKEND_BUILT
  EXPECT_EQ(result.error_code, "ROBOT_DEVICE_PATH_REQUIRED");
  EXPECT_FALSE(runtime_factory.called);
#else
  EXPECT_EQ(result.error_code, "ROBOT_MOTION_PLAYER_BACKEND_NOT_BUILT");
#endif
}

TEST(MotionBackendFactory, FixedProfileMismatchDoesNotCallRuntimeFactory)
{
  FakeRuntimeFactory runtime_factory;
  irc_step_motion_executor::MotionBackendFactoryOptions options;
  options.backend_type = "robot_motion_player";
  configure_valid_hardware_policy(options);
  options.robot_motion_player.device_path = "/dev/ttyUSB2";
  options.robot_motion_runtime_factory = &runtime_factory;

  const auto result =
    irc_step_motion_executor::create_motion_backend(options);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.backend, nullptr);
#if EXPECT_ROBOT_MOTION_PLAYER_BACKEND_BUILT
  EXPECT_EQ(result.error_code, "ROBOT_DEVICE_PATH_MISMATCH");
  EXPECT_FALSE(runtime_factory.called);
#else
  EXPECT_EQ(result.error_code, "ROBOT_MOTION_PLAYER_BACKEND_NOT_BUILT");
#endif
}

TEST(MotionBackendFactory, PassesConfigAndPreservesRuntimeOwnership)
{
  FakeRuntimeFactory runtime_factory;
  irc_step_motion_executor::MotionBackendFactoryOptions options;
  options.backend_type = "robot_motion_player";
  configure_valid_hardware_policy(options);
  options.robot_motion_runtime_factory = &runtime_factory;

  {
    auto result = irc_step_motion_executor::create_motion_backend(options);
#if EXPECT_ROBOT_MOTION_PLAYER_BACKEND_BUILT
    ASSERT_TRUE(result);
    EXPECT_TRUE(runtime_factory.called);
    EXPECT_EQ(
      runtime_factory.received_config.motion_json_path,
      TEST_EXISTING_RUNTIME_FILE);
    EXPECT_TRUE(runtime_factory.received_config.enable_robot_hardware);
    EXPECT_EQ(
      runtime_factory.received_config.device_path, "/dev/ttyUSB0");
    EXPECT_EQ(runtime_factory.received_config.baud_rate, 4000000);
    std::vector<std::int64_t> expected_motor_ids;
    for (std::int64_t motor_id = 0; motor_id <= 22; ++motor_id) {
      expected_motor_ids.push_back(motor_id);
    }
    EXPECT_EQ(runtime_factory.received_config.motor_ids, expected_motor_ids);
    EXPECT_TRUE(
      runtime_factory.received_config.explicit_torque_approval);
#else
    EXPECT_FALSE(result);
    EXPECT_FALSE(runtime_factory.called);
    EXPECT_EQ(
      result.error_code, "ROBOT_MOTION_PLAYER_BACKEND_NOT_BUILT");
#endif
  }
#if EXPECT_ROBOT_MOTION_PLAYER_BACKEND_BUILT
  EXPECT_TRUE(runtime_factory.backend_destroyed_safely);
#endif
}

TEST(MotionBackendFactory, RuntimeFactoryFailureNeverFallsBack)
{
  FakeRuntimeFactory runtime_factory;
  runtime_factory.fail = true;
  irc_step_motion_executor::MotionBackendFactoryOptions options;
  options.backend_type = "robot_motion_player";
  configure_valid_hardware_policy(options);
  options.robot_motion_runtime_factory = &runtime_factory;

  auto result = irc_step_motion_executor::create_motion_backend(options);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.backend, nullptr);
#if EXPECT_ROBOT_MOTION_PLAYER_BACKEND_BUILT
  EXPECT_TRUE(runtime_factory.called);
  EXPECT_EQ(result.error_code, "FAKE_RUNTIME_FAILURE");
#else
  EXPECT_FALSE(runtime_factory.called);
  EXPECT_EQ(
    result.error_code, "ROBOT_MOTION_PLAYER_BACKEND_NOT_BUILT");
#endif
}

}  // namespace
