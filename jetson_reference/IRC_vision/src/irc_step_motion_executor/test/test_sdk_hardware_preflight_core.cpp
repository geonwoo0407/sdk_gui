#include "irc_step_motion_executor/sdk_hardware_preflight_core.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

class FakePreflightFactory final
  : public irc_step_motion_executor::RobotMotionPreflightFactory
{
public:
  irc_step_motion_executor::RobotMotionPreflightResult preflight(
    const irc_step_motion_executor::RobotMotionRuntimeConfig & config) override
  {
    ++preflight_calls;
    received_config = config;
    if (throw_exception) {
      throw std::runtime_error("fake preflight exception");
    }
    return result;
  }

  int preflight_calls{0};
  bool throw_exception{false};
  irc_step_motion_executor::RobotMotionRuntimeConfig received_config;
  irc_step_motion_executor::RobotMotionPreflightResult result;
};

std::vector<std::string> valid_arguments()
{
  return {
    "--device", "/dev/ttyUSB0",
    "--baud", "4000000",
    "--motor-ids", "0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22",
    "--confirm-hardware-access", "PREFLIGHT_ONLY_TORQUE_OFF"};
}

TEST(SdkHardwarePreflightCore, CallsPreflightOnceAndReportsSafeSuccess)
{
  FakePreflightFactory factory;
  factory.result.runtime_owner = std::make_shared<int>(1);
  irc_step_motion_executor::RobotMotionRuntimeConfig config;

  const auto result =
    irc_step_motion_executor::run_hardware_preflight(factory, config);

  EXPECT_EQ(result.exit_code, 0);
  EXPECT_EQ(factory.preflight_calls, 1);
  EXPECT_NE(result.output.find("Torque remains OFF"), std::string::npos);
  EXPECT_NE(result.output.find("not motion-ready"), std::string::npos);
  EXPECT_TRUE(result.error.empty());
}

TEST(SdkHardwarePreflightCore, PreservesFailureCodeAndMessage)
{
  FakePreflightFactory factory;
  factory.result.error_code = "FAKE_PREFLIGHT_FAILURE";
  factory.result.message = "fake diagnostic detail";

  const auto result = irc_step_motion_executor::run_hardware_preflight(
    factory, irc_step_motion_executor::RobotMotionRuntimeConfig{});

  EXPECT_NE(result.exit_code, 0);
  EXPECT_EQ(factory.preflight_calls, 1);
  EXPECT_NE(result.error.find("FAKE_PREFLIGHT_FAILURE"), std::string::npos);
  EXPECT_NE(result.error.find("fake diagnostic detail"), std::string::npos);
  EXPECT_TRUE(result.output.empty());
}

TEST(SdkHardwarePreflightCore, ConvertsFactoryException)
{
  FakePreflightFactory factory;
  factory.throw_exception = true;

  const auto result = irc_step_motion_executor::run_hardware_preflight(
    factory, irc_step_motion_executor::RobotMotionRuntimeConfig{});

  EXPECT_NE(result.exit_code, 0);
  EXPECT_EQ(factory.preflight_calls, 1);
  EXPECT_NE(
    result.error.find("ROBOT_MOTION_PREFLIGHT_COMMAND_EXCEPTION"),
    std::string::npos);
  EXPECT_NE(result.error.find("fake preflight exception"), std::string::npos);
}

TEST(SdkHardwarePreflightParser, ParsesCompleteArguments)
{
  const auto result =
    irc_step_motion_executor::parse_hardware_preflight_arguments(valid_arguments());

  ASSERT_TRUE(result);
  EXPECT_TRUE(result.config.motion_json_path.empty());
  EXPECT_TRUE(result.config.enable_robot_hardware);
  EXPECT_EQ(result.config.device_path, "/dev/ttyUSB0");
  EXPECT_EQ(result.config.baud_rate, 4000000);
  ASSERT_EQ(result.config.motor_ids.size(), 23U);
  EXPECT_EQ(result.config.motor_ids.front(), 0);
  EXPECT_EQ(result.config.motor_ids.back(), 22);
  EXPECT_FALSE(result.config.explicit_torque_approval);
}

TEST(SdkHardwarePreflightParser, RejectsMissingRequiredOption)
{
  auto arguments = valid_arguments();
  arguments.erase(arguments.begin(), arguments.begin() + 2);

  const auto result =
    irc_step_motion_executor::parse_hardware_preflight_arguments(arguments);

  EXPECT_FALSE(result);
  EXPECT_NE(result.error.find("--device"), std::string::npos);
}

TEST(SdkHardwarePreflightParser, RejectsMissingHardwareAccessConfirmation)
{
  auto arguments = valid_arguments();
  arguments.erase(arguments.end() - 2, arguments.end());

  const auto result =
    irc_step_motion_executor::parse_hardware_preflight_arguments(arguments);

  EXPECT_FALSE(result);
  EXPECT_NE(
    result.error.find("--confirm-hardware-access"), std::string::npos);
}

TEST(SdkHardwarePreflightParser, RejectsIncorrectHardwareAccessConfirmation)
{
  auto arguments = valid_arguments();
  arguments.back() = "PREFLIGHT_ONLY";

  const auto result =
    irc_step_motion_executor::parse_hardware_preflight_arguments(arguments);

  EXPECT_FALSE(result);
  EXPECT_NE(
    result.error.find("must exactly equal PREFLIGHT_ONLY_TORQUE_OFF"),
    std::string::npos);
}

TEST(SdkHardwarePreflightParser, ParserFailureDoesNotCallFakeFactory)
{
  FakePreflightFactory factory;
  auto arguments = valid_arguments();
  arguments.back() = "TORQUE_ON";

  const auto parsed =
    irc_step_motion_executor::parse_hardware_preflight_arguments(arguments);
  if (parsed) {
    (void)irc_step_motion_executor::run_hardware_preflight(
      factory, parsed.config);
  }

  EXPECT_FALSE(parsed);
  EXPECT_EQ(factory.preflight_calls, 0);
}

TEST(SdkHardwarePreflightParser, RejectsMissingOptionValue)
{
  const auto result =
    irc_step_motion_executor::parse_hardware_preflight_arguments({"--baud"});

  EXPECT_FALSE(result);
  EXPECT_NE(result.error.find("missing value"), std::string::npos);
}

TEST(SdkHardwarePreflightParser, RejectsUnknownAndTorqueApprovalOptions)
{
  const auto unknown =
    irc_step_motion_executor::parse_hardware_preflight_arguments({"--unknown"});
  const auto torque =
    irc_step_motion_executor::parse_hardware_preflight_arguments(
    {"--approve-torque"});
  const auto motion_json =
    irc_step_motion_executor::parse_hardware_preflight_arguments(
    {"--motion-json", "/tmp/motions.json"});

  EXPECT_FALSE(unknown);
  EXPECT_NE(unknown.error.find("unknown option"), std::string::npos);
  EXPECT_FALSE(torque);
  EXPECT_NE(torque.error.find("unknown option"), std::string::npos);
  EXPECT_FALSE(motion_json);
  EXPECT_NE(motion_json.error.find("unknown option"), std::string::npos);
}

TEST(SdkHardwarePreflightParser, RejectsMalformedBaud)
{
  auto arguments = valid_arguments();
  arguments[3] = "4000000baud";

  const auto result =
    irc_step_motion_executor::parse_hardware_preflight_arguments(arguments);

  EXPECT_FALSE(result);
  EXPECT_NE(result.error.find("complete integer"), std::string::npos);
}

TEST(SdkHardwarePreflightParser, RejectsMalformedMotorIdLists)
{
  for (const std::string motor_ids :
    {"0,,2", "0,two,2", "0,1,", "0,1,1"})
  {
    auto arguments = valid_arguments();
    arguments[5] = motor_ids;

    const auto result =
      irc_step_motion_executor::parse_hardware_preflight_arguments(arguments);

    EXPECT_FALSE(result) << motor_ids;
    EXPECT_NE(result.error.find("--motor-ids"), std::string::npos);
  }
}

}  // namespace
