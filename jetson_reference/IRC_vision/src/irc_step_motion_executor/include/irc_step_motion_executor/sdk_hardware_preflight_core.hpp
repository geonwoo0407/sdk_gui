#ifndef IRC_STEP_MOTION_EXECUTOR__SDK_HARDWARE_PREFLIGHT_CORE_HPP_
#define IRC_STEP_MOTION_EXECUTOR__SDK_HARDWARE_PREFLIGHT_CORE_HPP_

#include "irc_step_motion_executor/robot_motion_runtime_config.hpp"
#include "irc_step_motion_executor/robot_motion_runtime_factory.hpp"

#include <string>
#include <vector>

namespace irc_step_motion_executor
{

struct HardwarePreflightArgumentsResult
{
  RobotMotionRuntimeConfig config;
  std::string error;

  explicit operator bool() const noexcept
  {
    return error.empty();
  }
};

struct HardwarePreflightCommandResult
{
  int exit_code{1};
  std::string output;
  std::string error;
};

std::string hardware_preflight_usage();

HardwarePreflightArgumentsResult parse_hardware_preflight_arguments(
  const std::vector<std::string> & arguments) noexcept;

HardwarePreflightCommandResult run_hardware_preflight(
  RobotMotionPreflightFactory & factory,
  const RobotMotionRuntimeConfig & config) noexcept;

}  // namespace irc_step_motion_executor

#endif  // IRC_STEP_MOTION_EXECUTOR__SDK_HARDWARE_PREFLIGHT_CORE_HPP_
