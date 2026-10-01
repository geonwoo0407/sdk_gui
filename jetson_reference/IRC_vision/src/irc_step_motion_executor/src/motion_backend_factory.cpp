#include "irc_step_motion_executor/motion_backend_factory.hpp"

#include <memory>

#ifndef IRC_STEP_ROBOT_MOTION_PLAYER_BACKEND_BUILT
#define IRC_STEP_ROBOT_MOTION_PLAYER_BACKEND_BUILT 0
#endif

namespace irc_step_motion_executor
{

MotionBackendFactoryResult create_motion_backend(
  const MotionBackendFactoryOptions & options)
{
  if (options.backend_type == "simulated") {
    MotionBackendFactoryResult result;
    result.backend =
      std::make_unique<SimulatedMotionBackend>(options.simulated);
    return result;
  }

  if (options.backend_type == "robot_motion_player") {
    if (!options.enable_robot_hardware) {
      return {
        nullptr,
        nullptr,
        "ROBOT_HARDWARE_NOT_ENABLED",
        "RobotMotionPlayer hardware requires enable_robot_hardware=true"};
    }
#if IRC_STEP_ROBOT_MOTION_PLAYER_BACKEND_BUILT
    const auto config_result =
      validate_robot_hardware_initialization_policy(
      options.robot_motion_player);
    if (!config_result) {
      return {
        nullptr, nullptr,
        config_result.error_code, config_result.message};
    }
    if (options.robot_motion_runtime_factory == nullptr) {
      return {
        nullptr,
        nullptr,
        "ROBOT_MOTION_PLAYER_RUNTIME_NOT_CONFIGURED",
        "RobotMotionPlayer adapter is built, but the production runtime "
        "factory is not configured"};
    }
    auto runtime_result =
      options.robot_motion_runtime_factory->create(config_result.config);
    if (!runtime_result) {
      return {
        nullptr, nullptr,
        runtime_result.error_code, runtime_result.message};
    }
    MotionBackendFactoryResult result;
    result.startup_pose_controller =
      runtime_result.runtime.startup_pose_controller;
    result.runtime_owner = std::move(runtime_result.runtime.runtime_owner);
    result.backend = std::move(runtime_result.runtime.backend);
    return result;
#else
    return {
      nullptr,
      nullptr,
      "ROBOT_MOTION_PLAYER_BACKEND_NOT_BUILT",
      "RobotMotionPlayer backend was not built; enable the SDK explicitly"};
#endif
  }

  return {
    nullptr,
    nullptr,
    "UNSUPPORTED_BACKEND_TYPE",
    "unsupported backend_type '" + options.backend_type +
    "'; allowed values are: simulated, robot_motion_player"};
}

}  // namespace irc_step_motion_executor
