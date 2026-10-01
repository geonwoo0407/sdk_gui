#ifndef IRC_STEP_MOTION_EXECUTOR__MOTION_BACKEND_FACTORY_HPP_
#define IRC_STEP_MOTION_EXECUTOR__MOTION_BACKEND_FACTORY_HPP_

#include "irc_step_motion_executor/motion_backend.hpp"
#include "irc_step_motion_executor/robot_motion_runtime_factory.hpp"
#include "irc_step_motion_executor/simulated_motion_backend.hpp"

#include <memory>
#include <string>

namespace irc_step_motion_executor
{

struct MotionBackendFactoryOptions
{
  std::string backend_type{"simulated"};
  SimulatedMotionBackendConfig simulated;
  bool enable_robot_hardware{false};
  RobotMotionRuntimeConfig robot_motion_player;
  RobotMotionRuntimeFactory * robot_motion_runtime_factory{nullptr};
};

struct MotionBackendFactoryResult
{
  // Declared first so it is destroyed after a backend that may borrow it.
  std::shared_ptr<void> runtime_owner;
  std::unique_ptr<MotionBackend> backend;
  std::string error_code;
  std::string message;
  StartupPoseController * startup_pose_controller{nullptr};

  explicit operator bool() const noexcept
  {
    return backend != nullptr;
  }
};

MotionBackendFactoryResult create_motion_backend(
  const MotionBackendFactoryOptions & options);

}  // namespace irc_step_motion_executor

#endif  // IRC_STEP_MOTION_EXECUTOR__MOTION_BACKEND_FACTORY_HPP_
