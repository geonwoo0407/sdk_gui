#ifndef IRC_STEP_MOTION_EXECUTOR__ROBOT_MOTION_RUNTIME_FACTORY_HPP_
#define IRC_STEP_MOTION_EXECUTOR__ROBOT_MOTION_RUNTIME_FACTORY_HPP_

#include "irc_step_motion_executor/motion_backend.hpp"
#include "irc_step_motion_executor/robot_motion_runtime_config.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace irc_step_motion_executor
{

enum class StartupPoseState
{
  MOVING,
  SETTLING,
  SUCCEEDED,
  FAILED,
};

struct StartupPoseUpdate
{
  StartupPoseState state{StartupPoseState::FAILED};
  std::string error_code;
  std::string message;
};

class StartupPoseController
{
public:
  virtual ~StartupPoseController() = default;
  virtual bool start(
    const std::vector<double> & target_angles_deg, std::int64_t duration_ms,
    std::string & error_message) = 0;
  virtual StartupPoseUpdate update() = 0;
};

struct RobotMotionRuntime
{
  // Declared first so it is destroyed after the backend that may borrow it.
  std::shared_ptr<void> runtime_owner;
  std::unique_ptr<MotionBackend> backend;
  // Borrowed from runtime_owner; valid for the lifetime of runtime_owner.
  StartupPoseController * startup_pose_controller{nullptr};
};

struct RobotMotionRuntimeFactoryResult
{
  RobotMotionRuntime runtime;
  std::string error_code;
  std::string message;

  explicit operator bool() const noexcept
  {
    return runtime.backend != nullptr;
  }
};

struct RobotMotionPreflightResult
{
  std::shared_ptr<void> runtime_owner;
  std::string error_code;
  std::string message;

  explicit operator bool() const noexcept
  {
    return runtime_owner != nullptr && error_code.empty();
  }
};

class RobotMotionPreflightFactory
{
public:
  virtual ~RobotMotionPreflightFactory() = default;

  virtual RobotMotionPreflightResult preflight(
    const RobotMotionRuntimeConfig & config) = 0;
};

class RobotMotionRuntimeFactory
{
public:
  virtual ~RobotMotionRuntimeFactory() = default;

  virtual RobotMotionRuntimeFactoryResult create(
    const RobotMotionRuntimeConfig & config) = 0;
};

// The SDK-backed implementation is available only in SDK-enabled builds.
class ProductionRobotMotionRuntimeFactory final
  : public RobotMotionRuntimeFactory,
  public RobotMotionPreflightFactory
{
public:
  RobotMotionPreflightResult preflight(
    const RobotMotionRuntimeConfig & config) override;

  RobotMotionRuntimeFactoryResult create(
    const RobotMotionRuntimeConfig & config) override;
};

}  // namespace irc_step_motion_executor

#endif  // IRC_STEP_MOTION_EXECUTOR__ROBOT_MOTION_RUNTIME_FACTORY_HPP_
