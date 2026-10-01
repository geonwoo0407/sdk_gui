#ifndef FAKE_ROBOT_MOTION_PLAYER_HPP_
#define FAKE_ROBOT_MOTION_PLAYER_HPP_

#include "motion_hardware.hpp"

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace irc_step
{

enum class MotionStatus : std::uint8_t
{
  Idle,
  Running,
  Settling,
  Succeeded,
  Cancelled,
  Failed,
};

enum class StartResult : std::uint8_t
{
  Accepted,
  RejectedBusy,
  MotionNotFound,
  HardwareNotReady,
  InvalidMotion,
};

enum class QueueResult : std::uint8_t
{
  Queued,
  RejectedNotRunning,
  RejectedQueueFull,
  MotionNotFound,
  HardwareNotReady,
  IncompatibleTransition,
  InvalidMotion,
};

enum class CancelResult : std::uint8_t
{
  Cancelled,
  NotRunning,
  HardwareNotReady,
  HoldFailed,
};

enum class MotionError : std::uint8_t
{
  None,
  JsonError,
  HardwareNotReady,
  CommunicationError,
  FrameSendFailed,
  PresentPositionReadFailed,
  PositionTimeout,
  CancelFailed,
  InternalError,
};

class RobotMotionPlayer
{
public:
  RobotMotionPlayer() = default;
  RobotMotionPlayer(const std::string & json_path, IMotionHardware & hardware);
  ~RobotMotionPlayer();

  bool initialize() noexcept;
  StartResult start(std::string_view motion_name) noexcept;
  QueueResult queueNext(std::string_view motion_name) noexcept;
  std::uint64_t completionSequence() const noexcept;
  bool startPoseTransition(
    const std::vector<double> & target_angles_deg,
    std::int64_t duration_ms) noexcept;
  MotionStatus updateStartupPose() noexcept;
  CancelResult cancel() noexcept;
  MotionStatus update() noexcept;
  MotionError result() const noexcept;
  std::string_view lastError() const noexcept;
  void setJointOverride(int motor_id, double target_deg);
  void clearJointOverride(int motor_id) noexcept;

private:
  IMotionHardware * hardware_{nullptr};
  bool initialized_{false};
  std::string last_error_;
  std::uint64_t completion_sequence_{0};
};

}  // namespace irc_step

#endif  // FAKE_ROBOT_MOTION_PLAYER_HPP_
