#ifndef IRC_STEP_MOTION_EXECUTOR__MOTION_BACKEND_HPP_
#define IRC_STEP_MOTION_EXECUTOR__MOTION_BACKEND_HPP_

#include <string>
#include <cstdint>

namespace irc_step_motion_executor
{

enum class BackendState
{
  IDLE,
  RUNNING,
  SETTLING,
  SUCCEEDED,
  CANCELLED,
  FAILED,
};

struct BackendStartResult
{
  bool accepted{false};
  std::string error_code;
  std::string message;
};

struct BackendCancelResult
{
  bool accepted{false};
  std::string error_code;
  std::string message;
};

struct BackendQueueResult
{
  bool accepted{false};
  std::string error_code;
  std::string message;
};

struct BackendStatus
{
  BackendState state{BackendState::IDLE};
  std::string error_code;
  std::string message;
};

class MotionBackend
{
public:
  virtual ~MotionBackend() = default;

  virtual BackendStartResult start_motion(
    const std::string & resolved_motion_name) = 0;
  virtual BackendCancelResult cancel_motion() = 0;
  virtual BackendStatus poll_status() = 0;
  virtual BackendQueueResult queue_motion(const std::string &)
  {
    return {false, "QUEUE_UNSUPPORTED", "backend does not support queuing"};
  }
  virtual bool set_joint_override(int motor_id, double target_deg)
  {
    static_cast<void>(motor_id);
    static_cast<void>(target_deg);
    return false;
  }
  virtual void clear_joint_override(int motor_id) noexcept
  {
    static_cast<void>(motor_id);
  }
  virtual bool set_position_tolerance_enabled(bool enabled)
  {
    return enabled;
  }
  virtual std::uint64_t completion_sequence() const {return 0;}
};

}  // namespace irc_step_motion_executor

#endif  // IRC_STEP_MOTION_EXECUTOR__MOTION_BACKEND_HPP_
