#ifndef IRC_STEP_MOTION_EXECUTOR__ROBOT_MOTION_PLAYER_BACKEND_HPP_
#define IRC_STEP_MOTION_EXECUTOR__ROBOT_MOTION_PLAYER_BACKEND_HPP_

#include "irc_step_motion_executor/motion_backend.hpp"

#include "robot_motion_player.hpp"

#include <string>
#include <string_view>

namespace irc_step_motion_executor
{

class RobotMotionPlayerApi
{
public:
  virtual ~RobotMotionPlayerApi() = default;

  virtual irc_step::StartResult start(std::string_view motion_name) = 0;
  virtual irc_step::QueueResult queue_next(std::string_view motion_name) = 0;
  virtual irc_step::CancelResult cancel() = 0;
  virtual irc_step::MotionStatus update() = 0;
  virtual irc_step::MotionError result() const = 0;
  virtual std::string last_error() const = 0;
  virtual std::uint64_t completion_sequence() const = 0;
  virtual void set_joint_override(int motor_id, double target_deg) = 0;
  virtual void clear_joint_override(int motor_id) noexcept = 0;
  virtual bool set_position_tolerance_enabled(bool enabled)
  {
    return enabled;
  }
};

class BorrowedRobotMotionPlayerApi final : public RobotMotionPlayerApi
{
public:
  explicit BorrowedRobotMotionPlayerApi(irc_step::RobotMotionPlayer & player);

  irc_step::StartResult start(std::string_view motion_name) override;
  irc_step::QueueResult queue_next(std::string_view motion_name) override;
  irc_step::CancelResult cancel() override;
  irc_step::MotionStatus update() override;
  irc_step::MotionError result() const override;
  std::string last_error() const override;
  std::uint64_t completion_sequence() const override;
  void set_joint_override(int motor_id, double target_deg) override;
  void clear_joint_override(int motor_id) noexcept override;
  bool set_position_tolerance_enabled(bool enabled) override;

private:
  irc_step::RobotMotionPlayer & player_;
};

class RobotMotionPlayerBackend final : public MotionBackend
{
public:
  explicit RobotMotionPlayerBackend(RobotMotionPlayerApi & player_api);

  BackendStartResult start_motion(
    const std::string & resolved_motion_name) override;
  BackendCancelResult cancel_motion() override;
  BackendStatus poll_status() override;
  BackendQueueResult queue_motion(
    const std::string & resolved_motion_name) override;
  bool set_joint_override(int motor_id, double target_deg) override;
  void clear_joint_override(int motor_id) noexcept override;
  bool set_position_tolerance_enabled(bool enabled) override;
  std::uint64_t completion_sequence() const override;

private:
  RobotMotionPlayerApi & player_api_;
};

}  // namespace irc_step_motion_executor

#endif  // IRC_STEP_MOTION_EXECUTOR__ROBOT_MOTION_PLAYER_BACKEND_HPP_
