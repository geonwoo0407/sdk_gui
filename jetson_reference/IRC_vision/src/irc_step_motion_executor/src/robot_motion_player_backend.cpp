#include "irc_step_motion_executor/robot_motion_player_backend.hpp"

#include <exception>
#include <string>
#include <string_view>

namespace irc_step_motion_executor
{
namespace
{

std::string message_or_default(
  const std::string & message, const std::string & default_message)
{
  return message.empty() ? default_message : message;
}

template<typename PlayerT>
bool set_position_tolerance_enabled(PlayerT & player, bool enabled)
{
  if constexpr (requires {
      player.setPositionToleranceEnabled(enabled);
    })
  {
    player.setPositionToleranceEnabled(enabled);
    return true;
  }
  return enabled;
}

BackendStartResult map_start_result(
  irc_step::StartResult result, const std::string & message)
{
  switch (result) {
    case irc_step::StartResult::Accepted:
      return {true, "", message};
    case irc_step::StartResult::RejectedBusy:
      return {
        false, "SDK_BUSY",
        message_or_default(message, "RobotMotionPlayer is busy")};
    case irc_step::StartResult::MotionNotFound:
      return {
        false, "SDK_MOTION_NOT_FOUND",
        message_or_default(message, "SDK motion was not found")};
    case irc_step::StartResult::HardwareNotReady:
      return {
        false, "SDK_HARDWARE_NOT_READY",
        message_or_default(message, "SDK hardware is not ready")};
    case irc_step::StartResult::InvalidMotion:
      return {
        false, "SDK_INVALID_MOTION",
        message_or_default(message, "SDK rejected an invalid motion")};
    default:
      return {
        false, "SDK_UNKNOWN_START_RESULT",
        "RobotMotionPlayer returned an unknown start result"};
  }
}

BackendCancelResult map_cancel_result(
  irc_step::CancelResult result, const std::string & message)
{
  switch (result) {
    case irc_step::CancelResult::Cancelled:
      return {true, "", message};
    case irc_step::CancelResult::NotRunning:
      return {
        false, "NOT_RUNNING",
        message_or_default(message, "SDK has no running motion")};
    case irc_step::CancelResult::HardwareNotReady:
      return {
        false, "SDK_HARDWARE_NOT_READY",
        message_or_default(message, "SDK hardware is not ready for cancel")};
    case irc_step::CancelResult::HoldFailed:
      return {
        false, "SDK_CANCEL_HOLD_FAILED",
        message_or_default(message, "SDK failed to hold current position")};
    default:
      return {
        false, "SDK_UNKNOWN_CANCEL_RESULT",
        "RobotMotionPlayer returned an unknown cancel result"};
  }
}

BackendQueueResult map_queue_result(
  irc_step::QueueResult result, const std::string & message)
{
  switch (result) {
    case irc_step::QueueResult::Queued:
      return {true, "", message};
    case irc_step::QueueResult::RejectedNotRunning:
      return {false, "QUEUE_NOT_RUNNING", message};
    case irc_step::QueueResult::RejectedQueueFull:
      return {false, "QUEUE_FULL", message};
    case irc_step::QueueResult::MotionNotFound:
      return {false, "SDK_MOTION_NOT_FOUND", message};
    case irc_step::QueueResult::HardwareNotReady:
      return {false, "SDK_HARDWARE_NOT_READY", message};
    case irc_step::QueueResult::IncompatibleTransition:
      return {false, "INCOMPATIBLE_TRANSITION", message};
    case irc_step::QueueResult::InvalidMotion:
    default:
      return {false, "SDK_INVALID_MOTION", message};
  }
}

std::string map_motion_error(irc_step::MotionError error)
{
  switch (error) {
    case irc_step::MotionError::None:
      return "SDK_FAILED";
    case irc_step::MotionError::JsonError:
      return "SDK_JSON_ERROR";
    case irc_step::MotionError::HardwareNotReady:
      return "SDK_HARDWARE_NOT_READY";
    case irc_step::MotionError::CommunicationError:
      return "SDK_COMMUNICATION_ERROR";
    case irc_step::MotionError::FrameSendFailed:
      return "SDK_FRAME_SEND_FAILED";
    case irc_step::MotionError::PresentPositionReadFailed:
      return "SDK_PRESENT_POSITION_READ_FAILED";
    case irc_step::MotionError::PositionTimeout:
      return "SDK_POSITION_TIMEOUT";
    case irc_step::MotionError::CancelFailed:
      return "SDK_CANCEL_FAILED";
    case irc_step::MotionError::InternalError:
      return "SDK_INTERNAL_ERROR";
    default:
      return "SDK_UNKNOWN_ERROR";
  }
}

BackendStatus map_motion_status(
  irc_step::MotionStatus status, irc_step::MotionError error,
  const std::string & message)
{
  switch (status) {
    case irc_step::MotionStatus::Idle:
      return {BackendState::IDLE, "", message};
    case irc_step::MotionStatus::Running:
      return {BackendState::RUNNING, "", message};
    case irc_step::MotionStatus::Settling:
      return {BackendState::SETTLING, "", message};
    case irc_step::MotionStatus::Succeeded:
      return {BackendState::SUCCEEDED, "", message};
    case irc_step::MotionStatus::Cancelled:
      return {BackendState::CANCELLED, "", message};
    case irc_step::MotionStatus::Failed:
      return {
        BackendState::FAILED, map_motion_error(error),
        message_or_default(message, "RobotMotionPlayer reported failure")};
    default:
      return {
        BackendState::FAILED, "SDK_UNKNOWN_STATUS",
        "RobotMotionPlayer returned an unknown status"};
  }
}

std::string exception_message(
  const std::string & operation, const std::exception & exception)
{
  return operation + " exception: " + exception.what();
}

}  // namespace

BorrowedRobotMotionPlayerApi::BorrowedRobotMotionPlayerApi(
  irc_step::RobotMotionPlayer & player)
: player_(player)
{
}

irc_step::StartResult BorrowedRobotMotionPlayerApi::start(
  std::string_view motion_name)
{
  return player_.start(motion_name);
}

irc_step::QueueResult BorrowedRobotMotionPlayerApi::queue_next(
  std::string_view motion_name)
{
  return player_.queueNext(motion_name);
}

irc_step::CancelResult BorrowedRobotMotionPlayerApi::cancel()
{
  return player_.cancel();
}

irc_step::MotionStatus BorrowedRobotMotionPlayerApi::update()
{
  return player_.update();
}

irc_step::MotionError BorrowedRobotMotionPlayerApi::result() const
{
  return player_.result();
}

std::string BorrowedRobotMotionPlayerApi::last_error() const
{
  return std::string(player_.lastError());
}

std::uint64_t BorrowedRobotMotionPlayerApi::completion_sequence() const
{
  return player_.completionSequence();
}

void BorrowedRobotMotionPlayerApi::set_joint_override(
  int motor_id, double target_deg)
{
  player_.setJointOverride(motor_id, target_deg);
}

void BorrowedRobotMotionPlayerApi::clear_joint_override(int motor_id) noexcept
{
  player_.clearJointOverride(motor_id);
}

bool BorrowedRobotMotionPlayerApi::set_position_tolerance_enabled(bool enabled)
{
  return irc_step_motion_executor::set_position_tolerance_enabled(
    player_, enabled);
}

RobotMotionPlayerBackend::RobotMotionPlayerBackend(
  RobotMotionPlayerApi & player_api)
: player_api_(player_api)
{
}

BackendStartResult RobotMotionPlayerBackend::start_motion(
  const std::string & resolved_motion_name)
{
  try {
    const auto result = player_api_.start(resolved_motion_name);
    return map_start_result(result, player_api_.last_error());
  } catch (const std::exception & exception) {
    return {
      false, "SDK_EXCEPTION",
      exception_message("RobotMotionPlayer::start", exception)};
  } catch (...) {
    return {
      false, "SDK_EXCEPTION",
      "RobotMotionPlayer::start threw an unknown exception"};
  }
}

BackendCancelResult RobotMotionPlayerBackend::cancel_motion()
{
  try {
    const auto result = player_api_.cancel();
    return map_cancel_result(result, player_api_.last_error());
  } catch (const std::exception & exception) {
    return {
      false, "SDK_EXCEPTION",
      exception_message("RobotMotionPlayer::cancel", exception)};
  } catch (...) {
    return {
      false, "SDK_EXCEPTION",
      "RobotMotionPlayer::cancel threw an unknown exception"};
  }
}

BackendStatus RobotMotionPlayerBackend::poll_status()
{
  try {
    const auto status = player_api_.update();
    return map_motion_status(
      status, player_api_.result(), player_api_.last_error());
  } catch (const std::exception & exception) {
    return {
      BackendState::FAILED, "SDK_EXCEPTION",
      exception_message("RobotMotionPlayer::update", exception)};
  } catch (...) {
    return {
      BackendState::FAILED, "SDK_EXCEPTION",
      "RobotMotionPlayer::update threw an unknown exception"};
  }
}

BackendQueueResult RobotMotionPlayerBackend::queue_motion(
  const std::string & resolved_motion_name)
{
  try {
    return map_queue_result(
      player_api_.queue_next(resolved_motion_name), player_api_.last_error());
  } catch (const std::exception & exception) {
    return {false, "SDK_EXCEPTION", exception_message("queue_next", exception)};
  } catch (...) {
    return {false, "SDK_EXCEPTION", "queue_next threw an unknown exception"};
  }
}

bool RobotMotionPlayerBackend::set_joint_override(
  int motor_id, double target_deg)
{
  try {
    player_api_.set_joint_override(motor_id, target_deg);
    return true;
  } catch (...) {
    return false;
  }
}

void RobotMotionPlayerBackend::clear_joint_override(int motor_id) noexcept
{
  player_api_.clear_joint_override(motor_id);
}

bool RobotMotionPlayerBackend::set_position_tolerance_enabled(bool enabled)
{
  return player_api_.set_position_tolerance_enabled(enabled);
}

std::uint64_t RobotMotionPlayerBackend::completion_sequence() const
{
  return player_api_.completion_sequence();
}

}  // namespace irc_step_motion_executor
