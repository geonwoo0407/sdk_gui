#include "irc_step_motion_executor/startup_pose_gate.hpp"

#include <chrono>
#include <exception>
#include <utility>

namespace irc_step_motion_executor
{
namespace
{
constexpr std::uint64_t kAutoHoldDurationMs = 2000U;

std::uint64_t steady_now_ms()
{
  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}
}  // namespace

StartupPoseGate::StartupPoseGate(
  bool enabled, std::string pose_name, std::vector<double> target_angles_deg,
  std::int64_t duration_ms,
  StartupPoseController * controller, LogCallback log_callback,
  NowMsCallback now_ms_callback)
: state_(enabled ? State::LOCKED : State::DISABLED),
  pose_name_(std::move(pose_name)), target_angles_deg_(std::move(target_angles_deg)),
  duration_ms_(duration_ms),
  controller_(controller), log_callback_(std::move(log_callback)),
  now_ms_callback_(now_ms_callback ? std::move(now_ms_callback) : steady_now_ms)
{
  if (enabled && (pose_name_.empty() || target_angles_deg_.size() != 23U ||
    duration_ms_ <= 0 || controller_ == nullptr)) {
    fail("startup pose is enabled but its production controller/configuration is unavailable");
  }
}

void StartupPoseGate::poll()
{
  if (state_ == State::DISABLED || state_ == State::RELEASED || state_ == State::ERROR) {
    return;
  }
  try {
    if (state_ == State::HOLDING) {
      if (now_ms_callback_() - hold_started_at_ms_ >= kAutoHoldDurationMs) {
        state_ = State::RELEASED;
        log("[STARTUP POSE] AUTO ready");
      }
      return;
    }

    if (state_ == State::LOCKED) {
      std::string error;
      if (!controller_->start(target_angles_deg_, duration_ms_, error)) {
        fail(error.empty() ? "startup pose transition failed to start" : error);
        return;
      }
      log("[STARTUP POSE] Current pose captured");
      log("[STARTUP POSE] Moving to " + pose_name_);
      state_ = State::MOVING;
      return;
    }

    const StartupPoseUpdate update = controller_->update();
    if (update.state == StartupPoseState::MOVING) {
      return;
    }
    if (update.state == StartupPoseState::SETTLING) {
      if (state_ != State::SETTLING) {
        log("[STARTUP POSE] Settling");
      }
      state_ = State::SETTLING;
      return;
    }
    if (update.state == StartupPoseState::SUCCEEDED) {
      log("[STARTUP POSE] Reached " + pose_name_);
      hold_started_at_ms_ = now_ms_callback_();
      state_ = State::HOLDING;
      log("[STARTUP POSE] Holding for 2000 ms before AUTO");
      return;
    }
    fail(update.message.empty() ? "startup pose transition failed" : update.message);
  } catch (const std::exception & exception) {
    fail(std::string("startup pose exception: ") + exception.what());
  } catch (...) {
    fail("startup pose unknown exception");
  }
}

bool StartupPoseGate::navigation_allowed() const noexcept
{
  return state_ == State::DISABLED || state_ == State::RELEASED;
}

StartupPoseGate::State StartupPoseGate::state() const noexcept {return state_;}
const std::string & StartupPoseGate::error_message() const noexcept {return error_message_;}

void StartupPoseGate::fail(std::string message)
{
  error_message_ = std::move(message);
  state_ = State::ERROR;
  log("[STARTUP POSE] ERROR: " + error_message_);
}

void StartupPoseGate::log(const std::string & message) const
{
  if (log_callback_) {log_callback_(message);}
}

}  // namespace irc_step_motion_executor
