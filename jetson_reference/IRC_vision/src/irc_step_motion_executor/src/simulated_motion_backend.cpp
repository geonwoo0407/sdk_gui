#include "irc_step_motion_executor/simulated_motion_backend.hpp"

#include <string>
#include <utility>

namespace irc_step_motion_executor
{

SimulatedMotionBackend::SimulatedMotionBackend(
  SimulatedMotionBackendConfig config)
: config_(config)
{
}

BackendStartResult SimulatedMotionBackend::start_motion(
  const std::string & resolved_motion_name)
{
  if (config_.force_start_failure) {
    return {
      false, "SIMULATED_START_FAILURE",
      "simulated backend start failure is enabled"};
  }
  if (active_motion_name_) {
    return {
      false, "SIMULATED_BUSY",
      "simulated backend already has an active motion"};
  }

  active_motion_name_ = resolved_motion_name;
  poll_count_ = 0;
  cancel_pending_ = false;
  return {true, "", "simulated motion started"};
}

BackendCancelResult SimulatedMotionBackend::cancel_motion()
{
  if (!active_motion_name_) {
    return {
      false, "NOT_RUNNING",
      "simulated backend has no active motion"};
  }
  cancel_pending_ = true;
  return {true, "", "simulated cancel accepted"};
}

BackendStatus SimulatedMotionBackend::poll_status()
{
  if (!active_motion_name_) {
    return {BackendState::IDLE, "", "simulated backend is idle"};
  }
  if (cancel_pending_) {
    active_motion_name_.reset();
    cancel_pending_ = false;
    return {BackendState::CANCELLED, "", "simulated motion cancelled"};
  }
  if (config_.force_backend_failure) {
    active_motion_name_.reset();
    return {
      BackendState::FAILED, "SIMULATED_BACKEND_FAILURE",
      "simulated backend failure is enabled"};
  }

  ++poll_count_;
  if (poll_count_ <= config_.running_polls) {
    return {BackendState::RUNNING, "", "simulated motion running"};
  }
  if (poll_count_ <=
    config_.running_polls + config_.settling_polls)
  {
    return {BackendState::SETTLING, "", "simulated motion stabilizing"};
  }

  active_motion_name_.reset();
  return {BackendState::SUCCEEDED, "", "simulated motion succeeded"};
}

bool SimulatedMotionBackend::set_joint_override(int motor_id, double target_deg)
{
  if (motor_id != 0) {
    return false;
  }
  joint_zero_override_deg_ = target_deg;
  return true;
}

void SimulatedMotionBackend::clear_joint_override(int motor_id) noexcept
{
  if (motor_id == 0) {
    joint_zero_override_deg_.reset();
  }
}

const std::optional<std::string> &
SimulatedMotionBackend::active_motion_name() const
{
  return active_motion_name_;
}

}  // namespace irc_step_motion_executor
