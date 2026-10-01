#ifndef IRC_STEP_MOTION_EXECUTOR__SIMULATED_MOTION_BACKEND_HPP_
#define IRC_STEP_MOTION_EXECUTOR__SIMULATED_MOTION_BACKEND_HPP_

#include "irc_step_motion_executor/motion_backend.hpp"

#include <cstddef>
#include <optional>
#include <string>

namespace irc_step_motion_executor
{

struct SimulatedMotionBackendConfig
{
  std::size_t running_polls{2};
  std::size_t settling_polls{1};
  bool force_start_failure{false};
  bool force_backend_failure{false};
};

class SimulatedMotionBackend : public MotionBackend
{
public:
  explicit SimulatedMotionBackend(SimulatedMotionBackendConfig config);

  BackendStartResult start_motion(
    const std::string & resolved_motion_name) override;
  BackendCancelResult cancel_motion() override;
  BackendStatus poll_status() override;
  bool set_joint_override(int motor_id, double target_deg) override;
  void clear_joint_override(int motor_id) noexcept override;

  const std::optional<std::string> & active_motion_name() const;

private:
  SimulatedMotionBackendConfig config_;
  std::optional<std::string> active_motion_name_;
  std::size_t poll_count_{0};
  bool cancel_pending_{false};
  std::optional<double> joint_zero_override_deg_;
};

}  // namespace irc_step_motion_executor

#endif  // IRC_STEP_MOTION_EXECUTOR__SIMULATED_MOTION_BACKEND_HPP_
