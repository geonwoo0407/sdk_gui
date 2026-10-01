#ifndef IRC_STEP_MOTION_EXECUTOR__STARTUP_POSE_GATE_HPP_
#define IRC_STEP_MOTION_EXECUTOR__STARTUP_POSE_GATE_HPP_

#include "irc_step_motion_executor/robot_motion_runtime_factory.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace irc_step_motion_executor
{

class StartupPoseGate
{
public:
  enum class State {DISABLED, LOCKED, MOVING, SETTLING, HOLDING, RELEASED, ERROR};
  using LogCallback = std::function<void(const std::string &)>;
  using NowMsCallback = std::function<std::uint64_t()>;

  StartupPoseGate(
    bool enabled, std::string pose_name, std::vector<double> target_angles_deg,
    std::int64_t duration_ms,
    StartupPoseController * controller, LogCallback log_callback = {},
    NowMsCallback now_ms_callback = {});

  void poll();
  bool navigation_allowed() const noexcept;
  State state() const noexcept;
  const std::string & error_message() const noexcept;

private:
  void fail(std::string message);
  void log(const std::string & message) const;

  State state_;
  std::string pose_name_;
  std::vector<double> target_angles_deg_;
  std::int64_t duration_ms_;
  StartupPoseController * controller_;
  LogCallback log_callback_;
  NowMsCallback now_ms_callback_;
  std::uint64_t hold_started_at_ms_{0};
  std::string error_message_;
};

}  // namespace irc_step_motion_executor

#endif
