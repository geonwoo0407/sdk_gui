#ifndef IRC_STEP_MOTION_EXECUTOR__SDK_EXECUTOR_DRIVER_HPP_
#define IRC_STEP_MOTION_EXECUTOR__SDK_EXECUTOR_DRIVER_HPP_

#include "irc_step_motion_executor/sdk_executor_core.hpp"
#include "irc_step_motion_executor/startup_pose_gate.hpp"

#include <cstdint>
#include <functional>
#include <string>

namespace irc_step_motion_executor
{

class SdkExecutorDriver
{
public:
  using NowProvider = std::function<std::uint64_t()>;
  using StatusPublisher = std::function<void(const std::string &)>;

  SdkExecutorDriver(
    SdkExecutorCore & core, NowProvider now_provider,
    StatusPublisher status_publisher,
    StartupPoseGate * startup_pose_gate = nullptr);

  void handle_request(const std::string & payload);
  void handle_cancel(const std::string & payload);
  void poll();

private:
  void publish(
    const MotionStatus & status, bool suppress_duplicate = false);

  SdkExecutorCore & core_;
  NowProvider now_provider_;
  StatusPublisher status_publisher_;
  std::string last_published_payload_;
  StartupPoseGate * startup_pose_gate_;
};

}  // namespace irc_step_motion_executor

#endif  // IRC_STEP_MOTION_EXECUTOR__SDK_EXECUTOR_DRIVER_HPP_
