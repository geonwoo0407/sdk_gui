#include "irc_step_motion_executor/sdk_executor_driver.hpp"

#include "irc_step_motion_executor/catalog_only_core.hpp"

#include <utility>

namespace irc_step_motion_executor
{

SdkExecutorDriver::SdkExecutorDriver(
  SdkExecutorCore & core, NowProvider now_provider,
  StatusPublisher status_publisher, StartupPoseGate * startup_pose_gate)
: core_(core),
  now_provider_(std::move(now_provider)),
  status_publisher_(std::move(status_publisher)),
  startup_pose_gate_(startup_pose_gate)
{
}

void SdkExecutorDriver::handle_request(const std::string & payload)
{
  if (startup_pose_gate_ != nullptr &&
    !startup_pose_gate_->navigation_allowed())
  {
    const std::string message = startup_pose_gate_->error_message().empty() ?
      "startup pose transition has not completed" :
      startup_pose_gate_->error_message();
    publish(core_.reject_request(
      payload, "STARTUP_POSE_GATE_LOCKED", message));
    return;
  }
  publish(core_.handle_request(payload, now_provider_()));
}

void SdkExecutorDriver::handle_cancel(const std::string & payload)
{
  publish(core_.handle_cancel(payload));
}

void SdkExecutorDriver::poll()
{
  if (startup_pose_gate_ != nullptr &&
    !startup_pose_gate_->navigation_allowed())
  {
    startup_pose_gate_->poll();
    return;
  }
  const auto status = core_.poll(now_provider_());
  if (status) {
    publish(*status, status->status == "RUNNING");
  }
}

void SdkExecutorDriver::publish(
  const MotionStatus & status, bool suppress_duplicate)
{
  const std::string payload = CatalogOnlyCore::to_json(status);
  if (suppress_duplicate && payload == last_published_payload_) {
    return;
  }
  last_published_payload_ = payload;
  status_publisher_(payload);
}

}  // namespace irc_step_motion_executor
