#include "fake_motion_backend.hpp"
#include "irc_step_motion_executor/sdk_executor_core.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#ifndef TEST_ALIAS_CONFIG
#define TEST_ALIAS_CONFIG ""
#endif

namespace
{

irc_step_motion_executor::MotionAliasCatalog load_catalog()
{
  irc_step_motion_executor::MotionAliasCatalog catalog;
  std::string error;
  EXPECT_TRUE(catalog.load(TEST_ALIAS_CONFIG, error)) << error;
  return catalog;
}

std::string request_json(
  std::int64_t request_id, const std::string & motion_id,
  const std::string & action = "STRAIGHT",
  const std::string & command_id = "17",
  const std::string & event_id = "29",
  std::int64_t timeout_ms = 5000)
{
  return
    "{\"action\":\"" + action +
    "\",\"command_id\":" + command_id +
    ",\"event_id\":" + event_id +
    ",\"request_id\":" + std::to_string(request_id) +
    ",\"motion_id\":\"" + motion_id +
    "\",\"timeout_ms\":" + std::to_string(timeout_ms) + "}";
}

TEST(SdkExecutorCore, ResolvesForwardAliasBeforeStartingBackend)
{
  FakeMotionBackend backend;
  irc_step_motion_executor::SdkExecutorCore core(load_catalog(), backend);

  const auto status = core.handle_request(
    request_json(1, "forward"), 100);

  ASSERT_EQ(backend.started_motion_names.size(), 1U);
  EXPECT_EQ(backend.started_motion_names[0], "전진45도(8회)");
  EXPECT_EQ(status.status, "RUNNING");
  EXPECT_TRUE(core.has_active_request());
}

TEST(SdkExecutorCore, ResolvesPickupAliasWithoutFallback)
{
  FakeMotionBackend backend;
  irc_step_motion_executor::SdkExecutorCore core(load_catalog(), backend);

  const auto status = core.handle_request(
    request_json(2, "pickup", "PICKUP_NOW"), 100);

  ASSERT_EQ(backend.started_motion_names.size(), 1U);
  EXPECT_EQ(backend.started_motion_names[0], "공잡기리그랩까지 실전");
  EXPECT_EQ(status.motion_id, "pickup");
}

TEST(SdkExecutorCore, UnsupportedMotionNeverCallsBackend)
{
  FakeMotionBackend backend;
  irc_step_motion_executor::SdkExecutorCore core(load_catalog(), backend);

  const auto status = core.handle_request(
    request_json(3, "turn_left", "TURN_LEFT"), 100);

  EXPECT_TRUE(backend.started_motion_names.empty());
  EXPECT_EQ(status.status, "REJECTED");
  EXPECT_EQ(status.error_code, "INVALID_MOTION");
  EXPECT_FALSE(core.has_active_request());
}

TEST(SdkExecutorCore, CorrelatedRejectionNeverStartsOrActivatesBackend)
{
  FakeMotionBackend backend;
  irc_step_motion_executor::SdkExecutorCore core(load_catalog(), backend);

  const auto status = core.reject_request(
    request_json(30, "forward", "STRAIGHT", "10", "null"),
    "STARTUP_POSE_GATE_LOCKED",
    "startup pose transition has not completed");

  EXPECT_TRUE(backend.started_motion_names.empty());
  EXPECT_FALSE(core.has_active_request());
  EXPECT_EQ(status.status, "REJECTED");
  EXPECT_EQ(status.error_code, "STARTUP_POSE_GATE_LOCKED");
  EXPECT_EQ(status.message, "startup pose transition has not completed");
  ASSERT_TRUE(status.action.has_value());
  EXPECT_EQ(*status.action, "STRAIGHT");
  ASSERT_TRUE(status.command_id.has_value());
  EXPECT_EQ(*status.command_id, 10);
  EXPECT_FALSE(status.event_id.has_value());
  EXPECT_EQ(status.request_id, 30);
  EXPECT_EQ(status.motion_id, "forward");
}

TEST(SdkExecutorCore, CorrelatedRejectionPreservesInvalidRequestHandling)
{
  FakeMotionBackend backend;
  irc_step_motion_executor::SdkExecutorCore core(load_catalog(), backend);

  const auto status = core.reject_request(
    "{invalid", "STARTUP_POSE_GATE_LOCKED", "not used");

  EXPECT_EQ(status.status, "REJECTED");
  EXPECT_EQ(status.error_code, "INVALID_REQUEST");
  EXPECT_TRUE(backend.started_motion_names.empty());
  EXPECT_FALSE(core.has_active_request());
}

TEST(SdkExecutorCore, PreservesCorrelationIncludingNullIds)
{
  FakeMotionBackend backend;
  backend.statuses.push_back(
    {irc_step_motion_executor::BackendState::SUCCEEDED, "", "done"});
  irc_step_motion_executor::SdkExecutorCore core(load_catalog(), backend);

  const auto started = core.handle_request(
    request_json(
      4, "forward", "APPROACH", "null", "null"), 100);
  ASSERT_TRUE(started.action.has_value());
  EXPECT_EQ(*started.action, "APPROACH");
  EXPECT_FALSE(started.command_id.has_value());
  EXPECT_FALSE(started.event_id.has_value());
  EXPECT_EQ(started.request_id, 4);
  EXPECT_EQ(started.motion_id, "forward");

  const auto terminal = core.poll(101);
  ASSERT_TRUE(terminal.has_value());
  EXPECT_EQ(terminal->status, "SUCCEEDED");
  EXPECT_EQ(terminal->action, started.action);
  EXPECT_EQ(terminal->command_id, started.command_id);
  EXPECT_EQ(terminal->event_id, started.event_id);
  EXPECT_EQ(terminal->request_id, started.request_id);
  EXPECT_EQ(terminal->motion_id, started.motion_id);
}

TEST(SdkExecutorCore, MapsRunningSettlingAndSucceeded)
{
  FakeMotionBackend backend;
  backend.statuses = {
    {irc_step_motion_executor::BackendState::RUNNING, "", "moving"},
    {irc_step_motion_executor::BackendState::SETTLING, "", "stabilizing"},
    {irc_step_motion_executor::BackendState::SUCCEEDED, "", "done"}};
  irc_step_motion_executor::SdkExecutorCore core(load_catalog(), backend);
  core.handle_request(request_json(5, "forward"), 100);

  const auto running = core.poll(101);
  ASSERT_TRUE(running.has_value());
  EXPECT_EQ(running->status, "RUNNING");
  const auto settling = core.poll(102);
  ASSERT_TRUE(settling.has_value());
  EXPECT_EQ(settling->status, "RUNNING");
  EXPECT_NE(settling->message.find("settling"), std::string::npos);
  const auto succeeded = core.poll(103);
  ASSERT_TRUE(succeeded.has_value());
  EXPECT_EQ(succeeded->status, "SUCCEEDED");
  EXPECT_FALSE(core.has_active_request());
}

TEST(SdkExecutorCore, MapsBackendFailureToFailedTerminal)
{
  FakeMotionBackend backend;
  backend.statuses.push_back(
    {irc_step_motion_executor::BackendState::FAILED,
      "MOTION_ERROR", "fake failure"});
  irc_step_motion_executor::SdkExecutorCore core(load_catalog(), backend);
  core.handle_request(request_json(6, "forward"), 100);

  const auto status = core.poll(101);
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->status, "FAILED");
  EXPECT_EQ(status->error_code, "MOTION_ERROR");
  EXPECT_FALSE(core.has_active_request());
}

TEST(SdkExecutorCore, CancelRequestCompletesWhenBackendReportsCancelled)
{
  FakeMotionBackend backend;
  backend.statuses.push_back(
    {irc_step_motion_executor::BackendState::CANCELLED, "", "cancelled"});
  irc_step_motion_executor::SdkExecutorCore core(load_catalog(), backend);
  core.handle_request(request_json(7, "forward"), 100);

  const auto cancelling = core.handle_cancel(R"({"request_id":7})");
  EXPECT_EQ(cancelling.status, "RUNNING");
  EXPECT_EQ(backend.cancel_calls, 1);
  const auto cancelled = core.poll(101);
  ASSERT_TRUE(cancelled.has_value());
  EXPECT_EQ(cancelled->status, "CANCELLED");
  EXPECT_FALSE(core.has_active_request());
}

TEST(SdkExecutorCore, RejectsCancelWithoutActiveRequest)
{
  FakeMotionBackend backend;
  irc_step_motion_executor::SdkExecutorCore core(load_catalog(), backend);

  const auto status = core.handle_cancel(R"({"request_id":8})");

  EXPECT_EQ(status.status, "REJECTED");
  EXPECT_EQ(status.error_code, "NOT_RUNNING");
  EXPECT_EQ(status.request_id, 8);
  EXPECT_EQ(backend.cancel_calls, 0);
}

TEST(SdkExecutorCore, RejectsStaleCancelWithoutTouchingActiveRequest)
{
  FakeMotionBackend backend;
  irc_step_motion_executor::SdkExecutorCore core(load_catalog(), backend);
  core.handle_request(request_json(9, "forward"), 100);

  const auto status = core.handle_cancel(R"({"request_id":99})");

  EXPECT_EQ(status.status, "REJECTED");
  EXPECT_EQ(status.error_code, "STALE_REQUEST");
  EXPECT_EQ(status.request_id, 9);
  EXPECT_EQ(backend.cancel_calls, 0);
  EXPECT_TRUE(core.has_active_request());
}

TEST(SdkExecutorCore, RejectsSecondStartWhileActive)
{
  FakeMotionBackend backend;
  irc_step_motion_executor::SdkExecutorCore core(load_catalog(), backend);
  core.handle_request(request_json(10, "forward"), 100);

  const auto status = core.handle_request(
    request_json(11, "pickup", "PICKUP_NOW"), 101);

  EXPECT_EQ(status.status, "REJECTED");
  EXPECT_EQ(status.error_code, "BUSY");
  EXPECT_EQ(status.request_id, 11);
  ASSERT_EQ(backend.started_motion_names.size(), 1U);
  EXPECT_EQ(backend.started_motion_names[0], "전진45도(8회)");
}

TEST(SdkExecutorCore, TimeoutCancelsAndReturnsFailedWithCorrelation)
{
  FakeMotionBackend backend;
  irc_step_motion_executor::SdkExecutorCore core(load_catalog(), backend);
  const auto started = core.handle_request(
    request_json(12, "forward", "STRAIGHT", "112", "212", 50), 100);

  const auto status = core.poll(150);

  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(backend.cancel_calls, 1);
  EXPECT_EQ(status->status, "FAILED");
  EXPECT_EQ(status->error_code, "TIMEOUT");
  EXPECT_EQ(status->action, started.action);
  EXPECT_EQ(status->command_id, started.command_id);
  EXPECT_EQ(status->event_id, started.event_id);
  EXPECT_EQ(status->request_id, 12);
  EXPECT_FALSE(core.has_active_request());
}

TEST(SdkExecutorCore, AllowsNewRequestAfterTerminalStatus)
{
  FakeMotionBackend backend;
  backend.statuses.push_back(
    {irc_step_motion_executor::BackendState::SUCCEEDED, "", "done"});
  irc_step_motion_executor::SdkExecutorCore core(load_catalog(), backend);
  core.handle_request(request_json(13, "forward"), 100);
  ASSERT_TRUE(core.poll(101).has_value());

  const auto status = core.handle_request(
    request_json(14, "hurdle", "GO"), 102);

  EXPECT_EQ(status.status, "RUNNING");
  ASSERT_EQ(backend.started_motion_names.size(), 2U);
  EXPECT_EQ(backend.started_motion_names[1], "허들넘기 실전");
}

TEST(SdkExecutorCore, BackendStartExceptionBecomesFailed)
{
  FakeMotionBackend backend;
  backend.throw_on_start = true;
  irc_step_motion_executor::SdkExecutorCore core(load_catalog(), backend);

  const auto status = core.handle_request(
    request_json(15, "forward"), 100);

  EXPECT_EQ(status.status, "FAILED");
  EXPECT_EQ(status.error_code, "BACKEND_EXCEPTION");
  EXPECT_FALSE(core.has_active_request());
}

TEST(SdkExecutorCore, ExplicitBackendStartRejectionStaysRejected)
{
  FakeMotionBackend backend;
  backend.start_result = {
    false, "BACKEND_NOT_READY", "fake backend is not ready"};
  irc_step_motion_executor::SdkExecutorCore core(load_catalog(), backend);

  const auto status = core.handle_request(
    request_json(16, "forward"), 100);

  EXPECT_EQ(status.status, "REJECTED");
  EXPECT_EQ(status.error_code, "BACKEND_NOT_READY");
  EXPECT_FALSE(core.has_active_request());
}

TEST(SdkExecutorCore, BackendPollExceptionBecomesFailedAndReleasesRequest)
{
  FakeMotionBackend backend;
  backend.throw_on_poll = true;
  irc_step_motion_executor::SdkExecutorCore core(load_catalog(), backend);
  core.handle_request(request_json(16, "forward"), 100);

  const auto failed = core.poll(101);
  ASSERT_TRUE(failed.has_value());
  EXPECT_EQ(failed->status, "FAILED");
  EXPECT_EQ(failed->error_code, "BACKEND_EXCEPTION");
  EXPECT_FALSE(core.has_active_request());

  backend.throw_on_poll = false;
  EXPECT_EQ(
    core.handle_request(request_json(17, "forward"), 102).status,
    "RUNNING");
}

TEST(SdkExecutorCore, BackendCancelExceptionBecomesFailedTerminal)
{
  FakeMotionBackend backend;
  backend.throw_on_cancel = true;
  irc_step_motion_executor::SdkExecutorCore core(load_catalog(), backend);
  core.handle_request(request_json(18, "forward"), 100);

  const auto status = core.handle_cancel(R"({"request_id":18})");

  EXPECT_EQ(status.status, "FAILED");
  EXPECT_EQ(status.error_code, "BACKEND_EXCEPTION");
  EXPECT_FALSE(core.has_active_request());
}

TEST(SdkExecutorCore, RejectsInvalidRequestWithoutCallingBackend)
{
  FakeMotionBackend backend;
  irc_step_motion_executor::SdkExecutorCore core(load_catalog(), backend);

  const auto missing_timeout = core.handle_request(
    R"({"action":"STRAIGHT","command_id":1,"event_id":2,)"
    R"("request_id":18,"motion_id":"forward"})",
    100);
  const auto invalid_json = core.handle_request("{invalid", 100);

  EXPECT_EQ(missing_timeout.status, "REJECTED");
  EXPECT_EQ(missing_timeout.error_code, "INVALID_REQUEST");
  EXPECT_EQ(invalid_json.error_code, "INVALID_REQUEST");
  EXPECT_TRUE(backend.started_motion_names.empty());
}

TEST(SdkExecutorCore, PromotesOneQueuedRequestAtSeamlessBoundary)
{
  FakeMotionBackend backend;
  backend.queue_result = {true, "", "queued"};
  irc_step_motion_executor::SdkExecutorCore core(load_catalog(), backend);

  EXPECT_EQ(
    core.handle_request(request_json(30, "forward"), 100).status,
    "RUNNING");
  const auto queued = core.handle_request(
    request_json(31, "forward", "STRAIGHT"), 101);
  EXPECT_EQ(queued.status, "QUEUED");
  ASSERT_EQ(backend.queued_motion_names.size(), 1U);
  EXPECT_EQ(backend.queued_motion_names[0], "전진45도(8회)");

  backend.sequence = 1;
  const auto completed = core.poll(200);
  ASSERT_TRUE(completed.has_value());
  EXPECT_EQ(completed->request_id, 30);
  EXPECT_EQ(completed->status, "SUCCEEDED");

  const auto promoted = core.poll(201);
  ASSERT_TRUE(promoted.has_value());
  EXPECT_EQ(promoted->request_id, 31);
  EXPECT_EQ(promoted->status, "RUNNING");
}

}  // namespace
