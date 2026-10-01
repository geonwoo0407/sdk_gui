#include "fake_motion_backend.hpp"
#include "irc_step_motion_executor/sdk_executor_driver.hpp"
#include "irc_step_motion_executor/startup_pose_gate.hpp"

#include <gtest/gtest.h>
#include <json-c/json.h>

#include <deque>
#include <string>
#include <vector>

#ifndef TEST_ALIAS_CONFIG
#define TEST_ALIAS_CONFIG ""
#endif

namespace
{

class FakeStartupPoseController : public irc_step_motion_executor::StartupPoseController
{
public:
  bool start_result{true};
  int start_calls{0};
  std::string received_name;
  std::int64_t received_duration_ms{0};
  std::deque<irc_step_motion_executor::StartupPoseUpdate> updates;

  bool start(const std::vector<double> & angles, std::int64_t duration, std::string & error) override
  {
    ++start_calls;
    received_name = angles.size() == 23U ? "angles-0-22" : "invalid";
    received_duration_ms = duration;
    if (!start_result) {error = "write failed";}
    return start_result;
  }

  irc_step_motion_executor::StartupPoseUpdate update() override
  {
    if (updates.empty()) {
      return {irc_step_motion_executor::StartupPoseState::MOVING, "", ""};
    }
    auto value = updates.front();
    updates.pop_front();
    return value;
  }
};

irc_step_motion_executor::MotionAliasCatalog catalog()
{
  irc_step_motion_executor::MotionAliasCatalog value;
  std::string error;
  EXPECT_TRUE(value.load(TEST_ALIAS_CONFIG, error)) << error;
  return value;
}

std::string request()
{
  return R"({"action":"STRAIGHT","command_id":1,"event_id":2,"request_id":3,"motion_id":"forward","timeout_ms":5000})";
}

std::string string_field(const std::string & payload, const char * key)
{
  json_object * object = json_tokener_parse(payload.c_str());
  EXPECT_NE(object, nullptr);
  if (object == nullptr) {
    return "";
  }
  json_object * value = nullptr;
  EXPECT_TRUE(json_object_object_get_ex(object, key, &value));
  const std::string result =
    value == nullptr ? "" : json_object_get_string(value);
  json_object_put(object);
  return result;
}

std::int64_t int_field(const std::string & payload, const char * key)
{
  json_object * object = json_tokener_parse(payload.c_str());
  EXPECT_NE(object, nullptr);
  if (object == nullptr) {
    return 0;
  }
  json_object * value = nullptr;
  EXPECT_TRUE(json_object_object_get_ex(object, key, &value));
  const std::int64_t result =
    value == nullptr ? 0 : json_object_get_int64(value);
  json_object_put(object);
  return result;
}

struct Fixture
{
  std::uint64_t now_ms{0};
  FakeMotionBackend backend;
  irc_step_motion_executor::SdkExecutorCore core{catalog(), backend};
  FakeStartupPoseController startup;
  irc_step_motion_executor::StartupPoseGate gate{
    true, "오뒤307", std::vector<double>(23, 0.0), 1800, &startup, {},
    [this]() {return now_ms;}};
  std::vector<std::string> statuses;
  irc_step_motion_executor::SdkExecutorDriver driver{
    core, []() {return 100U;},
    [this](const std::string & value) {statuses.push_back(value);}, &gate};
};

TEST(StartupPoseGate, DoesNotStartBeforeFirstReadyPollAndStartsExactlyOnce)
{
  Fixture fixture;
  EXPECT_EQ(fixture.startup.start_calls, 0);
  fixture.driver.poll();
  fixture.driver.poll();
  EXPECT_EQ(fixture.startup.start_calls, 1);
  EXPECT_EQ(fixture.startup.received_name, "angles-0-22");
  EXPECT_EQ(fixture.startup.received_duration_ms, 1800);
}

TEST(StartupPoseGate, HoldsForTwoSecondsAfterSuccessThenAllowsNavigation)
{
  Fixture fixture;
  fixture.driver.handle_request(request());
  EXPECT_TRUE(fixture.backend.started_motion_names.empty());
  EXPECT_FALSE(fixture.core.has_active_request());
  ASSERT_EQ(fixture.statuses.size(), 1U);
  EXPECT_EQ(string_field(fixture.statuses[0], "status"), "REJECTED");
  EXPECT_EQ(
    string_field(fixture.statuses[0], "error_code"),
    "STARTUP_POSE_GATE_LOCKED");
  EXPECT_EQ(string_field(fixture.statuses[0], "action"), "STRAIGHT");
  EXPECT_EQ(int_field(fixture.statuses[0], "command_id"), 1);
  EXPECT_EQ(int_field(fixture.statuses[0], "event_id"), 2);
  EXPECT_EQ(int_field(fixture.statuses[0], "request_id"), 3);
  EXPECT_EQ(string_field(fixture.statuses[0], "motion_id"), "forward");
  fixture.startup.updates.push_back(
    {irc_step_motion_executor::StartupPoseState::SETTLING, "", ""});
  fixture.startup.updates.push_back(
    {irc_step_motion_executor::StartupPoseState::SUCCEEDED, "", ""});
  fixture.driver.poll();
  fixture.driver.poll();
  fixture.driver.poll();
  EXPECT_EQ(fixture.gate.state(), irc_step_motion_executor::StartupPoseGate::State::HOLDING);
  EXPECT_FALSE(fixture.gate.navigation_allowed());

  fixture.now_ms = 1999;
  fixture.driver.poll();
  EXPECT_FALSE(fixture.gate.navigation_allowed());

  fixture.now_ms = 2000;
  fixture.driver.poll();
  EXPECT_TRUE(fixture.gate.navigation_allowed());
  fixture.driver.handle_request(request());
  ASSERT_EQ(fixture.backend.started_motion_names.size(), 1U);
  EXPECT_TRUE(fixture.core.has_active_request());
  ASSERT_EQ(fixture.statuses.size(), 2U);
  EXPECT_EQ(string_field(fixture.statuses[1], "status"), "RUNNING");
}

TEST(StartupPoseGate, FailurePermanentlyKeepsNavigationBlocked)
{
  Fixture fixture;
  fixture.startup.start_result = false;
  fixture.driver.poll();
  fixture.driver.poll();
  fixture.driver.handle_request(request());
  EXPECT_EQ(fixture.startup.start_calls, 1);
  EXPECT_TRUE(fixture.backend.started_motion_names.empty());
  EXPECT_EQ(fixture.gate.state(), irc_step_motion_executor::StartupPoseGate::State::ERROR);
}

TEST(StartupPoseGate, DisabledGateNeverRunsAndAllowsSimulatedNavigation)
{
  FakeStartupPoseController startup;
  irc_step_motion_executor::StartupPoseGate gate{
    false, "오뒤307", {}, 1800, &startup};
  gate.poll();
  EXPECT_EQ(startup.start_calls, 0);
  EXPECT_TRUE(gate.navigation_allowed());
}

}  // namespace
