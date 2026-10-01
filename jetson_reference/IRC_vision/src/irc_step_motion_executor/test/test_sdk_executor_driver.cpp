#include "fake_motion_backend.hpp"
#include "irc_step_motion_executor/sdk_executor_driver.hpp"

#include <gtest/gtest.h>
#include <json-c/json.h>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

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

bool field_is_null(const std::string & payload, const char * key)
{
  json_object * object = json_tokener_parse(payload.c_str());
  EXPECT_NE(object, nullptr);
  if (object == nullptr) {
    return false;
  }
  json_object * value = nullptr;
  EXPECT_TRUE(json_object_object_get_ex(object, key, &value));
  const bool result =
    value == nullptr || json_object_get_type(value) == json_type_null;
  json_object_put(object);
  return result;
}

struct DriverFixture
{
  FakeMotionBackend backend;
  irc_step_motion_executor::SdkExecutorCore core{
    load_catalog(), backend};
  std::uint64_t now_ms{100};
  std::vector<std::string> published;
  irc_step_motion_executor::SdkExecutorDriver driver{
    core,
    [this]() {return now_ms;},
    [this](const std::string & payload) {published.push_back(payload);}};
};

TEST(SdkExecutorDriver, RequestPublishesImmediateRunningStatus)
{
  DriverFixture fixture;
  fixture.driver.handle_request(request_json(1, "forward"));

  ASSERT_EQ(fixture.published.size(), 1U);
  EXPECT_EQ(string_field(fixture.published[0], "status"), "RUNNING");
  EXPECT_EQ(int_field(fixture.published[0], "request_id"), 1);
  ASSERT_EQ(fixture.backend.started_motion_names.size(), 1U);
  EXPECT_EQ(fixture.backend.started_motion_names[0], "전진45도(8회)");
}

TEST(SdkExecutorDriver, InvalidAndUnsupportedRequestsPublishRejections)
{
  DriverFixture fixture;
  fixture.driver.handle_request("{invalid");
  fixture.driver.handle_request(
    request_json(2, "turn_left", "TURN_LEFT"));

  ASSERT_EQ(fixture.published.size(), 2U);
  EXPECT_EQ(string_field(fixture.published[0], "status"), "REJECTED");
  EXPECT_EQ(
    string_field(fixture.published[0], "error_code"), "INVALID_REQUEST");
  EXPECT_EQ(
    string_field(fixture.published[1], "error_code"), "INVALID_MOTION");
  EXPECT_TRUE(fixture.backend.started_motion_names.empty());
}

TEST(SdkExecutorDriver, NullableCorrelationFieldsRemainNull)
{
  DriverFixture fixture;
  fixture.driver.handle_request(
    request_json(3, "pickup", "PICKUP_NOW", "null", "null"));

  ASSERT_EQ(fixture.published.size(), 1U);
  EXPECT_TRUE(field_is_null(fixture.published[0], "command_id"));
  EXPECT_TRUE(field_is_null(fixture.published[0], "event_id"));
  EXPECT_EQ(
    string_field(fixture.published[0], "action"), "PICKUP_NOW");
}

TEST(SdkExecutorDriver, CancelCallbackPublishesAndTerminalPollPublishes)
{
  DriverFixture fixture;
  fixture.backend.statuses.push_back(
    {irc_step_motion_executor::BackendState::CANCELLED, "", "cancelled"});
  fixture.driver.handle_request(request_json(4, "forward"));
  fixture.driver.handle_cancel(R"({"request_id":4})");
  fixture.driver.poll();

  ASSERT_EQ(fixture.published.size(), 3U);
  EXPECT_EQ(string_field(fixture.published[1], "status"), "RUNNING");
  EXPECT_EQ(string_field(fixture.published[2], "status"), "CANCELLED");
}

TEST(SdkExecutorDriver, StaleCancelDoesNotCancelActiveMotion)
{
  DriverFixture fixture;
  fixture.driver.handle_request(request_json(5, "forward"));
  fixture.driver.handle_cancel(R"({"request_id":99})");

  ASSERT_EQ(fixture.published.size(), 2U);
  EXPECT_EQ(
    string_field(fixture.published[1], "error_code"), "STALE_REQUEST");
  EXPECT_EQ(fixture.backend.cancel_calls, 0);
  EXPECT_TRUE(fixture.core.has_active_request());
}

TEST(SdkExecutorDriver, PollWithoutActiveRequestPublishesNothing)
{
  DriverFixture fixture;
  fixture.driver.poll();
  EXPECT_TRUE(fixture.published.empty());
  EXPECT_EQ(fixture.backend.poll_calls, 0);
}

TEST(SdkExecutorDriver, TerminalPollPublishesSucceededStatus)
{
  DriverFixture fixture;
  fixture.backend.statuses.push_back(
    {irc_step_motion_executor::BackendState::SUCCEEDED, "", "done"});
  fixture.driver.handle_request(request_json(6, "forward"));
  fixture.driver.poll();

  ASSERT_EQ(fixture.published.size(), 2U);
  EXPECT_EQ(string_field(fixture.published[1], "status"), "SUCCEEDED");
  EXPECT_FALSE(fixture.core.has_active_request());
}

TEST(SdkExecutorDriver, SuppressesIdenticalRunningStatusAtControlRate)
{
  DriverFixture fixture;
  fixture.backend.statuses = {
    {irc_step_motion_executor::BackendState::RUNNING, "", "moving"},
    {irc_step_motion_executor::BackendState::RUNNING, "", "moving"}};
  fixture.driver.handle_request(request_json(8, "forward"));
  fixture.driver.poll();
  fixture.driver.poll();

  ASSERT_EQ(fixture.published.size(), 2U);
  EXPECT_EQ(string_field(fixture.published[1], "status"), "RUNNING");
  EXPECT_EQ(string_field(fixture.published[1], "message"), "moving");
}

TEST(SdkExecutorDriver, BackendExceptionPublishesFailedWithoutEscaping)
{
  DriverFixture fixture;
  fixture.backend.throw_on_poll = true;
  fixture.driver.handle_request(request_json(7, "forward"));

  EXPECT_NO_THROW(fixture.driver.poll());
  ASSERT_EQ(fixture.published.size(), 2U);
  EXPECT_EQ(string_field(fixture.published[1], "status"), "FAILED");
  EXPECT_EQ(
    string_field(fixture.published[1], "error_code"),
    "BACKEND_EXCEPTION");
}

}  // namespace
