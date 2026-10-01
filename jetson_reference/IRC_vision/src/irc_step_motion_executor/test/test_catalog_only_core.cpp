#include "irc_step_motion_executor/catalog_only_core.hpp"

#include <gtest/gtest.h>
#include <json-c/json.h>

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#ifndef TEST_ALIAS_CONFIG
#define TEST_ALIAS_CONFIG ""
#endif

#ifndef TEST_CONTRACT_VECTORS
#define TEST_CONTRACT_VECTORS ""
#endif

namespace
{

irc_step_motion_executor::CatalogOnlyCore make_core()
{
  irc_step_motion_executor::MotionAliasCatalog catalog;
  std::string error;
  EXPECT_TRUE(catalog.load(TEST_ALIAS_CONFIG, error)) << error;
  return irc_step_motion_executor::CatalogOnlyCore(std::move(catalog));
}

json_object * load_contract_vectors()
{
  json_object * vectors = json_object_from_file(TEST_CONTRACT_VECTORS);
  EXPECT_NE(vectors, nullptr);
  if (vectors != nullptr) {
    EXPECT_EQ(json_object_get_type(vectors), json_type_array);
  }
  return vectors;
}

TEST(MotionAliasCatalog, LoadsLatestSdkAndCanonicalAliases)
{
  irc_step_motion_executor::MotionAliasCatalog catalog;
  std::string error;
  ASSERT_TRUE(catalog.load(TEST_ALIAS_CONFIG, error)) << error;
  EXPECT_EQ(catalog.size(), 107U);
  EXPECT_EQ(
    catalog.resolve("forward"),
    std::optional<std::string>("전진45도(8회)"));
  EXPECT_EQ(
    catalog.resolve("line_turn_left_15"),
    std::optional<std::string>("좌회전실실전(15회)"));
  EXPECT_EQ(
    catalog.resolve("line_turn_right_large"),
    std::optional<std::string>("우회전실실전(15회)"));
  EXPECT_EQ(
    catalog.resolve("line_recovery_left_4"),
    std::optional<std::string>("라인복귀좌회전(4회)"));
  EXPECT_EQ(
    catalog.resolve("line_recovery_left_8"),
    std::optional<std::string>("라인보귀좌회전(8번)"));
  EXPECT_EQ(
    catalog.resolve("line_recovery_right_4"),
    std::optional<std::string>("라인복귀우회전(4회)"));
  EXPECT_EQ(
    catalog.resolve("line_recovery_right_8"),
    std::optional<std::string>("라인보귀우회전(8번)"));
  EXPECT_EQ(
    catalog.resolve("stationary_turn_left"),
    std::optional<std::string>("제자리좌회전(6번)"));
  EXPECT_EQ(
    catalog.resolve("stationary_turn_right"),
    std::optional<std::string>("제자리우회전(9회)"));
  EXPECT_EQ(
    catalog.resolve("pickup_camera_down_turn_left_6"),
    std::optional<std::string>("제좌카메라내린거(6회)"));
  EXPECT_EQ(
    catalog.resolve("pickup_camera_down_turn_right_9"),
    std::optional<std::string>("제우카메라내린거(9회)"));
  EXPECT_EQ(
    catalog.resolve("pickup"),
    std::optional<std::string>("공잡기리그랩까지 실전"));
  EXPECT_FALSE(catalog.resolve("forward_short").has_value());
  EXPECT_FALSE(catalog.resolve("turn_left").has_value());
  EXPECT_FALSE(catalog.resolve("fine_left").has_value());
  EXPECT_FALSE(catalog.resolve("align_left").has_value());
  EXPECT_FALSE(catalog.resolve("shoot").has_value());
}

TEST(CatalogOnlyCore, PreservesCorrelationFieldsAndRejectsExecution)
{
  const auto core = make_core();
  const auto status = core.handle_request(
    R"({"action":"STRAIGHT","command_id":17,"event_id":29,)"
    R"("request_id":41,"motion_id":"forward","timeout_ms":5000})");

  EXPECT_EQ(status.status, "REJECTED");
  EXPECT_EQ(status.action, std::optional<std::string>("STRAIGHT"));
  EXPECT_EQ(status.command_id, std::optional<std::int64_t>(17));
  EXPECT_EQ(status.event_id, std::optional<std::int64_t>(29));
  EXPECT_EQ(status.request_id, 41);
  EXPECT_EQ(status.motion_id, "forward");
  EXPECT_EQ(status.error_code, "HARDWARE_NOT_READY");
  EXPECT_NE(status.message.find("catalog-only mode"), std::string::npos);
}

TEST(CatalogOnlyCore, RejectsInvalidJsonSafely)
{
  const auto status = make_core().handle_request("{not-json");
  EXPECT_EQ(status.status, "REJECTED");
  EXPECT_EQ(status.error_code, "INVALID_REQUEST");
  EXPECT_FALSE(status.action.has_value());
  EXPECT_FALSE(status.command_id.has_value());
  EXPECT_FALSE(status.event_id.has_value());
  EXPECT_EQ(status.request_id, 0);
  EXPECT_TRUE(status.motion_id.empty());
}

TEST(CatalogOnlyCore, DistinguishesRequiredNullEventIdFromMissingEventId)
{
  const auto core = make_core();
  const auto null_status = core.handle_request(
    R"({"action":"STRAIGHT","command_id":17,"event_id":null,)"
    R"("request_id":41,"motion_id":"forward"})");
  EXPECT_EQ(null_status.error_code, "HARDWARE_NOT_READY");
  EXPECT_FALSE(null_status.event_id.has_value());

  const auto missing_status = core.handle_request(
    R"({"action":"STRAIGHT","command_id":17,)"
    R"("request_id":41,"motion_id":"forward"})");
  EXPECT_EQ(missing_status.status, "REJECTED");
  EXPECT_EQ(missing_status.error_code, "INVALID_REQUEST");
}

TEST(CatalogOnlyCore, RequiresEveryRequestContractField)
{
  const auto core = make_core();
  for (const std::string payload : {
      R"({"command_id":1,"event_id":2,"request_id":3,"motion_id":"forward"})",
      R"({"action":"STRAIGHT","event_id":2,"request_id":3,"motion_id":"forward"})",
      R"({"action":"STRAIGHT","command_id":1,"request_id":3,"motion_id":"forward"})",
      R"({"action":"STRAIGHT","command_id":1,"event_id":2,"motion_id":"forward"})",
      R"({"action":"STRAIGHT","command_id":1,"event_id":2,"request_id":3})"})
  {
    EXPECT_EQ(core.handle_request(payload).error_code, "INVALID_REQUEST");
  }
}

TEST(CatalogOnlyCore, RejectsUnknownAliasWithoutFallback)
{
  const auto status = make_core().handle_request(
    R"({"action":"TURN_LEFT","command_id":1,"event_id":2,)"
    R"("request_id":3,"motion_id":"turn_left"})");
  EXPECT_EQ(status.status, "REJECTED");
  EXPECT_EQ(status.motion_id, "turn_left");
  EXPECT_EQ(status.error_code, "INVALID_MOTION");
  EXPECT_NE(status.message.find("no fallback"), std::string::npos);
}

TEST(CatalogOnlyCore, SerializedStatusContainsContractFields)
{
  const auto status = make_core().handle_request(
    R"({"action":"STEP","command_id":4,"event_id":5,)"
    R"("request_id":6,"motion_id":"pickup"})");
  const std::string payload =
    irc_step_motion_executor::CatalogOnlyCore::to_json(status);
  json_object * object = json_tokener_parse(payload.c_str());
  ASSERT_NE(object, nullptr);
  for (const char * field : {
      "status", "action", "command_id", "event_id", "request_id",
      "motion_id", "error_code", "message"})
  {
    json_object * value = nullptr;
    EXPECT_TRUE(json_object_object_get_ex(object, field, &value)) << field;
  }
  json_object * value = nullptr;
  ASSERT_TRUE(json_object_object_get_ex(object, "status", &value));
  EXPECT_EQ(json_object_get_type(value), json_type_string);
  ASSERT_TRUE(json_object_object_get_ex(object, "action", &value));
  EXPECT_EQ(json_object_get_type(value), json_type_string);
  ASSERT_TRUE(json_object_object_get_ex(object, "command_id", &value));
  EXPECT_EQ(json_object_get_type(value), json_type_int);
  ASSERT_TRUE(json_object_object_get_ex(object, "event_id", &value));
  EXPECT_EQ(json_object_get_type(value), json_type_int);
  ASSERT_TRUE(json_object_object_get_ex(object, "request_id", &value));
  EXPECT_EQ(json_object_get_type(value), json_type_int);
  ASSERT_TRUE(json_object_object_get_ex(object, "motion_id", &value));
  EXPECT_EQ(json_object_get_type(value), json_type_string);
  ASSERT_TRUE(json_object_object_get_ex(object, "error_code", &value));
  EXPECT_EQ(json_object_get_type(value), json_type_string);
  ASSERT_TRUE(json_object_object_get_ex(object, "message", &value));
  EXPECT_EQ(json_object_get_type(value), json_type_string);
  json_object_put(object);
}

TEST(CatalogOnlyCore, SerializedStatusPreservesNullCorrelationTypes)
{
  const auto status = make_core().handle_request(
    R"({"action":"PICKUP_NOW","command_id":null,"event_id":null,)"
    R"("request_id":7,"motion_id":"pickup"})");
  const std::string payload =
    irc_step_motion_executor::CatalogOnlyCore::to_json(status);
  json_object * object = json_tokener_parse(payload.c_str());
  ASSERT_NE(object, nullptr);
  json_object * value = nullptr;
  ASSERT_TRUE(json_object_object_get_ex(object, "command_id", &value));
  EXPECT_EQ(json_object_get_type(value), json_type_null);
  ASSERT_TRUE(json_object_object_get_ex(object, "event_id", &value));
  EXPECT_EQ(json_object_get_type(value), json_type_null);
  json_object_put(object);
}

TEST(CatalogOnlyContract, SharedVectorsPreserveFieldsWithoutFallback)
{
  json_object * vectors = load_contract_vectors();
  ASSERT_NE(vectors, nullptr);
  const auto core = make_core();

  const std::size_t count = json_object_array_length(vectors);
  for (std::size_t index = 0; index < count; ++index) {
    json_object * vector = json_object_array_get_idx(vectors, index);
    json_object * request = nullptr;
    json_object * expected_error = nullptr;
    ASSERT_TRUE(json_object_object_get_ex(vector, "request", &request));
    ASSERT_TRUE(
      json_object_object_get_ex(
        vector, "expected_error_code", &expected_error));

    const auto status = core.handle_request(
      json_object_to_json_string_ext(request, JSON_C_TO_STRING_PLAIN));
    EXPECT_EQ(status.status, "REJECTED");
    EXPECT_EQ(status.error_code, json_object_get_string(expected_error));

    json_object * value = nullptr;
    ASSERT_TRUE(json_object_object_get_ex(request, "action", &value));
    ASSERT_TRUE(status.action.has_value());
    EXPECT_EQ(*status.action, json_object_get_string(value));
    ASSERT_TRUE(json_object_object_get_ex(request, "request_id", &value));
    EXPECT_EQ(status.request_id, json_object_get_int64(value));
    ASSERT_TRUE(json_object_object_get_ex(request, "motion_id", &value));
    EXPECT_EQ(status.motion_id, json_object_get_string(value));

    ASSERT_TRUE(json_object_object_get_ex(request, "command_id", &value));
    if (json_object_get_type(value) == json_type_null) {
      EXPECT_FALSE(status.command_id.has_value());
    } else {
      ASSERT_TRUE(status.command_id.has_value());
      EXPECT_EQ(*status.command_id, json_object_get_int64(value));
    }
    ASSERT_TRUE(json_object_object_get_ex(request, "event_id", &value));
    if (json_object_get_type(value) == json_type_null) {
      EXPECT_FALSE(status.event_id.has_value());
    } else {
      ASSERT_TRUE(status.event_id.has_value());
      EXPECT_EQ(*status.event_id, json_object_get_int64(value));
    }
  }
  json_object_put(vectors);
}

TEST(CatalogOnlyCore, CancelNeverTouchesHardware)
{
  const auto status = make_core().handle_cancel(R"({"request_id":88})");
  EXPECT_EQ(status.status, "REJECTED");
  EXPECT_EQ(status.request_id, 88);
  EXPECT_EQ(status.error_code, "NOT_RUNNING");
  EXPECT_NE(status.message.find("catalog-only mode"), std::string::npos);
}

}  // namespace
