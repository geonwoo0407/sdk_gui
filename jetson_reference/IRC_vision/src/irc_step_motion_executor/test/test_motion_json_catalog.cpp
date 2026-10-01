#include <gtest/gtest.h>

#include <json-c/json.h>

#include <cmath>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <map>
#include <set>
#include <stdexcept>
#include <string>

namespace
{

struct CatalogSummary
{
  std::map<std::string, std::size_t> frame_counts;
};

bool required_object(
  json_object * parent, const char * key, json_object *& output,
  std::string & error)
{
  if (!json_object_object_get_ex(parent, key, &output) ||
    json_object_get_type(output) != json_type_object)
  {
    error = std::string("missing or invalid object field: ") + key;
    return false;
  }
  return true;
}

bool required_array(
  json_object * parent, const char * key, json_object *& output,
  std::string & error)
{
  if (!json_object_object_get_ex(parent, key, &output) ||
    json_object_get_type(output) != json_type_array)
  {
    error = std::string("missing or invalid array field: ") + key;
    return false;
  }
  return true;
}

bool required_string(
  json_object * parent, const char * key, std::string & output,
  std::string & error)
{
  json_object * value = nullptr;
  if (!json_object_object_get_ex(parent, key, &value) ||
    json_object_get_type(value) != json_type_string)
  {
    error = std::string("missing or invalid string field: ") + key;
    return false;
  }
  output = json_object_get_string(value);
  if (output.empty()) {
    error = std::string("empty string field: ") + key;
    return false;
  }
  return true;
}

bool required_finite_number(
  json_object * parent, const char * key, double & output,
  std::string & error)
{
  json_object * value = nullptr;
  if (!json_object_object_get_ex(parent, key, &value) ||
    (json_object_get_type(value) != json_type_int &&
    json_object_get_type(value) != json_type_double))
  {
    error = std::string("missing or invalid numeric field: ") + key;
    return false;
  }
  output = json_object_get_double(value);
  if (!std::isfinite(output)) {
    error = std::string("non-finite numeric field: ") + key;
    return false;
  }
  return true;
}

bool validate_motor_map(
  json_object * frame, const char * key, bool boolean_values,
  std::string & error)
{
  json_object * values = nullptr;
  if (!required_object(frame, key, values, error)) {
    return false;
  }
  std::set<int> motor_ids;
  json_object_object_foreach(values, id_text, value) {
    try {
      std::size_t parsed = 0;
      const int motor_id = std::stoi(id_text, &parsed);
      if (parsed != std::string(id_text).size()) {
        throw std::invalid_argument("trailing characters");
      }
      motor_ids.insert(motor_id);
    } catch (const std::exception &) {
      error = std::string(key) + " contains a non-integer motor ID";
      return false;
    }
    const json_type expected_type =
      boolean_values ? json_type_boolean : json_type_double;
    const json_type actual_type = json_object_get_type(value);
    if ((boolean_values && actual_type != expected_type) ||
      (!boolean_values && actual_type != json_type_int &&
      actual_type != json_type_double))
    {
      error = std::string(key) + " contains an invalid value";
      return false;
    }
    if (!boolean_values && !std::isfinite(json_object_get_double(value))) {
      error = std::string(key) + " contains a non-finite angle";
      return false;
    }
  }
  std::set<int> expected_ids;
  for (int motor_id = 0; motor_id <= 22; ++motor_id) {
    expected_ids.insert(motor_id);
  }
  if (motor_ids != expected_ids) {
    error = std::string(key) + " must contain exactly motor IDs 0-22";
    return false;
  }
  return true;
}

bool parse_catalog(
  const std::string & source, CatalogSummary & summary,
  std::string & error)
{
  summary.frame_counts.clear();
  json_tokener * tokener = json_tokener_new();
  json_object * root =
    json_tokener_parse_ex(tokener, source.c_str(), source.size());
  const json_tokener_error parse_error = json_tokener_get_error(tokener);
  std::size_t parsed_end = json_tokener_get_parse_end(tokener);
  json_tokener_free(tokener);
  while (parsed_end < source.size() &&
    std::isspace(static_cast<unsigned char>(source[parsed_end])))
  {
    ++parsed_end;
  }
  if (parse_error != json_tokener_success || root == nullptr ||
    parsed_end != source.size())
  {
    if (root != nullptr) {
      json_object_put(root);
    }
    error = "invalid motion catalog JSON";
    return false;
  }
  if (json_object_get_type(root) != json_type_object) {
    json_object_put(root);
    error = "motion catalog root must be an object";
    return false;
  }

  json_object * motions = nullptr;
  if (!required_array(root, "motions", motions, error)) {
    json_object_put(root);
    return false;
  }
  const std::size_t motion_count = json_object_array_length(motions);
  for (std::size_t motion_index = 0; motion_index < motion_count; ++motion_index) {
    json_object * motion = json_object_array_get_idx(motions, motion_index);
    if (motion == nullptr || json_object_get_type(motion) != json_type_object) {
      error = "motion entry must be an object";
      json_object_put(root);
      return false;
    }
    std::string name;
    double max_seq_ms = 0.0;
    double repeat_count = 0.0;
    double playback_speed = 0.0;
    json_object * frames = nullptr;
    if (!required_string(motion, "name", name, error) ||
      !required_finite_number(motion, "max_seq_ms", max_seq_ms, error) ||
      !required_finite_number(motion, "repeat_count", repeat_count, error) ||
      !required_finite_number(motion, "playback_speed", playback_speed, error) ||
      !required_array(motion, "frames", frames, error))
    {
      json_object_put(root);
      return false;
    }
    if (max_seq_ms < 0.0 || repeat_count < 1.0 || playback_speed <= 0.0) {
      error = "invalid motion timing metadata: " + name;
      json_object_put(root);
      return false;
    }
    if (!summary.frame_counts.emplace(
        name, json_object_array_length(frames)).second)
    {
      error = "duplicate motion name: " + name;
      json_object_put(root);
      return false;
    }

    std::int64_t previous_end = 0;
    const std::size_t frame_count = json_object_array_length(frames);
    if (frame_count == 0) {
      error = "motion has no frames: " + name;
      json_object_put(root);
      return false;
    }
    for (std::size_t frame_index = 0; frame_index < frame_count; ++frame_index) {
      json_object * frame = json_object_array_get_idx(frames, frame_index);
      std::string frame_name;
      double start_ms = 0.0;
      double time_ms = 0.0;
      if (frame == nullptr || json_object_get_type(frame) != json_type_object ||
        !required_string(frame, "name", frame_name, error) ||
        !required_finite_number(frame, "start_ms", start_ms, error) ||
        !required_finite_number(frame, "time_ms", time_ms, error) ||
        !validate_motor_map(frame, "angles", false, error) ||
        !validate_motor_map(frame, "torques", true, error))
      {
        json_object_put(root);
        return false;
      }
      if (start_ms < 0.0 || time_ms <= 0.0 || start_ms < previous_end) {
        error = "invalid or overlapping frame timing: " + frame_name;
        json_object_put(root);
        return false;
      }
      previous_end = static_cast<std::int64_t>(start_ms + time_ms);
    }
    if (max_seq_ms < previous_end) {
      error = "max_seq_ms is shorter than frames: " + name;
      json_object_put(root);
      return false;
    }
  }
  json_object_put(root);
  return true;
}

std::string read_file(const std::string & path)
{
  std::ifstream input(path, std::ios::binary);
  return std::string(
    std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

TEST(MotionJsonCatalog, CandidateCatalogMatchesExecutorLoaderContract)
{
  const std::string source = read_file(TEST_MOTION_CANDIDATE_CATALOG);
  ASSERT_FALSE(source.empty());
  CatalogSummary summary;
  std::string error;
  ASSERT_TRUE(parse_catalog(source, summary, error)) << error;

  const std::map<std::string, std::size_t> expected = {
    {"기본", 1},
    {"첫발", 3},
    {"전진", 5},
    {"전신 최신1", 4},
    {"전진 최신2", 4},
    {"전진 가장 좋음", 4},
    {"공잡기", 6},
    {"공잡기 리그랩까지", 10},
    {"좌회전1", 6},
    {"우회전", 4},
  };
  EXPECT_EQ(summary.frame_counts, expected);
  EXPECT_EQ(summary.frame_counts.size(), 10U);
}

TEST(MotionJsonCatalog, RejectsInvalidJsonWithClearError)
{
  CatalogSummary summary;
  std::string error;
  EXPECT_FALSE(parse_catalog("{not-json", summary, error));
  EXPECT_EQ(error, "invalid motion catalog JSON");
}

TEST(MotionJsonCatalog, RejectsMissingRequiredFramesField)
{
  CatalogSummary summary;
  std::string error;
  const std::string source =
    R"({"version":1,"motions":[{"name":"broken","max_seq_ms":1,)"
    R"("repeat_count":1,"playback_speed":1.0}]})";
  EXPECT_FALSE(parse_catalog(source, summary, error));
  EXPECT_EQ(error, "missing or invalid array field: frames");
}

}  // namespace
