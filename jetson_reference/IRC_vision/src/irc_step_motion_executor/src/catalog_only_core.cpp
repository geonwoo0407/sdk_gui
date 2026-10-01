#include "irc_step_motion_executor/catalog_only_core.hpp"

#include <json-c/json.h>
#include <yaml-cpp/yaml.h>

#include <utility>

namespace irc_step_motion_executor
{
namespace
{

void add_optional_int(
  json_object * object, const char * key,
  const std::optional<std::int64_t> & value)
{
  json_object_object_add(
    object, key, value ? json_object_new_int64(*value) : nullptr);
}

void add_optional_string(
  json_object * object, const char * key,
  const std::optional<std::string> & value)
{
  json_object_object_add(
    object, key, value ? json_object_new_string(value->c_str()) : nullptr);
}

bool get_required_int(
  json_object * object, const char * key, std::int64_t & output)
{
  json_object * value = nullptr;
  if (!json_object_object_get_ex(object, key, &value) ||
    json_object_get_type(value) != json_type_int)
  {
    return false;
  }
  output = json_object_get_int64(value);
  return true;
}

bool get_required_string(
  json_object * object, const char * key, std::string & output)
{
  json_object * value = nullptr;
  if (!json_object_object_get_ex(object, key, &value) ||
    json_object_get_type(value) != json_type_string)
  {
    return false;
  }
  output = json_object_get_string(value);
  return !output.empty();
}

bool get_required_nullable_int(
  json_object * object, const char * key,
  std::optional<std::int64_t> & output)
{
  json_object * value = nullptr;
  if (!json_object_object_get_ex(object, key, &value)) {
    return false;
  }
  if (json_object_get_type(value) == json_type_null) {
    output.reset();
    return true;
  }
  if (json_object_get_type(value) != json_type_int) {
    return false;
  }
  output = json_object_get_int64(value);
  return true;
}

bool get_required_string(
  json_object * object, const char * key,
  std::optional<std::string> & output)
{
  json_object * value = nullptr;
  if (!json_object_object_get_ex(object, key, &value) ||
    json_object_get_type(value) != json_type_string)
  {
    return false;
  }
  const std::string parsed = json_object_get_string(value);
  if (parsed.empty()) {
    return false;
  }
  output = parsed;
  return true;
}

MotionStatus invalid_request(const std::string & message)
{
  MotionStatus status;
  status.error_code = "INVALID_REQUEST";
  status.message = message;
  return status;
}

}  // namespace

bool MotionAliasCatalog::load(
  const std::string & path, std::string & error_message)
{
  aliases_.clear();
  try {
    const YAML::Node root = YAML::LoadFile(path);
    const YAML::Node aliases = root["motion_aliases"];
    if (!aliases || !aliases.IsMap()) {
      error_message = "motion_aliases must be a YAML mapping";
      return false;
    }
    for (const auto & item : aliases) {
      const std::string motion_id = item.first.as<std::string>();
      const std::string catalog_name = item.second.as<std::string>();
      if (motion_id.empty() || catalog_name.empty()) {
        error_message = "motion alias keys and values must be non-empty strings";
        aliases_.clear();
        return false;
      }
      aliases_.emplace(motion_id, catalog_name);
    }
  } catch (const YAML::Exception & exception) {
    error_message = exception.what();
    aliases_.clear();
    return false;
  }
  return true;
}

bool MotionAliasCatalog::contains(const std::string & motion_id) const
{
  return aliases_.find(motion_id) != aliases_.end();
}

std::optional<std::string> MotionAliasCatalog::resolve(
  const std::string & motion_id) const
{
  const auto alias = aliases_.find(motion_id);
  if (alias == aliases_.end()) {
    return std::nullopt;
  }
  return alias->second;
}

std::size_t MotionAliasCatalog::size() const
{
  return aliases_.size();
}

CatalogOnlyCore::CatalogOnlyCore(MotionAliasCatalog catalog)
: catalog_(std::move(catalog))
{
}

MotionStatus CatalogOnlyCore::handle_request(const std::string & payload) const
{
  json_tokener * tokener = json_tokener_new();
  json_object * object =
    json_tokener_parse_ex(tokener, payload.c_str(), payload.size());
  const json_tokener_error parse_error = json_tokener_get_error(tokener);
  json_tokener_free(tokener);

  if (parse_error != json_tokener_success || object == nullptr) {
    if (object != nullptr) {
      json_object_put(object);
    }
    return invalid_request("invalid JSON request");
  }
  if (json_object_get_type(object) != json_type_object) {
    json_object_put(object);
    return invalid_request("request must be a JSON object");
  }

  MotionRequest request;
  const bool valid =
    get_required_int(object, "request_id", request.request_id) &&
    get_required_string(object, "motion_id", request.motion_id) &&
    get_required_nullable_int(object, "command_id", request.command_id) &&
    get_required_nullable_int(object, "event_id", request.event_id) &&
    get_required_string(object, "action", request.action);
  json_object_put(object);
  if (!valid) {
    return invalid_request(
      "request fields have missing or invalid correlation values");
  }

  MotionStatus status;
  status.action = request.action;
  status.command_id = request.command_id;
  status.event_id = request.event_id;
  status.request_id = request.request_id;
  status.motion_id = request.motion_id;
  if (!catalog_.contains(request.motion_id)) {
    status.error_code = "INVALID_MOTION";
    status.message =
      "unsupported motion_id in catalog-only mode; no fallback was applied";
    return status;
  }

  status.error_code = "HARDWARE_NOT_READY";
  status.message =
    "catalog-only mode: motion alias validated, execution is disabled";
  return status;
}

MotionStatus CatalogOnlyCore::handle_cancel(const std::string & payload) const
{
  json_tokener * tokener = json_tokener_new();
  json_object * object =
    json_tokener_parse_ex(tokener, payload.c_str(), payload.size());
  const json_tokener_error parse_error = json_tokener_get_error(tokener);
  json_tokener_free(tokener);

  if (parse_error != json_tokener_success || object == nullptr ||
    json_object_get_type(object) != json_type_object)
  {
    if (object != nullptr) {
      json_object_put(object);
    }
    return invalid_request("invalid cancel JSON");
  }

  MotionStatus status;
  if (!get_required_int(object, "request_id", status.request_id)) {
    json_object_put(object);
    return invalid_request("cancel request_id must be an integer");
  }
  json_object_put(object);
  status.error_code = "NOT_RUNNING";
  status.message = "catalog-only mode: no hardware motion can be running";
  return status;
}

std::string CatalogOnlyCore::to_json(const MotionStatus & status)
{
  json_object * object = json_object_new_object();
  json_object_object_add(
    object, "status", json_object_new_string(status.status.c_str()));
  add_optional_string(object, "action", status.action);
  add_optional_int(object, "command_id", status.command_id);
  add_optional_int(object, "event_id", status.event_id);
  json_object_object_add(
    object, "request_id", json_object_new_int64(status.request_id));
  json_object_object_add(
    object, "motion_id", json_object_new_string(status.motion_id.c_str()));
  json_object_object_add(
    object, "error_code", json_object_new_string(status.error_code.c_str()));
  json_object_object_add(
    object, "message", json_object_new_string(status.message.c_str()));
  const std::string serialized =
    json_object_to_json_string_ext(object, JSON_C_TO_STRING_PLAIN);
  json_object_put(object);
  return serialized;
}

}  // namespace irc_step_motion_executor
