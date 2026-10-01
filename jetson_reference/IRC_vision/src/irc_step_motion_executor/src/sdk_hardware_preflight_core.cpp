#include "irc_step_motion_executor/sdk_hardware_preflight_core.hpp"

#include <charconv>
#include <cstdint>
#include <exception>
#include <set>
#include <string_view>
#include <utility>

namespace irc_step_motion_executor
{
namespace
{

bool parse_integer(std::string_view text, std::int64_t & value) noexcept
{
  if (text.empty()) {
    return false;
  }
  const char * const begin = text.data();
  const char * const end = begin + text.size();
  const auto result = std::from_chars(begin, end, value);
  return result.ec == std::errc{} && result.ptr == end;
}

bool parse_motor_ids(
  std::string_view text, std::vector<std::int64_t> & motor_ids,
  std::string & error)
{
  if (text.empty()) {
    error = "--motor-ids must not be empty";
    return false;
  }

  std::size_t item_begin = 0;
  std::set<std::int64_t> unique_motor_ids;
  while (item_begin <= text.size()) {
    const std::size_t separator = text.find(',', item_begin);
    const std::size_t item_end =
      separator == std::string_view::npos ? text.size() : separator;
    const auto item = text.substr(item_begin, item_end - item_begin);
    std::int64_t motor_id = 0;
    if (!parse_integer(item, motor_id)) {
      error = item.empty() ?
        "--motor-ids contains an empty item" :
        "--motor-ids contains a non-integer item: " + std::string(item);
      return false;
    }
    if (!unique_motor_ids.insert(motor_id).second) {
      error = "--motor-ids contains a duplicate ID: " +
        std::to_string(motor_id);
      return false;
    }
    motor_ids.push_back(motor_id);
    if (separator == std::string_view::npos) {
      break;
    }
    item_begin = separator + 1;
  }
  return true;
}

}  // namespace

std::string hardware_preflight_usage()
{
  return
    "Usage: sdk_hardware_preflight --device <path> --baud <integer> "
    "--motor-ids <comma-separated IDs> "
    "--confirm-hardware-access PREFLIGHT_ONLY_TORQUE_OFF\n"
    "The confirmation approves hardware port access only. It does not approve "
    "torque ON.\n"
    "This command never initializes or runs motion, and success does not mean "
    "motion-ready.";
}

HardwarePreflightArgumentsResult parse_hardware_preflight_arguments(
  const std::vector<std::string> & arguments) noexcept
{
  try {
    RobotMotionRuntimeConfig config;
    config.enable_robot_hardware = true;
    config.explicit_torque_approval = false;
    bool has_device = false;
    bool has_baud = false;
    bool has_motor_ids = false;
    bool has_hardware_access_confirmation = false;

    for (std::size_t index = 0; index < arguments.size(); ++index) {
      const std::string & option = arguments[index];
      if (option != "--device" && option != "--baud" &&
        option != "--motor-ids" &&
        option != "--confirm-hardware-access")
      {
        return {{}, "unknown option: " + option};
      }
      if (index + 1 >= arguments.size() ||
        arguments[index + 1].rfind("--", 0) == 0)
      {
        return {{}, "missing value for option: " + option};
      }
      const std::string & value = arguments[++index];

      if (option == "--device") {
        if (has_device) {
          return {{}, "duplicate option: --device"};
        }
        has_device = true;
        config.device_path = value;
      } else if (option == "--baud") {
        if (has_baud) {
          return {{}, "duplicate option: --baud"};
        }
        has_baud = true;
        if (!parse_integer(value, config.baud_rate)) {
          return {{}, "--baud must be a complete integer"};
        }
      } else if (option == "--motor-ids") {
        if (has_motor_ids) {
          return {{}, "duplicate option: --motor-ids"};
        }
        has_motor_ids = true;
        std::string motor_id_error;
        if (!parse_motor_ids(value, config.motor_ids, motor_id_error)) {
          return {{}, std::move(motor_id_error)};
        }
      } else {
        if (has_hardware_access_confirmation) {
          return {{}, "duplicate option: --confirm-hardware-access"};
        }
        has_hardware_access_confirmation = true;
        if (value != "PREFLIGHT_ONLY_TORQUE_OFF") {
          return {
            {},
            "--confirm-hardware-access must exactly equal "
            "PREFLIGHT_ONLY_TORQUE_OFF"};
        }
      }
    }

    if (!has_device) {
      return {{}, "required option is missing: --device"};
    }
    if (!has_baud) {
      return {{}, "required option is missing: --baud"};
    }
    if (!has_motor_ids) {
      return {{}, "required option is missing: --motor-ids"};
    }
    if (!has_hardware_access_confirmation) {
      return {
        {}, "required option is missing: --confirm-hardware-access"};
    }
    return {std::move(config), ""};
  } catch (const std::exception & exception) {
    return {{}, "failed to parse arguments: " + std::string(exception.what())};
  } catch (...) {
    return {{}, "failed to parse arguments: unknown error"};
  }
}

HardwarePreflightCommandResult run_hardware_preflight(
  RobotMotionPreflightFactory & factory,
  const RobotMotionRuntimeConfig & config) noexcept
{
  try {
    const auto result = factory.preflight(config);
    if (!result) {
      std::string error = result.error_code.empty() ?
        "ROBOT_MOTION_RUNTIME_PREFLIGHT_FAILED" : result.error_code;
      if (!result.message.empty()) {
        error += ": " + result.message;
      }
      return {1, "", std::move(error)};
    }
    return {
      0,
      "Hardware preflight succeeded. Torque remains OFF; hardware is not "
      "motion-ready.\n",
      ""};
  } catch (const std::exception & exception) {
    return {
      1, "", "ROBOT_MOTION_PREFLIGHT_COMMAND_EXCEPTION: " +
      std::string(exception.what())};
  } catch (...) {
    return {
      1, "", "ROBOT_MOTION_PREFLIGHT_COMMAND_EXCEPTION: unknown exception"};
  }
}

}  // namespace irc_step_motion_executor
