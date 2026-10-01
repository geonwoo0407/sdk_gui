#include "irc_step_motion_executor/robot_motion_runtime_config.hpp"

#include <algorithm>
#include <array>
#include <filesystem>
#include <set>
#include <string_view>
#include <utility>

namespace irc_step_motion_executor
{
namespace
{

constexpr std::int64_t kMinimumSdkMotorId = 0;
constexpr std::int64_t kMaximumSdkMotorId = 22;

struct FixedSdkHardwareProfile
{
  std::string_view device_path;
  std::int64_t baud_rate;
  std::array<std::int64_t, 23> motor_ids;
};

constexpr FixedSdkHardwareProfile kFixedSdkHardwareProfile{
  "/dev/ttyUSB0",
  4000000,
  {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
    12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22}};

constexpr std::string_view kAlternateSdkDevicePath = "/dev/ttyUSB1";

RobotMotionRuntimeConfigResult error(
  std::string error_code, std::string message)
{
  return {{}, std::move(error_code), std::move(message)};
}

RobotMotionRuntimeConfigResult validate_robot_hardware_policy(
  const RobotMotionRuntimeConfig & config, bool require_motion_runtime,
  bool require_torque_approval)
{
  if (!config.enable_robot_hardware) {
    return error(
      "ROBOT_HARDWARE_NOT_ENABLED",
      "hardware initialization requires enable_robot_hardware=true");
  }

  if (require_motion_runtime) {
    const auto runtime_config_result =
      validate_robot_motion_runtime_config(config);
    if (!runtime_config_result) {
      return runtime_config_result;
    }
  }

  if (config.device_path.empty()) {
    return error(
      "ROBOT_DEVICE_PATH_REQUIRED",
      "hardware initialization requires a non-empty device_path");
  }
  if (config.baud_rate <= 0) {
    return error(
      "ROBOT_BAUD_RATE_INVALID",
      "hardware initialization requires baud_rate greater than zero");
  }
  if (config.motor_ids.empty()) {
    return error(
      "ROBOT_MOTOR_IDS_REQUIRED",
      "hardware initialization requires at least one motor ID");
  }

  std::set<std::int64_t> unique_motor_ids;
  for (const std::int64_t motor_id : config.motor_ids) {
    if (motor_id < kMinimumSdkMotorId || motor_id > kMaximumSdkMotorId) {
      return error(
        "ROBOT_MOTOR_ID_OUT_OF_RANGE",
        "motor IDs must match the current SDK range 0..22");
    }
    if (!unique_motor_ids.insert(motor_id).second) {
      return error(
        "ROBOT_MOTOR_ID_DUPLICATED",
        "hardware initialization motor IDs must not contain duplicates");
    }
  }

  if (require_torque_approval && !config.explicit_torque_approval) {
    return error(
      "ROBOT_TORQUE_APPROVAL_REQUIRED",
      "hardware initialization requires explicit torque approval");
  }

  if (config.device_path != kFixedSdkHardwareProfile.device_path &&
    config.device_path != kAlternateSdkDevicePath)
  {
    return error(
      "ROBOT_DEVICE_PATH_MISMATCH",
      "device_path does not match an approved SDK hardware device path");
  }
  if (config.baud_rate != kFixedSdkHardwareProfile.baud_rate) {
    return error(
      "ROBOT_BAUD_RATE_MISMATCH",
      "baud_rate does not match the current fixed SDK hardware profile");
  }
  if (!std::equal(
      unique_motor_ids.begin(), unique_motor_ids.end(),
      kFixedSdkHardwareProfile.motor_ids.begin(),
      kFixedSdkHardwareProfile.motor_ids.end()))
  {
    return error(
      "ROBOT_MOTOR_IDS_MISMATCH",
      "motor IDs must exactly match the current fixed SDK set 0..22");
  }

  return {config, "", ""};
}

}  // namespace

RobotMotionRuntimeConfig make_robot_motion_runtime_config(
  std::string motion_json_path,
  bool enable_robot_hardware,
  std::string device_path,
  std::int64_t baud_rate,
  std::vector<std::int64_t> motor_ids,
  bool explicit_torque_approval)
{
  RobotMotionRuntimeConfig config;
  config.motion_json_path = std::move(motion_json_path);
  config.enable_robot_hardware = enable_robot_hardware;
  config.device_path = std::move(device_path);
  config.baud_rate = baud_rate;
  config.motor_ids = std::move(motor_ids);
  config.explicit_torque_approval = explicit_torque_approval;
  return config;
}

RobotMotionRuntimeConfigResult parse_robot_motion_runtime_config(
  const std::map<std::string, std::string> & settings)
{
  for (const auto & [name, unused] : settings) {
    static_cast<void>(unused);
    if (name != "motion_json_path") {
      return error(
        "UNKNOWN_ROBOT_MOTION_RUNTIME_SETTING",
        "unknown RobotMotionPlayer runtime setting '" + name +
        "'; allowed setting is: motion_json_path");
    }
  }

  RobotMotionRuntimeConfig config;
  const auto motion_json = settings.find("motion_json_path");
  if (motion_json != settings.end()) {
    config.motion_json_path = motion_json->second;
  }
  return validate_robot_motion_runtime_config(config);
}

RobotMotionRuntimeConfigResult validate_robot_motion_runtime_config(
  const RobotMotionRuntimeConfig & config)
{
  if (config.motion_json_path.empty()) {
    return error(
      "MOTION_JSON_PATH_REQUIRED",
      "motion_json_path must be explicitly configured");
  }

  const std::filesystem::path path(config.motion_json_path);
  std::error_code filesystem_error;
  if (!std::filesystem::exists(path, filesystem_error) || filesystem_error) {
    return error(
      "MOTION_JSON_FILE_NOT_FOUND",
      "motion JSON file does not exist: " + config.motion_json_path);
  }
  if (!std::filesystem::is_regular_file(path, filesystem_error) ||
    filesystem_error)
  {
    return error(
      "MOTION_JSON_PATH_NOT_FILE",
      "motion_json_path is not a regular file: " + config.motion_json_path);
  }

  return {config, "", ""};
}

RobotMotionRuntimeConfigResult validate_robot_hardware_initialization_policy(
  const RobotMotionRuntimeConfig & config)
{
  return validate_robot_hardware_policy(config, true, true);
}

RobotMotionRuntimeConfigResult validate_robot_hardware_preflight_policy(
  const RobotMotionRuntimeConfig & config)
{
  // Preflight diagnoses hardware only; it does not construct a motion runtime.
  return validate_robot_hardware_policy(config, false, false);
}

}  // namespace irc_step_motion_executor
