#include "irc_step_motion_executor/motion_backend_factory.hpp"
#include "irc_step_motion_executor/sdk_executor_core.hpp"
#include "irc_step_motion_executor/sdk_executor_driver.hpp"
#include "irc_step_motion_executor/startup_pose_gate.hpp"
#include "irc_step_motion_executor/startup_pose_catalog.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <json-c/json.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace irc_step_motion_executor
{
namespace
{

constexpr std::int64_t kDefaultPollPeriodMs = 20;
constexpr std::int64_t kDefaultRunningPolls = 2;
constexpr std::int64_t kDefaultSettlingPolls = 1;
constexpr std::int64_t kDefaultHeartbeatPeriodMs = 500;

std::uint64_t steady_now_ms()
{
  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

}  // namespace

class SdkMotionExecutorNode : public rclcpp::Node
{
public:
  SdkMotionExecutorNode()
  : Node("sdk_motion_executor")
  {
    const std::string default_alias_path =
      ament_index_cpp::get_package_share_directory(
      "irc_step_motion_executor") + "/config/motion_aliases.yaml";
    const std::string alias_path = declare_parameter<std::string>(
      "motion_aliases_file", default_alias_path);
    const std::string backend_type = declare_parameter<std::string>(
      "backend_type", "simulated");
    const bool enable_robot_hardware = declare_parameter<bool>(
      "enable_robot_hardware", false);
    const std::string motion_json_path = declare_parameter<std::string>(
      "motion_json_path", "");
    const std::string robot_device_path = declare_parameter<std::string>(
      "robot_device_path", "");
    const std::int64_t robot_baud_rate = declare_parameter<std::int64_t>(
      "robot_baud_rate", 0);
    const auto robot_motor_ids =
      declare_parameter<std::vector<std::int64_t>>(
      "robot_motor_ids", std::vector<std::int64_t>{});
    const bool explicit_torque_approval = declare_parameter<bool>(
      "explicit_torque_approval", false);
    const bool position_tolerance_enabled = declare_parameter<bool>(
      "position_tolerance_enabled", true);
    const std::int64_t poll_period_ms = positive_parameter_or_default(
      "poll_period_ms", kDefaultPollPeriodMs);
    const std::int64_t running_polls = nonnegative_parameter_or_default(
      "running_polls", kDefaultRunningPolls);
    const std::int64_t settling_polls = nonnegative_parameter_or_default(
      "settling_polls", kDefaultSettlingPolls);
    const std::int64_t heartbeat_period_ms = positive_parameter_or_default(
      "heartbeat_period_ms", kDefaultHeartbeatPeriodMs);
    const bool startup_pose_enabled = declare_parameter<bool>(
      "startup_pose_enabled", false);
    const std::string startup_pose_name = declare_parameter<std::string>(
      "startup_pose_name", "오뒤410");
    const std::int64_t startup_pose_duration_ms = positive_parameter_or_default(
      "startup_pose_duration_ms", 1800);
    ball_head_override_enabled_ = declare_parameter<bool>(
      "ball_head_override_enabled", true);
    ball_head_override_deg_ = declare_parameter<double>(
      "ball_head_override_deg", -60.0);
    ball_head_camera_up_deg_ = declare_parameter<double>(
      "ball_head_camera_up_deg", -33.0);
    ball_head_transition_ms_ = positive_parameter_or_default(
      "ball_head_transition_ms", 400);
    ball_head_override_motion_ids_ =
      declare_parameter<std::vector<std::string>>(
      "ball_head_override_motion_ids",
      std::vector<std::string>{
        "line_forward_2", "line_forward_4", "line_forward_6",
        "line_forward_8", "line_forward_10", "sdk_forward_4", "forward"});
    if (!std::isfinite(ball_head_override_deg_) ||
      !std::isfinite(ball_head_camera_up_deg_) ||
      ball_head_override_deg_ < -90.0 || ball_head_override_deg_ > 30.0)
    {
      throw std::runtime_error(
              "ball head angles must be finite and the override must be "
              "in [-90, 30]");
    }

    MotionAliasCatalog catalog;
    std::string error_message;
    if (!catalog.load(alias_path, error_message)) {
      throw std::runtime_error(
              "failed to load motion alias catalog: " + error_message);
    }

    MotionBackendFactoryOptions backend_options;
    backend_options.backend_type = backend_type;
    backend_options.enable_robot_hardware = enable_robot_hardware;
    backend_options.robot_motion_player = make_robot_motion_runtime_config(
      motion_json_path, enable_robot_hardware, robot_device_path,
      robot_baud_rate, robot_motor_ids, explicit_torque_approval);
#if IRC_STEP_ROBOT_MOTION_PLAYER_BACKEND_BUILT
    if (backend_type == "robot_motion_player") {
      robot_motion_runtime_factory_ =
        std::make_unique<ProductionRobotMotionRuntimeFactory>();
      backend_options.robot_motion_runtime_factory =
        robot_motion_runtime_factory_.get();
    }
#endif
    backend_options.simulated.running_polls =
      static_cast<std::size_t>(running_polls);
    backend_options.simulated.settling_polls =
      static_cast<std::size_t>(settling_polls);
    backend_options.simulated.force_start_failure = declare_parameter<bool>(
      "force_start_failure", false);
    backend_options.simulated.force_backend_failure = declare_parameter<bool>(
      "force_backend_failure", false);

    auto backend_result = create_motion_backend(backend_options);
    if (!backend_result) {
      throw std::runtime_error(
              backend_result.error_code + ": " + backend_result.message);
    }
    runtime_owner_ = std::move(backend_result.runtime_owner);
    backend_ = std::move(backend_result.backend);
    if (backend_type == "robot_motion_player" &&
      !backend_->set_position_tolerance_enabled(position_tolerance_enabled))
    {
      throw std::runtime_error(
              "external RobotMotionPlayer SDK does not support disabling "
              "position tolerance checks");
    }
    RCLCPP_INFO(
      get_logger(), "Motion position tolerance check: %s",
      position_tolerance_enabled ? "ENABLED" : "DISABLED");
    std::vector<double> startup_pose_angles;
    if (startup_pose_enabled && !load_startup_pose_angles(
        motion_json_path, startup_pose_name, startup_pose_angles, error_message))
    {
      RCLCPP_ERROR(
        get_logger(), "[STARTUP POSE] ERROR: %s", error_message.c_str());
    }
    startup_pose_gate_ = std::make_unique<StartupPoseGate>(
      startup_pose_enabled, startup_pose_name, std::move(startup_pose_angles),
      startup_pose_duration_ms,
      backend_result.startup_pose_controller,
      [this](const std::string & message) {
        if (message.find("ERROR:") != std::string::npos) {
          RCLCPP_ERROR(get_logger(), "%s", message.c_str());
        } else {
          RCLCPP_INFO(get_logger(), "%s", message.c_str());
        }
      });
    core_ = std::make_unique<SdkExecutorCore>(
      std::move(catalog), *backend_);
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/motion/executor/status", 10);
    heartbeat_publisher_ = create_publisher<std_msgs::msg::String>(
      "/motion/executor/heartbeat", 10);
    driver_ = std::make_unique<SdkExecutorDriver>(
      *core_, steady_now_ms,
      [this](const std::string & payload) {
        std_msgs::msg::String message;
        message.data = payload;
        status_publisher_->publish(message);
      }, startup_pose_gate_.get());

    request_subscription_ = create_subscription<std_msgs::msg::String>(
      "/motion/executor/request", 10,
      [this](const std_msgs::msg::String::SharedPtr message) {
        clear_ball_head_override_before_pickup(message->data);
        driver_->handle_request(message->data);
      });
    cancel_subscription_ = create_subscription<std_msgs::msg::String>(
      "/motion/executor/cancel", 10,
      [this](const std_msgs::msg::String::SharedPtr message) {
        clear_ball_head_override();
        driver_->handle_cancel(message->data);
      });
    ball_info_subscription_ = create_subscription<std_msgs::msg::String>(
      "/vision/ball_info", 10,
      [this](const std_msgs::msg::String::SharedPtr message) {
        handle_ball_info(message->data);
      });
    poll_timer_ = create_wall_timer(
      std::chrono::milliseconds(poll_period_ms),
      [this]() {
        const auto now_ms = steady_now_ms();
        update_ball_head_override(now_ms);
        driver_->poll();
      });
    heartbeat_timer_ = create_wall_timer(
      std::chrono::milliseconds(heartbeat_period_ms),
      [this, backend_type]() {publish_heartbeat(backend_type);});

    if (backend_type == "simulated") {
      RCLCPP_WARN(
        get_logger(),
        "Simulated backend only: no SDK or hardware access is available");
    }
  }

private:
  bool ball_head_motion_active() const
  {
    const auto motion_id = core_->active_motion_id();
    return motion_id && std::find(
      ball_head_override_motion_ids_.begin(),
      ball_head_override_motion_ids_.end(),
      *motion_id) != ball_head_override_motion_ids_.end();
  }

  void handle_ball_info(const std::string & payload)
  {
    if (!ball_head_override_enabled_ || ball_head_override_latched_) {
      return;
    }
    json_object * object = json_tokener_parse(payload.c_str());
    if (object == nullptr || json_object_get_type(object) != json_type_object) {
      if (object != nullptr) {
        json_object_put(object);
      }
      return;
    }
    json_object * detected = nullptr;
    json_object * requested = nullptr;
    const bool should_override =
      json_object_object_get_ex(object, "detected", &detected) &&
      json_object_get_type(detected) == json_type_boolean &&
      json_object_get_boolean(detected) &&
      json_object_object_get_ex(
        object, "head_down_requested", &requested) &&
      json_object_get_type(requested) == json_type_boolean &&
      json_object_get_boolean(requested);
    json_object_put(object);
    if (!should_override || !ball_head_motion_active()) {
      return;
    }
    ball_head_override_latched_ = true;
    ball_head_override_started_ms_ = steady_now_ms();
    RCLCPP_INFO(
      get_logger(),
      "Ball entered bottom trigger: motor 0 override %.1f deg latched",
      ball_head_override_deg_);
  }

  void update_ball_head_override(std::uint64_t now_ms)
  {
    if (!ball_head_override_latched_) {
      return;
    }
    const auto elapsed_ms = now_ms >= ball_head_override_started_ms_ ?
      now_ms - ball_head_override_started_ms_ : 0;
    const double progress = std::min(
      1.0,
      static_cast<double>(elapsed_ms) /
      static_cast<double>(ball_head_transition_ms_));
    const double target_deg = ball_head_camera_up_deg_ +
      (ball_head_override_deg_ - ball_head_camera_up_deg_) * progress;
    if (!backend_->set_joint_override(0, target_deg)) {
      RCLCPP_ERROR(
        get_logger(), "Motion backend rejected motor 0 override");
      clear_ball_head_override();
    }
  }

  void clear_ball_head_override_before_pickup(const std::string & payload)
  {
    if (!ball_head_override_latched_) {
      return;
    }
    json_object * object = json_tokener_parse(payload.c_str());
    if (object == nullptr || json_object_get_type(object) != json_type_object) {
      if (object != nullptr) {
        json_object_put(object);
      }
      return;
    }
    json_object * motion_id_value = nullptr;
    const bool starts_pickup =
      json_object_object_get_ex(object, "motion_id", &motion_id_value) &&
      json_object_get_type(motion_id_value) == json_type_string &&
      (std::string(json_object_get_string(motion_id_value)) == "pickup" ||
      std::string(json_object_get_string(motion_id_value)) == "sdk_pickup");
    json_object_put(object);
    if (starts_pickup) {
      clear_ball_head_override();
      RCLCPP_INFO(
        get_logger(),
        "Motor 0 override cleared before pickup motion");
    }
  }

  void clear_ball_head_override() noexcept
  {
    if (!ball_head_override_latched_) {
      return;
    }
    backend_->clear_joint_override(0);
    ball_head_override_latched_ = false;
  }

  void publish_heartbeat(const std::string & backend_type)
  {
    json_object * object = json_object_new_object();
    json_object_object_add(
      object, "sequence",
      json_object_new_uint64(heartbeat_sequence_++));
    json_object_object_add(
      object, "backend_type", json_object_new_string(backend_type.c_str()));
    json_object_object_add(
      object, "active", json_object_new_boolean(core_->has_active_request()));
    json_object_object_add(
      object, "auto_ready",
      json_object_new_boolean(startup_pose_gate_->navigation_allowed()));
    json_object_object_add(
      object, "ball_head_override_active",
      json_object_new_boolean(ball_head_override_latched_));
    json_object_object_add(
      object, "ball_head_override_deg",
      json_object_new_double(ball_head_override_deg_));

    std_msgs::msg::String message;
    message.data = json_object_to_json_string_ext(
      object, JSON_C_TO_STRING_PLAIN);
    json_object_put(object);
    heartbeat_publisher_->publish(message);
  }

  std::int64_t positive_parameter_or_default(
    const std::string & name, std::int64_t default_value)
  {
    const std::int64_t value =
      declare_parameter<std::int64_t>(name, default_value);
    if (value >= 1) {
      return value;
    }
    RCLCPP_WARN(
      get_logger(), "%s must be at least 1; using %ld",
      name.c_str(), default_value);
    return default_value;
  }

  std::int64_t nonnegative_parameter_or_default(
    const std::string & name, std::int64_t default_value)
  {
    const std::int64_t value =
      declare_parameter<std::int64_t>(name, default_value);
    if (value >= 0) {
      return value;
    }
    RCLCPP_WARN(
      get_logger(), "%s must be nonnegative; using %ld",
      name.c_str(), default_value);
    return default_value;
  }

#if IRC_STEP_ROBOT_MOTION_PLAYER_BACKEND_BUILT
  // Declared first so the injected factory outlives backend construction/use.
  std::unique_ptr<RobotMotionRuntimeFactory> robot_motion_runtime_factory_;
#endif
  // Declared before the backend so borrowed SDK runtime state outlives it.
  std::shared_ptr<void> runtime_owner_;
  std::unique_ptr<MotionBackend> backend_;
  std::unique_ptr<SdkExecutorCore> core_;
  std::unique_ptr<StartupPoseGate> startup_pose_gate_;
  std::unique_ptr<SdkExecutorDriver> driver_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr request_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cancel_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    ball_info_subscription_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  std::uint64_t heartbeat_sequence_{0};
  bool ball_head_override_enabled_{true};
  bool ball_head_override_latched_{false};
  double ball_head_override_deg_{-60.0};
  double ball_head_camera_up_deg_{-33.0};
  std::int64_t ball_head_transition_ms_{400};
  std::uint64_t ball_head_override_started_ms_{0};
  std::vector<std::string> ball_head_override_motion_ids_;
};

}  // namespace irc_step_motion_executor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<irc_step_motion_executor::SdkMotionExecutorNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("sdk_motion_executor"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
