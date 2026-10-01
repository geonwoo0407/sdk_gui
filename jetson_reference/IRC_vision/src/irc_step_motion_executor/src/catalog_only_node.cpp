#include "irc_step_motion_executor/catalog_only_core.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace irc_step_motion_executor
{

class CatalogOnlyNode : public rclcpp::Node
{
public:
  CatalogOnlyNode()
  : Node("irc_step_motion_catalog_only")
  {
    const std::string default_alias_path =
      ament_index_cpp::get_package_share_directory(
      "irc_step_motion_executor") + "/config/motion_aliases.yaml";
    const std::string alias_path =
      declare_parameter<std::string>("motion_aliases_file", default_alias_path);
    const bool hardware_enable =
      declare_parameter<bool>("hardware_enable", false);
    const std::string sdk_path =
      declare_parameter<std::string>("robot_motion_sdk_dir", "");
    if (hardware_enable || !sdk_path.empty()) {
      throw std::runtime_error(
              "catalog-only node forbids hardware_enable and SDK runtime paths");
    }

    MotionAliasCatalog catalog;
    std::string error_message;
    if (!catalog.load(alias_path, error_message)) {
      throw std::runtime_error(
              "failed to load motion alias catalog: " + error_message);
    }
    core_ = std::make_unique<CatalogOnlyCore>(std::move(catalog));

    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/motion/executor/status", 10);
    request_subscription_ = create_subscription<std_msgs::msg::String>(
      "/motion/executor/request", 10,
      std::bind(&CatalogOnlyNode::on_request, this, std::placeholders::_1));
    cancel_subscription_ = create_subscription<std_msgs::msg::String>(
      "/motion/executor/cancel", 10,
      std::bind(&CatalogOnlyNode::on_cancel, this, std::placeholders::_1));
    RCLCPP_WARN(
      get_logger(),
      "Catalog-only mode active: RobotMotionPlayer and hardware access are disabled");
  }

private:
  void publish(const MotionStatus & status)
  {
    std_msgs::msg::String message;
    message.data = CatalogOnlyCore::to_json(status);
    status_publisher_->publish(message);
  }

  void on_request(const std_msgs::msg::String::SharedPtr message)
  {
    publish(core_->handle_request(message->data));
  }

  void on_cancel(const std_msgs::msg::String::SharedPtr message)
  {
    publish(core_->handle_cancel(message->data));
  }

  std::unique_ptr<CatalogOnlyCore> core_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr request_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cancel_subscription_;
};

}  // namespace irc_step_motion_executor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<irc_step_motion_executor::CatalogOnlyNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("irc_step_motion_executor"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
