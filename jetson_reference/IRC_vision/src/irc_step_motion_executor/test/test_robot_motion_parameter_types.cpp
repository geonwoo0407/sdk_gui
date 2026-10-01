#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>

#include <gtest/gtest.h>

#include <string>

TEST(RobotMotionParameterTypes, RejectsValuesOfTheWrongRosType)
{
  const rclcpp::Parameter wrong_hardware_enable(
    "enable_robot_hardware", std::string("true"));
  const rclcpp::Parameter wrong_device_path("robot_device_path", 1);
  const rclcpp::Parameter wrong_baud_rate(
    "robot_baud_rate", std::string("4000000"));
  const rclcpp::Parameter wrong_motor_ids(
    "robot_motor_ids", std::string("0,1,2"));
  const rclcpp::Parameter wrong_torque_approval(
    "explicit_torque_approval", std::string("true"));

  EXPECT_THROW(
    static_cast<void>(wrong_hardware_enable.as_bool()),
    rclcpp::ParameterTypeException);
  EXPECT_THROW(
    static_cast<void>(wrong_device_path.as_string()),
    rclcpp::ParameterTypeException);
  EXPECT_THROW(
    static_cast<void>(wrong_baud_rate.as_int()),
    rclcpp::ParameterTypeException);
  EXPECT_THROW(
    static_cast<void>(wrong_motor_ids.as_integer_array()),
    rclcpp::ParameterTypeException);
  EXPECT_THROW(
    static_cast<void>(wrong_torque_approval.as_bool()),
    rclcpp::ParameterTypeException);
}
