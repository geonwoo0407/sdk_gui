"""Launch only the SDK motion executor, hardware-disabled by default."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create an explicit, single-purpose executor launch description."""
    backend_type = LaunchConfiguration("backend_type")
    enable_robot_hardware = LaunchConfiguration("enable_robot_hardware")
    explicit_torque_approval = LaunchConfiguration("explicit_torque_approval")
    motion_json_path = LaunchConfiguration("motion_json_path")
    robot_device_path = LaunchConfiguration("robot_device_path")
    robot_baud_rate = LaunchConfiguration("robot_baud_rate")
    robot_motor_ids = LaunchConfiguration("robot_motor_ids")
    motion_aliases_file = LaunchConfiguration("motion_aliases_file")
    poll_period_ms = LaunchConfiguration("poll_period_ms")

    return LaunchDescription([
        DeclareLaunchArgument("backend_type", default_value="simulated"),
        DeclareLaunchArgument(
            "enable_robot_hardware", default_value="false"
        ),
        DeclareLaunchArgument(
            "explicit_torque_approval", default_value="false"
        ),
        DeclareLaunchArgument("motion_json_path", default_value=""),
        DeclareLaunchArgument("robot_device_path", default_value=""),
        DeclareLaunchArgument("robot_baud_rate", default_value="0"),
        DeclareLaunchArgument(
            "robot_motor_ids",
            default_value=(
                "[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,"
                "19,20,21,22]"
            ),
        ),
        DeclareLaunchArgument(
            "motion_aliases_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("irc_step_motion_executor"),
                "config",
                "motion_aliases.yaml",
            ]),
        ),
        DeclareLaunchArgument("poll_period_ms", default_value="5"),
        Node(
            package="irc_step_motion_executor",
            executable="sdk_motion_executor",
            name="sdk_motion_executor",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "backend_type": ParameterValue(
                    backend_type, value_type=str
                ),
                "enable_robot_hardware": ParameterValue(
                    enable_robot_hardware, value_type=bool
                ),
                "explicit_torque_approval": ParameterValue(
                    explicit_torque_approval, value_type=bool
                ),
                "motion_json_path": ParameterValue(
                    motion_json_path, value_type=str
                ),
                "robot_device_path": ParameterValue(
                    robot_device_path, value_type=str
                ),
                "robot_baud_rate": ParameterValue(
                    robot_baud_rate, value_type=int
                ),
                "robot_motor_ids": ParameterValue(robot_motor_ids),
                "motion_aliases_file": motion_aliases_file,
                "poll_period_ms": ParameterValue(
                    poll_period_ms, value_type=int
                ),
            }],
        ),
    ])
