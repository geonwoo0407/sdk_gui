"""Launch the mock-safe, catalog-only C++ motion wrapper."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hardware_enable = LaunchConfiguration("hardware_enable")
    robot_motion_sdk_dir = LaunchConfiguration("robot_motion_sdk_dir")

    return LaunchDescription([
        DeclareLaunchArgument(
            "hardware_enable",
            default_value="false",
            description="Must remain false for the catalog-only node.",
        ),
        DeclareLaunchArgument(
            "robot_motion_sdk_dir",
            default_value="",
            description="Unused runtime SDK path; empty in catalog-only mode.",
        ),
        Node(
            package="irc_step_motion_executor",
            executable="catalog_only_node",
            name="irc_step_motion_catalog_only",
            output="screen",
            parameters=[{
                "hardware_enable": hardware_enable,
                "robot_motion_sdk_dir": robot_motion_sdk_dir,
            }],
        ),
    ])
