"""Launch the hardware-free C++ SDK executor with its simulated backend."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    poll_period_ms = LaunchConfiguration("poll_period_ms")
    motion_aliases_file = LaunchConfiguration("motion_aliases_file")
    running_polls = LaunchConfiguration("running_polls")
    settling_polls = LaunchConfiguration("settling_polls")
    force_start_failure = LaunchConfiguration("force_start_failure")
    force_backend_failure = LaunchConfiguration("force_backend_failure")

    return LaunchDescription([
        DeclareLaunchArgument(
            "motion_aliases_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("irc_step_motion_executor"),
                "config",
                "motion_aliases.yaml",
            ]),
        ),
        DeclareLaunchArgument("poll_period_ms", default_value="5"),
        DeclareLaunchArgument("running_polls", default_value="2"),
        DeclareLaunchArgument("settling_polls", default_value="1"),
        DeclareLaunchArgument("force_start_failure", default_value="false"),
        DeclareLaunchArgument("force_backend_failure", default_value="false"),
        Node(
            package="irc_step_motion_executor",
            executable="sdk_motion_executor",
            name="sdk_motion_executor",
            output="screen",
            parameters=[{
                "backend_type": "simulated",
                "enable_robot_hardware": False,
                "motion_aliases_file": motion_aliases_file,
                "poll_period_ms": poll_period_ms,
                "running_polls": running_polls,
                "settling_polls": settling_polls,
                "force_start_failure": force_start_failure,
                "force_backend_failure": force_backend_failure,
            }],
        ),
    ])
