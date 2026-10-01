"""
Launch the mock-only Motion Executor integration path.

This launch file uses MockRobotMotionPlayer only. It does not start the real
RobotMotionPlayer, access Dynamixel, or move a physical robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    tick_period_ms = LaunchConfiguration("tick_period_ms")
    mock_fail_after_updates = LaunchConfiguration(
        "mock_fail_after_updates"
    )
    mock_failure_code = LaunchConfiguration("mock_failure_code")
    player_backend = LaunchConfiguration("player_backend")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "player_backend",
                default_value="mock",
                description=(
                    "Safe player backend: mock or disconnected sdk placeholder"
                ),
            ),
            DeclareLaunchArgument(
                "tick_period_ms",
                default_value="100",
                description="Mock Motion Executor tick period in milliseconds",
            ),
            DeclareLaunchArgument(
                "mock_fail_after_updates",
                default_value="-1",
                description=(
                    "Test-only mock failure update count; -1 disables failure"
                ),
            ),
            DeclareLaunchArgument(
                "mock_failure_code",
                default_value="COMMUNICATION_ERROR",
                description="Test-only MotionError enum name to inject",
            ),
            Node(
                package="mission_control",
                executable="motion_executor_node",
                name="motion_executor_node",
                output="screen",
                parameters=[
                    {
                        "tick_period_ms": ParameterValue(
                            tick_period_ms, value_type=int
                        ),
                        "player_backend": ParameterValue(
                            player_backend, value_type=str
                        ),
                        "mock_fail_after_updates": ParameterValue(
                            mock_fail_after_updates, value_type=int
                        ),
                        "mock_failure_code": ParameterValue(
                            mock_failure_code, value_type=str
                        ),
                    }
                ],
            ),
            Node(
                package="mission_control",
                executable="legacy_motion_executor_adapter",
                name="legacy_motion_executor_adapter",
                output="screen",
            ),
            Node(
                package="mission_control",
                executable="legacy_motion_status_adapter",
                name="legacy_motion_status_adapter",
                output="screen",
            ),
        ]
    )
