"""Launch the camera-free mission-to-Motion-Executor mock integration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    player_backend = LaunchConfiguration("player_backend")
    tick_period_ms = LaunchConfiguration("tick_period_ms")
    mock_fail_after_updates = LaunchConfiguration(
        "mock_fail_after_updates"
    )
    mock_failure_code = LaunchConfiguration("mock_failure_code")
    scenario = LaunchConfiguration("scenario")
    publish_delay_sec = LaunchConfiguration("publish_delay_sec")
    publish_once = LaunchConfiguration("publish_once")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "player_backend",
                default_value="mock",
                description="Motion Executor backend for safe mock validation",
            ),
            DeclareLaunchArgument(
                "tick_period_ms",
                default_value="500",
                description="Mock Motion Executor tick period in milliseconds",
            ),
            DeclareLaunchArgument(
                "mock_fail_after_updates",
                default_value="-1",
                description="Mock failure update count; -1 disables failure",
            ),
            DeclareLaunchArgument(
                "mock_failure_code",
                default_value="COMMUNICATION_ERROR",
                description="MotionError enum name used for mock failure",
            ),
            DeclareLaunchArgument(
                "scenario",
                default_value="straight",
                description=(
                    "Mock input scenario: straight, turn_left, or turn_right"
                ),
            ),
            DeclareLaunchArgument(
                "publish_delay_sec",
                default_value="1.0",
                description="Delay before publishing mock vision input",
            ),
            DeclareLaunchArgument(
                "publish_once",
                default_value="true",
                description="Publish the mock input only once",
            ),
            Node(
                package="mission_control",
                executable="motion_decision_node",
                name="motion_decision_node",
                output="screen",
            ),
            Node(
                package="mission_control",
                executable="mock_mission_input_node",
                name="mock_mission_input_node",
                output="screen",
                parameters=[
                    {
                        "scenario": scenario,
                        "publish_delay_sec": ParameterValue(
                            publish_delay_sec, value_type=float
                        ),
                        "publish_once": ParameterValue(
                            publish_once, value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="mission_control",
                executable="motion_executor_node",
                name="motion_executor_node",
                output="screen",
                parameters=[
                    {
                        "player_backend": ParameterValue(
                            player_backend, value_type=str
                        ),
                        "tick_period_ms": ParameterValue(
                            tick_period_ms, value_type=int
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
