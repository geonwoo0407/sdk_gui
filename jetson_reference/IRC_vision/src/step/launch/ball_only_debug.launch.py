#!/usr/bin/env python3
"""Launch RealSense and an independent ball-only ONNX diagnostic."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Build the standalone camera-to-ball-debug graph."""
    enable_camera = LaunchConfiguration("enable_camera")
    device = LaunchConfiguration("device")
    display = LaunchConfiguration("display")
    comparison_threshold = LaunchConfiguration("comparison_threshold")
    max_fps = LaunchConfiguration("max_fps")

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        condition=IfCondition(enable_camera),
        launch_arguments={
            "align_depth.enable": "true",
            "enable_gyro": "false",
            "enable_accel": "false",
            "rgb_camera.color_profile": "1280x720x30",
            "depth_module.depth_profile": "848x480x30",
        }.items(),
    )

    ball_debug = Node(
        package="step",
        executable="ball_only_debug",
        name="ball_only_debug",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "device": device,
                "display": ParameterValue(display, value_type=bool),
                "comparison_threshold": ParameterValue(
                    comparison_threshold,
                    value_type=float,
                ),
                "max_fps": ParameterValue(max_fps, value_type=float),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_camera",
                default_value="true",
                description="Launch RealSense for a fully isolated test.",
            ),
            DeclareLaunchArgument(
                "device",
                default_value="cpu",
                description="ONNX Runtime provider: cpu, cuda, or auto.",
            ),
            DeclareLaunchArgument(
                "display",
                default_value="true",
                description="Show the independent raw ball debug window.",
            ),
            DeclareLaunchArgument(
                "comparison_threshold",
                default_value="0.20",
                description="Only compare the raw score; never filters it.",
            ),
            DeclareLaunchArgument(
                "max_fps",
                default_value="15.0",
                description="Maximum independent inference rate.",
            ),
            camera,
            ball_debug,
        ]
    )
