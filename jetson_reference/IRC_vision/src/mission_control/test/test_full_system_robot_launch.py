"""Tests for the hardware-disabled robot full-system launch defaults."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


LEGACY_EXECUTABLES = {
    "legacy_motion_executor_adapter",
    "motion_executor_node",
    "legacy_motion_status_adapter",
    "sdk_motion_stub_node",
}


def launch_description(monkeypatch, tmp_path):
    """Load the robot launch description without executing it."""
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "full_system_robot.launch.py"
    )
    spec = importlib.util.spec_from_file_location(
        "full_system_robot_launch",
        launch_path,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def default_context(description):
    """Populate a launch context with declared default arguments."""
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return context


def executor_parameters(executor, context):
    """Evaluate sdk_motion_executor parameters in one launch context."""
    parameters = executor._Node__parameters[0]
    output = {}
    for name, value in parameters.items():
        key = "".join(part.text for part in name)
        output[key] = value.evaluate(context) if hasattr(value, "evaluate") else value
    return output


def test_robot_launch_has_one_cpp_executor_and_no_legacy_nodes(
    monkeypatch,
    tmp_path,
):
    description = launch_description(monkeypatch, tmp_path)
    executables = [
        entity.node_executable
        for entity in description.entities
        if isinstance(entity, Node)
    ]

    assert executables.count("sdk_motion_executor") == 1
    assert "motion_command_bridge_node" in executables
    assert set(executables).isdisjoint(LEGACY_EXECUTABLES)


def test_robot_launch_defaults_are_production_ready(
    monkeypatch,
    tmp_path,
):
    description = launch_description(monkeypatch, tmp_path)
    context = default_context(description)
    executor = next(
        entity
        for entity in description.entities
        if isinstance(entity, Node)
        and entity.node_executable == "sdk_motion_executor"
    )
    parameters = executor_parameters(executor, context)

    assert context.launch_configurations["enable_camera"] == "true"
    assert context.launch_configurations["device"] == "tensorrt"
    assert context.launch_configurations["model_path"].endswith(
        "/step/models/best.engine"
    )
    assert context.launch_configurations["display"] == "true"
    assert context.launch_configurations["initial_mission_phase"] == "AUTO"
    assert parameters == {
        "backend_type": "robot_motion_player",
        "enable_robot_hardware": True,
        "poll_period_ms": 5,
        "running_polls": 2,
        "settling_polls": 1,
        "explicit_torque_approval": True,
        "position_tolerance_enabled": True,
        "motion_json_path": str(
            Path(__file__).resolve().parents[3]
            / "install"
            / "irc_step_motion_executor"
            / "share"
            / "irc_step_motion_executor"
            / "config"
            / "robot_motions_runtime.json"
        ),
        "robot_device_path": "/dev/ttyUSB0",
        "robot_baud_rate": 4000000,
        "robot_motor_ids": list(range(23)),
        "startup_pose_enabled": True,
        "startup_pose_name": "오뒤410",
        "startup_pose_duration_ms": 4000,
        "ball_head_override_deg": -60.0,
    }


def test_robot_launch_passes_realsense_topic_parameters(
    monkeypatch,
    tmp_path,
):
    description = launch_description(monkeypatch, tmp_path)
    context = default_context(description)
    nodes = {
        entity.node_executable: entity
        for entity in description.entities
        if isinstance(entity, Node)
    }

    detector_parameters = executor_parameters(
        nodes["yolo26_detector"],
        context,
    )
    assert detector_parameters["image_topic"] == (
        "/camera/camera/color/image_raw"
    )
    assert (
        context.launch_configurations["max_rgb_depth_delta_sec"] == "0.05"
    )
    assert (
        context.launch_configurations["overlay_max_stamp_delta_sec"]
        == "0.12"
    )
    assert detector_parameters["overlay_max_stamp_delta_sec"] == 0.12

    vision_parameters = executor_parameters(
        nodes["unified_vision_node"],
        context,
    )
    assert vision_parameters["image_topic"] == (
        "/camera/camera/color/image_raw"
    )
    assert vision_parameters["depth_topic"] == (
        "/camera/camera/aligned_depth_to_color/image_raw"
    )
    assert vision_parameters["camera_info_topic"] == (
        "/camera/camera/color/camera_info"
    )
    assert vision_parameters["max_rgb_depth_delta_sec"] == 0.05
    assert vision_parameters["head_down_trigger_bottom_distance_px"] == 120

    decision_parameters = executor_parameters(
        nodes["motion_decision_node"],
        context,
    )
    assert decision_parameters["pickup_fine_step_bottom_distance_px"] == 500
