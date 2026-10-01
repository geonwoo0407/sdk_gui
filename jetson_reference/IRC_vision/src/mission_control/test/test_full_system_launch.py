"""Tests for the selectable C++ full-system launch topology."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


LEGACY_EXECUTABLES = {
    "legacy_motion_executor_adapter",
    "motion_executor_node",
    "legacy_motion_status_adapter",
    "sdk_motion_stub_node",
}


def load_launch_module():
    """Load full_system.launch.py as a Python module."""
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "full_system.launch.py"
    )
    spec = importlib.util.spec_from_file_location(
        "full_system_launch",
        launch_path,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_description(monkeypatch, tmp_path):
    """Create the launch description without starting any node."""
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    return load_launch_module().generate_launch_description()


def launch_context(description, overrides=None):
    """Populate a context with launch defaults and optional overrides."""
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    context.launch_configurations.update(overrides or {})
    return context


def node_parameters(node, context):
    """Evaluate executor parameters in a launch context."""
    parameters = node._Node__parameters[0]

    def evaluate(value):
        if hasattr(value, "evaluate"):
            return value.evaluate(context)
        if isinstance(value, tuple):
            if all(hasattr(part, "perform") for part in value):
                return perform_substitutions(context, list(value))
            return list(value)
        return value

    return {
        "".join(part.text for part in name): evaluate(value)
        for name, value in parameters.items()
    }


def test_full_system_uses_bridge_and_cpp_simulated_executor(
    monkeypatch,
    tmp_path,
):
    description = launch_description(monkeypatch, tmp_path)
    nodes = [
        entity for entity in description.entities if isinstance(entity, Node)
    ]
    executables = {node.node_executable for node in nodes}

    assert {
        "yolo26_detector",
        "unified_vision_node",
        "motion_decision_node",
        "motion_command_bridge_node",
        "sdk_motion_executor",
    }.issubset(executables)
    assert executables.isdisjoint(LEGACY_EXECUTABLES)
    assert sum(
        node.node_executable == "sdk_motion_executor" for node in nodes
    ) == 1


def test_cpp_executor_defaults_are_safe_and_simulated(
    monkeypatch,
    tmp_path,
):
    description = launch_description(monkeypatch, tmp_path)
    executor = next(
        node
        for node in description.entities
        if isinstance(node, Node)
        and node.node_executable == "sdk_motion_executor"
    )
    parameters = node_parameters(executor, launch_context(description))

    assert parameters == {
        "backend_type": "simulated",
        "enable_robot_hardware": False,
        "poll_period_ms": 5,
        "running_polls": 2,
        "settling_polls": 1,
        "explicit_torque_approval": False,
        "position_tolerance_enabled": True,
        "motion_json_path": "",
        "robot_device_path": "/dev/ttyUSB0",
        "robot_baud_rate": 4000000,
        "robot_motor_ids": list(range(23)),
        "ball_head_override_deg": -60.0,
    }


def test_production_arguments_reach_cpp_executor(monkeypatch, tmp_path):
    description = launch_description(monkeypatch, tmp_path)
    executor = next(
        node
        for node in description.entities
        if isinstance(node, Node)
        and node.node_executable == "sdk_motion_executor"
    )
    context = launch_context(
        description,
        {
            "backend_type": "robot_motion_player",
            "enable_robot_hardware": "true",
            "explicit_torque_approval": "true",
            "position_tolerance_enabled": "false",
            "motion_json_path": "/tmp/robot_motions.json",
        },
    )
    parameters = node_parameters(executor, context)

    assert parameters["backend_type"] == "robot_motion_player"
    assert parameters["enable_robot_hardware"] is True
    assert parameters["explicit_torque_approval"] is True
    assert parameters["position_tolerance_enabled"] is False
    assert parameters["motion_json_path"] == "/tmp/robot_motions.json"
    assert parameters["robot_device_path"] == "/dev/ttyUSB0"
    assert parameters["robot_baud_rate"] == 4000000
    assert parameters["robot_motor_ids"] == list(range(23))
    assert parameters["ball_head_override_deg"] == -60.0


def test_overlay_tolerance_is_separate_from_depth_sync(
    monkeypatch,
    tmp_path,
):
    description = launch_description(monkeypatch, tmp_path)
    context = launch_context(description)
    detector = next(
        node
        for node in description.entities
        if isinstance(node, Node)
        and node.node_executable == "yolo26_detector"
    )
    parameters = node_parameters(detector, context)

    assert (
        context.launch_configurations["max_rgb_depth_delta_sec"] == "0.05"
    )
    assert (
        context.launch_configurations["overlay_max_stamp_delta_sec"]
        == "0.12"
    )
    assert parameters["overlay_max_stamp_delta_sec"] == 0.12

    unified_vision = next(
        node
        for node in description.entities
        if isinstance(node, Node)
        and node.node_executable == "unified_vision_node"
    )
    vision_parameters = node_parameters(unified_vision, context)
    assert vision_parameters["max_rgb_depth_delta_sec"] == 0.05
    assert vision_parameters["head_down_trigger_bottom_distance_px"] == 120

    motion_decision = next(
        node
        for node in description.entities
        if isinstance(node, Node)
        and node.node_executable == "motion_decision_node"
    )
    decision_parameters = node_parameters(motion_decision, context)
    assert decision_parameters["pickup_fine_step_bottom_distance_px"] == 500


def test_detector_defaults_to_prebuilt_tensorrt_engine(monkeypatch, tmp_path):
    description = launch_description(monkeypatch, tmp_path)
    context = launch_context(description)
    detector = next(
        node
        for node in description.entities
        if isinstance(node, Node)
        and node.node_executable == "yolo26_detector"
    )
    parameters = node_parameters(detector, context)

    assert context.launch_configurations["device"] == "tensorrt"
    assert context.launch_configurations["model_path"].endswith(
        "/models/best.engine"
    )
    assert parameters["device"] == "tensorrt"
    assert parameters["model_path"].endswith("/models/best.engine")


def test_obsolete_motion_launch_arguments_are_removed(monkeypatch, tmp_path):
    description = launch_description(monkeypatch, tmp_path)
    arguments = {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }

    assert "execution_mode" not in arguments
    assert "player_backend" not in arguments
    assert "mock_fail_after_updates" not in arguments
    assert "mock_failure_code" not in arguments
