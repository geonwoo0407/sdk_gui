import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def load_launch_module():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "motion_executor_mock.launch.py"
    )
    spec = importlib.util.spec_from_file_location(
        "motion_executor_mock_launch", launch_path
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_mock_launch_description(monkeypatch, tmp_path):
    # Launch actions initialize logging, but no ROS graph or process is run.
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    module = load_launch_module()
    description = module.generate_launch_description()

    arguments = {
        action.name
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    nodes = [
        action
        for action in description.entities
        if isinstance(action, Node)
    ]
    executables = {node.node_executable for node in nodes}

    mission_control_nodes = [
        node for node in nodes if node.node_package == "mission_control"
    ]

    assert len(mission_control_nodes) == 3
    assert "motion_executor_node" in executables
    assert "legacy_motion_executor_adapter" in executables
    assert "legacy_motion_status_adapter" in executables
    assert "tick_period_ms" in arguments
    assert "mock_fail_after_updates" in arguments
    assert "mock_failure_code" in arguments
    assert "player_backend" in arguments

    player_backend_argument = next(
        action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
        and action.name == "player_backend"
    )
    default_backend = "".join(
        substitution.text
        for substitution in player_backend_argument.default_value
    )
    assert default_backend == "mock"

    forbidden_executables = {
        "robot_motion_player",
        "sdk_motion_stub_node",
        "dynamixel",
        "dynamixel_node",
    }
    assert executables.isdisjoint(forbidden_executables)
