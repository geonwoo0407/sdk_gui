from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "mission_control"))

from mock_mission_input_node import (  # noqa: E402
    MockScenarioError,
    REFRESH_PERIOD_SEC,
    build_mock_vision_input,
    should_publish,
)
from mission_control.motion_decision_planner import (  # noqa: E402
    MotionDecisionPlanner,
)


def test_straight_scenario_builds_centered_line_input():
    topic, payload = build_mock_vision_input("straight")
    assert topic == "/vision/line_info"
    assert payload["detected"] is True
    assert payload["filtered_heading_error_deg"] == 0.0
    assert payload["filtered_lateral_offset_norm"] == 0.0
    assert payload["heading_quality"] == 1.0
    assert payload["geometry_quality"] == 1.0
    assert payload["detection_quality"] == 1.0


@pytest.mark.parametrize(
    ("scenario", "heading_error"),
    [
        ("turn_left", -10.0),
        ("turn_right", 10.0),
    ],
)
def test_turn_scenario_builds_deterministic_line_input(
    scenario, heading_error
):
    topic, payload = build_mock_vision_input(scenario)
    assert topic == "/vision/line_info"
    assert payload["detected"] is True
    assert payload["filtered_heading_error_deg"] == heading_error
    assert payload["filtered_lateral_offset_norm"] == 0.0


def test_scenario_payloads_are_distinct():
    payloads = [
        build_mock_vision_input(scenario)[1]
        for scenario in ("straight", "turn_left", "turn_right")
    ]
    headings = {
        payload["filtered_heading_error_deg"] for payload in payloads
    }
    assert headings == {0.0, -10.0, 10.0}


@pytest.mark.parametrize(
    ("scenario", "expected_action"),
    [
        ("straight", "STRAIGHT"),
        ("turn_left", "RECOVER_LEFT_TURN_LEFT_2"),
        ("turn_right", "RECOVER_RIGHT_TURN_RIGHT_4"),
    ],
)
def test_mock_input_produces_existing_mission_action(
    scenario, expected_action
):
    _, line_info = build_mock_vision_input(scenario)
    observations = {
        "line": line_info,
        "ball": None,
        "goal": None,
        "hurdle": None,
        "finish": None,
    }
    decision = MotionDecisionPlanner().plan(
        "AUTO", observations, dt_sec=0.1
    )
    assert decision.action == expected_action
    assert decision.valid is True


def test_unsupported_scenario_is_rejected():
    with pytest.raises(MockScenarioError):
        build_mock_vision_input("camera")


def test_module_has_no_camera_or_model_imports():
    source_path = (
        Path(__file__).resolve().parents[1]
        / "mission_control"
        / "mock_mission_input_node.py"
    )
    source = source_path.read_text(encoding="utf-8").lower()
    forbidden_imports = (
        "import cv2",
        "import pyrealsense",
        "import ultralytics",
        "from ultralytics",
        "import yolo",
        "import dynamixel",
        "import serial",
        "import sdk",
    )
    assert all(item not in source for item in forbidden_imports)


def test_publish_once_setting_stops_after_first_publication():
    assert should_publish(True, False) is True
    assert should_publish(True, True) is False
    assert should_publish(False, False) is True
    assert should_publish(False, True) is True
    assert REFRESH_PERIOD_SEC < 0.5
