import copy
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "mission_control"))

from legacy_motion_executor_adapter import (  # noqa: E402
    LegacyMotionExecutorAdapter,
    LegacyCommandValidationError,
    build_executor_request,
    map_action_to_motion_id,
    parse_legacy_motion_command,
    timeout_for_motion,
)


@pytest.mark.parametrize(
    ("action", "motion_id"),
    [
        ("STRAIGHT", "forward"),
        ("APPROACH", "forward"),
        ("SLOW_APPROACH", "forward_short"),
        ("FINE_FORWARD_STEP", "forward_short"),
        ("APPROACH_GOAL", "forward_short"),
        ("APPROACH_HURDLE", "forward_short"),
        ("ALIGN_LEFT", "adjust_left"),
        ("ALIGN_RIGHT", "adjust_right"),
        ("RETREAT_GOAL", "backward"),
        ("TURN_LEFT", "turn_left"),
        ("LEFT", "turn_left"),
        ("RIGHT", "turn_right"),
        ("PICKUP_NOW", "pick_ball"),
        ("SHOT", "shoot"),
        ("GO", "forward"),
        ("CROSS_FINISH", "hurdle"),
    ],
)
def test_action_mapping(action, motion_id):
    assert map_action_to_motion_id(action) == motion_id


def test_timeout_mapping():
    assert timeout_for_motion("forward") == 5000
    assert timeout_for_motion("pick_ball") == 10000
    assert timeout_for_motion("shoot") == 10000
    assert timeout_for_motion("hurdle") == 12000
    assert timeout_for_motion("recover") == 8000


def test_build_request_preserves_correlation_ids():
    assert build_executor_request(37, "shoot", 123, "SHOT", 456) == {
        "request_id": 37,
        "command_id": 123,
        "event_id": 456,
        "action": "SHOT",
        "motion_id": "shoot",
        "timeout_ms": 10000,
    }


def test_invalid_json():
    with pytest.raises(LegacyCommandValidationError):
        parse_legacy_motion_command("{invalid")


def test_missing_action():
    with pytest.raises(LegacyCommandValidationError):
        parse_legacy_motion_command({"angle_deg": 0.0})


def test_unsupported_action():
    assert map_action_to_motion_id("FLY") is None
    assert map_action_to_motion_id("WAIT") is None
    assert map_action_to_motion_id("STOP") is None
    assert map_action_to_motion_id("HEAD_SCAN_LEFT") is None
    assert map_action_to_motion_id("HEAD_SCAN_RIGHT") is None
    assert map_action_to_motion_id("HEAD_CENTER") is None
    assert map_action_to_motion_id("RECOVER_GOAL_TURN_LEFT") is None
    assert map_action_to_motion_id("RECOVER_GOAL_TURN_RIGHT") is None


def test_parse_does_not_modify_input_object():
    source = {
        "action": "STRAIGHT",
        "angle_deg": 12.5,
        "command_id": 99,
        "event_id": 199,
    }
    original = copy.deepcopy(source)
    command = parse_legacy_motion_command(source)
    assert source == original
    assert command.action == "STRAIGHT"
    assert command.angle_deg == 12.5
    assert command.command_id == 99
    assert command.event_id == 199


def test_legacy_command_without_command_id_remains_supported():
    command = parse_legacy_motion_command({"action": "STRAIGHT"})
    assert command.command_id is None
    assert build_executor_request(1, "forward")["command_id"] is None
    assert build_executor_request(1, "forward")["event_id"] is None
    assert build_executor_request(1, "forward")["action"] is None


class CapturePublisher:

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeLogger:

    def warning(self, _message):
        pass


class FakeAdapter:

    def __init__(self):
        self._next_request_id = 1
        self._request_publisher = CapturePublisher()
        self._logger = FakeLogger()

    def get_logger(self):
        return self._logger


def send_legacy_command(adapter, **payload):
    message = SimpleNamespace(data=json.dumps(payload))
    LegacyMotionExecutorAdapter._on_legacy_command(adapter, message)


def test_left_and_right_create_executor_requests():
    adapter = FakeAdapter()
    send_legacy_command(
        adapter, action="LEFT", valid=True, command_id=101
    )
    send_legacy_command(
        adapter, action="RIGHT", valid=True, command_id=202
    )

    requests = [
        json.loads(message.data)
        for message in adapter._request_publisher.messages
    ]
    assert requests == [
        {
            "request_id": 1,
            "command_id": 101,
            "event_id": None,
            "action": "LEFT",
            "motion_id": "turn_left",
            "timeout_ms": 5000,
        },
        {
            "request_id": 2,
            "command_id": 202,
            "event_id": None,
            "action": "RIGHT",
            "motion_id": "turn_right",
            "timeout_ms": 5000,
        },
    ]


@pytest.mark.parametrize(
    "payload",
    [
        {"action": "WAIT", "valid": False},
        {"action": "STRAIGHT", "valid": False},
        {"action": "FLY", "valid": True},
        {"action": "BALL_LOST_STOP", "valid": False},
        {"action": "GOAL_LOST_STOP", "valid": False},
        {"action": "HEAD_SCAN_LEFT", "valid": False},
        {"action": "HEAD_SCAN_RIGHT", "valid": False},
        {"action": "HEAD_CENTER", "valid": False},
        {"action": "WAIT_SCORE_CONFIRMATION", "valid": False},
        {"action": "WAIT_GO_CONFIRMATION", "valid": False},
    ],
)
def test_non_executable_command_does_not_create_request(payload):
    adapter = FakeAdapter()
    send_legacy_command(adapter, **payload)
    assert adapter._request_publisher.messages == []
    assert adapter._next_request_id == 1


@pytest.mark.parametrize(
    ("action", "command_id", "motion_id"),
    [
        ("STRAIGHT", 101, "forward"),
        ("PICKUP_NOW", 102, "pick_ball"),
        ("SHOT", 103, "shoot"),
        ("GO", 104, "forward"),
        ("CROSS_FINISH", 105, "hurdle"),
    ],
)
def test_original_action_and_command_id_are_added_to_request(
    action, command_id, motion_id
):
    adapter = FakeAdapter()
    send_legacy_command(
        adapter,
        action=action,
        valid=True,
        command_id=command_id,
    )
    request = json.loads(adapter._request_publisher.messages[0].data)
    assert request["action"] == action
    assert request["command_id"] == command_id
    assert request["motion_id"] == motion_id


def test_event_id_is_added_to_executor_request():
    adapter = FakeAdapter()
    send_legacy_command(
        adapter,
        action="PICKUP_NOW",
        valid=True,
        command_id=501,
        event_id=601,
    )

    request = json.loads(adapter._request_publisher.messages[0].data)
    assert request["command_id"] == 501
    assert request["event_id"] == 601
