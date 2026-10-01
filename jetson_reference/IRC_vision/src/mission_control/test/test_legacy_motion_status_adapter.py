import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "mission_control"))

from legacy_motion_status_adapter import (  # noqa: E402
    ExecutorStatusValidationError,
    convert_executor_status,
    convert_executor_status_json,
)


def executor_status(status, **overrides):
    payload = {
        "request_id": 7,
        "command_id": 123,
        "action": "PICKUP_NOW",
        "motion_id": "pick_ball",
        "status": status,
        "error_code": "",
        "message": "",
    }
    payload.update(overrides)
    return payload


@pytest.mark.parametrize(
    ("executor_value", "legacy_value"),
    [
        ("RUNNING", "RUNNING"),
        ("SUCCEEDED", "SUCCEEDED"),
        ("CANCELLED", "CANCELLED"),
        ("TIMEOUT", "TIMEOUT"),
        ("REJECTED", "REJECTED"),
    ],
)
def test_status_conversion(executor_value, legacy_value):
    converted = convert_executor_status(
        executor_status(executor_value)
    )
    assert converted["status"] == legacy_value
    assert converted["motion_in_progress"] is (
        executor_value == "RUNNING"
    )
    assert converted["action"] == "PICKUP_NOW"
    assert converted["command_id"] == 123
    assert converted["event_id"] is None


def test_failed_preserves_executor_details():
    converted = convert_executor_status(
        executor_status(
            "FAILED",
            error_code="COMMUNICATION_ERROR",
            message="mock failure",
        )
    )
    assert converted["status"] == "FAILED"
    assert converted["request_id"] == 7
    assert converted["command_id"] == 123
    assert converted["motion_id"] == "pick_ball"
    assert converted["error_code"] == "COMMUNICATION_ERROR"
    assert converted["message"] == "mock failure"


def test_invalid_json_is_rejected():
    with pytest.raises(ExecutorStatusValidationError):
        convert_executor_status("{invalid")


def test_missing_status_is_rejected():
    with pytest.raises(ExecutorStatusValidationError):
        convert_executor_status({"request_id": 1})


def test_unknown_status_is_rejected():
    with pytest.raises(ExecutorStatusValidationError):
        convert_executor_status(executor_status("PAUSED"))


def test_one_input_produces_one_output_object():
    output = convert_executor_status_json(
        json.dumps(executor_status("SUCCEEDED"))
    )
    assert isinstance(json.loads(output), dict)
    assert output.count('"status"') == 1


def test_legacy_executor_status_without_command_id_uses_null():
    payload = executor_status("RUNNING")
    del payload["command_id"]
    converted = convert_executor_status(payload)
    assert converted["command_id"] is None
    assert converted["request_id"] == 7


@pytest.mark.parametrize(
    ("action", "motion_id"),
    [
        ("STRAIGHT", "forward"),
        ("PICKUP_NOW", "pick_ball"),
        ("SHOT", "shoot"),
        ("GO", "forward"),
        ("CROSS_FINISH", "hurdle"),
    ],
)
@pytest.mark.parametrize("status", ["RUNNING", "SUCCEEDED"])
def test_original_action_wins_over_lossy_motion_id_mapping(
    action, motion_id, status
):
    converted = convert_executor_status(
        executor_status(
            status,
            command_id=100,
            action=action,
            motion_id=motion_id,
        )
    )
    assert converted["action"] == action
    assert converted["command_id"] == 100
    assert converted["request_id"] == 7


def test_executor_event_id_is_preserved_for_mission_correlation():
    converted = convert_executor_status(
        executor_status(
            "SUCCEEDED",
            action="GO",
            motion_id="forward",
            command_id=123,
            event_id=456,
        )
    )

    assert converted["command_id"] == 123
    assert converted["event_id"] == 456


def test_old_status_without_original_action_uses_motion_id_fallback():
    payload = executor_status("RUNNING", motion_id="forward")
    del payload["action"]
    assert convert_executor_status(payload)["action"] == "STRAIGHT"
