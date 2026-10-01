import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "mission_control"))

from mock_motion_player import MockRobotMotionPlayer, MotionError  # noqa: E402
from motion_executor_core import (  # noqa: E402
    ExecutorState,
    MotionExecutionResult,
    MotionExecutorCore,
)
from motion_executor_node import (  # noqa: E402
    CancelRequest,
    DEFAULT_PLAYER_BACKEND,
    ExecutionPublicationState,
    MotionRequest,
    MotionExecutorNode,
    RequestValidationError,
    build_status_payload,
    handle_cancel_request,
    parse_cancel_request,
    parse_motion_request,
)
from motion_player_factory import create_motion_player  # noqa: E402


def result(status, error_code="NONE", message="done"):
    return MotionExecutionResult(
        motion_id="forward",
        final_status=status,
        success=status is ExecutorState.SUCCEEDED,
        error_code=error_code,
        message=message,
    )


def test_parse_valid_request_json():
    request = parse_motion_request(
        '{"request_id": 1, "command_id": 123, '
        '"event_id": 456, "action": "GO", '
        '"motion_id": "forward", "timeout_ms": 5000}'
    )
    assert request == MotionRequest(1, "forward", 5000, 123, "GO", 456)


def test_default_player_backend_is_mock():
    assert DEFAULT_PLAYER_BACKEND == "mock"
    assert isinstance(
        create_motion_player(DEFAULT_PLAYER_BACKEND),
        MockRobotMotionPlayer,
    )


def test_missing_required_field():
    with pytest.raises(RequestValidationError) as exc_info:
        parse_motion_request('{"request_id": 1, "motion_id": "forward"}')
    assert exc_info.value.error_code == "INVALID_REQUEST"


def test_invalid_json():
    with pytest.raises(RequestValidationError) as exc_info:
        parse_motion_request("{invalid")
    assert exc_info.value.error_code == "INVALID_REQUEST"


def test_non_positive_timeout():
    for timeout_ms in (0, -1):
        with pytest.raises(RequestValidationError) as exc_info:
            parse_motion_request(
                '{"request_id": 1, "motion_id": "forward", '
                f'"timeout_ms": {timeout_ms}}}'
            )
        assert exc_info.value.error_code == "INVALID_REQUEST"


def test_running_status_payload():
    state = ExecutionPublicationState()
    state.begin(MotionRequest(7, "turn_left", 1000, 123, "LEFT"))
    assert state.running_payload() == {
        "request_id": 7,
        "command_id": 123,
        "event_id": None,
        "action": "LEFT",
        "motion_id": "turn_left",
        "status": "RUNNING",
        "error_code": "",
        "message": "",
    }


def test_succeeded_terminal_payload_preserves_correlation_ids():
    state = ExecutionPublicationState()
    state.begin(MotionRequest(1, "forward", 5000, 123, "GO", 456))
    payload = state.terminal_payload(result(ExecutorState.SUCCEEDED))
    assert payload["status"] == "SUCCEEDED"
    assert payload["command_id"] == 123
    assert payload["event_id"] == 456
    assert payload["action"] == "GO"
    assert payload["error_code"] == "NONE"


@pytest.mark.parametrize(
    "status",
    [
        ExecutorState.SUCCEEDED,
        ExecutorState.FAILED,
        ExecutorState.TIMEOUT,
        ExecutorState.CANCELLED,
    ],
)
def test_all_terminal_payloads_preserve_command_id(status):
    state = ExecutionPublicationState()
    state.begin(MotionRequest(7, "forward", 5000, 321, "GO"))
    payload = state.terminal_payload(result(status))
    assert payload["status"] == status.name
    assert payload["command_id"] == 321
    assert payload["request_id"] == 7
    assert payload["action"] == "GO"


def test_failed_payload():
    payload = build_status_payload(
        1,
        "forward",
        "FAILED",
        "COMMUNICATION_ERROR",
        "send failed",
        123,
    )
    assert payload["status"] == "FAILED"
    assert payload["command_id"] == 123
    assert payload["error_code"] == "COMMUNICATION_ERROR"


def test_rejected_payload():
    payload = build_status_payload(
        2, "fly", "REJECTED", "INVALID_MOTION", "unsupported motion_id"
    )
    assert payload["status"] == "REJECTED"
    assert payload["error_code"] == "INVALID_MOTION"


def test_rejected_busy_preserves_both_request_ids():
    class BusyCore:

        @staticmethod
        def busy():
            return True

    class FakeNode:

        def __init__(self):
            self._core = BusyCore()
            self.payloads = []

        def _publish_payload(self, payload):
            self.payloads.append(payload)

    node = FakeNode()
    message = SimpleNamespace(
        data=json.dumps(
            {
                "request_id": 7,
                "command_id": 321,
                "event_id": 654,
                "action": "GO",
                "motion_id": "forward",
                "timeout_ms": 5000,
            }
        )
    )
    MotionExecutorNode._on_request(node, message)

    assert node.payloads == [
        {
            "request_id": 7,
            "command_id": 321,
            "event_id": 654,
            "action": "GO",
            "motion_id": "forward",
            "status": "REJECTED",
            "error_code": "REJECTED_BUSY",
            "message": "another motion is already running",
        }
    ]


def test_original_request_id_and_motion_id_are_preserved():
    state = ExecutionPublicationState()
    state.begin(MotionRequest(42, "shoot", 8000, 9001))
    payload = state.terminal_payload(
        MotionExecutionResult(
            motion_id="internal_motion",
            final_status=ExecutorState.SUCCEEDED,
            success=True,
            error_code="NONE",
            message="done",
        )
    )
    assert payload["request_id"] == 42
    assert payload["motion_id"] == "shoot"
    assert payload["command_id"] == 9001


def test_request_without_command_id_remains_supported():
    request = parse_motion_request(
        '{"request_id": 1, "motion_id": "forward", "timeout_ms": 5000}'
    )
    assert request.command_id is None
    state = ExecutionPublicationState()
    state.begin(request)
    assert state.running_payload()["command_id"] is None


def test_terminal_payload_is_not_duplicated():
    state = ExecutionPublicationState()
    state.begin(MotionRequest(1, "forward", 5000))
    terminal = result(ExecutorState.SUCCEEDED)
    assert state.terminal_payload(terminal) is not None
    assert state.terminal_payload(terminal) is None


def active_execution(request_id=10, motion_id="forward", player=None):
    player = player or MockRobotMotionPlayer()
    core = MotionExecutorCore(player)
    state = ExecutionPublicationState()
    request = MotionRequest(request_id, motion_id, 5000)
    assert core.start_motion(motion_id, 5000) is None
    state.begin(request)
    return core, state, player


def test_parse_cancel_request():
    assert parse_cancel_request('{"request_id": 10}') == CancelRequest(10)


def test_cancel_with_matching_request_id():
    core, state, _ = active_execution(request_id=10, motion_id="shoot")
    handled = handle_cancel_request('{"request_id": 10}', core, state)
    assert handled.terminal is True
    assert handled.payload["request_id"] == 10
    assert handled.payload["motion_id"] == "shoot"
    assert handled.payload["status"] == "CANCELLED"


def test_cancel_with_mismatched_request_id_is_rejected():
    core, state, _ = active_execution(request_id=10)
    handled = handle_cancel_request('{"request_id": 11}', core, state)
    assert handled.terminal is False
    assert handled.payload["status"] == "REJECTED"
    assert handled.payload["error_code"] == "REQUEST_ID_MISMATCH"
    assert core.busy() is True


def test_cancel_when_not_running_is_rejected():
    core = MotionExecutorCore(MockRobotMotionPlayer())
    state = ExecutionPublicationState()
    handled = handle_cancel_request('{"request_id": 10}', core, state)
    assert handled.terminal is False
    assert handled.payload["status"] == "REJECTED"
    assert handled.payload["error_code"] == "NOT_RUNNING"


def test_cancelled_terminal_is_generated_only_once():
    core, state, _ = active_execution()
    first = handle_cancel_request('{"request_id": 10}', core, state)
    second = state.terminal_payload(core.terminal_result())
    assert first.payload["status"] == "CANCELLED"
    assert second is None


def test_mock_failure_payload_preserves_original_request():
    player = MockRobotMotionPlayer(
        fail_after_updates=1,
        failure_error=MotionError.COMMUNICATION_ERROR,
        failure_message="injected communication failure",
    )
    core, state, _ = active_execution(
        request_id=77, motion_id="turn_right", player=player
    )

    terminal = core.tick(10)
    payload = state.terminal_payload(terminal)
    assert payload["request_id"] == 77
    assert payload["motion_id"] == "turn_right"
    assert payload["status"] == "FAILED"
    assert payload["error_code"] == "COMMUNICATION_ERROR"
    assert isinstance(payload["error_code"], str)
    assert "injected communication failure" in payload["message"]
    assert state.terminal_payload(terminal) is None
