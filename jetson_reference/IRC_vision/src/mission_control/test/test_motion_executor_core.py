import sys
from enum import Enum, auto
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "mission_control"))

from motion_executor_core import ExecutorState, MotionExecutorCore  # noqa: E402


class StartResult(Enum):
    ACCEPTED = auto()
    REJECTED_BUSY = auto()
    MOTION_NOT_FOUND = auto()
    HARDWARE_NOT_READY = auto()
    INVALID_MOTION = auto()


class CancelResult(Enum):
    CANCELLED = auto()
    NOT_RUNNING = auto()
    HARDWARE_NOT_READY = auto()
    HOLD_FAILED = auto()


class MotionStatus(Enum):
    IDLE = auto()
    RUNNING = auto()
    SETTLING = auto()
    SUCCEEDED = auto()
    CANCELLED = auto()
    FAILED = auto()


class MotionError(Enum):
    NONE = auto()
    JSON_ERROR = auto()
    HARDWARE_NOT_READY = auto()
    COMMUNICATION_ERROR = auto()
    POSITION_TIMEOUT = auto()


class MockPlayer:

    def __init__(self):
        self.start_result = StartResult.ACCEPTED
        self.cancel_result = CancelResult.CANCELLED
        self.ready = True
        self.motion_status = MotionStatus.IDLE
        self.motion_result = MotionStatus.IDLE
        self.error = ""
        self.update_count = 0
        self.cancel_count = 0
        self.start_calls = []
        self.current_motion = ""

    def start(self, motion_id):
        self.start_calls.append(motion_id)
        if self.start_result is StartResult.ACCEPTED:
            self.current_motion = motion_id
            self.motion_status = MotionStatus.RUNNING
        return self.start_result

    def update(self):
        self.update_count += 1

    def running(self):
        return self.motion_status in (MotionStatus.RUNNING, MotionStatus.SETTLING)

    def status(self):
        return self.motion_status

    def succeeded(self):
        return self.motion_status is MotionStatus.SUCCEEDED

    def result(self):
        return self.motion_result

    def lastError(self):
        return self.error

    def cancel(self):
        self.cancel_count += 1
        return self.cancel_result

    def hardwareReady(self):
        return self.ready

    def currentMotion(self):
        return self.current_motion


def accepted_executor():
    player = MockPlayer()
    core = MotionExecutorCore(player)
    assert core.start_motion("forward", 1000) is None
    return core, player


@pytest.mark.parametrize(
    "motion_id",
    [
        "forward",
        "forward_short",
        "turn_left",
        "turn_right",
        "adjust_left",
        "adjust_right",
        "backward",
        "pick_ball",
        "shoot",
        "hurdle",
    ],
)
def test_supported_adapter_motion_id_is_passed_to_player_once(motion_id):
    player = MockPlayer()
    core = MotionExecutorCore(player)

    assert core.start_motion(motion_id, 1000) is None
    assert player.start_calls == [motion_id]


def test_running_settling_succeeded():
    core, player = accepted_executor()
    assert core.state() is ExecutorState.STARTING

    assert core.tick(10) is None
    assert core.state() is ExecutorState.RUNNING
    player.motion_status = MotionStatus.SETTLING
    assert core.tick(10) is None
    assert core.state() is ExecutorState.SETTLING
    player.motion_status = MotionStatus.SUCCEEDED
    result = core.tick(10)

    assert result.success is True
    assert result.final_status is ExecutorState.SUCCEEDED
    assert result.motion_id == "forward"
    assert result.error_code == "NONE"
    assert isinstance(result.error_code, str)


def test_start_result_rejected_busy():
    player = MockPlayer()
    player.start_result = StartResult.REJECTED_BUSY
    result = MotionExecutorCore(player).start_motion("forward", 1000)
    assert result.error_code == "REJECTED_BUSY"
    assert isinstance(result.error_code, str)


def test_start_result_motion_not_found():
    player = MockPlayer()
    player.start_result = StartResult.MOTION_NOT_FOUND
    result = MotionExecutorCore(player).start_motion("forward", 1000)
    assert result.error_code == "MOTION_NOT_FOUND"
    assert isinstance(result.error_code, str)


def test_hardware_not_ready_skips_player_start():
    player = MockPlayer()
    player.ready = False
    result = MotionExecutorCore(player).start_motion("forward", 1000)
    assert result.error_code == "HARDWARE_NOT_READY"
    assert player.start_calls == []


def test_start_result_hardware_not_ready():
    player = MockPlayer()
    player.start_result = StartResult.HARDWARE_NOT_READY
    result = MotionExecutorCore(player).start_motion("forward", 1000)
    assert result.error_code == "HARDWARE_NOT_READY"
    assert player.start_calls == ["forward"]


def test_start_result_invalid_motion():
    player = MockPlayer()
    player.start_result = StartResult.INVALID_MOTION
    result = MotionExecutorCore(player).start_motion("forward", 1000)
    assert result.error_code == "INVALID_MOTION"


def test_update_failure_preserves_communication_error_and_result():
    core, player = accepted_executor()
    player.motion_status = MotionStatus.FAILED
    player.motion_result = MotionError.COMMUNICATION_ERROR
    player.error = "motor communication failed"
    result = core.tick(10)
    assert result.error_code == "COMMUNICATION_ERROR"
    assert isinstance(result.error_code, str)
    assert "result=COMMUNICATION_ERROR" in result.message
    assert "motor communication failed" in result.message


def test_update_failure_preserves_position_timeout():
    core, player = accepted_executor()
    player.motion_status = MotionStatus.FAILED
    player.motion_result = MotionError.POSITION_TIMEOUT
    player.error = "final pose was not reached"
    result = core.tick(10)
    assert result.error_code == "POSITION_TIMEOUT"
    assert isinstance(result.error_code, str)


def test_executor_timeout_calls_cancel():
    core, player = accepted_executor()
    result = core.tick(1000)
    assert result.final_status is ExecutorState.TIMEOUT
    assert result.error_code == "POSITION_TIMEOUT"
    assert isinstance(result.error_code, str)
    assert player.cancel_count == 1
    assert player.update_count == 0


def test_cancel_success():
    core, _ = accepted_executor()
    result = core.cancel()
    assert result.final_status is ExecutorState.CANCELLED
    assert result.success is False


def test_cancel_hold_failed():
    core, player = accepted_executor()
    player.cancel_result = CancelResult.HOLD_FAILED
    result = core.cancel()
    assert result.final_status is ExecutorState.FAILED
    assert result.error_code == "CANCEL_FAILED"
    assert isinstance(result.error_code, str)


def test_unsupported_motion_id_skips_sdk():
    player = MockPlayer()
    result = MotionExecutorCore(player).start_motion("fly", 1000)
    assert result.error_code == "INVALID_MOTION"
    assert player.start_calls == []


def test_terminal_result_is_created_only_once():
    core, player = accepted_executor()
    player.motion_status = MotionStatus.SUCCEEDED
    first = core.tick(10)
    second = core.tick(10)
    assert second is first
    assert player.update_count == 1


def test_new_start_is_rejected_while_running():
    core, player = accepted_executor()
    result = core.start_motion("turn_left", 1000)
    assert result.error_code == "REJECTED_BUSY"
    assert isinstance(result.error_code, str)
    assert core.active_motion() == "forward"
    assert player.start_calls == ["forward"]


def test_reset_allows_another_motion():
    core, player = accepted_executor()
    player.motion_status = MotionStatus.SUCCEEDED
    assert core.tick(10).success
    assert core.reset() is True
    assert core.state() is ExecutorState.IDLE
    assert core.terminal_result() is None
    assert core.start_motion("turn_left", 1000) is None
    assert core.active_motion() == "turn_left"


def test_reset_is_rejected_while_running():
    core, _ = accepted_executor()
    assert core.reset() is False
    assert core.state() is ExecutorState.STARTING
    assert core.active_motion() == "forward"


def test_non_positive_timeout_is_rejected():
    for timeout_ms in (0, -1):
        player = MockPlayer()
        result = MotionExecutorCore(player).start_motion("forward", timeout_ms)
        assert result.error_code == "INVALID_MOTION"
        assert player.start_calls == []
