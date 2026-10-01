import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "mission_control"))

from mock_motion_player import (  # noqa: E402
    CancelResult,
    MockRobotMotionPlayer,
    MotionError,
    MotionStatus,
    StartResult,
)


def test_running_settling_succeeded():
    player = MockRobotMotionPlayer()
    assert player.start("forward") is StartResult.ACCEPTED

    for _ in range(3):
        player.update()
        assert player.status() is MotionStatus.RUNNING
    for _ in range(2):
        player.update()
        assert player.status() is MotionStatus.SETTLING

    player.update()
    assert player.status() is MotionStatus.SUCCEEDED
    assert player.succeeded() is True
    assert player.running() is False
    assert player.result() is MotionError.NONE


def test_start_is_rejected_while_running():
    player = MockRobotMotionPlayer()
    assert player.start("forward") is StartResult.ACCEPTED
    assert player.start("turn_left") is StartResult.REJECTED_BUSY
    assert player.currentMotion() == "forward"


def test_cancel_success():
    player = MockRobotMotionPlayer()
    assert player.start("forward") is StartResult.ACCEPTED
    assert player.cancel() is CancelResult.CANCELLED
    assert player.status() is MotionStatus.CANCELLED
    assert player.running() is False


def test_unsupported_motion_is_rejected():
    player = MockRobotMotionPlayer()
    assert player.start("fly") is StartResult.INVALID_MOTION
    assert player.status() is MotionStatus.IDLE


def test_hardware_not_ready():
    player = MockRobotMotionPlayer(hardware_ready=False)
    assert player.hardwareReady() is False
    assert player.start("forward") is StartResult.HARDWARE_NOT_READY
    assert player.result() is MotionError.HARDWARE_NOT_READY


def test_injected_failure_does_not_change_to_succeeded():
    player = MockRobotMotionPlayer(
        fail_after_updates=2,
        failure_error=MotionError.COMMUNICATION_ERROR,
        failure_message="test communication failure",
    )
    assert player.start("forward") is StartResult.ACCEPTED

    player.update()
    assert player.status() is MotionStatus.RUNNING
    player.update()
    assert player.status() is MotionStatus.FAILED
    assert player.update_count == 2
    assert player.result() is MotionError.COMMUNICATION_ERROR
    assert player.lastError() == "test communication failure"

    for _ in range(10):
        player.update()
    assert player.status() is MotionStatus.FAILED
    assert player.succeeded() is False
    assert player.update_count == 2
