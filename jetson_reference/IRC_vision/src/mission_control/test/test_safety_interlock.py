"""Tests for the ROS-independent mission safety latch."""

import pytest

from mission_control.safety_interlock import CRITICAL_EXECUTOR_ERROR_CODES
from mission_control.safety_interlock import CRITICAL_LATCHED
from mission_control.safety_interlock import NORMAL
from mission_control.safety_interlock import RECOVERABLE_MOTOR_ERROR_CODES
from mission_control.safety_interlock import SafetyInterlock


def observe(interlock, error_code, *, action="STRAIGHT", command_id=10):
    return interlock.observe_executor_status(
        error_code=error_code,
        message="executor result",
        action=action,
        command_id=command_id,
        event_id=None,
    )


def test_critical_fault_latches_first_fault_until_restart():
    interlock = SafetyInterlock()
    assert interlock.snapshot.state == NORMAL
    assert not interlock.latched

    assert observe(interlock, "BACKEND_EXCEPTION")
    assert interlock.latched
    assert interlock.snapshot.state == CRITICAL_LATCHED
    assert interlock.snapshot.error_code == "BACKEND_EXCEPTION"

    assert not observe(interlock, "BUSY", action="LEFT", command_id=11)
    assert interlock.snapshot.error_code == "BACKEND_EXCEPTION"
    assert interlock.snapshot.action == "STRAIGHT"


@pytest.mark.parametrize("error_code", sorted(CRITICAL_EXECUTOR_ERROR_CODES))
def test_known_runtime_faults_are_critical(error_code):
    interlock = SafetyInterlock()
    assert observe(interlock, error_code)
    assert interlock.latched


@pytest.mark.parametrize("error_code", sorted(RECOVERABLE_MOTOR_ERROR_CODES))
def test_motor_faults_do_not_latch_mission_control(error_code):
    interlock = SafetyInterlock()
    assert not observe(interlock, error_code)
    assert not interlock.latched


@pytest.mark.parametrize(
    "error_code",
    [
        "",
        "BUSY",
        "SDK_BUSY",
        "INVALID_REQUEST",
        "INVALID_MOTION",
        "SDK_MOTION_NOT_FOUND",
        "SDK_INVALID_MOTION",
        "STALE_REQUEST",
        "TIMEOUT",
    ],
)
def test_command_local_faults_do_not_latch(error_code):
    interlock = SafetyInterlock()
    assert not observe(interlock, error_code)
    assert not interlock.latched
