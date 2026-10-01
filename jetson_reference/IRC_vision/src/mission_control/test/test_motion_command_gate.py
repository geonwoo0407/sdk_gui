import pytest

from mission_control.motion_command_gate import GeneralMotionCommandGate


def gate_with_vision():
    gate = GeneralMotionCommandGate()
    gate.on_new_vision_input()
    return gate


def test_first_left_publishes_once_and_repeated_left_is_blocked():
    gate = gate_with_vision()
    assert gate.can_publish("LEFT")
    gate.on_command_published("LEFT")
    gate.on_new_vision_input()
    assert not gate.can_publish("LEFT")


def test_right_is_blocked_while_left_is_running():
    gate = gate_with_vision()
    gate.on_command_published("LEFT")
    gate.on_motion_status("LEFT", "RUNNING")
    assert not gate.can_publish("RIGHT")


def test_running_keeps_lock_with_same_action():
    gate = gate_with_vision()
    gate.on_command_published("LEFT")
    transition = gate.on_motion_status("LEFT", "RUNNING")
    assert transition.matched
    assert not transition.released
    assert gate.locked


def test_success_requires_new_vision_before_republishing():
    gate = gate_with_vision()
    gate.on_command_published("LEFT")
    gate.on_motion_status("LEFT", "RUNNING")
    assert gate.on_motion_status("LEFT", "SUCCEEDED").released
    assert not gate.can_publish("LEFT")
    gate.on_new_vision_input()
    assert gate.can_publish("LEFT")


@pytest.mark.parametrize(
    "action",
    [
        "GOAL_CAMERA_90_FORWARD",
        "GOAL_CAMERA_90_FORWARD_1",
        "GOAL_CAMERA_90_FORWARD_2",
        "GOAL_CAMERA_90_FORWARD_4",
        "BALL_FINE_FORWARD_8",
    ],
)
def test_context_forward_requires_fresh_vision_after_success(action):
    gate = gate_with_vision()
    gate.on_command_published(action, command_id=41)
    gate.on_motion_status(
        action,
        "RUNNING",
        command_id=41,
    )
    assert gate.on_motion_status(
        action,
        "SUCCEEDED",
        command_id=41,
    ).released

    assert not gate.can_publish(action)
    gate.on_new_vision_input()
    assert gate.can_publish(action)


@pytest.mark.parametrize(
    "action",
    ["POST_BALL_LINE_TURN_RIGHT_3", "GOAL_CAMERA90_TURN_RIGHT_3"],
)
def test_closed_loop_alignment_actions_require_new_vision(action):
    gate = gate_with_vision()
    gate.on_command_published(action, command_id=42)
    gate.on_motion_status(action, "RUNNING", command_id=42)
    assert gate.on_motion_status(
        action,
        "SUCCEEDED",
        command_id=42,
    ).released

    assert not gate.can_publish(action)
    gate.on_new_vision_input()
    assert gate.can_publish(action)


def test_terminal_general_motion_requires_fresh_vision_globally():
    gate = gate_with_vision()
    gate.on_command_published("STRAIGHT", command_id=40)

    assert gate.on_motion_status(
        "STRAIGHT",
        "RUNNING",
        command_id=40,
    ).matched

    assert gate.on_motion_status(
        "STRAIGHT",
        "SUCCEEDED",
        command_id=40,
    ).released

    assert not gate.has_required_fresh_vision()

    gate.on_new_vision_input()

    assert gate.has_required_fresh_vision()


def test_unknown_rejection_does_not_retry_same_action_forever():
    gate = gate_with_vision()
    gate.on_command_published("LEFT")
    assert gate.on_motion_status("LEFT", "REJECTED").released
    for _ in range(3):
        gate.on_new_vision_input()
        assert not gate.can_publish("LEFT")
    assert gate.can_publish("RIGHT")


@pytest.mark.parametrize(
    "error_code",
    [
        "REJECTED_BUSY",
        "BUSY",
        "SDK_BUSY",
        "STARTUP_POSE_GATE_LOCKED",
    ],
)
def test_transient_rejection_retries_only_after_new_vision(error_code):
    gate = gate_with_vision()
    gate.on_command_published("STRAIGHT")
    assert gate.on_motion_status(
        "STRAIGHT",
        "REJECTED",
        error_code=error_code,
    ).released
    assert not gate.can_publish("STRAIGHT")
    gate.on_new_vision_input()
    assert gate.can_publish("STRAIGHT")


def test_permanent_rejection_rearms_only_after_action_change():
    gate = gate_with_vision()
    gate.on_command_published("STRAIGHT")
    gate.on_motion_status(
        "STRAIGHT",
        "REJECTED",
        error_code="INVALID_MOTION",
    )
    for _ in range(3):
        gate.on_new_vision_input()
        assert not gate.can_publish("STRAIGHT")

    assert gate.can_publish("LEFT")
    assert gate.can_publish("STRAIGHT")


@pytest.mark.parametrize(
    "error_code",
    ["BUSY", "SDK_BUSY", "STARTUP_POSE_GATE_LOCKED"],
)
def test_transient_retries_are_bounded_and_rearm_on_action_change(error_code):
    gate = GeneralMotionCommandGate(max_transient_retries=2)
    gate.on_new_vision_input()

    for _ in range(3):
        assert gate.can_publish("STRAIGHT")
        gate.on_command_published("STRAIGHT")
        assert gate.on_motion_status(
            "STRAIGHT",
            "REJECTED",
            error_code=error_code,
        ).released
        gate.on_new_vision_input()

    assert not gate.can_publish("STRAIGHT")
    assert gate.can_publish("RIGHT")
    assert gate.can_publish("STRAIGHT")


@pytest.mark.parametrize("error_code", [None, "", "UNRECOGNIZED"])
def test_unknown_rejection_code_fails_closed(error_code):
    gate = gate_with_vision()
    gate.on_command_published("STRAIGHT")
    gate.on_motion_status(
        "STRAIGHT",
        "REJECTED",
        error_code=error_code,
    )
    gate.on_new_vision_input()
    assert not gate.can_publish("STRAIGHT")


def test_old_success_cannot_release_new_lock_without_running():
    gate = gate_with_vision()
    gate.on_command_published("LEFT")
    transition = gate.on_motion_status("LEFT", "SUCCEEDED")
    assert not transition.matched
    assert not transition.released
    assert gate.locked


def test_mismatched_terminal_cannot_release_lock():
    gate = gate_with_vision()
    gate.on_command_published("RIGHT")
    gate.on_motion_status("RIGHT", "RUNNING")
    assert not gate.on_motion_status("LEFT", "SUCCEEDED").matched
    assert gate.locked


def test_different_command_id_cannot_mark_running_or_release_lock():
    gate = gate_with_vision()
    gate.on_command_published("LEFT", command_id=20)
    assert not gate.on_motion_status(
        "LEFT", "RUNNING", command_id=19
    ).matched
    assert not gate.on_motion_status(
        "LEFT", "SUCCEEDED", command_id=19
    ).matched
    assert gate.locked
    assert not gate.running_seen


def test_same_command_id_releases_only_after_running():
    gate = gate_with_vision()
    gate.on_command_published("LEFT", command_id=20)
    assert gate.on_motion_status(
        "LEFT", "RUNNING", command_id=20
    ).matched
    assert gate.on_motion_status(
        "LEFT", "SUCCEEDED", command_id=20
    ).released


def test_missing_status_command_id_does_not_release_current_request():
    gate = gate_with_vision()
    gate.on_command_published("LEFT", command_id=20)
    assert not gate.on_motion_status("LEFT", "RUNNING").matched
    assert not gate.on_motion_status("LEFT", "SUCCEEDED").released
    assert gate.locked


@pytest.mark.parametrize(
    ("command_action", "status_action"),
    [
        ("LEFT", "LEFT"),
        ("RIGHT", "RIGHT"),
        ("STRAIGHT", "STRAIGHT"),
    ],
)
def test_identical_actions_correlate(command_action, status_action):
    gate = gate_with_vision()
    assert gate.can_publish(command_action)
    gate.on_command_published(command_action)
    assert gate.on_motion_status(status_action, "RUNNING").matched


@pytest.mark.parametrize(
    "action",
    ["WAIT", "PICKUP_NOW", "STRAIGHT_0"],
)
def test_non_general_action_is_not_managed_as_execution(action):
    assert not gate_with_vision().can_publish(action)


@pytest.mark.parametrize(
    "recovery_action",
    [
        *[
            f"RECOVER_{line_side}_TURN_LEFT_{suffix}"
            for line_side in ("LEFT", "RIGHT")
            for suffix in (2, 4, 6, 8, 10, 13)
        ],
        *[
            f"RECOVER_{line_side}_TURN_RIGHT_{suffix}"
            for line_side in ("LEFT", "RIGHT")
            for suffix in (4, 6, 8, 10, 12, 15)
        ],
    ],
)
def test_recovery_turn_uses_general_motion_lock_and_fresh_vision(
    recovery_action,
):
    gate = gate_with_vision()
    assert gate.can_publish(recovery_action)

    gate.on_command_published(recovery_action, command_id=50)
    for blocked in (
        recovery_action,
        "STRAIGHT",
        "LEFT",
        "RIGHT",
    ):
        assert not gate.can_publish(blocked)

    assert gate.on_motion_status(
        recovery_action, "RUNNING", command_id=50
    ).matched
    assert gate.active_action == recovery_action
    assert gate.on_motion_status(
        recovery_action, "SUCCEEDED", command_id=50
    ).released
    assert not gate.can_publish("STRAIGHT")

    gate.on_new_vision_input()
    assert gate.can_publish("STRAIGHT")


@pytest.mark.parametrize(
    "action",
    [
        "STRAIGHT",
        "STRAIGHT_1",
        "STRAIGHT_2",
        "STRAIGHT_3",
        "STRAIGHT_4",
        "STRAIGHT_5",
        "APPROACH",
        "SLOW_APPROACH",
        "FINE_FORWARD_STEP",
        "APPROACH_GOAL",
        "APPROACH_HURDLE",
        "TURN_LEFT",
        "TURN_RIGHT",
        "ALIGN_LEFT",
        "ALIGN_RIGHT",
        "RETREAT_GOAL",
    ],
)
def test_all_executable_or_bridge_rejected_actions_use_the_gate(action):
    gate = gate_with_vision()

    assert gate.can_publish(action)
    gate.on_command_published(action, command_id=30)
    assert not gate.can_publish(action)


def test_unsupported_status_releases_without_running_and_blocks_retry():
    gate = gate_with_vision()
    gate.on_command_published("ALIGN_LEFT", command_id=31)

    transition = gate.on_motion_status(
        "ALIGN_LEFT",
        "UNSUPPORTED",
        command_id=31,
    )

    assert transition.released
    gate.on_new_vision_input()
    assert not gate.can_publish("ALIGN_LEFT")


@pytest.mark.parametrize("status", ["FAILED", "TIMEOUT", "CANCELLED"])
def test_failed_or_cancelled_action_is_not_retried_forever(status):
    gate = gate_with_vision()
    gate.on_command_published("APPROACH", command_id=32)
    assert gate.on_motion_status(
        "APPROACH", "RUNNING", command_id=32
    ).matched
    assert gate.on_motion_status(
        "APPROACH", status, command_id=32
    ).released

    for _ in range(3):
        gate.on_new_vision_input()
        assert not gate.can_publish("APPROACH")

    assert gate.can_publish("STRAIGHT")
