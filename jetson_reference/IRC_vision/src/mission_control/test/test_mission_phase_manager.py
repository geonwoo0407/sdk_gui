"""Tests for the ROS-independent mission phase manager."""

import pytest

from mission_control.mission_phase_manager import MissionPhaseManager


def complete(manager, action, command_id, status):
    assert manager.start_special_action(action, command_id)
    running = manager.handle_motion_status(action, command_id, "RUNNING")
    assert running.handled
    return manager.handle_motion_status(action, command_id, status)


def test_default_initial_state():
    manager = MissionPhaseManager()
    assert manager.snapshot() == {
        "current_phase": "AUTO",
        "pickups_completed": 0,
        "shots_completed": 0,
        "ball_sections_processed": 0,
        "finish_enabled": False,
        "mission_complete": False,
        "post_ball_goal_transition_failed": False,
        "required_pickups": 2,
        "required_shots": 2,
        "required_ball_sections": 2,
        "active_special_action": None,
        "active_special_command_id": None,
        "active_special_running": False,
    }


@pytest.mark.parametrize(
    "phase",
    sorted(MissionPhaseManager.ALLOWED_PHASES),
)
def test_valid_phase_is_trimmed_and_normalized(phase):
    manager = MissionPhaseManager()
    assert manager.set_phase(f"  {phase.lower()}  ")
    assert manager.current_phase == phase


@pytest.mark.parametrize("phase", ["", "   "])
def test_empty_phase_is_rejected(phase):
    manager = MissionPhaseManager()
    assert not manager.set_phase(phase)
    assert manager.current_phase == "AUTO"


@pytest.mark.parametrize("phase", ["UNKNOWN", "AUTO_LOCK", "HURDLE_LOCK"])
def test_unknown_and_dynamic_lock_phases_are_rejected(phase):
    manager = MissionPhaseManager()
    assert not manager.set_phase(phase)
    assert manager.current_phase == "AUTO"


def test_matching_pickup_running_does_not_change_progress():
    manager = MissionPhaseManager(initial_phase="BALL_APPROACH")
    assert manager.start_special_action("PICKUP_NOW", 1)
    before = manager.snapshot()
    result = manager.handle_motion_status("PICKUP_NOW", 1, "RUNNING")
    after = manager.snapshot()
    assert result.handled and not result.terminal
    assert after["current_phase"] == before["current_phase"]
    assert after["pickups_completed"] == before["pickups_completed"]
    assert after["ball_sections_processed"] == before[
        "ball_sections_processed"
    ]
    assert after["active_special_running"] is True


def test_pickup_succeeded():
    manager = MissionPhaseManager(initial_phase="BALL_APPROACH")
    result = complete(manager, "PICKUP_NOW", 1, "SUCCEEDED")
    assert result.handled and result.terminal
    assert manager.pickups_completed == 1
    assert manager.ball_sections_processed == 0
    assert manager.current_phase == "POST_BALL_LINE_ALIGN"


def test_grasp_results_are_attempt_scoped_and_reset_on_retry():
    manager = MissionPhaseManager(initial_phase="BALL_APPROACH")
    assert manager.start_special_action("PICKUP_NOW", 1)
    assert manager.active_pickup_attempt == 1
    assert manager.record_active_pickup_grasp_result("GRABBED")
    assert manager.grasp_result_for_ball(1) == "GRABBED"
    manager.handle_motion_status("PICKUP_NOW", 1, "RUNNING")
    manager.handle_motion_status("PICKUP_NOW", 1, "FAILED")

    assert manager.start_special_action("PICKUP_NOW", 2)
    assert manager.grasp_result_for_ball(1) == "UNKNOWN"


def test_first_and_second_ball_grasp_results_do_not_mix():
    manager = MissionPhaseManager(initial_phase="BALL_APPROACH")
    assert manager.start_special_action("PICKUP_NOW", 1)
    assert manager.record_active_pickup_grasp_result("NOT_GRABBED")
    manager.handle_motion_status("PICKUP_NOW", 1, "RUNNING")
    manager.handle_motion_status("PICKUP_NOW", 1, "SUCCEEDED")

    assert manager.start_special_action("PICKUP_NOW", 2)
    assert manager.record_active_pickup_grasp_result("GRABBED")
    assert manager.grasp_result_for_ball(1) == "NOT_GRABBED"
    assert manager.grasp_result_for_ball(2) == "GRABBED"


def test_first_ball_line_alignment_completes_before_goal_transition():
    manager = MissionPhaseManager(initial_phase="BALL_APPROACH")
    complete(manager, "PICKUP_NOW", 1, "SUCCEEDED")

    assert manager.complete_post_ball_line_align()
    assert manager.current_phase == "POST_BALL_GOAL_TRANSITION"
    assert not manager.complete_post_ball_line_align()


def test_second_pickup_skips_first_ball_line_alignment():
    manager = MissionPhaseManager(initial_phase="BALL_APPROACH")
    manager.pickups_completed = 1

    complete(manager, "PICKUP_NOW", 1, "SUCCEEDED")

    assert manager.pickups_completed == 2
    assert manager.current_phase == "POST_BALL_GOAL_TRANSITION"


def test_post_ball_goal_transition_success_enters_goal_approach():
    manager = MissionPhaseManager(
        initial_phase="POST_BALL_GOAL_TRANSITION"
    )

    result = complete(
        manager,
        "POST_BALL_GOAL_TRANSITION",
        2,
        "SUCCEEDED",
    )

    assert result.handled and result.terminal
    assert manager.current_phase == "GOAL_APPROACH"
    assert manager.post_ball_goal_transition_failed is False


@pytest.mark.parametrize("status", ["FAILED", "TIMEOUT", "REJECTED"])
def test_post_ball_goal_transition_failure_latches_abort(status):
    manager = MissionPhaseManager(
        initial_phase="POST_BALL_GOAL_TRANSITION"
    )
    if status == "REJECTED":
        assert manager.start_special_action("POST_BALL_GOAL_TRANSITION", 2)
        result = manager.handle_motion_status(
            "POST_BALL_GOAL_TRANSITION",
            2,
            status,
        )
    else:
        result = complete(
            manager,
            "POST_BALL_GOAL_TRANSITION",
            2,
            status,
        )

    assert result.handled and result.terminal
    assert manager.current_phase == "POST_BALL_GOAL_TRANSITION"
    assert manager.post_ball_goal_transition_failed is True


@pytest.mark.parametrize(
    "status",
    ["FAILED", "TIMEOUT", "CANCELLED", "REJECTED", "UNSUPPORTED"],
)
def test_pickup_failure_does_not_complete_section(status):
    manager = MissionPhaseManager(
        required_ball_sections=2,
        initial_phase="BALL_APPROACH",
    )
    if status in {"REJECTED", "UNSUPPORTED"}:
        assert manager.start_special_action("PICKUP_NOW", 1)
        manager.handle_motion_status("PICKUP_NOW", 1, status)
    else:
        complete(manager, "PICKUP_NOW", 1, status)
    assert manager.pickups_completed == 0
    assert manager.ball_sections_processed == 0
    assert manager.finish_enabled is False
    assert manager.current_phase == "BALL_APPROACH"


def test_failed_pickup_cannot_enable_finish():
    manager = MissionPhaseManager(required_ball_sections=1)
    complete(manager, "PICKUP_NOW", 1, "FAILED")
    assert manager.finish_enabled is False
    assert manager.ball_sections_processed == 0
    assert manager.current_phase == "BALL_APPROACH"


def test_pickup_failed_or_timeout_counts_toward_failure_limit():
    manager = MissionPhaseManager(
        initial_phase="BALL_APPROACH",
        max_pickup_failures=2,
    )

    complete(manager, "PICKUP_NOW", 1, "FAILED")

    assert manager.pickup_failure_count == 1
    assert not manager.special_action_exhausted("PICKUP_NOW")

    complete(manager, "PICKUP_NOW", 2, "TIMEOUT")

    assert manager.pickup_failure_count == 2
    assert manager.special_action_exhausted("PICKUP_NOW")


def test_success_resets_special_failure_count():
    manager = MissionPhaseManager(
        initial_phase="BALL_APPROACH",
        max_pickup_failures=2,
    )

    complete(manager, "PICKUP_NOW", 1, "FAILED")
    assert manager.pickup_failure_count == 1

    complete(manager, "PICKUP_NOW", 2, "SUCCEEDED")

    assert manager.pickup_failure_count == 0
    assert not manager.special_action_exhausted("PICKUP_NOW")


@pytest.mark.parametrize(
    "status",
    ["REJECTED", "UNSUPPORTED", "CANCELLED"],
)
def test_non_execution_special_terminal_does_not_count_as_mission_failure(
    status,
):
    manager = MissionPhaseManager(
        initial_phase="BALL_APPROACH",
        max_pickup_failures=1,
    )

    complete(manager, "PICKUP_NOW", 1, status)

    assert manager.pickup_failure_count == 0
    assert not manager.special_action_exhausted("PICKUP_NOW")


@pytest.mark.parametrize(
    ("status", "expected_shots"),
    [
        ("SUCCEEDED", 1),
        ("FAILED", 0),
        ("TIMEOUT", 0),
        ("CANCELLED", 0),
    ],
)
def test_shot_terminal_updates_section_and_phase(status, expected_shots):
    manager = MissionPhaseManager(
        required_ball_sections=2,
        initial_phase="GOAL_APPROACH",
    )
    complete(manager, "SHOT", 1, status)
    assert manager.shots_completed == expected_shots
    expected_sections = 1 if status == "SUCCEEDED" else 0
    assert manager.ball_sections_processed == expected_sections
    assert manager.finish_enabled is False
    assert manager.current_phase == (
        "AUTO" if status == "SUCCEEDED" else "GOAL_APPROACH"
    )


def test_only_required_shot_enters_line_track_without_finishing():
    manager = MissionPhaseManager(
        required_ball_sections=1,
        initial_phase="GOAL_APPROACH",
    )
    complete(manager, "SHOT", 1, "SUCCEEDED")
    assert manager.shots_completed == 1
    assert manager.ball_sections_processed == 1
    assert manager.finish_enabled is True
    assert manager.current_phase == "LINE_TRACK"
    assert manager.current_phase not in {
        "FINISH",
        "WALK_TO_FINISH",
        "FINISHED",
    }
    assert manager.mission_complete is False


def test_second_required_shot_enters_line_track():
    manager = MissionPhaseManager(
        required_ball_sections=2,
        initial_phase="GOAL_APPROACH",
    )
    complete(manager, "SHOT", 1, "SUCCEEDED")
    assert manager.ball_sections_processed == 1
    assert manager.current_phase == "AUTO"

    assert manager.set_phase("GOAL_APPROACH")
    complete(manager, "SHOT", 2, "SUCCEEDED")

    assert manager.shots_completed == 2
    assert manager.ball_sections_processed == 2
    assert manager.finish_enabled is True
    assert manager.current_phase == "LINE_TRACK"
    assert manager.mission_complete is False


def test_zero_required_sections_preserves_success_phase_policy():
    manager = MissionPhaseManager(
        required_ball_sections=0,
        initial_phase="GOAL_APPROACH",
    )

    complete(manager, "SHOT", 1, "SUCCEEDED")

    assert manager.ball_sections_processed == 0
    assert manager.current_phase == "AUTO"
    assert manager.finish_enabled is True


@pytest.mark.parametrize(
    "origin_phase",
    [
        "AUTO",
        "BALL_SEARCH",
        "BALL_APPROACH",
        "GOAL_SEARCH",
        "GOAL_APPROACH",
        "HURDLE_APPROACH",
        "LINE_TRACK",
    ],
)
def test_successful_go_restores_exact_origin_without_progress(origin_phase):
    manager = MissionPhaseManager(initial_phase=origin_phase)
    before = manager.snapshot()

    complete(manager, "GO", 1, "SUCCEEDED")

    assert manager.current_phase == origin_phase
    assert manager.pickups_completed == before["pickups_completed"]
    assert manager.shots_completed == before["shots_completed"]
    assert manager.ball_sections_processed == before[
        "ball_sections_processed"
    ]
    assert manager.finish_enabled == before["finish_enabled"]
    assert manager.mission_complete == before["mission_complete"]


@pytest.mark.parametrize("invalid_origin", [None, "UNKNOWN"])
def test_successful_go_without_valid_origin_falls_back_to_auto(
    invalid_origin,
):
    manager = MissionPhaseManager(initial_phase="LINE_TRACK")
    assert manager.start_special_action("GO", 1)
    manager._active_special_origin_phase = invalid_origin
    assert manager.handle_motion_status("GO", 1, "RUNNING").handled

    result = manager.handle_motion_status("GO", 1, "SUCCEEDED")

    assert result.handled and result.terminal
    assert manager.current_phase == "AUTO"


@pytest.mark.parametrize(
    "status",
    ["FAILED", "TIMEOUT", "CANCELLED", "REJECTED", "UNSUPPORTED"],
)
def test_failed_go_returns_to_hurdle_approach(status):
    manager = MissionPhaseManager(initial_phase="GOAL_APPROACH")
    before = manager.snapshot()
    if status in {"REJECTED", "UNSUPPORTED"}:
        assert manager.start_special_action("GO", 1)
        manager.handle_motion_status("GO", 1, status)
    else:
        complete(manager, "GO", 1, status)
    assert manager.current_phase == "HURDLE_APPROACH"
    assert manager.pickups_completed == before["pickups_completed"]
    assert manager.shots_completed == before["shots_completed"]
    assert manager.ball_sections_processed == before[
        "ball_sections_processed"
    ]
    assert manager.finish_enabled == before["finish_enabled"]
    assert manager.mission_complete == before["mission_complete"]


def test_duplicate_successful_go_terminal_is_not_applied_again():
    manager = MissionPhaseManager(initial_phase="BALL_SEARCH")
    complete(manager, "GO", 1, "SUCCEEDED")
    assert manager.current_phase == "BALL_SEARCH"

    assert manager.set_phase("LINE_TRACK")
    duplicate = manager.handle_motion_status("GO", 1, "SUCCEEDED")

    assert not duplicate.handled
    assert duplicate.duplicate
    assert manager.current_phase == "LINE_TRACK"


def test_cross_finish_succeeded():
    manager = MissionPhaseManager(initial_phase="WALK_TO_FINISH")
    complete(manager, "CROSS_FINISH", 1, "SUCCEEDED")
    assert manager.mission_complete is True
    assert manager.current_phase == "FINISHED"


@pytest.mark.parametrize("status", ["FAILED", "TIMEOUT"])
def test_cross_finish_failure_or_timeout(status):
    manager = MissionPhaseManager(initial_phase="WALK_TO_FINISH")
    complete(manager, "CROSS_FINISH", 1, status)
    assert manager.mission_complete is False
    assert manager.current_phase == "WALK_TO_FINISH"


def test_stale_command_id_is_ignored():
    manager = MissionPhaseManager(initial_phase="BALL_APPROACH")
    assert manager.start_special_action("PICKUP_NOW", 2)
    result = manager.handle_motion_status("PICKUP_NOW", 1, "RUNNING")
    assert not result.handled
    assert manager.active_special_command_id == 2
    assert manager.current_phase == "BALL_APPROACH"


def test_wrong_action_is_ignored():
    manager = MissionPhaseManager(initial_phase="BALL_APPROACH")
    assert manager.start_special_action("PICKUP_NOW", 1)
    result = manager.handle_motion_status("SHOT", 1, "RUNNING")
    assert not result.handled
    assert manager.active_special_action == "PICKUP_NOW"


def test_duplicate_terminal_is_ignored():
    manager = MissionPhaseManager(initial_phase="BALL_APPROACH")
    complete(manager, "PICKUP_NOW", 1, "SUCCEEDED")
    duplicate = manager.handle_motion_status(
        "PICKUP_NOW", 1, "SUCCEEDED"
    )
    assert not duplicate.handled
    assert duplicate.duplicate
    assert manager.pickups_completed == 1
    assert manager.current_phase == "POST_BALL_LINE_ALIGN"


def test_duplicate_successful_shot_does_not_increment_twice():
    manager = MissionPhaseManager(
        required_shots=2,
        required_ball_sections=2,
        initial_phase="GOAL_APPROACH",
    )
    complete(manager, "SHOT", 10, "SUCCEEDED")

    duplicate = manager.handle_motion_status("SHOT", 10, "SUCCEEDED")

    assert not duplicate.handled
    assert duplicate.duplicate
    assert manager.shots_completed == 1
    assert manager.ball_sections_processed == 1
    assert manager.current_phase == "AUTO"


def test_mismatched_shot_command_does_not_change_progress_or_phase():
    manager = MissionPhaseManager(initial_phase="GOAL_APPROACH")
    assert manager.start_special_action("SHOT", 20)
    before = manager.snapshot()

    result = manager.handle_motion_status("SHOT", 21, "RUNNING")

    assert not result.handled
    assert manager.snapshot() == before


def test_new_special_action_is_rejected_while_one_is_active():
    manager = MissionPhaseManager()
    assert manager.start_special_action("PICKUP_NOW", 1)
    assert manager.start_special_action("PICKUP_NOW", 1)
    assert not manager.start_special_action("SHOT", 2)
    assert manager.active_special_action == "PICKUP_NOW"
    assert manager.active_special_command_id == 1


def test_counters_are_capped_at_required_values():
    manager = MissionPhaseManager(
        required_pickups=1,
        required_shots=1,
        required_ball_sections=1,
    )
    complete(manager, "PICKUP_NOW", 1, "SUCCEEDED")
    complete(manager, "PICKUP_NOW", 2, "SUCCEEDED")
    complete(manager, "SHOT", 3, "SUCCEEDED")
    complete(manager, "SHOT", 4, "SUCCEEDED")
    assert manager.pickups_completed == 1
    assert manager.shots_completed == 1
    assert manager.ball_sections_processed == 1


def test_snapshot_is_copy_safe():
    manager = MissionPhaseManager()
    snapshot = manager.snapshot()
    snapshot["current_phase"] = "FINISHED"
    snapshot["pickups_completed"] = 99
    assert manager.current_phase == "AUTO"
    assert manager.pickups_completed == 0


def test_terminal_before_running_is_ignored_and_active_remains():
    manager = MissionPhaseManager(initial_phase="BALL_APPROACH")
    assert manager.start_special_action("PICKUP_NOW", 1)
    result = manager.handle_motion_status(
        "PICKUP_NOW", 1, "SUCCEEDED"
    )
    assert not result.handled
    assert result.reason == "terminal_before_running"
    assert manager.active_special_command_id == 1
    assert manager.pickups_completed == 0


def test_cancelled_before_running_is_ignored_and_active_remains():
    manager = MissionPhaseManager(initial_phase="BALL_APPROACH")
    assert manager.start_special_action("PICKUP_NOW", 1)

    result = manager.handle_motion_status(
        "PICKUP_NOW", 1, "CANCELLED"
    )

    assert not result.handled
    assert result.reason == "terminal_before_running"
    assert manager.active_special_action == "PICKUP_NOW"
    assert manager.active_special_command_id == 1
    assert manager.active_special_running is False
    assert manager.current_phase == "BALL_APPROACH"


@pytest.mark.parametrize(
    ("initial_phase", "action", "expected_phase"),
    [
        ("BALL_APPROACH", "PICKUP_NOW", "BALL_APPROACH"),
        ("GOAL_APPROACH", "SHOT", "GOAL_APPROACH"),
        ("HURDLE_APPROACH", "GO", "HURDLE_APPROACH"),
        ("GOAL_APPROACH", "GO", "HURDLE_APPROACH"),
    ],
)
def test_matching_rejected_without_running_uses_failure_phase(
    initial_phase,
    action,
    expected_phase,
):
    manager = MissionPhaseManager(initial_phase=initial_phase)
    assert manager.start_special_action(action, 1)

    result = manager.handle_motion_status(action, 1, "REJECTED")

    assert result.handled and result.terminal
    assert manager.current_phase == expected_phase
    assert manager.active_special_action is None
    assert manager.active_special_command_id is None
    assert manager.active_special_running is False
    assert manager.pickups_completed == 0
    assert manager.shots_completed == 0


def test_duplicate_rejected_does_not_apply_failure_twice():
    manager = MissionPhaseManager(
        required_ball_sections=2,
        initial_phase="BALL_APPROACH",
    )
    assert manager.start_special_action("PICKUP_NOW", 1)
    first = manager.handle_motion_status("PICKUP_NOW", 1, "REJECTED")
    duplicate = manager.handle_motion_status(
        "PICKUP_NOW", 1, "REJECTED"
    )

    assert first.handled
    assert not duplicate.handled
    assert duplicate.duplicate
    assert manager.ball_sections_processed == 0
    assert manager.current_phase == "BALL_APPROACH"


@pytest.mark.parametrize(
    "action",
    [
        "STRAIGHT",
        "LEFT",
        "RIGHT",
        "WAIT",
    ],
)
def test_general_actions_cannot_be_registered(action):
    manager = MissionPhaseManager()
    assert not manager.start_special_action(action, 1)


def test_required_counts_are_clamped_to_zero():
    manager = MissionPhaseManager(
        required_pickups=-1,
        required_shots=0,
        required_ball_sections=-3,
    )
    assert manager.required_pickups == 0
    assert manager.required_shots == 0
    assert manager.required_ball_sections == 0
    assert manager.finish_enabled is True


@pytest.mark.parametrize("command_id", [0, -1, True, 1.5, "1"])
def test_invalid_command_ids_are_rejected(command_id):
    manager = MissionPhaseManager()
    assert not manager.start_special_action("SHOT", command_id)
