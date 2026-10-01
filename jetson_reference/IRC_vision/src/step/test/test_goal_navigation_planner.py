"""Unit tests for hardware-independent goal navigation decisions."""

from step.goal_navigation_planner import GoalNavigationPlanner


def goal_info(**overrides):
    """Create one valid backboard-based goal sample."""
    sample = {
        "detected": True,
        "confidence": 0.9,
        "depth_valid": True,
        "depth_m": 0.5,
        "ground_distance_m": 0.5,
        "distance_m": 0.5,
        "bearing_deg": 0.0,
        "offset_x_norm": 0.0,
    }
    sample.update(overrides)
    if "ground_distance_m" not in overrides and "depth_m" in overrides:
        sample["ground_distance_m"] = overrides["depth_m"]
    return sample


def test_goal_outside_50cm_control_range_waits():
    planner = GoalNavigationPlanner()

    command = planner.plan(goal_info(depth_m=0.51))

    assert command.valid is False
    assert command.action == "WAIT"
    assert command.reason == "goal_outside_control_range"


def test_goal_without_valid_depth_still_waits():
    planner = GoalNavigationPlanner()
    command = planner.plan(goal_info(depth_valid=False, depth_m=None))
    assert command.valid is False
    assert command.action == "WAIT"
    assert command.reason == "missing_valid_goal_depth"


def test_centered_goal_at_50cm_approaches():
    planner = GoalNavigationPlanner()

    command = planner.plan(goal_info(depth_m=0.50))

    assert command.valid is True
    assert command.action == "STRAIGHT_3"
    assert command.to_dict()["approach_level"] == 3


def test_centered_goal_at_25cm_requests_score_motion():
    planner = GoalNavigationPlanner()

    command = planner.plan(goal_info(depth_m=0.25))

    assert command.action == "SHOT"
    assert command.sdk_motion_requested is True


def test_goal_decision_uses_raw_depth_not_ground_distance():
    planner = GoalNavigationPlanner()

    command = planner.plan(
        goal_info(depth_m=0.25, ground_distance_m=0.80)
    )

    assert command.action == "SHOT"
    assert command.depth_m == 0.25


def test_goal_waits_until_analyzer_confirms_score_condition():
    planner = GoalNavigationPlanner()

    command = planner.plan(goal_info(depth_m=0.25, score_now=False))

    assert command.action == "WAIT_SCORE_CONFIRMATION"
    assert command.sdk_motion_requested is False


def test_misaligned_goal_at_25cm_aligns_before_scoring():
    planner = GoalNavigationPlanner()

    command = planner.plan(
        goal_info(depth_m=0.25, offset_x_norm=-0.2)
    )

    assert command.action == "TURN_LEFT"
    assert command.sdk_motion_requested is False


def test_centered_goal_below_scoring_range_requests_unmapped_retreat():
    planner = GoalNavigationPlanner()
    depth = (
        planner.config.score_target_depth_m
        - planner.config.score_depth_tolerance_m
        - 0.001
    )
    command = planner.plan(
        goal_info(depth_m=depth, distance_m=0.18, score_now=False)
    )
    assert command.valid is True
    assert command.action == "RETREAT_GOAL"
    assert command.sdk_motion_requested is False
    assert command.depth_m == depth


def test_scoring_lower_boundary_is_inclusive_before_retreat():
    planner = GoalNavigationPlanner()
    boundary = (
        planner.config.score_target_depth_m
        - planner.config.score_depth_tolerance_m
    )
    at_boundary = planner.plan(goal_info(depth_m=boundary, score_now=False))
    below_boundary = planner.plan(
        goal_info(depth_m=boundary - 0.001, score_now=False)
    )
    assert at_boundary.action == "WAIT_SCORE_CONFIRMATION"
    assert below_boundary.action == "RETREAT_GOAL"
