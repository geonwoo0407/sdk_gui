"""Unit tests for hardware-independent line navigation decisions."""

import pytest

from step.line_navigation_planner import LineNavigationPlanner
from step.line_navigation_planner import NavigationConfig
from step.line_navigation_planner import _recovery_turn_action_suffix


def line_info(**overrides):
    """Create a valid line analysis sample with optional changes."""
    sample = {
        "detected": True,
        "filtered_heading_error_deg": 0.0,
        "filtered_lateral_offset_norm": 0.0,
        "turn_angle_deg": 0.0,
        "turn_consistency": 0.9,
        "heading_quality": 0.9,
        "geometry_quality": 0.8,
        "detection_quality": 0.85,
    }
    sample.update(overrides)
    return sample


def test_straight_command_contains_speed_and_distance():
    planner = LineNavigationPlanner()

    command = planner.plan(line_info(), 0.1)

    assert command.valid is True
    assert command.motion == "STRAIGHT"
    assert command.linear_speed_mps > 0.0
    assert command.travel_distance_m == pytest.approx(
        command.linear_speed_mps * command.command_duration_sec
    )


@pytest.mark.parametrize("reported_depth", [0.10, 0.50, 1.00, 3.00])
def test_line_straight_action_does_not_use_distance_buckets(reported_depth):
    planner = LineNavigationPlanner()

    command = planner.plan(
        line_info(depth_m=reported_depth, distance_m=reported_depth),
        0.1,
    )

    assert command.motion == "STRAIGHT"


@pytest.mark.parametrize(
    ("heading", "expected"),
    [(14.0, "RIGHT"), (-14.0, "LEFT")],
)
def test_heading_sign_selects_turn_direction(heading, expected):
    planner = LineNavigationPlanner()

    for _ in range(3):
        command = planner.plan(
            line_info(
                filtered_heading_error_deg=heading,
                turn_angle_deg=heading * 2.0,
                turn_consistency=0.9,
            ),
            0.1,
        )

    assert command.motion == expected


@pytest.mark.parametrize("heading", [-9.999, 0.0, 9.999])
def test_wider_straight_deadband(heading):
    planner = LineNavigationPlanner()

    command = planner.plan(
        line_info(filtered_heading_error_deg=heading),
        0.1,
    )

    assert command.motion == "STRAIGHT"


def test_far_curve_slows_down_without_starting_turn_early():
    planner = LineNavigationPlanner(
        NavigationConfig(direction_confirmation_frames=1)
    )

    command = planner.plan(
        line_info(
            filtered_lateral_offset_norm=0.1,
            turn_angle_deg=80.0,
        ),
        0.1,
    )

    assert command.motion == "STRAIGHT"
    assert command.reason == "turn_approach_pending"
    assert command.steering_error_deg > 12.0
    assert command.linear_speed_mps < planner.config.max_linear_speed_mps


def test_confirmed_corner_does_not_override_normal_line_tracking():
    planner = LineNavigationPlanner(
        NavigationConfig(direction_confirmation_frames=1)
    )

    command = planner.plan(
        line_info(
            filtered_heading_error_deg=0.0,
            filtered_lateral_offset_norm=0.0,
            turn_angle_deg=80.0,
            turn_consistency=0.95,
            corner_preview_confirmed=True,
            corner_direction="RIGHT",
            corner_start_distance_m=0.82,
            corner_remaining_forward_m=0.67,
            corner_straight_motion_count=13,
        ),
        0.1,
    )

    payload = command.to_dict()
    assert command.motion == "STRAIGHT"
    assert command.reason == "line_tracking"
    assert payload["corner_prepare"] is False
    assert payload["corner_direction"] is None
    assert payload["corner_start_distance_m"] is None
    assert payload["corner_straight_motion_count"] is None


def test_confirmed_corner_does_not_create_discrete_approach_motion():
    planner = LineNavigationPlanner(
        NavigationConfig(direction_confirmation_frames=1)
    )

    command = planner.plan(
        line_info(
            corner_preview_confirmed=True,
            corner_direction="LEFT",
            corner_start_distance_m=0.68,
            corner_remaining_forward_m=0.53,
        ),
        0.1,
    )

    payload = command.to_dict()
    assert command.motion == "STRAIGHT"
    assert command.reason == "line_tracking"
    assert payload["approach_level"] is None
    assert payload["approach_target_distance_m"] is None


def test_confirmed_corner_remaining_distance_does_not_block_line_turn():
    planner = LineNavigationPlanner(
        NavigationConfig(direction_confirmation_frames=1)
    )

    command = planner.plan(
        line_info(
            filtered_heading_error_deg=-20.0,
            filtered_lateral_offset_norm=0.0,
            turn_angle_deg=-80.0,
            turn_consistency=0.95,
            corner_preview_confirmed=True,
            corner_direction="LEFT",
            corner_start_distance_m=0.82,
            corner_remaining_forward_m=0.67,
            corner_straight_motion_count=13,
        ),
        0.1,
    )

    assert command.motion == "LEFT"
    assert command.reason == "line_tracking"


def test_confirmed_corner_allows_left_turn_at_turning_distance():
    planner = LineNavigationPlanner(
        NavigationConfig(direction_confirmation_frames=1)
    )

    command = planner.plan(
        line_info(
            filtered_heading_error_deg=-20.0,
            filtered_lateral_offset_norm=0.0,
            turn_angle_deg=-80.0,
            turn_consistency=0.95,
            corner_preview_confirmed=True,
            corner_direction="LEFT",
            corner_start_distance_m=0.14,
            corner_remaining_forward_m=0.0,
            corner_straight_motion_count=0,
        ),
        0.1,
    )

    assert command.motion == "LEFT"
    assert command.reason == "line_tracking"


def test_far_curve_turn_starts_after_near_heading_reaches_corner():
    planner = LineNavigationPlanner(
        NavigationConfig(direction_confirmation_frames=1)
    )

    command = planner.plan(
        line_info(
            filtered_heading_error_deg=6.0,
            filtered_lateral_offset_norm=0.1,
            turn_angle_deg=80.0,
        ),
        0.1,
    )

    assert command.motion == "RIGHT"


def test_conflicting_heading_and_preview_holds_slow_straight():
    planner = LineNavigationPlanner()

    command = planner.plan(
        line_info(
            filtered_heading_error_deg=35.5,
            filtered_lateral_offset_norm=0.049,
            turn_angle_deg=-49.4,
        ),
        0.1,
    )

    assert command.motion == "STRAIGHT"
    assert command.reason == "conflicting_heading_and_preview"
    assert command.steering_error_deg == 0.0
    assert command.linear_speed_mps == planner.config.min_linear_speed_mps


def test_turn_requires_three_consecutive_frames():
    planner = LineNavigationPlanner()
    sample = line_info(
        filtered_heading_error_deg=20.0,
        turn_angle_deg=30.0,
        turn_consistency=0.9,
    )

    commands = [planner.plan(sample, 0.1) for _ in range(3)]

    assert [command.motion for command in commands] == [
        "STRAIGHT",
        "STRAIGHT",
        "RIGHT",
    ]


@pytest.mark.parametrize(
    ("offset", "heading"),
    [
        (0.55, 0.0),
        (-0.55, 0.0),
        (-0.237, 5.0),
        (0.237, -5.0),
    ],
)
def test_large_offset_without_turn_heading_stays_straight(offset, heading):
    planner = LineNavigationPlanner()

    command = planner.plan(
        line_info(
            filtered_lateral_offset_norm=offset,
            filtered_heading_error_deg=heading,
        ),
        0.1,
    )

    assert command.motion == "STRAIGHT"
    assert not command.motion.startswith("RECOVER_")


@pytest.mark.parametrize(
    ("offset", "heading", "expected", "angular_sign"),
    [
        (0.593, -32.0, "RECOVER_RIGHT_TURN_LEFT_4", -1),
        (0.366, 15.0, "RECOVER_RIGHT_TURN_RIGHT_4", 1),
        (-0.40, -15.0, "RECOVER_LEFT_TURN_LEFT_2", -1),
        (-0.40, 15.0, "STRAIGHT", 1),
    ],
)
def test_recovery_separates_line_side_from_turn_direction(
    offset,
    heading,
    expected,
    angular_sign,
):
    planner = LineNavigationPlanner(
        NavigationConfig(direction_confirmation_frames=1)
    )

    command = planner.plan(
        line_info(
            filtered_lateral_offset_norm=offset,
            filtered_heading_error_deg=heading,
        ),
        0.1,
    )

    assert command.motion == expected
    assert command.angular_speed_rad_s * angular_sign > 0.0


def test_straight_line_heading_error_uses_recovery_not_plain_left():
    planner = LineNavigationPlanner()

    command = planner.plan(
        line_info(
            filtered_lateral_offset_norm=-0.10,
            filtered_heading_error_deg=-20.0,
            turn_angle_deg=None,
            turn_consistency=None,
        ),
        0.1,
    )

    assert command.motion == "RECOVER_LEFT_TURN_LEFT_2"


def test_plain_left_is_reserved_for_confirmed_left_curve():
    planner = LineNavigationPlanner()
    curve = line_info(
        filtered_lateral_offset_norm=-0.10,
        filtered_heading_error_deg=-20.0,
        turn_angle_deg=-30.0,
        turn_consistency=0.9,
    )

    commands = [planner.plan(curve, 0.1) for _ in range(3)]

    assert commands[-1].motion == "LEFT"


def test_moderate_offset_and_parallel_line_can_continue_straight():
    planner = LineNavigationPlanner()

    command = planner.plan(
        line_info(
            filtered_lateral_offset_norm=-0.40,
            filtered_heading_error_deg=-0.2,
            turn_angle_deg=-2.3,
        ),
        0.1,
    )

    assert command.motion == "STRAIGHT"


def test_robot_right_of_line_pointing_farther_right_recovers_at_three_deg():
    planner = LineNavigationPlanner()

    command = planner.plan(
        line_info(
            filtered_lateral_offset_norm=-0.383,
            filtered_heading_error_deg=-9.4,
            turn_angle_deg=None,
        ),
        0.1,
    )

    assert command.motion == "RECOVER_LEFT_TURN_LEFT_2"
    assert command.angular_speed_rad_s < 0.0


@pytest.mark.parametrize(
    ("offset", "heading", "expected"),
    [
        (-0.30, -2.999, "STRAIGHT"),
        (-0.30, -3.0, "RECOVER_LEFT_TURN_LEFT_2"),
        (-0.30, 9.999, "STRAIGHT"),
        (-0.30, 10.0, "RECOVER_LEFT_TURN_RIGHT_4"),
        (0.30, -9.999, "STRAIGHT"),
        (0.30, -10.0, "RECOVER_RIGHT_TURN_LEFT_2"),
        (0.30, 2.999, "STRAIGHT"),
        (0.30, 3.0, "RECOVER_RIGHT_TURN_RIGHT_4"),
    ],
)
def test_off_center_robot_uses_asymmetric_heading_deadband(
    offset,
    heading,
    expected,
):
    planner = LineNavigationPlanner()

    command = planner.plan(
        line_info(
            filtered_lateral_offset_norm=offset,
            filtered_heading_error_deg=heading,
            turn_angle_deg=None,
            turn_consistency=None,
        ),
        0.1,
    )

    assert command.motion == expected


@pytest.mark.parametrize(
    ("heading", "expected_level"),
    [
        (10.0, 1),
        (15.0, 1),
        (22.499, 1),
        (22.5, 2),
        (37.499, 2),
        (37.5, 3),
        (52.5, 4),
        (67.5, 5),
        (82.499, 5),
        (82.5, 6),
        (90.0, 6),
    ],
)
def test_right_recovery_turn_is_split_into_six_levels(
    heading,
    expected_level,
):
    planner = LineNavigationPlanner()

    command = planner.plan(
        line_info(
            filtered_lateral_offset_norm=0.30,
            filtered_heading_error_deg=heading,
            turn_angle_deg=None,
            turn_consistency=None,
        ),
        0.1,
    )
    payload = command.to_dict()

    expected_suffix = _recovery_turn_action_suffix(heading, "RIGHT")
    assert command.motion == (
        f"RECOVER_RIGHT_TURN_RIGHT_{expected_suffix}"
    )
    assert payload["recovery_side"] == "RIGHT"
    assert payload["turn_motion"] == f"TURN_RIGHT_{expected_suffix}"
    assert payload["turn_level"] == expected_suffix
    assert payload["turn_angle_deg"] == expected_level * 15.0
    assert command.target_heading_change_deg == expected_level * 15.0


def test_small_heading_and_centered_offset_stays_straight():
    planner = LineNavigationPlanner()

    command = planner.plan(
        line_info(
            filtered_heading_error_deg=-5.7,
            filtered_lateral_offset_norm=0.007,
            turn_angle_deg=70.7,
            turn_consistency=0.9,
        ),
        0.1,
    )

    assert command.motion == "STRAIGHT"


def test_ten_degree_recovery_threshold_uses_numbered_turn_not_plain_right():
    planner = LineNavigationPlanner(
        NavigationConfig(direction_confirmation_frames=1)
    )

    command = planner.plan(
        line_info(
            filtered_heading_error_deg=12.4,
            filtered_lateral_offset_norm=0.106,
            turn_angle_deg=0.2,
            path_turn_delta_deg=0.3,
            turn_consistency=1.0,
        ),
        0.1,
    )

    assert command.motion == "RECOVER_RIGHT_TURN_RIGHT_4"
    assert command.reason == "line_center_recovery"
    assert command.angular_speed_rad_s > 0.0


def test_real_right_curve_emits_plain_right_without_far_fit():
    planner = LineNavigationPlanner()
    curve = line_info(
        filtered_heading_error_deg=22.2,
        filtered_lateral_offset_norm=-0.079,
        turn_angle_deg=None,
        path_turn_delta_deg=18.0,
        turn_consistency=0.9,
    )

    commands = [planner.plan(curve, 0.1) for _ in range(3)]

    assert commands[-1].motion == "RIGHT"
    assert commands[-1].preview_turn_deg == 18.0


def test_large_heading_toward_center_uses_confirmed_plain_right():
    planner = LineNavigationPlanner()
    line = line_info(
        filtered_heading_error_deg=30.0,
        filtered_lateral_offset_norm=-0.416,
        turn_angle_deg=2.8,
        turn_consistency=0.9,
    )

    commands = [planner.plan(line, 0.1) for _ in range(3)]

    assert [command.motion for command in commands] == [
        "STRAIGHT",
        "STRAIGHT",
        "RIGHT",
    ]
    assert commands[-1].reason == "line_tracking"


def test_reliable_opposite_preview_still_suppresses_plain_right():
    planner = LineNavigationPlanner(
        NavigationConfig(direction_confirmation_frames=1)
    )

    command = planner.plan(
        line_info(
            filtered_heading_error_deg=30.0,
            filtered_lateral_offset_norm=-0.416,
            turn_angle_deg=-30.0,
            turn_consistency=0.9,
        ),
        0.1,
    )

    assert command.motion == "STRAIGHT"
    assert command.reason == "conflicting_heading_and_preview"


@pytest.mark.parametrize(
    ("offset", "heading", "curve_turn", "expected"),
    [
        (0.273, 6.3, 48.8, "RIGHT"),
        (-0.273, -6.3, -48.8, "LEFT"),
    ],
)
def test_matching_curve_ignores_normal_turn_induced_offset(
    offset,
    heading,
    curve_turn,
    expected,
):
    planner = LineNavigationPlanner(
        NavigationConfig(direction_confirmation_frames=1)
    )

    command = planner.plan(
        line_info(
            filtered_lateral_offset_norm=offset,
            filtered_heading_error_deg=heading,
            turn_angle_deg=curve_turn,
            turn_consistency=0.9,
        ),
        0.1,
    )

    assert command.motion == expected
    assert not command.motion.startswith("RECOVER_")


def test_matching_curve_does_not_hide_emergency_offset_recovery():
    planner = LineNavigationPlanner(
        NavigationConfig(
            direction_confirmation_frames=1,
            curve_follow_max_offset_norm=0.55,
        )
    )

    command = planner.plan(
        line_info(
            filtered_lateral_offset_norm=0.56,
            filtered_heading_error_deg=6.3,
            turn_angle_deg=48.8,
            turn_consistency=0.9,
        ),
        0.1,
    )

    assert command.motion == "RECOVER_RIGHT_TURN_RIGHT_4"


@pytest.mark.parametrize(
    ("recovery_side", "turn_direction"),
    [("LEFT", "LEFT"), ("RIGHT", "RIGHT")],
)
def test_recovery_side_does_not_change_numbered_turn_motion(
    recovery_side,
    turn_direction,
):
    planner = LineNavigationPlanner()
    offset = -0.30 if recovery_side == "LEFT" else 0.30
    heading = -46.0 if turn_direction == "LEFT" else 46.0

    command = planner.plan(
        line_info(
            filtered_lateral_offset_norm=offset,
            filtered_heading_error_deg=heading,
            turn_angle_deg=None,
            turn_consistency=None,
        ),
        0.1,
    )
    payload = command.to_dict()

    expected_suffix = _recovery_turn_action_suffix(
        heading,
        turn_direction,
    )
    assert command.motion == (
        f"RECOVER_{recovery_side}_TURN_{turn_direction}_{expected_suffix}"
    )
    assert payload["recovery_side"] == recovery_side
    assert payload["turn_motion"] == (
        f"TURN_{turn_direction}_{expected_suffix}"
    )
    assert payload["turn_angle_deg"] == (
        -45.0 if turn_direction == "LEFT" else 45.0
    )


def test_numbered_recovery_does_not_fall_back_to_standalone_recovery():
    planner = LineNavigationPlanner()

    first = planner.plan(
        line_info(
            filtered_lateral_offset_norm=0.55,
            filtered_heading_error_deg=-30.0,
        ),
        0.1,
    )
    no_turn_heading = planner.plan(
        line_info(filtered_lateral_offset_norm=0.50),
        0.1,
    )

    assert first.motion == "RECOVER_RIGHT_TURN_LEFT_4"
    assert no_turn_heading.motion == "STRAIGHT"


def test_small_or_inconsistent_far_curve_is_not_used_early():
    planner = LineNavigationPlanner()

    small_curve = planner.plan(
        line_info(turn_angle_deg=7.9, turn_consistency=1.0),
        0.1,
    )
    inconsistent_curve = LineNavigationPlanner().plan(
        line_info(turn_angle_deg=30.0, turn_consistency=0.2),
        0.1,
    )

    assert small_curve.preview_component_deg == 0.0
    assert inconsistent_curve.preview_component_deg == 0.0


def test_angular_acceleration_is_limited():
    config = NavigationConfig(
        max_angular_accel_rad_s2=0.5,
        direction_confirmation_frames=1,
    )
    planner = LineNavigationPlanner(config)

    command = planner.plan(
        line_info(
            filtered_heading_error_deg=40.0,
            turn_angle_deg=40.0,
            turn_consistency=0.9,
        ),
        0.1,
    )

    assert command.angular_accel_rad_s2 == pytest.approx(0.5)
    assert command.angular_speed_rad_s == pytest.approx(0.05)


@pytest.mark.parametrize(
    ("sample", "reason"),
    [
        ({"detected": False}, "line_not_detected"),
        (
            line_info(geometry_quality=0.1),
            "low_line_quality",
        ),
    ],
)
def test_unsafe_input_produces_stop(sample, reason):
    planner = LineNavigationPlanner()

    command = planner.plan(sample, 0.1)

    assert command.valid is False
    assert command.motion == "STOP"
    assert command.reason == reason
    assert command.linear_speed_mps == 0.0
