"""Unit tests for hardware-independent ball navigation decisions."""

import pytest

from step.ball_navigation_planner import BallNavigationConfig
from step.ball_navigation_planner import BallNavigationPlanner


def ball_info(**overrides):
    """Create one valid ball analysis sample with optional changes."""
    sample = {
        "detected": True,
        "confidence": 0.9,
        "bearing_deg": 0.0,
        "offset_x_norm": 0.0,
        "depth_m": 0.88,
        "ground_distance_m": 0.88,
        "distance_m": 0.89,
        "depth_valid": True,
        "depth_age_sec": 0.05,
        "pickup_ready": False,
        "pickup_now": False,
    }
    sample.update(overrides)
    if "distance_m" not in overrides and "depth_m" in overrides:
        sample["distance_m"] = overrides["depth_m"]
    if "ground_distance_m" not in overrides and "depth_m" in overrides:
        sample["ground_distance_m"] = overrides["depth_m"]
    return sample


@pytest.mark.parametrize(
    "bearing",
    [-12.0, 0.0, 12.0],
)
def test_ball_distance_motion_does_not_follow_bearing(bearing):
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            bearing_deg=bearing,
            depth_m=0.70,
            ground_distance_m=0.70,
        ),
        0.1,
    )

    assert command.valid is True
    assert command.motion == "STRAIGHT_2"
    assert command.linear_speed_mps > 0.0


def test_centered_ball_uses_four_repeat_forward():
    planner = BallNavigationPlanner()

    command = planner.plan(ball_info(), 0.1)

    assert command.motion == "STRAIGHT_2"
    assert command.linear_speed_mps > 0.0
    assert command.travel_distance_m == pytest.approx(
        command.linear_speed_mps * command.command_duration_sec
    )


def test_ball_at_68cm_uses_four_repeat_forward():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(depth_m=0.68, distance_m=0.68, pickup_ready=False),
        0.1,
    )

    assert command.motion == "STRAIGHT_2"


@pytest.mark.parametrize(
    ("ground_distance", "expected_motion"),
    [
        (0.80, "STRAIGHT_2"),
        (1.50, "STRAIGHT_2"),
        (0.70, "STRAIGHT_2"),
        (0.60, "STRAIGHT_2"),
        (0.50, "PICKUP_NOW"),
    ],
)
def test_aligned_general_ball_approach_distance_policy(
    ground_distance,
    expected_motion,
):
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            depth_m=ground_distance,
            ground_distance_m=ground_distance,
            bearing_deg=0.0,
            offset_x_norm=0.0,
        ),
        0.1,
    )

    assert command.motion == expected_motion


@pytest.mark.parametrize(
    ("ground_distance", "bearing"),
    [(0.70, 30.0), (0.70, -30.0), (0.60, 30.0), (0.60, -30.0)],
)
def test_valid_general_approach_does_not_emit_raw_numbered_turn(
    ground_distance,
    bearing,
):
    command = BallNavigationPlanner().plan(
        ball_info(
            depth_m=ground_distance,
            ground_distance_m=ground_distance,
            bearing_deg=bearing,
        ),
        0.1,
    )

    assert not command.motion.startswith(("TURN_LEFT_", "TURN_RIGHT_"))


def test_fresh_distance_starts_pickup_sequence_at_570mm_without_vision_gate():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            depth_m=0.62,
            ground_distance_m=0.30,
            distance_m=0.570,
            pickup_ready=False,
            pickup_now=False,
        ),
        0.1,
    )

    assert command.valid is True
    assert command.motion == "PICKUP_NOW"
    assert command.reason == "ball_distance_entered_pickup_sequence"
    assert command.linear_speed_mps == 0.0
    assert command.pickup_now is True
    assert command.pickup_approach_motion is None
    assert command.distance_m == 0.570
    assert command.ground_distance_m == 0.30


@pytest.mark.parametrize("depth_age_sec", [None, 0.701])
def test_invalid_or_stale_depth_does_not_start_pickup_sequence(
    depth_age_sec,
):
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            depth_m=0.40,
            ground_distance_m=0.15,
            pickup_ready=True,
            pickup_now=True,
            depth_age_sec=depth_age_sec,
        ),
        0.1,
    )

    assert command.valid is False
    assert command.motion == "STOP"
    assert command.reason == "stale_ball_distance_for_pickup_sequence"


def test_pickup_now_false_still_starts_sequence_at_close_distance():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(depth_m=0.40, ground_distance_m=0.13),
        0.1,
    )

    assert command.motion == "PICKUP_NOW"
    assert command.pickup_now is True
    assert command.pickup_ready is False
    assert command.pickup_approach_motion is None


def test_pickup_sequence_does_not_start_above_570mm():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            depth_m=0.40,
            distance_m=0.571,
            ground_distance_m=0.15,
            pickup_ready=True,
            pickup_now=True,
        ),
        0.1,
    )

    assert command.valid is True
    assert command.motion != "PICKUP_NOW"
    assert command.pickup_now is False


def test_580mm_keeps_general_ball_approach():
    command = BallNavigationPlanner().plan(
        ball_info(
            depth_m=0.40,
            distance_m=0.580,
            ground_distance_m=0.30,
            pickup_ready=True,
            pickup_now=True,
        ),
        0.1,
    )

    assert command.motion == "STRAIGHT_2"
    assert command.pickup_now is False


def test_invalid_distance_never_falls_back_to_ground_distance():
    command = BallNavigationPlanner().plan(
        ball_info(
            depth_m=0.40,
            distance_m=None,
            ground_distance_m=0.30,
        ),
        0.1,
    )

    assert command.valid is False
    assert command.motion == "STOP"
    assert command.reason == "missing_valid_ball_distance"


def test_pickup_sequence_uses_fine_motion_at_130mm():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            depth_m=0.13,
            ground_distance_m=0.05,
            pickup_ready=True,
            pickup_now=True,
        ),
        0.1,
    )

    assert command.motion == "PICKUP_NOW"
    assert command.pickup_approach_motion is None


def test_stale_80cm_pickup_flag_cannot_trigger_pickup():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(depth_m=0.80, distance_m=0.81, pickup_now=True),
        0.1,
    )

    assert command.motion == "STRAIGHT_2"
    assert command.pickup_now is False


def test_offset_is_used_when_camera_bearing_is_missing():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            bearing_deg=None,
            offset_x_norm=0.3,
            depth_m=0.70,
            ground_distance_m=0.70,
        ),
        0.1,
    )

    assert command.motion == "STRAIGHT_2"


def test_bottom_center_path_angle_has_priority_over_camera_bearing():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            steering_angle_deg=-30.0,
            bearing_deg=12.0,
            offset_x_norm=-0.3,
            depth_m=0.60,
            ground_distance_m=0.60,
        ),
        0.1,
    )

    assert command.motion == "STRAIGHT_2"
    assert command.linear_speed_mps > 0.0


def test_near_center_deadband_suppresses_path_angle_turn():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            steering_angle_deg=12.0,
            offset_x_norm=0.04,
            depth_m=0.60,
            ground_distance_m=0.55,
        ),
        0.1,
    )

    assert not command.motion.startswith("TURN_")
    assert command.target_heading_change_deg == 0.0


@pytest.mark.parametrize(
    "depth",
    [1.0, 1.5, 1.51, 2.0, 10.0],
)
def test_valid_far_depth_uses_four_repeat_forward(depth):
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            depth_m=depth,
            ground_distance_m=depth,
            distance_m=depth,
        ),
        0.1,
    )

    assert command.valid is True
    assert command.motion == "STRAIGHT_2"
    assert command.reason == "ball_aligned_discrete_approach"


@pytest.mark.parametrize(
    "depth",
    [
        None,
        "invalid",
        float("nan"),
        float("inf"),
        float("-inf"),
        0.0,
        -0.1,
    ],
)
def test_invalid_distance_stops(depth):
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            depth_m=depth,
            ground_distance_m=1.0,
            distance_m=depth,
        ),
        0.1,
    )

    assert command.valid is False
    assert command.motion == "STOP"
    assert command.reason == "missing_valid_ball_distance"


def test_missing_distance_does_not_emit_unsupported_raw_turn():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            bearing_deg=12.0,
            depth_m=None,
            ground_distance_m=None,
            distance_m=None,
            depth_valid=False,
        ),
        0.1,
    )

    assert command.valid is False
    assert command.motion == "STOP"
    assert command.reason == "missing_valid_ball_distance"
    assert command.depth_valid is False


@pytest.mark.parametrize(
    ("ball_occurrence", "ground_distance", "bearing"),
    [
        (1, 0.70, -30.0),
        (1, 0.60, 30.0),
        (2, 0.70, 30.0),
        (2, 0.60, -30.0),
    ],
)
def test_ball_occurrence_does_not_change_straight_distance_bucket(
    ball_occurrence,
    ground_distance,
    bearing,
):
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            bearing_deg=bearing,
            depth_m=ground_distance,
            ground_distance_m=ground_distance,
            ball_occurrence=ball_occurrence,
        ),
        0.1,
    )

    assert command.motion == "STRAIGHT_2"
    assert command.linear_speed_mps > 0.0
    assert command.reason == "ball_aligned_discrete_approach"


@pytest.mark.parametrize("distance_m", [1.5, 0.9, 0.681, 0.571])
def test_general_ball_approach_always_uses_four_repeat_forward(distance_m):
    planner = BallNavigationPlanner()

    motion = planner._general_approach_motion(distance_m)

    assert motion == "STRAIGHT_2"


def test_angular_acceleration_is_limited():
    config = BallNavigationConfig(max_angular_accel_rad_s2=0.5)
    planner = BallNavigationPlanner(config)

    command = planner.plan(ball_info(bearing_deg=30.0), 0.1)

    assert command.angular_accel_rad_s2 == pytest.approx(0.5)
    assert command.angular_speed_rad_s == pytest.approx(0.05)


@pytest.mark.parametrize(
    ("sample", "reason"),
    [
        ({"detected": False}, "ball_not_detected"),
        (ball_info(confidence=0.1), "low_ball_confidence"),
        (
            ball_info(depth_m=None, depth_valid=False),
            "missing_valid_ball_distance",
        ),
    ],
)
def test_unsafe_input_produces_stop(sample, reason):
    planner = BallNavigationPlanner()

    command = planner.plan(sample, 0.1)

    assert command.valid is False
    assert command.motion == "STOP"
    assert command.reason == reason
