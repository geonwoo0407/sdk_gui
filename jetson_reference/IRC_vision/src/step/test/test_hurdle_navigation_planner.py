"""Unit tests for hardware-independent hurdle action decisions."""

import pytest

from step.hurdle_navigation_planner import HurdleNavigationPlanner


def hurdle_info(**overrides):
    """Create one valid hurdle analysis sample with optional changes."""
    sample = {
        "detected": True,
        "confidence": 0.9,
        "bearing_deg": 0.0,
        "offset_x_norm": 0.0,
        "depth_m": 0.1,
        "distance_m": 0.1,
        "ground_gap_m": 0.1,
        "camera_bottom_gap_m": 0.02,
        "depth_valid": True,
        "hurdle_angle_deg": 0.0,
    }
    sample.update(overrides)
    return sample


def test_parallel_hurdle_at_target_depth_requests_sdk_motion():
    planner = HurdleNavigationPlanner()

    command = planner.plan(hurdle_info())

    assert command.valid is True
    assert command.action == "GO"
    assert command.go_now is True
    assert command.sdk_motion_requested is True


def test_hurdle_waits_until_analyzer_confirms_go_condition():
    planner = HurdleNavigationPlanner()

    command = planner.plan(hurdle_info(go_now=False))

    assert command.action == "WAIT_GO_CONFIRMATION"
    assert command.sdk_motion_requested is False


@pytest.mark.parametrize("depth", [0.001, 0.2])
def test_go_depth_tolerance_includes_boundary(depth):
    planner = HurdleNavigationPlanner()

    command = planner.plan(hurdle_info(depth_m=depth))

    assert command.action == "GO"


@pytest.mark.parametrize(
    ("angle", "expected"),
    [(12.0, "ALIGN_LEFT"), (-12.0, "ALIGN_RIGHT")],
)
def test_hurdle_angle_selects_parallel_alignment_direction(angle, expected):
    planner = HurdleNavigationPlanner()

    command = planner.plan(hurdle_info(hurdle_angle_deg=angle))

    assert command.action == expected
    assert command.sdk_motion_requested is False


def test_horizontal_offset_does_not_affect_hurdle_action():
    planner = HurdleNavigationPlanner()

    command = planner.plan(hurdle_info(offset_x_norm=0.9))

    assert command.action == "GO"


@pytest.mark.parametrize(
    ("depth", "ground_gap", "expected"),
    [
        (0.45, 0.05, "STRAIGHT_3"),
        (0.05, 0.47, "GO"),
    ],
)
def test_raw_depth_selects_go_timing(
    depth,
    ground_gap,
    expected,
):
    planner = HurdleNavigationPlanner()

    command = planner.plan(
        hurdle_info(depth_m=depth, ground_gap_m=ground_gap)
    )

    assert command.action == expected


@pytest.mark.parametrize(
    ("sample", "reason"),
    [
        ({"detected": False}, "hurdle_not_detected"),
        (hurdle_info(confidence=0.1), "low_hurdle_confidence"),
        (
            hurdle_info(depth_m=None, depth_valid=False),
            "missing_valid_hurdle_depth",
        ),
    ],
)
def test_unsafe_input_produces_wait(sample, reason):
    planner = HurdleNavigationPlanner()

    command = planner.plan(sample)

    assert command.valid is False
    assert command.action == "WAIT"
    assert command.reason == reason
    assert command.sdk_motion_requested is False


def test_missing_legacy_ground_gap_does_not_block_raw_depth_control():
    planner = HurdleNavigationPlanner()

    far = planner.plan(
        hurdle_info(
            depth_m=0.50,
            ground_gap_m=None,
            camera_bottom_gap_m=0.08,
        )
    )
    close = planner.plan(
        hurdle_info(
            depth_m=0.05,
            ground_gap_m=None,
            camera_bottom_gap_m=0.02,
        )
    )

    assert far.action == "STRAIGHT_3"
    assert far.go_now is False
    assert close.action == "GO"
    assert close.go_now is True
