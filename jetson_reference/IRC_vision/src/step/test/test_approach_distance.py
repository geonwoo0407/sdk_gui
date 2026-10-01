"""Unit tests for the shared measured straight-motion distance bands."""

import pytest

from step.approach_distance import approach_motion_for_distance


@pytest.mark.parametrize(
    ("distance_m", "expected"),
    [
        (0.0, "STRAIGHT_0"),
        (0.130, "STRAIGHT_0"),
        (0.131, "STRAIGHT_1"),
        (0.263, "STRAIGHT_1"),
        (0.264, "STRAIGHT_2"),
        (0.427, "STRAIGHT_2"),
        (0.428, "STRAIGHT_3"),
        (0.564, "STRAIGHT_3"),
        (0.565, "STRAIGHT_4"),
        (0.680, "STRAIGHT_4"),
        (0.681, "STRAIGHT_5"),
        (0.780, "STRAIGHT_5"),
        (0.781, "STRAIGHT"),
        (None, "STRAIGHT"),
    ],
)
def test_measured_distance_bands(distance_m, expected):
    assert approach_motion_for_distance(distance_m) == expected
