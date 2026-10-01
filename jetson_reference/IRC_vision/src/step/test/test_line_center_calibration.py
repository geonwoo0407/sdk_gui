"""Tests for robot-center calibration and corner geometry."""

import pytest

from step.yolo_line_analyzer import calibrated_robot_center_x
from step.yolo_line_analyzer import ground_forward_distance_from_depth
from step.yolo_line_analyzer import LinePoint
from step.yolo_line_analyzer import YoloLineAnalyzer


def test_1280_image_center_is_shifted_70_pixels_right():
    assert calibrated_robot_center_x(1280, 70.0) == pytest.approx(710.0)


def test_center_calibration_is_clipped_inside_image():
    assert calibrated_robot_center_x(1280, -1000.0) == 0.0
    assert calibrated_robot_center_x(1280, 1000.0) == 1279.0


def _corner_geometry(points):
    return YoloLineAnalyzer._detect_corner_start_geometry(
        points,
        min_points=3,
        min_segment_length_px=20.0,
        straight_max_turn_delta_deg=15.0,
        min_turn_delta_deg=30.0,
        onset_deviation_deg=15.0,
        min_consistent_segments=1,
        min_consistency=0.75,
    )


@pytest.mark.parametrize(
    ("far_x", "expected_direction"),
    [(600.0, "RIGHT"), (400.0, "LEFT")],
)
def test_three_points_find_corner_at_middle_point(far_x, expected_direction):
    points = [
        LinePoint(500.0, 700.0, 0.9),
        LinePoint(500.0, 600.0, 0.9),
        LinePoint(far_x, 600.0, 0.9),
    ]
    result = _corner_geometry(points)
    assert result["detected"] is True
    assert result["direction"] == expected_direction
    assert result["start_index"] == 1


def test_slanted_straight_line_never_creates_corner_preview():
    points = [
        LinePoint(500.0 + index * 20.0, 700.0 - index * 70.0, 0.9)
        for index in range(7)
    ]
    result = _corner_geometry(points)
    assert result["detected"] is False
    assert result["state"] == "STRAIGHT"


def test_floor_forward_projection_removes_camera_pitch_slant():
    depth_m = (0.50 + 0.70) / (2.0 ** 0.5)
    camera_down_m = (-0.50 + 0.70) / (2.0 ** 0.5)
    y_px = 360.0 + 1000.0 * camera_down_m / depth_m
    lateral_m, forward_m = ground_forward_distance_from_depth(
        x_px=640.0,
        y_px=y_px,
        depth_m=depth_m,
        fx=1000.0,
        fy=1000.0,
        cx=640.0,
        cy=360.0,
        camera_pitch_down_deg=45.0,
        camera_height_m=0.70,
        reference_height_m=0.0,
        camera_forward_offset_m=0.0,
    )
    assert lateral_m == pytest.approx(0.0)
    assert forward_m == pytest.approx(0.50)
