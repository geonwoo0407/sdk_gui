"""Tests for hurdle-owned line-intersection geometry."""

import pytest

from mission_control.hurdle_line_fusion import build_hurdle_path_reference


def test_route_line_is_extended_to_hurdle_lower_edge():
    hurdle = {
        "detected": True,
        "bbox": [500, 300, 900, 500],
        "image_width": 1280,
        "image_height": 720,
    }
    line = {
        "detected": True,
        "robot_center_x_px": 710.0,
        "center_points_px": [[800, 650], [800, 580], [800, 250], [800, 180]],
    }
    result = build_hurdle_path_reference(hurdle, line)
    assert result["path_reference_valid"] is True
    assert result["path_reference_point_px"] == [800, 500]
    assert result["path_offset_x_norm"] == pytest.approx(90 / 640)
    assert result["path_direction"] == "RIGHT"
    assert result["line_mode_allowed"] is False


def test_points_hidden_inside_hurdle_are_not_used_for_fit():
    hurdle = {
        "detected": True,
        "bbox": [500, 300, 900, 500],
        "image_width": 1280,
        "image_height": 720,
    }
    line = {"detected": True, "center_points_px": [[600, 450], [800, 430]]}
    result = build_hurdle_path_reference(hurdle, line)
    assert result["path_reference_valid"] is False
    assert result["path_reference_reason"] == "insufficient_unoccluded_line_points"


def test_consistent_points_rejected_across_occlusion_can_be_recovered():
    hurdle = {
        "detected": True,
        "bbox": [500, 300, 900, 500],
        "image_width": 1280,
        "image_height": 720,
    }
    line = {
        "detected": True,
        "robot_center_x_px": 710.0,
        "center_points_px": [[800, 650]],
        "path_rejected_points_px": [[800, 580], [800, 250]],
    }
    result = build_hurdle_path_reference(hurdle, line)
    assert result["path_reference_valid"] is True
    assert result["path_reference_point_px"] == [800, 500]
    assert len(result["path_support_points_px"]) == 3
