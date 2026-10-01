"""Tests for goal ground-plane distance reporting."""

import pytest

from step.goal_analyzer import GoalAnalyzer


def test_goal_projection_keeps_raw_depth_without_height_correction():
    """Do not infer floor range from the elevated goal/backboard target."""
    analyzer = object.__new__(GoalAnalyzer)
    analyzer.fx = 600.0
    analyzer.fy = 600.0
    analyzer.cx = 640.0
    analyzer.cy = 360.0
    projection = analyzer._project(640, 360, 1.0)

    assert projection[3] == pytest.approx(1.0)
    assert projection[4] is None
