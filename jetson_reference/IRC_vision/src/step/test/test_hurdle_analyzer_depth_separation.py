"""Tests separating RGB hurdle detection from required motion depth."""

from step.hurdle_analyzer import HurdleAnalyzer


def _rgb_only_analyzer() -> HurdleAnalyzer:
    analyzer = object.__new__(HurdleAnalyzer)
    analyzer.min_confidence = 0.40
    analyzer.direction_deadband_norm = 0.04
    analyzer.max_valid_depth_m = 4.0
    analyzer.go_target_depth_m = 0.10
    analyzer.go_depth_tolerance_m = 0.10
    analyzer.go_angle_tolerance_deg = 8.0
    analyzer.fx = None
    analyzer.fy = None
    analyzer.cx = None
    analyzer.cy = None
    analyzer._sample_depths = lambda *_args: (None, None, None, 0)
    return analyzer


def test_rgb_hurdle_remains_candidate_without_depth():
    """Keep a strong hurdle bbox even when RealSense depth is absent."""
    analyzer = _rgb_only_analyzer()
    detection = {
        "confidence": 0.72,
        "bbox": [300, 320, 980, 390],
        "center": [640, 355],
    }

    candidate = analyzer._build_candidate(detection, 1280, 720)

    assert candidate is not None
    assert candidate.depth_valid is False
    assert candidate.depth_m is None
    assert candidate.ground_distance_m is None
    state = analyzer._state(candidate)
    assert state[0] == "NO_DEPTH"
    assert state[4] is False
