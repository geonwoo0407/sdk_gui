"""Unit tests for independent raw ball candidate selection."""

import numpy as np

from step.ball_only_debug import select_raw_candidates


def test_selects_ball_below_production_threshold_without_filtering():
    """Expose a weak raw ball even when another class has a higher score."""
    output = np.asarray(
        [
            [10, 10, 20, 20, 0.91, 0],
            [30, 30, 50, 50, 0.07, 1],
            [60, 60, 80, 80, 0.03, 1],
        ],
        dtype=np.float32,
    )[None, ...]

    ball, strongest = select_raw_candidates(output, ball_class_id=1)

    assert ball is not None
    assert abs(ball.confidence - 0.07) < 1e-6
    assert strongest is not None
    assert strongest.class_id == 0


def test_returns_none_when_model_emits_no_ball_class_row():
    """Report no raw ball when every output row belongs to another class."""
    output = np.asarray(
        [[10, 10, 20, 20, 0.91, 0]],
        dtype=np.float32,
    )[None, ...]

    ball, strongest = select_raw_candidates(output, ball_class_id=1)

    assert ball is None
    assert strongest is not None
