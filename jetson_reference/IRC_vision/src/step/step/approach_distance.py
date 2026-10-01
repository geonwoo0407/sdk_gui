"""Shared distance bands for discrete straight walking motions."""

from __future__ import annotations

import math


# Measured forward-walk averages (metres): 2, 4, 6, 8, and 10 steps.
# STRAIGHT_0 is deliberately left to the behavior layer: it may mean hold,
# one micro-step, or a small retreat depending on the active mission.
APPROACH_DISTANCE_LIMITS_M = (
    0.130,
    0.263,
    0.427,
    0.564,
    0.680,
    0.780,
)


def approach_motion_for_distance(distance_m: float | None) -> str:
    """Return STRAIGHT_0..5, or generic STRAIGHT outside known range."""
    if distance_m is None or isinstance(distance_m, bool):
        return "STRAIGHT"
    try:
        distance = float(distance_m)
    except (TypeError, ValueError):
        return "STRAIGHT"
    if not math.isfinite(distance) or distance < 0.0:
        return "STRAIGHT"
    for level, upper_bound in enumerate(APPROACH_DISTANCE_LIMITS_M):
        if distance <= upper_bound + 1e-9:
            return f"STRAIGHT_{level}"
    return "STRAIGHT"


def approach_level_from_motion(motion: str) -> int | None:
    """Extract a valid 0..5 approach level from a motion name."""
    normalized = motion.strip().upper()
    if not normalized.startswith("STRAIGHT_"):
        return None
    try:
        level = int(normalized.rsplit("_", 1)[1])
    except (TypeError, ValueError):
        return None
    return level if 0 <= level <= 5 else None
