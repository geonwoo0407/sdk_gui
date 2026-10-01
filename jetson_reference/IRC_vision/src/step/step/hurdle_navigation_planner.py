#!/usr/bin/env python3
"""Convert hurdle geometry into SDK-oriented jump action candidates."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from .approach_distance import approach_level_from_motion
from .approach_distance import approach_motion_for_distance


@dataclass(frozen=True)
class HurdleNavigationConfig:
    """Provisional hurdle alignment and jump thresholds."""

    min_confidence: float = 0.60
    control_start_depth_m: float = 1.0
    go_target_depth_m: float = 0.10
    go_depth_tolerance_m: float = 0.10
    go_angle_tolerance_deg: float = 8.0
    path_center_tolerance_norm: float = 0.10


@dataclass(frozen=True)
class HurdleActionCommand:
    """One abstract hurdle action; it does not drive robot hardware."""

    valid: bool
    action: str
    reason: str
    sdk_motion_requested: bool
    confidence: float
    depth_m: float | None
    distance_m: float | None
    ground_gap_m: float | None
    camera_bottom_gap_m: float | None
    ground_gap_error_m: float | None
    hurdle_angle_deg: float | None
    is_parallel: bool
    ground_gap_in_go_range: bool
    go_now: bool

    def to_dict(self) -> dict[str, Any]:
        """Return a rounded JSON-compatible representation."""
        approach_level = approach_level_from_motion(self.action)
        return {
            "valid": self.valid,
            "action": self.action,
            "reason": self.reason,
            "sdk_motion_requested": self.sdk_motion_requested,
            "confidence": round(self.confidence, 4),
            "depth_m": _round_optional(self.depth_m, 3),
            # Legacy distance keys carry raw Depth Z for compatibility.
            "distance_m": _round_optional(self.depth_m, 3),
            "ground_gap_m": _round_optional(self.depth_m, 3),
            "camera_bottom_gap_m": _round_optional(
                self.camera_bottom_gap_m,
                3,
            ),
            "ground_gap_error_m": _round_optional(
                self.ground_gap_error_m,
                3,
            ),
            "hurdle_angle_deg": _round_optional(
                self.hurdle_angle_deg,
                3,
            ),
            "is_parallel": self.is_parallel,
            "ground_gap_in_go_range": self.ground_gap_in_go_range,
            "go_now": self.go_now,
            "approach_motion": (
                self.action
                if self.action == "STRAIGHT" or approach_level is not None
                else None
            ),
            "approach_level": approach_level,
            "approach_target_distance_m": _round_optional(
                self.depth_m,
                3,
            ),
        }


def _round_optional(value: float | None, digits: int) -> float | None:
    if value is None:
        return None
    return round(value, digits)


def _number(data: dict[str, Any], key: str) -> float | None:
    value = data.get(key)
    if value is None or isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


class HurdleNavigationPlanner:
    """Choose a jump SDK action from one fresh ``hurdle_info`` sample."""

    def __init__(
        self,
        config: HurdleNavigationConfig | None = None,
    ) -> None:
        self.config = config or HurdleNavigationConfig()

    def wait(self, reason: str) -> HurdleActionCommand:
        """Return a non-action command for missing or unsafe input."""
        return HurdleActionCommand(
            valid=False,
            action="WAIT",
            reason=reason,
            sdk_motion_requested=False,
            confidence=0.0,
            depth_m=None,
            distance_m=None,
            ground_gap_m=None,
            camera_bottom_gap_m=None,
            ground_gap_error_m=None,
            hurdle_angle_deg=None,
            is_parallel=False,
            ground_gap_in_go_range=False,
            go_now=False,
        )

    def plan(self, hurdle_info: dict[str, Any]) -> HurdleActionCommand:
        """Create one alignment, distance-adjustment, or GO action."""
        if not bool(hurdle_info.get("detected", False)):
            return self.wait("hurdle_not_detected")
        confidence = _number(hurdle_info, "confidence")
        if confidence is None or confidence < self.config.min_confidence:
            return self.wait("low_hurdle_confidence")
        depth = _number(hurdle_info, "depth_m")
        if not bool(hurdle_info.get("depth_valid", False)) or depth is None:
            return self.wait("missing_valid_hurdle_depth")
        distance = depth
        ground_gap = depth
        if depth <= 0.0:
            return self.wait("missing_valid_hurdle_depth")
        if depth > self.config.control_start_depth_m:
            return self.wait("hurdle_outside_control_range")
        camera_bottom_gap = _number(
            hurdle_info,
            "camera_bottom_gap_m",
        )
        hurdle_angle = _number(hurdle_info, "hurdle_angle_deg")
        if hurdle_angle is None:
            return self.wait("missing_hurdle_parallel_angle")
        parallel = (
            abs(hurdle_angle) <= self.config.go_angle_tolerance_deg
        )
        path_reference_valid = bool(
            hurdle_info.get("path_reference_valid", False)
        )
        path_offset = _number(hurdle_info, "path_offset_x_norm")
        path_centered = bool(
            not path_reference_valid
            or path_offset is None
            or abs(path_offset) <= self.config.path_center_tolerance_norm
        )
        ground_gap_error = (
            depth - self.config.go_target_depth_m
        )
        ground_gap_in_range = (
            abs(ground_gap_error)
            <= self.config.go_depth_tolerance_m + 1e-9
        )
        ready_geometry = (
            parallel
            and path_centered
            and ground_gap_in_range
        )
        analyzer_go_now = hurdle_info.get("go_now")
        go_now = bool(
            ready_geometry
            and (
                ready_geometry
                if analyzer_go_now is None
                else analyzer_go_now
            )
        )

        if go_now:
            action = "GO"
            reason = "hurdle_parallel_at_close_depth"
        elif (
            path_reference_valid
            and path_offset is not None
            and not path_centered
        ):
            action = "TURN_RIGHT" if path_offset > 0.0 else "TURN_LEFT"
            reason = "align_to_hurdle_line_intersection"
        elif not parallel:
            action = "ALIGN_LEFT" if hurdle_angle > 0.0 else "ALIGN_RIGHT"
            reason = "align_robot_parallel_to_hurdle"
        elif ready_geometry:
            action = "WAIT_GO_CONFIRMATION"
            reason = "waiting_for_stable_hurdle_condition"
        elif ground_gap_error > self.config.go_depth_tolerance_m:
            action = approach_motion_for_distance(depth)
            reason = "hurdle_aligned_discrete_approach"
        else:
            action = "WAIT_GO_CONFIRMATION"
            reason = "waiting_for_stable_hurdle_condition"

        return HurdleActionCommand(
            valid=True,
            action=action,
            reason=reason,
            sdk_motion_requested=go_now,
            confidence=confidence,
            depth_m=depth,
            distance_m=distance,
            ground_gap_m=ground_gap,
            camera_bottom_gap_m=camera_bottom_gap,
            ground_gap_error_m=ground_gap_error,
            hurdle_angle_deg=hurdle_angle,
            is_parallel=parallel,
            ground_gap_in_go_range=ground_gap_in_range,
            go_now=go_now,
        )
