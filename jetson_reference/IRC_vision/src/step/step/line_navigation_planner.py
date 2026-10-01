#!/usr/bin/env python3
"""Convert line geometry into bounded, hardware-independent motion targets."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from .approach_distance import approach_level_from_motion


RECOVERY_TURN_STEP_DEG = 15
RECOVERY_TURN_MAX_LEVEL = 6
RECOVERY_TURN_ACTION_SUFFIXES = {
    "LEFT": (2, 4, 6, 8, 10, 13),
    "RIGHT": (4, 6, 8, 10, 12, 15),
}


@dataclass(frozen=True)
class NavigationConfig:
    """Tunable limits and gains for line-following command generation."""

    min_line_quality: float = 0.35
    max_linear_speed_mps: float = 0.05
    min_linear_speed_mps: float = 0.015
    recovery_lateral_speed_mps: float = 0.025
    recovery_turn_speed_rad_s: float = 0.22
    recovery_heading_turn_deg: float = 10.0
    recovery_away_heading_turn_deg: float = 3.0
    curve_follow_max_offset_norm: float = 0.55
    recovery_straight_offset_norm: float = 0.45
    recovery_parallel_heading_deg: float = 2.0
    max_angular_speed_rad_s: float = 0.60
    max_angular_accel_rad_s2: float = 1.20
    heading_gain: float = 1.0
    offset_gain_deg: float = 24.0
    preview_gain: float = 0.15
    preview_min_turn_deg: float = 8.0
    preview_min_consistency: float = 0.55
    steering_response_sec: float = 0.70
    turn_enter_deg: float = 12.0
    turn_exit_deg: float = 7.0
    turn_min_heading_deg: float = 5.0
    direction_confirmation_frames: int = 3
    ambiguity_min_angle_deg: float = 25.0
    recovery_enter_offset_norm: float = 0.20
    recovery_exit_offset_norm: float = 0.12
    command_duration_sec: float = 0.40


@dataclass(frozen=True)
class NavigationCommand:
    """One abstract command for the behavior or walking algorithm."""

    valid: bool
    motion: str
    reason: str
    linear_speed_mps: float
    lateral_speed_mps: float
    angular_speed_rad_s: float
    angular_accel_rad_s2: float
    command_duration_sec: float
    travel_distance_m: float
    lateral_travel_distance_m: float
    target_heading_change_deg: float
    steering_error_deg: float
    heading_component_deg: float
    offset_component_deg: float
    preview_component_deg: float
    heading_error_deg: float | None
    lateral_offset_norm: float | None
    preview_turn_deg: float | None
    line_quality: float
    corner_prepare: bool
    corner_direction: str | None
    corner_start_distance_m: float | None
    corner_remaining_forward_m: float | None
    corner_straight_motion_count: int | None

    def to_dict(self) -> dict[str, Any]:
        """Return a rounded JSON-compatible representation."""
        recovery_side, turn_motion, turn_level, turn_angle_deg = (
            _recovery_motion_metadata(self.motion)
        )
        approach_level = approach_level_from_motion(self.motion)
        return {
            "valid": self.valid,
            "motion": self.motion,
            "reason": self.reason,
            "linear_speed_mps": round(self.linear_speed_mps, 4),
            "lateral_speed_mps": round(self.lateral_speed_mps, 4),
            "angular_speed_rad_s": round(self.angular_speed_rad_s, 4),
            "angular_accel_rad_s2": round(self.angular_accel_rad_s2, 4),
            "command_duration_sec": round(self.command_duration_sec, 3),
            "travel_distance_m": round(self.travel_distance_m, 4),
            "lateral_travel_distance_m": round(
                self.lateral_travel_distance_m,
                4,
            ),
            "target_heading_change_deg": round(
                self.target_heading_change_deg, 3
            ),
            "steering_error_deg": round(self.steering_error_deg, 3),
            "heading_component_deg": round(self.heading_component_deg, 3),
            "offset_component_deg": round(self.offset_component_deg, 3),
            "preview_component_deg": round(self.preview_component_deg, 3),
            "heading_error_deg": _round_optional(self.heading_error_deg, 3),
            "lateral_offset_norm": _round_optional(
                self.lateral_offset_norm, 6
            ),
            "preview_turn_deg": _round_optional(self.preview_turn_deg, 3),
            "line_quality": round(self.line_quality, 4),
            "corner_prepare": self.corner_prepare,
            "corner_direction": self.corner_direction,
            "corner_start_distance_m": _round_optional(
                self.corner_start_distance_m,
                4,
            ),
            "corner_remaining_forward_m": _round_optional(
                self.corner_remaining_forward_m,
                4,
            ),
            "corner_straight_motion_count": (
                self.corner_straight_motion_count
            ),
            "approach_motion": (
                self.motion
                if self.motion == "STRAIGHT" or approach_level is not None
                else None
            ),
            "approach_level": approach_level,
            "approach_target_distance_m": _round_optional(
                self.corner_start_distance_m,
                4,
            ),
            "recovery_side": recovery_side,
            "turn_motion": turn_motion,
            "turn_level": turn_level,
            "turn_angle_deg": turn_angle_deg,
        }


def _round_optional(value: float | None, digits: int) -> float | None:
    if value is None:
        return None
    return round(value, digits)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _number(data: dict[str, Any], key: str) -> float | None:
    value = data.get(key)
    if value is None or isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _recovery_turn_level(heading_error_deg: float) -> int:
    """Quantize a recovery heading to the nearest 15-degree motion."""
    magnitude = abs(heading_error_deg)
    level = math.floor(
        (magnitude + RECOVERY_TURN_STEP_DEG / 2.0)
        / RECOVERY_TURN_STEP_DEG
    )
    return int(_clamp(level, 1, RECOVERY_TURN_MAX_LEVEL))


def _recovery_turn_action_suffix(
    heading_error_deg: float,
    turn_direction: str,
) -> int:
    """Map the nearest 15-degree level to the deployed action suffix."""
    turn_direction = turn_direction.strip().upper()
    if turn_direction not in RECOVERY_TURN_ACTION_SUFFIXES:
        raise ValueError(f"unsupported turn direction: {turn_direction}")
    level = _recovery_turn_level(heading_error_deg)
    return RECOVERY_TURN_ACTION_SUFFIXES[turn_direction][level - 1]


def numbered_turn_motion(
    heading_error_deg: float,
    turn_direction: str,
) -> str:
    """Return the deployed numbered turn for one signed heading error."""
    normalized_direction = turn_direction.strip().upper()
    suffix = _recovery_turn_action_suffix(
        heading_error_deg,
        normalized_direction,
    )
    return f"TURN_{normalized_direction}_{suffix}"


def numbered_turn_motion_metadata(
    motion: str,
) -> tuple[str | None, int | None, float | None]:
    """Decode a deployed numbered turn into its suffix and target angle."""
    normalized = motion.strip().upper()
    for turn_direction, sign in (("RIGHT", 1.0), ("LEFT", -1.0)):
        prefix = f"TURN_{turn_direction}_"
        if not normalized.startswith(prefix):
            continue
        try:
            suffix = int(normalized.removeprefix(prefix))
        except ValueError:
            return None, None, None
        supported_suffixes = RECOVERY_TURN_ACTION_SUFFIXES[turn_direction]
        if suffix not in supported_suffixes:
            return None, None, None
        angle_level = supported_suffixes.index(suffix) + 1
        return normalized, suffix, sign * angle_level * RECOVERY_TURN_STEP_DEG
    return None, None, None


def _recovery_motion_metadata(
    motion: str,
) -> tuple[str | None, str | None, int | None, float | None]:
    """Extract recovery side and the actual numbered turn motion."""
    normalized = motion.strip().upper()
    if normalized.startswith("RECOVER_RIGHT"):
        recovery_side = "RIGHT"
    elif normalized.startswith("RECOVER_LEFT"):
        recovery_side = "LEFT"
    else:
        return None, None, None, None

    turn_prefix = f"RECOVER_{recovery_side}_"
    turn_motion, suffix, angle_deg = numbered_turn_motion_metadata(
        normalized.removeprefix(turn_prefix)
    )
    if turn_motion is not None:
        return recovery_side, turn_motion, suffix, angle_deg
    return recovery_side, None, None, None


class LineNavigationPlanner:
    """Create smooth line-following setpoints from ``/vision/line_info``."""

    def __init__(self, config: NavigationConfig | None = None) -> None:
        self.config = config or NavigationConfig()
        self.previous_motion = "STOP"
        self.previous_angular_speed_rad_s = 0.0
        self.turn_candidate: str | None = None
        self.turn_candidate_hits = 0

    def _reset_turn_state(self) -> None:
        """Reset confirmation state used by production Vision replay."""
        self.turn_candidate = None
        self.turn_candidate_hits = 0

    def stop(self, reason: str) -> NavigationCommand:
        """Create an immediate stop and reset steering state."""
        self.previous_motion = "STOP"
        self.previous_angular_speed_rad_s = 0.0
        self.turn_candidate = None
        self.turn_candidate_hits = 0
        return NavigationCommand(
            valid=False,
            motion="STOP",
            reason=reason,
            linear_speed_mps=0.0,
            lateral_speed_mps=0.0,
            angular_speed_rad_s=0.0,
            angular_accel_rad_s2=0.0,
            command_duration_sec=self.config.command_duration_sec,
            travel_distance_m=0.0,
            lateral_travel_distance_m=0.0,
            target_heading_change_deg=0.0,
            steering_error_deg=0.0,
            heading_component_deg=0.0,
            offset_component_deg=0.0,
            preview_component_deg=0.0,
            heading_error_deg=None,
            lateral_offset_norm=None,
            preview_turn_deg=None,
            line_quality=0.0,
            corner_prepare=False,
            corner_direction=None,
            corner_start_distance_m=None,
            corner_remaining_forward_m=None,
            corner_straight_motion_count=None,
        )

    def plan(
        self,
        line_info: dict[str, Any],
        dt_sec: float,
    ) -> NavigationCommand:
        """Generate one bounded command from a fresh line analysis sample."""
        if not bool(line_info.get("detected", False)):
            return self.stop("line_not_detected")

        heading = _number(line_info, "filtered_heading_error_deg")
        if heading is None:
            heading = _number(line_info, "heading_error_deg")

        offset = _number(line_info, "filtered_lateral_offset_norm")
        if offset is None:
            offset = _number(line_info, "lateral_offset_norm")

        if heading is None or offset is None:
            return self.stop("invalid_line_geometry")

        qualities = [
            _number(line_info, "heading_quality"),
            _number(line_info, "geometry_quality"),
            _number(line_info, "detection_quality"),
        ]
        valid_qualities = [value for value in qualities if value is not None]
        if not valid_qualities:
            return self.stop("invalid_line_quality")
        quality = min(valid_qualities)
        quality = _clamp(quality, 0.0, 1.0)
        if quality < self.config.min_line_quality:
            return self.stop("low_line_quality")

        preview_turn = _number(line_info, "turn_angle_deg")
        path_turn_delta = _number(line_info, "path_turn_delta_deg")
        turn_consistency = _number(line_info, "turn_consistency")
        preview_is_reliable = (
            preview_turn is not None
            and abs(preview_turn) >= self.config.preview_min_turn_deg
            and turn_consistency is not None
            and turn_consistency >= self.config.preview_min_consistency
        )
        path_turn_is_reliable = (
            path_turn_delta is not None
            and abs(path_turn_delta) >= self.config.preview_min_turn_deg
            and turn_consistency is not None
            and turn_consistency >= self.config.preview_min_consistency
        )
        curve_turn = (
            preview_turn
            if preview_is_reliable
            else path_turn_delta
            if path_turn_is_reliable
            else None
        )
        curve_is_reliable = curve_turn is not None
        direction_is_ambiguous = bool(
            curve_is_reliable
            and abs(heading) >= self.config.ambiguity_min_angle_deg
            and abs(curve_turn) >= self.config.ambiguity_min_angle_deg
            and heading * curve_turn < 0.0
        )
        curve_matches_heading = bool(
            curve_is_reliable
            and heading * curve_turn > 0.0
        )
        recovery_motion = (
            None
            if direction_is_ambiguous
            else self._classify_recovery(
                offset,
                heading,
                curve_matches_heading,
            )
        )
        if recovery_motion is not None:
            return self._recovery_command(
                recovery_motion,
                heading,
                offset,
                quality,
            )

        preview_component = curve_turn if curve_is_reliable else 0.0
        heading_component = self.config.heading_gain * heading
        offset_component = self.config.offset_gain_deg * offset
        preview_component = self.config.preview_gain * preview_component
        steering_error = (
            heading_component
            + offset_component
            + preview_component
        )
        if direction_is_ambiguous:
            steering_error = 0.0

        max_steering_deg = math.degrees(
            self.config.max_angular_speed_rad_s
            * self.config.steering_response_sec
        )
        steering_error = _clamp(
            steering_error,
            -max_steering_deg,
            max_steering_deg,
        )

        requested_motion = self._classify_motion(steering_error)
        curve_matches_requested_motion = bool(
            curve_is_reliable
            and (
                (requested_motion == "RIGHT" and curve_turn > 0.0)
                or (requested_motion == "LEFT" and curve_turn < 0.0)
            )
        )
        straight_line_turn_suppressed = bool(
            requested_motion in {"LEFT", "RIGHT"}
            and curve_is_reliable
            and not curve_matches_requested_motion
        )
        if straight_line_turn_suppressed:
            requested_motion = "STRAIGHT"
        turn_approach_pending = bool(
            (requested_motion == "RIGHT"
             and heading <= self.config.turn_min_heading_deg)
            or (requested_motion == "LEFT"
                and heading >= -self.config.turn_min_heading_deg)
        )
        if turn_approach_pending:
            requested_motion = "STRAIGHT"
        motion = self._confirm_motion(requested_motion)
        turn_confirmation_pending = bool(
            motion == "STRAIGHT" and requested_motion in {"LEFT", "RIGHT"}
        )
        control_steering_error = (
            0.0
            if turn_confirmation_pending or straight_line_turn_suppressed
            else steering_error
        )

        desired_angular_speed = math.radians(control_steering_error) / max(
            self.config.steering_response_sec,
            1e-3,
        )
        desired_angular_speed = _clamp(
            desired_angular_speed,
            -self.config.max_angular_speed_rad_s,
            self.config.max_angular_speed_rad_s,
        )

        dt_sec = _clamp(dt_sec, 1e-3, 1.0)
        max_delta = self.config.max_angular_accel_rad_s2 * dt_sec
        angular_delta = _clamp(
            desired_angular_speed - self.previous_angular_speed_rad_s,
            -max_delta,
            max_delta,
        )
        angular_speed = self.previous_angular_speed_rad_s + angular_delta
        angular_accel = angular_delta / dt_sec

        speed = self._calculate_linear_speed(steering_error, quality)
        reason = "line_tracking"
        if direction_is_ambiguous:
            speed = self.config.min_linear_speed_mps
            reason = "conflicting_heading_and_preview"
        elif straight_line_turn_suppressed:
            reason = "straight_line_turn_suppressed"
        elif turn_approach_pending:
            reason = "turn_approach_pending"
        elif turn_confirmation_pending:
            speed = self.config.min_linear_speed_mps
            reason = "turn_confirmation_pending"
        duration = self.config.command_duration_sec

        self.previous_motion = motion
        self.previous_angular_speed_rad_s = angular_speed

        return NavigationCommand(
            valid=True,
            motion=motion,
            reason=reason,
            linear_speed_mps=speed,
            lateral_speed_mps=0.0,
            angular_speed_rad_s=angular_speed,
            angular_accel_rad_s2=angular_accel,
            command_duration_sec=duration,
            travel_distance_m=speed * duration,
            lateral_travel_distance_m=0.0,
            target_heading_change_deg=math.degrees(angular_speed * duration),
            steering_error_deg=steering_error,
            heading_component_deg=heading_component,
            offset_component_deg=offset_component,
            preview_component_deg=preview_component,
            heading_error_deg=heading,
            lateral_offset_norm=offset,
            preview_turn_deg=curve_turn,
            line_quality=quality,
            corner_prepare=False,
            corner_direction=None,
            corner_start_distance_m=None,
            corner_remaining_forward_m=None,
            corner_straight_motion_count=None,
        )

    def _classify_recovery(
        self,
        lateral_offset_norm: float,
        heading_error_deg: float,
        curve_matches_heading: bool,
    ) -> str | None:
        """Classify line side and required turn as separate recovery axes."""
        is_recovering = self.previous_motion.startswith("RECOVER_")
        threshold = (
            self.config.recovery_exit_offset_norm
            if is_recovering
            else self.config.recovery_enter_offset_norm
        )
        if (
            curve_matches_heading
            and abs(lateral_offset_norm)
            < self.config.curve_follow_max_offset_norm
        ):
            return None
        inside_straight_corridor = bool(
            abs(lateral_offset_norm)
            <= self.config.recovery_straight_offset_norm
        )
        nearly_parallel = bool(
            abs(heading_error_deg)
            <= self.config.recovery_parallel_heading_deg
        )
        offset_requires_recovery = abs(lateral_offset_norm) > threshold
        heading_points_away = bool(
            offset_requires_recovery
            and lateral_offset_norm * heading_error_deg > 0.0
        )
        heading_points_toward_center = bool(
            offset_requires_recovery
            and lateral_offset_norm * heading_error_deg < 0.0
        )
        if (
            heading_points_toward_center
            and abs(heading_error_deg) >= self.config.turn_enter_deg
            and abs(lateral_offset_norm)
            < self.config.curve_follow_max_offset_norm
        ):
            return None
        active_heading_threshold = (
            self.config.recovery_away_heading_turn_deg
            if heading_points_away
            else self.config.recovery_heading_turn_deg
        )
        inside_active_heading_deadband = bool(
            abs(heading_error_deg) < active_heading_threshold
        )
        if (
            inside_straight_corridor
            and (nearly_parallel or inside_active_heading_deadband)
        ):
            return None

        heading_requires_recovery = bool(
            abs(heading_error_deg) >= active_heading_threshold
            and not curve_matches_heading
        )
        if not offset_requires_recovery and not heading_requires_recovery:
            return None

        if lateral_offset_norm > 0.0:
            line_side = "RIGHT"
        elif lateral_offset_norm < 0.0:
            line_side = "LEFT"
        else:
            line_side = "RIGHT" if heading_error_deg > 0.0 else "LEFT"

        if heading_error_deg >= active_heading_threshold:
            turn_motion = numbered_turn_motion(
                heading_error_deg,
                "RIGHT",
            )
            return f"RECOVER_{line_side}_{turn_motion}"
        if heading_error_deg <= -active_heading_threshold:
            turn_motion = numbered_turn_motion(
                heading_error_deg,
                "LEFT",
            )
            return f"RECOVER_{line_side}_{turn_motion}"
        # A lateral offset by itself must not emit a standalone recovery
        # motion.  Numbered recovery turns are reserved for a heading error
        # of at least the active side-aware heading threshold.
        return None

    def _recovery_command(
        self,
        motion: str,
        heading: float,
        offset: float,
        quality: float,
    ) -> NavigationCommand:
        """Create a slow sideways command to return to the line center."""
        line_side = "RIGHT" if motion.startswith("RECOVER_RIGHT") else "LEFT"
        direction = 1.0 if line_side == "RIGHT" else -1.0
        lateral_speed = direction * self.config.recovery_lateral_speed_mps
        _, _, turn_level, turn_angle_deg = _recovery_motion_metadata(motion)
        if "_TURN_RIGHT_" in motion:
            angular_speed = self.config.recovery_turn_speed_rad_s
        elif "_TURN_LEFT_" in motion:
            angular_speed = -self.config.recovery_turn_speed_rad_s
        else:
            angular_speed = 0.0
        duration = self.config.command_duration_sec
        self.previous_motion = motion
        self.previous_angular_speed_rad_s = angular_speed
        return NavigationCommand(
            valid=True,
            motion=motion,
            reason="line_center_recovery",
            linear_speed_mps=0.0,
            lateral_speed_mps=lateral_speed,
            angular_speed_rad_s=angular_speed,
            angular_accel_rad_s2=0.0,
            command_duration_sec=duration,
            travel_distance_m=0.0,
            lateral_travel_distance_m=lateral_speed * duration,
            target_heading_change_deg=(
                turn_angle_deg
                if turn_level is not None and turn_angle_deg is not None
                else math.degrees(angular_speed * duration)
            ),
            steering_error_deg=0.0,
            heading_component_deg=heading,
            offset_component_deg=self.config.offset_gain_deg * offset,
            preview_component_deg=0.0,
            heading_error_deg=heading,
            lateral_offset_norm=offset,
            preview_turn_deg=None,
            line_quality=quality,
            corner_prepare=False,
            corner_direction=None,
            corner_start_distance_m=None,
            corner_remaining_forward_m=None,
            corner_straight_motion_count=None,
        )

    def _classify_motion(self, steering_error_deg: float) -> str:
        """Classify with hysteresis so the command does not chatter."""
        threshold = (
            self.config.turn_exit_deg
            if self.previous_motion in {"LEFT", "RIGHT"}
            else self.config.turn_enter_deg
        )
        if steering_error_deg > threshold:
            return "RIGHT"
        if steering_error_deg < -threshold:
            return "LEFT"
        return "STRAIGHT"

    def _confirm_motion(self, requested_motion: str) -> str:
        """Require repeated LEFT/RIGHT observations before changing direction."""
        if requested_motion not in {"LEFT", "RIGHT"}:
            self.turn_candidate = None
            self.turn_candidate_hits = 0
            return "STRAIGHT"

        if requested_motion == self.previous_motion:
            self.turn_candidate = None
            self.turn_candidate_hits = 0
            return requested_motion

        if requested_motion == self.turn_candidate:
            self.turn_candidate_hits += 1
        else:
            self.turn_candidate = requested_motion
            self.turn_candidate_hits = 1

        if self.turn_candidate_hits >= max(
            1, self.config.direction_confirmation_frames
        ):
            self.turn_candidate = None
            self.turn_candidate_hits = 0
            return requested_motion
        return "STRAIGHT"

    def _calculate_linear_speed(
        self,
        steering_error_deg: float,
        quality: float,
    ) -> float:
        """Slow down for large turns and uncertain line geometry."""
        max_steering_deg = max(
            math.degrees(
                self.config.max_angular_speed_rad_s
                * self.config.steering_response_sec
            ),
            1e-3,
        )
        turn_scale = 1.0 - 0.70 * min(
            abs(steering_error_deg) / max_steering_deg,
            1.0,
        )
        quality_scale = 0.50 + 0.50 * quality
        speed = self.config.max_linear_speed_mps * turn_scale * quality_scale
        return _clamp(
            speed,
            self.config.min_linear_speed_mps,
            self.config.max_linear_speed_mps,
        )
