#!/usr/bin/env python3
"""Convert ball geometry into bounded, hardware-independent motion targets."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from .approach_distance import approach_level_from_motion
from .line_navigation_planner import numbered_turn_motion_metadata


@dataclass(frozen=True)
class BallNavigationConfig:
    """Tunable limits for ball alignment and approach commands."""

    min_confidence: float = 0.55
    max_linear_speed_mps: float = 0.04
    min_linear_speed_mps: float = 0.012
    max_angular_speed_rad_s: float = 0.50
    max_angular_accel_rad_s2: float = 1.00
    bearing_gain: float = 1.0
    steering_response_sec: float = 0.70
    turn_enter_deg: float = 5.0
    turn_exit_deg: float = 2.5
    path_center_deadband_norm: float = 0.05
    path_center_deadband_max_depth_m: float = 0.60
    fallback_half_fov_deg: float = 35.0
    control_start_depth_m: float = 1.50
    max_pickup_depth_age_sec: float = 0.70
    pickup_sequence_start_distance_m: float = 0.570
    slowdown_depth_m: float = 1.0
    fine_step_depth_m: float = 0.95
    pickup_depth_m: float = 0.07
    pickup_trigger_depth_m: float = 0.48
    pickup_depth_tolerance_m: float = 0.02
    command_duration_sec: float = 0.40


@dataclass(frozen=True)
class BallNavigationCommand:
    """One abstract ball-approach command for a behavior layer."""

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
    bearing_error_deg: float | None
    offset_x_norm: float | None
    depth_m: float | None
    distance_m: float | None
    ground_distance_m: float | None
    distance_error_m: float | None
    confidence: float
    depth_valid: bool
    pickup_ready: bool
    pickup_now: bool
    pickup_approach_motion: str | None

    def to_dict(self) -> dict[str, Any]:
        """Return a rounded JSON-compatible representation."""
        approach_motion = (
            self.pickup_approach_motion
            if self.motion == "PICKUP_NOW"
            else self.motion
        )
        approach_level = (
            approach_level_from_motion(approach_motion)
            if isinstance(approach_motion, str)
            else None
        )
        turn_motion, turn_level, turn_angle_deg = (
            numbered_turn_motion_metadata(self.motion)
        )
        return {
            "valid": self.valid,
            "motion": self.motion,
            "reason": self.reason,
            "linear_speed_mps": round(self.linear_speed_mps, 4),
            "lateral_speed_mps": round(self.lateral_speed_mps, 4),
            "angular_speed_rad_s": round(self.angular_speed_rad_s, 4),
            "angular_accel_rad_s2": round(
                self.angular_accel_rad_s2,
                4,
            ),
            "command_duration_sec": round(self.command_duration_sec, 3),
            "travel_distance_m": round(self.travel_distance_m, 4),
            "lateral_travel_distance_m": round(
                self.lateral_travel_distance_m,
                4,
            ),
            "target_heading_change_deg": round(
                self.target_heading_change_deg,
                3,
            ),
            "bearing_error_deg": _round_optional(
                self.bearing_error_deg,
                3,
            ),
            "offset_x_norm": _round_optional(self.offset_x_norm, 6),
            "depth_m": _round_optional(self.depth_m, 3),
            "distance_m": _round_optional(self.distance_m, 3),
            "ground_distance_m": _round_optional(
                self.ground_distance_m,
                3,
            ),
            "distance_error_m": _round_optional(
                self.distance_error_m,
                3,
            ),
            "confidence": round(self.confidence, 4),
            "depth_valid": self.depth_valid,
            "pickup_ready": self.pickup_ready,
            "pickup_now": self.pickup_now,
            "pickup_approach_motion": self.pickup_approach_motion,
            "approach_motion": (
                approach_motion
                if approach_motion == "STRAIGHT" or approach_level is not None
                else None
            ),
            "approach_level": approach_level,
            "approach_target_distance_m": _round_optional(
                self.distance_m,
                3,
            ),
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


class BallNavigationPlanner:
    """Create ball alignment and approach targets from ``ball_info``."""

    def __init__(
        self,
        config: BallNavigationConfig | None = None,
    ) -> None:
        self.config = config or BallNavigationConfig()
        self.previous_motion = "STOP"
        self.previous_angular_speed_rad_s = 0.0

    def stop(self, reason: str) -> BallNavigationCommand:
        """Create an immediate stop and reset steering state."""
        self.previous_motion = "STOP"
        self.previous_angular_speed_rad_s = 0.0
        return BallNavigationCommand(
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
            bearing_error_deg=None,
            offset_x_norm=None,
            depth_m=None,
            distance_m=None,
            ground_distance_m=None,
            distance_error_m=None,
            confidence=0.0,
            depth_valid=False,
            pickup_ready=False,
            pickup_now=False,
            pickup_approach_motion=None,
        )

    def plan(
        self,
        ball_info: dict[str, Any],
        dt_sec: float,
    ) -> BallNavigationCommand:
        """Generate one safe alignment, approach, or pickup target."""
        if not bool(ball_info.get("detected", False)):
            return self.stop("ball_not_detected")

        confidence = _number(ball_info, "confidence")
        if confidence is None or confidence < self.config.min_confidence:
            return self.stop("low_ball_confidence")

        steering_angle = _number(ball_info, "steering_angle_deg")
        bearing = _number(ball_info, "bearing_deg")
        offset = _number(ball_info, "offset_x_norm")
        depth = _number(ball_info, "depth_m")
        distance = _number(ball_info, "distance_m")
        ground_distance = _number(ball_info, "ground_distance_m")
        steering_error = self._steering_error(
            steering_angle,
            bearing,
            offset,
            distance,
        )
        if steering_error is None:
            return self.stop("invalid_ball_alignment")

        depth_valid = bool(ball_info.get("depth_valid", False))
        pickup_ready = bool(ball_info.get("pickup_ready", False))

        # depth_valid/depth_age describe the aligned sample's validity and
        # freshness; distance_m is the sole BALL range-control value.
        if (
            not depth_valid
            or distance is None
            or distance <= 0.0
        ):
            return self.stop("missing_valid_ball_distance")

        # pickup_ready gates only the later fixed grasp block. A fresh valid
        # distance independently owns entry into the atomic pickup sequence.
        if distance <= self.config.pickup_sequence_start_distance_m:
            depth_age = _number(ball_info, "depth_age_sec")
            if (
                depth_age is None
                or depth_age < 0.0
                or depth_age > self.config.max_pickup_depth_age_sec
            ):
                return self.stop("stale_ball_distance_for_pickup_sequence")

            return self._pickup_command(
                bearing,
                offset,
                depth,
                distance,
                ground_distance,
                confidence,
                pickup_ready,
            )

        speed = self._approach_speed(distance, pickup_ready)
        motion = self._general_approach_motion(
            distance,
        )
        return self._moving_command(
            motion=motion,
            reason="ball_aligned_discrete_approach",
            linear_speed_mps=speed,
            steering_error_deg=steering_error,
            dt_sec=dt_sec,
            bearing=bearing,
            offset=offset,
            depth=depth,
            distance=distance,
            ground_distance=ground_distance,
            confidence=confidence,
            depth_valid=True,
            pickup_ready=pickup_ready,
            pickup_now=False,
        )

    def _steering_error(
        self,
        steering_angle_deg: float | None,
        bearing_deg: float | None,
        offset_x_norm: float | None,
        distance_m: float | None,
    ) -> float | None:
        """Prefer the bottom-center path angle, then camera-ray fallbacks."""
        if steering_angle_deg is not None:
            if (
                offset_x_norm is not None
                and distance_m is not None
                and abs(offset_x_norm)
                <= self.config.path_center_deadband_norm
                and distance_m
                <= self.config.path_center_deadband_max_depth_m
            ):
                return 0.0
            return self.config.bearing_gain * steering_angle_deg
        if bearing_deg is not None:
            return self.config.bearing_gain * bearing_deg
        if offset_x_norm is None:
            return None
        return (
            self.config.bearing_gain
            * offset_x_norm
            * self.config.fallback_half_fov_deg
        )

    def _general_approach_motion(
        self,
        _distance_m: float,
    ) -> str:
        """Use the fixed four-repeat forward motion during BALL approach."""
        return "STRAIGHT_2"

    def _approach_speed(
        self,
        distance_m: float,
        pickup_ready: bool,
    ) -> float:
        """Reduce forward speed smoothly near pickup distance."""
        span = max(
            self.config.slowdown_depth_m - self.config.pickup_depth_m,
            1e-3,
        )
        scale = _clamp(
            (distance_m - self.config.pickup_depth_m) / span,
            0.0,
            1.0,
        )
        speed = self.config.min_linear_speed_mps + scale * (
            self.config.max_linear_speed_mps
            - self.config.min_linear_speed_mps
        )
        if pickup_ready:
            speed = min(speed, self.config.min_linear_speed_mps)
        return _clamp(
            speed,
            self.config.min_linear_speed_mps,
            self.config.max_linear_speed_mps,
        )

    def _limited_angular_speed(
        self,
        steering_error_deg: float,
        dt_sec: float,
    ) -> tuple[float, float]:
        """Convert steering error to an acceleration-limited angular speed."""
        desired = math.radians(steering_error_deg) / max(
            self.config.steering_response_sec,
            1e-3,
        )
        desired = _clamp(
            desired,
            -self.config.max_angular_speed_rad_s,
            self.config.max_angular_speed_rad_s,
        )
        dt_sec = _clamp(dt_sec, 1e-3, 1.0)
        max_delta = self.config.max_angular_accel_rad_s2 * dt_sec
        delta = _clamp(
            desired - self.previous_angular_speed_rad_s,
            -max_delta,
            max_delta,
        )
        speed = self.previous_angular_speed_rad_s + delta
        return speed, delta / dt_sec

    def _moving_command(
        self,
        *,
        motion: str,
        reason: str,
        linear_speed_mps: float,
        steering_error_deg: float,
        dt_sec: float,
        bearing: float | None,
        offset: float | None,
        depth: float | None,
        distance: float | None,
        ground_distance: float | None,
        confidence: float,
        depth_valid: bool,
        pickup_ready: bool,
        pickup_now: bool,
    ) -> BallNavigationCommand:
        """Build one moving command and retain steering state."""
        angular_speed, angular_accel = self._limited_angular_speed(
            steering_error_deg,
            dt_sec,
        )
        duration = self.config.command_duration_sec
        self.previous_motion = motion
        self.previous_angular_speed_rad_s = angular_speed
        distance_error = (
            distance - self.config.pickup_depth_m
            if distance is not None
            else None
        )
        _, _, numbered_turn_angle_deg = numbered_turn_motion_metadata(
            motion
        )
        return BallNavigationCommand(
            valid=True,
            motion=motion,
            reason=reason,
            linear_speed_mps=linear_speed_mps,
            lateral_speed_mps=0.0,
            angular_speed_rad_s=angular_speed,
            angular_accel_rad_s2=angular_accel,
            command_duration_sec=duration,
            travel_distance_m=linear_speed_mps * duration,
            lateral_travel_distance_m=0.0,
            target_heading_change_deg=(
                numbered_turn_angle_deg
                if numbered_turn_angle_deg is not None
                else math.degrees(angular_speed * duration)
            ),
            bearing_error_deg=bearing,
            offset_x_norm=offset,
            depth_m=depth,
            distance_m=distance,
            ground_distance_m=ground_distance,
            distance_error_m=distance_error,
            confidence=confidence,
            depth_valid=depth_valid,
            pickup_ready=pickup_ready,
            pickup_now=pickup_now,
            pickup_approach_motion=None,
        )

    def _pickup_command(
        self,
        bearing: float | None,
        offset: float | None,
        depth: float,
        distance: float | None,
        ground_distance: float | None,
        confidence: float,
        pickup_ready: bool,
    ) -> BallNavigationCommand:
        """Hold general approach and request the atomic pickup sequence."""
        self.previous_motion = "PICKUP_NOW"
        self.previous_angular_speed_rad_s = 0.0
        return BallNavigationCommand(
            valid=True,
            motion="PICKUP_NOW",
            reason="ball_distance_entered_pickup_sequence",
            linear_speed_mps=0.0,
            lateral_speed_mps=0.0,
            angular_speed_rad_s=0.0,
            angular_accel_rad_s2=0.0,
            command_duration_sec=self.config.command_duration_sec,
            travel_distance_m=0.0,
            lateral_travel_distance_m=0.0,
            target_heading_change_deg=0.0,
            bearing_error_deg=bearing,
            offset_x_norm=offset,
            depth_m=depth,
            distance_m=distance,
            ground_distance_m=ground_distance,
            distance_error_m=distance - self.config.pickup_depth_m,
            confidence=confidence,
            depth_valid=True,
            pickup_ready=pickup_ready,
            pickup_now=True,
            # Distance approach is deliberately selected later from a fresh
            # frame, after the heading-alignment checkpoint.
            pickup_approach_motion=None,
        )
