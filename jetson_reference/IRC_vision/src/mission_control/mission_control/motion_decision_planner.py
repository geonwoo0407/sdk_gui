#!/usr/bin/env python3
"""Select one mission command from the existing navigation planners."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from step.approach_distance import approach_level_from_motion
from step.ball_navigation_planner import BallNavigationConfig
from step.ball_navigation_planner import BallNavigationPlanner
from step.goal_navigation_planner import GoalNavigationPlanner
from step.hurdle_navigation_planner import HurdleNavigationPlanner
from step.line_navigation_planner import LineNavigationPlanner
from step.line_navigation_planner import NavigationConfig

from .hurdle_line_fusion import build_hurdle_path_reference


@dataclass(frozen=True)
class MotionDecision:
    """One normalized command selected from a mission-specific planner."""

    phase: str
    source: str
    action: str
    valid: bool
    reason: str
    sdk_motion_requested: bool
    requires_ack: bool
    source_command: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-compatible representation."""
        return {
            "phase": self.phase,
            "source": self.source,
            "action": self.action,
            "valid": self.valid,
            "reason": self.reason,
            "sdk_motion_requested": self.sdk_motion_requested,
            "requires_ack": self.requires_ack,
            "source_command": self.source_command,
        }


@dataclass(frozen=True)
class MotionDecisionConfig:
    """Tunable mission-selection and lost-ball recovery limits."""

    enable_ball_lost_recovery: bool = False
    recovery_heading_turn_deg: float = 10.0
    recovery_away_heading_turn_deg: float = 3.0
    curve_follow_max_offset_norm: float = 0.55
    ball_tracking_range_m: float = 1.5
    ball_control_range_m: float = 1.5
    pickup_fine_step_bottom_distance_px: int = 500
    ball_lost_stop_sec: float = 0.35
    ball_recovery_timeout_sec: float = 8.0
    ball_recovery_turn_rad_s: float = 0.22
    ball_recovery_command_sec: float = 0.40
    ball_recovery_direction_deadband_deg: float = 1.0
    ball_reacquire_center_deg: float = 5.0
    ball_reacquire_center_norm: float = 0.08
    goal_tracking_range_m: float = 1.0
    goal_control_range_m: float = 0.5
    hurdle_control_range_m: float = 1.0
    hurdle_path_reference_hold_sec: float = 0.50
    goal_lost_stop_sec: float = 0.35
    goal_recovery_timeout_sec: float = 8.0
    goal_recovery_turn_rad_s: float = 0.22
    goal_recovery_command_sec: float = 0.40
    goal_reacquire_center_deg: float = 5.0
    goal_reacquire_center_norm: float = 0.10


class MotionDecisionPlanner:
    """Run one existing planner according to the active mission phase."""

    AUTO_PRIORITY = ("line",)
    NON_EXECUTABLE_ACTIONS = frozenset(
        {
            "STRAIGHT_0",
            "BALL_LOST_STOP",
            "GOAL_LOST_STOP",
            "HEAD_SCAN_LEFT",
            "HEAD_SCAN_RIGHT",
            "HEAD_CENTER",
            "WAIT_SCORE_CONFIRMATION",
            "WAIT_GO_CONFIRMATION",
        }
    )
    TERMINAL_ACTIONS = {
        ("ball", "PICKUP_NOW"),
        ("goal", "SHOT"),
        ("hurdle", "GO"),
    }
    STRICT_BOOLEAN_FIELDS = {
        "ball": ("pickup_ready", "pickup_now"),
        "goal": ("score_now",),
        "hurdle": ("depth_valid",),
        "line": ("detected",),
    }
    TURN_REPEAT_DEG = 10.0
    MAX_TURN_REPEAT_COUNT = 9
    PICKUP_INITIAL_HEADING_TOLERANCE_DEG = 7.0
    PICKUP_INITIAL_CENTER_BOUND_PX = 70.0
    PICKUP_CLOSE_CRAB_BOTTOM_DISTANCE_PX = 120.0
    BALL_APPROACH_LEFT_TURN_REPEAT_DEG = 15.0
    BALL_APPROACH_LEFT_MAX_TURN_REPEAT_COUNT = 6
    PICKUP_FINE_LEFT_BOUND_PX = -30.0
    PICKUP_FINE_RIGHT_BOUND_PX = 50.0
    POST_BALL_LINE_LEFT_COUNTS = frozenset({2, 3, 4, 6})
    BALL_APPROACH_LEFT_COUNTS = (2, 3, 4, 5, 6)
    BALL_LOST_LINE_LEFT_TURN_COUNT = 6
    BALL_LOST_LINE_RIGHT_TURN_COUNT = 9

    def __init__(
        self,
        config: MotionDecisionConfig | None = None,
    ) -> None:
        self.config = config or MotionDecisionConfig()
        self.line_planner = LineNavigationPlanner(
            NavigationConfig(
                recovery_heading_turn_deg=(
                    self.config.recovery_heading_turn_deg
                ),
                recovery_away_heading_turn_deg=(
                    self.config.recovery_away_heading_turn_deg
                ),
                curve_follow_max_offset_norm=(
                    self.config.curve_follow_max_offset_norm
                ),
            )
        )
        self.ball_planner = BallNavigationPlanner(
            BallNavigationConfig(
                control_start_depth_m=self.config.ball_control_range_m,
            )
        )
        self.goal_planner = GoalNavigationPlanner()
        self.hurdle_planner = HurdleNavigationPlanner()
        self.previous_source = "none"
        self.ball_tracking_active = False
        self.ball_recovery_centering = False
        self.ball_lost_elapsed_sec = 0.0
        self.last_ball_bearing_deg: float | None = None
        self.last_ball_offset_x_norm: float | None = None
        self.last_ball_turn_direction = "RIGHT"
        self.last_ball_line_offset_norm: float | None = None
        self.last_ball_line_side: str | None = None
        self.ball_lock_active = False
        self.ball_terminal_requested = False
        self.ball_ignore_until_clear = False
        self.goal_tracking_active = False
        self.goal_recovery_centering = False
        self.goal_lost_elapsed_sec = 0.0
        self.last_goal_bearing_deg: float | None = None
        self.last_goal_offset_x_norm: float | None = None
        self.last_goal_turn_direction = "RIGHT"
        self.goal_lock_active = False
        self.goal_terminal_requested = False
        self.goal_ignore_until_clear = False
        self.hurdle_lock_active = False
        self.hurdle_go_requested = False
        self.hurdle_ignore_until_clear = False
        self.last_hurdle_path_reference: dict[str, Any] | None = None
        self.hurdle_path_reference_age_sec = 0.0

    @staticmethod
    def source_for_phase(phase: str) -> str | None:
        """Map a mission phase name to the sensor that owns that phase."""
        normalized = phase.strip().upper()
        if normalized == "AUTO":
            return None
        if normalized == "POST_BALL_LINE_ALIGN":
            return "line"
        if normalized.startswith("BALL") or normalized.startswith("PICK"):
            return "ball"
        if (
            normalized.startswith("GOAL")
            or normalized.startswith("SHOOT")
            or normalized.startswith("SCORE_GOAL")
        ):
            return "goal"
        if normalized.startswith("HURDLE") or normalized.startswith("JUMP"):
            return "hurdle"
        if (
            normalized.startswith("LINE")
            or normalized.startswith("FOLLOW_LINE")
            or normalized == "FINISH"
        ):
            return "line"
        return "none"

    def approach_phase_for_search(
        self,
        phase: str,
        observations: dict[str, dict[str, Any] | None],
    ) -> str | None:
        """Return an approach phase only for a fresh controllable target."""
        normalized = phase.strip().upper()
        if (
            normalized == "BALL_SEARCH"
            and self._ball_is_inside_control_range(observations.get("ball"))
        ):
            return "BALL_APPROACH"
        if (
            normalized == "GOAL_SEARCH"
            and self._goal_is_inside_control_range(observations.get("goal"))
        ):
            return "GOAL_APPROACH"
        return None

    def plan(
        self,
        phase: str,
        observations: dict[str, dict[str, Any] | None],
        dt_sec: float,
    ) -> MotionDecision:
        """Select a source and normalize its planner-specific command."""
        normalized_phase = phase.strip().upper() or "AUTO"
        invalid_source = self._invalid_boolean_observation_source(
            observations
        )
        if invalid_source is not None:
            return MotionDecision(
                phase=normalized_phase,
                source=invalid_source,
                action="WAIT",
                valid=False,
                reason="invalid_vision_boolean_type",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command={},
            )
        if normalized_phase == "POST_BALL_LINE_ALIGN":
            return self._plan_post_ball_line_align(observations.get("line"))
        self._update_ball_recovery_line_tracking(observations.get("line"))
        self._update_ball_tracking(observations.get("ball"), dt_sec)
        self._update_goal_tracking(observations.get("goal"), dt_sec)
        self._update_hurdle_lock(
            normalized_phase,
            observations.get("hurdle"),
        )
        self._update_object_locks(normalized_phase, observations)
        if (
            normalized_phase.endswith("_LOCK")
            and self.source_for_phase(normalized_phase) != "line"
        ):
            self._reset_previous_source()
            return MotionDecision(
                phase=normalized_phase,
                source="none",
                action="WAIT",
                valid=False,
                reason="mission_locked_waiting_for_motion_status",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command={},
            )

        source = self._select_source(normalized_phase, observations)
        if source == "none":
            self._reset_previous_source()
            return MotionDecision(
                phase=normalized_phase,
                source="none",
                action="WAIT",
                valid=False,
                reason="no_fresh_detected_target",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command={},
            )

        if source != self.previous_source:
            self._reset_source(source)
        self.previous_source = source
        info = observations.get(source)
        hurdle_reference: dict[str, Any] | None = None
        if source == "hurdle":
            info, hurdle_reference = self._hurdle_observation_with_path(
                info,
                observations.get("line"),
                dt_sec,
            )
        command = self._plan_source(source, info, dt_sec)
        if hurdle_reference is not None:
            command.update(hurdle_reference)
        action_key = "motion" if source in {"line", "ball"} else "action"
        action = str(command.get(action_key, "WAIT"))
        if source == "ball" and action == "STRAIGHT_0":
            command = dict(command)
            command["semantic_motion"] = "STRAIGHT_0"
            action = "BALL_FINE_FORWARD_8"
        valid = bool(command.get("valid", False))
        if action in self.NON_EXECUTABLE_ACTIONS:
            valid = False
            command = dict(command)
            command["valid"] = False
            command["sdk_motion_requested"] = False
        terminal = (source, action) in self.TERMINAL_ACTIONS
        requested = terminal and bool(
            command.get("sdk_motion_requested", terminal)
        )
        if source == "hurdle" and action == "GO":
            self.hurdle_go_requested = True
        elif source == "ball" and action == "PICKUP_NOW":
            self.ball_terminal_requested = True
        elif source == "goal" and action == "SHOT":
            self.goal_terminal_requested = True
        return MotionDecision(
            phase=normalized_phase,
            source=source,
            action=action,
            valid=valid,
            reason=str(command.get("reason", "unknown")),
            sdk_motion_requested=requested,
            requires_ack=terminal,
            source_command=command,
        )

    @classmethod
    def _invalid_boolean_observation_source(
        cls,
        observations: dict[str, dict[str, Any] | None],
    ) -> str | None:
        """Return the source containing a present non-boolean safety field."""
        for source, fields in cls.STRICT_BOOLEAN_FIELDS.items():
            info = observations.get(source)
            if info is None:
                continue
            for field in fields:
                if field in info and not isinstance(info[field], bool):
                    return source
        return None

    def _hurdle_observation_with_path(
        self,
        hurdle_info: dict[str, Any] | None,
        line_info: dict[str, Any] | None,
        dt_sec: float,
    ) -> tuple[dict[str, Any] | None, dict[str, Any]]:
        """Attach hurdle-owned path geometry without running line planner."""
        reference = build_hurdle_path_reference(hurdle_info, line_info)
        if bool(reference.get("path_reference_valid", False)):
            self.last_hurdle_path_reference = dict(reference)
            self.hurdle_path_reference_age_sec = 0.0
        else:
            self.hurdle_path_reference_age_sec += max(0.0, dt_sec)
            if (
                hurdle_info is not None
                and bool(hurdle_info.get("detected", False))
                and self.last_hurdle_path_reference is not None
                and self.hurdle_path_reference_age_sec
                <= self.config.hurdle_path_reference_hold_sec
            ):
                reference = dict(self.last_hurdle_path_reference)
                reference["path_reference_source"] = "held"
                reference["path_reference_reason"] = (
                    "temporarily_held_line_hurdle_intersection"
                )
                reference["path_reference_age_sec"] = round(
                    self.hurdle_path_reference_age_sec,
                    3,
                )

        if hurdle_info is None:
            return None, reference
        enriched = dict(hurdle_info)
        enriched.update(reference)
        return enriched, reference

    def _update_hurdle_lock(
        self,
        phase: str,
        hurdle_info: dict[str, Any] | None,
    ) -> None:
        """Latch hurdle ownership until an acknowledged jump changes phase."""
        requested = self.source_for_phase(phase)
        post_jump_phase_change = bool(
            self.hurdle_lock_active
            and self.hurdle_go_requested
            and requested in {"line", "ball", "goal"}
            and not phase.endswith("_LOCK")
        )
        if phase in {"HURDLE_DONE", "JUMP_DONE", "POST_HURDLE"}:
            post_jump_phase_change = True
        if post_jump_phase_change:
            self.hurdle_lock_active = False
            self.hurdle_go_requested = False
            self.hurdle_ignore_until_clear = True
            self.last_hurdle_path_reference = None
            self.hurdle_path_reference_age_sec = 0.0
            return
        if self.hurdle_ignore_until_clear:
            if not self._confirmed_hurdle(hurdle_info):
                self.hurdle_ignore_until_clear = False
            return
        if self._confirmed_hurdle(hurdle_info):
            self.hurdle_lock_active = True

    def _update_object_locks(
        self,
        phase: str,
        observations: dict[str, dict[str, Any] | None],
    ) -> None:
        """Keep ball/goal ownership after control-range mission entry."""
        requested = self.source_for_phase(phase)
        explicit_next_source = (
            requested
            if requested in {"line", "ball", "goal", "hurdle"}
            and not phase.endswith("_LOCK")
            else None
        )

        if (
            self.ball_lock_active
            and self.ball_terminal_requested
            and explicit_next_source not in {None, "ball"}
        ):
            self.ball_lock_active = False
            self.ball_terminal_requested = False
            self.ball_ignore_until_clear = True
        if (
            self.goal_lock_active
            and self.goal_terminal_requested
            and explicit_next_source not in {None, "goal"}
        ):
            self.goal_lock_active = False
            self.goal_terminal_requested = False
            self.goal_ignore_until_clear = True

        ball_info = observations.get("ball")
        goal_info = observations.get("goal")
        if self.ball_ignore_until_clear:
            if not self._ball_is_inside_control_range(ball_info):
                self.ball_ignore_until_clear = False
        if self.goal_ignore_until_clear:
            if not self._goal_is_inside_control_range(goal_info):
                self.goal_ignore_until_clear = False

        if self.ball_lock_active or self.goal_lock_active:
            return
        if self.hurdle_lock_active:
            return
        phase_allows_ball = requested in {None, "line", "ball"}
        phase_allows_goal = requested in {None, "line", "goal"}
        if (
            phase_allows_ball
            and not self.ball_ignore_until_clear
            and self._ball_is_inside_control_range(ball_info)
        ):
            self.ball_lock_active = True
        elif (
            phase_allows_goal
            and not self.goal_ignore_until_clear
            and self._goal_is_inside_control_range(goal_info)
        ):
            self.goal_lock_active = True

    def _select_source(
        self,
        phase: str,
        observations: dict[str, dict[str, Any] | None],
    ) -> str:
        if self.hurdle_lock_active:
            return "hurdle"
        if self.ball_lock_active:
            return "ball"
        if self.goal_lock_active:
            return "goal"
        requested = self.source_for_phase(phase)
        if requested is None:
            return self._select_auto_source(observations)
        if requested == "none":
            return "none"
        if phase.endswith("_SEARCH"):
            target = observations.get(requested)
            if requested == "ball":
                if self._ball_is_inside_control_range(target):
                    return "ball"
            elif requested == "goal":
                if self._goal_is_inside_control_range(target):
                    return "goal"
            elif target is not None and bool(target.get("detected", False)):
                return requested
            line = observations.get("line")
            if line is not None and bool(line.get("detected", False)):
                return "line"
            return "none"
        return requested

    def _select_auto_source(
        self,
        observations: dict[str, dict[str, Any] | None],
    ) -> str:
        if self.hurdle_lock_active:
            return "hurdle"
        if self.ball_lock_active:
            return "ball"
        if self.goal_lock_active:
            return "goal"
        ball = observations.get("ball")
        if self._ball_is_inside_control_range(ball):
            return "ball"
        goal = observations.get("goal")
        if self._goal_is_inside_control_range(goal):
            return "goal"
        for source in self.AUTO_PRIORITY:
            info = observations.get(source)
            if info is not None and bool(info.get("detected", False)):
                return source
        return "none"

    def _confirmed_hurdle(self, info: dict[str, Any] | None) -> bool:
        """Return true for a confirmed hurdle inside its control range."""
        if info is None or not bool(info.get("detected", False)):
            return False
        if (
            "confirmation_confirmed" in info
            and not bool(info.get("confirmation_confirmed", False))
        ):
            return False
        depth = self._number(info, "depth_m")
        return bool(
            info.get("depth_valid", False)
            and depth is not None
            and depth <= self.config.hurdle_control_range_m
        )

    def _plan_source(
        self,
        source: str,
        info: dict[str, Any] | None,
        dt_sec: float,
    ) -> dict[str, Any]:
        if source == "line":
            command = (
                self.line_planner.stop("waiting_for_line_info")
                if info is None
                else self.line_planner.plan(info, dt_sec)
            )
        elif source == "ball":
            if (
                self.config.enable_ball_lost_recovery
                and not self._is_detected_ball(info)
                and self.ball_tracking_active
            ):
                return self._lost_ball_recovery_command()
            command = (
                self.ball_planner.stop("waiting_for_ball_info")
                if info is None
                else self.ball_planner.plan(info, dt_sec)
            )
        elif source == "goal":
            if not self._is_detected_goal(info) and self.goal_tracking_active:
                return self._lost_goal_recovery_command()
            alignment = self._goal_camera90_alignment_command(info)
            if alignment is not None:
                return alignment
            lateral = self._goal_camera90_lateral_command(info)
            if lateral is not None:
                return lateral
            if self.goal_recovery_centering and self._is_detected_goal(info):
                self.goal_recovery_centering = False
            command = (
                self.goal_planner.wait("waiting_for_goal_info")
                if info is None
                else self.goal_planner.plan(info)
            )
            goal_action = command.action
            analyzer_requests_forward = bool(
                goal_action == "WAIT_SCORE_CONFIRMATION"
                and info is not None
                and info.get("depth_in_score_range") is False
            )
            close_safe_hold = bool(
                goal_action == "RETREAT_GOAL"
                and info is not None
                and (self._number(info, "depth_m") or math.inf) <= 0.130
            )
            if (
                self._goal_needs_camera_90_approach(info)
                or goal_action.startswith("STRAIGHT")
                or analyzer_requests_forward
                or close_safe_hold
            ):
                return self._goal_camera_90_approach_command(info)
        else:
            command = (
                self.hurdle_planner.wait("waiting_for_hurdle_info")
                if info is None
                else self.hurdle_planner.plan(info)
            )
        return command.to_dict()

    @classmethod
    def _turn_repeat_count(cls, angle_deg: float) -> int:
        """Round an absolute angle to the documented 10-degree repeat."""
        count = int(math.floor(abs(angle_deg) / cls.TURN_REPEAT_DEG + 0.5))
        return max(1, min(cls.MAX_TURN_REPEAT_COUNT, count))

    def plan_ball_approach_alignment(
        self,
        info: dict[str, Any] | None,
    ) -> MotionDecision:
        """Choose one ordinary stationary turn from a fresh Ball sample."""
        phase = "BALL_APPROACH_ALIGN"
        if info is None or info.get("detected") is not True:
            return MotionDecision(
                phase=phase,
                source="ball",
                action="WAIT",
                valid=False,
                reason="ball_approach_alignment_waiting_for_ball",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command={},
            )

        confidence = self._number(info, "confidence")
        distance = self._number(info, "distance_m")
        steering_error = self.ball_planner._steering_error(
            self._number(info, "steering_angle_deg"),
            self._number(info, "bearing_deg"),
            self._number(info, "offset_x_norm"),
            distance,
        )
        if (
            confidence is None
            or confidence < self.ball_planner.config.min_confidence
            or steering_error is None
        ):
            return MotionDecision(
                phase=phase,
                source="ball",
                action="WAIT",
                valid=False,
                reason="invalid_ball_approach_alignment_input",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command={},
            )

        tolerance = self.ball_planner.config.turn_enter_deg
        common = {
            "steering_error_deg": steering_error,
            "heading_tolerance_deg": tolerance,
            "distance_m": distance,
            "confidence": confidence,
        }
        if abs(steering_error) <= tolerance:
            return MotionDecision(
                phase=phase,
                source="ball",
                action="BALL_APPROACH_ALIGNED",
                valid=True,
                reason="ball_approach_heading_aligned",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command=common,
            )

        direction = "RIGHT" if steering_error > 0.0 else "LEFT"
        turn_repeat_deg = self.TURN_REPEAT_DEG
        requested_count = self._turn_repeat_count(steering_error)
        count = requested_count
        if direction == "LEFT":
            turn_repeat_deg = self.BALL_APPROACH_LEFT_TURN_REPEAT_DEG
            requested_count = int(
                math.floor(abs(steering_error) / turn_repeat_deg + 0.5)
            )
            requested_count = max(
                1,
                min(
                    self.BALL_APPROACH_LEFT_MAX_TURN_REPEAT_COUNT,
                    requested_count,
                ),
            )
            count = min(
                self.BALL_APPROACH_LEFT_COUNTS,
                key=lambda supported: abs(supported - requested_count),
            )
        return MotionDecision(
            phase=phase,
            source="ball",
            action=f"BALL_APPROACH_TURN_{direction}_{count}",
            valid=True,
            reason="ball_approach_stationary_heading_correction",
            sdk_motion_requested=False,
            requires_ack=False,
            source_command={
                **common,
                "turn_direction": direction,
                "turn_count": count,
                "requested_turn_count": requested_count,
                "turn_repeat_deg": turn_repeat_deg,
                "turn_angle_deg": count * turn_repeat_deg,
                "catalog_motion_available": True,
            },
        )

    def plan_lost_ball_approach_alignment(
        self,
        info: dict[str, Any] | None,
    ) -> MotionDecision:
        """Turn about 90 degrees toward the remembered Line side."""
        phase = "BALL_APPROACH_LOST_ALIGN"
        if info is None or info.get("detected") is not True:
            return MotionDecision(
                phase=phase,
                source="ball",
                action="WAIT",
                valid=False,
                reason="lost_ball_alignment_has_no_remembered_line",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command={},
            )

        line_offset = self._number(info, "filtered_lateral_offset_norm")
        if line_offset is None:
            line_offset = self._number(info, "lateral_offset_norm")
        if line_offset is None or line_offset == 0.0:
            return MotionDecision(
                phase=phase,
                source="ball",
                action="WAIT",
                valid=False,
                reason="lost_ball_alignment_has_no_line_side",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command={},
            )

        direction = "RIGHT" if line_offset > 0.0 else "LEFT"
        if direction == "RIGHT":
            count = self.BALL_LOST_LINE_RIGHT_TURN_COUNT
            turn_repeat_deg = self.TURN_REPEAT_DEG
        else:
            count = self.BALL_LOST_LINE_LEFT_TURN_COUNT
            turn_repeat_deg = self.BALL_APPROACH_LEFT_TURN_REPEAT_DEG
        return MotionDecision(
            phase=phase,
            source="ball",
            action=f"BALL_APPROACH_TURN_{direction}_{count}",
            valid=True,
            reason="ball_lost_turn_toward_last_line_side",
            sdk_motion_requested=False,
            requires_ack=False,
            source_command={
                "line_lateral_offset_norm": line_offset,
                "line_side": direction,
                "turn_direction": direction,
                "turn_count": count,
                "turn_repeat_deg": turn_repeat_deg,
                "turn_angle_deg": count * turn_repeat_deg,
                "lost_ball_alignment_from_memory": True,
                "alignment_reference": "line_side",
                "catalog_motion_available": True,
            },
        )

    def _plan_post_ball_line_align(
        self,
        info: dict[str, Any] | None,
    ) -> MotionDecision:
        """Select one fixed-pose correction from one fresh Line sample."""
        phase = "POST_BALL_LINE_ALIGN"
        if info is None or info.get("detected") is not True:
            return MotionDecision(
                phase=phase,
                source="line",
                action="WAIT",
                valid=False,
                reason="post_ball_line_not_detected",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command={},
            )

        heading = self._number(info, "filtered_heading_error_deg")
        if heading is None:
            heading = self._number(info, "heading_error_deg")
        offset = self._number(info, "filtered_lateral_offset_norm")
        if offset is None:
            offset = self._number(info, "lateral_offset_norm")
        qualities = [
            self._number(info, key)
            for key in (
                "heading_quality",
                "geometry_quality",
                "detection_quality",
            )
        ]
        valid_qualities = [value for value in qualities if value is not None]
        min_quality = self.line_planner.config.min_line_quality
        if (
            heading is None
            or offset is None
            or not valid_qualities
            or min(valid_qualities) < min_quality
        ):
            return MotionDecision(
                phase=phase,
                source="line",
                action="WAIT",
                valid=False,
                reason="invalid_post_ball_line_alignment_input",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command={},
            )

        heading_tolerance = self.line_planner.config.turn_enter_deg
        offset_tolerance = self.line_planner.config.recovery_exit_offset_norm
        common = {
            "heading_error_deg": heading,
            "lateral_offset_norm": offset,
            "heading_tolerance_deg": heading_tolerance,
            "offset_tolerance_norm": offset_tolerance,
            "offset_in_tolerance": abs(offset) <= offset_tolerance,
        }
        if abs(heading) <= heading_tolerance:
            return MotionDecision(
                phase=phase,
                source="line",
                action="POST_BALL_LINE_ALIGNED",
                valid=True,
                reason="post_ball_line_heading_aligned",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command=common,
            )

        direction = "RIGHT" if heading > 0.0 else "LEFT"
        count = self._turn_repeat_count(heading)
        action = f"POST_BALL_LINE_TURN_{direction}_{count}"
        available = bool(
            direction == "RIGHT"
            or count in self.POST_BALL_LINE_LEFT_COUNTS
        )
        command = {
            **common,
            "turn_direction": direction,
            "turn_count": count,
            "turn_angle_deg": count * self.TURN_REPEAT_DEG,
            "catalog_motion_available": available,
        }
        return MotionDecision(
            phase=phase,
            source="line",
            action=action,
            valid=available,
            reason=(
                "post_ball_line_heading_correction"
                if available
                else "post_ball_line_left_turn_not_available"
            ),
            sdk_motion_requested=False,
            requires_ack=False,
            source_command=command,
        )

    def plan_ball_pickup_fine_alignment(
        self,
        info: dict[str, Any] | None,
    ) -> MotionDecision:
        """Choose one lateral pickup correction from a fresh Ball sample."""
        phase = "BALL_PICKUP_FINE_ALIGN"
        if info is None or info.get("detected") is not True:
            return MotionDecision(
                phase=phase,
                source="ball",
                action="WAIT",
                valid=False,
                reason="ball_pickup_fine_alignment_waiting_for_ball",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command={},
            )

        confidence = self._number(info, "confidence")
        offset = self._number(info, "offset_x_norm")
        robot_center_offset_px = self._number(
            info,
            "offset_x_px",
        )
        depth = self._number(info, "depth_m")
        distance = self._number(info, "distance_m")
        depth_valid = info.get("depth_valid")
        depth_age = self._number(info, "depth_age_sec")
        pickup_ready = info.get("pickup_ready")
        in_pickup_window = info.get("is_in_pickup_window")
        if (
            confidence is None
            or confidence < self.ball_planner.config.min_confidence
            or robot_center_offset_px is None
        ):
            return MotionDecision(
                phase=phase,
                source="ball",
                action="WAIT",
                valid=False,
                reason="invalid_ball_pickup_fine_alignment_input",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command={},
            )

        common = {
            "offset_x_norm": offset,
            "offset_x_px": robot_center_offset_px,
            "pickup_left_bound_px": self.PICKUP_FINE_LEFT_BOUND_PX,
            "pickup_right_bound_px": self.PICKUP_FINE_RIGHT_BOUND_PX,
            "confidence": confidence,
            "depth_m": depth,
            "distance_m": distance,
            "depth_valid": depth_valid,
            "depth_age_sec": depth_age,
            "pickup_ready": pickup_ready,
            "is_in_pickup_window": in_pickup_window,
        }
        # BallAnalyzer computes this offset from the calibrated robot center;
        # positive therefore means the ball is to the screen-right.
        if (
            self.PICKUP_FINE_LEFT_BOUND_PX
            <= robot_center_offset_px
            <= self.PICKUP_FINE_RIGHT_BOUND_PX
        ):
            return MotionDecision(
                phase=phase,
                source="ball",
                action="BALL_PICKUP_FINE_ALIGN_CONTINUE",
                valid=True,
                reason="ball_pickup_fine_motion_complete_and_centered",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command={
                    **common,
                    "lateral_direction": "CENTERED",
                    "catalog_motion_available": True,
                },
            )

        direction = "RIGHT" if robot_center_offset_px > 0.0 else "LEFT"
        return MotionDecision(
            phase=phase,
            source="ball",
            action=f"BALL_PICKUP_CRAB_{direction}",
            valid=True,
            reason=(
                "ball_pickup_crab_right_required"
                if direction == "RIGHT"
                else "ball_pickup_crab_left_required"
            ),
            sdk_motion_requested=True,
            requires_ack=False,
            source_command={
                **common,
                "lateral_direction": direction,
                "catalog_motion_available": True,
            },
        )

    def plan_ball_pickup_initial_alignment(
        self,
        info: dict[str, Any] | None,
    ) -> MotionDecision:
        """Align pickup heading, then choose a step from Ball image height."""
        phase = "BALL_PICKUP_INITIAL_ALIGN"
        if info is None:
            return MotionDecision(
                phase=phase,
                source="ball",
                action="WAIT",
                valid=False,
                reason="ball_pickup_initial_alignment_waiting_for_frame",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command={},
            )
        if info.get("detected") is not True:
            return MotionDecision(
                phase=phase,
                source="ball",
                action="WAIT",
                valid=False,
                reason="ball_pickup_initial_alignment_waiting_for_ball",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command={},
            )

        confidence = self._number(info, "confidence")
        steering_angle = self._number(info, "steering_angle_deg")
        bearing = self._number(info, "bearing_deg")
        offset = self._number(info, "offset_x_norm")
        robot_center_offset_px = self._number(info, "offset_x_px")
        depth = self._number(info, "depth_m")
        distance = self._number(info, "distance_m")
        bottom_distance_px = self._number(info, "bottom_distance_px")
        steering_error = self.ball_planner._steering_error(
            steering_angle,
            bearing,
            offset,
            distance,
        )
        if (
            confidence is None
            or confidence < self.ball_planner.config.min_confidence
            or steering_error is None
            or robot_center_offset_px is None
        ):
            return MotionDecision(
                phase=phase,
                source="ball",
                action="WAIT",
                valid=False,
                reason="invalid_ball_pickup_initial_alignment_input",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command={},
            )

        tolerance = self.PICKUP_INITIAL_HEADING_TOLERANCE_DEG
        common = {
            "steering_angle_deg": steering_angle,
            "bearing_deg": bearing,
            "offset_x_norm": offset,
            "offset_x_px": robot_center_offset_px,
            "pickup_center_left_bound_px": (
                -self.PICKUP_INITIAL_CENTER_BOUND_PX
            ),
            "pickup_center_right_bound_px": (
                self.PICKUP_INITIAL_CENTER_BOUND_PX
            ),
            "steering_error_deg": steering_error,
            "heading_tolerance_deg": tolerance,
            "confidence": confidence,
            "depth_m": depth,
            "distance_m": distance,
            "bottom_distance_px": bottom_distance_px,
            "pickup_fine_step_bottom_distance_px": (
                self.config.pickup_fine_step_bottom_distance_px
            ),
            "depth_valid": info.get("depth_valid"),
            "depth_age_sec": self._number(info, "depth_age_sec"),
        }
        if bottom_distance_px is None or bottom_distance_px < 0.0:
            return MotionDecision(
                phase=phase,
                source="ball",
                action="WAIT",
                valid=False,
                reason="ball_pickup_waiting_for_bottom_distance_px",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command=common,
            )

        outside_center_window = bool(
            abs(robot_center_offset_px)
            > self.PICKUP_INITIAL_CENTER_BOUND_PX
        )
        if (
            outside_center_window
            and bottom_distance_px
            < self.PICKUP_CLOSE_CRAB_BOTTOM_DISTANCE_PX
        ):
            direction = (
                "RIGHT" if robot_center_offset_px > 0.0 else "LEFT"
            )
            return MotionDecision(
                phase=phase,
                source="ball",
                action=f"BALL_PICKUP_INITIAL_CRAB_{direction}",
                valid=True,
                reason="ball_pickup_close_lateral_correction",
                sdk_motion_requested=True,
                requires_ack=False,
                source_command={
                    **common,
                    "lateral_direction": direction,
                    "close_crab_bottom_distance_px": (
                        self.PICKUP_CLOSE_CRAB_BOTTOM_DISTANCE_PX
                    ),
                    "catalog_motion_available": True,
                },
            )

        if (
            bottom_distance_px
            >= self.PICKUP_CLOSE_CRAB_BOTTOM_DISTANCE_PX
            and (
                outside_center_window
                or abs(steering_error) > tolerance
            )
        ):
            if outside_center_window:
                direction = (
                    "RIGHT" if robot_center_offset_px > 0.0 else "LEFT"
                )
            else:
                direction = "RIGHT" if steering_error > 0.0 else "LEFT"
            count = self._turn_repeat_count(steering_error)
            return MotionDecision(
                phase=phase,
                source="ball",
                action=f"BALL_PICKUP_CAMERA_DOWN_TURN_{direction}_{count}",
                valid=True,
                reason="ball_pickup_initial_heading_correction",
                sdk_motion_requested=True,
                requires_ack=False,
                source_command={
                    **common,
                    "turn_direction": direction,
                    "turn_count": count,
                    "turn_angle_deg": count * self.TURN_REPEAT_DEG,
                    "catalog_motion_available": True,
                },
            )

        fine_step_threshold_px = max(
            0,
            self.config.pickup_fine_step_bottom_distance_px,
        )
        if bottom_distance_px > fine_step_threshold_px:
            approach_motion = "STRAIGHT_2"
        else:
            approach_motion = "STRAIGHT_0"
        approach_level = approach_level_from_motion(approach_motion)
        if approach_level not in {0, 2}:
            return MotionDecision(
                phase=phase,
                source="ball",
                action="WAIT",
                valid=False,
                reason="pickup_camera_down_motion_unavailable_for_pixel_range",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command=common,
            )
        return MotionDecision(
            phase=phase,
            source="ball",
            action="BALL_PICKUP_INITIAL_ALIGN_CONTINUE",
            valid=True,
            reason="ball_pickup_heading_aligned_for_pixel_approach",
            sdk_motion_requested=True,
            requires_ack=False,
            source_command={
                **common,
                "pickup_approach_motion": approach_motion,
                "approach_level": approach_level,
                "catalog_motion_available": True,
            },
        )

    def plan_ball_pickup_post_backward_alignment(
        self,
        info: dict[str, Any] | None,
    ) -> MotionDecision:
        """Align heading and lateral position after the no-Ball fallback."""
        phase = "BALL_PICKUP_POST_BACKWARD_ALIGN"
        if info is None or info.get("detected") is not True:
            return MotionDecision(
                phase=phase,
                source="ball",
                action="WAIT",
                valid=False,
                reason="ball_pickup_post_backward_alignment_waiting_for_ball",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command={},
            )

        confidence = self._number(info, "confidence")
        steering_angle = self._number(info, "steering_angle_deg")
        bearing = self._number(info, "bearing_deg")
        offset = self._number(info, "offset_x_norm")
        distance = self._number(info, "distance_m")
        tolerance = self._number(info, "pickup_x_tolerance_norm")
        steering_error = self.ball_planner._steering_error(
            steering_angle,
            bearing,
            offset,
            distance,
        )
        if (
            confidence is None
            or confidence < self.ball_planner.config.min_confidence
            or steering_error is None
            or offset is None
            or tolerance is None
            or tolerance <= 0.0
        ):
            return MotionDecision(
                phase=phase,
                source="ball",
                action="WAIT",
                valid=False,
                reason="invalid_ball_pickup_post_backward_alignment_input",
                sdk_motion_requested=False,
                requires_ack=False,
                source_command={},
            )

        heading_tolerance = self.ball_planner.config.turn_enter_deg
        common = {
            "steering_angle_deg": steering_angle,
            "bearing_deg": bearing,
            "offset_x_norm": offset,
            "steering_error_deg": steering_error,
            "heading_tolerance_deg": heading_tolerance,
            "pickup_x_tolerance_norm": tolerance,
            "confidence": confidence,
            "distance_m": distance,
        }
        if abs(steering_error) > heading_tolerance:
            direction = "RIGHT" if steering_error > 0.0 else "LEFT"
            count = self._turn_repeat_count(steering_error)
            return MotionDecision(
                phase=phase,
                source="ball",
                action=(
                    "BALL_PICKUP_POST_BACKWARD_TURN_"
                    f"{direction}_{count}"
                ),
                valid=True,
                reason="ball_pickup_post_backward_heading_correction",
                sdk_motion_requested=True,
                requires_ack=False,
                source_command={
                    **common,
                    "turn_direction": direction,
                    "turn_count": count,
                    "turn_angle_deg": count * self.TURN_REPEAT_DEG,
                    "catalog_motion_available": True,
                },
            )

        if abs(offset) > tolerance:
            direction = "RIGHT" if offset > 0.0 else "LEFT"
            return MotionDecision(
                phase=phase,
                source="ball",
                action=f"BALL_PICKUP_POST_BACKWARD_CRAB_{direction}",
                valid=True,
                reason="ball_pickup_post_backward_lateral_correction",
                sdk_motion_requested=True,
                requires_ack=False,
                source_command={
                    **common,
                    "lateral_direction": direction,
                    "catalog_motion_available": True,
                },
            )

        return MotionDecision(
            phase=phase,
            source="ball",
            action="BALL_PICKUP_POST_BACKWARD_ALIGN_CONTINUE",
            valid=True,
            reason="ball_pickup_post_backward_alignment_complete",
            sdk_motion_requested=False,
            requires_ack=False,
            source_command={
                **common,
                "lateral_direction": "CENTERED",
                "catalog_motion_available": True,
            },
        )

    def _goal_camera90_alignment_command(
        self,
        info: dict[str, Any] | None,
    ) -> dict[str, Any] | None:
        """Select one camera-90 yaw turn inside precision-control range."""
        if not self._is_detected_goal(info) or info is None:
            return None
        confidence = self._number(info, "confidence")
        depth = self._number(info, "depth_m")
        offset = self._number(info, "offset_x_norm")
        if (
            confidence is None
            or confidence < self.goal_planner.config.min_confidence
            or info.get("depth_valid") is not True
            or depth is None
            or offset is None
        ):
            return None
        if depth > self.config.goal_control_range_m:
            return None
        bearing = self._number(info, "bearing_deg")
        centered_by_bearing = bool(
            bearing is not None
            and abs(bearing) <= self.config.goal_reacquire_center_deg
        )
        if bearing is None:
            return {
                "valid": False,
                "action": "WAIT",
                "reason": "missing_goal_bearing_for_camera90_alignment",
                "sdk_motion_requested": False,
                "confidence": confidence,
                "depth_m": depth,
                "distance_m": self._number(info, "distance_m"),
                "bearing_error_deg": None,
                "offset_x_norm": offset,
                "is_centered": False,
                "score_now": False,
            }
        if centered_by_bearing:
            return None

        direction = "RIGHT" if bearing > 0.0 else "LEFT"
        count = self._turn_repeat_count(bearing)
        if direction == "LEFT":
            count = min(count, 6)
        return {
            "valid": True,
            "action": f"GOAL_CAMERA90_TURN_{direction}_{count}",
            "reason": "align_goal_yaw_camera90",
            "sdk_motion_requested": False,
            "confidence": confidence,
            "depth_m": depth,
            "distance_m": self._number(info, "distance_m"),
            "depth_error_m": None,
            "bearing_error_deg": bearing,
            "offset_x_norm": offset,
            "is_centered": False,
            "depth_in_score_range": False,
            "score_now": False,
            "turn_direction": direction,
            "turn_count": count,
            "turn_angle_deg": count * self.TURN_REPEAT_DEG,
            "catalog_motion_available": True,
        }

    def _goal_camera90_lateral_command(
        self,
        info: dict[str, Any] | None,
    ) -> dict[str, Any] | None:
        """Select one camera-90 crab after yaw is aligned."""
        if not self._is_detected_goal(info) or info is None:
            return None
        confidence = self._number(info, "confidence")
        depth = self._number(info, "depth_m")
        bearing = self._number(info, "bearing_deg")
        offset = self._number(info, "offset_x_norm")
        if (
            confidence is None
            or confidence < self.goal_planner.config.min_confidence
            or info.get("depth_valid") is not True
            or depth is None
            or depth > self.config.goal_control_range_m
            or bearing is None
            or abs(bearing) > self.config.goal_reacquire_center_deg
            or offset is None
        ):
            return None
        tolerance = self.goal_planner.config.score_center_tolerance_norm
        if abs(offset) <= tolerance:
            return None

        direction = "RIGHT" if offset > 0.0 else "LEFT"
        return {
            "valid": True,
            "action": f"GOAL_CAMERA90_CRAB_{direction}",
            "reason": "align_goal_lateral_camera90",
            "sdk_motion_requested": False,
            "confidence": confidence,
            "depth_m": depth,
            "distance_m": self._number(info, "distance_m"),
            "depth_error_m": None,
            "bearing_error_deg": bearing,
            "offset_x_norm": offset,
            "is_centered": False,
            "depth_in_score_range": False,
            "score_now": False,
            "lateral_direction": direction,
            "center_tolerance_norm": tolerance,
            "catalog_motion_available": True,
        }

    @staticmethod
    def _number(data: dict[str, Any] | None, key: str) -> float | None:
        if data is None:
            return None
        value = data.get(key)
        if value is None or isinstance(value, bool):
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if math.isfinite(number) else None

    def _ball_range_m(self, info: dict[str, Any] | None) -> float | None:
        """Return the published distance used by every BALL threshold."""
        return self._number(info, "distance_m")

    def _ball_direction_error_deg(
        self,
        info: dict[str, Any] | None,
    ) -> float | None:
        """Prefer the calibrated bottom-center ball path angle."""
        steering = self._number(info, "steering_angle_deg")
        if steering is not None:
            return steering
        return self._number(info, "bearing_deg")

    @staticmethod
    def _is_detected_ball(info: dict[str, Any] | None) -> bool:
        return bool(info is not None and info.get("detected", False))

    def _ball_is_inside_control_range(
        self,
        info: dict[str, Any] | None,
    ) -> bool:
        """Enter BALL control only with a confirmed, valid in-range sample."""
        if (
            not self._is_detected_ball(info)
            or not bool(info.get("depth_valid", False))
        ):
            return False
        ball_range = self._ball_range_m(info)
        return bool(
            ball_range is not None
            and ball_range > 0.0
            and ball_range <= self.config.ball_control_range_m
        )

    def _update_ball_tracking(
        self,
        info: dict[str, Any] | None,
        dt_sec: float,
    ) -> None:
        """Remember an in-range ball and time any later image loss."""
        if not self.config.enable_ball_lost_recovery:
            self._clear_ball_tracking()
            return
        if (
            info is not None
            and info.get("note") == "ball_outside_tracking_range"
        ):
            self._clear_ball_tracking()
            return
        detected = self._is_detected_ball(info)
        confidence = self._number(info, "confidence")
        reliable = detected and confidence is not None and confidence >= 0.35

        if reliable:
            bearing = self._ball_direction_error_deg(info)
            offset = self._number(info, "offset_x_norm")
            if bearing is not None:
                self.last_ball_bearing_deg = bearing
            if offset is not None:
                self.last_ball_offset_x_norm = offset

            direction_value = bearing
            if direction_value is None and offset is not None:
                direction_value = offset * 35.0
            if direction_value is not None:
                deadband = self.config.ball_recovery_direction_deadband_deg
                if direction_value > deadband:
                    self.last_ball_turn_direction = "RIGHT"
                elif direction_value < -deadband:
                    self.last_ball_turn_direction = "LEFT"

            ball_range = self._ball_range_m(info)
            depth_valid = bool(info.get("depth_valid", False))
            visual_alignment_only = not depth_valid or ball_range is None
            if visual_alignment_only or (
                ball_range is not None
                and ball_range <= self.config.ball_control_range_m
            ):
                self.ball_tracking_active = True
            if self.ball_tracking_active:
                self.ball_lost_elapsed_sec = 0.0
                self.ball_recovery_centering = False
            return

        if not self.ball_tracking_active:
            return
        self.ball_recovery_centering = True
        self.ball_lost_elapsed_sec += max(0.0, dt_sec)
        # Recovery retries are intentionally observation-driven with no
        # elapsed-time or attempt-count limit.

    def _update_ball_recovery_line_tracking(
        self,
        info: dict[str, Any] | None,
    ) -> None:
        """Remember which side of the robot the latest detected Line is on."""
        if info is None or info.get("detected") is not True:
            return
        offset = self._number(info, "filtered_lateral_offset_norm")
        if offset is None:
            offset = self._number(info, "lateral_offset_norm")
        if offset is None or offset == 0.0:
            return
        self.last_ball_line_offset_norm = offset
        self.last_ball_line_side = "RIGHT" if offset > 0.0 else "LEFT"

    def _lost_ball_recovery_command(self) -> dict[str, Any]:
        """Stop first, then turn about 90 degrees toward the last Line side."""
        direction = self.last_ball_line_side
        stopping = (
            self.ball_lost_elapsed_sec <= self.config.ball_lost_stop_sec
        )
        if stopping or direction is None:
            motion = "BALL_LOST_STOP"
            reason = (
                "ball_lost_stop_before_line_turn"
                if stopping
                else "ball_lost_stop_without_line_side"
            )
            count = 0
            turn_repeat_deg = 0.0
        else:
            if direction == "RIGHT":
                count = self.BALL_LOST_LINE_RIGHT_TURN_COUNT
                turn_repeat_deg = self.TURN_REPEAT_DEG
            else:
                count = self.BALL_LOST_LINE_LEFT_TURN_COUNT
                turn_repeat_deg = self.BALL_APPROACH_LEFT_TURN_REPEAT_DEG
            motion = f"BALL_APPROACH_TURN_{direction}_{count}"
            reason = "turn_toward_last_seen_line_side"

        duration = self.config.ball_recovery_command_sec
        return {
            "valid": True,
            "motion": motion,
            "reason": reason,
            "linear_speed_mps": 0.0,
            "lateral_speed_mps": 0.0,
            "angular_speed_rad_s": 0.0,
            "angular_accel_rad_s2": 0.0,
            "command_duration_sec": round(duration, 3),
            "travel_distance_m": 0.0,
            "lateral_travel_distance_m": 0.0,
            "target_heading_change_deg": count * turn_repeat_deg,
            "bearing_error_deg": self.last_ball_bearing_deg,
            "offset_x_norm": self.last_ball_offset_x_norm,
            "depth_m": None,
            "distance_m": None,
            "distance_error_m": None,
            "confidence": 0.0,
            "depth_valid": False,
            "pickup_ready": False,
            "pickup_now": False,
            "tracking_active": True,
            "lost_elapsed_sec": round(self.ball_lost_elapsed_sec, 3),
            "last_seen_line_side": direction,
            "line_lateral_offset_norm": self.last_ball_line_offset_norm,
            "turn_direction": direction,
            "turn_count": count,
            "turn_repeat_deg": turn_repeat_deg,
            "lost_ball_alignment_from_memory": not stopping,
            "alignment_reference": "line_side",
            "catalog_motion_available": direction is not None,
        }

    def _clear_ball_tracking(self) -> None:
        self.ball_tracking_active = False
        self.ball_recovery_centering = False
        self.ball_lost_elapsed_sec = 0.0
        self.last_ball_bearing_deg = None
        self.last_ball_offset_x_norm = None
        self.last_ball_line_offset_norm = None
        self.last_ball_line_side = None

    def clear_collected_ball_tracking(self) -> None:
        """Discard recovery state for a ball that was picked up successfully."""
        self._clear_ball_tracking()

    def disable_completed_ball_missions(self) -> None:
        """Release BALL ownership after all configured pickups complete."""
        self._clear_ball_tracking()
        self.ball_lock_active = False
        self.ball_terminal_requested = False
        self.ball_ignore_until_clear = True
        if self.previous_source == "ball":
            self._reset_previous_source()

    def ball_tracking_status(self) -> dict[str, Any]:
        """Expose remembered-ball state for debugging and behavior logs."""
        return {
            "active": self.ball_tracking_active,
            "recovery_centering": self.ball_recovery_centering,
            "tracking_range_m": self.config.ball_tracking_range_m,
            "control_range_m": self.config.ball_control_range_m,
            "lost_elapsed_sec": round(self.ball_lost_elapsed_sec, 3),
            "last_bearing_deg": self.last_ball_bearing_deg,
            "last_offset_x_norm": self.last_ball_offset_x_norm,
            "last_direction": self.last_ball_turn_direction,
            "last_line_offset_norm": self.last_ball_line_offset_norm,
            "last_line_side": self.last_ball_line_side,
        }

    @staticmethod
    def _is_detected_goal(info: dict[str, Any] | None) -> bool:
        return bool(info is not None and info.get("detected", False))

    def _goal_is_inside_control_range(
        self,
        info: dict[str, Any] | None,
    ) -> bool:
        if not self._is_detected_goal(info):
            return False
        depth = self._number(info, "depth_m")
        return bool(
            info.get("depth_valid", False)
            and depth is not None
            and depth <= self.config.goal_control_range_m
        )

    def _goal_needs_camera_90_approach(
        self,
        info: dict[str, Any] | None,
    ) -> bool:
        """Use the cataloged 90-degree-camera walk before close control."""
        if not self._is_detected_goal(info):
            return False
        confidence = self._number(info, "confidence")
        depth = self._number(info, "depth_m")
        return bool(
            confidence is not None
            and confidence >= self.goal_planner.config.min_confidence
            and info is not None
            and info.get("depth_valid", False)
            and depth is not None
            and self.config.goal_control_range_m < depth
            and depth <= self.config.goal_tracking_range_m
        )

    def _goal_camera_90_approach_command(
        self,
        info: dict[str, Any],
    ) -> dict[str, Any]:
        """Select one Depth-bucketed camera-90 motion, then await Vision."""
        depth = self._number(info, "depth_m")
        if depth is None or depth <= 0.0:
            action = "STRAIGHT_0"
            semantic_motion = "STRAIGHT_0"
        elif depth > 0.680:
            action = "GOAL_CAMERA_90_FORWARD"
            semantic_motion = "STRAIGHT_3"
        elif depth > 0.564:
            action = "GOAL_CAMERA_90_FORWARD_4"
            semantic_motion = "STRAIGHT_4"
        elif depth > 0.427:
            action = "GOAL_CAMERA_90_FORWARD"
            semantic_motion = "STRAIGHT_3"
        elif depth > 0.263:
            action = "GOAL_CAMERA_90_FORWARD_2"
            semantic_motion = "STRAIGHT_2"
        elif depth > 0.130:
            action = "GOAL_CAMERA_90_FORWARD_1"
            semantic_motion = "STRAIGHT_1"
        else:
            action = "STRAIGHT_0"
            semantic_motion = "STRAIGHT_0"
        return {
            "valid": True,
            "action": action,
            "reason": (
                "goal_camera90_fine_motion_unavailable"
                if semantic_motion == "STRAIGHT_0"
                else "goal_camera90_depth_bucket_approach"
            ),
            "sdk_motion_requested": False,
            "confidence": self._number(info, "confidence") or 0.0,
            "depth_m": depth,
            "distance_m": self._number(info, "distance_m"),
            "depth_error_m": None,
            "bearing_error_deg": self._number(info, "bearing_deg"),
            "offset_x_norm": self._number(info, "offset_x_norm"),
            "is_centered": True,
            "depth_in_score_range": False,
            "score_now": False,
            "approach_motion": semantic_motion,
            "approach_level": (
                int(semantic_motion.rsplit("_", 1)[1])
                if semantic_motion.startswith("STRAIGHT_")
                else None
            ),
            "approach_target_distance_m": depth,
        }

    def _update_goal_tracking(
        self,
        info: dict[str, Any] | None,
        dt_sec: float,
    ) -> None:
        """Remember an in-range backboard and time any later image loss."""
        if (
            info is not None
            and info.get("note") == "goal_outside_tracking_range"
        ):
            self._clear_goal_tracking()
            return
        detected = self._is_detected_goal(info)
        confidence = self._number(info, "confidence")
        reliable = detected and confidence is not None and confidence >= 0.35

        if reliable:
            bearing = self._number(info, "bearing_deg")
            offset = self._number(info, "offset_x_norm")
            if bearing is not None:
                self.last_goal_bearing_deg = bearing
            if offset is not None:
                self.last_goal_offset_x_norm = offset

            direction_value = bearing
            if direction_value is None and offset is not None:
                direction_value = offset * 35.0
            if direction_value is not None:
                if direction_value > 1.0:
                    self.last_goal_turn_direction = "RIGHT"
                elif direction_value < -1.0:
                    self.last_goal_turn_direction = "LEFT"

            depth = self._number(info, "depth_m")
            if (
                bool(info.get("depth_valid", False))
                and depth is not None
                and depth <= self.config.goal_tracking_range_m
            ):
                self.goal_tracking_active = True
            if self.goal_tracking_active:
                self.goal_lost_elapsed_sec = 0.0
                if self.goal_recovery_centering and self._goal_is_centered(
                    info
                ):
                    self.goal_recovery_centering = False
            return

        if not self.goal_tracking_active:
            return
        self.goal_recovery_centering = True
        self.goal_lost_elapsed_sec += max(0.0, dt_sec)
        if (
            self.goal_lost_elapsed_sec
            > self.config.goal_recovery_timeout_sec
        ):
            self._clear_goal_tracking()

    def _lost_goal_recovery_command(self) -> dict[str, Any]:
        """Stop first, then rotate toward the last observed backboard side."""
        direction = self.last_goal_turn_direction
        stopping = (
            self.goal_lost_elapsed_sec <= self.config.goal_lost_stop_sec
        )
        if stopping:
            action = "GOAL_LOST_STOP"
            angular_speed = 0.0
            reason = "goal_lost_stop_before_search"
        else:
            action = f"RECOVER_GOAL_TURN_{direction}"
            sign = 1.0 if direction == "RIGHT" else -1.0
            angular_speed = sign * self.config.goal_recovery_turn_rad_s
            reason = "turn_toward_last_seen_goal_side"

        duration = self.config.goal_recovery_command_sec
        return {
            "valid": True,
            "action": action,
            "reason": reason,
            "sdk_motion_requested": False,
            "linear_speed_mps": 0.0,
            "angular_speed_rad_s": round(angular_speed, 4),
            "command_duration_sec": round(duration, 3),
            "target_heading_change_deg": round(
                math.degrees(angular_speed * duration),
                3,
            ),
            "confidence": 0.0,
            "depth_m": None,
            "distance_m": None,
            "depth_error_m": None,
            "bearing_error_deg": self.last_goal_bearing_deg,
            "offset_x_norm": self.last_goal_offset_x_norm,
            "is_centered": False,
            "depth_in_score_range": False,
            "score_now": False,
            "tracking_active": True,
            "lost_elapsed_sec": round(self.goal_lost_elapsed_sec, 3),
            "last_seen_direction": direction,
        }

    def _goal_is_centered(self, info: dict[str, Any] | None) -> bool:
        bearing = self._number(info, "bearing_deg")
        if bearing is not None:
            return abs(bearing) <= self.config.goal_reacquire_center_deg
        offset = self._number(info, "offset_x_norm")
        return bool(
            offset is not None
            and abs(offset) <= self.config.goal_reacquire_center_norm
        )

    def _reacquired_goal_centering_command(
        self,
        info: dict[str, Any],
    ) -> dict[str, Any]:
        """Keep rotating after goal reacquisition until it is centered."""
        bearing = self._number(info, "bearing_deg")
        offset = self._number(info, "offset_x_norm")
        direction_value = bearing
        if direction_value is None and offset is not None:
            direction_value = offset * 35.0
        if direction_value is not None and direction_value < 0.0:
            direction = "LEFT"
            sign = -1.0
        else:
            direction = "RIGHT"
            sign = 1.0
        angular_speed = sign * self.config.goal_recovery_turn_rad_s
        duration = self.config.goal_recovery_command_sec
        return {
            "valid": True,
            "action": f"RECOVER_GOAL_TURN_{direction}",
            "reason": "reacquired_goal_centering_in_place",
            "sdk_motion_requested": False,
            "linear_speed_mps": 0.0,
            "angular_speed_rad_s": round(angular_speed, 4),
            "command_duration_sec": round(duration, 3),
            "target_heading_change_deg": round(
                math.degrees(angular_speed * duration),
                3,
            ),
            "confidence": self._number(info, "confidence") or 0.0,
            "depth_m": self._number(info, "depth_m"),
            "distance_m": self._number(info, "distance_m"),
            "depth_error_m": None,
            "bearing_error_deg": bearing,
            "offset_x_norm": offset,
            "is_centered": False,
            "depth_in_score_range": False,
            "score_now": False,
            "tracking_active": True,
            "lost_elapsed_sec": 0.0,
            "last_seen_direction": direction,
        }

    def _clear_goal_tracking(self) -> None:
        self.goal_tracking_active = False
        self.goal_recovery_centering = False
        self.goal_lost_elapsed_sec = 0.0
        self.last_goal_bearing_deg = None
        self.last_goal_offset_x_norm = None

    def goal_tracking_status(self) -> dict[str, Any]:
        """Expose remembered-goal state for debugging and behavior logs."""
        return {
            "active": self.goal_tracking_active,
            "recovery_centering": self.goal_recovery_centering,
            "tracking_range_m": self.config.goal_tracking_range_m,
            "control_range_m": self.config.goal_control_range_m,
            "lost_elapsed_sec": round(self.goal_lost_elapsed_sec, 3),
            "last_bearing_deg": self.last_goal_bearing_deg,
            "last_offset_x_norm": self.last_goal_offset_x_norm,
            "last_direction": self.last_goal_turn_direction,
        }

    def _reset_source(self, source: str) -> None:
        if source == "line":
            self.line_planner.stop("source_changed")
        elif source == "ball":
            self.ball_planner.stop("source_changed")

    def _reset_previous_source(self) -> None:
        self._reset_source(self.previous_source)
        self.previous_source = "none"
