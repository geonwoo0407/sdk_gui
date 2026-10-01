"""ROS-independent mission phase and special-motion progress state."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class MotionStatusResult:
    """Describe how one motion status affected mission state."""

    handled: bool
    terminal: bool
    duplicate: bool
    reason: str
    previous_phase: str
    current_phase: str


class MissionPhaseManager:
    """Own mission progress and special-motion phase transitions."""

    GRASP_UNKNOWN = "UNKNOWN"
    GRASPED = "GRABBED"
    GRASP_NOT_GRABBED = "NOT_GRABBED"
    GRASP_RESULTS = frozenset(
        {GRASP_UNKNOWN, GRASPED, GRASP_NOT_GRABBED}
    )

    ALLOWED_PHASES = frozenset(
        {
            "AUTO",
            "BALL_SEARCH",
            "BALL_APPROACH",
            "GOAL_SEARCH",
            "GOAL_APPROACH",
            "POST_BALL_LINE_ALIGN",
            "POST_BALL_GOAL_TRANSITION",
            "HURDLE_APPROACH",
            "LINE_TRACK",
            "FINISH",
            "WALK_TO_FINISH",
            "FINISHED",
        }
    )
    SPECIAL_ACTIONS = frozenset(
        {
            "PICKUP_NOW",
            "POST_BALL_GOAL_TRANSITION",
            "SHOT",
            "GO",
            "CROSS_FINISH",
        }
    )
    SUPPORTED_STATUSES = frozenset(
        {
            "RUNNING",
            "SUCCEEDED",
            "FAILED",
            "TIMEOUT",
            "CANCELLED",
            "REJECTED",
            "UNSUPPORTED",
        }
    )
    TERMINAL_STATUSES = frozenset(
        {
            "SUCCEEDED",
            "FAILED",
            "TIMEOUT",
            "CANCELLED",
            "REJECTED",
            "UNSUPPORTED",
        }
    )

    def __init__(
        self,
        *,
        required_pickups: int = 2,
        required_shots: int = 2,
        required_ball_sections: int = 2,
        max_pickup_failures: int = 3,
        max_shot_failures: int = 3,
        max_go_failures: int = 3,
        initial_phase: str = "AUTO",
    ) -> None:
        """Initialize bounded progress counters and one validated phase."""
        self.required_pickups = self._nonnegative_count(required_pickups)
        self.required_shots = self._nonnegative_count(required_shots)
        self.required_ball_sections = self._nonnegative_count(
            required_ball_sections
        )

        self.max_pickup_failures = self._nonnegative_count(
        max_pickup_failures
        )
        self.max_shot_failures = self._nonnegative_count(
            max_shot_failures
        )
        self.max_go_failures = self._nonnegative_count(
            max_go_failures
        )

        normalized_phase = self._normalize_phase(initial_phase)
        if normalized_phase is None:
            raise ValueError("initial_phase must be a supported phase")

        self.current_phase = normalized_phase
        self.pickups_completed = 0
        self.ball_grasp_results = {
            index: self.GRASP_UNKNOWN
            for index in range(1, self.required_pickups + 1)
        }
        self.active_pickup_attempt: int | None = None
        self.shots_completed = 0
        self.ball_sections_processed = 0
        self.finish_enabled = self.required_ball_sections == 0
        self.mission_complete = False
        self.post_ball_goal_transition_failed = False

        self.pickup_failure_count = 0
        self.shot_failure_count = 0
        self.go_failure_count = 0

        self.active_special_action: str | None = None
        self.active_special_command_id: int | None = None
        self.active_special_running = False
        self._active_special_origin_phase: str | None = None
        self._completed_command_ids: set[int] = set()

    @staticmethod
    def _nonnegative_count(value: Any) -> int:
        """Match the existing node policy by clamping integer counts to zero."""
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError("required counts must be integers")
        return max(0, value)

    @classmethod
    def _normalize_phase(cls, phase: Any) -> str | None:
        if not isinstance(phase, str):
            return None
        normalized = phase.strip().upper()
        return normalized if normalized in cls.ALLOWED_PHASES else None

    @classmethod
    def _normalize_action(cls, action: Any) -> str | None:
        if not isinstance(action, str):
            return None
        normalized = action.strip().upper()
        return normalized if normalized in cls.SPECIAL_ACTIONS else None

    @staticmethod
    def _valid_command_id(command_id: Any) -> bool:
        return (
            isinstance(command_id, int)
            and not isinstance(command_id, bool)
            and command_id > 0
        )

    def set_phase(self, phase: str) -> bool:
        """Set one permanent supported phase; dynamic lock phases are rejected."""
        normalized = self._normalize_phase(phase)
        if normalized is None:
            return False
        self.current_phase = normalized
        if normalized == "POST_BALL_GOAL_TRANSITION":
            self.post_ball_goal_transition_failed = False
        return True

    def special_action_exhausted(self, action: str) -> bool:
        """Return whether one mission-level special action exhausted failures."""
        normalized = self._normalize_action(action)

        if normalized == "PICKUP_NOW":
            return (
                self.pickup_failure_count
                >= self.max_pickup_failures
            )

        if normalized == "SHOT":
            return self.shot_failure_count >= self.max_shot_failures

        if normalized == "GO":
            return self.go_failure_count >= self.max_go_failures

        return False

    def complete_post_ball_line_align(self) -> bool:
        """Advance only a completed first-Ball line alignment."""
        if (
            self.current_phase != "POST_BALL_LINE_ALIGN"
            or self.active_special_command_id is not None
        ):
            return False
        self.current_phase = "POST_BALL_GOAL_TRANSITION"
        self.post_ball_goal_transition_failed = False
        return True


    def start_special_action(self, action: str, command_id: int) -> bool:
        """Register one special command before its RUNNING status arrives."""
        normalized_action = self._normalize_action(action)
        if normalized_action is None or not self._valid_command_id(command_id):
            return False
        if command_id in self._completed_command_ids:
            return False
        if self.active_special_command_id is not None:
            return (
                self.active_special_action == normalized_action
                and self.active_special_command_id == command_id
            )

        self.active_special_action = normalized_action
        self.active_special_command_id = command_id
        self.active_special_running = False
        self._active_special_origin_phase = self.current_phase
        if normalized_action == "PICKUP_NOW":
            attempt = self.pickups_completed + 1
            self.active_pickup_attempt = (
                attempt if attempt in self.ball_grasp_results else None
            )
            if self.active_pickup_attempt is not None:
                self.ball_grasp_results[
                    self.active_pickup_attempt
                ] = self.GRASP_UNKNOWN
        return True

    def record_active_pickup_grasp_result(self, result: str) -> bool:
        """Latch one verified result for the currently active BALL pickup."""
        normalized = result.strip().upper() if isinstance(result, str) else ""
        attempt = self.active_pickup_attempt
        if (
            normalized not in self.GRASP_RESULTS
            or self.active_special_action != "PICKUP_NOW"
            or attempt is None
        ):
            return False
        self.ball_grasp_results[attempt] = normalized
        return True

    def grasp_result_for_ball(self, ball_index: int) -> str:
        """Return a BALL-specific grasp result without sharing stale state."""
        return self.ball_grasp_results.get(ball_index, self.GRASP_UNKNOWN)

    def grasp_result_for_next_shot(self) -> str:
        """Return the result belonging to the next unprocessed BALL section."""
        return self.grasp_result_for_ball(self.ball_sections_processed + 1)

    def handle_motion_status(
        self,
        action: str,
        command_id: int,
        status: str,
    ) -> MotionStatusResult:
        """Apply one correlated special-motion status."""
        previous_phase = self.current_phase
        normalized_action = self._normalize_action(action)
        normalized_status = (
            status.strip().upper() if isinstance(status, str) else ""
        )

        if normalized_status not in self.SUPPORTED_STATUSES:
            return self._result(
                False, False, False, "unsupported_status", previous_phase
            )
        if not self._valid_command_id(command_id):
            return self._result(
                False, False, False, "invalid_command_id", previous_phase
            )
        if (
            normalized_status in self.TERMINAL_STATUSES
            and command_id in self._completed_command_ids
        ):
            return self._result(
                False, True, True, "duplicate_terminal", previous_phase
            )
        if (
            normalized_action is None
            or normalized_action != self.active_special_action
            or command_id != self.active_special_command_id
        ):
            return self._result(
                False, False, False, "active_command_mismatch", previous_phase
            )

        if normalized_status == "RUNNING":
            self.active_special_running = True
            return self._result(
                True, False, False, "running", previous_phase
            )

        if (
            normalized_status not in {"REJECTED", "UNSUPPORTED"}
            and not self.active_special_running
        ):
            return self._result(
                False,
                True,
                False,
                "terminal_before_running",
                previous_phase,
            )

        self._apply_terminal(normalized_action, normalized_status)
        self._completed_command_ids.add(command_id)
        self.active_special_action = None
        self.active_special_command_id = None
        self.active_special_running = False
        self._active_special_origin_phase = None
        self.active_pickup_attempt = None
        return self._result(
            True, True, False, "terminal_applied", previous_phase
        )

    def _apply_terminal(self, action: str, status: str) -> None:
        succeeded = status == "SUCCEEDED"

        if action == "PICKUP_NOW":
            if succeeded:
                self.pickup_failure_count = 0
                self.pickups_completed = min(
                    self.pickups_completed + 1,
                    self.required_pickups,
                )
                self.post_ball_goal_transition_failed = False
                self.current_phase = (
                    "POST_BALL_LINE_ALIGN"
                    if self.pickups_completed == 1
                    else "POST_BALL_GOAL_TRANSITION"
                )
                return

            if status in {"FAILED", "TIMEOUT"}:
                self.pickup_failure_count += 1

            self.current_phase = "BALL_APPROACH"
            return

        if action == "POST_BALL_GOAL_TRANSITION":
            if succeeded:
                self.post_ball_goal_transition_failed = False
                self.current_phase = "GOAL_APPROACH"
            else:
                self.post_ball_goal_transition_failed = True
                self.current_phase = "POST_BALL_GOAL_TRANSITION"
            return

        if action == "SHOT":
            if succeeded:
                self.shot_failure_count = 0
                self.shots_completed = min(
                    self.shots_completed + 1,
                    self.required_shots,
                )
                self.ball_sections_processed = min(
                    self.ball_sections_processed + 1,
                    self.required_ball_sections,
                )
                self._update_finish_enabled()
                all_ball_sections_processed = (
                    self.required_ball_sections > 0
                    and self.ball_sections_processed
                    >= self.required_ball_sections
                )
                self.current_phase = (
                    "LINE_TRACK"
                    if all_ball_sections_processed
                    else "AUTO"
                )
                return

            if status in {"FAILED", "TIMEOUT"}:
                self.shot_failure_count += 1

            self.current_phase = "GOAL_APPROACH"
            return

        if action == "GO":
            if not succeeded:
                if status in {"FAILED", "TIMEOUT"}:
                    self.go_failure_count += 1
                self.current_phase = "HURDLE_APPROACH"
                return

            self.go_failure_count = 0
            origin_phase = self._active_special_origin_phase
            self.current_phase = (
                origin_phase
                if origin_phase in self.ALLOWED_PHASES
                else "AUTO"
            )
            return

        if succeeded:
            self.mission_complete = True
            self.current_phase = "FINISHED"
        else:
            self.mission_complete = False
            self.current_phase = "WALK_TO_FINISH"

    def _update_finish_enabled(self) -> None:
        self.finish_enabled = (
            self.ball_sections_processed >= self.required_ball_sections
        )

    def _result(
        self,
        handled: bool,
        terminal: bool,
        duplicate: bool,
        reason: str,
        previous_phase: str,
    ) -> MotionStatusResult:
        return MotionStatusResult(
            handled=handled,
            terminal=terminal,
            duplicate=duplicate,
            reason=reason,
            previous_phase=previous_phase,
            current_phase=self.current_phase,
        )

    def snapshot(self) -> dict[str, int | bool | str | None]:
        """Return a new dictionary containing the externally visible state."""
        return {
            "current_phase": self.current_phase,
            "pickups_completed": self.pickups_completed,
            "shots_completed": self.shots_completed,
            "ball_sections_processed": self.ball_sections_processed,
            "finish_enabled": self.finish_enabled,
            "mission_complete": self.mission_complete,
            "post_ball_goal_transition_failed": (
                self.post_ball_goal_transition_failed
            ),
            "required_pickups": self.required_pickups,
            "required_shots": self.required_shots,
            "required_ball_sections": self.required_ball_sections,
            "active_special_action": self.active_special_action,
            "active_special_command_id": self.active_special_command_id,
            "active_special_running": self.active_special_running,
        }
