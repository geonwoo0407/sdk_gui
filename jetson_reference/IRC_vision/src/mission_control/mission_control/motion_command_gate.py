"""ROS-free state for suppressing duplicate general motion commands."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


GENERAL_ACTIONS = frozenset(
    {
        "STRAIGHT",
        "STRAIGHT_1",
        "STRAIGHT_2",
        "STRAIGHT_3",
        "STRAIGHT_4",
        "STRAIGHT_5",
        "APPROACH",
        "SLOW_APPROACH",
        "FINE_FORWARD_STEP",
        "APPROACH_GOAL",
        "BALL_FINE_FORWARD_8",
        "GOAL_CAMERA_90_FORWARD",
        "GOAL_CAMERA_90_FORWARD_1",
        "GOAL_CAMERA_90_FORWARD_2",
        "GOAL_CAMERA_90_FORWARD_4",
        "GOAL_CAMERA90_CRAB_RIGHT",
        "GOAL_CAMERA90_CRAB_LEFT",
        *{
            f"POST_BALL_LINE_TURN_RIGHT_{count}"
            for count in range(1, 10)
        },
        *{
            f"POST_BALL_LINE_TURN_LEFT_{count}"
            for count in (2, 3, 4, 6)
        },
        *{
            f"GOAL_CAMERA90_TURN_RIGHT_{count}"
            for count in range(1, 10)
        },
        *{
            f"GOAL_CAMERA90_TURN_LEFT_{count}"
            for count in range(1, 7)
        },
        *{
            f"BALL_APPROACH_TURN_RIGHT_{count}"
            for count in range(1, 10)
        },
        *{
            f"BALL_APPROACH_TURN_LEFT_{count}"
            for count in (2, 3, 4, 5, 6)
        },
        "APPROACH_HURDLE",
        "LEFT",
        "RIGHT",
        "TURN_LEFT",
        "TURN_RIGHT",
        "ALIGN_LEFT",
        "ALIGN_RIGHT",
        "RETREAT_GOAL",
        "STOP",
        *{
            f"RECOVER_{line_side}_TURN_LEFT_{suffix}"
            for line_side in ("LEFT", "RIGHT")
            for suffix in (2, 4, 6, 8, 10, 13)
        },
        *{
            f"RECOVER_{line_side}_TURN_RIGHT_{suffix}"
            for line_side in ("LEFT", "RIGHT")
            for suffix in (4, 6, 8, 10, 12, 15)
        },
    }
)
TERMINAL_STATUSES = frozenset(
    {
        "SUCCEEDED",
        "FAILED",
        "TIMEOUT",
        "REJECTED",
        "CANCELED",
        "CANCELLED",
        "UNSUPPORTED",
    }
)
ACTION_ALIASES = {action: action for action in GENERAL_ACTIONS}
TRANSIENT_REJECTION_CODES = frozenset(
    {
        "REJECTED_BUSY",
        "BUSY",
        "SDK_BUSY",
        "STARTUP_POSE_GATE_LOCKED",
    }
)
PERMANENT_REJECTION_CODES = frozenset(
    {
        "INVALID_REQUEST",
        "INVALID_MOTION",
        "MOTION_NOT_FOUND",
        "INTERNAL_ERROR",
    }
)
DEFAULT_MAX_TRANSIENT_RETRIES = 2


def normalize_general_action(action: Any) -> str | None:
    """Normalize only action names used by the general-motion path."""
    if not isinstance(action, str):
        return None
    normalized = action.strip().upper()
    return ACTION_ALIASES.get(normalized)


@dataclass(frozen=True)
class GateTransition:
    """Describe whether one status matched and released the active lock."""

    matched: bool
    released: bool


class GeneralMotionCommandGate:
    """Allow one general command until its matching execution terminates."""

    def __init__(
        self,
        max_transient_retries: int = DEFAULT_MAX_TRANSIENT_RETRIES,
    ) -> None:
        self.locked = False
        self.active_action: str | None = None
        self.active_command_id: int | None = None
        self.running_seen = False
        self.vision_generation = 0
        self.required_vision_generation = 0
        self.rejected_action: str | None = None
        self.transient_rejection_action: str | None = None
        self.transient_rejection_count = 0
        self.max_transient_retries = max(0, max_transient_retries)

    def on_new_vision_input(self) -> None:
        """Record one valid Vision JSON message."""
        self.vision_generation += 1

    def has_required_fresh_vision(self) -> bool:
        """Return whether the required post-terminal Vision input has arrived."""
        return self.vision_generation >= self.required_vision_generation


    def can_publish(self, action: Any) -> bool:
        """Return whether a general action may be published now."""
        normalized = normalize_general_action(action)
        if normalized is None or self.locked:
            return False
        if not self.has_required_fresh_vision():
            return False
        if self.rejected_action == normalized:
            return False
        if (
            self.rejected_action is not None
            and self.rejected_action != normalized
        ):
            self.rejected_action = None
            self._reset_transient_rejections()
        elif (
            self.transient_rejection_action is not None
            and self.transient_rejection_action != normalized
        ):
            self._reset_transient_rejections()
        return True

    def on_command_published(
        self, action: Any, command_id: int | None = None
    ) -> None:
        """Lock immediately after publishing one general command."""
        normalized = normalize_general_action(action)
        if normalized is None:
            raise ValueError("unsupported general motion action")
        if self.locked:
            raise RuntimeError("general motion command is already locked")
        self.locked = True
        self.active_action = normalized
        self.active_command_id = command_id
        self.running_seen = False

    def on_motion_status(
        self,
        action: Any,
        status: Any,
        command_id: Any = None,
        error_code: Any = None,
    ) -> GateTransition:
        """Apply a matching status while protecting against stale terminals."""
        normalized_action = normalize_general_action(action)
        normalized_status = (
            status.strip().upper() if isinstance(status, str) else ""
        )
        if (
            not self.locked
            or normalized_action is None
            or normalized_action != self.active_action
        ):
            return GateTransition(False, False)

        # Correlate new messages by mission command ID. A status without the
        # active ID must not release a newer same-action command.
        if (
            self.active_command_id is not None
            and command_id != self.active_command_id
        ):
            return GateTransition(False, False)

        if normalized_status == "RUNNING":
            self.running_seen = True
            self._reset_transient_rejections()
            return GateTransition(True, False)

        if normalized_status not in TERMINAL_STATUSES:
            return GateTransition(False, False)

        # Accepted Executor requests publish RUNNING before their terminal
        # result. Requiring it prevents an old same-action terminal message
        # from releasing a newly-created lock. REJECTED has no RUNNING.
        if (
            normalized_status not in {"REJECTED", "UNSUPPORTED"}
            and not self.running_seen
        ):
            return GateTransition(False, False)

        completed_action = self.active_action
        self.locked = False
        self.active_action = None
        self.active_command_id = None
        self.running_seen = False
        self.required_vision_generation = self.vision_generation + 1

        if normalized_status in {"REJECTED", "UNSUPPORTED"}:
            self._handle_rejection(completed_action, error_code)
        elif normalized_status in {
            "FAILED",
            "TIMEOUT",
            "CANCELED",
            "CANCELLED",
        }:
            self.rejected_action = completed_action
            self._reset_transient_rejections()
        else:
            self._reset_transient_rejections()

        return GateTransition(True, True)

    def _handle_rejection(
        self,
        action: str | None,
        error_code: Any,
    ) -> None:
        """Classify one rejection and update bounded retry state."""
        normalized_code = (
            error_code.strip().upper()
            if isinstance(error_code, str)
            else ""
        )
        if normalized_code not in TRANSIENT_REJECTION_CODES:
            self.rejected_action = action
            self._reset_transient_rejections()
            return

        if self.transient_rejection_action != action:
            self.transient_rejection_action = action
            self.transient_rejection_count = 0
        self.transient_rejection_count += 1

        if self.transient_rejection_count > self.max_transient_retries:
            self.rejected_action = action
            self._reset_transient_rejections()

    def _reset_transient_rejections(self) -> None:
        """Clear the retry budget associated with the previous action."""
        self.transient_rejection_action = None
        self.transient_rejection_count = 0
