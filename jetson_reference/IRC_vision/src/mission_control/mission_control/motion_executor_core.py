"""ROS-independent core state machine for motion execution."""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Optional


class ExecutorState(Enum):
    IDLE = auto()
    STARTING = auto()
    RUNNING = auto()
    SETTLING = auto()
    SUCCEEDED = auto()
    FAILED = auto()
    CANCELLED = auto()
    TIMEOUT = auto()


@dataclass(frozen=True)
class MotionExecutionResult:
    motion_id: str
    final_status: ExecutorState
    success: bool
    error_code: str
    message: str


class MotionExecutorCore:
    SUPPORTED_MOTION_IDS = frozenset(
        {
            "home",
            "forward",
            "forward_short",
            "turn_left",
            "turn_right",
            "adjust_left",
            "adjust_right",
            "backward",
            "pick_ball",
            "shoot",
            "hurdle",
            "recover",
            "head_left",
            "head_right",
            "head_center",
        }
    )

    _ACTIVE_STATES = frozenset(
        {
            ExecutorState.STARTING,
            ExecutorState.RUNNING,
            ExecutorState.SETTLING,
        }
    )

    def __init__(self, player: Any) -> None:
        self._player = player
        self._state = ExecutorState.IDLE
        self._active_motion: Optional[str] = None
        self._timeout_ms = 0
        self._elapsed_ms = 0
        self._terminal_result: Optional[MotionExecutionResult] = None

    @staticmethod
    def _name(value: Any) -> str:
        if isinstance(value, str):
            return value
        name = getattr(value, "name", None)
        return name if isinstance(name, str) else str(value)

    def _last_error_message(self) -> str:
        """Return the SDK's human-readable diagnostic text."""
        error = self._player.lastError()
        if error is None:
            return ""
        if isinstance(error, str):
            return error
        return self._name(error)

    @staticmethod
    def _rejection(
        motion_id: str, error_code: str, message: str
    ) -> MotionExecutionResult:
        return MotionExecutionResult(
            motion_id=motion_id,
            final_status=ExecutorState.FAILED,
            success=False,
            error_code=error_code,
            message=message,
        )

    def _finish(
        self,
        final_status: ExecutorState,
        success: bool,
        error_code: str,
        message: str,
    ) -> MotionExecutionResult:
        if self._terminal_result is None:
            self._state = final_status
            self._terminal_result = MotionExecutionResult(
                motion_id=self._active_motion or "",
                final_status=final_status,
                success=success,
                error_code=error_code,
                message=message,
            )
        return self._terminal_result

    def start_motion(
        self, motion_id: str, timeout_ms: int
    ) -> Optional[MotionExecutionResult]:
        if self.busy():
            return self._rejection(
                motion_id, "REJECTED_BUSY", "another motion is already running"
            )
        if self._state is not ExecutorState.IDLE:
            return self._rejection(
                motion_id, "INTERNAL_ERROR", "reset is required before a new motion"
            )
        if motion_id not in self.SUPPORTED_MOTION_IDS:
            return self._rejection(
                motion_id, "INVALID_MOTION", "unsupported motion_id"
            )
        if timeout_ms <= 0:
            return self._rejection(
                motion_id, "INVALID_MOTION", "timeout_ms must be greater than zero"
            )
        if not self._player.hardwareReady():
            return self._rejection(
                motion_id, "HARDWARE_NOT_READY", "motion hardware is not ready"
            )

        start_result = self._player.start(motion_id)
        start_name = self._name(start_result)
        if start_name != "ACCEPTED":
            messages = {
                "REJECTED_BUSY": "player rejected the request because it is busy",
                "MOTION_NOT_FOUND": "motion data was not found",
                "HARDWARE_NOT_READY": "motion hardware is not ready",
                "INVALID_MOTION": "player rejected an invalid motion",
            }
            message = messages.get(
                start_name,
                f"unexpected start result: {start_name}",
            )
            detail = self._last_error_message()
            if detail and detail != "NONE":
                message = f"{message}: {detail}"
            return self._rejection(
                motion_id,
                start_name,
                message,
            )

        self._active_motion = motion_id
        self._timeout_ms = timeout_ms
        self._elapsed_ms = 0
        self._terminal_result = None
        self._state = ExecutorState.STARTING
        return None

    def tick(self, elapsed_ms: int) -> Optional[MotionExecutionResult]:
        if not self.busy():
            return self._terminal_result

        self._elapsed_ms += max(0, elapsed_ms)
        if self._elapsed_ms >= self._timeout_ms:
            self._player.cancel()
            return self._finish(
                ExecutorState.TIMEOUT,
                False,
                "POSITION_TIMEOUT",
                "executor timeout exceeded",
            )

        self._player.update()
        status_name = self._name(self._player.status())

        if status_name == "RUNNING":
            self._state = ExecutorState.RUNNING
        elif status_name == "SETTLING":
            self._state = ExecutorState.SETTLING
        elif status_name == "SUCCEEDED":
            return self._finish(
                ExecutorState.SUCCEEDED,
                True,
                "NONE",
                "motion completed successfully",
            )
        elif status_name == "FAILED":
            player_result = self._player.result()
            error_code = self._name(player_result)
            detail = self._last_error_message()
            message = f"motion failed: result={error_code}"
            if detail and detail != "NONE":
                message = f"{message}, error={detail}"
            return self._finish(
                ExecutorState.FAILED,
                False,
                error_code,
                message,
            )
        elif status_name == "CANCELLED":
            return self._finish(
                ExecutorState.CANCELLED,
                False,
                "NONE",
                "motion was cancelled",
            )
        elif status_name != "IDLE":
            return self._finish(
                ExecutorState.FAILED,
                False,
                "INTERNAL_ERROR",
                f"unexpected player status: {status_name}",
            )

        return None

    def cancel(self) -> Optional[MotionExecutionResult]:
        if not self.busy():
            return self._terminal_result

        cancel_result = self._player.cancel()
        cancel_name = self._name(cancel_result)
        if cancel_name == "CANCELLED":
            return self._finish(
                ExecutorState.CANCELLED,
                False,
                "NONE",
                "motion was cancelled",
            )
        if cancel_name == "HOLD_FAILED":
            return self._finish(
                ExecutorState.FAILED,
                False,
                "CANCEL_FAILED",
                "cancel failed while holding the current position",
            )
        if cancel_name == "HARDWARE_NOT_READY":
            return self._finish(
                ExecutorState.FAILED,
                False,
                "HARDWARE_NOT_READY",
                "cancel failed because hardware is not ready",
            )
        if cancel_name == "NOT_RUNNING":
            return self._terminal_result

        return self._finish(
            ExecutorState.FAILED,
            False,
            "INTERNAL_ERROR",
            f"unexpected cancel result: {cancel_name}",
        )

    def reset(self) -> bool:
        if self.busy():
            return False
        self._state = ExecutorState.IDLE
        self._active_motion = None
        self._timeout_ms = 0
        self._elapsed_ms = 0
        self._terminal_result = None
        return True

    def state(self) -> ExecutorState:
        return self._state

    def active_motion(self) -> Optional[str]:
        return self._active_motion

    def terminal_result(self) -> Optional[MotionExecutionResult]:
        return self._terminal_result

    def busy(self) -> bool:
        return self._state in self._ACTIVE_STATES
