"""Test-only, deterministic mock implementation of RobotMotionPlayer."""

from enum import Enum, auto
from typing import Optional


class MotionStatus(Enum):
    IDLE = auto()
    RUNNING = auto()
    SETTLING = auto()
    SUCCEEDED = auto()
    CANCELLED = auto()
    FAILED = auto()


class StartResult(Enum):
    ACCEPTED = auto()
    REJECTED_BUSY = auto()
    MOTION_NOT_FOUND = auto()
    HARDWARE_NOT_READY = auto()
    INVALID_MOTION = auto()


class CancelResult(Enum):
    CANCELLED = auto()
    NOT_RUNNING = auto()
    HARDWARE_NOT_READY = auto()
    HOLD_FAILED = auto()


class MotionError(Enum):
    NONE = auto()
    JSON_ERROR = auto()
    HARDWARE_NOT_READY = auto()
    COMMUNICATION_ERROR = auto()
    FRAME_SEND_FAILED = auto()
    PRESENT_POSITION_READ_FAILED = auto()
    POSITION_TIMEOUT = auto()
    CANCEL_FAILED = auto()
    INTERNAL_ERROR = auto()


class MockRobotMotionPlayer:
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

    def __init__(
        self,
        hardware_ready: bool = True,
        running_updates: int = 3,
        settling_updates: int = 2,
        fail_after_updates: int = -1,
        failure_error: MotionError = MotionError.COMMUNICATION_ERROR,
        failure_message: str = "injected mock motion failure",
    ) -> None:
        self._hardware_ready = hardware_ready
        self._running_updates = max(0, running_updates)
        self._settling_updates = max(0, settling_updates)
        self._fail_after_updates = fail_after_updates
        self._failure_error = failure_error
        self._failure_message = failure_message
        self._update_count = 0
        self._status = MotionStatus.IDLE
        self._result = MotionError.NONE
        self._last_error = ""
        self._current_motion: Optional[str] = None

    def hardwareReady(self) -> bool:
        return self._hardware_ready

    def start(self, motion_id: str) -> StartResult:
        if self.running():
            return StartResult.REJECTED_BUSY
        if not self._hardware_ready:
            self._result = MotionError.HARDWARE_NOT_READY
            self._last_error = "motion hardware is not ready"
            return StartResult.HARDWARE_NOT_READY
        if motion_id not in self.SUPPORTED_MOTION_IDS:
            self._result = MotionError.NONE
            self._last_error = "unsupported motion_id"
            return StartResult.INVALID_MOTION

        self._current_motion = motion_id
        self._update_count = 0
        self._status = MotionStatus.RUNNING
        self._result = MotionError.NONE
        self._last_error = ""
        return StartResult.ACCEPTED

    def update(self) -> None:
        if not self.running():
            return

        self._update_count += 1
        if (
            self._fail_after_updates >= 0
            and self._update_count >= self._fail_after_updates
        ):
            self._status = MotionStatus.FAILED
            self._result = self._failure_error
            self._last_error = self._failure_message
            return

        running_end = self._running_updates
        settling_end = running_end + self._settling_updates

        if self._update_count <= running_end:
            self._status = MotionStatus.RUNNING
        elif self._update_count <= settling_end:
            self._status = MotionStatus.SETTLING
        else:
            self._status = MotionStatus.SUCCEEDED
            self._result = MotionError.NONE

    def running(self) -> bool:
        return self._status in (MotionStatus.RUNNING, MotionStatus.SETTLING)

    def status(self) -> MotionStatus:
        return self._status

    def succeeded(self) -> bool:
        return self._status is MotionStatus.SUCCEEDED

    def result(self) -> MotionError:
        return self._result

    def lastError(self) -> str:
        return self._last_error

    def cancel(self) -> CancelResult:
        if not self._hardware_ready:
            self._result = MotionError.HARDWARE_NOT_READY
            self._last_error = "motion hardware is not ready"
            return CancelResult.HARDWARE_NOT_READY
        if not self.running():
            return CancelResult.NOT_RUNNING

        self._status = MotionStatus.CANCELLED
        self._result = MotionError.NONE
        self._last_error = ""
        return CancelResult.CANCELLED

    def currentMotion(self) -> Optional[str]:
        return self._current_motion

    @property
    def update_count(self) -> int:
        return self._update_count
