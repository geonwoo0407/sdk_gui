"""
Safe placeholder for an SDK backend that is not connected yet.

No SDK, serial, Dynamixel, file, or hardware API is imported or accessed.
"""

from typing import Optional

try:
    from .mock_motion_player import (
        CancelResult,
        MotionError,
        MotionStatus,
        StartResult,
    )
except ImportError:  # Allows direct unit-test imports.
    from mock_motion_player import (
        CancelResult,
        MotionError,
        MotionStatus,
        StartResult,
    )


class SdkMotionPlayerPlaceholder:
    ERROR_MESSAGE = "SDK backend is not connected yet"

    def hardwareReady(self) -> bool:
        return False

    def start(self, motion_id: str) -> StartResult:
        del motion_id
        return StartResult.HARDWARE_NOT_READY

    def update(self) -> None:
        return None

    def running(self) -> bool:
        return False

    def status(self) -> MotionStatus:
        return MotionStatus.IDLE

    def succeeded(self) -> bool:
        return False

    def result(self) -> MotionError:
        return MotionError.HARDWARE_NOT_READY

    def lastError(self) -> str:
        return self.ERROR_MESSAGE

    def cancel(self) -> CancelResult:
        return CancelResult.HARDWARE_NOT_READY

    def currentMotion(self) -> Optional[str]:
        return None
