"""ROS-independent mission-level safety latch for executor faults."""

from __future__ import annotations

from dataclasses import asdict
from dataclasses import dataclass
from typing import Any


NORMAL = "NORMAL"
CRITICAL_LATCHED = "CRITICAL_LATCHED"
EXECUTOR_RUNTIME_CRITICAL = "EXECUTOR_RUNTIME_CRITICAL"

RECOVERABLE_MOTOR_ERROR_CODES = frozenset(
    {
        "SDK_COMMUNICATION_ERROR",
        "SDK_FRAME_SEND_FAILED",
        "SDK_PRESENT_POSITION_READ_FAILED",
        "SDK_POSITION_TIMEOUT",
    }
)

CRITICAL_EXECUTOR_ERROR_CODES = frozenset(
    {
        "SDK_HARDWARE_NOT_READY",
        "SDK_CANCEL_FAILED",
        "SDK_CANCEL_HOLD_FAILED",
        "SDK_INTERNAL_ERROR",
        "BACKEND_EXCEPTION",
        "BACKEND_FAILED",
        "BACKEND_UNEXPECTED_IDLE",
        "SDK_EXCEPTION",
        "SDK_UNKNOWN_ERROR",
        "SDK_UNKNOWN_STATUS",
        "EXECUTOR_HEARTBEAT_TIMEOUT",
    }
)


@dataclass(frozen=True)
class SafetySnapshot:
    """Describe the first critical fault retained by the latch."""

    state: str = NORMAL
    latched: bool = False
    category: str = ""
    error_code: str = ""
    message: str = ""
    source: str = ""
    action: str | None = None
    command_id: int | None = None
    event_id: int | None = None

    def to_dict(self) -> dict[str, Any]:
        """Return a detached representation suitable for diagnostics."""
        return asdict(self)


class SafetyInterlock:
    """Latch the first correlated critical executor fault until restart."""

    def __init__(self) -> None:
        self._snapshot = SafetySnapshot()

    @property
    def latched(self) -> bool:
        return self._snapshot.latched

    @property
    def snapshot(self) -> SafetySnapshot:
        return self._snapshot

    def observe_executor_status(
        self,
        *,
        error_code: Any,
        message: Any,
        action: Any,
        command_id: Any,
        event_id: Any,
    ) -> bool:
        """Latch a known critical code and report whether latching occurred."""
        return self.observe_critical_fault(
            error_code=error_code,
            message=message,
            source="motion_executor",
            action=action,
            command_id=command_id,
            event_id=event_id,
        )

    def observe_critical_fault(
        self,
        *,
        error_code: Any,
        message: Any,
        source: Any,
        action: Any,
        command_id: Any,
        event_id: Any,
    ) -> bool:
        """Latch the first recognized critical fault from any safety source."""
        if self.latched:
            return False
        normalized_code = (
            error_code.strip().upper()
            if isinstance(error_code, str)
            else ""
        )
        if normalized_code not in CRITICAL_EXECUTOR_ERROR_CODES:
            return False

        self._snapshot = SafetySnapshot(
            state=CRITICAL_LATCHED,
            latched=True,
            category=EXECUTOR_RUNTIME_CRITICAL,
            error_code=normalized_code,
            message=message if isinstance(message, str) else "",
            source=source if isinstance(source, str) else "",
            action=action if isinstance(action, str) else None,
            command_id=(
                command_id
                if isinstance(command_id, int) and not isinstance(command_id, bool)
                else None
            ),
            event_id=(
                event_id
                if isinstance(event_id, int) and not isinstance(event_id, bool)
                else None
            ),
        )
        return True
