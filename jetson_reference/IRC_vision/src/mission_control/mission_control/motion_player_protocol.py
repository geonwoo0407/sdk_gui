"""Structural interface shared by Motion Executor player backends."""

from typing import Any, Optional, Protocol, runtime_checkable


@runtime_checkable
class MotionPlayerProtocol(Protocol):

    def hardwareReady(self) -> bool:
        ...

    def start(self, motion_id: str) -> Any:
        ...

    def update(self) -> None:
        ...

    def running(self) -> bool:
        ...

    def status(self) -> Any:
        ...

    def succeeded(self) -> bool:
        ...

    def result(self) -> Any:
        ...

    def lastError(self) -> str:
        ...

    def cancel(self) -> Any:
        ...

    def currentMotion(self) -> Optional[str]:
        ...
