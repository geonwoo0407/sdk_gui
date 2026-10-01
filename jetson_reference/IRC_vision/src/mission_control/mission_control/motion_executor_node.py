"""Backend-selectable ROS 2 Motion Executor and ROS-free helpers."""

import json
from dataclasses import dataclass
from typing import Any, Dict, Optional

try:
    from .motion_executor_core import MotionExecutionResult, MotionExecutorCore
    from .motion_player_factory import create_motion_player
except ImportError:  # Allows direct, ROS-free unit-test imports.
    from motion_executor_core import MotionExecutionResult, MotionExecutorCore
    from motion_player_factory import create_motion_player

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:  # Pure helpers remain importable without a ROS installation.
    rclpy = None
    Node = object
    String = None


DEFAULT_TICK_PERIOD_MS = 10
DEFAULT_PLAYER_BACKEND = "mock"


@dataclass(frozen=True)
class MotionRequest:
    request_id: int
    motion_id: str
    timeout_ms: int
    command_id: Optional[int] = None
    action: Optional[str] = None
    event_id: Optional[int] = None


@dataclass(frozen=True)
class CancelRequest:
    request_id: int


@dataclass(frozen=True)
class CancelHandlingResult:
    payload: Dict[str, Any]
    terminal: bool


class RequestValidationError(ValueError):

    def __init__(self, error_code: str, message: str) -> None:
        super().__init__(message)
        self.error_code = error_code
        self.message = message


def parse_motion_request(payload: str) -> MotionRequest:
    try:
        data = json.loads(payload)
    except (json.JSONDecodeError, TypeError) as exc:
        raise RequestValidationError(
            "INVALID_REQUEST", f"invalid JSON: {exc}"
        ) from exc

    if not isinstance(data, dict):
        raise RequestValidationError(
            "INVALID_REQUEST", "request must be a JSON object"
        )

    missing = [
        field
        for field in ("request_id", "motion_id", "timeout_ms")
        if field not in data
    ]
    if missing:
        raise RequestValidationError(
            "INVALID_REQUEST",
            f"missing required field: {', '.join(missing)}",
        )

    request_id = data["request_id"]
    motion_id = data["motion_id"]
    timeout_ms = data["timeout_ms"]
    command_id = data.get("command_id")
    action = data.get("action")
    event_id = data.get("event_id")

    if isinstance(request_id, bool) or not isinstance(request_id, int):
        raise RequestValidationError(
            "INVALID_REQUEST", "request_id must be an integer"
        )
    if not isinstance(motion_id, str) or not motion_id:
        raise RequestValidationError(
            "INVALID_REQUEST", "motion_id must be a non-empty string"
        )
    if isinstance(timeout_ms, bool) or not isinstance(timeout_ms, int):
        raise RequestValidationError(
            "INVALID_REQUEST", "timeout_ms must be an integer"
        )
    if timeout_ms <= 0:
        raise RequestValidationError(
            "INVALID_REQUEST", "timeout_ms must be greater than zero"
        )
    if command_id is not None and (
        isinstance(command_id, bool) or not isinstance(command_id, int)
    ):
        raise RequestValidationError(
            "INVALID_REQUEST", "command_id must be an integer or null"
        )
    if action is not None and (
        not isinstance(action, str) or not action.strip()
    ):
        raise RequestValidationError(
            "INVALID_REQUEST", "action must be a non-empty string or null"
        )
    if event_id is not None and (
        isinstance(event_id, bool) or not isinstance(event_id, int)
    ):
        raise RequestValidationError(
            "INVALID_REQUEST", "event_id must be an integer or null"
        )

    return MotionRequest(
        request_id,
        motion_id,
        timeout_ms,
        command_id,
        action,
        event_id,
    )


def parse_cancel_request(payload: str) -> CancelRequest:
    try:
        data = json.loads(payload)
    except (json.JSONDecodeError, TypeError) as exc:
        raise RequestValidationError(
            "INVALID_REQUEST", f"invalid cancel JSON: {exc}"
        ) from exc

    if not isinstance(data, dict):
        raise RequestValidationError(
            "INVALID_REQUEST", "cancel request must be a JSON object"
        )
    if "request_id" not in data:
        raise RequestValidationError(
            "INVALID_REQUEST", "missing required field: request_id"
        )

    request_id = data["request_id"]
    if isinstance(request_id, bool) or not isinstance(request_id, int):
        raise RequestValidationError(
            "INVALID_REQUEST", "request_id must be an integer"
        )
    return CancelRequest(request_id)


def build_status_payload(
    request_id: int,
    motion_id: str,
    status: str,
    error_code: str = "",
    message: str = "",
    command_id: Optional[int] = None,
    action: Optional[str] = None,
    event_id: Optional[int] = None,
) -> Dict[str, Any]:
    return {
        "request_id": request_id,
        "command_id": command_id,
        "event_id": event_id,
        "action": action,
        "motion_id": motion_id,
        "status": status,
        "error_code": error_code,
        "message": message,
    }


class ExecutionPublicationState:
    """Preserves the accepted request and emits its terminal payload once."""

    def __init__(self) -> None:
        self._request: Optional[MotionRequest] = None
        self._terminal_published = False

    def begin(self, request: MotionRequest) -> None:
        self._request = request
        self._terminal_published = False

    def running_payload(self) -> Dict[str, Any]:
        if self._request is None:
            raise RuntimeError("no active request")
        return build_status_payload(
            self._request.request_id,
            self._request.motion_id,
            "RUNNING",
            command_id=self._request.command_id,
            action=self._request.action,
            event_id=self._request.event_id,
        )

    def terminal_payload(
        self, result: MotionExecutionResult
    ) -> Optional[Dict[str, Any]]:
        if self._request is None or self._terminal_published:
            return None

        self._terminal_published = True
        return build_status_payload(
            self._request.request_id,
            self._request.motion_id,
            result.final_status.name,
            result.error_code,
            result.message,
            self._request.command_id,
            self._request.action,
            self._request.event_id,
        )

    def clear(self) -> None:
        self._request = None

    def active_request(self) -> Optional[MotionRequest]:
        return self._request


def handle_cancel_request(
    payload: str,
    core: MotionExecutorCore,
    publication_state: ExecutionPublicationState,
) -> CancelHandlingResult:
    """Process the test-only cancel interface without requiring a ROS graph."""
    try:
        cancel_request = parse_cancel_request(payload)
    except RequestValidationError as exc:
        return CancelHandlingResult(
            build_status_payload(
                0, "", "REJECTED", exc.error_code, exc.message
            ),
            False,
        )

    active_request = publication_state.active_request()
    if not core.busy() or active_request is None:
        return CancelHandlingResult(
            build_status_payload(
                cancel_request.request_id,
                "",
                "REJECTED",
                "NOT_RUNNING",
                "no motion is currently running",
            ),
            False,
        )
    if cancel_request.request_id != active_request.request_id:
        return CancelHandlingResult(
            build_status_payload(
                cancel_request.request_id,
                active_request.motion_id,
                "REJECTED",
                "REQUEST_ID_MISMATCH",
                "cancel request_id does not match the active request",
            ),
            False,
        )

    result = core.cancel()
    if result is None:
        return CancelHandlingResult(
            build_status_payload(
                cancel_request.request_id,
                active_request.motion_id,
                "REJECTED",
                "NOT_RUNNING",
                "player reported that no motion is running",
            ),
            False,
        )

    terminal_payload = publication_state.terminal_payload(result)
    if terminal_payload is None:
        return CancelHandlingResult(
            build_status_payload(
                cancel_request.request_id,
                active_request.motion_id,
                "REJECTED",
                "NOT_RUNNING",
                "terminal result was already published",
            ),
            False,
        )
    return CancelHandlingResult(terminal_payload, True)


class MotionExecutorNode(Node):

    def __init__(self) -> None:
        if rclpy is None or String is None:
            raise RuntimeError("rclpy and std_msgs are required to run the node")

        super().__init__("motion_executor_node")
        self.declare_parameter("tick_period_ms", DEFAULT_TICK_PERIOD_MS)
        self.declare_parameter("player_backend", DEFAULT_PLAYER_BACKEND)
        self.declare_parameter("mock_fail_after_updates", -1)
        self.declare_parameter(
            "mock_failure_code", "COMMUNICATION_ERROR"
        )
        configured_period = self.get_parameter(
            "tick_period_ms"
        ).get_parameter_value().integer_value
        self._tick_period_ms = (
            configured_period
            if configured_period > 0
            else DEFAULT_TICK_PERIOD_MS
        )
        fail_after_updates = self.get_parameter(
            "mock_fail_after_updates"
        ).get_parameter_value().integer_value
        failure_code = self.get_parameter(
            "mock_failure_code"
        ).get_parameter_value().string_value
        player_backend = self.get_parameter(
            "player_backend"
        ).get_parameter_value().string_value
        try:
            player = create_motion_player(
                player_backend,
                mock_fail_after_updates=fail_after_updates,
                mock_failure_code=failure_code,
            )
        except ValueError as exc:
            self.get_logger().error(str(exc))
            raise

        self._core = MotionExecutorCore(player)
        self._publication_state = ExecutionPublicationState()
        self._status_publisher = self.create_publisher(
            String, "/motion/executor/status", 10
        )
        self._request_subscription = self.create_subscription(
            String,
            "/motion/executor/request",
            self._on_request,
            10,
        )
        self._cancel_subscription = self.create_subscription(
            String,
            "/motion/executor/cancel",
            self._on_cancel,
            10,
        )
        self._timer = self.create_timer(
            self._tick_period_ms / 1000.0, self._on_tick
        )

    def _publish_payload(self, payload: Dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self._status_publisher.publish(message)

    def _on_request(self, message: Any) -> None:
        try:
            request = parse_motion_request(message.data)
        except RequestValidationError as exc:
            self._publish_payload(
                build_status_payload(
                    0,
                    "",
                    "REJECTED",
                    exc.error_code,
                    exc.message,
                )
            )
            return

        if self._core.busy():
            self._publish_payload(
                build_status_payload(
                    request.request_id,
                    request.motion_id,
                    "REJECTED",
                    "REJECTED_BUSY",
                    "another motion is already running",
                    request.command_id,
                    request.action,
                    request.event_id,
                )
            )
            return

        if request.motion_id not in MotionExecutorCore.SUPPORTED_MOTION_IDS:
            self._publish_payload(
                build_status_payload(
                    request.request_id,
                    request.motion_id,
                    "REJECTED",
                    "INVALID_MOTION",
                    "unsupported motion_id",
                    request.command_id,
                    request.action,
                    request.event_id,
                )
            )
            return

        rejection = self._core.start_motion(
            request.motion_id, request.timeout_ms
        )
        if rejection is not None:
            self._publish_payload(
                build_status_payload(
                    request.request_id,
                    request.motion_id,
                    "REJECTED",
                    rejection.error_code,
                    rejection.message,
                    request.command_id,
                    request.action,
                    request.event_id,
                )
            )
            return

        self._publication_state.begin(request)
        self._publish_payload(self._publication_state.running_payload())

    def _on_cancel(self, message: Any) -> None:
        handled = handle_cancel_request(
            message.data, self._core, self._publication_state
        )
        self._publish_payload(handled.payload)
        if handled.terminal:
            self._core.reset()
            self._publication_state.clear()

    def _on_tick(self) -> None:
        result = self._core.tick(self._tick_period_ms)
        if result is None:
            return

        payload = self._publication_state.terminal_payload(result)
        if payload is None:
            return

        self._publish_payload(payload)
        self._core.reset()
        self._publication_state.clear()


def main(args: Optional[list] = None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is required to run motion_executor_node")

    rclpy.init(args=args)
    node = MotionExecutorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
