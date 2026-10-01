"""
Parallel adapter for mock Motion Executor integration validation.

This module does not replace the existing motion_command_bridge_node.
"""

import json
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional, Union

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:  # Pure helpers remain testable without ROS.
    rclpy = None
    Node = object
    String = None


ACTION_TO_MOTION_ID = {
    "STRAIGHT": "forward",
    "APPROACH": "forward",
    "SLOW_APPROACH": "forward_short",
    "FINE_FORWARD_STEP": "forward_short",
    "APPROACH_GOAL": "forward_short",
    "APPROACH_HURDLE": "forward_short",
    "ALIGN_LEFT": "adjust_left",
    "ALIGN_RIGHT": "adjust_right",
    "RETREAT_GOAL": "backward",
    "PICKUP_NOW": "pick_ball",
    "SHOT": "shoot",
    "GO": "forward",
    "TURN_LEFT": "turn_left",
    "TURN_RIGHT": "turn_right",
    "LEFT": "turn_left",
    "RIGHT": "turn_right",
    "CROSS_FINISH": "hurdle",
}
UNSUPPORTED_ACTIONS: frozenset[str] = frozenset()

DEFAULT_TIMEOUT_MS = 5000
MOTION_TIMEOUT_MS = {
    "pick_ball": 10000,
    "shoot": 10000,
    "hurdle": 12000,
    "recover": 8000,
}


@dataclass(frozen=True)
class LegacyMotionCommand:
    action: str
    angle_deg: Any = None
    valid: Any = None
    command_id: Optional[int] = None
    event_id: Optional[int] = None


class LegacyCommandValidationError(ValueError):
    pass


def parse_legacy_motion_command(
    payload: Union[str, Mapping[str, Any]]
) -> LegacyMotionCommand:
    if isinstance(payload, str):
        try:
            data = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise LegacyCommandValidationError(
                f"invalid legacy command JSON: {exc}"
            ) from exc
    elif isinstance(payload, Mapping):
        data = payload
    else:
        raise LegacyCommandValidationError(
            "legacy command must be JSON text or an object"
        )

    if not isinstance(data, Mapping):
        raise LegacyCommandValidationError(
            "legacy command must be a JSON object"
        )

    action = data.get("action")
    if not isinstance(action, str) or not action:
        raise LegacyCommandValidationError(
            "action must be a non-empty string"
        )

    command_id = data.get("command_id")
    if command_id is not None and (
        isinstance(command_id, bool) or not isinstance(command_id, int)
    ):
        raise LegacyCommandValidationError(
            "command_id must be an integer or null"
        )

    event_id = data.get("event_id")
    if event_id is not None and (
        isinstance(event_id, bool) or not isinstance(event_id, int)
    ):
        raise LegacyCommandValidationError(
            "event_id must be an integer or null"
        )

    # angle_deg is parsed for compatibility but is not sent to the Executor.
    return LegacyMotionCommand(
        action=action,
        angle_deg=data.get("angle_deg"),
        valid=data.get("valid"),
        command_id=command_id,
        event_id=event_id,
    )


def map_action_to_motion_id(action: str) -> Optional[str]:
    return ACTION_TO_MOTION_ID.get(action)


def timeout_for_motion(motion_id: str) -> int:
    return MOTION_TIMEOUT_MS.get(motion_id, DEFAULT_TIMEOUT_MS)


def build_executor_request(
    request_id: int,
    motion_id: str,
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
        "timeout_ms": timeout_for_motion(motion_id),
    }


class LegacyMotionExecutorAdapter(Node):
    """Mock-integration adapter running in parallel with the legacy bridge."""

    def __init__(self) -> None:
        if rclpy is None or String is None:
            raise RuntimeError("rclpy and std_msgs are required to run adapter")

        super().__init__("legacy_motion_executor_adapter")
        self._next_request_id = 1
        self._request_publisher = self.create_publisher(
            String, "/motion/executor/request", 10
        )
        self._legacy_subscription = self.create_subscription(
            String,
            "/navigation/motion_command",
            self._on_legacy_command,
            10,
        )

    def _on_legacy_command(self, message: Any) -> None:
        try:
            command = parse_legacy_motion_command(message.data)
        except LegacyCommandValidationError as exc:
            self.get_logger().warning(str(exc))
            return

        if command.valid is False:
            self.get_logger().warning(
                f"invalid legacy command ignored: {command.action}"
            )
            return

        if command.action in UNSUPPORTED_ACTIONS:
            self.get_logger().warning(
                "unsupported legacy action (motion mapping not configured): "
                f"{command.action}"
            )
            return

        motion_id = map_action_to_motion_id(command.action)
        if motion_id is None:
            self.get_logger().warning(
                f"unsupported legacy action: {command.action}"
            )
            return

        request = build_executor_request(
            self._next_request_id,
            motion_id,
            command.command_id,
            command.action,
            command.event_id,
        )
        output = String()
        output.data = json.dumps(request)
        self._request_publisher.publish(output)
        self._next_request_id += 1


def main(args: Optional[list] = None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is required to run adapter")

    rclpy.init(args=args)
    node = LegacyMotionExecutorAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
