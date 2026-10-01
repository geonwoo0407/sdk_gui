"""
Adapt Motion Executor status JSON to the legacy mission status shape.

The pure conversion helpers intentionally have no dependency on the robot SDK
or Dynamixel.  ROS imports are optional so the conversion can be unit tested
without starting a ROS graph.
"""

from __future__ import annotations

import json
from typing import Any, Mapping, Optional, Union

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:  # Pure helpers remain testable without ROS.
    rclpy = None
    Node = object
    String = None


SUPPORTED_STATUSES = {
    "RUNNING",
    "SUCCEEDED",
    "FAILED",
    "CANCELLED",
    "TIMEOUT",
    "REJECTED",
}

# The command adapter is intentionally lossy: several legacy actions can map
# to one motion_id.  These canonical names preserve the special-action names
# consumed by motion_decision_node where the mapping is unambiguous.
MOTION_ID_TO_LEGACY_ACTION = {
    "forward": "STRAIGHT",
    "forward_short": "SLOW_APPROACH",
    "adjust_left": "ALIGN_LEFT",
    "adjust_right": "ALIGN_RIGHT",
    "backward": "RETREAT_GOAL",
    "pick_ball": "PICKUP_NOW",
    "shoot": "SHOT",
    "turn_left": "TURN_LEFT",
    "turn_right": "TURN_RIGHT",
    "hurdle": "CROSS_FINISH",
}


class ExecutorStatusValidationError(ValueError):
    """Raised when one Executor status message cannot be adapted safely."""


def convert_executor_status(
    payload: Union[str, Mapping[str, Any]],
) -> dict[str, Any]:
    """Return one legacy-compatible status object for one Executor status."""
    if isinstance(payload, str):
        try:
            data = json.loads(payload)
        except (json.JSONDecodeError, TypeError) as exc:
            raise ExecutorStatusValidationError(
                f"invalid executor status JSON: {exc}"
            ) from exc
    elif isinstance(payload, Mapping):
        data = payload
    else:
        raise ExecutorStatusValidationError(
            "executor status must be JSON text or an object"
        )

    if not isinstance(data, Mapping):
        raise ExecutorStatusValidationError(
            "executor status must be a JSON object"
        )

    if "status" not in data:
        raise ExecutorStatusValidationError(
            "executor status is missing status"
        )

    raw_status = data["status"]
    if not isinstance(raw_status, str) or not raw_status.strip():
        raise ExecutorStatusValidationError(
            "executor status must be a non-empty string"
        )

    status = raw_status.strip().upper()
    if status not in SUPPORTED_STATUSES:
        raise ExecutorStatusValidationError(
            f"unknown executor status: {raw_status}"
        )

    request_id = data.get("request_id")
    command_id = data.get("command_id")
    motion_id = data.get("motion_id")
    original_action = data.get("action")
    action = (
        original_action
        if isinstance(original_action, str) and original_action
        else MOTION_ID_TO_LEGACY_ACTION.get(motion_id)
    )

    return {
        # Fields consumed by the existing /motion/status subscriber.
        "status": status,
        "command_id": command_id,
        "event_id": data.get("event_id"),
        "action": action,
        "dynamics_command": None,
        "motion_in_progress": status == "RUNNING",
        "reason": "motion_executor_status",
        # Additional Executor fields are safe for the legacy JSON consumer.
        "request_id": request_id,
        "motion_id": motion_id,
        "error_code": data.get("error_code", ""),
        "message": data.get("message", ""),
    }


def convert_executor_status_json(payload: str) -> str:
    """Convert one Executor JSON string to one legacy JSON string."""
    return json.dumps(convert_executor_status(payload))


class LegacyMotionStatusAdapter(Node):
    """Relay new Executor statuses on the legacy mission status topic."""

    def __init__(self) -> None:
        if rclpy is None or String is None:
            raise RuntimeError("rclpy and std_msgs are required to run adapter")

        super().__init__("legacy_motion_status_adapter")
        self._legacy_publisher = self.create_publisher(
            String, "/motion/status", 10
        )
        self._executor_subscription = self.create_subscription(
            String,
            "/motion/executor/status",
            self._on_executor_status,
            10,
        )

    def _on_executor_status(self, message: Any) -> None:
        try:
            converted = convert_executor_status_json(message.data)
        except ExecutorStatusValidationError as exc:
            self.get_logger().warning(str(exc))
            return

        output = String()
        output.data = converted
        self._legacy_publisher.publish(output)


def main(args: Optional[list] = None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is required to run adapter")

    rclpy.init(args=args)
    node = LegacyMotionStatusAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
