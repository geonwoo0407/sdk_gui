#!/usr/bin/env python3
"""Publish exactly one explicitly selected SDK motion request."""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


TERMINAL_STATUSES = {"SUCCEEDED", "FAILED", "CANCELLED", "REJECTED"}


def build_request(motion_id: str, timeout_ms: int, request_id: int) -> dict:
    """Build the manual request while preserving the executor contract."""
    if not motion_id:
        raise ValueError("motion_id must be explicitly specified")
    if isinstance(timeout_ms, bool) or timeout_ms <= 0:
        raise ValueError("timeout_ms must be a positive integer")
    return {
        "request_id": request_id,
        "command_id": request_id,
        "event_id": None,
        "action": "MANUAL_TEST",
        "motion_id": motion_id,
        "timeout_ms": timeout_ms,
    }


class ManualMotionRequest(Node):
    """Send one request after an executor subscriber becomes available."""

    def __init__(self) -> None:
        """Read the explicit selection and prepare a one-shot publisher."""
        super().__init__("manual_motion_request")
        self.declare_parameter("motion_id", "")
        self.declare_parameter("timeout_ms", 15000)
        motion_id = self.get_parameter("motion_id").value
        timeout_ms = self.get_parameter("timeout_ms").value
        request_id = time.time_ns() // 1_000_000
        self.request = build_request(motion_id, timeout_ms, request_id)
        self.finished = False
        self.published = False
        self.publisher = self.create_publisher(
            String, "/motion/executor/request", 10
        )
        self.subscription = self.create_subscription(
            String, "/motion/executor/status", self._status_callback, 10
        )
        self.timer = self.create_timer(0.1, self._publish_once)

    def _publish_once(self) -> None:
        if self.published or self.publisher.get_subscription_count() == 0:
            return
        message = String()
        message.data = json.dumps(
            self.request, ensure_ascii=True, separators=(",", ":")
        )
        self.publisher.publish(message)
        self.published = True
        self.timer.cancel()
        self.get_logger().warning(
            "Published exactly one manual request: "
            f"request_id={self.request['request_id']} "
            f"motion_id={self.request['motion_id']} "
            f"timeout_ms={self.request['timeout_ms']}"
        )

    def _status_callback(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if status.get("request_id") != self.request["request_id"]:
            return
        state = status.get("status", "UNKNOWN")
        error_code = status.get("error_code", "")
        detail = status.get("message", "")
        self.get_logger().info(
            f"status={state} error_code={error_code!r} message={detail!r}"
        )
        if state in TERMINAL_STATUSES:
            self.finished = True


def main(args=None) -> int:
    """Run until the matching request reaches a terminal status."""
    rclpy.init(args=args)
    try:
        node = ManualMotionRequest()
    except (TypeError, ValueError) as exc:
        print(f"manual motion request rejected: {exc}", file=sys.stderr)
        rclpy.shutdown()
        return 2
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.get_logger().warning(
            "Interrupted locally; no emergency-stop command was invented. "
            "Use the documented executor cancel request if appropriate."
        )
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
