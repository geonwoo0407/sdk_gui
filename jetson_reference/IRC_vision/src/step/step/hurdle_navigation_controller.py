#!/usr/bin/env python3
"""ROS 2 node publishing abstract hurdle jump action candidates."""

from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .hurdle_navigation_planner import HurdleNavigationConfig
from .hurdle_navigation_planner import HurdleNavigationPlanner


class HurdleNavigationController(Node):
    """Validate hurdle information and publish SDK action candidates."""

    def __init__(self) -> None:
        super().__init__("hurdle_navigation_controller")

        self.declare_parameter("hurdle_info_topic", "/vision/hurdle_info")
        self.declare_parameter(
            "command_topic",
            "/navigation/hurdle_command",
        )
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("hurdle_timeout_sec", 0.50)
        self.declare_parameter("min_confidence", 0.60)
        self.declare_parameter("go_target_depth_m", 0.10)
        self.declare_parameter("go_depth_tolerance_m", 0.10)
        self.declare_parameter("go_angle_tolerance_deg", 8.0)

        config = HurdleNavigationConfig(
            min_confidence=self._float_parameter("min_confidence"),
            go_target_depth_m=self._float_parameter(
                "go_target_depth_m"
            ),
            go_depth_tolerance_m=self._float_parameter(
                "go_depth_tolerance_m"
            ),
            go_angle_tolerance_deg=self._float_parameter(
                "go_angle_tolerance_deg"
            ),
        )
        self.planner = HurdleNavigationPlanner(config)
        self.hurdle_timeout_sec = self._float_parameter(
            "hurdle_timeout_sec"
        )
        self.latest_hurdle_info: dict[str, Any] | None = None
        self.latest_hurdle_time: float | None = None
        self.command_id = 0

        hurdle_topic = str(self.get_parameter("hurdle_info_topic").value)
        command_topic = str(self.get_parameter("command_topic").value)
        self.publisher = self.create_publisher(String, command_topic, 10)
        self.subscription = self.create_subscription(
            String,
            hurdle_topic,
            self._hurdle_callback,
            10,
        )
        publish_rate = max(self._float_parameter("publish_rate_hz"), 1.0)
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self._publish_command,
        )

        self.get_logger().info(
            f"Subscribing hurdle info: {hurdle_topic}"
        )
        self.get_logger().info(
            f"Publishing hurdle action: {command_topic}"
        )

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _hurdle_callback(self, message: String) -> None:
        """Store one validated hurdle analysis object."""
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("hurdle_info JSON must be an object")
            self.latest_hurdle_info = payload
            self.latest_hurdle_time = time.monotonic()
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                f"Invalid hurdle_info: {type(exc).__name__}: {exc}"
            )

    def _publish_command(self) -> None:
        """Publish a fresh hurdle action, or WAIT after input timeout."""
        now = time.monotonic()
        if (
            self.latest_hurdle_time is None
            or self.latest_hurdle_info is None
        ):
            command = self.planner.wait("waiting_for_hurdle_info")
            hurdle_age_sec = None
        else:
            hurdle_age_sec = now - self.latest_hurdle_time
            if hurdle_age_sec > self.hurdle_timeout_sec:
                command = self.planner.wait("stale_hurdle_info")
            else:
                command = self.planner.plan(self.latest_hurdle_info)

        self.command_id += 1
        payload = command.to_dict()
        payload.update(
            {
                "command_id": self.command_id,
                "hurdle_age_sec": (
                    round(hurdle_age_sec, 3)
                    if hurdle_age_sec is not None
                    else None
                ),
                "valid_for_sec": round(self.hurdle_timeout_sec, 3),
                "source": "hurdle_navigation_controller",
            }
        )
        output = String()
        output.data = json.dumps(
            payload,
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self.publisher.publish(output)


def main(args: list[str] | None = None) -> None:
    """Run the ROS 2 hurdle navigation controller node."""
    rclpy.init(args=args)
    node = HurdleNavigationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
