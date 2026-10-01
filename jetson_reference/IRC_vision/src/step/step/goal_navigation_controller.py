#!/usr/bin/env python3
"""ROS 2 node publishing abstract goal-scoring action candidates."""

from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .goal_navigation_planner import GoalNavigationConfig
from .goal_navigation_planner import GoalNavigationPlanner


class GoalNavigationController(Node):
    """Validate fresh goal information and publish SDK action candidates."""

    def __init__(self) -> None:
        super().__init__("goal_navigation_controller")

        self.declare_parameter("goal_info_topic", "/vision/goal_info")
        self.declare_parameter("command_topic", "/navigation/goal_command")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("goal_timeout_sec", 0.50)
        self.declare_parameter("min_confidence", 0.35)
        self.declare_parameter("control_start_depth_m", 0.50)
        self.declare_parameter("score_target_depth_m", 0.25)
        self.declare_parameter("score_depth_tolerance_m", 0.05)
        self.declare_parameter("score_center_tolerance_norm", 0.10)

        config = GoalNavigationConfig(
            min_confidence=self._float_parameter("min_confidence"),
            control_start_depth_m=self._float_parameter(
                "control_start_depth_m"
            ),
            score_target_depth_m=self._float_parameter(
                "score_target_depth_m"
            ),
            score_depth_tolerance_m=self._float_parameter(
                "score_depth_tolerance_m"
            ),
            score_center_tolerance_norm=self._float_parameter(
                "score_center_tolerance_norm"
            ),
        )
        self.planner = GoalNavigationPlanner(config)
        self.goal_timeout_sec = self._float_parameter("goal_timeout_sec")
        self.latest_goal_info: dict[str, Any] | None = None
        self.latest_goal_time: float | None = None
        self.command_id = 0

        goal_topic = str(self.get_parameter("goal_info_topic").value)
        command_topic = str(self.get_parameter("command_topic").value)
        self.publisher = self.create_publisher(String, command_topic, 10)
        self.subscription = self.create_subscription(
            String,
            goal_topic,
            self._goal_callback,
            10,
        )
        publish_rate = max(self._float_parameter("publish_rate_hz"), 1.0)
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self._publish_command,
        )

        self.get_logger().info(f"Subscribing goal info: {goal_topic}")
        self.get_logger().info(
            f"Publishing goal action: {command_topic}"
        )

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _goal_callback(self, message: String) -> None:
        """Store one validated goal analysis object."""
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("goal_info JSON must be an object")
            self.latest_goal_info = payload
            self.latest_goal_time = time.monotonic()
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                f"Invalid goal_info message: {type(exc).__name__}: {exc}"
            )

    def _publish_command(self) -> None:
        """Publish a fresh goal action, or WAIT after input timeout."""
        now = time.monotonic()
        if self.latest_goal_time is None or self.latest_goal_info is None:
            command = self.planner.wait("waiting_for_goal_info")
            goal_age_sec = None
        else:
            goal_age_sec = now - self.latest_goal_time
            if goal_age_sec > self.goal_timeout_sec:
                command = self.planner.wait("stale_goal_info")
            else:
                command = self.planner.plan(self.latest_goal_info)

        self.command_id += 1
        payload = command.to_dict()
        payload.update(
            {
                "command_id": self.command_id,
                "goal_age_sec": (
                    round(goal_age_sec, 3)
                    if goal_age_sec is not None
                    else None
                ),
                "valid_for_sec": round(self.goal_timeout_sec, 3),
                "source": "goal_navigation_controller",
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
    """Run the ROS 2 goal navigation controller node."""
    rclpy.init(args=args)
    node = GoalNavigationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
