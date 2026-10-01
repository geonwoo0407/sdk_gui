#!/usr/bin/env python3
"""ROS 2 node publishing abstract line-following navigation commands."""

from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .line_navigation_planner import LineNavigationPlanner, NavigationConfig


class LineNavigationController(Node):
    """Translate fresh ``line_info`` messages into safe motion setpoints."""

    def __init__(self) -> None:
        super().__init__("line_navigation_controller")

        self.declare_parameter("line_info_topic", "/vision/line_info")
        self.declare_parameter("command_topic", "/navigation/line_command")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("line_timeout_sec", 0.50)
        self.declare_parameter("min_line_quality", 0.35)
        self.declare_parameter("max_linear_speed_mps", 0.05)
        self.declare_parameter("min_linear_speed_mps", 0.015)
        self.declare_parameter("recovery_lateral_speed_mps", 0.025)
        self.declare_parameter("recovery_turn_speed_rad_s", 0.22)
        self.declare_parameter("recovery_heading_turn_deg", 10.0)
        self.declare_parameter("recovery_away_heading_turn_deg", 3.0)
        self.declare_parameter("curve_follow_max_offset_norm", 0.55)
        self.declare_parameter("recovery_straight_offset_norm", 0.45)
        self.declare_parameter("recovery_parallel_heading_deg", 2.0)
        self.declare_parameter("max_angular_speed_rad_s", 0.60)
        self.declare_parameter("max_angular_accel_rad_s2", 1.20)
        self.declare_parameter("heading_gain", 1.0)
        self.declare_parameter("offset_gain_deg", 24.0)
        self.declare_parameter("preview_gain", 0.15)
        self.declare_parameter("preview_min_turn_deg", 8.0)
        self.declare_parameter("preview_min_consistency", 0.55)
        self.declare_parameter("steering_response_sec", 0.70)
        self.declare_parameter("turn_enter_deg", 12.0)
        self.declare_parameter("turn_exit_deg", 7.0)
        self.declare_parameter("turn_min_heading_deg", 5.0)
        self.declare_parameter("direction_confirmation_frames", 3)
        self.declare_parameter("ambiguity_min_angle_deg", 25.0)
        self.declare_parameter("recovery_enter_offset_norm", 0.20)
        self.declare_parameter("recovery_exit_offset_norm", 0.12)
        self.declare_parameter("command_duration_sec", 0.40)

        config = NavigationConfig(
            min_line_quality=self._float_parameter("min_line_quality"),
            max_linear_speed_mps=self._float_parameter(
                "max_linear_speed_mps"
            ),
            min_linear_speed_mps=self._float_parameter(
                "min_linear_speed_mps"
            ),
            recovery_lateral_speed_mps=self._float_parameter(
                "recovery_lateral_speed_mps"
            ),
            recovery_turn_speed_rad_s=self._float_parameter(
                "recovery_turn_speed_rad_s"
            ),
            recovery_heading_turn_deg=self._float_parameter(
                "recovery_heading_turn_deg"
            ),
            recovery_away_heading_turn_deg=self._float_parameter(
                "recovery_away_heading_turn_deg"
            ),
            curve_follow_max_offset_norm=self._float_parameter(
                "curve_follow_max_offset_norm"
            ),
            recovery_straight_offset_norm=self._float_parameter(
                "recovery_straight_offset_norm"
            ),
            recovery_parallel_heading_deg=self._float_parameter(
                "recovery_parallel_heading_deg"
            ),
            max_angular_speed_rad_s=self._float_parameter(
                "max_angular_speed_rad_s"
            ),
            max_angular_accel_rad_s2=self._float_parameter(
                "max_angular_accel_rad_s2"
            ),
            heading_gain=self._float_parameter("heading_gain"),
            offset_gain_deg=self._float_parameter("offset_gain_deg"),
            preview_gain=self._float_parameter("preview_gain"),
            preview_min_turn_deg=self._float_parameter(
                "preview_min_turn_deg"
            ),
            preview_min_consistency=self._float_parameter(
                "preview_min_consistency"
            ),
            steering_response_sec=self._float_parameter(
                "steering_response_sec"
            ),
            turn_enter_deg=self._float_parameter("turn_enter_deg"),
            turn_exit_deg=self._float_parameter("turn_exit_deg"),
            turn_min_heading_deg=self._float_parameter(
                "turn_min_heading_deg"
            ),
            direction_confirmation_frames=int(
                self.get_parameter("direction_confirmation_frames").value
            ),
            ambiguity_min_angle_deg=self._float_parameter(
                "ambiguity_min_angle_deg"
            ),
            recovery_enter_offset_norm=self._float_parameter(
                "recovery_enter_offset_norm"
            ),
            recovery_exit_offset_norm=self._float_parameter(
                "recovery_exit_offset_norm"
            ),
            command_duration_sec=self._float_parameter(
                "command_duration_sec"
            ),
        )
        self.planner = LineNavigationPlanner(config)
        self.line_timeout_sec = self._float_parameter("line_timeout_sec")
        self.latest_line_info: dict[str, Any] | None = None
        self.latest_line_time: float | None = None
        self.previous_publish_time = time.monotonic()
        self.command_id = 0

        line_topic = str(self.get_parameter("line_info_topic").value)
        command_topic = str(self.get_parameter("command_topic").value)
        self.publisher = self.create_publisher(String, command_topic, 10)
        self.subscription = self.create_subscription(
            String,
            line_topic,
            self._line_callback,
            10,
        )
        publish_rate = max(self._float_parameter("publish_rate_hz"), 1.0)
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self._publish_command,
        )

        self.get_logger().info(f"Subscribing line info: {line_topic}")
        self.get_logger().info(
            f"Publishing navigation command: {command_topic}"
        )

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _line_callback(self, message: String) -> None:
        """Store a validated line analysis object."""
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("line_info JSON must be an object")
            self.latest_line_info = payload
            self.latest_line_time = time.monotonic()
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                f"Invalid line_info message: {type(exc).__name__}: {exc}"
            )

    def _publish_command(self) -> None:
        """Publish a tracking command, or STOP when input is stale."""
        now = time.monotonic()
        dt_sec = now - self.previous_publish_time
        self.previous_publish_time = now

        if self.latest_line_time is None or self.latest_line_info is None:
            command = self.planner.stop("waiting_for_line_info")
            line_age_sec = None
        else:
            line_age_sec = now - self.latest_line_time
            if line_age_sec > self.line_timeout_sec:
                command = self.planner.stop("stale_line_info")
            else:
                command = self.planner.plan(self.latest_line_info, dt_sec)

        self.command_id += 1
        payload = command.to_dict()
        payload.update(
            {
                "command_id": self.command_id,
                "line_age_sec": (
                    round(line_age_sec, 3)
                    if line_age_sec is not None
                    else None
                ),
                "valid_for_sec": round(
                    min(
                        self.line_timeout_sec,
                        command.command_duration_sec,
                    ),
                    3,
                ),
                "source": "line_navigation_controller",
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
    """Run the ROS 2 line navigation controller node."""
    rclpy.init(args=args)
    node = LineNavigationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
