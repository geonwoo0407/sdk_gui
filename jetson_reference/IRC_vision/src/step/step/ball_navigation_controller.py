#!/usr/bin/env python3
"""ROS 2 node publishing abstract ball-approach navigation commands."""

from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .ball_navigation_planner import BallNavigationConfig
from .ball_navigation_planner import BallNavigationPlanner


class BallNavigationController(Node):
    """Translate fresh ``ball_info`` into safe abstract motion targets."""

    def __init__(self) -> None:
        super().__init__("ball_navigation_controller")

        self.declare_parameter("ball_info_topic", "/vision/ball_info")
        self.declare_parameter("command_topic", "/navigation/ball_command")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("ball_timeout_sec", 0.50)
        self.declare_parameter("min_confidence", 0.35)
        self.declare_parameter("max_linear_speed_mps", 0.04)
        self.declare_parameter("min_linear_speed_mps", 0.012)
        self.declare_parameter("max_angular_speed_rad_s", 0.50)
        self.declare_parameter("max_angular_accel_rad_s2", 1.00)
        self.declare_parameter("bearing_gain", 1.0)
        self.declare_parameter("steering_response_sec", 0.70)
        self.declare_parameter("turn_enter_deg", 5.0)
        self.declare_parameter("turn_exit_deg", 2.5)
        self.declare_parameter("path_center_deadband_norm", 0.05)
        self.declare_parameter("path_center_deadband_max_depth_m", 0.60)
        self.declare_parameter("fallback_half_fov_deg", 35.0)
        self.declare_parameter("control_start_depth_m", 1.50)
        self.declare_parameter("max_pickup_depth_age_sec", 0.70)
        self.declare_parameter("slowdown_depth_m", 1.0)
        self.declare_parameter("fine_step_depth_m", 0.95)
        self.declare_parameter("pickup_depth_m", 0.07)
        self.declare_parameter("pickup_trigger_depth_m", 0.48)
        self.declare_parameter("pickup_depth_tolerance_m", 0.02)
        self.declare_parameter("command_duration_sec", 0.40)

        config = BallNavigationConfig(
            min_confidence=self._float_parameter("min_confidence"),
            max_linear_speed_mps=self._float_parameter(
                "max_linear_speed_mps"
            ),
            min_linear_speed_mps=self._float_parameter(
                "min_linear_speed_mps"
            ),
            max_angular_speed_rad_s=self._float_parameter(
                "max_angular_speed_rad_s"
            ),
            max_angular_accel_rad_s2=self._float_parameter(
                "max_angular_accel_rad_s2"
            ),
            bearing_gain=self._float_parameter("bearing_gain"),
            steering_response_sec=self._float_parameter(
                "steering_response_sec"
            ),
            turn_enter_deg=self._float_parameter("turn_enter_deg"),
            turn_exit_deg=self._float_parameter("turn_exit_deg"),
            path_center_deadband_norm=self._float_parameter(
                "path_center_deadband_norm"
            ),
            path_center_deadband_max_depth_m=self._float_parameter(
                "path_center_deadband_max_depth_m"
            ),
            fallback_half_fov_deg=self._float_parameter(
                "fallback_half_fov_deg"
            ),
            control_start_depth_m=self._float_parameter(
                "control_start_depth_m"
            ),
            max_pickup_depth_age_sec=self._float_parameter(
                "max_pickup_depth_age_sec"
            ),
            slowdown_depth_m=self._float_parameter("slowdown_depth_m"),
            fine_step_depth_m=self._float_parameter("fine_step_depth_m"),
            pickup_depth_m=self._float_parameter("pickup_depth_m"),
            pickup_trigger_depth_m=self._float_parameter(
                "pickup_trigger_depth_m"
            ),
            pickup_depth_tolerance_m=self._float_parameter(
                "pickup_depth_tolerance_m"
            ),
            command_duration_sec=self._float_parameter(
                "command_duration_sec"
            ),
        )
        self.planner = BallNavigationPlanner(config)
        self.ball_timeout_sec = self._float_parameter("ball_timeout_sec")
        self.latest_ball_info: dict[str, Any] | None = None
        self.latest_ball_time: float | None = None
        self.previous_publish_time = time.monotonic()
        self.command_id = 0

        ball_topic = str(self.get_parameter("ball_info_topic").value)
        command_topic = str(self.get_parameter("command_topic").value)
        self.publisher = self.create_publisher(String, command_topic, 10)
        self.subscription = self.create_subscription(
            String,
            ball_topic,
            self._ball_callback,
            10,
        )
        publish_rate = max(self._float_parameter("publish_rate_hz"), 1.0)
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self._publish_command,
        )

        self.get_logger().info(f"Subscribing ball info: {ball_topic}")
        self.get_logger().info(
            f"Publishing navigation command: {command_topic}"
        )

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _ball_callback(self, message: String) -> None:
        """Store one validated ball analysis object."""
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("ball_info JSON must be an object")
            self.latest_ball_info = payload
            self.latest_ball_time = time.monotonic()
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                f"Invalid ball_info message: {type(exc).__name__}: {exc}"
            )

    def _publish_command(self) -> None:
        """Publish a ball command, or STOP when input is stale."""
        now = time.monotonic()
        dt_sec = now - self.previous_publish_time
        self.previous_publish_time = now

        if self.latest_ball_time is None or self.latest_ball_info is None:
            command = self.planner.stop("waiting_for_ball_info")
            ball_age_sec = None
        else:
            ball_age_sec = now - self.latest_ball_time
            if ball_age_sec > self.ball_timeout_sec:
                command = self.planner.stop("stale_ball_info")
            else:
                command = self.planner.plan(self.latest_ball_info, dt_sec)

        self.command_id += 1
        payload = command.to_dict()
        payload.update(
            {
                "command_id": self.command_id,
                "ball_age_sec": (
                    round(ball_age_sec, 3)
                    if ball_age_sec is not None
                    else None
                ),
                "valid_for_sec": round(
                    min(
                        self.ball_timeout_sec,
                        command.command_duration_sec,
                    ),
                    3,
                ),
                "source": "ball_navigation_controller",
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
    """Run the ROS 2 ball navigation controller node."""
    rclpy.init(args=args)
    node = BallNavigationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
