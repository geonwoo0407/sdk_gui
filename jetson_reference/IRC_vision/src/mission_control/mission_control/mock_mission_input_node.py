"""Publish deterministic vision JSON for the mock mission integration."""

from __future__ import annotations

import json
from typing import Any, Optional

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:  # Pure helpers remain testable without ROS.
    rclpy = None
    Node = object
    String = None


SCENARIO_HEADING_ERROR_DEG = {
    "straight": 0.0,
    "turn_left": -10.0,
    "turn_right": 10.0,
}
SUPPORTED_SCENARIOS = frozenset(SCENARIO_HEADING_ERROR_DEG)
REFRESH_PERIOD_SEC = 0.1


class MockScenarioError(ValueError):
    """Raised when a requested deterministic scenario is unsupported."""


def build_mock_vision_input(scenario: str) -> tuple[str, dict[str, Any]]:
    """Return the actual vision topic and JSON-compatible scenario payload."""
    normalized = scenario.strip().lower()
    if normalized not in SUPPORTED_SCENARIOS:
        raise MockScenarioError(f"unsupported mock scenario: {scenario}")

    return (
        "/vision/line_info",
        {
            "detected": True,
            "filtered_heading_error_deg": (
                SCENARIO_HEADING_ERROR_DEG[normalized]
            ),
            "filtered_lateral_offset_norm": 0.0,
            "heading_quality": 1.0,
            "geometry_quality": 1.0,
            "detection_quality": 1.0,
            "turn_angle_deg": 0.0,
            "turn_consistency": 1.0,
        },
    )


def should_publish(publish_once: bool, has_published: bool) -> bool:
    """Decide whether the next timer event may publish."""
    return not (publish_once and has_published)


class MockMissionInputNode(Node):
    """Publish fixed String JSON without camera or detector dependencies."""

    def __init__(self) -> None:
        if rclpy is None or String is None:
            raise RuntimeError("rclpy and std_msgs are required to run mock")

        super().__init__("mock_mission_input_node")
        self.declare_parameter("scenario", "straight")
        self.declare_parameter("publish_delay_sec", 1.0)
        self.declare_parameter("publish_once", True)

        self._scenario = str(self.get_parameter("scenario").value)
        self._publish_once = bool(
            self.get_parameter("publish_once").value
        )
        self._has_published = False
        self._topic, self._payload = build_mock_vision_input(
            self._scenario
        )
        self._publisher = self.create_publisher(
            String, self._topic, 10
        )

        self._publish_delay_sec = max(
            0.01,
            float(self.get_parameter("publish_delay_sec").value),
        )
        self._timer = self.create_timer(
            self._publish_delay_sec,
            self._publish_mock_input,
        )

    def _publish_mock_input(self) -> None:
        if not should_publish(self._publish_once, self._has_published):
            return

        message = String()
        message.data = json.dumps(
            self._payload,
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self._publisher.publish(message)
        self._has_published = True
        self.get_logger().info(
            f"Published mock scenario={self._scenario} on {self._topic}"
        )

        if self._publish_once:
            self.destroy_timer(self._timer)
        elif self._publish_delay_sec != REFRESH_PERIOD_SEC:
            # publish_delay_sec is an initial delay, not a refresh period.
            # Refresh faster than motion_decision_node's 0.5 s timeout.
            self.destroy_timer(self._timer)
            self._timer = self.create_timer(
                REFRESH_PERIOD_SEC,
                self._publish_mock_input,
            )


def main(args: Optional[list] = None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is required to run mock input node")

    rclpy.init(args=args)
    node = MockMissionInputNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
