#!/usr/bin/env python3
"""
Compact terminal monitor for /vision/line_info.

This node does not calculate line geometry.
It only reads the JSON published by yolo_line_analyzer and displays
the most useful values in a compact human-readable format.
"""

from __future__ import annotations

import json
import os
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LineDebugMonitor(Node):
    """Display compact line-analysis information in the terminal."""

    def __init__(self) -> None:
        super().__init__("line_debug_monitor")

        self.declare_parameter(
            "line_info_topic",
            "/vision/line_info",
        )

        self.declare_parameter(
            "clear_screen",
            True,
        )

        self.declare_parameter(
            "show_center_points",
            False,
        )

        topic = str(
            self.get_parameter("line_info_topic").value
        )

        self.clear_screen = bool(
            self.get_parameter("clear_screen").value
        )

        self.show_center_points = bool(
            self.get_parameter("show_center_points").value
        )

        self.subscription = self.create_subscription(
            String,
            topic,
            self._callback,
            10,
        )

        self.get_logger().info(
            f"Monitoring: {topic}"
        )

    @staticmethod
    def _format_value(
        value: Any,
        decimals: int = 2,
        suffix: str = "",
    ) -> str:
        """Format optional numeric values safely."""

        if value is None:
            return "null"

        try:
            return f"{float(value):.{decimals}f}{suffix}"

        except (TypeError, ValueError):
            return str(value)

    @staticmethod
    def _quality_bar(
        value: Any,
        width: int = 10,
    ) -> str:
        """Create a small ASCII quality bar."""

        if value is None:
            return "[" + "-" * width + "]"

        try:
            numeric = max(
                0.0,
                min(
                    1.0,
                    float(value),
                ),
            )

        except (TypeError, ValueError):
            return "[" + "?" * width + "]"

        filled = int(
            round(
                numeric * width
            )
        )

        return (
            "["
            + "#" * filled
            + "-" * (width - filled)
            + "]"
        )

    def _callback(
        self,
        message: String,
    ) -> None:
        """Read one line-info JSON message and display it."""

        try:
            data = json.loads(
                message.data
            )

        except json.JSONDecodeError as exc:
            self.get_logger().warning(
                f"Invalid JSON: {exc}"
            )
            return

        if self.clear_screen:
            os.system(
                "clear"
            )

        detected = bool(
            data.get(
                "detected",
                False,
            )
        )

        line_count = int(
            data.get(
                "line_count",
                0,
            )
        )

        print(
            "=" * 64
        )

        print(
            " IRC VISION - LINE DEBUG MONITOR"
        )

        print(
            "=" * 64
        )

        print()

        if not detected:

            print(
                " LINE: NOT DETECTED"
            )

            print(
                f" missed_line_frames : "
                f"{data.get('missed_line_frames', 0)}"
            )

            print(
                f" filter_history     : "
                f"{data.get('filter_history_size', 0)}"
            )

            print()

            print(
                " Filtered values retained from previous valid frame:"
            )

            print(
                " filtered_heading    : "
                + self._format_value(
                    data.get(
                        "filtered_heading_error_deg"
                    ),
                    decimals=2,
                    suffix=" deg",
                )
            )

            print(
                " filtered_offset      : "
                + self._format_value(
                    data.get(
                        "filtered_lateral_offset_norm"
                    ),
                    decimals=4,
                )
            )

            print()

            print(
                "=" * 64
            )

            return

        raw_heading = data.get(
            "heading_error_deg"
        )

        filtered_heading = data.get(
            "filtered_heading_error_deg"
        )

        raw_offset = data.get(
            "lateral_offset_norm"
        )

        filtered_offset = data.get(
            "filtered_lateral_offset_norm"
        )

        turn_consistency = data.get(
            "turn_consistency"
        )

        angle_change_mean = data.get(
            "angle_change_mean_deg"
        )

        angle_change_max = data.get(
            "angle_change_max_deg"
        )

        heading_quality = data.get(
            "heading_quality"
        )

        geometry_quality = data.get(
            "geometry_quality"
        )

        detection_quality = data.get(
            "detection_quality"
        )

        print(
            f" DETECTED : YES"
            f"    LINE COUNT : {line_count}"
        )

        print()

        print(
            "--- CURRENT PATH ---"
        )

        print(
            " raw heading         : "
            + self._format_value(
                raw_heading,
                decimals=2,
                suffix=" deg",
            )
        )

        print(
            " filtered heading    : "
            + self._format_value(
                filtered_heading,
                decimals=2,
                suffix=" deg",
            )
        )

        print(
            " raw lateral offset  : "
            + self._format_value(
                raw_offset,
                decimals=4,
            )
        )

        print(
            " filtered offset     : "
            + self._format_value(
                filtered_offset,
                decimals=4,
            )
        )

        print()

        print(
            "--- TURN / CURVE ---"
        )

        print(
            " turn consistency    : "
            + self._format_value(
                turn_consistency,
                decimals=3,
            )
        )

        print(
            " angle change mean   : "
            + self._format_value(
                angle_change_mean,
                decimals=2,
                suffix=" deg",
            )
        )

        print(
            " angle change max    : "
            + self._format_value(
                angle_change_max,
                decimals=2,
                suffix=" deg",
            )
        )

        print(
            " far heading         : "
            + self._format_value(
                data.get(
                    "far_heading_deg"
                ),
                decimals=2,
                suffix=" deg",
            )
        )

        print(
            " turn angle          : "
            + self._format_value(
                data.get(
                    "turn_angle_deg"
                ),
                decimals=2,
                suffix=" deg",
            )
        )

        print()

        print(
            "--- QUALITY ---"
        )

        print(
            " heading quality     : "
            + self._format_value(
                heading_quality,
                decimals=3,
            )
            + " "
            + self._quality_bar(
                heading_quality
            )
        )

        print(
            " geometry quality    : "
            + self._format_value(
                geometry_quality,
                decimals=3,
            )
            + " "
            + self._quality_bar(
                geometry_quality
            )
        )

        print(
            " detection quality   : "
            + self._format_value(
                detection_quality,
                decimals=3,
            )
            + " "
            + self._quality_bar(
                detection_quality
            )
        )

        print()

        print(
            "--- FILTER ---"
        )

        print(
            f" ready               : "
            f"{data.get('filter_ready', False)}"
        )

        print(
            f" history size        : "
            f"{data.get('filter_history_size', 0)}"
        )

        print(
            f" missed frames       : "
            f"{data.get('missed_line_frames', 0)}"
        )

        print(
            " processing time     : "
            + self._format_value(
                data.get(
                    "processing_ms"
                ),
                decimals=3,
                suffix=" ms",
            )
        )

        if self.show_center_points:

            print()

            print(
                "--- CENTER POINTS ---"
            )

            print(
                data.get(
                    "center_points_px",
                    [],
                )
            )

        print()

        print(
            "=" * 64
        )


def main(
    args: list[str] | None = None,
) -> None:

    rclpy.init(
        args=args
    )

    node = LineDebugMonitor()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":
    main()
