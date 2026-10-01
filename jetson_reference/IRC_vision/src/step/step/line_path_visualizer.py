#!/usr/bin/env python3
"""
ROS 2 debug visualizer for YOLO line-path analysis.

Inputs
------
/camera/color/image_raw
    sensor_msgs/msg/Image

/vision/line_info
    std_msgs/msg/String
    JSON output from yolo_line_analyzer

/navigation/line_command
    std_msgs/msg/String
    JSON output from line_navigation_controller

Outputs
-------
Optional annotated image:
    /vision/line_path/image

Display
-------
OpenCV window showing:

- Camera center line
- YOLO line center points
- Near-to-far point ordering
- Connected path polyline
- Immediate heading arrow
- Lateral offset
- Final STRAIGHT / numbered RECOVER_LEFT_n / RECOVER_RIGHT_n decision
- Target linear/angular speed, travel distance, and rotation
- Compact steering calculation inputs

This node does not make navigation decisions. It displays the decision made by
line_navigation_controller together with the detected path.

"""

from __future__ import annotations

import json
import math
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class LinePathVisualizer(Node):
    """Visualize line geometry from /vision/line_info."""

    def __init__(self) -> None:
        super().__init__("line_path_visualizer")

        # ====================================================
        # ROS topics
        # ====================================================

        self.declare_parameter(
            "image_topic",
            "/camera/color/image_raw",
        )

        self.declare_parameter(
            "line_info_topic",
            "/vision/line_info",
        )

        self.declare_parameter(
            "navigation_command_topic",
            "/navigation/line_command",
        )

        self.declare_parameter(
            "annotated_image_topic",
            "/vision/line_path/image",
        )

        # ====================================================
        # Display / publishing
        # ====================================================

        self.declare_parameter(
            "display",
            True,
        )

        self.declare_parameter(
            "publish_annotated_image",
            False,
        )

        self.declare_parameter(
            "window_name",
            "LINE NAVIGATION",
        )

        # ====================================================
        # Data freshness
        # ====================================================

        # Ignore line_info older than this many seconds.
        self.declare_parameter(
            "max_line_info_age_sec",
            0.5,
        )

        self.declare_parameter(
            "max_navigation_command_age_sec",
            0.5,
        )

        # ====================================================
        # Drawing
        # ====================================================

        self.declare_parameter(
            "point_radius",
            7,
        )

        self.declare_parameter(
            "line_thickness",
            4,
        )

        self.declare_parameter(
            "heading_arrow_length_px",
            180,
        )

        self.declare_parameter(
            "show_point_numbers",
            False,
        )

        self.declare_parameter(
            "show_all_metrics",
            False,
        )

        self.declare_parameter(
            "show_geometry_labels",
            False,
        )

        # ====================================================
        # Read parameters
        # ====================================================

        self.display = bool(
            self.get_parameter("display").value
        )

        self.publish_annotated_image = bool(
            self.get_parameter(
                "publish_annotated_image"
            ).value
        )

        self.window_name = str(
            self.get_parameter("window_name").value
        )

        self.max_line_info_age_sec = float(
            self.get_parameter(
                "max_line_info_age_sec"
            ).value
        )

        self.max_navigation_command_age_sec = float(
            self.get_parameter(
                "max_navigation_command_age_sec"
            ).value
        )

        self.point_radius = int(
            self.get_parameter("point_radius").value
        )

        self.line_thickness = int(
            self.get_parameter("line_thickness").value
        )

        self.heading_arrow_length_px = int(
            self.get_parameter(
                "heading_arrow_length_px"
            ).value
        )

        self.show_point_numbers = bool(
            self.get_parameter(
                "show_point_numbers"
            ).value
        )

        self.show_all_metrics = bool(
            self.get_parameter(
                "show_all_metrics"
            ).value
        )

        self.show_geometry_labels = bool(
            self.get_parameter(
                "show_geometry_labels"
            ).value
        )

        image_topic = str(
            self.get_parameter("image_topic").value
        )

        line_info_topic = str(
            self.get_parameter("line_info_topic").value
        )

        navigation_command_topic = str(
            self.get_parameter(
                "navigation_command_topic"
            ).value
        )

        annotated_image_topic = str(
            self.get_parameter(
                "annotated_image_topic"
            ).value
        )

        # ====================================================
        # State
        # ====================================================

        self.bridge = CvBridge()

        self.latest_line_info: dict[str, Any] | None = None

        self.latest_line_info_time: float | None = None

        self.latest_navigation_command: dict[str, Any] | None = None

        self.latest_navigation_command_time: float | None = None

        # ====================================================
        # ROS publisher / subscribers
        # ====================================================

        self.line_info_subscription = self.create_subscription(
            String,
            line_info_topic,
            self._line_info_callback,
            10,
        )

        self.navigation_command_subscription = self.create_subscription(
            String,
            navigation_command_topic,
            self._navigation_command_callback,
            10,
        )

        self.image_subscription = self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )

        self.annotated_image_publisher = None

        if self.publish_annotated_image:
            self.annotated_image_publisher = self.create_publisher(
                Image,
                annotated_image_topic,
                10,
            )

        # ====================================================
        # Startup log
        # ====================================================

        self.get_logger().info(
            f"Image topic: {image_topic}"
        )

        self.get_logger().info(
            f"Line info topic: {line_info_topic}"
        )

        self.get_logger().info(
            "Navigation command topic: "
            f"{navigation_command_topic}"
        )

        self.get_logger().info(
            f"Display: {self.display}"
        )

        self.get_logger().info(
            "Publish annotated image: "
            f"{self.publish_annotated_image}"
        )

    # ========================================================
    # ROS callbacks
    # ========================================================

    def _line_info_callback(
        self,
        message: String,
    ) -> None:
        """Store latest line-analysis JSON."""
        try:
            payload = json.loads(message.data)

            if not isinstance(payload, dict):
                raise ValueError(
                    "line_info JSON must be an object"
                )

            self.latest_line_info = payload
            self.latest_line_info_time = time.monotonic()

        except (
            json.JSONDecodeError,
            TypeError,
            ValueError,
        ) as exc:
            self.get_logger().warning(
                "Invalid line-info message: "
                f"{type(exc).__name__}: {exc}"
            )

    def _navigation_command_callback(
        self,
        message: String,
    ) -> None:
        """Store latest navigation-command JSON."""
        try:
            payload = json.loads(message.data)

            if not isinstance(payload, dict):
                raise ValueError(
                    "navigation command JSON must be an object"
                )

            self.latest_navigation_command = payload
            self.latest_navigation_command_time = time.monotonic()

        except (
            json.JSONDecodeError,
            TypeError,
            ValueError,
        ) as exc:
            self.get_logger().warning(
                "Invalid navigation-command message: "
                f"{type(exc).__name__}: {exc}"
            )

    def _image_callback(
        self,
        message: Image,
    ) -> None:
        """Draw latest line data over current camera image."""
        try:
            frame = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )

        except Exception as exc:
            self.get_logger().error(
                "cv_bridge conversion failed: "
                f"{type(exc).__name__}: {exc}"
            )
            return

        annotated = frame.copy()

        self._draw_camera_center(
            annotated
        )

        line_info = self._get_fresh_line_info()

        navigation_command = (
            self._get_fresh_navigation_command()
        )

        if line_info is None:
            self._draw_no_data_message(
                annotated,
                "WAITING FOR LINE INFO",
            )

        elif not bool(
            line_info.get(
                "detected",
                False,
            )
        ):
            self._draw_no_data_message(
                annotated,
                "LINE NOT DETECTED",
            )

            self._draw_status_panel(
                annotated,
                line_info,
                navigation_command,
            )

        else:
            self._draw_detected_line(
                annotated,
                line_info,
                navigation_command,
            )

        if navigation_command is not None:
            self._draw_navigation_arrow(
                annotated,
                navigation_command,
            )

        if (
            self.publish_annotated_image
            and self.annotated_image_publisher is not None
        ):
            try:
                output_message = (
                    self.bridge.cv2_to_imgmsg(
                        annotated,
                        encoding="bgr8",
                    )
                )

                output_message.header = message.header

                self.annotated_image_publisher.publish(
                    output_message
                )

            except Exception as exc:
                self.get_logger().warning(
                    "Annotated image publish failed: "
                    f"{type(exc).__name__}: {exc}"
                )

        if self.display:
            try:
                cv2.imshow(
                    self.window_name,
                    annotated,
                )

                cv2.waitKey(1)

            except cv2.error as exc:
                self.get_logger().error(
                    f"OpenCV display failed: {exc}"
                )

    # ========================================================
    # Freshness
    # ========================================================

    def _get_fresh_line_info(
        self,
    ) -> dict[str, Any] | None:
        """Return latest line info only when sufficiently fresh."""
        if (
            self.latest_line_info is None
            or self.latest_line_info_time is None
        ):
            return None

        age = (
            time.monotonic()
            - self.latest_line_info_time
        )

        if age > self.max_line_info_age_sec:
            return None

        return self.latest_line_info

    def _get_fresh_navigation_command(
        self,
    ) -> dict[str, Any] | None:
        """Return latest navigation command only while it is fresh."""
        if (
            self.latest_navigation_command is None
            or self.latest_navigation_command_time is None
        ):
            return None

        age = (
            time.monotonic()
            - self.latest_navigation_command_time
        )

        if age > self.max_navigation_command_age_sec:
            return None

        return self.latest_navigation_command

    # ========================================================
    # Main drawing
    # ========================================================

    def _draw_detected_line(
        self,
        frame: np.ndarray,
        data: dict[str, Any],
        navigation_command: dict[str, Any] | None,
    ) -> None:
        """Draw all valid detected-line information."""
        height, width = frame.shape[:2]

        points = self._parse_center_points(
            data.get(
                "center_points_px",
                [],
            )
        )

        if points:
            self._draw_path_polyline(
                frame,
                points,
            )

            self._draw_center_points(
                frame,
                points,
            )

        self._draw_lateral_offset(
            frame,
            data,
        )

        if self.show_all_metrics:
            self._draw_heading_arrow(
                frame,
                data,
                points,
            )

        self._draw_status_panel(
            frame,
            data,
            navigation_command,
        )

        # Small image dimensions reference.
        cv2.putText(
            frame,
            f"{width}x{height}",
            (
                width - 130,
                height - 15,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
            cv2.LINE_AA,
        )

    # ========================================================
    # Camera center
    # ========================================================

    def _draw_camera_center(
        self,
        frame: np.ndarray,
    ) -> None:
        """Draw camera/image center reference."""
        height, width = frame.shape[:2]

        center_x = width // 2

        cv2.line(
            frame,
            (center_x, 0),
            (center_x, height),
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )

        if self.show_geometry_labels:
            cv2.putText(
                frame,
                "CAMERA CENTER",
                (
                    center_x + 10,
                    25,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 0),
                2,
                cv2.LINE_AA,
            )

    # ========================================================
    # Point parsing
    # ========================================================

    @staticmethod
    def _parse_center_points(
        raw_points: Any,
    ) -> list[tuple[int, int]]:
        """Safely parse JSON center-point list."""
        if not isinstance(
            raw_points,
            list,
        ):
            return []

        points: list[tuple[int, int]] = []

        for raw_point in raw_points:
            if (
                not isinstance(raw_point, list)
                or len(raw_point) != 2
            ):
                continue

            try:
                x = int(
                    round(
                        float(raw_point[0])
                    )
                )

                y = int(
                    round(
                        float(raw_point[1])
                    )
                )

            except (
                TypeError,
                ValueError,
            ):
                continue

            points.append(
                (x, y)
            )

        return points

    # ========================================================
    # Polyline
    # ========================================================

    def _draw_path_polyline(
        self,
        frame: np.ndarray,
        points: list[tuple[int, int]],
    ) -> None:
        """
        Connect center points from near to far.

        center_points_px from yolo_line_analyzer are already
        sorted:

            nearest -> farthest
        """
        if len(points) < 2:
            return

        polyline = np.asarray(
            points,
            dtype=np.int32,
        ).reshape(
            (-1, 1, 2)
        )

        # Thick dark outline first.
        cv2.polylines(
            frame,
            [polyline],
            False,
            (0, 0, 0),
            self.line_thickness + 4,
            cv2.LINE_AA,
        )

        # Main path line.
        cv2.polylines(
            frame,
            [polyline],
            False,
            (0, 255, 0),
            self.line_thickness,
            cv2.LINE_AA,
        )

    # ========================================================
    # Points
    # ========================================================

    def _draw_center_points(
        self,
        frame: np.ndarray,
        points: list[tuple[int, int]],
    ) -> None:
        """Draw each path point and its near-to-far index."""
        for index, point in enumerate(points):
            x, y = point

            # Near point gets a different marker.
            if index == 0:
                point_color = (0, 0, 255)
                radius = self.point_radius + 3

            else:
                point_color = (0, 255, 255)
                radius = self.point_radius

            cv2.circle(
                frame,
                (x, y),
                radius + 2,
                (0, 0, 0),
                -1,
                cv2.LINE_AA,
            )

            cv2.circle(
                frame,
                (x, y),
                radius,
                point_color,
                -1,
                cv2.LINE_AA,
            )

            if self.show_point_numbers:
                cv2.putText(
                    frame,
                    str(index),
                    (
                        x + 12,
                        y - 8,
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

        if self.show_geometry_labels:
            # Label first and last point explicitly in detailed mode.
            near_x, near_y = points[0]

            cv2.putText(
                frame,
                "NEAR",
                (
                    near_x + 15,
                    near_y + 25,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

            far_x, far_y = points[-1]

            cv2.putText(
                frame,
                "FAR",
                (
                    far_x + 15,
                    far_y - 15,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )

    # ========================================================
    # Heading arrow
    # ========================================================

    def _draw_heading_arrow(
        self,
        frame: np.ndarray,
        data: dict[str, Any],
        points: list[tuple[int, int]],
    ) -> None:
        """Draw filtered/current heading from nearest line point."""
        heading = data.get(
            "filtered_heading_error_deg"
        )

        if heading is None:
            heading = data.get(
                "heading_error_deg"
            )

        if heading is None:
            return

        try:
            heading_deg = float(
                heading
            )

        except (
            TypeError,
            ValueError,
        ):
            return

        height, width = frame.shape[:2]

        if points:
            start_x, start_y = points[0]

        else:
            start_x = width // 2
            start_y = int(
                height * 0.82
            )

        angle_rad = math.radians(
            heading_deg
        )

        # Image coordinate convention:
        #
        # heading 0:
        #   arrow goes upward.
        #
        # positive:
        #   arrow turns toward image right.
        end_x = int(
            round(
                start_x
                + self.heading_arrow_length_px
                * math.sin(angle_rad)
            )
        )

        end_y = int(
            round(
                start_y
                - self.heading_arrow_length_px
                * math.cos(angle_rad)
            )
        )

        cv2.arrowedLine(
            frame,
            (start_x, start_y),
            (end_x, end_y),
            (255, 0, 255),
            5,
            cv2.LINE_AA,
            tipLength=0.15,
        )

        cv2.putText(
            frame,
            f"HEADING {heading_deg:+.1f} deg",
            (
                end_x + 10,
                end_y,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 0, 255),
            2,
            cv2.LINE_AA,
        )

    # ========================================================
    # Lateral offset
    # ========================================================

    def _draw_lateral_offset(
        self,
        frame: np.ndarray,
        data: dict[str, Any],
    ) -> None:
        """Draw camera center to predicted line offset."""
        offset_px = data.get(
            "lateral_offset_px"
        )

        if offset_px is None:
            return

        try:
            offset_value = float(
                offset_px
            )

        except (
            TypeError,
            ValueError,
        ):
            return

        height, width = frame.shape[:2]

        center_x = width // 2

        eval_y = int(
            height * 0.82
        )

        line_x = int(
            round(
                center_x
                + offset_value
            )
        )

        line_x = max(
            0,
            min(
                width - 1,
                line_x,
            ),
        )

        cv2.line(
            frame,
            (center_x, eval_y),
            (line_x, eval_y),
            (0, 165, 255),
            5,
            cv2.LINE_AA,
        )

        cv2.circle(
            frame,
            (line_x, eval_y),
            9,
            (0, 165, 255),
            -1,
            cv2.LINE_AA,
        )

        if self.show_geometry_labels:
            cv2.putText(
                frame,
                f"OFFSET {offset_value:+.1f}px",
                (
                    min(
                        center_x,
                        line_x,
                    ),
                    eval_y - 15,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 165, 255),
                2,
                cv2.LINE_AA,
            )

    # ========================================================
    # Final navigation command
    # ========================================================

    @staticmethod
    def _motion_color(motion: str) -> tuple[int, int, int]:
        """Return a stable BGR color for each final motion."""
        if motion.startswith("RECOVER_LEFT_"):
            return (255, 200, 80)
        if motion.startswith("RECOVER_RIGHT_"):
            return (80, 200, 255)
        return {
            "STRAIGHT": (0, 220, 0),
            "LEFT": (255, 160, 0),
            "RIGHT": (0, 165, 255),
            "STOP": (0, 0, 255),
        }.get(motion, (180, 180, 180))

    def _draw_navigation_arrow(
        self,
        frame: np.ndarray,
        command: dict[str, Any],
    ) -> None:
        """Draw the final steering target from the robot/camera center."""
        motion = str(command.get("motion", "STOP")).upper()
        color = self._motion_color(motion)
        height, width = frame.shape[:2]
        start = (width // 2, int(height * 0.93))

        if motion == "STOP" or not bool(command.get("valid", False)):
            radius = 32
            cv2.circle(frame, start, radius, color, 5, cv2.LINE_AA)
            cv2.line(
                frame,
                (start[0] - 20, start[1] - 20),
                (start[0] + 20, start[1] + 20),
                color,
                5,
                cv2.LINE_AA,
            )
            cv2.line(
                frame,
                (start[0] + 20, start[1] - 20),
                (start[0] - 20, start[1] + 20),
                color,
                5,
                cv2.LINE_AA,
            )
            return

        try:
            steering_deg = float(command.get("steering_error_deg", 0.0))
        except (TypeError, ValueError):
            steering_deg = 0.0

        steering_deg = max(-50.0, min(50.0, steering_deg))
        angle_rad = math.radians(steering_deg)
        arrow_length = int(min(width, height) * 0.24)
        end = (
            int(round(start[0] + arrow_length * math.sin(angle_rad))),
            int(round(start[1] - arrow_length * math.cos(angle_rad))),
        )

        cv2.arrowedLine(
            frame,
            start,
            end,
            (0, 0, 0),
            12,
            cv2.LINE_AA,
            tipLength=0.20,
        )
        cv2.arrowedLine(
            frame,
            start,
            end,
            color,
            7,
            cv2.LINE_AA,
            tipLength=0.20,
        )
        cv2.putText(
            frame,
            motion,
            (end[0] + 12, end[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            color,
            3,
            cv2.LINE_AA,
        )

    def _draw_status_panel(
        self,
        frame: np.ndarray,
        data: dict[str, Any],
        command: dict[str, Any] | None,
    ) -> None:
        """Draw only final command and the inputs that produced it."""
        if command is None:
            lines = [
                "NAVIGATION : WAITING",
                "Start line_navigation_controller",
                f"LINE POINTS: {data.get('line_count', 0)}",
            ]
            self._draw_text_panel(frame, lines, (180, 180, 180))
            return

        motion = str(command.get("motion", "STOP")).upper()
        speed = self._format_number(command.get("linear_speed_mps"), 3)
        lateral_speed = self._format_number(
            command.get("lateral_speed_mps"),
            3,
        )
        turn_rate = self._format_number(
            command.get("angular_speed_rad_s"),
            3,
        )
        distance = self._format_number(command.get("travel_distance_m"), 3)
        lateral_distance = self._format_number(
            command.get("lateral_travel_distance_m"),
            3,
        )
        rotation = self._format_number(
            command.get("target_heading_change_deg"),
            1,
        )
        duration = self._format_number(
            command.get("command_duration_sec"),
            2,
        )
        steering = self._format_number(command.get("steering_error_deg"), 1)
        heading_part = self._format_number(
            command.get("heading_component_deg"),
            1,
        )
        offset_part = self._format_number(
            command.get("offset_component_deg"),
            1,
        )
        preview_part = self._format_number(
            command.get("preview_component_deg"),
            1,
        )
        quality = self._format_number(command.get("line_quality"), 2)

        lines = [
            f"NAVIGATION : {motion}",
            f"SPEED {speed} m/s  |  TURN {turn_rate} rad/s",
            f"NEXT {distance} m  |  ROTATE {rotation} deg  |  "
            f"{duration} s",
            f"STEERING : {steering} deg",
            f"= HEADING {heading_part} + OFFSET {offset_part} "
            f"+ CURVE {preview_part}",
            "QUALITY "
            f"{quality}  |  LINE POINTS {data.get('line_count', 0)}",
        ]

        if self.show_all_metrics:
            lines.extend(
                [
                    "RAW H/OFF: "
                    + self._format_number(data.get("heading_error_deg"), 1)
                    + " deg / "
                    + self._format_number(data.get("lateral_offset_norm"), 3),
                    "FAR/TURN: "
                    + self._format_number(data.get("far_heading_deg"), 1)
                    + " / "
                    + self._format_number(data.get("turn_angle_deg"), 1)
                    + " deg",
                    "REASON: " + str(command.get("reason", "unknown")),
                ]
            )

        self._draw_text_panel(
            frame,
            lines,
            self._motion_color(motion),
        )

    # ========================================================
    # Generic text helpers
    # ========================================================

    @staticmethod
    def _draw_text_panel(
        frame: np.ndarray,
        lines: list[str],
        title_color: tuple[int, int, int],
    ) -> None:
        """Draw readable semi-transparent status panel."""
        if not lines:
            return

        x = 24
        y = 48

        line_height = 32

        panel_width = min(
            590,
            frame.shape[1] - 20,
        )

        panel_height = (
            len(lines) * line_height
            + 25
        )

        overlay = frame.copy()

        cv2.rectangle(
            overlay,
            (10, 10),
            (
                10 + panel_width,
                10 + panel_height,
            ),
            (0, 0, 0),
            -1,
        )

        cv2.addWeighted(
            overlay,
            0.65,
            frame,
            0.35,
            0,
            frame,
        )

        for index, text in enumerate(lines):
            text_y = (
                y
                + index * line_height
            )

            cv2.putText(
                frame,
                text,
                (x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.78 if index == 0 else 0.56,
                title_color if index == 0 else (255, 255, 255),
                3 if index == 0 else 2,
                cv2.LINE_AA,
            )

    @staticmethod
    def _draw_no_data_message(
        frame: np.ndarray,
        message: str,
    ) -> None:
        """Draw centered warning message."""
        height, width = frame.shape[:2]

        cv2.putText(
            frame,
            message,
            (
                max(
                    20,
                    width // 2 - 180,
                ),
                height // 2,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            3,
            cv2.LINE_AA,
        )

    @staticmethod
    def _format_number(
        value: Any,
        decimals: int,
    ) -> str:
        """Safely format optional numeric value."""
        if value is None:
            return "null"

        try:
            return (
                f"{float(value):.{decimals}f}"
            )

        except (
            TypeError,
            ValueError,
        ):
            return str(value)

    # ========================================================
    # Shutdown
    # ========================================================

    def destroy_node(self) -> bool:
        """Close OpenCV windows before node shutdown."""
        if self.display:
            try:
                cv2.destroyAllWindows()

            except cv2.error:
                pass

        return super().destroy_node()


def main(
    args: list[str] | None = None,
) -> None:
    rclpy.init(
        args=args
    )

    node = LinePathVisualizer()

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
