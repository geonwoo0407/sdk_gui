#!/usr/bin/env python3
"""OpenCV minimap visualizer for IRC mission state."""

from __future__ import annotations

import json
import math
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MissionMapVisualizer(Node):
    """Draw a simple top-down course map from /vision/mission_state."""

    def __init__(self) -> None:
        super().__init__("mission_map_visualizer")

        self.declare_parameter("mission_state_topic", "/vision/mission_state")
        self.declare_parameter("line_info_topic", "/vision/line_info")
        self.declare_parameter("robot_pose_topic", "/vision/robot_pose")
        self.declare_parameter("window_name", "IRC MISSION MAP")
        self.declare_parameter("publish_rate_hz", 10.0)

        mission_state_topic = str(
            self.get_parameter("mission_state_topic").value
        )
        line_info_topic = str(
            self.get_parameter("line_info_topic").value
        )
        robot_pose_topic = str(
            self.get_parameter("robot_pose_topic").value
        )
        self.window_name = str(
            self.get_parameter("window_name").value
        )

        self.latest_mission_state: dict[str, Any] | None = None
        self.latest_mission_time: float | None = None
        self.latest_line_info: dict[str, Any] | None = None
        self.latest_line_time: float | None = None
        self.latest_robot_pose: dict[str, Any] | None = None
        self.latest_robot_pose_time: float | None = None

        self.mission_subscription = self.create_subscription(
            String,
            mission_state_topic,
            self._mission_state_callback,
            10,
        )

        self.line_subscription = self.create_subscription(
            String,
            line_info_topic,
            self._line_info_callback,
            10,
        )

        self.robot_pose_subscription = self.create_subscription(
            String,
            robot_pose_topic,
            self._robot_pose_callback,
            10,
        )

        publish_rate_hz = max(
            1.0,
            float(self.get_parameter("publish_rate_hz").value),
        )

        self.timer = self.create_timer(
            1.0 / publish_rate_hz,
            self._draw,
        )

        self.grid_origin = (70, 75)
        self.cell_size = 78
        self.course_cells = [
            (0, 0), (0, 1), (0, 2), (0, 3), (0, 4),
            (1, 0), (2, 0),
            (2, 1), (2, 2), (2, 3), (2, 4),
            (3, 4), (4, 4),
            (4, 3), (4, 2), (4, 1), (4, 0),
        ]

        self.path_points_grid = self._build_route_points()

        self.zone_points = {
            "START": self._grid_to_px(0.5, 4.5),
            "WALK_TO_BALL_A": self._grid_to_px(0.5, 2.7),
            "PICK_BALL_A": self._grid_to_px(0.2, 0.2),
            "SCORE_GOAL_A": self._grid_to_px(3.15, 0.4),
            "WALK_TO_BALL_B": self._grid_to_px(2.5, 2.7),
            "PICK_BALL_B": self._grid_to_px(2.2, 4.8),
            "SCORE_GOAL_B": self._grid_to_px(5.0, 4.55),
            "WALK_TO_FINISH": self._grid_to_px(4.5, 2.3),
            "FINISH": self._grid_to_px(4.5, 0.5),
            "LINE_RECOVERY": self._grid_to_px(1.0, 4.9),
            "UNKNOWN": self._grid_to_px(-0.6, 4.8),
        }

        self.checkpoints = [
            ("START", (0.5, 4.5)),
            ("PICK A", (0.2, 0.2)),
            ("GOAL A", (3.15, 0.4)),
            ("PICK B", (2.2, 4.8)),
            ("GOAL B", (5.0, 4.55)),
            ("FINISH", (4.5, 0.5)),
        ]

        self.landmarks = [
            ("BALL A", (0.20, 0.20), (0, 190, 255)),
            ("BALL B", (2.20, 4.80), (0, 190, 255)),
            ("GOAL A", (3.15, 0.40), (255, 210, 80)),
            ("GOAL B", (5.00, 4.55), (255, 210, 80)),
        ]

        self.mission_steps = [
            "START",
            "WALK_TO_BALL_A",
            "PICK_BALL_A",
            "SCORE_GOAL_A",
            "WALK_TO_BALL_B",
            "PICK_BALL_B",
            "SCORE_GOAL_B",
            "WALK_TO_FINISH",
            "FINISH",
        ]

        self.get_logger().info(
            f"Subscribing mission state: {mission_state_topic}"
        )
        self.get_logger().info(f"Subscribing line info: {line_info_topic}")
        self.get_logger().info(f"Subscribing robot pose: {robot_pose_topic}")

    @staticmethod
    def _quarter_arc_points(
        center: tuple[float, float],
        radius: float,
        start_deg: float,
        end_deg: float,
        steps: int = 8,
    ) -> list[tuple[float, float]]:
        """Build points along a quarter-circle arc in grid coordinates."""

        return [
            (
                center[0] + math.cos(math.radians(angle_deg)) * radius,
                center[1] + math.sin(math.radians(angle_deg)) * radius,
            )
            for angle_deg in np.linspace(start_deg, end_deg, steps)
        ]

    def _build_route_points(self) -> list[tuple[float, float]]:
        """Build the route as straight lines connected by quarter arcs."""

        points: list[tuple[float, float]] = []

        points.extend(
            [
                (0.5, 4.5),
                (0.5, 1.0),
            ]
        )
        points.extend(self._quarter_arc_points((1.0, 1.0), 0.5, 180.0, 270.0)[1:])
        points.extend([(2.0, 0.5)])
        points.extend(self._quarter_arc_points((2.0, 1.0), 0.5, 270.0, 360.0)[1:])
        points.extend([(2.5, 4.0)])
        points.extend(self._quarter_arc_points((3.0, 4.0), 0.5, 180.0, 90.0)[1:])
        points.extend([(4.0, 4.5)])
        points.extend(self._quarter_arc_points((4.0, 4.0), 0.5, 90.0, 0.0)[1:])
        points.extend([(4.5, 0.5)])

        return points

    def _mission_state_callback(self, message: String) -> None:
        """Store latest mission-state JSON."""

        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("mission_state JSON must be an object")
            self.latest_mission_state = payload
            self.latest_mission_time = time.monotonic()

        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                f"Invalid mission state: {type(exc).__name__}: {exc}"
            )

    def _line_info_callback(self, message: String) -> None:
        """Store latest line-info JSON for extra map annotations."""

        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("line_info JSON must be an object")
            self.latest_line_info = payload
            self.latest_line_time = time.monotonic()

        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                f"Invalid line info: {type(exc).__name__}: {exc}"
            )

    def _robot_pose_callback(self, message: String) -> None:
        """Store latest robot-pose JSON from imu_line_pose_estimator."""

        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("robot_pose JSON must be an object")
            self.latest_robot_pose = payload
            self.latest_robot_pose_time = time.monotonic()

        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                f"Invalid robot pose: {type(exc).__name__}: {exc}"
            )

    @staticmethod
    def _fresh(
        timestamp: float | None,
        timeout_sec: float = 1.0,
    ) -> bool:
        if timestamp is None:
            return False
        return (time.monotonic() - timestamp) <= timeout_sec

    @staticmethod
    def _put_text(
        image: np.ndarray,
        text: str,
        origin: tuple[int, int],
        scale: float = 0.58,
        color: tuple[int, int, int] = (235, 235, 235),
        thickness: int = 1,
    ) -> None:
        cv2.putText(
            image,
            text,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            thickness,
            cv2.LINE_AA,
        )

    def _grid_to_px(
        self,
        grid_x: float,
        grid_y: float,
    ) -> tuple[int, int]:
        """Convert 1m grid coordinates to minimap pixels."""

        origin_x, origin_y = self.grid_origin
        return (
            int(
                origin_x
                + grid_x * self.cell_size
            ),
            int(
                origin_y
                + grid_y * self.cell_size
            ),
        )

    @staticmethod
    def _draw_dashed_line(
        image: np.ndarray,
        start: tuple[int, int],
        end: tuple[int, int],
        color: tuple[int, int, int],
        thickness: int = 3,
        dash_length: int = 16,
        gap_length: int = 12,
    ) -> None:
        """Draw one dashed line segment."""

        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)

        if length <= 1e-6:
            return

        direction_x = dx / length
        direction_y = dy / length
        distance = 0.0

        while distance < length:
            dash_end = min(
                distance + dash_length,
                length,
            )

            point_a = (
                int(start[0] + direction_x * distance),
                int(start[1] + direction_y * distance),
            )
            point_b = (
                int(start[0] + direction_x * dash_end),
                int(start[1] + direction_y * dash_end),
            )

            cv2.line(
                image,
                point_a,
                point_b,
                color,
                thickness,
                cv2.LINE_AA,
            )

            distance += dash_length + gap_length

    def _draw_course(
        self,
        image: np.ndarray,
        active_zone: str,
    ) -> None:
        """Draw static map zones and route."""

        for col, row in self.course_cells:
            x0, y0 = self._grid_to_px(col, row)
            x1, y1 = self._grid_to_px(col + 1, row + 1)
            cv2.rectangle(
                image,
                (x0, y0),
                (x1, y1),
                (248, 248, 248),
                -1,
            )
            cv2.rectangle(
                image,
                (x0, y0),
                (x1, y1),
                (0, 0, 0),
                2,
            )

        path_pixels = [
            self._grid_to_px(x, y)
            for x, y in self.path_points_grid
        ]

        for start, end in zip(path_pixels[:-1], path_pixels[1:]):
            self._draw_dashed_line(
                image,
                start,
                end,
                (30, 30, 30),
                thickness=3,
                dash_length=14,
                gap_length=11,
            )

        start_left = self._grid_to_px(0.02, 4.5)
        start_right = self._grid_to_px(0.98, 4.5)
        finish_left = self._grid_to_px(4.02, 0.5)
        finish_right = self._grid_to_px(4.98, 0.5)

        cv2.line(
            image,
            start_left,
            start_right,
            (255, 0, 255),
            4,
            cv2.LINE_AA,
        )
        cv2.line(
            image,
            finish_left,
            finish_right,
            (255, 0, 255),
            4,
            cv2.LINE_AA,
        )

        self._put_text(
            image,
            "START LINE: 0.5m from edge",
            self._grid_to_px(1.08, 4.58),
            scale=0.45,
            color=(255, 0, 255),
            thickness=1,
        )
        self._put_text(
            image,
            "FINISH LINE: 0.5m from edge",
            self._grid_to_px(5.12, 0.56),
            scale=0.45,
            color=(255, 0, 255),
            thickness=1,
        )
        self._put_text(
            image,
            "1 cell = 1m x 1m",
            self._grid_to_px(0.0, -0.25),
            scale=0.55,
            color=(220, 230, 235),
            thickness=1,
        )

        for label, (grid_x, grid_y), color in self.landmarks:
            x, y = self._grid_to_px(
                grid_x,
                grid_y,
            )
            cv2.circle(
                image,
                (x, y),
                13,
                color,
                -1,
                cv2.LINE_AA,
            )
            cv2.circle(
                image,
                (x, y),
                17,
                (235, 235, 235),
                2,
                cv2.LINE_AA,
            )
            self._put_text(
                image,
                label,
                (x + 20, y + 6),
                scale=0.48,
                color=color,
                thickness=2,
            )

    def _draw_robot(
        self,
        image: np.ndarray,
        zone: str,
        heading_error_deg: float | None,
        lateral_offset_norm: float | None,
        robot_pose: dict[str, Any] | None = None,
    ) -> None:
        """Draw a robot marker from robot_pose, or fallback to active zone."""

        using_pose = robot_pose is not None

        if using_pose:
            grid_x = float(robot_pose.get("x_m", 0.5) or 0.5)
            grid_y = float(robot_pose.get("y_m", 4.5) or 4.5)
            x, y = self._grid_to_px(grid_x, grid_y)
            route_x = float(
                robot_pose.get("route_center_x_m", grid_x) or grid_x
            )
            route_y = float(
                robot_pose.get("route_center_y_m", grid_y) or grid_y
            )
            route_px = self._grid_to_px(route_x, route_y)
            heading = float(robot_pose.get("heading_deg", -90.0) or -90.0)
            confidence = float(robot_pose.get("confidence", 0.0) or 0.0)
            marker_color = (
                0,
                int(150 + 105 * np.clip(confidence, 0.0, 1.0)),
                255,
            )
            path_status = str(
                robot_pose.get("path_deviation_status", "unknown")
            )
            if path_status == "off_path_estimated":
                marker_color = (0, 80, 255)
            elif path_status == "near_edge_estimated":
                marker_color = (0, 190, 255)
        else:
            x, y = self.zone_points.get(
                zone,
                self.zone_points["UNKNOWN"],
            )
            route_px = (x, y)
            heading = -90.0 + float(heading_error_deg or 0.0)
            marker_color = (0, 240, 255)

        if using_pose:
            cv2.line(
                image,
                route_px,
                (x, y),
                (0, 165, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.circle(
                image,
                route_px,
                6,
                (255, 255, 255),
                -1,
                cv2.LINE_AA,
            )

        cv2.circle(
            image,
            (x, y),
            13,
            marker_color,
            -1,
            cv2.LINE_AA,
        )
        cv2.circle(
            image,
            (x, y),
            18,
            (250, 250, 250),
            2,
            cv2.LINE_AA,
        )

        angle_rad = math.radians(
            heading
        )
        arrow_len = 58
        end = (
            int(x + math.cos(angle_rad) * arrow_len),
            int(y + math.sin(angle_rad) * arrow_len),
        )

        cv2.arrowedLine(
            image,
            (x, y),
            end,
            (0, 255, 255),
            4,
            cv2.LINE_AA,
            tipLength=0.32,
        )

        if using_pose:
            progress_norm = float(
                robot_pose.get("route_progress_norm", 0.0) or 0.0
            )
            self._put_text(
                image,
                f"VIEW {progress_norm * 100.0:.1f}%",
                (x + 18, y - 18),
                scale=0.45,
                color=(0, 255, 255),
                thickness=1,
            )

    def _draw_side_panel(
        self,
        image: np.ndarray,
        mission_state: dict[str, Any] | None,
        line_info: dict[str, Any] | None,
        robot_pose: dict[str, Any] | None,
    ) -> None:
        """Draw current state values on the right side."""

        x = 930
        y = 68

        cv2.rectangle(
            image,
            (900, 25),
            (1270, 935),
            (34, 37, 43),
            -1,
        )
        cv2.rectangle(
            image,
            (900, 25),
            (1270, 935),
            (95, 105, 120),
            1,
        )

        self._put_text(
            image,
            "MISSION STATE",
            (x, y),
            scale=0.72,
            color=(0, 255, 255),
            thickness=2,
        )

        if mission_state is None:
            self._put_text(
                image,
                "Waiting for /vision/mission_state",
                (x, y + 45),
                color=(180, 190, 200),
            )
            return

        rows = [
            ("zone", mission_state.get("zone")),
            ("mission", mission_state.get("mission")),
            ("confidence", mission_state.get("confidence")),
            ("expected", ",".join(mission_state.get("expected_objects", []))),
            ("objects", ",".join(mission_state.get("last_seen_objects", []))),
            ("line", mission_state.get("line_detected")),
            ("line_q", mission_state.get("line_quality")),
            ("heading", mission_state.get("heading_error_deg")),
            ("offset", mission_state.get("lateral_offset_norm")),
            ("missed", mission_state.get("missed_line_frames")),
        ]

        for index, (label, value) in enumerate(rows):
            self._put_text(
                image,
                f"{label:>10}: {value}",
                (x, y + 45 + index * 31),
                scale=0.55,
            )

        note = str(
            mission_state.get("note", "")
        )
        self._put_text(
            image,
            note[:36],
            (x, 455),
            scale=0.48,
            color=(170, 210, 255),
        )

        if line_info is not None:
            processing_ms = line_info.get("processing_ms")
            self._put_text(
                image,
                f"line processing: {processing_ms} ms",
                (x, 482),
                scale=0.45,
                color=(170, 180, 190),
            )

        if robot_pose is not None:
            pose_y = 505
            self._put_text(
                image,
                "ROBOT POSE",
                (x, pose_y),
                scale=0.50,
                color=(0, 255, 255),
                thickness=1,
            )
            pose_rows = [
                ("x,y", f"{robot_pose.get('x_m')}, {robot_pose.get('y_m')} m"),
                ("lat_off", f"{robot_pose.get('lateral_offset_m')} m"),
                ("path", robot_pose.get("path_deviation_status")),
                ("heading", f"{robot_pose.get('heading_deg')} deg"),
                ("speed", f"{robot_pose.get('estimated_speed_mps')} m/s"),
                ("motion", robot_pose.get("motion_score")),
                ("progress", f"{robot_pose.get('route_progress_m')} m"),
                ("landmark", ",".join(robot_pose.get("visible_landmarks", []))),
                ("correct", robot_pose.get("correction_source")),
                ("moving", robot_pose.get("moving")),
            ]
            for index, (label, value) in enumerate(pose_rows):
                self._put_text(
                    image,
                    f"{label:>8}: {value}",
                    (x, pose_y + 24 + index * 20),
                    scale=0.39,
                    color=(200, 215, 225),
                )

        step_y = 735
        self._put_text(
            image,
            "CHECKLIST",
            (x, step_y),
            scale=0.55,
            color=(0, 255, 255),
            thickness=1,
        )

        active_zone = str(
            mission_state.get("zone", "")
        )
        active_index = (
            self.mission_steps.index(active_zone)
            if active_zone in self.mission_steps
            else -1
        )

        for index, step in enumerate(
            self.mission_steps
        ):
            if index < active_index:
                marker = "[x]"
            elif index == active_index:
                marker = "[>]"
            else:
                marker = "[ ]"
            self._put_text(
                image,
                f"{marker} {step}",
                (x, step_y + 24 + index * 18),
                scale=0.35,
                color=(
                    (80, 255, 130)
                    if step == active_zone
                    else (180, 190, 200)
                ),
            )

    def _draw(self) -> None:
        """Render one minimap frame."""

        image = np.zeros(
            (960, 1280, 3),
            dtype=np.uint8,
        )
        image[:] = (24, 27, 32)

        mission_state = (
            self.latest_mission_state
            if self._fresh(self.latest_mission_time)
            else None
        )

        line_info = (
            self.latest_line_info
            if self._fresh(self.latest_line_time)
            else None
        )

        robot_pose = (
            self.latest_robot_pose
            if self._fresh(self.latest_robot_pose_time)
            else None
        )

        active_zone = (
            str(mission_state.get("zone", "UNKNOWN"))
            if mission_state is not None
            else "UNKNOWN"
        )

        heading_error = (
            mission_state.get("heading_error_deg")
            if mission_state is not None
            else None
        )
        lateral_offset = (
            mission_state.get("lateral_offset_norm")
            if mission_state is not None
            else None
        )

        self._put_text(
            image,
            "IRC VISION MINIMAP",
            (40, 52),
            scale=0.9,
            color=(0, 255, 255),
            thickness=2,
        )
        self._draw_course(image, active_zone)
        self._draw_robot(
            image,
            active_zone,
            heading_error,
            lateral_offset,
            robot_pose,
        )
        self._draw_side_panel(
            image,
            mission_state,
            line_info,
            robot_pose,
        )

        cv2.imshow(
            self.window_name,
            image,
        )
        cv2.waitKey(1)

    def destroy_node(self) -> bool:
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    node = MissionMapVisualizer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
