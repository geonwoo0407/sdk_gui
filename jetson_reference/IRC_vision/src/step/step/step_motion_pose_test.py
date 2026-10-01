#!/usr/bin/env python3
"""Test pose estimator that fakes motion-step feedback from line vision."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class RobotPose:
    """Robot pose message compatible with mission_map_visualizer."""

    x_m: float
    y_m: float
    route_center_x_m: float
    route_center_y_m: float
    heading_deg: float
    route_progress_m: float
    route_progress_norm: float
    lateral_offset_m: float
    path_deviation_status: str
    line_detected: bool
    line_quality: float
    imu_yaw_delta_deg: float
    gyro_z_rad_s: float
    accel_motion_mps2: float
    estimated_speed_mps: float
    motion_score: float
    moving: bool
    visible_landmarks: list[str]
    correction_source: str
    confidence: float
    source: str
    note: str
    fake_step_count: int
    fake_step_length_m: float
    command: str


class StepMotionPoseTest(Node):
    """Pretend an algorithm/motion stack sends completed walking steps.

    This node is intentionally a simulator:
    - it reads /vision/line_info
    - if the line is reliable, it pretends the algorithm commands forward walking
    - every fake_step_period_sec it adds one completed step
    - one step moves route progress by fake_step_length_m

    Later, fake_step_count can be replaced by a real motion feedback topic.
    """

    def __init__(self) -> None:
        super().__init__("step_motion_pose_test")

        self.declare_parameter("line_info_topic", "/vision/line_info")
        self.declare_parameter("robot_pose_topic", "/vision/robot_pose")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("line_timeout_sec", 0.7)
        self.declare_parameter("line_quality_threshold", 0.45)
        self.declare_parameter("fake_step_length_m", 0.15)
        self.declare_parameter("fake_step_period_sec", 0.8)
        self.declare_parameter("auto_step_enabled", True)
        self.declare_parameter("max_heading_for_forward_deg", 55.0)
        self.declare_parameter("lateral_offset_sign", -1.0)
        self.declare_parameter("map_lateral_offset_scale_m", 0.45)
        self.declare_parameter("path_deviation_warn_m", 0.25)
        self.declare_parameter("line_heading_sign", -1.0)

        self.line_info_topic = str(self.get_parameter("line_info_topic").value)
        self.robot_pose_topic = str(self.get_parameter("robot_pose_topic").value)
        self.line_timeout_sec = float(
            self.get_parameter("line_timeout_sec").value
        )
        self.line_quality_threshold = float(
            self.get_parameter("line_quality_threshold").value
        )
        self.fake_step_length_m = float(
            self.get_parameter("fake_step_length_m").value
        )
        self.fake_step_period_sec = float(
            self.get_parameter("fake_step_period_sec").value
        )
        self.auto_step_enabled = bool(
            self.get_parameter("auto_step_enabled").value
        )
        self.max_heading_for_forward_deg = float(
            self.get_parameter("max_heading_for_forward_deg").value
        )
        self.lateral_offset_sign = float(
            self.get_parameter("lateral_offset_sign").value
        )
        self.map_lateral_offset_scale_m = float(
            self.get_parameter("map_lateral_offset_scale_m").value
        )
        self.path_deviation_warn_m = float(
            self.get_parameter("path_deviation_warn_m").value
        )
        self.line_heading_sign = float(
            self.get_parameter("line_heading_sign").value
        )

        self.publisher = self.create_publisher(String, self.robot_pose_topic, 10)
        self.line_subscription = self.create_subscription(
            String,
            self.line_info_topic,
            self._line_info_callback,
            10,
        )

        publish_rate_hz = max(
            1.0,
            float(self.get_parameter("publish_rate_hz").value),
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_pose)

        self.route_points = self._build_route_points()
        self.segment_lengths = self._calculate_segment_lengths(self.route_points)
        self.total_route_length_m = sum(self.segment_lengths)

        self.latest_line_info: dict[str, Any] | None = None
        self.latest_line_time: float | None = None
        self.route_progress_m = 0.0
        self.fake_step_count = 0
        self.last_fake_step_time = time.monotonic()
        self.latest_publish_time = time.monotonic()

        self.get_logger().info(f"Subscribing line info: {self.line_info_topic}")
        self.get_logger().info(f"Publishing fake robot pose: {self.robot_pose_topic}")
        self.get_logger().info(
            f"Fake step length: {self.fake_step_length_m:.3f} m"
        )

    @staticmethod
    def _wrap_deg(angle_deg: float) -> float:
        while angle_deg > 180.0:
            angle_deg -= 360.0
        while angle_deg < -180.0:
            angle_deg += 360.0
        return angle_deg

    @staticmethod
    def _quarter_arc_points(
        center: tuple[float, float],
        radius: float,
        start_deg: float,
        end_deg: float,
        steps: int = 8,
    ) -> list[tuple[float, float]]:
        step_count = max(2, steps)
        return [
            (
                center[0] + math.cos(math.radians(angle_deg)) * radius,
                center[1] + math.sin(math.radians(angle_deg)) * radius,
            )
            for angle_deg in [
                start_deg
                + (end_deg - start_deg) * index / (step_count - 1)
                for index in range(step_count)
            ]
        ]

    def _build_route_points(self) -> list[tuple[float, float]]:
        points: list[tuple[float, float]] = []

        points.extend([(0.5, 4.5), (0.5, 1.0)])
        points.extend(self._quarter_arc_points((1.0, 1.0), 0.5, 180.0, 270.0)[1:])
        points.extend([(2.0, 0.5)])
        points.extend(self._quarter_arc_points((2.0, 1.0), 0.5, 270.0, 360.0)[1:])
        points.extend([(2.5, 4.0)])
        points.extend(self._quarter_arc_points((3.0, 4.0), 0.5, 180.0, 90.0)[1:])
        points.extend([(4.0, 4.5)])
        points.extend(self._quarter_arc_points((4.0, 4.0), 0.5, 90.0, 0.0)[1:])
        points.extend([(4.5, 0.5)])

        return points

    @staticmethod
    def _calculate_segment_lengths(
        points: list[tuple[float, float]],
    ) -> list[float]:
        return [
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(points[:-1], points[1:])
        ]

    def _line_info_callback(self, message: String) -> None:
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

    def _line_is_fresh(self) -> bool:
        if self.latest_line_time is None:
            return False
        return (time.monotonic() - self.latest_line_time) <= self.line_timeout_sec

    def _line_value(self, key: str, default: Any = None) -> Any:
        if not self._line_is_fresh() or self.latest_line_info is None:
            return default
        return self.latest_line_info.get(key, default)

    def _line_quality(self) -> float:
        heading_quality = float(self._line_value("heading_quality", 0.0) or 0.0)
        geometry_quality = float(self._line_value("geometry_quality", 0.0) or 0.0)
        detection_quality = float(self._line_value("detection_quality", 0.0) or 0.0)
        return min(heading_quality, geometry_quality, detection_quality)

    def _lateral_offset_m(self) -> float:
        offset_norm = float(
            self._line_value("filtered_lateral_offset_norm", 0.0) or 0.0
        )
        offset_norm = max(-1.0, min(1.0, offset_norm))
        return (
            offset_norm
            * self.lateral_offset_sign
            * self.map_lateral_offset_scale_m
        )

    def _path_deviation_status(self, lateral_offset_m: float) -> str:
        if abs(lateral_offset_m) >= self.path_deviation_warn_m:
            return "off_path_estimated"
        if abs(lateral_offset_m) >= self.path_deviation_warn_m * 0.55:
            return "near_edge_estimated"
        return "on_path_estimated"

    def _route_pose_at(self, progress_m: float) -> tuple[float, float, float]:
        progress = max(0.0, min(progress_m, self.total_route_length_m))
        remaining = progress

        for index, segment_length in enumerate(self.segment_lengths):
            start = self.route_points[index]
            end = self.route_points[index + 1]

            if remaining <= segment_length or index == len(self.segment_lengths) - 1:
                ratio = remaining / max(segment_length, 1e-6)
                x_m = start[0] + (end[0] - start[0]) * ratio
                y_m = start[1] + (end[1] - start[1]) * ratio
                heading_deg = math.degrees(
                    math.atan2(end[1] - start[1], end[0] - start[0])
                )
                return x_m, y_m, heading_deg

            remaining -= segment_length

        x_m, y_m = self.route_points[-1]
        prev_x, prev_y = self.route_points[-2]
        heading_deg = math.degrees(math.atan2(y_m - prev_y, x_m - prev_x))
        return x_m, y_m, heading_deg

    def _maybe_add_fake_step(
        self,
        line_detected: bool,
        line_quality: float,
        line_heading_error_deg: float,
    ) -> tuple[bool, str]:
        if not self.auto_step_enabled:
            return False, "fake_step_disabled"

        line_ready = (
            line_detected
            and line_quality >= self.line_quality_threshold
            and abs(line_heading_error_deg) <= self.max_heading_for_forward_deg
        )
        if not line_ready:
            return False, "waiting_for_reliable_line"

        now = time.monotonic()
        if (now - self.last_fake_step_time) < self.fake_step_period_sec:
            return False, "waiting_next_fake_step"

        self.last_fake_step_time = now
        self.fake_step_count += 1
        self.route_progress_m = min(
            self.route_progress_m + self.fake_step_length_m,
            self.total_route_length_m,
        )
        return True, "fake_forward_step_completed"

    def _publish_pose(self) -> None:
        now = time.monotonic()
        self.latest_publish_time = now

        line_detected = bool(self._line_value("detected", False))
        line_quality = self._line_quality()
        line_heading_error_deg = float(
            self._line_value("filtered_heading_error_deg", 0.0) or 0.0
        )

        stepped, step_note = self._maybe_add_fake_step(
            line_detected,
            line_quality,
            line_heading_error_deg,
        )

        route_center_x_m, route_center_y_m, route_heading_deg = self._route_pose_at(
            self.route_progress_m
        )
        lateral_offset_m = self._lateral_offset_m()
        lateral_angle_rad = math.radians(route_heading_deg + 90.0)
        x_m = route_center_x_m + math.cos(lateral_angle_rad) * lateral_offset_m
        y_m = route_center_y_m + math.sin(lateral_angle_rad) * lateral_offset_m

        heading_deg = self._wrap_deg(
            route_heading_deg
            + self.line_heading_sign * line_heading_error_deg
        )

        estimated_speed_mps = (
            self.fake_step_length_m / max(self.fake_step_period_sec, 1e-6)
            if stepped
            else 0.0
        )

        command = "walk_forward" if stepped else "hold_or_align"
        pose = RobotPose(
            x_m=round(x_m, 3),
            y_m=round(y_m, 3),
            route_center_x_m=round(route_center_x_m, 3),
            route_center_y_m=round(route_center_y_m, 3),
            heading_deg=round(heading_deg, 3),
            route_progress_m=round(self.route_progress_m, 3),
            route_progress_norm=round(
                self.route_progress_m / max(self.total_route_length_m, 1e-6),
                4,
            ),
            lateral_offset_m=round(lateral_offset_m, 3),
            path_deviation_status=self._path_deviation_status(lateral_offset_m),
            line_detected=line_detected,
            line_quality=round(line_quality, 3),
            imu_yaw_delta_deg=0.0,
            gyro_z_rad_s=0.0,
            accel_motion_mps2=0.0,
            estimated_speed_mps=round(estimated_speed_mps, 3),
            motion_score=1.0 if stepped else 0.0,
            moving=stepped,
            visible_landmarks=[],
            correction_source="fake_step_feedback",
            confidence=round(max(0.25, min(line_quality, 0.95)), 3),
            source="fake_algorithm_motion_steps",
            note=step_note,
            fake_step_count=self.fake_step_count,
            fake_step_length_m=round(self.fake_step_length_m, 3),
            command=command,
        )

        message = String()
        message.data = json.dumps(
            asdict(pose),
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self.publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    node = StepMotionPoseTest()

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
