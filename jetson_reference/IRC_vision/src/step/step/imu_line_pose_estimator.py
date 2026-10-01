#!/usr/bin/env python3
"""Estimate a lightweight robot pose from RealSense IMU and line vision."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String


@dataclass
class RobotPose:
    """Simple map pose for the IRC minimap and future algorithm nodes."""

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


class ImuLinePoseEstimator(Node):
    """Fuse D435i gyro yaw and line quality into a coarse route pose.

    This is not full SLAM. It is a practical competition-map estimator:
    - gyro.z integrates short-term heading changes
    - line_info decides whether the route is visible enough
    - accel/gyro motion gate decides whether the robot is actually moving
    - a nominal walking speed moves the marker along the known course path
    """

    def __init__(self) -> None:
        super().__init__("imu_line_pose_estimator")

        self.declare_parameter("imu_topic", "/camera/camera/gyro/sample")
        self.declare_parameter("accel_topic", "/camera/camera/accel/sample")
        self.declare_parameter("line_info_topic", "/vision/line_info")
        self.declare_parameter("detections_topic", "/vision/detections")
        self.declare_parameter("robot_pose_topic", "/vision/robot_pose")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("line_timeout_sec", 0.7)
        self.declare_parameter("line_quality_threshold", 0.45)
        self.declare_parameter("enable_route_progress", True)
        self.declare_parameter("nominal_forward_speed_mps", 0.035)
        self.declare_parameter("auto_progress_when_line_visible", False)
        self.declare_parameter("use_accel_motion_gate", True)
        self.declare_parameter("accel_motion_threshold_mps2", 0.12)
        self.declare_parameter("gyro_motion_threshold_rad_s", 0.03)
        self.declare_parameter("motion_hold_sec", 0.45)
        self.declare_parameter("motion_start_sec", 0.35)
        self.declare_parameter("motion_score_start_threshold", 0.55)
        self.declare_parameter("motion_score_stop_threshold", 0.25)
        self.declare_parameter("max_estimated_speed_mps", 0.05)
        self.declare_parameter("min_motion_speed_scale", 0.20)
        self.declare_parameter("object_timeout_sec", 0.8)
        self.declare_parameter("min_landmark_confidence", 0.35)
        self.declare_parameter("enable_landmark_correction", False)
        self.declare_parameter("landmark_correction_gain", 0.18)
        self.declare_parameter("enable_lateral_map_offset", True)
        self.declare_parameter("lateral_offset_sign", -1.0)
        self.declare_parameter("map_lateral_offset_scale_m", 0.45)
        self.declare_parameter("path_deviation_warn_m", 0.25)
        self.declare_parameter("line_heading_sign", -1.0)
        self.declare_parameter("imu_yaw_heading_gain", 0.15)
        self.declare_parameter("gyro_z_sign", 1.0)
        self.declare_parameter("gyro_bias_calibration_sec", 1.5)
        self.declare_parameter("max_imu_dt_sec", 0.2)
        self.declare_parameter("line_heading_correction_gain", 0.04)
        self.declare_parameter("route_correction_gain", 0.02)

        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.accel_topic = str(self.get_parameter("accel_topic").value)
        self.line_info_topic = str(self.get_parameter("line_info_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.robot_pose_topic = str(self.get_parameter("robot_pose_topic").value)
        self.line_timeout_sec = float(
            self.get_parameter("line_timeout_sec").value
        )
        self.line_quality_threshold = float(
            self.get_parameter("line_quality_threshold").value
        )
        self.enable_route_progress = bool(
            self.get_parameter("enable_route_progress").value
        )
        self.nominal_forward_speed_mps = float(
            self.get_parameter("nominal_forward_speed_mps").value
        )
        self.auto_progress_when_line_visible = bool(
            self.get_parameter("auto_progress_when_line_visible").value
        )
        self.use_accel_motion_gate = bool(
            self.get_parameter("use_accel_motion_gate").value
        )
        self.accel_motion_threshold_mps2 = float(
            self.get_parameter("accel_motion_threshold_mps2").value
        )
        self.gyro_motion_threshold_rad_s = float(
            self.get_parameter("gyro_motion_threshold_rad_s").value
        )
        self.motion_hold_sec = float(
            self.get_parameter("motion_hold_sec").value
        )
        self.motion_start_sec = float(
            self.get_parameter("motion_start_sec").value
        )
        self.motion_score_start_threshold = float(
            self.get_parameter("motion_score_start_threshold").value
        )
        self.motion_score_stop_threshold = float(
            self.get_parameter("motion_score_stop_threshold").value
        )
        self.max_estimated_speed_mps = float(
            self.get_parameter("max_estimated_speed_mps").value
        )
        self.min_motion_speed_scale = float(
            self.get_parameter("min_motion_speed_scale").value
        )
        self.object_timeout_sec = float(
            self.get_parameter("object_timeout_sec").value
        )
        self.min_landmark_confidence = float(
            self.get_parameter("min_landmark_confidence").value
        )
        self.enable_landmark_correction = bool(
            self.get_parameter("enable_landmark_correction").value
        )
        self.landmark_correction_gain = float(
            self.get_parameter("landmark_correction_gain").value
        )
        self.enable_lateral_map_offset = bool(
            self.get_parameter("enable_lateral_map_offset").value
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
        self.imu_yaw_heading_gain = float(
            self.get_parameter("imu_yaw_heading_gain").value
        )
        self.gyro_z_sign = float(self.get_parameter("gyro_z_sign").value)
        self.gyro_bias_calibration_sec = float(
            self.get_parameter("gyro_bias_calibration_sec").value
        )
        self.max_imu_dt_sec = float(self.get_parameter("max_imu_dt_sec").value)
        self.line_heading_correction_gain = float(
            self.get_parameter("line_heading_correction_gain").value
        )
        self.route_correction_gain = float(
            self.get_parameter("route_correction_gain").value
        )

        self.publisher = self.create_publisher(String, self.robot_pose_topic, 10)
        self.imu_subscription = self.create_subscription(
            Imu,
            self.imu_topic,
            self._imu_callback,
            qos_profile_sensor_data,
        )
        self.accel_subscription = self.create_subscription(
            Imu,
            self.accel_topic,
            self._accel_callback,
            qos_profile_sensor_data,
        )
        self.line_subscription = self.create_subscription(
            String,
            self.line_info_topic,
            self._line_info_callback,
            10,
        )
        self.detection_subscription = self.create_subscription(
            String,
            self.detections_topic,
            self._detections_callback,
            10,
        )

        publish_rate_hz = max(
            1.0,
            float(self.get_parameter("publish_rate_hz").value),
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_pose)

        # Same 1m-grid course route as mission_map_visualizer.
        self.route_points = self._build_route_points()
        self.segment_lengths = self._calculate_segment_lengths(self.route_points)
        self.total_route_length_m = sum(self.segment_lengths)

        self.route_progress_m = 0.0
        self.imu_yaw_delta_rad = 0.0
        self.latest_gyro_z_rad_s = 0.0
        self.latest_accel_motion_mps2 = 0.0
        self.latest_imu_time: float | None = None
        self.latest_motion_time: float | None = None
        self.motion_candidate_since: float | None = None
        self.motion_active = False
        self.latest_publish_time = time.monotonic()
        self.latest_line_info: dict[str, Any] | None = None
        self.latest_line_time: float | None = None
        self.latest_objects: dict[str, dict[str, Any]] = {}
        self.latest_object_time: float | None = None

        self.started_at = time.monotonic()
        self.gyro_bias_sum = 0.0
        self.gyro_bias_count = 0
        self.gyro_bias_rad_s = 0.0
        self.accel_bias_sum = [0.0, 0.0, 0.0]
        self.accel_bias_count = 0
        self.accel_bias = [0.0, 0.0, 0.0]
        self.bias_ready = False
        self.landmark_progress = self._build_landmark_progress()

        self.get_logger().info(f"Subscribing IMU gyro: {self.imu_topic}")
        self.get_logger().info(f"Subscribing IMU accel: {self.accel_topic}")
        self.get_logger().info(f"Subscribing line info: {self.line_info_topic}")
        self.get_logger().info(f"Subscribing detections: {self.detections_topic}")
        self.get_logger().info(f"Publishing robot pose: {self.robot_pose_topic}")
        self.get_logger().info(
            "Keep the robot still for the first "
            f"{self.gyro_bias_calibration_sec:.1f}s for gyro/accel calibration."
        )

    @staticmethod
    def _calculate_segment_lengths(
        points: list[tuple[float, float]],
    ) -> list[float]:
        return [
            math.hypot(
                b[0] - a[0],
                b[1] - a[1],
            )
            for a, b in zip(points[:-1], points[1:])
        ]

    @staticmethod
    def _quarter_arc_points(
        center: tuple[float, float],
        radius: float,
        start_deg: float,
        end_deg: float,
        steps: int = 8,
    ) -> list[tuple[float, float]]:
        """Build points along a quarter-circle arc in grid coordinates."""

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

    @staticmethod
    def _wrap_deg(angle_deg: float) -> float:
        while angle_deg > 180.0:
            angle_deg -= 360.0
        while angle_deg < -180.0:
            angle_deg += 360.0
        return angle_deg

    @staticmethod
    def _distance_point_to_segment(
        point: tuple[float, float],
        start: tuple[float, float],
        end: tuple[float, float],
    ) -> tuple[float, float]:
        """Return distance and 0..1 segment ratio for closest point."""

        px, py = point
        sx, sy = start
        ex, ey = end
        dx = ex - sx
        dy = ey - sy
        segment_sq = dx * dx + dy * dy

        if segment_sq <= 1e-9:
            return math.hypot(px - sx, py - sy), 0.0

        ratio = (
            ((px - sx) * dx + (py - sy) * dy)
            / segment_sq
        )
        ratio = max(0.0, min(1.0, ratio))
        cx = sx + dx * ratio
        cy = sy + dy * ratio
        return math.hypot(px - cx, py - cy), ratio

    def _nearest_route_progress(
        self,
        point: tuple[float, float],
    ) -> float:
        """Project a fixed field landmark onto the known route path."""

        best_distance = float("inf")
        best_progress = 0.0
        accumulated = 0.0

        for index, segment_length in enumerate(self.segment_lengths):
            start = self.route_points[index]
            end = self.route_points[index + 1]
            distance, ratio = self._distance_point_to_segment(
                point,
                start,
                end,
            )

            if distance < best_distance:
                best_distance = distance
                best_progress = accumulated + segment_length * ratio

            accumulated += segment_length

        return best_progress

    def _build_landmark_progress(self) -> dict[str, list[float]]:
        """Map fixed landmark classes to route-progress positions."""

        return {
            "ball": [
                self._nearest_route_progress((0.20, 0.20)),
                self._nearest_route_progress((2.20, 4.80)),
            ],
            "goal": [
                self._nearest_route_progress((3.15, 0.40)),
                self._nearest_route_progress((5.00, 4.55)),
            ],
            "backboard": [
                self._nearest_route_progress((3.15, 0.40)),
                self._nearest_route_progress((5.00, 4.55)),
            ],
        }

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

    def _detections_callback(self, message: String) -> None:
        """Store strongest recent landmark detections."""

        try:
            payload = json.loads(message.data)
            detections = payload.get("detections", [])
            if not isinstance(detections, list):
                raise ValueError("'detections' must be a list")

            objects: dict[str, dict[str, Any]] = {}

            for detection in detections:
                if not isinstance(detection, dict):
                    continue

                class_name = str(detection.get("class_name", ""))
                if class_name not in {"ball", "goal", "backboard"}:
                    continue

                confidence = float(detection.get("confidence", 0.0) or 0.0)
                if confidence < self.min_landmark_confidence:
                    continue

                previous = objects.get(class_name)
                if (
                    previous is None
                    or confidence > float(previous.get("confidence", 0.0) or 0.0)
                ):
                    objects[class_name] = detection

            self.latest_objects = objects
            self.latest_object_time = time.monotonic()

        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                f"Invalid detections message: {type(exc).__name__}: {exc}"
            )

    def _imu_callback(self, message: Imu) -> None:
        now = time.monotonic()

        raw_gyro_z = float(message.angular_velocity.z) * self.gyro_z_sign
        elapsed = now - self.started_at

        if elapsed <= self.gyro_bias_calibration_sec:
            self.gyro_bias_sum += raw_gyro_z
            self.gyro_bias_count += 1
            self.gyro_bias_rad_s = self.gyro_bias_sum / max(1, self.gyro_bias_count)
            self.latest_imu_time = now
            self.latest_gyro_z_rad_s = 0.0
            return

        if not self.bias_ready:
            self.bias_ready = True
            self.get_logger().info(
                f"Gyro bias calibrated: {self.gyro_bias_rad_s:.6f} rad/s"
            )

        if self.latest_imu_time is None:
            self.latest_imu_time = now
            return

        dt = now - self.latest_imu_time
        self.latest_imu_time = now

        if dt <= 0.0 or dt > self.max_imu_dt_sec:
            return

        corrected_gyro_z = raw_gyro_z - self.gyro_bias_rad_s
        self.latest_gyro_z_rad_s = corrected_gyro_z
        self.imu_yaw_delta_rad += corrected_gyro_z * dt

    def _accel_callback(self, message: Imu) -> None:
        """Detect real camera/robot motion from acceleration changes."""

        now = time.monotonic()
        elapsed = now - self.started_at
        accel = [
            float(message.linear_acceleration.x),
            float(message.linear_acceleration.y),
            float(message.linear_acceleration.z),
        ]

        if elapsed <= self.gyro_bias_calibration_sec:
            for index, value in enumerate(accel):
                self.accel_bias_sum[index] += value
            self.accel_bias_count += 1
            count = max(1, self.accel_bias_count)
            self.accel_bias = [
                value / count
                for value in self.accel_bias_sum
            ]
            self.latest_accel_motion_mps2 = 0.0
            return

        delta = [
            accel[index] - self.accel_bias[index]
            for index in range(3)
        ]
        self.latest_accel_motion_mps2 = math.sqrt(
            sum(value * value for value in delta)
        )

    def _line_is_fresh(self) -> bool:
        if self.latest_line_time is None:
            return False
        return (time.monotonic() - self.latest_line_time) <= self.line_timeout_sec

    def _line_quality(self) -> float:
        if not self._line_is_fresh() or self.latest_line_info is None:
            return 0.0

        heading_quality = float(
            self.latest_line_info.get("heading_quality", 0.0) or 0.0
        )
        geometry_quality = float(
            self.latest_line_info.get("geometry_quality", 0.0) or 0.0
        )
        detection_quality = float(
            self.latest_line_info.get("detection_quality", 0.0) or 0.0
        )

        return min(heading_quality, geometry_quality, detection_quality)

    def _objects_are_fresh(self) -> bool:
        if self.latest_object_time is None:
            return False
        return (time.monotonic() - self.latest_object_time) <= self.object_timeout_sec

    def _visible_landmarks(self) -> list[str]:
        if not self._objects_are_fresh():
            return []
        return sorted(self.latest_objects)

    def _apply_landmark_correction(self) -> str:
        """Correct route progress using fixed ball/goal/backboard landmarks."""

        if not self.enable_landmark_correction:
            return "disabled"

        if not self._objects_are_fresh() or not self.latest_objects:
            return "none"

        best_class = ""
        best_target = None
        best_confidence = 0.0

        for class_name, detection in self.latest_objects.items():
            candidates = self.landmark_progress.get(class_name, [])
            if not candidates:
                continue

            confidence = float(detection.get("confidence", 0.0) or 0.0)
            target = min(
                candidates,
                key=lambda value: abs(value - self.route_progress_m),
            )

            if confidence > best_confidence:
                best_class = class_name
                best_target = target
                best_confidence = confidence

        if best_target is None:
            return "none"

        gain = self.landmark_correction_gain * max(
            0.2,
            min(best_confidence, 1.0),
        )
        previous = self.route_progress_m
        self.route_progress_m = (
            (1.0 - gain) * self.route_progress_m
            + gain * best_target
        )
        self.route_progress_m = max(
            0.0,
            min(self.route_progress_m, self.total_route_length_m),
        )

        delta = self.route_progress_m - previous
        return f"{best_class}:{delta:+.2f}m"

    def _motion_is_recent(self) -> bool:
        if not self.use_accel_motion_gate:
            return True

        now = time.monotonic()
        score = self._motion_score()

        if score >= self.motion_score_start_threshold:
            if self.motion_candidate_since is None:
                self.motion_candidate_since = now
            if (now - self.motion_candidate_since) >= self.motion_start_sec:
                self.motion_active = True
                self.latest_motion_time = now

        elif score <= self.motion_score_stop_threshold:
            self.motion_candidate_since = None
            if (
                self.latest_motion_time is None
                or (now - self.latest_motion_time) > self.motion_hold_sec
            ):
                self.motion_active = False

        return self.motion_active

    def _motion_score(self) -> float:
        """Return 0..1 rough motion score from accel and gyro changes."""

        accel_score = (
            self.latest_accel_motion_mps2
            / max(self.accel_motion_threshold_mps2, 1e-6)
        )
        gyro_score = (
            abs(self.latest_gyro_z_rad_s)
            / max(self.gyro_motion_threshold_rad_s, 1e-6)
        )

        return float(
            max(
                0.0,
                min(
                    max(accel_score, gyro_score),
                    1.5,
                ),
            )
            / 1.5
        )

    def _estimated_forward_speed(self, moving: bool) -> float:
        """Estimate forward speed for the map marker from motion intensity."""

        if not moving:
            return 0.0

        score = self._motion_score()
        if score < self.motion_score_stop_threshold:
            return 0.0

        speed_scale = max(
            self.min_motion_speed_scale,
            min(score, 1.0),
        )

        return min(
            self.nominal_forward_speed_mps * speed_scale,
            self.max_estimated_speed_mps,
        )

    def _lateral_offset_m(self) -> float:
        """Estimate lateral offset from the detected line center on the map."""

        if (
            not self.enable_lateral_map_offset
            or not self._line_is_fresh()
            or self.latest_line_info is None
        ):
            return 0.0

        offset_norm = float(
            self.latest_line_info.get("filtered_lateral_offset_norm", 0.0)
            or 0.0
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
        """Return x, y, route tangent heading at progress along the course."""

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
                    math.atan2(
                        end[1] - start[1],
                        end[0] - start[0],
                    )
                )
                return x_m, y_m, heading_deg

            remaining -= segment_length

        x_m, y_m = self.route_points[-1]
        prev_x, prev_y = self.route_points[-2]
        heading_deg = math.degrees(math.atan2(y_m - prev_y, x_m - prev_x))
        return x_m, y_m, heading_deg

    def _publish_pose(self) -> None:
        now = time.monotonic()
        dt = now - self.latest_publish_time
        self.latest_publish_time = now

        line_detected = bool(
            self.latest_line_info.get("detected", False)
            if self.latest_line_info is not None and self._line_is_fresh()
            else False
        )
        line_quality = self._line_quality()
        line_ready = line_detected and line_quality >= self.line_quality_threshold
        motion_ready = self._motion_is_recent()
        moving = line_ready and (
            motion_ready
            or self.auto_progress_when_line_visible
        )
        estimated_speed_mps = self._estimated_forward_speed(moving)
        motion_score = self._motion_score()

        if self.enable_route_progress and moving:
            self.route_progress_m += estimated_speed_mps * max(0.0, dt)
            self.route_progress_m = min(
                self.route_progress_m,
                self.total_route_length_m,
            )

        correction_source = self._apply_landmark_correction()

        route_center_x_m, route_center_y_m, route_heading_deg = self._route_pose_at(
            self.route_progress_m
        )
        lateral_offset_m = self._lateral_offset_m()
        lateral_angle_rad = math.radians(route_heading_deg + 90.0)
        x_m = (
            route_center_x_m
            + math.cos(lateral_angle_rad) * lateral_offset_m
        )
        y_m = (
            route_center_y_m
            + math.sin(lateral_angle_rad) * lateral_offset_m
        )
        path_deviation_status = self._path_deviation_status(lateral_offset_m)

        line_heading_error_deg = 0.0
        if self.latest_line_info is not None and self._line_is_fresh():
            line_heading_error_deg = float(
                self.latest_line_info.get("filtered_heading_error_deg", 0.0)
                or 0.0
            )

            # Keep the yaw estimate from drifting too far away from the line.
            correction_rad = math.radians(
                line_heading_error_deg
                * line_quality
                * self.line_heading_correction_gain
            )
            self.imu_yaw_delta_rad = (
                (1.0 - self.route_correction_gain) * self.imu_yaw_delta_rad
                + correction_rad
            )

        heading_deg = self._wrap_deg(
            route_heading_deg
            + self.line_heading_sign * line_heading_error_deg
            + self.imu_yaw_heading_gain * math.degrees(self.imu_yaw_delta_rad)
        )

        confidence = 0.25
        note = "Waiting for reliable line and IMU."
        if not self.enable_route_progress:
            confidence = max(0.30, min(line_quality, 0.80))
            note = "Route progress is locked; showing lateral map offset."
        elif self.bias_ready and moving:
            confidence = max(0.35, min(line_quality, 0.95))
            note = "Line is reliable and sustained IMU motion is detected."
            if correction_source != "none":
                confidence = min(0.98, confidence + 0.08)
                note = f"Vision landmark correction: {correction_source}"
        elif self.bias_ready and line_detected:
            confidence = max(0.30, min(line_quality * 0.7, 0.75))
            if line_ready and not motion_ready:
                note = "Line visible, waiting for sustained robot motion."
            else:
                note = "Line visible, but quality is low for movement."
            if correction_source != "none":
                confidence = min(0.95, confidence + 0.10)
                note = f"Vision landmark correction: {correction_source}"
        elif not self.bias_ready:
            note = "Calibrating IMU bias; keep robot still."

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
            path_deviation_status=path_deviation_status,
            line_detected=line_detected,
            line_quality=round(line_quality, 3),
            imu_yaw_delta_deg=round(math.degrees(self.imu_yaw_delta_rad), 3),
            gyro_z_rad_s=round(self.latest_gyro_z_rad_s, 6),
            accel_motion_mps2=round(self.latest_accel_motion_mps2, 4),
            estimated_speed_mps=round(estimated_speed_mps, 3),
            motion_score=round(motion_score, 3),
            moving=moving,
            visible_landmarks=self._visible_landmarks(),
            correction_source=correction_source,
            confidence=round(confidence, 3),
            source="imu+line+nominal_speed",
            note=note,
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

    node = ImuLinePoseEstimator()

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
