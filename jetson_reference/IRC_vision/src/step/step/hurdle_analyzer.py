#!/usr/bin/env python3
"""Analyze YOLO hurdle detections and aligned depth for jump readiness."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import time
from typing import Any

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .approach_distance import approach_level_from_motion
from .approach_distance import approach_motion_for_distance
from .depth_frame_cache import DepthFrameCache
from .depth_frame_cache import DepthFrameConsumer
from .temporal_confirmation import depth_is_within_range
from .temporal_confirmation import TemporalConfirmationFilter


@dataclass(frozen=True)
class HurdleCandidate:
    """One hurdle candidate with image and depth geometry."""

    confidence: float
    bbox: list[int]
    center: list[int]
    width_px: int
    height_px: int
    area_px: int
    offset_x_px: int
    offset_y_px: int
    offset_x_norm: float
    offset_y_norm: float
    horizontal_direction: str
    bearing_deg: float | None
    elevation_deg: float | None
    depth_m: float | None
    horizontal_distance_m: float | None
    distance_m: float | None
    ground_gap_m: float | None
    ground_distance_m: float | None
    camera_bottom_gap_px: int | None
    camera_bottom_gap_m: float | None
    lateral_offset_m: float | None
    estimated_width_m: float | None
    left_depth_m: float | None
    right_depth_m: float | None
    hurdle_angle_deg: float | None
    depth_valid: bool
    depth_sample_count: int
    score: float


@dataclass(frozen=True)
class HurdleInfo:
    """Selected hurdle geometry and provisional jump conditions."""

    detected: bool
    state: str
    confidence: float
    center_x: int | None
    center_y: int | None
    bbox: list[int] | None
    width_px: int | None
    height_px: int | None
    area_px: int | None
    offset_x_px: int | None
    offset_y_px: int | None
    offset_x_norm: float | None
    offset_y_norm: float | None
    horizontal_direction: str
    bearing_deg: float | None
    elevation_deg: float | None
    depth_m: float | None
    horizontal_distance_m: float | None
    distance_m: float | None
    ground_gap_m: float | None
    ground_distance_m: float | None
    camera_bottom_gap_px: int | None
    camera_bottom_gap_m: float | None
    lateral_offset_m: float | None
    estimated_width_m: float | None
    left_depth_m: float | None
    right_depth_m: float | None
    hurdle_angle_deg: float | None
    depth_valid: bool
    depth_sample_count: int
    is_parallel: bool
    ground_gap_in_go_range: bool
    go_ground_gap_error_m: float | None
    go_now: bool
    approach_motion: str
    approach_level: int | None
    approach_target_distance_m: float | None
    target_priority_score: float
    candidate_count: int
    candidates: list[HurdleCandidate]
    image_width: int | None
    image_height: int | None
    camera_info_ready: bool
    depth_age_sec: float | None
    note: str


class HurdleAnalyzer(DepthFrameConsumer, Node):
    """Convert raw hurdle detections into an SDK jump-ready signal."""

    def __init__(self, depth_cache: DepthFrameCache | None = None) -> None:
        super().__init__("hurdle_analyzer")

        self.declare_parameter("detections_topic", "/vision/detections")
        self.declare_parameter(
            "depth_topic",
            "/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/camera/color/camera_info",
        )
        self.declare_parameter("output_topic", "/vision/hurdle_info")
        self.declare_parameter("hurdle_class_name", "hurdle")
        self.declare_parameter("min_confidence", 0.60)
        self.declare_parameter("depth_timeout_sec", 0.7)
        self.declare_parameter("depth_window_px", 9)
        self.declare_parameter("max_valid_depth_m", 4.0)
        self.declare_parameter("detect_depth_m", 1.5)
        self.declare_parameter("go_target_depth_m", 0.10)
        self.declare_parameter("go_depth_tolerance_m", 0.10)
        self.declare_parameter("go_angle_tolerance_deg", 8.0)
        self.declare_parameter("direction_deadband_norm", 0.04)
        self.declare_parameter("confirmation_window_size", 20)
        self.declare_parameter("confirmation_required_hits", 12)
        self.declare_parameter("confirmation_max_missed_frames", 4)
        self.declare_parameter("confirmation_max_center_shift_norm", 0.20)
        self.declare_parameter("confirmation_min_area_ratio", 0.40)
        self.declare_parameter("go_confirmation_window_size", 7)
        self.declare_parameter("go_confirmation_required_hits", 5)
        self.declare_parameter("publish_empty_when_missing", True)

        self.detections_topic = self._string_parameter("detections_topic")
        self.depth_topic = self._string_parameter("depth_topic")
        self.camera_info_topic = self._string_parameter(
            "camera_info_topic"
        )
        self.output_topic = self._string_parameter("output_topic")
        self.hurdle_class_name = self._string_parameter(
            "hurdle_class_name"
        )
        self.min_confidence = self._float_parameter("min_confidence")
        self.depth_timeout_sec = self._float_parameter(
            "depth_timeout_sec"
        )
        self.depth_window_px = max(
            3,
            int(self.get_parameter("depth_window_px").value),
        )
        if self.depth_window_px % 2 == 0:
            self.depth_window_px += 1
        self.max_valid_depth_m = self._float_parameter(
            "max_valid_depth_m"
        )
        self.detect_depth_m = self._float_parameter("detect_depth_m")
        self.go_target_depth_m = max(
            0.0,
            self._float_parameter("go_target_depth_m"),
        )
        self.go_depth_tolerance_m = max(
            0.0,
            self._float_parameter("go_depth_tolerance_m"),
        )
        self.go_angle_tolerance_deg = max(
            0.0,
            self._float_parameter("go_angle_tolerance_deg"),
        )
        self.direction_deadband_norm = max(
            0.0,
            self._float_parameter("direction_deadband_norm"),
        )
        self.publish_empty_when_missing = bool(
            self.get_parameter("publish_empty_when_missing").value
        )

        self.confirmation_filter = TemporalConfirmationFilter(
            window_size=int(
                self.get_parameter("confirmation_window_size").value
            ),
            required_hits=int(
                self.get_parameter("confirmation_required_hits").value
            ),
            max_missed_frames=int(
                self.get_parameter("confirmation_max_missed_frames").value
            ),
            max_center_shift_norm=float(
                self.get_parameter(
                    "confirmation_max_center_shift_norm"
                ).value
            ),
            min_area_ratio=float(
                self.get_parameter("confirmation_min_area_ratio").value
            ),
        )
        self.go_confirmation_filter = TemporalConfirmationFilter(
            window_size=int(
                self.get_parameter("go_confirmation_window_size").value
            ),
            required_hits=int(
                self.get_parameter("go_confirmation_required_hits").value
            ),
            max_missed_frames=0,
            spatial_matching=False,
        )
        self.confirmation_fields: dict[str, object] = {}
        self.go_confirmation_fields: dict[str, object] = {}

        self.depth_cache = depth_cache or DepthFrameCache()
        self._owns_depth_subscription = depth_cache is None
        self.bridge = CvBridge() if self._owns_depth_subscription else None
        self.fx: float | None = None
        self.fy: float | None = None
        self.cx: float | None = None
        self.cy: float | None = None

        self._detection_callback_group = MutuallyExclusiveCallbackGroup()
        self._depth_callback_group = MutuallyExclusiveCallbackGroup()
        self._camera_info_callback_group = MutuallyExclusiveCallbackGroup()
        latest_detection_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latest_depth_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(String, self.output_topic, 10)
        self.create_subscription(
            String,
            self.detections_topic,
            self._detections_callback,
            latest_detection_qos,
            callback_group=self._detection_callback_group,
        )
        if self._owns_depth_subscription:
            self.create_subscription(
                Image,
                self.depth_topic,
                self._depth_callback,
                latest_depth_qos,
                callback_group=self._depth_callback_group,
            )
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            10,
            callback_group=self._camera_info_callback_group,
        )

        self.get_logger().info(
            f"Subscribing detections: {self.detections_topic}"
        )
        if self._owns_depth_subscription:
            self.get_logger().info(
                f"Subscribing aligned depth: {self.depth_topic}"
            )
        self.get_logger().info(
            f"Publishing hurdle info: {self.output_topic}"
        )

    def _string_parameter(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _camera_info_callback(self, message: CameraInfo) -> None:
        if len(message.k) < 6:
            return
        fx = float(message.k[0])
        fy = float(message.k[4])
        if fx <= 0.0 or fy <= 0.0:
            return
        self.fx = fx
        self.fy = fy
        self.cx = float(message.k[2])
        self.cy = float(message.k[5])

    def _depth_callback(self, message: Image) -> None:
        try:
            if self.bridge is not None:
                self._store_depth_message(message, self.bridge)
        except Exception as exc:
            self.get_logger().warning(f"Could not read depth image: {exc}")

    def _depth_age_sec(self) -> float | None:
        if self.latest_depth_time is None:
            return None
        return time.monotonic() - self.latest_depth_time

    def _valid_patch_m(self, x: int, y: int) -> np.ndarray:
        if self.latest_depth_image is None:
            return np.empty(0, dtype=np.float32)
        depth = self.latest_depth_image
        height, width = depth.shape[:2]
        if not (0 <= x < width and 0 <= y < height):
            return np.empty(0, dtype=np.float32)
        radius = self.depth_window_px // 2
        patch = depth[
            max(0, y - radius):min(height, y + radius + 1),
            max(0, x - radius):min(width, x + radius + 1),
        ]
        if patch.dtype == np.uint16:
            patch_m = patch.astype(np.float32) * 0.001
        else:
            patch_m = patch.astype(np.float32)
        mask = (
            np.isfinite(patch_m)
            & (patch_m > 0.05)
            & (patch_m <= self.max_valid_depth_m)
        )
        return patch_m[mask]

    def _sample_depths(
        self,
        bbox: list[int],
        source_width: int | None,
        source_height: int | None,
    ) -> tuple[float | None, float | None, float | None, int]:
        depth_age = self._depth_age_sec()
        if (
            self.latest_depth_image is None
            or self.latest_depth_time is None
            or depth_age is None
            or depth_age > self.depth_timeout_sec
            or self.latest_depth_image.ndim != 2
        ):
            return None, None, None, 0
        depth_height, depth_width = self.latest_depth_image.shape[:2]
        source_width = int(source_width or depth_width)
        source_height = int(source_height or depth_height)
        scale_x = depth_width / max(source_width, 1)
        scale_y = depth_height / max(source_height, 1)
        left, top, right, bottom = (
            int(round(bbox[0] * scale_x)),
            int(round(bbox[1] * scale_y)),
            int(round(bbox[2] * scale_x)),
            int(round(bbox[3] * scale_y)),
        )
        left = max(0, min(depth_width - 1, left))
        right = max(left + 1, min(depth_width, right))
        top = max(0, min(depth_height - 1, top))
        bottom = max(top + 1, min(depth_height, bottom))
        width = right - left
        sample_y = int(round(top + (bottom - top) * 0.55))
        ratios = [0.15, 0.325, 0.50, 0.675, 0.85]
        medians: list[float | None] = []
        for ratio in ratios:
            values = self._valid_patch_m(
                int(round(left + width * ratio)),
                sample_y,
            )
            medians.append(
                float(np.median(values)) if values.size else None
            )
        valid = [value for value in medians if value is not None]
        if not valid:
            return None, None, None, 0
        left_depth = next(
            (value for value in medians if value is not None),
            None,
        )
        right_depth = next(
            (value for value in reversed(medians) if value is not None),
            None,
        )
        return float(np.median(valid)), left_depth, right_depth, len(valid)

    def _image_size(
        self,
        payload: dict[str, Any],
    ) -> tuple[int | None, int | None]:
        try:
            width = int(payload.get("image_width", 0))
            height = int(payload.get("image_height", 0))
        except (TypeError, ValueError):
            width = 0
            height = 0
        if width > 0 and height > 0:
            return width, height
        return self.latest_image_width, self.latest_image_height

    def _build_candidate(
        self,
        detection: dict[str, Any],
        image_width: int | None,
        image_height: int | None,
    ) -> HurdleCandidate | None:
        try:
            confidence = float(detection.get("confidence", 0.0))
            bbox = [int(value) for value in detection.get("bbox", [])]
            center = [int(value) for value in detection.get("center", [])]
        except (TypeError, ValueError):
            return None
        if (
            confidence < self.min_confidence
            or len(bbox) != 4
            or len(center) != 2
        ):
            return None
        left, top, right, bottom = bbox
        width_px = right - left
        height_px = bottom - top
        if width_px <= 0 or height_px <= 0:
            return None

        center_x, center_y = center
        offset_x_px = 0
        offset_y_px = 0
        offset_x_norm = 0.0
        offset_y_norm = 0.0
        if image_width and image_height:
            offset_x_px = int(center_x - image_width / 2)
            offset_y_px = int(center_y - image_height / 2)
            offset_x_norm = offset_x_px / max(image_width / 2, 1.0)
            offset_y_norm = offset_y_px / max(image_height / 2, 1.0)

        depth, left_depth, right_depth, sample_count = (
            self._sample_depths(bbox, image_width, image_height)
        )
        depth_valid = depth is not None
        bearing: float | None = None
        elevation: float | None = None
        lateral: float | None = None
        horizontal_distance: float | None = None
        distance: float | None = depth
        ground_gap: float | None = depth
        camera_bottom_gap_px: int | None = None
        camera_bottom_gap: float | None = None
        estimated_width: float | None = None
        hurdle_angle: float | None = None
        if self.fx and self.fy and self.cx is not None and self.cy is not None:
            x_ratio = (center_x - self.cx) / self.fx
            depth_sample_y = int(round(top + height_px * 0.55))
            y_ratio = (depth_sample_y - self.cy) / self.fy
            bearing = math.degrees(math.atan(x_ratio))
            elevation = math.degrees(math.atan(y_ratio))
            if depth is not None:
                lateral = x_ratio * depth
                # HURDLE distance control uses raw aligned Depth Z only.  The
                # old horizontal/slant/ground calculations are deliberately
                # not evaluated; legacy JSON distance fields alias depth.
                if image_height is not None:
                    camera_bottom_gap_px = max(0, image_height - bottom)
                    camera_bottom_gap = (
                        camera_bottom_gap_px * depth / self.fy
                    )
                estimated_width = width_px * depth / self.fx
                if (
                    left_depth is not None
                    and right_depth is not None
                    and estimated_width > 1e-3
                ):
                    hurdle_angle = math.degrees(
                        math.atan2(
                            right_depth - left_depth,
                            estimated_width,
                        )
                    )

        if offset_x_norm < -self.direction_deadband_norm:
            direction = "LEFT"
        elif offset_x_norm > self.direction_deadband_norm:
            direction = "RIGHT"
        else:
            direction = "CENTER"

        area_px = width_px * height_px
        angle_score = (
            max(0.0, 1.0 - abs(hurdle_angle) / 45.0)
            if hurdle_angle is not None
            else 0.0
        )
        depth_score = (
            max(0.0, 1.0 - depth / self.max_valid_depth_m)
            if depth is not None
            else 0.0
        )
        area_score = min(1.0, math.sqrt(area_px) / 250.0)
        priority = (
            confidence * 0.50
            + angle_score * 0.25
            + depth_score * 0.15
            + area_score * 0.10
        )
        return HurdleCandidate(
            confidence=round(confidence, 4),
            bbox=bbox,
            center=center,
            width_px=width_px,
            height_px=height_px,
            area_px=area_px,
            offset_x_px=offset_x_px,
            offset_y_px=offset_y_px,
            offset_x_norm=round(offset_x_norm, 4),
            offset_y_norm=round(offset_y_norm, 4),
            horizontal_direction=direction,
            bearing_deg=round(bearing, 3) if bearing is not None else None,
            elevation_deg=(
                round(elevation, 3) if elevation is not None else None
            ),
            depth_m=round(depth, 3) if depth is not None else None,
            horizontal_distance_m=(
                round(horizontal_distance, 3)
                if horizontal_distance is not None
                else None
            ),
            distance_m=(
                round(distance, 3) if distance is not None else None
            ),
            ground_gap_m=(
                round(ground_gap, 3) if ground_gap is not None else None
            ),
            ground_distance_m=(
                round(ground_gap, 3) if ground_gap is not None else None
            ),
            camera_bottom_gap_px=camera_bottom_gap_px,
            camera_bottom_gap_m=(
                round(camera_bottom_gap, 3)
                if camera_bottom_gap is not None
                else None
            ),
            lateral_offset_m=(
                round(lateral, 3) if lateral is not None else None
            ),
            estimated_width_m=(
                round(estimated_width, 3)
                if estimated_width is not None
                else None
            ),
            left_depth_m=(
                round(left_depth, 3) if left_depth is not None else None
            ),
            right_depth_m=(
                round(right_depth, 3) if right_depth is not None else None
            ),
            hurdle_angle_deg=(
                round(hurdle_angle, 3)
                if hurdle_angle is not None
                else None
            ),
            depth_valid=depth_valid,
            depth_sample_count=sample_count,
            score=round(priority, 4),
        )

    def _state(
        self,
        target: HurdleCandidate,
    ) -> tuple[str, bool, bool, float | None, bool, str]:
        parallel = (
            target.hurdle_angle_deg is not None
            and abs(target.hurdle_angle_deg)
            <= self.go_angle_tolerance_deg
        )
        if (
            not target.depth_valid
            or target.depth_m is None
        ):
            return (
                "NO_DEPTH",
                parallel,
                False,
                None,
                False,
                "hurdle_detected_without_valid_depth",
            )
        if target.hurdle_angle_deg is None:
            return (
                "NO_ANGLE",
                False,
                False,
                None,
                False,
                "hurdle_parallel_angle_unavailable",
            )
        error = (
            target.depth_m - self.go_target_depth_m
        )
        depth_in_range = (
            abs(error) <= self.go_depth_tolerance_m + 1e-9
        )
        go_now = parallel and depth_in_range
        if go_now:
            return (
                "GO_READY",
                parallel,
                True,
                error,
                True,
                "hurdle_parallel_at_close_depth",
            )
        if not parallel:
            return (
                "ALIGN_ANGLE",
                False,
                depth_in_range,
                error,
                False,
                "align_robot_parallel_to_hurdle",
            )
        if (
            error > self.go_depth_tolerance_m
        ):
            return "APPROACH", True, False, error, False, "hurdle_depth_too_far"
        return "GO_READY", True, True, error, True, "hurdle_close_enough"

    def _empty_info(
        self,
        note: str = "no_hurdle_detection",
    ) -> HurdleInfo:
        age = self._depth_age_sec()
        return HurdleInfo(
            detected=False,
            state="SEARCH",
            confidence=0.0,
            center_x=None,
            center_y=None,
            bbox=None,
            width_px=None,
            height_px=None,
            area_px=None,
            offset_x_px=None,
            offset_y_px=None,
            offset_x_norm=None,
            offset_y_norm=None,
            horizontal_direction="UNKNOWN",
            bearing_deg=None,
            elevation_deg=None,
            depth_m=None,
            horizontal_distance_m=None,
            distance_m=None,
            ground_gap_m=None,
            ground_distance_m=None,
            camera_bottom_gap_px=None,
            camera_bottom_gap_m=None,
            lateral_offset_m=None,
            estimated_width_m=None,
            left_depth_m=None,
            right_depth_m=None,
            hurdle_angle_deg=None,
            depth_valid=False,
            depth_sample_count=0,
            is_parallel=False,
            ground_gap_in_go_range=False,
            go_ground_gap_error_m=None,
            go_now=False,
            approach_motion="STRAIGHT",
            approach_level=None,
            approach_target_distance_m=None,
            target_priority_score=0.0,
            candidate_count=0,
            candidates=[],
            image_width=self.latest_image_width,
            image_height=self.latest_image_height,
            camera_info_ready=self.fx is not None,
            depth_age_sec=round(age, 3) if age is not None else None,
            note=note,
        )

    def _publish(self, info: HurdleInfo) -> None:
        message = String()
        payload = asdict(info)
        payload.update(self.confirmation_fields)
        payload.update(self.go_confirmation_fields)
        message.data = json.dumps(
            payload,
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self.publisher.publish(message)

    def _detections_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("detections JSON must be an object")
            detections = payload.get("detections", [])
            if not isinstance(detections, list):
                raise ValueError("detections must be a list")
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Invalid detections message: {exc}")
            return

        image_width, image_height = self._image_size(payload)
        candidates: list[HurdleCandidate] = []
        outside_tracking_range = False
        for detection in detections:
            if not isinstance(detection, dict):
                continue
            if str(detection.get("class_name", "")) != self.hurdle_class_name:
                continue
            candidate = self._build_candidate(
                detection,
                image_width,
                image_height,
            )
            if candidate is None:
                continue
            if not candidate.depth_valid or depth_is_within_range(
                candidate.depth_valid,
                candidate.depth_m,
                self.detect_depth_m,
            ):
                candidates.append(candidate)
            elif candidate.depth_valid and candidate.depth_m is not None:
                outside_tracking_range = True
        candidates.sort(key=lambda item: item.score, reverse=True)
        if not candidates:
            confirmation = self.confirmation_filter.update(False)
            go_confirmation = self.go_confirmation_filter.update(False)
            self.confirmation_fields = confirmation.as_dict()
            self.go_confirmation_fields = go_confirmation.as_dict(
                "go_confirmation"
            )
            if self.publish_empty_when_missing:
                note = (
                    "hurdle_outside_tracking_range"
                    if outside_tracking_range
                    else "no_hurdle_detection"
                )
                self._publish(self._empty_info(note))
            return

        target = candidates[0]
        confirmation = self.confirmation_filter.update(
            True,
            bbox=target.bbox,
            image_width=image_width,
            image_height=image_height,
        )
        self.confirmation_fields = confirmation.as_dict()
        if not confirmation.confirmed:
            go_confirmation = self.go_confirmation_filter.update(False)
            self.go_confirmation_fields = go_confirmation.as_dict(
                "go_confirmation"
            )
            if self.publish_empty_when_missing:
                self._publish(
                    self._empty_info("hurdle_confirmation_pending")
                )
            return

        state, parallel, in_range, error, go_now, note = self._state(target)
        raw_go_now = go_now
        go_confirmation = self.go_confirmation_filter.update(raw_go_now)
        self.go_confirmation_fields = go_confirmation.as_dict(
            "go_confirmation"
        )
        go_now = raw_go_now and go_confirmation.confirmed
        if raw_go_now and not go_now:
            note = "go_confirmation_pending"
        age = self._depth_age_sec()
        self._publish(
            HurdleInfo(
                detected=True,
                state=state,
                confidence=target.confidence,
                center_x=target.center[0],
                center_y=target.center[1],
                bbox=target.bbox,
                width_px=target.width_px,
                height_px=target.height_px,
                area_px=target.area_px,
                offset_x_px=target.offset_x_px,
                offset_y_px=target.offset_y_px,
                offset_x_norm=target.offset_x_norm,
                offset_y_norm=target.offset_y_norm,
                horizontal_direction=target.horizontal_direction,
                bearing_deg=target.bearing_deg,
                elevation_deg=target.elevation_deg,
                depth_m=target.depth_m,
                horizontal_distance_m=target.horizontal_distance_m,
                distance_m=target.distance_m,
                ground_gap_m=target.ground_gap_m,
                ground_distance_m=target.ground_distance_m,
                camera_bottom_gap_px=target.camera_bottom_gap_px,
                camera_bottom_gap_m=target.camera_bottom_gap_m,
                lateral_offset_m=target.lateral_offset_m,
                estimated_width_m=target.estimated_width_m,
                left_depth_m=target.left_depth_m,
                right_depth_m=target.right_depth_m,
                hurdle_angle_deg=target.hurdle_angle_deg,
                depth_valid=target.depth_valid,
                depth_sample_count=target.depth_sample_count,
                is_parallel=parallel,
                ground_gap_in_go_range=in_range,
                go_ground_gap_error_m=(
                    round(error, 3) if error is not None else None
                ),
                go_now=go_now,
                approach_motion=approach_motion_for_distance(
                    target.depth_m
                ),
                approach_level=approach_level_from_motion(
                    approach_motion_for_distance(target.depth_m)
                ),
                approach_target_distance_m=target.depth_m,
                target_priority_score=target.score,
                candidate_count=len(candidates),
                candidates=candidates[:5],
                image_width=image_width,
                image_height=image_height,
                camera_info_ready=self.fx is not None,
                depth_age_sec=round(age, 3) if age is not None else None,
                note=note,
            )
        )


def main(args: list[str] | None = None) -> None:
    """Run the ROS 2 hurdle analyzer node."""
    rclpy.init(args=args)
    node: HurdleAnalyzer | None = None
    try:
        node = HurdleAnalyzer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
