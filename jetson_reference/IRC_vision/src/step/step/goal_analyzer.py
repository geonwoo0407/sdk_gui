#!/usr/bin/env python3
"""Analyze YOLO goal detections and aligned depth for scoring logic."""

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
class GoalCandidate:
    """One goal candidate with image and camera-relative geometry."""

    confidence: float
    bbox: list[int]
    aim_source: str
    aim_bbox: list[int]
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
    distance_m: float | None
    ground_distance_m: float | None
    lateral_offset_m: float | None
    depth_valid: bool
    depth_sample_count: int
    score: float


@dataclass(frozen=True)
class GoalInfo:
    """Selected goal geometry and provisional scoring conditions."""

    detected: bool
    state: str
    confidence: float
    aim_source: str
    aim_bbox: list[int] | None
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
    distance_m: float | None
    ground_distance_m: float | None
    lateral_offset_m: float | None
    depth_valid: bool
    depth_sample_count: int
    is_centered: bool
    depth_in_score_range: bool
    score_depth_error_m: float | None
    score_now: bool
    approach_motion: str
    approach_level: int | None
    approach_target_distance_m: float | None
    target_priority_score: float
    candidate_count: int
    candidates: list[GoalCandidate]
    image_width: int | None
    image_height: int | None
    camera_info_ready: bool
    depth_age_sec: float | None
    note: str


class GoalAnalyzer(DepthFrameConsumer, Node):
    """Convert raw goal detections into an SDK-friendly scoring signal."""

    def __init__(self, depth_cache: DepthFrameCache | None = None) -> None:
        super().__init__("goal_analyzer")

        self.declare_parameter("detections_topic", "/vision/detections")
        self.declare_parameter(
            "depth_topic",
            "/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/camera/color/camera_info",
        )
        self.declare_parameter("output_topic", "/vision/goal_info")
        self.declare_parameter("goal_class_name", "goal")
        self.declare_parameter("backboard_class_name", "backboard")
        self.declare_parameter("prefer_backboard_center", True)
        self.declare_parameter("min_confidence", 0.55)
        self.declare_parameter("depth_timeout_sec", 0.7)
        self.declare_parameter("depth_window_px", 11)
        self.declare_parameter("max_valid_depth_m", 6.0)
        self.declare_parameter("detect_depth_m", 1.0)
        self.declare_parameter("approach_depth_m", 0.5)
        self.declare_parameter("score_target_depth_m", 0.25)
        self.declare_parameter("score_depth_tolerance_m", 0.05)
        self.declare_parameter("score_center_tolerance_norm", 0.10)
        self.declare_parameter("direction_deadband_norm", 0.04)
        self.declare_parameter("confirmation_window_size", 40)
        self.declare_parameter("confirmation_required_hits", 30)
        self.declare_parameter("confirmation_max_missed_frames", 2)
        self.declare_parameter("confirmation_max_center_shift_norm", 0.18)
        self.declare_parameter("confirmation_min_area_ratio", 0.40)
        self.declare_parameter("score_confirmation_window_size", 5)
        self.declare_parameter("score_confirmation_required_hits", 3)
        self.declare_parameter("publish_empty_when_missing", True)

        self.detections_topic = self._string_parameter("detections_topic")
        self.depth_topic = self._string_parameter("depth_topic")
        self.camera_info_topic = self._string_parameter(
            "camera_info_topic"
        )
        self.output_topic = self._string_parameter("output_topic")
        self.goal_class_name = self._string_parameter("goal_class_name")
        self.backboard_class_name = self._string_parameter(
            "backboard_class_name"
        )
        self.prefer_backboard_center = bool(
            self.get_parameter("prefer_backboard_center").value
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
        self.approach_depth_m = self._float_parameter("approach_depth_m")
        self.score_target_depth_m = self._float_parameter(
            "score_target_depth_m"
        )
        self.score_depth_tolerance_m = max(
            0.0,
            self._float_parameter("score_depth_tolerance_m"),
        )
        self.score_center_tolerance_norm = max(
            0.0,
            self._float_parameter("score_center_tolerance_norm"),
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
        self.score_confirmation_filter = TemporalConfirmationFilter(
            window_size=int(
                self.get_parameter("score_confirmation_window_size").value
            ),
            required_hits=int(
                self.get_parameter("score_confirmation_required_hits").value
            ),
            max_missed_frames=0,
            spatial_matching=False,
        )
        self.confirmation_fields: dict[str, object] = {}
        self.score_confirmation_fields: dict[str, object] = {}

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
        self.get_logger().info(f"Publishing goal info: {self.output_topic}")

    def _string_parameter(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _camera_info_callback(self, message: CameraInfo) -> None:
        """Store color-camera intrinsics for angle and 3-D projection."""
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
        """Store the latest aligned depth image."""
        try:
            if self.bridge is not None:
                self._store_depth_message(message, self.bridge)
        except Exception as exc:
            self.get_logger().warning(f"Could not read depth image: {exc}")

    def _depth_age_sec(self) -> float | None:
        if self.latest_depth_time is None:
            return None
        return time.monotonic() - self.latest_depth_time

    def _depth_is_fresh(self) -> bool:
        age = self._depth_age_sec()
        return age is not None and age <= self.depth_timeout_sec

    def _valid_patch_m(self, center_x: int, center_y: int) -> np.ndarray:
        """Return valid metric depth values around one image point."""
        if self.latest_depth_image is None:
            return np.empty(0, dtype=np.float32)
        depth = self.latest_depth_image
        height, width = depth.shape[:2]
        if not (0 <= center_x < width and 0 <= center_y < height):
            return np.empty(0, dtype=np.float32)
        radius = self.depth_window_px // 2
        patch = depth[
            max(0, center_y - radius):min(height, center_y + radius + 1),
            max(0, center_x - radius):min(width, center_x + radius + 1),
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

    def _sample_goal_depth_m(
        self,
        bbox: list[int],
    ) -> tuple[float | None, bool, int]:
        """Sample several goal regions to avoid an empty goal center."""
        if (
            self.latest_depth_image is None
            or not self._depth_is_fresh()
            or self.latest_depth_image.ndim != 2
        ):
            return None, False, 0
        left, top, right, bottom = bbox
        width = right - left
        height = bottom - top
        ratios = [
            (0.50, 0.50),
            (0.25, 0.35),
            (0.75, 0.35),
            (0.25, 0.65),
            (0.75, 0.65),
        ]
        medians: list[float] = []
        for x_ratio, y_ratio in ratios:
            values = self._valid_patch_m(
                int(round(left + width * x_ratio)),
                int(round(top + height * y_ratio)),
            )
            if values.size:
                medians.append(float(np.median(values)))
        if not medians:
            return None, False, 0
        return float(np.median(medians)), True, len(medians)

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

    def _project(
        self,
        center_x: int,
        center_y: int,
        depth_m: float | None,
    ) -> tuple[
        float | None,
        float | None,
        float | None,
        float | None,
        float | None,
    ]:
        if (
            self.fx is None
            or self.fy is None
            or self.cx is None
            or self.cy is None
        ):
            return None, None, None, None, None
        x_ratio = (center_x - self.cx) / self.fx
        y_ratio = (center_y - self.cy) / self.fy
        bearing = math.degrees(math.atan(x_ratio))
        elevation = math.degrees(math.atan(y_ratio))
        if depth_m is None:
            return bearing, elevation, None, None, None
        lateral = x_ratio * depth_m
        # GOAL already controls from raw Depth Z.  Keep legacy distance output
        # as a depth alias and do not calculate slant or floor distance.
        return bearing, elevation, lateral, depth_m, None

    def _build_candidate(
        self,
        detection: dict[str, Any],
        image_width: int | None,
        image_height: int | None,
        aim_detection: dict[str, Any] | None = None,
    ) -> GoalCandidate | None:
        try:
            confidence = float(detection.get("confidence", 0.0))
            bbox = [int(value) for value in detection.get("bbox", [])]
            goal_center = [
                int(value) for value in detection.get("center", [])
            ]
        except (TypeError, ValueError):
            return None
        if (
            confidence < self.min_confidence
            or len(bbox) != 4
            or len(goal_center) != 2
        ):
            return None
        left, top, right, bottom = bbox
        width_px = right - left
        height_px = bottom - top
        if width_px <= 0 or height_px <= 0:
            return None

        aim_source = "goal"
        aim_bbox = bbox
        center = goal_center
        if aim_detection is not None:
            try:
                candidate_aim_bbox = [
                    int(value)
                    for value in aim_detection.get("bbox", [])
                ]
                candidate_aim_center = [
                    int(value)
                    for value in aim_detection.get("center", [])
                ]
            except (TypeError, ValueError):
                candidate_aim_bbox = []
                candidate_aim_center = []
            if len(candidate_aim_bbox) == 4 and len(candidate_aim_center) == 2:
                aim_source = "backboard"
                aim_bbox = candidate_aim_bbox
                center = candidate_aim_center

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

        depth_m, depth_valid, sample_count = self._sample_goal_depth_m(
            aim_bbox
        )
        bearing, elevation, lateral, distance, ground_distance = self._project(
            center_x,
            center_y,
            depth_m,
        )
        if offset_x_norm < -self.direction_deadband_norm:
            direction = "LEFT"
        elif offset_x_norm > self.direction_deadband_norm:
            direction = "RIGHT"
        else:
            direction = "CENTER"

        area_px = width_px * height_px
        center_score = max(0.0, 1.0 - abs(offset_x_norm))
        depth_score = (
            max(0.0, 1.0 - depth_m / self.max_valid_depth_m)
            if depth_m is not None
            else 0.0
        )
        area_score = min(1.0, math.sqrt(area_px) / 300.0)
        priority = (
            confidence * 0.50
            + center_score * 0.25
            + depth_score * 0.15
            + area_score * 0.10
        )
        return GoalCandidate(
            confidence=round(confidence, 4),
            bbox=bbox,
            aim_source=aim_source,
            aim_bbox=aim_bbox,
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
            depth_m=round(depth_m, 3) if depth_m is not None else None,
            distance_m=(
                round(distance, 3) if distance is not None else None
            ),
            ground_distance_m=(
                round(ground_distance, 3)
                if ground_distance is not None
                else None
            ),
            lateral_offset_m=(
                round(lateral, 3) if lateral is not None else None
            ),
            depth_valid=depth_valid,
            depth_sample_count=sample_count,
            score=round(priority, 4),
        )

    def _goal_state(
        self,
        target: GoalCandidate,
    ) -> tuple[str, bool, bool, float | None, bool, str]:
        centered = (
            abs(target.offset_x_norm) <= self.score_center_tolerance_norm
        )
        if not target.depth_valid or target.depth_m is None:
            return (
                "NO_DEPTH",
                centered,
                False,
                None,
                False,
                "goal_detected_without_valid_depth",
            )
        error = target.depth_m - self.score_target_depth_m
        depth_in_range = abs(error) <= self.score_depth_tolerance_m
        score_now = depth_in_range and centered
        if score_now:
            return (
                "SCORE_READY",
                centered,
                depth_in_range,
                error,
                True,
                "goal_centered_at_scoring_depth",
            )
        if not centered:
            return (
                "ALIGN",
                centered,
                depth_in_range,
                error,
                False,
                "align_goal_horizontally",
            )
        if error > self.score_depth_tolerance_m:
            state = (
                "APPROACH"
                if target.depth_m <= self.approach_depth_m
                else "FAR"
            )
            return state, centered, False, error, False, "goal_too_far"
        return (
            "TOO_CLOSE",
            centered,
            False,
            error,
            False,
            "goal_too_close",
        )

    def _publish(self, info: GoalInfo) -> None:
        message = String()
        payload = asdict(info)
        payload.update(self.confirmation_fields)
        payload.update(self.score_confirmation_fields)
        message.data = json.dumps(
            payload,
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self.publisher.publish(message)

    def _empty_info(self, note: str = "no_goal_detection") -> GoalInfo:
        age = self._depth_age_sec()
        return GoalInfo(
            detected=False,
            state="SEARCH",
            confidence=0.0,
            aim_source="none",
            aim_bbox=None,
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
            distance_m=None,
            ground_distance_m=None,
            lateral_offset_m=None,
            depth_valid=False,
            depth_sample_count=0,
            is_centered=False,
            depth_in_score_range=False,
            score_depth_error_m=None,
            score_now=False,
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

    def _matching_backboard(
        self,
        goal_detection: dict[str, Any],
        backboards: list[dict[str, Any]],
    ) -> dict[str, Any] | None:
        """Return the strongest backboard whose center is inside the goal."""
        try:
            goal_bbox = [
                int(value) for value in goal_detection.get("bbox", [])
            ]
        except (TypeError, ValueError):
            return None
        if len(goal_bbox) != 4:
            return None
        left, top, right, bottom = goal_bbox
        matches: list[tuple[float, dict[str, Any]]] = []
        for backboard in backboards:
            try:
                confidence = float(backboard.get("confidence", 0.0))
                center = [
                    int(value)
                    for value in backboard.get("center", [])
                ]
            except (TypeError, ValueError):
                continue
            if confidence < self.min_confidence or len(center) != 2:
                continue
            if left <= center[0] <= right and top <= center[1] <= bottom:
                matches.append((confidence, backboard))
        if not matches:
            return None
        return max(matches, key=lambda item: item[0])[1]

    def _detections_callback(self, message: String) -> None:
        """Select and publish the strongest valid goal target."""
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("detections JSON must be an object")
            detections = payload.get("detections", [])
            if not isinstance(detections, list):
                raise ValueError("detections JSON must contain a list")
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Invalid detections message: {exc}")
            return

        image_width, image_height = self._image_size(payload)
        backboards = [
            detection
            for detection in detections
            if isinstance(detection, dict)
            and str(detection.get("class_name", ""))
            == self.backboard_class_name
        ]
        candidates: list[GoalCandidate] = []
        outside_tracking_range = False
        for detection in detections:
            if not isinstance(detection, dict):
                continue
            if str(detection.get("class_name", "")) != self.goal_class_name:
                continue
            aim_detection = (
                self._matching_backboard(detection, backboards)
                if self.prefer_backboard_center
                else None
            )
            candidate = self._build_candidate(
                detection,
                image_width,
                image_height,
                aim_detection,
            )
            if candidate is None:
                continue
            if depth_is_within_range(
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
            score_confirmation = self.score_confirmation_filter.update(False)
            self.confirmation_fields = confirmation.as_dict()
            self.score_confirmation_fields = score_confirmation.as_dict(
                "score_confirmation"
            )
            if self.publish_empty_when_missing:
                note = (
                    "goal_outside_tracking_range"
                    if outside_tracking_range
                    else "no_goal_detection"
                )
                self._publish(self._empty_info(note))
            return

        target = candidates[0]
        confirmation = self.confirmation_filter.update(
            True,
            bbox=target.aim_bbox,
            image_width=image_width,
            image_height=image_height,
        )
        self.confirmation_fields = confirmation.as_dict()
        if not confirmation.confirmed:
            score_confirmation = self.score_confirmation_filter.update(False)
            self.score_confirmation_fields = score_confirmation.as_dict(
                "score_confirmation"
            )
            if self.publish_empty_when_missing:
                self._publish(self._empty_info("goal_confirmation_pending"))
            return

        (
            state,
            centered,
            depth_in_range,
            depth_error,
            score_now,
            note,
        ) = self._goal_state(target)
        raw_score_now = score_now
        score_confirmation = self.score_confirmation_filter.update(
            raw_score_now
        )
        self.score_confirmation_fields = score_confirmation.as_dict(
            "score_confirmation"
        )
        score_now = raw_score_now and score_confirmation.confirmed
        if raw_score_now and not score_now:
            note = "score_confirmation_pending"
        age = self._depth_age_sec()
        self._publish(
            GoalInfo(
                detected=True,
                state=state,
                confidence=target.confidence,
                aim_source=target.aim_source,
                aim_bbox=target.aim_bbox,
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
                distance_m=target.distance_m,
                ground_distance_m=target.ground_distance_m,
                lateral_offset_m=target.lateral_offset_m,
                depth_valid=target.depth_valid,
                depth_sample_count=target.depth_sample_count,
                is_centered=centered,
                depth_in_score_range=depth_in_range,
                score_depth_error_m=(
                    round(depth_error, 3)
                    if depth_error is not None
                    else None
                ),
                score_now=score_now,
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
    """Run the ROS 2 goal analyzer node."""
    rclpy.init(args=args)
    node: GoalAnalyzer | None = None
    try:
        node = GoalAnalyzer()
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
