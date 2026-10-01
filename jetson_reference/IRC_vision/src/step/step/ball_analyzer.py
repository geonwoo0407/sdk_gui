#!/usr/bin/env python3
"""Analyze YOLO26 ball detections and RealSense depth for pickup logic."""

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
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .approach_distance import approach_level_from_motion
from .approach_distance import approach_motion_for_distance
from .depth_frame_cache import DepthFrame
from .depth_frame_cache import DepthFrameCache
from .depth_frame_cache import DepthFrameConsumer
from .temporal_confirmation import TemporalConfirmationFilter
from .yolo_line_analyzer import calibrated_robot_center_x


def ball_path_heading_deg(
    *,
    target_x: float,
    target_y: float,
    image_width: int,
    image_height: int,
    robot_center_offset_px: float,
) -> float | None:
    """Measure the ball vector from the calibrated bottom-center axis."""
    if image_width <= 0 or image_height <= 0:
        return None
    origin_x = calibrated_robot_center_x(
        image_width,
        robot_center_offset_px,
    )
    origin_y = float(image_height - 1)
    dx = float(target_x) - origin_x
    forward_dy = origin_y - float(target_y)
    return math.degrees(math.atan2(dx, forward_dy))


def ball_display_offsets_px(
    *,
    target_x: float,
    target_y: float,
    image_width: int,
    image_height: int,
) -> tuple[int, int] | None:
    """Return camera-center dx and image-bottom dy in RGB pixels."""
    if image_width <= 0 or image_height <= 0:
        return None
    camera_center_dx = int(round(float(target_x) - image_width / 2.0))
    bottom_dy = max(0, image_height - 1 - int(round(float(target_y))))
    return camera_center_dx, bottom_dy


def should_request_head_down(
    bottom_distance_px: int | None,
    trigger_distance_px: int,
) -> bool:
    """Return whether a confirmed ball has entered the bottom trigger."""
    return (
        bottom_distance_px is not None
        and bottom_distance_px <= max(0, trigger_distance_px)
    )


@dataclass(frozen=True)
class BallCandidate:
    """One ball candidate with image geometry and optional depth."""

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
    steering_angle_deg: float | None
    bearing_deg: float | None
    elevation_deg: float | None
    depth_m: float | None
    lateral_offset_m: float | None
    vertical_offset_m: float | None
    horizontal_distance_m: float | None
    ground_distance_m: float | None
    distance_m: float | None
    depth_valid: bool
    depth_source: str
    depth_sync_delta_ms: float | None
    depth_sample_count: int
    depth_width: int | None
    depth_height: int | None
    depth_stamp_ns: int | None
    depth_age_sec: float | None
    score: float


@dataclass(frozen=True)
class BallInfo:
    """Ball target information for future mission/behavior nodes."""

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
    camera_center_offset_x_px: int | None
    bottom_distance_px: int | None
    head_down_requested: bool
    horizontal_direction: str
    steering_angle_deg: float | None
    bearing_deg: float | None
    elevation_deg: float | None
    depth_m: float | None
    lateral_offset_m: float | None
    vertical_offset_m: float | None
    horizontal_distance_m: float | None
    ground_distance_m: float | None
    distance_m: float | None
    approach_motion: str
    approach_level: int | None
    approach_target_distance_m: float | None
    depth_valid: bool
    depth_source: str
    depth_sync_delta_ms: float | None
    depth_sample_count: int
    depth_width: int | None
    depth_height: int | None
    rgb_stamp_ns: int | None
    depth_stamp_ns: int | None
    is_centered: bool
    is_close: bool
    approach_ready: bool
    pickup_ready: bool
    pickup_now: bool
    is_in_pickup_window: bool
    pickup_target_x_ratio: float
    pickup_target_y_ratio: float
    pickup_x_tolerance_norm: float
    pickup_y_tolerance_ratio: float
    target_priority_score: float
    candidate_count: int
    candidates: list[BallCandidate]
    image_width: int | None
    image_height: int | None
    image_center_x: float | None
    image_center_y: float | None
    robot_center_x_px: float | None
    robot_center_offset_px: float
    camera_info_ready: bool
    depth_age_sec: float | None
    note: str


class BallAnalyzer(DepthFrameConsumer, Node):
    """Convert raw YOLO ball detections into stable ball target information.

    This node does not decide robot motion. The mission/behavior layer should
    still check the current mission phase before allowing a pickup command.
    That is what prevents a far Ball B from stealing priority while the robot
    is scoring at Goal A.
    """

    def __init__(self, depth_cache: DepthFrameCache | None = None) -> None:
        super().__init__("ball_analyzer")

        self.declare_parameter("detections_topic", "/vision/detections")
        self.declare_parameter(
            "depth_topic",
            "/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/camera/color/camera_info",
        )
        self.declare_parameter("output_topic", "/vision/ball_info")
        self.declare_parameter("ball_class_name", "ball")
        self.declare_parameter("min_confidence", 0.45)
        self.declare_parameter("depth_timeout_sec", 0.7)
        self.declare_parameter("depth_window_px", 9)
        self.declare_parameter("depth_bbox_inner_ratio", 0.70)
        self.declare_parameter("depth_min_valid_pixels", 5)
        self.declare_parameter("depth_hold_sec", 0.40)
        self.declare_parameter("max_rgb_depth_delta_sec", 0.05)
        self.declare_parameter("max_valid_depth_m", 4.0)
        self.declare_parameter("detect_depth_m", 1.5)
        self.declare_parameter("approach_depth_m", 0.9)
        self.declare_parameter("pickup_ready_depth_m", 0.15)
        self.declare_parameter("pickup_now_depth_m", 0.48)
        self.declare_parameter("pickup_depth_tolerance_m", 0.02)
        self.declare_parameter("pickup_center_tolerance_norm", 0.08)
        self.declare_parameter("pickup_target_y_ratio", 0.82)
        self.declare_parameter("pickup_y_tolerance_ratio", 0.12)
        self.declare_parameter("horizontal_deadband_px", 30)
        self.declare_parameter("center_tolerance_px", 140)
        self.declare_parameter("head_down_trigger_bottom_distance_px", 120)
        # Ball pickup uses its own calibrated robot axis. Keep this separate
        # from the line analyzer's robot_center_offset_px parameter.
        self.declare_parameter("ball_robot_center_offset_px", 96.0)
        # At the 30 FPS competition setting this requires about 0.4 seconds of
        # spatially consistent detection before ball_info becomes detected.
        self.declare_parameter("confirmation_window_size", 20)
        self.declare_parameter("confirmation_required_hits", 12)
        self.declare_parameter("confirmation_max_missed_frames", 10)
        self.declare_parameter("confirmation_max_center_shift_norm", 0.18)
        self.declare_parameter("confirmation_min_area_ratio", 0.40)
        self.declare_parameter("pickup_confirmation_window_size", 5)
        self.declare_parameter("pickup_confirmation_required_hits", 3)
        self.declare_parameter("publish_empty_when_missing", True)

        self.detections_topic = str(
            self.get_parameter("detections_topic").value
        )
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
        )
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.ball_class_name = str(
            self.get_parameter("ball_class_name").value
        )
        self.min_confidence = float(
            self.get_parameter("min_confidence").value
        )
        self.depth_timeout_sec = float(
            self.get_parameter("depth_timeout_sec").value
        )
        self.depth_window_px = max(
            3,
            int(self.get_parameter("depth_window_px").value),
        )
        if self.depth_window_px % 2 == 0:
            self.depth_window_px += 1
        self.depth_bbox_inner_ratio = min(
            1.0,
            max(
                0.1,
                float(
                    self.get_parameter("depth_bbox_inner_ratio").value
                ),
            ),
        )
        self.depth_min_valid_pixels = max(
            1,
            int(self.get_parameter("depth_min_valid_pixels").value),
        )
        self.depth_hold_sec = max(
            0.0,
            float(self.get_parameter("depth_hold_sec").value),
        )
        self.max_rgb_depth_delta_sec = max(
            0.0,
            float(
                self.get_parameter("max_rgb_depth_delta_sec").value
            ),
        )
        self.max_valid_depth_m = float(
            self.get_parameter("max_valid_depth_m").value
        )
        self.detect_depth_m = float(self.get_parameter("detect_depth_m").value)
        self.approach_depth_m = float(
            self.get_parameter("approach_depth_m").value
        )
        self.pickup_ready_depth_m = float(
            self.get_parameter("pickup_ready_depth_m").value
        )
        self.pickup_now_depth_m = float(
            self.get_parameter("pickup_now_depth_m").value
        )
        self.pickup_depth_tolerance_m = max(
            0.0,
            float(self.get_parameter("pickup_depth_tolerance_m").value),
        )
        self.pickup_center_tolerance_norm = max(
            0.0,
            float(
                self.get_parameter("pickup_center_tolerance_norm").value
            ),
        )
        self.pickup_target_y_ratio = float(
            self.get_parameter("pickup_target_y_ratio").value
        )
        self.pickup_y_tolerance_ratio = max(
            0.0,
            float(self.get_parameter("pickup_y_tolerance_ratio").value),
        )
        self.horizontal_deadband_px = max(
            0,
            int(self.get_parameter("horizontal_deadband_px").value),
        )
        self.center_tolerance_px = int(
            self.get_parameter("center_tolerance_px").value
        )
        self.head_down_trigger_bottom_distance_px = max(
            0,
            int(
                self.get_parameter(
                    "head_down_trigger_bottom_distance_px"
                ).value
            ),
        )
        self.robot_center_offset_px = float(
            self.get_parameter("ball_robot_center_offset_px").value
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
        self.pickup_confirmation_filter = TemporalConfirmationFilter(
            window_size=int(
                self.get_parameter(
                    "pickup_confirmation_window_size"
                ).value
            ),
            required_hits=int(
                self.get_parameter(
                    "pickup_confirmation_required_hits"
                ).value
            ),
            max_missed_frames=0,
            spatial_matching=False,
        )
        self.confirmation_fields: dict[str, object] = {}
        self.pickup_confirmation_fields: dict[str, object] = {}

        self.depth_cache = depth_cache or DepthFrameCache()
        self._owns_depth_subscription = depth_cache is None
        self._depth_lock = self.depth_cache.lock
        self.bridge = CvBridge() if self._owns_depth_subscription else None
        self.last_valid_ball_depth_m: float | None = None
        self.last_valid_ball_depth_time: float | None = None
        self.last_valid_ball_depth_center: tuple[int, int] | None = None
        self.last_valid_ball_depth_sample_count = 0
        self.last_valid_ball_depth_frame: DepthFrame | None = None
        self._active_depth_frame: DepthFrame | None = None
        self._active_depth_sync_delta_sec: float | None = None
        self._active_rgb_width: int | None = None
        self._active_rgb_height: int | None = None
        self._active_rgb_stamp_ns: int | None = None
        self._depth_sync_required = False
        self._last_depth_source = "invalid"
        self._last_depth_control_valid = False
        self._last_depth_sample_count = 0
        self._last_depth_width: int | None = None
        self._last_depth_height: int | None = None
        self._last_depth_stamp_ns: int | None = None
        self._last_depth_age_sec: float | None = None
        self._last_depth_sync_delta_sec: float | None = None
        self.fx: float | None = None
        self.fy: float | None = None
        self.cx: float | None = None
        self.cy: float | None = None

        # In the unified process, high-rate detection callbacks previously
        # starved this node's depth callback for roughly 0.7 seconds. Give the
        # three input streams independent groups and keep only their newest
        # queued samples so aligned depth cannot sit behind detection work.
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
            f"Subscribing camera info: {self.camera_info_topic}"
        )
        self.get_logger().info(f"Publishing ball info: {self.output_topic}")

    def _camera_info_callback(self, message: CameraInfo) -> None:
        """Store color-camera intrinsics used for angle and 3-D position."""
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
        with self._depth_lock:
            if self.latest_depth_time is None:
                return None
            return time.monotonic() - self.latest_depth_time

    def _depth_is_fresh(self) -> bool:
        age = self._depth_age_sec()
        return age is not None and age <= self.depth_timeout_sec

    def _sample_depth_m(
        self,
        center_x: int,
        center_y: int,
        bbox: list[int] | None = None,
    ) -> tuple[float | None, bool]:
        """Sample the RGB-matched depth frame and retain source metadata."""
        now = time.monotonic()
        frame = getattr(self, "_active_depth_frame", None)
        sync_delta = getattr(self, "_active_depth_sync_delta_sec", None)
        sync_required = bool(getattr(self, "_depth_sync_required", False))
        if frame is None and not sync_required:
            frame = self._get_depth_cache().latest()
        if (
            frame is None
            or now - frame.received_at > self.depth_timeout_sec
            or (
                sync_required
                and (
                    sync_delta is None
                    or sync_delta
                    > getattr(self, "max_rgb_depth_delta_sec", 0.05)
                )
            )
        ):
            return self._held_depth_sample(center_x, center_y, bbox, now)

        depth_image = frame.image
        if depth_image.ndim != 2:
            return self._held_depth_sample(center_x, center_y, bbox, now)

        depth_height, depth_width = depth_image.shape[:2]
        rgb_width = int(
            getattr(self, "_active_rgb_width", None) or depth_width
        )
        rgb_height = int(
            getattr(self, "_active_rgb_height", None) or depth_height
        )
        scale_x = depth_width / max(rgb_width, 1)
        scale_y = depth_height / max(rgb_height, 1)
        sample_x = int(round(center_x * scale_x))
        sample_y = int(round(center_y * scale_y))
        sample_x = max(0, min(depth_width - 1, sample_x))
        sample_y = max(0, min(depth_height - 1, sample_y))
        depth_bbox = (
            [
                int(round(bbox[0] * scale_x)),
                int(round(bbox[1] * scale_y)),
                int(round(bbox[2] * scale_x)),
                int(round(bbox[3] * scale_y)),
            ]
            if bbox is not None and len(bbox) == 4
            else None
        )

        radius = self.depth_window_px // 2
        regions = [
            (
                "center",
                max(0, sample_x - radius),
                max(0, sample_y - radius),
                min(depth_width, sample_x + radius + 1),
                min(depth_height, sample_y + radius + 1),
            )
        ]
        inner_bbox = self._inner_depth_bbox(
            depth_bbox,
            depth_width,
            depth_height,
        )
        if inner_bbox is not None:
            regions.append(("inner_roi", *inner_bbox))

        for source, x1, y1, x2, y2 in regions:
            depth, sample_count = self._median_valid_depth_sample(
                depth_image[y1:y2, x1:x2]
            )
            if depth is not None:
                with self._depth_lock:
                    self.last_valid_ball_depth_m = depth
                    self.last_valid_ball_depth_time = now
                    self.last_valid_ball_depth_center = (center_x, center_y)
                    self.last_valid_ball_depth_sample_count = sample_count
                    self.last_valid_ball_depth_frame = frame
                self._set_depth_metadata(
                    source=source,
                    control_valid=True,
                    sample_count=sample_count,
                    frame=frame,
                    sync_delta_sec=sync_delta,
                    age_sec=now - frame.received_at,
                )
                return depth, True

        return self._held_depth_sample(center_x, center_y, bbox, now)

    def _set_depth_metadata(
        self,
        *,
        source: str,
        control_valid: bool,
        sample_count: int,
        frame: DepthFrame | None,
        sync_delta_sec: float | None,
        age_sec: float | None,
    ) -> None:
        """Record metadata for the most recent candidate depth sample."""
        self._last_depth_source = source
        self._last_depth_control_valid = control_valid
        self._last_depth_sample_count = sample_count
        self._last_depth_width = frame.width if frame is not None else None
        self._last_depth_height = frame.height if frame is not None else None
        self._last_depth_stamp_ns = (
            frame.stamp_ns if frame is not None else None
        )
        self._last_depth_age_sec = age_sec
        self._last_depth_sync_delta_sec = sync_delta_sec

    def _held_depth_sample(
        self,
        center_x: int,
        center_y: int,
        bbox: list[int] | None,
        now: float,
    ) -> tuple[float | None, bool]:
        """Return held depth for display while making it control-invalid."""
        depth, available = self._held_ball_depth(
            center_x,
            center_y,
            bbox,
            now,
        )
        frame = getattr(self, "last_valid_ball_depth_frame", None)
        stamp = getattr(self, "last_valid_ball_depth_time", None)
        rgb_stamp_ns = getattr(self, "_active_rgb_stamp_ns", None)
        if (
            available
            and frame is not None
            and frame.stamp_ns is not None
            and rgb_stamp_ns is not None
        ):
            held_sync_delta_sec = (
                abs(frame.stamp_ns - rgb_stamp_ns) / 1_000_000_000.0
            )
        else:
            held_sync_delta_sec = getattr(
                self,
                "_active_depth_sync_delta_sec",
                None,
            )
        self._set_depth_metadata(
            source="held" if available else "invalid",
            control_valid=False,
            sample_count=(
                int(getattr(self, "last_valid_ball_depth_sample_count", 0))
                if available
                else 0
            ),
            frame=frame,
            sync_delta_sec=held_sync_delta_sec,
            age_sec=(now - stamp if available and stamp is not None else None),
        )
        return depth, available

    def _median_valid_depth(self, crop: np.ndarray) -> float | None:
        """Return robust depth from one ROI when enough pixels are valid."""
        depth, _ = self._median_valid_depth_sample(crop)
        return depth

    def _median_valid_depth_sample(
        self,
        crop: np.ndarray,
    ) -> tuple[float | None, int]:
        """Return median metric depth and the contributing pixel count."""
        if crop.size == 0:
            return None, 0
        if crop.dtype == np.uint16:
            crop_m = crop.astype(np.float32) * 0.001
        else:
            crop_m = crop.astype(np.float32)
        valid_mask = (
            np.isfinite(crop_m)
            & (crop_m > 0.05)
            & (crop_m <= self.max_valid_depth_m)
        )
        valid = crop_m[valid_mask]
        if valid.size < self.depth_min_valid_pixels:
            return None, int(valid.size)
        return float(np.median(valid)), int(valid.size)

    def _inner_depth_bbox(
        self,
        bbox: list[int] | None,
        image_width: int,
        image_height: int,
    ) -> tuple[int, int, int, int] | None:
        """Return a central ball-box ROI that avoids most background pixels."""
        if bbox is None or len(bbox) != 4:
            return None
        left, top, right, bottom = bbox
        if right <= left or bottom <= top:
            return None
        half_width = (right - left) * self.depth_bbox_inner_ratio / 2.0
        half_height = (bottom - top) * self.depth_bbox_inner_ratio / 2.0
        center_x = (left + right) / 2.0
        center_y = (top + bottom) / 2.0
        x1 = max(0, int(round(center_x - half_width)))
        x2 = min(image_width, int(round(center_x + half_width)) + 1)
        y1 = max(0, int(round(center_y - half_height)))
        y2 = min(image_height, int(round(center_y + half_height)) + 1)
        return (x1, y1, x2, y2) if x2 > x1 and y2 > y1 else None

    def _held_ball_depth(
        self,
        center_x: int,
        center_y: int,
        bbox: list[int] | None,
        now: float,
    ) -> tuple[float | None, bool]:
        """Briefly reuse a nearby valid sample across stereo depth holes."""
        with self._depth_lock:
            depth = self.last_valid_ball_depth_m
            stamp = self.last_valid_ball_depth_time
            old_center = self.last_valid_ball_depth_center
        if depth is None or stamp is None or old_center is None:
            return None, False
        if now - stamp > self.depth_hold_sec:
            return None, False
        if bbox is not None and len(bbox) == 4:
            bbox_span = max(bbox[2] - bbox[0], bbox[3] - bbox[1])
            gate_px = max(20.0, 0.75 * bbox_span)
        else:
            gate_px = 20.0
        center_shift = math.hypot(
            center_x - old_center[0],
            center_y - old_center[1],
        )
        return (depth, True) if center_shift <= gate_px else (None, False)

    def _image_size_from_payload(
        self,
        payload: dict[str, Any],
        detections: list[dict[str, Any]],
    ) -> tuple[int | None, int | None]:
        """Get the source image size without estimating it from one object."""
        try:
            payload_width = int(payload.get("image_width", 0))
            payload_height = int(payload.get("image_height", 0))
        except (TypeError, ValueError):
            payload_width = 0
            payload_height = 0
        if payload_width > 0 and payload_height > 0:
            return payload_width, payload_height

        if (
            self.latest_image_width is not None
            and self.latest_image_height is not None
        ):
            return self.latest_image_width, self.latest_image_height

        max_x = 0
        max_y = 0
        for detection in detections:
            bbox = detection.get("bbox", [])
            if isinstance(bbox, list) and len(bbox) == 4:
                max_x = max(max_x, int(bbox[2]))
                max_y = max(max_y, int(bbox[3]))
        if max_x > 0 and max_y > 0:
            return max_x + 1, max_y + 1
        return None, None

    @staticmethod
    def _closeness_score(distance_m: float | None, valid: bool) -> float:
        if not valid or distance_m is None:
            return 0.0
        return max(0.0, min(1.0, 1.0 - distance_m / 2.0))

    def _project_ball_position(
        self,
        center_x: int,
        center_y: int,
        depth_m: float | None,
        depth_valid: bool,
    ) -> tuple[
        float | None,
        float | None,
        float | None,
        float | None,
        float | None,
        float | None,
        float | None,
    ]:
        """Convert one image point to camera-relative angles and position.

        Camera coordinates follow the ROS optical convention: x is right,
        y is down, and z is forward. Angles are positive right/down.
        """
        if (
            self.fx is None
            or self.fy is None
            or self.cx is None
            or self.cy is None
        ):
            return None, None, None, None, None, None, None

        x_ratio = (center_x - self.cx) / self.fx
        y_ratio = (center_y - self.cy) / self.fy
        bearing_deg = math.degrees(math.atan(x_ratio))
        elevation_deg = math.degrees(math.atan(y_ratio))
        if not depth_valid or depth_m is None:
            return (
                bearing_deg,
                elevation_deg,
                None,
                None,
                None,
                None,
                None,
            )

        # Distance control intentionally uses raw aligned Depth Z only.  Keep
        # camera-coordinate X/Y as diagnostics, but do not derive a slant or
        # floor distance with Pythagoras.  The two legacy distance fields are
        # raw-depth aliases so older JSON consumers cannot accidentally keep
        # controlling from the removed horizontal-distance calculation.
        lateral_offset_m = x_ratio * depth_m
        vertical_offset_m = y_ratio * depth_m
        return (
            bearing_deg,
            elevation_deg,
            lateral_offset_m,
            vertical_offset_m,
            None,
            depth_m,
            depth_m,
        )

    def _build_candidate(
        self,
        detection: dict[str, Any],
        image_width: int | None,
        image_height: int | None,
    ) -> BallCandidate | None:
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
        width_px = max(0, right - left)
        height_px = max(0, bottom - top)
        if width_px <= 0 or height_px <= 0:
            return None

        center_x, center_y = center
        offset_x_px = 0
        offset_y_px = 0
        offset_x_norm = 0.0
        offset_y_norm = 0.0
        steering_angle_deg = None
        if image_width and image_height:
            robot_center_x = calibrated_robot_center_x(
                image_width,
                self.robot_center_offset_px,
            )
            offset_x_px = int(round(center_x - robot_center_x))
            offset_y_px = int(center_y - image_height / 2)
            offset_x_norm = offset_x_px / max(image_width / 2, 1.0)
            offset_y_norm = offset_y_px / max(image_height / 2, 1.0)
            steering_angle_deg = ball_path_heading_deg(
                target_x=center_x,
                target_y=center_y,
                image_width=image_width,
                image_height=image_height,
                robot_center_offset_px=self.robot_center_offset_px,
            )

        depth_m, depth_available = self._sample_depth_m(
            center_x,
            center_y,
            bbox,
        )
        depth_source = getattr(
            self,
            "_last_depth_source",
            "center" if depth_available else "invalid",
        )
        depth_valid = bool(
            getattr(
                self,
                "_last_depth_control_valid",
                depth_available,
            )
        )
        (
            bearing_deg,
            elevation_deg,
            lateral_offset_m,
            vertical_offset_m,
            horizontal_distance_m,
            ground_distance_m,
            distance_m,
        ) = self._project_ball_position(
            center_x,
            center_y,
            depth_m,
            depth_available,
        )
        if offset_x_px < -self.horizontal_deadband_px:
            horizontal_direction = "LEFT"
        elif offset_x_px > self.horizontal_deadband_px:
            horizontal_direction = "RIGHT"
        else:
            horizontal_direction = "CENTER"
        area_px = width_px * height_px
        center_score = max(0.0, 1.0 - abs(offset_x_norm))
        depth_score = self._closeness_score(distance_m, depth_valid)
        area_score = min(1.0, math.sqrt(area_px) / 180.0)
        score = (
            confidence * 0.45
            + center_score * 0.25
            + depth_score * 0.20
            + area_score * 0.10
        )

        return BallCandidate(
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
            horizontal_direction=horizontal_direction,
            steering_angle_deg=(
                round(steering_angle_deg, 3)
                if steering_angle_deg is not None
                else None
            ),
            bearing_deg=(
                round(bearing_deg, 3) if bearing_deg is not None else None
            ),
            elevation_deg=(
                round(elevation_deg, 3)
                if elevation_deg is not None
                else None
            ),
            depth_m=round(depth_m, 3) if depth_m is not None else None,
            lateral_offset_m=(
                round(lateral_offset_m, 3)
                if lateral_offset_m is not None
                else None
            ),
            vertical_offset_m=(
                round(vertical_offset_m, 3)
                if vertical_offset_m is not None
                else None
            ),
            horizontal_distance_m=(
                round(horizontal_distance_m, 3)
                if horizontal_distance_m is not None
                else None
            ),
            ground_distance_m=(
                round(ground_distance_m, 3)
                if ground_distance_m is not None
                else None
            ),
            distance_m=(
                round(distance_m, 3) if distance_m is not None else None
            ),
            depth_valid=depth_valid,
            depth_source=depth_source,
            depth_sync_delta_ms=(
                round(self._last_depth_sync_delta_sec * 1000.0, 3)
                if getattr(self, "_last_depth_sync_delta_sec", None)
                is not None
                else None
            ),
            depth_sample_count=int(
                getattr(self, "_last_depth_sample_count", 0)
            ),
            depth_width=getattr(self, "_last_depth_width", None),
            depth_height=getattr(self, "_last_depth_height", None),
            depth_stamp_ns=getattr(self, "_last_depth_stamp_ns", None),
            depth_age_sec=(
                round(self._last_depth_age_sec, 3)
                if getattr(self, "_last_depth_age_sec", None) is not None
                else None
            ),
            score=round(score, 4),
        )

    def _state_for_candidate(
        self,
        candidate: BallCandidate,
        image_height: int | None,
    ) -> tuple[str, bool, bool, bool, bool, bool, bool, str]:
        centered = abs(candidate.offset_x_px) <= self.center_tolerance_px
        close = (
            candidate.depth_valid
            and candidate.distance_m is not None
            and candidate.distance_m <= self.detect_depth_m
        )
        approach_ready = (
            candidate.depth_valid
            and candidate.distance_m is not None
            and candidate.distance_m <= self.approach_depth_m
        )

        in_pickup_window = False
        if image_height:
            center_y_ratio = candidate.center[1] / max(image_height, 1)
            in_pickup_window = (
                abs(candidate.offset_x_norm)
                <= self.pickup_center_tolerance_norm
                and abs(center_y_ratio - self.pickup_target_y_ratio)
                <= self.pickup_y_tolerance_ratio
            )

        pickup_ready = (
            candidate.depth_valid
            and candidate.distance_m is not None
            and candidate.distance_m <= self.pickup_ready_depth_m
            and centered
            and in_pickup_window
        )
        pickup_now = (
            candidate.depth_valid
            and candidate.distance_m is not None
            and candidate.distance_m
            <= self.pickup_now_depth_m + self.pickup_depth_tolerance_m
            and in_pickup_window
        )

        if pickup_now:
            return (
                "PICKUP_NOW",
                centered,
                close,
                approach_ready,
                pickup_ready,
                pickup_now,
                in_pickup_window,
                "close_centered_ball_ready_to_pick",
            )
        if pickup_ready:
            return (
                "PICKUP_READY",
                centered,
                close,
                approach_ready,
                pickup_ready,
                pickup_now,
                in_pickup_window,
                "slow_down_and_prepare_pickup",
            )
        if approach_ready:
            return (
                "APPROACH",
                centered,
                close,
                approach_ready,
                pickup_ready,
                pickup_now,
                in_pickup_window,
                "ball_close_enough_to_approach",
            )
        if close:
            return (
                "TRACK",
                centered,
                close,
                approach_ready,
                pickup_ready,
                pickup_now,
                in_pickup_window,
                "ball_visible_with_depth",
            )
        if candidate.depth_valid:
            return (
                "FAR",
                centered,
                close,
                approach_ready,
                pickup_ready,
                pickup_now,
                in_pickup_window,
                "ball_detected_but_far",
            )
        return (
            "NO_DEPTH",
            centered,
            close,
            approach_ready,
            pickup_ready,
            pickup_now,
            in_pickup_window,
            "ball_detected_without_valid_depth",
        )

    def _publish(self, info: BallInfo) -> None:
        message = String()
        payload = asdict(info)
        payload.update(self.confirmation_fields)
        payload.update(self.pickup_confirmation_fields)
        message.data = json.dumps(
            payload,
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self.publisher.publish(message)

    @staticmethod
    def _payload_stamp_ns(payload: dict[str, Any]) -> int | None:
        """Read the detector's original RGB timestamp."""
        stamp = payload.get("stamp")
        if not isinstance(stamp, dict):
            return None
        try:
            sec = int(stamp.get("sec"))
            nanosec = int(stamp.get("nanosec"))
        except (TypeError, ValueError):
            return None
        if sec < 0 or not 0 <= nanosec < 1_000_000_000:
            return None
        return sec * 1_000_000_000 + nanosec

    def _empty_info(self, note: str = "no_ball_detection") -> BallInfo:
        age = self._depth_age_sec()
        return BallInfo(
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
            camera_center_offset_x_px=None,
            bottom_distance_px=None,
            head_down_requested=False,
            horizontal_direction="UNKNOWN",
            steering_angle_deg=None,
            bearing_deg=None,
            elevation_deg=None,
            depth_m=None,
            lateral_offset_m=None,
            vertical_offset_m=None,
            horizontal_distance_m=None,
            ground_distance_m=None,
            distance_m=None,
            approach_motion="STRAIGHT",
            approach_level=None,
            approach_target_distance_m=None,
            depth_valid=False,
            depth_source="invalid",
            depth_sync_delta_ms=None,
            depth_sample_count=0,
            depth_width=self.latest_image_width,
            depth_height=self.latest_image_height,
            rgb_stamp_ns=getattr(self, "_active_rgb_stamp_ns", None),
            depth_stamp_ns=None,
            is_centered=False,
            is_close=False,
            approach_ready=False,
            pickup_ready=False,
            pickup_now=False,
            is_in_pickup_window=False,
            pickup_target_x_ratio=(
                calibrated_robot_center_x(
                    self.latest_image_width,
                    self.robot_center_offset_px,
                )
                / max(self.latest_image_width, 1)
                if self.latest_image_width is not None
                else 0.5
            ),
            pickup_target_y_ratio=self.pickup_target_y_ratio,
            pickup_x_tolerance_norm=self.pickup_center_tolerance_norm,
            pickup_y_tolerance_ratio=self.pickup_y_tolerance_ratio,
            target_priority_score=0.0,
            candidate_count=0,
            candidates=[],
            image_width=self.latest_image_width,
            image_height=self.latest_image_height,
            image_center_x=(
                self.latest_image_width / 2.0
                if self.latest_image_width is not None
                else None
            ),
            image_center_y=(
                self.latest_image_height / 2.0
                if self.latest_image_height is not None
                else None
            ),
            robot_center_x_px=(
                calibrated_robot_center_x(
                    self.latest_image_width,
                    self.robot_center_offset_px,
                )
                if self.latest_image_width is not None
                else None
            ),
            robot_center_offset_px=self.robot_center_offset_px,
            camera_info_ready=self.fx is not None,
            depth_age_sec=round(age, 3) if age is not None else None,
            note=note,
        )

    def _detections_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            detections = payload.get("detections", [])
            if not isinstance(detections, list):
                raise ValueError("detections must be a list")
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                "Invalid detections message: "
                f"{type(exc).__name__}: {exc}"
            )
            return

        image_width, image_height = self._image_size_from_payload(
            payload,
            detections,
        )
        rgb_stamp_ns = self._payload_stamp_ns(payload)
        if rgb_stamp_ns is None:
            depth_frame = self._get_depth_cache().latest()
            sync_delta_sec = None
        else:
            depth_frame, sync_delta_sec = self._get_depth_cache().nearest(
                rgb_stamp_ns
            )
        self._active_depth_frame = depth_frame
        self._active_depth_sync_delta_sec = sync_delta_sec
        self._active_rgb_width = image_width
        self._active_rgb_height = image_height
        self._active_rgb_stamp_ns = rgb_stamp_ns
        self._depth_sync_required = rgb_stamp_ns is not None
        candidates: list[BallCandidate] = []
        for detection in detections:
            if not isinstance(detection, dict):
                continue
            if str(detection.get("class_name", "")) != self.ball_class_name:
                continue
            candidate = self._build_candidate(
                detection,
                image_width,
                image_height,
            )
            if candidate is None:
                continue
            # RGB detection owns object visibility and temporal confirmation.
            # Depth is optional metadata used later for FAR/NO_DEPTH states and
            # the motion planner's control-range gate.  A missing or distant
            # depth sample must not turn a visible YOLO ball into "not seen".
            candidates.append(candidate)

        candidates.sort(key=lambda item: item.score, reverse=True)
        if not candidates:
            confirmation = self.confirmation_filter.update(False)
            pickup_confirmation = self.pickup_confirmation_filter.update(
                False
            )
            self.confirmation_fields = confirmation.as_dict()
            self.pickup_confirmation_fields = pickup_confirmation.as_dict(
                "pickup_confirmation"
            )
            if self.publish_empty_when_missing:
                self._publish(self._empty_info("no_ball_detection"))
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
            pickup_confirmation = self.pickup_confirmation_filter.update(
                False
            )
            self.pickup_confirmation_fields = pickup_confirmation.as_dict(
                "pickup_confirmation"
            )
            if self.publish_empty_when_missing:
                self._publish(self._empty_info("ball_confirmation_pending"))
            return

        (
            state,
            centered,
            close,
            approach_ready,
            pickup_ready,
            pickup_now,
            in_pickup_window,
            note,
        ) = self._state_for_candidate(target, image_height)
        raw_pickup_now = pickup_now
        pickup_confirmation = self.pickup_confirmation_filter.update(
            raw_pickup_now
        )
        self.pickup_confirmation_fields = pickup_confirmation.as_dict(
            "pickup_confirmation"
        )
        pickup_now = raw_pickup_now and pickup_confirmation.confirmed
        if raw_pickup_now and not pickup_now:
            note = "pickup_confirmation_pending"
        display_offsets = (
            ball_display_offsets_px(
                target_x=target.center[0],
                target_y=target.center[1],
                image_width=image_width,
                image_height=image_height,
            )
            if image_width is not None and image_height is not None
            else None
        )
        self._publish(
            BallInfo(
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
                camera_center_offset_x_px=(
                    display_offsets[0] if display_offsets is not None else None
                ),
                bottom_distance_px=(
                    display_offsets[1] if display_offsets is not None else None
                ),
                head_down_requested=should_request_head_down(
                    display_offsets[1] if display_offsets is not None else None,
                    self.head_down_trigger_bottom_distance_px,
                ),
                horizontal_direction=target.horizontal_direction,
                steering_angle_deg=target.steering_angle_deg,
                bearing_deg=target.bearing_deg,
                elevation_deg=target.elevation_deg,
                depth_m=target.depth_m,
                lateral_offset_m=target.lateral_offset_m,
                vertical_offset_m=target.vertical_offset_m,
                horizontal_distance_m=target.horizontal_distance_m,
                ground_distance_m=target.ground_distance_m,
                distance_m=target.distance_m,
                approach_motion=approach_motion_for_distance(
                    target.distance_m
                ),
                approach_level=approach_level_from_motion(
                    approach_motion_for_distance(target.distance_m)
                ),
                approach_target_distance_m=target.distance_m,
                depth_valid=target.depth_valid,
                depth_source=target.depth_source,
                depth_sync_delta_ms=target.depth_sync_delta_ms,
                depth_sample_count=target.depth_sample_count,
                depth_width=target.depth_width,
                depth_height=target.depth_height,
                rgb_stamp_ns=rgb_stamp_ns,
                depth_stamp_ns=target.depth_stamp_ns,
                is_centered=centered,
                is_close=close,
                approach_ready=approach_ready,
                pickup_ready=pickup_ready,
                pickup_now=pickup_now,
                is_in_pickup_window=in_pickup_window,
                pickup_target_x_ratio=(
                    calibrated_robot_center_x(
                        image_width,
                        self.robot_center_offset_px,
                    )
                    / max(image_width, 1)
                    if image_width is not None
                    else 0.5
                ),
                pickup_target_y_ratio=self.pickup_target_y_ratio,
                pickup_x_tolerance_norm=(
                    self.pickup_center_tolerance_norm
                ),
                pickup_y_tolerance_ratio=self.pickup_y_tolerance_ratio,
                target_priority_score=target.score,
                candidate_count=len(candidates),
                candidates=candidates[:5],
                image_width=image_width,
                image_height=image_height,
                image_center_x=(
                    image_width / 2.0 if image_width is not None else None
                ),
                image_center_y=(
                    image_height / 2.0 if image_height is not None else None
                ),
                robot_center_x_px=(
                    calibrated_robot_center_x(
                        image_width,
                        self.robot_center_offset_px,
                    )
                    if image_width is not None
                    else None
                ),
                robot_center_offset_px=self.robot_center_offset_px,
                camera_info_ready=self.fx is not None,
                depth_age_sec=target.depth_age_sec,
                note=note,
            )
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: BallAnalyzer | None = None
    executor: MultiThreadedExecutor | None = None
    try:
        node = BallAnalyzer()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
