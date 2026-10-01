#!/usr/bin/env python3
"""
YOLO26 line geometry analyzer for ROS 2.

Input
-----
/vision/detections
    std_msgs/msg/String
    JSON output from yolo26_detector.

Output
------
/vision/line_info
    std_msgs/msg/String
    JSON containing line geometry and filtered navigation information.

Design principle
----------------
This node does NOT decide robot motion.

It only provides condition information such as:

- Whether the line is detected
- YOLO line center points
- Lateral offset from image center
- Immediate heading error
- Near/far path heading
- Segment angles
- Turn strength
- Turn consistency
- Detection quality
- Geometry quality
- Temporally filtered heading and lateral offset

The algorithm side should use these values to decide:

- Straight motion
- Left/right correction
- Line recovery
- Corner handling
- Other motion commands
"""

from __future__ import annotations

import json
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
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
from .temporal_confirmation import TemporalConfirmationFilter


# ============================================================
# Data classes
# ============================================================


@dataclass(frozen=True)
class LinePoint:
    """
    One YOLO line detection center.

    Attributes
    ----------
    x:
        Image x coordinate in pixels.

    y:
        Image y coordinate in pixels.

    confidence:
        YOLO confidence score.
    """

    x: float
    y: float
    confidence: float


@dataclass(frozen=True)
class FittedLine:
    """
    Image-space fitted path.

    Model
    -----
        x = slope * y + intercept

    Image coordinate convention
    ---------------------------
        +x -> image right
        +y -> image down

    angle_deg convention
    --------------------
        Negative -> path goes toward image left as it gets farther
        Zero     -> straight ahead
        Positive -> path goes toward image right as it gets farther
    """

    slope: float
    intercept: float
    angle_deg: float
    rmse_px: float
    point_count: int
    y_span_px: float


def calibrated_robot_center_x(
    image_width: int,
    center_offset_px: float,
) -> float:
    """Shift the image midpoint to the robot's calibrated center."""
    if image_width <= 0:
        return 0.0
    center_x = image_width / 2.0 + center_offset_px
    return float(np.clip(center_x, 0.0, image_width - 1.0))


def ground_forward_distance_from_depth(
    *,
    x_px: float,
    y_px: float,
    depth_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    camera_pitch_down_deg: float,
    camera_height_m: float,
    reference_height_m: float = 0.0,
    camera_forward_offset_m: float = 0.0,
) -> tuple[float, float]:
    """Project aligned depth into height-corrected floor coordinates."""
    lateral_m = (float(x_px) - float(cx)) * float(depth_m) / float(fx)
    camera_down_m = (
        (float(y_px) - float(cy)) * float(depth_m) / float(fy)
    )
    slant_distance_m = math.sqrt(
        lateral_m * lateral_m
        + camera_down_m * camera_down_m
        + float(depth_m) * float(depth_m)
    )
    vertical_separation_m = abs(
        float(camera_height_m) - float(reference_height_m)
    )
    ground_radius_m = math.sqrt(
        max(
            slant_distance_m * slant_distance_m
            - vertical_separation_m * vertical_separation_m,
            0.0,
        )
    )

    # Pitch only determines the front/back sign. The distance magnitude is
    # corrected with the measured camera height and Pythagoras.
    pitch_rad = math.radians(float(camera_pitch_down_deg))
    rotated_forward_m = (
        float(depth_m) * math.cos(pitch_rad)
        - camera_down_m * math.sin(pitch_rad)
    )
    forward_from_camera_m = math.copysign(
        math.sqrt(
            max(
                ground_radius_m * ground_radius_m
                - lateral_m * lateral_m,
                0.0,
            )
        ),
        rotated_forward_m,
    )
    forward_from_robot_m = (
        forward_from_camera_m + float(camera_forward_offset_m)
    )
    return lateral_m, max(0.0, forward_from_robot_m)


# ============================================================
# Main ROS 2 node
# ============================================================


class YoloLineAnalyzer(DepthFrameConsumer, Node):
    """
    Analyze YOLO26 line detections and publish navigation geometry.

    This node does not run YOLO inference again.

    Processing chain
    ----------------
    /vision/detections
        ->
    Extract class "line"
        ->
    Get bbox centers
        ->
    Remove duplicates
        ->
    Sort near to far
        ->
    Calculate reliable segment angles
        ->
    Estimate immediate heading
        ->
    Calculate lateral offset
        ->
    Analyze turn geometry
        ->
    Median + EMA temporal filtering
        ->
    /vision/line_info
    """

    def __init__(self, depth_cache: DepthFrameCache | None = None) -> None:
        super().__init__("yolo_line_analyzer")

        # ====================================================
        # ROS topic parameters
        # ====================================================

        self.declare_parameter(
            "detections_topic",
            "/vision/detections",
        )

        self.declare_parameter(
            "image_topic",
            "/camera/color/image_raw",
        )

        self.declare_parameter(
            "depth_topic",
            "/camera/aligned_depth_to_color/image_raw",
        )

        self.declare_parameter(
            "camera_info_topic",
            "/camera/color/camera_info",
        )

        self.declare_parameter(
            "output_topic",
            "/vision/line_info",
        )

        self.declare_parameter("depth_timeout_sec", 0.7)
        self.declare_parameter("depth_window_px", 9)
        self.declare_parameter("max_valid_depth_m", 4.0)
        self.declare_parameter("robot_center_offset_px", 70.0)
        self.declare_parameter("camera_height_m", 0.515)
        self.declare_parameter("camera_pitch_down_deg", 45.0)
        self.declare_parameter("camera_forward_offset_m", 0.0)

        # ====================================================
        # Detection filtering
        # ====================================================

        self.declare_parameter(
            "line_class_name",
            "line",
        )

        self.declare_parameter(
            "min_confidence",
            0.40,
        )

        self.declare_parameter(
            "min_bbox_width_px",
            3,
        )

        self.declare_parameter(
            "min_bbox_height_px",
            3,
        )

        # ====================================================
        # Region of interest
        # ====================================================

        self.declare_parameter(
            "roi_x_min_ratio",
            0.15,
        )

        self.declare_parameter(
            "roi_x_max_ratio",
            0.85,
        )

        self.declare_parameter(
            "roi_y_min_ratio",
            0.0,
        )

        self.declare_parameter(
            "roi_y_max_ratio",
            1.0,
        )

        # ====================================================
        # Point filtering
        # ====================================================

        self.declare_parameter(
            "min_point_distance_px",
            10.0,
        )

        self.declare_parameter(
            "max_points",
            30,
        )

        # ====================================================
        # Path continuity filtering
        # ====================================================

        self.declare_parameter(
            "path_continuity_filter_enabled",
            True,
        )

        self.declare_parameter(
            "min_path_vertical_progress_px",
            12.0,
        )

        self.declare_parameter(
            "max_path_jump_px",
            320.0,
        )

        self.declare_parameter(
            "max_path_dx_px",
            240.0,
        )

        self.declare_parameter(
            "max_path_angle_change_deg",
            55.0,
        )

        self.declare_parameter(
            "max_path_lateral_to_vertical_ratio",
            2.2,
        )

        self.declare_parameter(
            "min_path_continuity_score",
            0.50,
        )

        # ====================================================
        # Segment filtering
        # ====================================================

        # Very small vertical differences are dangerous because
        # perspective can cause image-space angles to become
        # extremely exaggerated.
        self.declare_parameter(
            "min_segment_dy_px",
            20.0,
        )

        # ====================================================
        # Immediate heading estimation
        # ====================================================

        # Number of nearest reliable segments used to calculate
        # the immediate heading.
        self.declare_parameter(
            "heading_segment_count",
            3,
        )

        # Weight decay:
        #
        # segment 1 -> 1.0
        # segment 2 -> 0.35
        # segment 3 -> 0.1225
        self.declare_parameter(
            "heading_weight_decay",
            0.65,
        )

        # Heading standard deviation regarded as poor stability.
        self.declare_parameter(
            "bad_heading_std_deg",
            25.0,
        )

        # ====================================================
        # Robust fitting
        # ====================================================

        self.declare_parameter(
            "min_points_for_fit",
            2,
        )

        self.declare_parameter(
            "min_fit_y_span_px",
            80.0,
        )

        self.declare_parameter(
            "outlier_residual_px",
            50.0,
        )

        self.declare_parameter(
            "max_fit_iterations",
            3,
        )

        # ====================================================
        # Near / far path regions
        # ====================================================

        self.declare_parameter(
            "near_region_ratio",
            0.55,
        )

        self.declare_parameter(
            "far_region_ratio",
            0.45,
        )

        # ====================================================
        # Lateral offset
        # ====================================================

        self.declare_parameter(
            "offset_eval_y_ratio",
            0.82,
        )

        # ====================================================
        # Confirmed corner preview
        # ====================================================

        self.declare_parameter("corner_min_points", 3)
        self.declare_parameter("corner_min_segment_length_px", 20.0)
        self.declare_parameter("corner_straight_max_turn_delta_deg", 15.0)
        self.declare_parameter("corner_min_turn_delta_deg", 30.0)
        self.declare_parameter("corner_onset_deviation_deg", 15.0)
        self.declare_parameter("corner_min_consistent_segments", 1)
        self.declare_parameter("corner_min_consistency", 0.75)
        self.declare_parameter("corner_confirmation_window_size", 5)
        self.declare_parameter("corner_confirmation_required_hits", 3)
        self.declare_parameter("corner_hold_sec", 0.30)
        self.declare_parameter("corner_distance_filter_size", 5)
        self.declare_parameter("corner_straight_motion_distance_m", 0.05)
        self.declare_parameter("corner_turn_margin_m", 0.15)

        # ====================================================
        # Quality calculation
        # ====================================================

        self.declare_parameter(
            "good_rmse_px",
            12.0,
        )

        self.declare_parameter(
            "bad_rmse_px",
            80.0,
        )

        # ====================================================
        # Temporal filtering
        # ====================================================

        # Median filter history.
        #
        # A window of 3 removes a single-frame spike well:
        #
        # [5, 31, 6] -> median = 6
        self.declare_parameter(
            "filter_window_size",
            3,
        )

        # EMA alpha.
        #
        # Higher:
        #   Faster response, less smoothing.
        #
        # Lower:
        #   Slower response, more smoothing.
        self.declare_parameter(
            "filter_ema_alpha",
            0.35,
        )

        # Reset temporal history after this many consecutive
        # invalid line frames.
        self.declare_parameter(
            "filter_reset_missed_frames",
            5,
        )

        self.declare_parameter(
            "confirmation_window_size",
            3,
        )

        self.declare_parameter(
            "confirmation_required_hits",
            2,
        )

        self.declare_parameter(
            "confirmation_max_missed_frames",
            2,
        )

        # ====================================================
        # Read parameters
        # ====================================================

        self.line_class_name = str(
            self.get_parameter(
                "line_class_name"
            ).value
        )

        self.min_confidence = float(
            self.get_parameter(
                "min_confidence"
            ).value
        )

        self.min_bbox_width_px = int(
            self.get_parameter(
                "min_bbox_width_px"
            ).value
        )

        self.min_bbox_height_px = int(
            self.get_parameter(
                "min_bbox_height_px"
            ).value
        )

        self.roi_x_min_ratio = float(
            self.get_parameter(
                "roi_x_min_ratio"
            ).value
        )

        self.roi_x_max_ratio = float(
            self.get_parameter(
                "roi_x_max_ratio"
            ).value
        )

        self.roi_y_min_ratio = float(
            self.get_parameter(
                "roi_y_min_ratio"
            ).value
        )

        self.roi_y_max_ratio = float(
            self.get_parameter(
                "roi_y_max_ratio"
            ).value
        )

        self.min_point_distance_px = float(
            self.get_parameter(
                "min_point_distance_px"
            ).value
        )

        self.max_points = int(
            self.get_parameter(
                "max_points"
            ).value
        )

        self.path_continuity_filter_enabled = bool(
            self.get_parameter(
                "path_continuity_filter_enabled"
            ).value
        )

        self.min_path_vertical_progress_px = float(
            self.get_parameter(
                "min_path_vertical_progress_px"
            ).value
        )

        self.max_path_jump_px = float(
            self.get_parameter(
                "max_path_jump_px"
            ).value
        )

        self.max_path_dx_px = float(
            self.get_parameter(
                "max_path_dx_px"
            ).value
        )

        self.max_path_angle_change_deg = float(
            self.get_parameter(
                "max_path_angle_change_deg"
            ).value
        )

        self.max_path_lateral_to_vertical_ratio = float(
            self.get_parameter(
                "max_path_lateral_to_vertical_ratio"
            ).value
        )

        self.min_path_continuity_score = float(
            np.clip(
                float(
                    self.get_parameter(
                        "min_path_continuity_score"
                    ).value
                ),
                0.0,
                1.0,
            )
        )

        self.min_segment_dy_px = float(
            self.get_parameter(
                "min_segment_dy_px"
            ).value
        )

        self.heading_segment_count = int(
            self.get_parameter(
                "heading_segment_count"
            ).value
        )

        self.heading_weight_decay = float(
            self.get_parameter(
                "heading_weight_decay"
            ).value
        )

        self.bad_heading_std_deg = float(
            self.get_parameter(
                "bad_heading_std_deg"
            ).value
        )

        self.min_points_for_fit = int(
            self.get_parameter(
                "min_points_for_fit"
            ).value
        )

        self.min_fit_y_span_px = float(
            self.get_parameter(
                "min_fit_y_span_px"
            ).value
        )

        self.outlier_residual_px = float(
            self.get_parameter(
                "outlier_residual_px"
            ).value
        )

        self.max_fit_iterations = int(
            self.get_parameter(
                "max_fit_iterations"
            ).value
        )

        self.near_region_ratio = float(
            self.get_parameter(
                "near_region_ratio"
            ).value
        )

        self.far_region_ratio = float(
            self.get_parameter(
                "far_region_ratio"
            ).value
        )

        self.offset_eval_y_ratio = float(
            self.get_parameter(
                "offset_eval_y_ratio"
            ).value
        )

        self.corner_min_points = max(
            3,
            int(self.get_parameter("corner_min_points").value),
        )
        self.corner_min_segment_length_px = max(
            1.0,
            float(
                self.get_parameter("corner_min_segment_length_px").value
            ),
        )
        self.corner_straight_max_turn_delta_deg = max(
            0.0,
            float(
                self.get_parameter(
                    "corner_straight_max_turn_delta_deg"
                ).value
            ),
        )
        self.corner_min_turn_delta_deg = max(
            0.0,
            float(self.get_parameter("corner_min_turn_delta_deg").value),
        )
        self.corner_onset_deviation_deg = max(
            0.0,
            float(self.get_parameter("corner_onset_deviation_deg").value),
        )
        self.corner_min_consistent_segments = max(
            1,
            int(
                self.get_parameter(
                    "corner_min_consistent_segments"
                ).value
            ),
        )
        self.corner_min_consistency = float(
            np.clip(
                float(
                    self.get_parameter("corner_min_consistency").value
                ),
                0.0,
                1.0,
            )
        )
        self.corner_straight_motion_distance_m = max(
            0.001,
            float(
                self.get_parameter(
                    "corner_straight_motion_distance_m"
                ).value
            ),
        )
        self.corner_turn_margin_m = max(
            0.0,
            float(self.get_parameter("corner_turn_margin_m").value),
        )

        self.good_rmse_px = float(
            self.get_parameter(
                "good_rmse_px"
            ).value
        )

        self.bad_rmse_px = float(
            self.get_parameter(
                "bad_rmse_px"
            ).value
        )

        self.filter_window_size = max(
            1,
            int(
                self.get_parameter(
                    "filter_window_size"
                ).value
            ),
        )

        self.filter_ema_alpha = float(
            np.clip(
                float(
                    self.get_parameter(
                        "filter_ema_alpha"
                    ).value
                ),
                0.0,
                1.0,
            )
        )

        self.filter_reset_missed_frames = max(
            1,
            int(
                self.get_parameter(
                    "filter_reset_missed_frames"
                ).value
            ),
        )

        self.confirmation_filter = TemporalConfirmationFilter(
            window_size=int(
                self.get_parameter(
                    "confirmation_window_size"
                ).value
            ),
            required_hits=int(
                self.get_parameter(
                    "confirmation_required_hits"
                ).value
            ),
            max_missed_frames=int(
                self.get_parameter(
                    "confirmation_max_missed_frames"
                ).value
            ),
            spatial_matching=False,
        )
        corner_confirmation_window_size = max(
            1,
            int(
                self.get_parameter(
                    "corner_confirmation_window_size"
                ).value
            ),
        )
        corner_confirmation_required_hits = max(
            1,
            int(
                self.get_parameter(
                    "corner_confirmation_required_hits"
                ).value
            ),
        )
        self.corner_confirmation_filter = TemporalConfirmationFilter(
            window_size=corner_confirmation_window_size,
            required_hits=corner_confirmation_required_hits,
            max_missed_frames=corner_confirmation_window_size,
            spatial_matching=False,
        )
        self.corner_hold_sec = max(
            0.0,
            float(self.get_parameter("corner_hold_sec").value),
        )
        corner_distance_filter_size = max(
            1,
            int(self.get_parameter("corner_distance_filter_size").value),
        )
        self.corner_candidate_direction: str | None = None
        self.corner_distance_history: deque[float] = deque(
            maxlen=corner_distance_filter_size
        )
        self.last_confirmed_corner: dict[str, Any] | None = None
        self.last_confirmed_corner_time: float | None = None

        # ====================================================
        # Camera resolution
        # ====================================================

        # Default until first Image message arrives.
        self.image_width = 1280
        self.image_height = 720
        self.robot_center_offset_px = float(
            self.get_parameter("robot_center_offset_px").value
        )

        self.depth_cache = depth_cache or DepthFrameCache()
        self._owns_depth_subscription = depth_cache is None
        self._depth_lock = self.depth_cache.lock
        self.bridge = CvBridge() if self._owns_depth_subscription else None
        self.fx: float | None = None
        self.fy: float | None = None
        self.cx: float | None = None
        self.cy: float | None = None
        self.depth_timeout_sec = max(
            0.0,
            float(self.get_parameter("depth_timeout_sec").value),
        )
        self.depth_window_px = max(
            3,
            int(self.get_parameter("depth_window_px").value),
        )
        if self.depth_window_px % 2 == 0:
            self.depth_window_px += 1
        self.max_valid_depth_m = max(
            0.05,
            float(self.get_parameter("max_valid_depth_m").value),
        )
        self.camera_height_m = max(
            0.0,
            float(self.get_parameter("camera_height_m").value),
        )
        self.camera_pitch_down_deg = float(
            self.get_parameter("camera_pitch_down_deg").value
        )
        self.camera_forward_offset_m = float(
            self.get_parameter("camera_forward_offset_m").value
        )

        # ====================================================
        # Temporal filter state
        # ====================================================

        self.heading_history: deque[float] = deque(
            maxlen=self.filter_window_size
        )

        self.offset_history: deque[float] = deque(
            maxlen=self.filter_window_size
        )

        self.filtered_heading: float | None = None
        self.filtered_offset: float | None = None

        self.missed_line_frames = 0

        # ====================================================
        # ROS topic names
        # ====================================================

        detections_topic = str(
            self.get_parameter(
                "detections_topic"
            ).value
        )

        depth_topic = str(self.get_parameter("depth_topic").value)
        camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
        )

        output_topic = str(
            self.get_parameter(
                "output_topic"
            ).value
        )

        # ====================================================
        # ROS publisher / subscribers
        # ====================================================

        self.publisher = self.create_publisher(
            String,
            output_topic,
            10,
        )

        self._detection_callback_group = MutuallyExclusiveCallbackGroup()
        self._depth_callback_group = MutuallyExclusiveCallbackGroup()
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
        self.detection_subscription = self.create_subscription(
            String,
            detections_topic,
            self._detections_callback,
            latest_detection_qos,
            callback_group=self._detection_callback_group,
        )
        if self._owns_depth_subscription:
            self.depth_subscription = self.create_subscription(
                Image,
                depth_topic,
                self._depth_callback,
                latest_depth_qos,
                callback_group=self._depth_callback_group,
            )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._camera_info_callback,
            10,
        )

        # ====================================================
        # Startup log
        # ====================================================

        self.get_logger().info(
            f"Subscribing detections: {detections_topic}"
        )

        if self._owns_depth_subscription:
            self.get_logger().info(
                f"Subscribing aligned depth: {depth_topic}"
            )

        self.get_logger().info(
            f"Subscribing camera info: {camera_info_topic}"
        )

        self.get_logger().info(
            f"Publishing line info: {output_topic}"
        )

        self.get_logger().info(
            "Robot center calibration: image midpoint "
            f"{self.robot_center_offset_px:+.1f}px"
        )

        self.get_logger().info(
            "Temporal filter: "
            f"window={self.filter_window_size}, "
            f"ema_alpha={self.filter_ema_alpha:.3f}, "
            "reset_after_missed_frames="
            f"{self.filter_reset_missed_frames}"
        )

    # ========================================================
    # ROS callbacks
    # ========================================================

    def _camera_info_callback(self, message: CameraInfo) -> None:
        """Store color-camera intrinsics for floor-forward projection."""
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
        """Store the latest depth image aligned to the color frame."""
        try:
            if self.bridge is not None:
                self._store_depth_message(message, self.bridge)
        except Exception as exc:
            self.get_logger().warning(f"Could not read depth image: {exc}")

    def _line_point_distance(
        self,
        point: LinePoint,
    ) -> dict[str, Any]:
        """Measure aligned-depth distance at any line point."""
        empty = {
            "point_px": [
                int(round(point.x)),
                int(round(point.y)),
            ],
            "depth_m": None,
            "lateral_offset_m": None,
            "ground_forward_distance_m": None,
            "horizontal_distance_m": None,
            "depth_valid": False,
        }
        if (
            self.latest_depth_image is None
            or self.latest_depth_time is None
            or time.monotonic() - self.latest_depth_time
            > self.depth_timeout_sec
            or self.fx is None
            or self.fy is None
            or self.cx is None
            or self.cy is None
        ):
            return empty

        depth_image = self.latest_depth_image
        if depth_image.ndim != 2:
            return empty
        height, width = depth_image.shape[:2]
        x = int(round(point.x))
        y = int(round(point.y))
        if not (0 <= x < width and 0 <= y < height):
            return empty

        radius = self.depth_window_px // 2
        crop = depth_image[
            max(0, y - radius):min(height, y + radius + 1),
            max(0, x - radius):min(width, x + radius + 1),
        ]
        if crop.dtype == np.uint16:
            crop_m = crop.astype(np.float32) * 0.001
        else:
            crop_m = crop.astype(np.float32)
        valid = crop_m[
            np.isfinite(crop_m)
            & (crop_m > 0.05)
            & (crop_m <= self.max_valid_depth_m)
        ]
        if valid.size == 0:
            return empty

        depth_m = float(np.median(valid))
        lateral_m, ground_forward_m = ground_forward_distance_from_depth(
            x_px=float(x),
            y_px=float(y),
            depth_m=depth_m,
            fx=self.fx,
            fy=self.fy,
            cx=self.cx,
            cy=self.cy,
            camera_pitch_down_deg=self.camera_pitch_down_deg,
            camera_height_m=self.camera_height_m,
            reference_height_m=0.0,
            camera_forward_offset_m=self.camera_forward_offset_m,
        )
        return {
            **empty,
            "depth_m": round(depth_m, 4),
            "lateral_offset_m": round(lateral_m, 4),
            "ground_forward_distance_m": round(ground_forward_m, 4),
            # Backward-compatible alias.  It now means robot-frame floor
            # forward distance rather than camera X-Z slant distance.
            "horizontal_distance_m": round(ground_forward_m, 4),
            "depth_valid": True,
        }

    def _nearest_line_distance(
        self,
        point: LinePoint,
    ) -> dict[str, Any]:
        """Measure ground-plane distance to the nearest line point."""
        measured = self._line_point_distance(point)
        return {
            "nearest_line_point_px": measured["point_px"],
            "nearest_line_depth_m": measured["depth_m"],
            "nearest_line_lateral_offset_m": measured[
                "lateral_offset_m"
            ],
            "nearest_line_ground_forward_distance_m": measured[
                "ground_forward_distance_m"
            ],
            "nearest_line_horizontal_distance_m": measured[
                "horizontal_distance_m"
            ],
            "nearest_line_depth_valid": measured["depth_valid"],
        }

    def _analyze_corner_preview(
        self,
        points: list[LinePoint],
    ) -> dict[str, Any]:
        """Confirm a real corner and estimate straight motions to its onset."""
        now = time.monotonic()
        geometry = self._detect_corner_start_geometry(
            points,
            min_points=self.corner_min_points,
            min_segment_length_px=self.corner_min_segment_length_px,
            straight_max_turn_delta_deg=(
                self.corner_straight_max_turn_delta_deg
            ),
            min_turn_delta_deg=self.corner_min_turn_delta_deg,
            onset_deviation_deg=self.corner_onset_deviation_deg,
            min_consistent_segments=self.corner_min_consistent_segments,
            min_consistency=self.corner_min_consistency,
        )
        state = str(geometry.get("state", "UNKNOWN"))
        direction = geometry["direction"]
        start_point = geometry["start_point"]

        if not hasattr(self, "corner_distance_history"):
            self.corner_distance_history = deque(maxlen=5)
        if not hasattr(self, "last_confirmed_corner"):
            self.last_confirmed_corner = None
            self.last_confirmed_corner_time = None
        if not hasattr(self, "corner_hold_sec"):
            self.corner_hold_sec = 0.30

        if state == "STRAIGHT":
            self.corner_confirmation_filter.reset()
            self.corner_candidate_direction = None
            self.corner_distance_history.clear()
            self.last_confirmed_corner = None
            self.last_confirmed_corner_time = None
        elif direction is not None:
            if (
                self.corner_candidate_direction is not None
                and direction != self.corner_candidate_direction
            ):
                self.corner_confirmation_filter.reset()
                self.corner_distance_history.clear()
                self.last_confirmed_corner = None
                self.last_confirmed_corner_time = None
            self.corner_candidate_direction = direction

        measured = {
            "point_px": None,
            "depth_m": None,
            "lateral_offset_m": None,
            "ground_forward_distance_m": None,
            "horizontal_distance_m": None,
            "depth_valid": False,
        }
        if isinstance(start_point, LinePoint):
            measured = self._line_point_distance(start_point)

        raw_ready = bool(
            geometry["detected"]
            and measured["depth_valid"]
        )
        measured_distance = measured["horizontal_distance_m"]
        if raw_ready and isinstance(measured_distance, (int, float)):
            self.corner_distance_history.append(float(measured_distance))
        confirmation = self.corner_confirmation_filter.update(raw_ready)
        confirmed = confirmation.confirmed
        held = False
        hold_age_sec: float | None = None
        effective_geometry = geometry
        effective_measured = measured

        if confirmed:
            if self.corner_distance_history:
                filtered_distance = float(
                    np.median(list(self.corner_distance_history))
                )
                effective_measured = dict(measured)
                effective_measured["ground_forward_distance_m"] = round(
                    filtered_distance,
                    4,
                )
                effective_measured["horizontal_distance_m"] = round(
                    filtered_distance,
                    4,
                )
            self.last_confirmed_corner = {
                "geometry": dict(geometry),
                "measured": dict(effective_measured),
            }
            self.last_confirmed_corner_time = now
        elif (
            state != "STRAIGHT"
            and self.last_confirmed_corner is not None
            and self.last_confirmed_corner_time is not None
            and (
                direction is None
                or direction
                == self.last_confirmed_corner["geometry"].get("direction")
            )
        ):
            hold_age_sec = max(0.0, now - self.last_confirmed_corner_time)
            if hold_age_sec <= self.corner_hold_sec:
                held = True
                confirmed = True
                effective_geometry = dict(
                    self.last_confirmed_corner["geometry"]
                )
                effective_measured = dict(
                    self.last_confirmed_corner["measured"]
                )
            else:
                self.last_confirmed_corner = None
                self.last_confirmed_corner_time = None

        straight_motion_count: int | None = None
        remaining_forward_m: float | None = None
        depth_m = effective_measured["depth_m"]
        ground_forward_m = effective_measured["horizontal_distance_m"]
        if confirmed and isinstance(ground_forward_m, (int, float)):
            remaining_forward_m = max(
                0.0,
                float(ground_forward_m) - self.corner_turn_margin_m,
            )
            straight_motion_count = int(
                math.floor(
                    remaining_forward_m
                    / self.corner_straight_motion_distance_m
                )
            )

        corner_approach_motion = (
            approach_motion_for_distance(ground_forward_m)
            if confirmed
            else None
        )
        corner_approach_level = (
            approach_level_from_motion(corner_approach_motion)
            if corner_approach_motion is not None
            else None
        )

        return {
            "corner_preview_raw_detected": bool(geometry["detected"]),
            "corner_preview_confirmed": confirmed,
            "corner_preview_state": state,
            "corner_preview_held": held,
            "corner_hold_age_sec": (
                round(hold_age_sec, 3)
                if held and hold_age_sec is not None
                else None
            ),
            "corner_direction": (
                effective_geometry["direction"] if confirmed else None
            ),
            "corner_start_index": effective_geometry["start_index"],
            "corner_start_point_px": effective_measured["point_px"],
            "corner_start_depth_m": depth_m,
            "corner_start_lateral_offset_m": effective_measured[
                "lateral_offset_m"
            ],
            "corner_start_ground_forward_distance_m": ground_forward_m,
            "corner_start_distance_m": ground_forward_m,
            "corner_start_depth_valid": bool(
                effective_measured["depth_valid"]
            ),
            "corner_turn_delta_deg": effective_geometry["turn_delta_deg"],
            "corner_turn_consistency": effective_geometry["consistency"],
            "corner_remaining_forward_m": (
                round(remaining_forward_m, 4)
                if remaining_forward_m is not None
                else None
            ),
            "corner_straight_motion_count": straight_motion_count,
            "corner_approach_motion": corner_approach_motion,
            "corner_approach_level": corner_approach_level,
            **confirmation.as_dict("corner_confirmation"),
        }

    def _detections_callback(
        self,
        message: String,
    ) -> None:
        """Process one YOLO detection message."""

        started = time.perf_counter()

        try:
            payload = json.loads(
                message.data
            )

            try:
                image_width = int(payload.get("image_width", 0))
                image_height = int(payload.get("image_height", 0))
            except (TypeError, ValueError):
                image_width = 0
                image_height = 0
            if image_width > 0 and image_height > 0:
                self.image_width = image_width
                self.image_height = image_height

            detections = payload.get(
                "detections",
                [],
            )

            if not isinstance(
                detections,
                list,
            ):
                raise ValueError(
                    "'detections' must be a list"
                )

            points = (
                self._extract_line_points(
                    detections
                )
            )

            raw_line_point_count = len(
                points
            )

            points = (
                self._remove_near_duplicates(
                    points
                )
            )

            deduplicated_line_point_count = len(
                points
            )

            (
                points,
                continuity_debug,
            ) = self._filter_path_continuity(
                points
            )

            result = self._analyze_line(
                points
            )

            raw_detected = bool(
                result.get(
                    "detected",
                    False,
                )
            )

            confirmation = (
                self.confirmation_filter.update(
                    raw_detected,
                )
            )

            result.update(
                confirmation.as_dict()
            )

            result["detected"] = bool(
                raw_detected
                and confirmation.confirmed
            )

            if (
                raw_detected
                and not confirmation.confirmed
            ):
                result["confirmation_note"] = (
                    "line_confirmation_pending"
                )

            result.update(
                {
                    "raw_line_point_count":
                        raw_line_point_count,

                    "deduplicated_line_point_count":
                        deduplicated_line_point_count,

                    **continuity_debug,
                }
            )

            result["processing_ms"] = round(
                (
                    time.perf_counter()
                    - started
                )
                * 1000.0,
                3,
            )

            output = String()

            output.data = json.dumps(
                result,
                ensure_ascii=True,
                separators=(",", ":"),
            )

            self.publisher.publish(
                output
            )

        except (
            json.JSONDecodeError,
            TypeError,
            ValueError,
            KeyError,
        ) as exc:
            self.get_logger().warning(
                "Invalid detection message: "
                f"{type(exc).__name__}: {exc}"
            )

        except Exception as exc:
            self.get_logger().error(
                "Line analysis failed: "
                f"{type(exc).__name__}: {exc}"
            )

    # ========================================================
    # Detection extraction
    # ========================================================

    def _extract_line_points(
        self,
        detections: list[dict[str, Any]],
    ) -> list[LinePoint]:
        """
        Extract valid class='line' centers from YOLO detections.
        """

        width = self.image_width
        height = self.image_height

        roi_left = (
            width
            * self.roi_x_min_ratio
        )

        roi_right = (
            width
            * self.roi_x_max_ratio
        )

        roi_top = (
            height
            * self.roi_y_min_ratio
        )

        roi_bottom = (
            height
            * self.roi_y_max_ratio
        )

        points: list[LinePoint] = []

        for detection in detections:
            if (
                detection.get("class_name")
                != self.line_class_name
            ):
                continue

            confidence = float(
                detection.get(
                    "confidence",
                    0.0,
                )
            )

            if (
                confidence
                < self.min_confidence
            ):
                continue

            bbox = detection.get(
                "bbox"
            )

            center = detection.get(
                "center"
            )

            if (
                not isinstance(bbox, list)
                or len(bbox) != 4
            ):
                continue

            if (
                not isinstance(center, list)
                or len(center) != 2
            ):
                continue

            left, top, right, bottom = (
                float(value)
                for value in bbox
            )

            center_x, center_y = (
                float(value)
                for value in center
            )

            bbox_width = (
                right - left
            )

            bbox_height = (
                bottom - top
            )

            if (
                bbox_width
                < self.min_bbox_width_px
            ):
                continue

            if (
                bbox_height
                < self.min_bbox_height_px
            ):
                continue

            if not (
                roi_left
                <= center_x
                <= roi_right
            ):
                continue

            if not (
                roi_top
                <= center_y
                <= roi_bottom
            ):
                continue

            points.append(
                LinePoint(
                    x=center_x,
                    y=center_y,
                    confidence=confidence,
                )
            )

        # Near -> far.
        #
        # In image coordinates:
        # larger y is generally closer to the robot.
        points.sort(
            key=lambda point: point.y,
            reverse=True,
        )

        return points[
            : self.max_points
        ]

    # ========================================================
    # Duplicate filtering
    # ========================================================

    def _remove_near_duplicates(
        self,
        points: list[LinePoint],
    ) -> list[LinePoint]:
        """
        Remove detections whose centers are extremely close.

        Higher-confidence detections are preferred.
        """

        if len(points) <= 1:
            return points

        by_confidence = sorted(
            points,
            key=lambda point:
                point.confidence,
            reverse=True,
        )

        accepted: list[LinePoint] = []

        for candidate in by_confidence:
            is_duplicate = False

            for existing in accepted:
                distance = math.hypot(
                    candidate.x
                    - existing.x,
                    candidate.y
                    - existing.y,
                )

                if (
                    distance
                    < self.min_point_distance_px
                ):
                    is_duplicate = True
                    break

            if not is_duplicate:
                accepted.append(
                    candidate
                )

        accepted.sort(
            key=lambda point: point.y,
            reverse=True,
        )

        return accepted

    # ========================================================
    # Path continuity filtering
    # ========================================================

    @staticmethod
    def _limit_score(
        value: float,
        good_limit: float,
        bad_limit: float,
    ) -> float:
        """Score 1.0 near the good limit and 0.0 past bad limit."""

        if bad_limit <= good_limit:
            return 1.0 if value <= good_limit else 0.0

        if value <= good_limit:
            return 1.0

        if value >= bad_limit:
            return 0.0

        return float(
            1.0
            - (
                value
                - good_limit
            )
            / (
                bad_limit
                - good_limit
            )
        )

    def _path_segment_angle(
        self,
        near_point: LinePoint,
        far_point: LinePoint,
    ) -> float:
        """Return image-space angle from a near point to a far point."""

        dx = (
            far_point.x
            - near_point.x
        )

        dy = max(
            near_point.y
            - far_point.y,
            1e-6,
        )

        return math.degrees(
            math.atan2(
                dx,
                dy,
            )
        )

    def _calculate_continuity_score(
        self,
        previous_point: LinePoint,
        candidate: LinePoint,
        previous_angle: float | None,
    ) -> tuple[
        float,
        float,
    ]:
        """Score whether candidate continues the current path."""

        dx = abs(
            candidate.x
            - previous_point.x
        )

        dy = (
            previous_point.y
            - candidate.y
        )

        distance = math.hypot(
            dx,
            dy,
        )

        angle = self._path_segment_angle(
            previous_point,
            candidate,
        )

        lateral_to_vertical_ratio = (
            dx
            / max(
                dy,
                1e-6,
            )
        )

        distance_score = self._limit_score(
            distance,
            self.max_path_jump_px * 0.65,
            self.max_path_jump_px,
        )

        lateral_score = self._limit_score(
            dx,
            self.max_path_dx_px * 0.65,
            self.max_path_dx_px,
        )

        vertical_score = float(
            np.clip(
                dy
                / max(
                    self.min_path_vertical_progress_px,
                    1e-6,
                ),
                0.0,
                1.0,
            )
        )

        ratio_score = self._limit_score(
            lateral_to_vertical_ratio,
            self.max_path_lateral_to_vertical_ratio * 0.65,
            self.max_path_lateral_to_vertical_ratio,
        )

        if previous_angle is None:
            angle_score = 1.0

        else:
            angle_change = abs(
                angle
                - previous_angle
            )

            angle_score = self._limit_score(
                angle_change,
                self.max_path_angle_change_deg,
                90.0,
            )

        continuity_score = (
            angle_score * 0.30
            + distance_score * 0.20
            + lateral_score * 0.20
            + vertical_score * 0.15
            + ratio_score * 0.15
        )

        # A sudden near-horizontal bridge to a far candidate is
        # usually an isolated YOLO line detection, not the real path.
        if (
            previous_angle is not None
            and dx > self.max_path_dx_px * 0.35
            and abs(angle) > 70.0
            and abs(
                angle
                - previous_angle
            ) > 25.0
        ):
            continuity_score = min(
                continuity_score,
                0.25,
            )

        return (
            float(
                np.clip(
                    continuity_score,
                    0.0,
                    1.0,
                )
            ),
            angle,
        )

    def _filter_path_continuity(
        self,
        points: list[LinePoint],
    ) -> tuple[
        list[LinePoint],
        dict[str, Any],
    ]:
        """
        Remove isolated line points that do not continue the path.

        The input is already sorted near -> far. The filter keeps
        gradual turns, but rejects sudden lateral jumps that would
        corrupt far-heading and turn analysis.
        """

        debug: dict[str, Any] = {
            "path_continuity_filter_enabled":
                bool(
                    self.path_continuity_filter_enabled
                ),

            "path_filtered_line_point_count":
                len(points),

            "path_rejected_point_count":
                0,

            "path_rejected_points_px":
                [],
        }

        if (
            not self.path_continuity_filter_enabled
            or len(points) <= 2
        ):
            return (
                points,
                debug,
            )

        accepted: list[LinePoint] = [
            points[0]
        ]

        rejected: list[LinePoint] = []

        previous_angle: float | None = None

        remaining = list(
            points[1:]
        )

        while remaining:
            scored_candidates: list[
                tuple[float, float, int, LinePoint]
            ] = []

            for index, candidate in enumerate(
                remaining
            ):
                if (
                    accepted[-1].y
                    - candidate.y
                ) <= 0.0:
                    continue

                (
                    continuity_score,
                    angle,
                ) = self._calculate_continuity_score(
                    accepted[-1],
                    candidate,
                    previous_angle,
                )

                scored_candidates.append(
                    (
                        continuity_score,
                        angle,
                        index,
                        candidate,
                    )
                )

            if not scored_candidates:
                rejected.extend(
                    remaining
                )
                break

            (
                best_score,
                best_angle,
                best_index,
                best_candidate,
            ) = max(
                scored_candidates,
                key=lambda item: item[0],
            )

            if (
                best_score
                < self.min_path_continuity_score
            ):
                rejected.extend(
                    remaining
                )
                break

            rejected.extend(
                remaining[:best_index]
            )

            accepted.append(
                best_candidate
            )

            previous_angle = best_angle

            remaining = remaining[
                best_index + 1:
            ]

        debug.update(
            {
                "path_filtered_line_point_count":
                    len(accepted),

                "path_rejected_point_count":
                    len(rejected),

                "path_rejected_points_px":
                    [
                        [
                            int(
                                round(
                                    point.x
                                )
                            ),
                            int(
                                round(
                                    point.y
                                )
                            ),
                        ]
                        for point in rejected
                    ],
            }
        )

        return (
            accepted,
            debug,
        )

    # ========================================================
    # Segment geometry
    # ========================================================

    def _calculate_segment_angles(
        self,
        points: list[LinePoint],
    ) -> tuple[
        list[float],
        list[float],
    ]:
        """
        Calculate reliable angles between consecutive centers.

        A segment is rejected when its vertical image spacing is
        smaller than min_segment_dy_px.

        This protects against perspective-induced angle explosions.

        Example
        -------
        Two distant points:

            dx = 54 px
            dy = 15 px

        Image angle becomes about 74 degrees, even though the
        real ground path may not actually turn that sharply.

        Returns
        -------
        segment_angles:
            Accepted segment angles in degrees.

        segment_vertical_spans:
            dy for every accepted segment.
        """

        angles: list[float] = []

        vertical_spans: list[float] = []

        if len(points) < 2:
            return (
                angles,
                vertical_spans,
            )

        for near_point, far_point in zip(
            points[:-1],
            points[1:],
        ):
            dx = (
                far_point.x
                - near_point.x
            )

            dy = (
                near_point.y
                - far_point.y
            )

            if (
                dy
                < self.min_segment_dy_px
            ):
                continue

            angle_deg = math.degrees(
                math.atan2(
                    dx,
                    dy,
                )
            )

            angles.append(
                round(
                    angle_deg,
                    3,
                )
            )

            vertical_spans.append(
                float(dy)
            )

        return (
            angles,
            vertical_spans,
        )

    # ========================================================
    # Immediate heading
    # ========================================================

    def _calculate_immediate_heading(
        self,
        segment_angles: list[float],
        segment_vertical_spans: list[float],
    ) -> dict[str, float | int | None]:
        """
        Estimate immediate path heading from nearest reliable segments.

        The nearest reliable segment receives the largest weight.

        By default:

            segment 1 weight = 1.0
            segment 2 weight = 0.35

        Vertical span is also considered because a larger dy tends
        to provide more stable image-space geometry.
        """

        if not segment_angles:
            return {
                "heading_deg": None,
                "segment_count_used": 0,
                "stability_deg": None,
                "quality": 0.0,
            }

        count = min(
            len(segment_angles),
            max(
                1,
                self.heading_segment_count,
            ),
        )

        selected_angles = np.asarray(
            segment_angles[:count],
            dtype=np.float64,
        )

        selected_spans = np.asarray(
            segment_vertical_spans[:count],
            dtype=np.float64,
        )

        distance_weights = np.asarray(
            [
                self.heading_weight_decay
                ** index
                for index in range(count)
            ],
            dtype=np.float64,
        )

        maximum_span = float(
            np.max(
                selected_spans
            )
        )

        if maximum_span > 1e-9:
            span_weights = (
                selected_spans
                / maximum_span
            )

        else:
            span_weights = np.ones(
                count,
                dtype=np.float64,
            )

        weights = (
            distance_weights
            * span_weights
        )

        weight_sum = float(
            np.sum(
                weights
            )
        )

        if weight_sum <= 1e-9:
            weights = np.ones(
                count,
                dtype=np.float64,
            )

            weight_sum = float(
                count
            )

        heading_deg = float(
            np.sum(
                selected_angles
                * weights
            )
            / weight_sum
        )

        variance = float(
            np.sum(
                weights
                * np.square(
                    selected_angles
                    - heading_deg
                )
            )
            / weight_sum
        )

        stability_deg = math.sqrt(
            max(
                variance,
                0.0,
            )
        )

        count_score = min(
            count
            / max(
                1,
                self.heading_segment_count,
            ),
            1.0,
        )

        if (
            self.bad_heading_std_deg
            <= 1e-9
        ):
            stability_score = 1.0

        else:
            stability_score = (
                1.0
                - min(
                    stability_deg
                    / self.bad_heading_std_deg,
                    1.0,
                )
            )

        quality = (
            count_score * 0.40
            + stability_score * 0.60
        )

        return {
            "heading_deg":
                heading_deg,

            "segment_count_used":
                count,

            "stability_deg":
                stability_deg,

            "quality":
                float(
                    np.clip(
                        quality,
                        0.0,
                        1.0,
                    )
                ),
        }

    # ========================================================
    # Robust line fitting
    # ========================================================

    def _robust_fit(
        self,
        points: list[LinePoint],
        require_y_span: bool = True,
    ) -> FittedLine | None:
        """
        Confidence-weighted robust fit:

            x = slope * y + intercept

        Large-residual outliers are iteratively removed.
        """

        if (
            len(points)
            < self.min_points_for_fit
        ):
            return None

        initial_y_span = (
            max(
                point.y
                for point in points
            )
            - min(
                point.y
                for point in points
            )
        )

        if (
            require_y_span
            and initial_y_span
            < self.min_fit_y_span_px
        ):
            return None

        working = list(
            points
        )

        for _ in range(
            max(
                1,
                self.max_fit_iterations,
            )
        ):
            if (
                len(working)
                < self.min_points_for_fit
            ):
                return None

            (
                slope,
                intercept,
                residuals,
            ) = self._fit_once(
                working
            )

            inlier_mask = (
                residuals
                <= self.outlier_residual_px
            )

            if np.all(
                inlier_mask
            ):
                break

            filtered = [
                point
                for point, keep
                in zip(
                    working,
                    inlier_mask.tolist(),
                )
                if keep
            ]

            if (
                len(filtered)
                < self.min_points_for_fit
            ):
                break

            working = filtered

        if (
            len(working)
            < self.min_points_for_fit
        ):
            return None

        (
            slope,
            intercept,
            residuals,
        ) = self._fit_once(
            working
        )

        rmse = float(
            np.sqrt(
                np.mean(
                    np.square(
                        residuals
                    )
                )
            )
        )

        y_values = np.asarray(
            [
                point.y
                for point in working
            ],
            dtype=np.float64,
        )

        y_span = float(
            np.max(y_values)
            - np.min(y_values)
        )

        # Because image y increases downward:
        #
        # positive angle ->
        # farther path goes toward image right.
        angle_deg = math.degrees(
            math.atan(
                -slope
            )
        )

        return FittedLine(
            slope=slope,
            intercept=intercept,
            angle_deg=angle_deg,
            rmse_px=rmse,
            point_count=len(working),
            y_span_px=y_span,
        )

    @staticmethod
    def _fit_once(
        points: list[LinePoint],
    ) -> tuple[
        float,
        float,
        np.ndarray,
    ]:
        """
        Run one confidence-weighted least-squares fit.

        Returns:
            slope,
            intercept,
            absolute residuals
        """

        y_values = np.asarray(
            [
                point.y
                for point in points
            ],
            dtype=np.float64,
        )

        x_values = np.asarray(
            [
                point.x
                for point in points
            ],
            dtype=np.float64,
        )

        weights = np.asarray(
            [
                max(
                    point.confidence,
                    1e-3,
                )
                for point in points
            ],
            dtype=np.float64,
        )

        design = np.column_stack(
            (
                y_values,
                np.ones_like(
                    y_values
                ),
            )
        )

        sqrt_weights = np.sqrt(
            weights
        )

        weighted_design = (
            design
            * sqrt_weights[:, None]
        )

        weighted_x = (
            x_values
            * sqrt_weights
        )

        coefficients, *_ = (
            np.linalg.lstsq(
                weighted_design,
                weighted_x,
                rcond=None,
            )
        )

        slope = float(
            coefficients[0]
        )

        intercept = float(
            coefficients[1]
        )

        predicted_x = (
            slope
            * y_values
            + intercept
        )

        residuals = np.abs(
            x_values
            - predicted_x
        )

        return (
            slope,
            intercept,
            residuals,
        )

    # ========================================================
    # Near / far regions
    # ========================================================

    def _split_near_far(
        self,
        points: list[LinePoint],
    ) -> tuple[
        list[LinePoint],
        list[LinePoint],
    ]:
        """Split detected centers into near and far image regions."""

        if not points:
            return [], []

        height = float(
            self.image_height
        )

        near_boundary = (
            height
            * self.near_region_ratio
        )

        far_boundary = (
            height
            * self.far_region_ratio
        )

        near_points = [
            point
            for point in points
            if (
                point.y
                >= near_boundary
            )
        ]

        far_points = [
            point
            for point in points
            if (
                point.y
                <= far_boundary
            )
        ]

        minimum = (
            self.min_points_for_fit
        )

        # Fallback using near-to-far order.
        if (
            len(near_points)
            < minimum
        ):
            count = max(
                minimum,
                len(points) // 2,
            )

            near_points = (
                points[:count]
            )

        if (
            len(far_points)
            < minimum
        ):
            count = max(
                minimum,
                len(points) // 2,
            )

            far_points = (
                points[-count:]
            )

        return (
            near_points,
            far_points,
        )

    # ========================================================
    # Turn statistics
    # ========================================================

    @staticmethod
    def _calculate_angle_change_statistics(
        segment_angles: list[float],
    ) -> dict[str, float | None]:
        """
        Calculate path-turn statistics.

        turn_consistency
        ----------------
        Near 1.0:
            Angle changes mostly continue in the same direction.

        Near 0.0:
            Angle changes alternate or are inconsistent.
        """

        if len(segment_angles) < 2:
            return {
                "path_turn_delta_deg":
                    None,

                "angle_change_mean_deg":
                    None,

                "angle_change_max_deg":
                    None,

                "turn_consistency":
                    None,
            }

        angle_array = np.asarray(
            segment_angles,
            dtype=np.float64,
        )

        changes = np.diff(
            angle_array
        )

        absolute_changes = np.abs(
            changes
        )

        mean_change = float(
            np.mean(
                absolute_changes
            )
        )

        max_change = float(
            np.max(
                absolute_changes
            )
        )

        total_change_strength = float(
            np.sum(
                absolute_changes
            )
        )

        path_turn_delta = float(
            angle_array[-1]
            - angle_array[0]
        )

        if (
            total_change_strength
            <= 1e-9
        ):
            consistency = 1.0

        else:
            signed_strength = float(
                abs(
                    np.sum(
                        changes
                    )
                )
            )

            consistency = (
                signed_strength
                / total_change_strength
            )

        return {
            "path_turn_delta_deg":
                path_turn_delta,

            "angle_change_mean_deg":
                mean_change,

            "angle_change_max_deg":
                max_change,

            "turn_consistency":
                float(
                    np.clip(
                        consistency,
                        0.0,
                        1.0,
                    )
                ),
        }

    @staticmethod
    def _detect_corner_start_geometry(
        points: list[LinePoint],
        *,
        min_points: int,
        min_segment_length_px: float,
        straight_max_turn_delta_deg: float,
        min_turn_delta_deg: float,
        onset_deviation_deg: float,
        min_consistent_segments: int,
        min_consistency: float,
    ) -> dict[str, Any]:
        """Classify a 3+ point path and find its first bending point."""
        empty = {
            "detected": False,
            "state": "UNKNOWN",
            "direction": None,
            "start_index": None,
            "start_point": None,
            "turn_delta_deg": None,
            "consistency": None,
        }
        if len(points) < min_points:
            return empty

        segments: list[tuple[int, float]] = []
        for index, (near_point, far_point) in enumerate(
            zip(points[:-1], points[1:])
        ):
            dy = near_point.y - far_point.y
            dx = far_point.x - near_point.x
            if math.hypot(dx, dy) < min_segment_length_px:
                continue
            angle = math.degrees(
                math.atan2(dx, dy)
            )
            segments.append((index, angle))

        if len(segments) < 2:
            return empty

        baseline_angle = float(segments[0][1])
        far_sample_count = min(2, len(segments) - 1)
        far_angles = [
            angle for _, angle in segments[-far_sample_count:]
        ]
        far_angle = float(np.median(far_angles))
        turn_delta = (
            (far_angle - baseline_angle + 180.0) % 360.0
        ) - 180.0
        if abs(turn_delta) <= straight_max_turn_delta_deg:
            return {
                **empty,
                "state": "STRAIGHT",
                "turn_delta_deg": turn_delta,
                "consistency": 1.0,
            }
        if abs(turn_delta) < min_turn_delta_deg:
            return {
                **empty,
                "state": "AMBIGUOUS",
                "turn_delta_deg": turn_delta,
            }

        direction_sign = 1.0 if turn_delta > 0.0 else -1.0
        deviations = [
            ((angle - baseline_angle + 180.0) % 360.0) - 180.0
            for _, angle in segments
        ]
        meaningful = [
            deviation
            for deviation in deviations
            if abs(deviation) >= onset_deviation_deg
        ]
        if not meaningful:
            return {
                **empty,
                "state": "AMBIGUOUS",
                "turn_delta_deg": turn_delta,
                "consistency": 0.0,
            }
        matching = sum(
            1
            for deviation in meaningful
            if deviation * direction_sign > 0.0
        )
        consistency = matching / len(meaningful)
        if consistency < min_consistency:
            return {
                **empty,
                "state": "AMBIGUOUS",
                "turn_delta_deg": turn_delta,
                "consistency": consistency,
            }

        for position in range(
            1,
            len(segments) - min_consistent_segments + 1,
        ):
            window = deviations[
                position:position + min_consistent_segments
            ]
            if not all(
                deviation * direction_sign >= onset_deviation_deg
                for deviation in window
            ):
                continue
            # The start of the first bending segment is the visible turn
            # onset.  With exactly three points this is the middle point.
            start_index = segments[position][0]
            return {
                "detected": True,
                "state": "CORNER",
                "direction": "RIGHT" if direction_sign > 0.0 else "LEFT",
                "start_index": start_index,
                "start_point": points[start_index],
                "turn_delta_deg": turn_delta,
                "consistency": consistency,
            }

        return {
            **empty,
            "state": "AMBIGUOUS",
            "turn_delta_deg": turn_delta,
            "consistency": consistency,
        }

    # ========================================================
    # Quality calculation
    # ========================================================

    @staticmethod
    def _calculate_detection_quality(
        points: list[LinePoint],
    ) -> float:
        """Calculate confidence/count-based detection quality."""

        if not points:
            return 0.0

        count_score = min(
            len(points) / 6.0,
            1.0,
        )

        confidence_score = float(
            np.mean(
                [
                    point.confidence
                    for point in points
                ]
            )
        )

        quality = (
            count_score * 0.35
            + confidence_score * 0.65
        )

        return float(
            np.clip(
                quality,
                0.0,
                1.0,
            )
        )

    def _calculate_geometry_quality(
        self,
        fit: FittedLine | None,
        usable_segment_count: int,
    ) -> float:
        """
        Estimate confidence in the extracted path geometry.

        Uses:
            - Fit RMSE
            - Reliable segment count
            - Vertical image coverage
        """

        if fit is None:
            return 0.0

        if (
            fit.rmse_px
            <= self.good_rmse_px
        ):
            fit_score = 1.0

        elif (
            fit.rmse_px
            >= self.bad_rmse_px
        ):
            fit_score = 0.0

        else:
            denominator = (
                self.bad_rmse_px
                - self.good_rmse_px
            )

            if (
                denominator
                <= 1e-9
            ):
                fit_score = 0.0

            else:
                fit_score = 1.0 - (
                    (
                        fit.rmse_px
                        - self.good_rmse_px
                    )
                    / denominator
                )

        segment_score = min(
            usable_segment_count
            / 5.0,
            1.0,
        )

        y_span_score = min(
            fit.y_span_px
            / 300.0,
            1.0,
        )

        quality = (
            fit_score * 0.50
            + segment_score * 0.25
            + y_span_score * 0.25
        )

        return float(
            np.clip(
                quality,
                0.0,
                1.0,
            )
        )

    # ========================================================
    # Temporal filtering
    # ========================================================

    def _reset_temporal_filter(
        self,
    ) -> None:
        """Reset all temporal filtering state."""

        self.heading_history.clear()

        self.offset_history.clear()

        self.filtered_heading = None

        self.filtered_offset = None

        self.missed_line_frames = 0

    def _register_invalid_line_frame(
        self,
    ) -> None:
        """
        Record one invalid line frame.

        Reset filter history after enough consecutive misses.
        """

        self.missed_line_frames += 1
        self.corner_confirmation_filter.update(False)

        if (
            self.missed_line_frames
            >= self.filter_reset_missed_frames
        ):
            self._reset_temporal_filter()

    def _update_temporal_filter(
        self,
        heading_deg: float,
        lateral_offset_norm: float,
    ) -> dict[str, float | int | bool]:
        """
        Apply:

            raw values
                ->
            recent-window median
                ->
            EMA smoothing

        Median:
            Rejects isolated spikes.

        EMA:
            Smooths smaller frame-to-frame variation.
        """

        self.heading_history.append(
            float(
                heading_deg
            )
        )

        self.offset_history.append(
            float(
                lateral_offset_norm
            )
        )

        heading_array = np.asarray(
            self.heading_history,
            dtype=np.float64,
        )

        offset_array = np.asarray(
            self.offset_history,
            dtype=np.float64,
        )

        median_heading = float(
            np.median(
                heading_array
            )
        )

        median_offset = float(
            np.median(
                offset_array
            )
        )

        alpha = (
            self.filter_ema_alpha
        )

        if (
            self.filtered_heading
            is None
        ):
            self.filtered_heading = (
                median_heading
            )

        else:
            self.filtered_heading = (
                alpha
                * median_heading
                + (
                    1.0 - alpha
                )
                * self.filtered_heading
            )

        if (
            self.filtered_offset
            is None
        ):
            self.filtered_offset = (
                median_offset
            )

        else:
            self.filtered_offset = (
                alpha
                * median_offset
                + (
                    1.0 - alpha
                )
                * self.filtered_offset
            )

        self.missed_line_frames = 0

        return {
            "median_heading_error_deg":
                median_heading,

            "median_lateral_offset_norm":
                median_offset,

            "filtered_heading_error_deg":
                float(
                    self.filtered_heading
                ),

            "filtered_lateral_offset_norm":
                float(
                    self.filtered_offset
                ),

            "filter_ready":
                (
                    len(
                        self.heading_history
                    )
                    >= self.filter_window_size
                ),

            "filter_history_size":
                len(
                    self.heading_history
                ),
        }

    def _robot_center_x_px(self) -> float:
        """Return the calibrated robot center in image pixels."""
        return calibrated_robot_center_x(
            self.image_width,
            self.robot_center_offset_px,
        )

    # ========================================================
    # Empty result
    # ========================================================

    def _empty_result(
        self,
    ) -> dict[str, Any]:
        """Create a consistent result when no valid line exists."""

        return {
            "detected":
                False,

            "line_count":
                0,

            "center_points_px":
                [],

            "robot_center_x_px":
                round(
                    self._robot_center_x_px(),
                    3,
                ),

            "robot_center_offset_px":
                round(
                    self.robot_center_offset_px,
                    3,
                ),

            "nearest_line_point_px":
                None,

            "nearest_line_depth_m":
                None,

            "nearest_line_lateral_offset_m":
                None,

            "nearest_line_ground_forward_distance_m":
                None,

            "nearest_line_horizontal_distance_m":
                None,

            "nearest_line_depth_valid":
                False,

            "corner_preview_raw_detected":
                False,

            "corner_preview_confirmed":
                False,

            "corner_preview_state":
                "UNKNOWN",

            "corner_preview_held":
                False,

            "corner_hold_age_sec":
                None,

            "corner_direction":
                None,

            "corner_start_index":
                None,

            "corner_start_point_px":
                None,

            "corner_start_depth_m":
                None,

            "corner_start_lateral_offset_m":
                None,

            "corner_start_ground_forward_distance_m":
                None,

            "corner_start_distance_m":
                None,

            "corner_start_depth_valid":
                False,

            "corner_turn_delta_deg":
                None,

            "corner_turn_consistency":
                None,

            "corner_remaining_forward_m":
                None,

            "corner_straight_motion_count":
                None,

            "corner_approach_motion":
                None,

            "corner_approach_level":
                None,

            "corner_confirmation_raw_detected":
                False,

            "corner_confirmation_confirmed":
                False,

            "corner_confirmation_hits":
                0,

            "corner_confirmation_required_hits":
                self.corner_confirmation_filter.required_hits,

            "corner_confirmation_matched_previous":
                False,

            "corner_confirmation_window_size":
                self.corner_confirmation_filter.window_size,

            "corner_confirmation_missed_frames":
                0,

            "segment_angles_deg":
                [],

            "segment_vertical_spans_px":
                [],

            "usable_segment_count":
                0,

            "lateral_offset_px":
                None,

            "lateral_offset_norm":
                None,

            "heading_error_deg":
                None,

            "near_heading_deg":
                None,

            "near_fit_heading_deg":
                None,

            "heading_segment_count_used":
                0,

            "heading_stability_deg":
                None,

            "heading_quality":
                0.0,

            "far_heading_deg":
                None,

            "turn_angle_deg":
                None,

            "path_turn_delta_deg":
                None,

            "angle_change_mean_deg":
                None,

            "angle_change_max_deg":
                None,

            "turn_consistency":
                None,

            "median_heading_error_deg":
                None,

            "median_lateral_offset_norm":
                None,

            "filtered_heading_error_deg":
                (
                    round(
                        self.filtered_heading,
                        3,
                    )
                    if (
                        self.filtered_heading
                        is not None
                    )
                    else None
                ),

            "filtered_lateral_offset_norm":
                (
                    round(
                        self.filtered_offset,
                        6,
                    )
                    if (
                        self.filtered_offset
                        is not None
                    )
                    else None
                ),

            "filter_ready":
                False,

            "filter_history_size":
                len(
                    self.heading_history
                ),

            "missed_line_frames":
                self.missed_line_frames,

            "mean_confidence":
                0.0,

            "fit_rmse_px":
                None,

            "detection_quality":
                0.0,

            "geometry_quality":
                0.0,

            "image_width":
                self.image_width,

            "image_height":
                self.image_height,
        }

    # ========================================================
    # Main line analysis
    # ========================================================

    def _analyze_line(
        self,
        points: list[LinePoint],
    ) -> dict[str, Any]:
        """Calculate all line-navigation information."""

        # ----------------------------------------------------
        # Minimum detection validity
        # ----------------------------------------------------

        if (
            len(points)
            < self.min_points_for_fit
        ):
            self._register_invalid_line_frame()

            return self._empty_result()

        # ----------------------------------------------------
        # Full path fit
        # ----------------------------------------------------

        full_fit = self._robust_fit(
            points,
            require_y_span=False,
        )

        if full_fit is None:
            self._register_invalid_line_frame()

            return self._empty_result()

        # ----------------------------------------------------
        # Near / far points
        # ----------------------------------------------------

        (
            near_points,
            far_points,
        ) = self._split_near_far(
            points
        )

        near_fit = self._robust_fit(
            near_points,
            require_y_span=True,
        )

        far_fit = self._robust_fit(
            far_points,
            require_y_span=True,
        )

        # ----------------------------------------------------
        # Reliable segment geometry
        # ----------------------------------------------------

        (
            segment_angles,
            segment_vertical_spans,
        ) = self._calculate_segment_angles(
            points
        )

        # ----------------------------------------------------
        # Immediate heading
        # ----------------------------------------------------

        heading_result = (
            self._calculate_immediate_heading(
                segment_angles,
                segment_vertical_spans,
            )
        )

        immediate_heading = (
            heading_result[
                "heading_deg"
            ]
        )

        near_fit_heading = (
            near_fit.angle_deg
            if near_fit is not None
            else None
        )

        # Current heading priority:
        #
        # 1. Nearest reliable segments
        # 2. Near-region fit
        # 3. Full fit
        if (
            immediate_heading
            is not None
        ):
            heading_error = float(
                immediate_heading
            )

        elif (
            near_fit
            is not None
        ):
            heading_error = float(
                near_fit.angle_deg
            )

        else:
            heading_error = float(
                full_fit.angle_deg
            )

        # For compatibility:
        #
        # near_heading_deg represents current/local heading.
        near_heading = (
            heading_error
        )

        # ----------------------------------------------------
        # Far heading
        # ----------------------------------------------------

        # Only publish when far geometry has enough y-span.
        far_heading = (
            float(
                far_fit.angle_deg
            )
            if (
                far_fit
                is not None
            )
            else None
        )

        turn_angle = (
            far_heading
            - near_heading
            if (
                far_heading
                is not None
            )
            else None
        )

        # ----------------------------------------------------
        # Lateral offset
        # ----------------------------------------------------

        eval_y = (
            self.image_height
            * self.offset_eval_y_ratio
        )

        offset_fit = (
            near_fit
            if (
                near_fit
                is not None
            )
            else full_fit
        )

        predicted_line_x = (
            offset_fit.slope
            * eval_y
            + offset_fit.intercept
        )

        robot_center_x = self._robot_center_x_px()
        image_half_width = self.image_width / 2.0

        lateral_offset_px = (
            predicted_line_x
            - robot_center_x
        )

        lateral_offset_norm = (
            lateral_offset_px
            / image_half_width
            if (
                image_half_width > 0
            )
            else 0.0
        )

        # ----------------------------------------------------
        # Turn statistics
        # ----------------------------------------------------

        angle_stats = (
            self._calculate_angle_change_statistics(
                segment_angles
            )
        )

        # ----------------------------------------------------
        # Confidence / quality
        # ----------------------------------------------------

        mean_confidence = float(
            np.mean(
                [
                    point.confidence
                    for point in points
                ]
            )
        )

        detection_quality = (
            self._calculate_detection_quality(
                points
            )
        )

        geometry_quality = (
            self._calculate_geometry_quality(
                full_fit,
                len(
                    segment_angles
                ),
            )
        )

        # ----------------------------------------------------
        # Temporal filtering
        # ----------------------------------------------------

        filter_result = (
            self._update_temporal_filter(
                heading_error,
                lateral_offset_norm,
            )
        )

        nearest_line_distance = self._nearest_line_distance(points[0])
        corner_preview = self._analyze_corner_preview(points)

        # ----------------------------------------------------
        # Return JSON-compatible result
        # ----------------------------------------------------

        return {
            "detected":
                True,

            "line_count":
                len(points),

            "center_points_px":
                [
                    [
                        int(
                            round(
                                point.x
                            )
                        ),
                        int(
                            round(
                                point.y
                            )
                        ),
                    ]
                    for point in points
                ],

            **nearest_line_distance,

            **corner_preview,

            "robot_center_x_px":
                round(
                    robot_center_x,
                    3,
                ),

            "robot_center_offset_px":
                round(
                    self.robot_center_offset_px,
                    3,
                ),

            "segment_angles_deg":
                segment_angles,

            "segment_vertical_spans_px":
                [
                    round(
                        value,
                        3,
                    )
                    for value
                    in segment_vertical_spans
                ],

            "usable_segment_count":
                len(
                    segment_angles
                ),

            "lateral_offset_px":
                round(
                    lateral_offset_px,
                    3,
                ),

            "lateral_offset_norm":
                round(
                    lateral_offset_norm,
                    6,
                ),

            "heading_error_deg":
                round(
                    heading_error,
                    3,
                ),

            "near_heading_deg":
                round(
                    near_heading,
                    3,
                ),

            "near_fit_heading_deg":
                (
                    round(
                        near_fit_heading,
                        3,
                    )
                    if (
                        near_fit_heading
                        is not None
                    )
                    else None
                ),

            "heading_segment_count_used":
                int(
                    heading_result[
                        "segment_count_used"
                    ]
                ),

            "heading_stability_deg":
                (
                    round(
                        float(
                            heading_result[
                                "stability_deg"
                            ]
                        ),
                        3,
                    )
                    if (
                        heading_result[
                            "stability_deg"
                        ]
                        is not None
                    )
                    else None
                ),

            "heading_quality":
                round(
                    float(
                        heading_result[
                            "quality"
                        ]
                    ),
                    6,
                ),

            "far_heading_deg":
                (
                    round(
                        far_heading,
                        3,
                    )
                    if (
                        far_heading
                        is not None
                    )
                    else None
                ),

            "turn_angle_deg":
                (
                    round(
                        turn_angle,
                        3,
                    )
                    if (
                        turn_angle
                        is not None
                    )
                    else None
                ),

            "path_turn_delta_deg":
                (
                    round(
                        float(
                            angle_stats[
                                "path_turn_delta_deg"
                            ]
                        ),
                        3,
                    )
                    if (
                        angle_stats[
                            "path_turn_delta_deg"
                        ]
                        is not None
                    )
                    else None
                ),

            "angle_change_mean_deg":
                (
                    round(
                        float(
                            angle_stats[
                                "angle_change_mean_deg"
                            ]
                        ),
                        3,
                    )
                    if (
                        angle_stats[
                            "angle_change_mean_deg"
                        ]
                        is not None
                    )
                    else None
                ),

            "angle_change_max_deg":
                (
                    round(
                        float(
                            angle_stats[
                                "angle_change_max_deg"
                            ]
                        ),
                        3,
                    )
                    if (
                        angle_stats[
                            "angle_change_max_deg"
                        ]
                        is not None
                    )
                    else None
                ),

            "turn_consistency":
                (
                    round(
                        float(
                            angle_stats[
                                "turn_consistency"
                            ]
                        ),
                        6,
                    )
                    if (
                        angle_stats[
                            "turn_consistency"
                        ]
                        is not None
                    )
                    else None
                ),

            # Raw median outputs are included for debugging.
            "median_heading_error_deg":
                round(
                    float(
                        filter_result[
                            "median_heading_error_deg"
                        ]
                    ),
                    3,
                ),

            "median_lateral_offset_norm":
                round(
                    float(
                        filter_result[
                            "median_lateral_offset_norm"
                        ]
                    ),
                    6,
                ),

            # Recommended stabilized values for algorithm use.
            "filtered_heading_error_deg":
                round(
                    float(
                        filter_result[
                            "filtered_heading_error_deg"
                        ]
                    ),
                    3,
                ),

            "filtered_lateral_offset_norm":
                round(
                    float(
                        filter_result[
                            "filtered_lateral_offset_norm"
                        ]
                    ),
                    6,
                ),

            "filter_ready":
                bool(
                    filter_result[
                        "filter_ready"
                    ]
                ),

            "filter_history_size":
                int(
                    filter_result[
                        "filter_history_size"
                    ]
                ),

            "missed_line_frames":
                self.missed_line_frames,

            "mean_confidence":
                round(
                    mean_confidence,
                    6,
                ),

            "fit_rmse_px":
                round(
                    full_fit.rmse_px,
                    3,
                ),

            "detection_quality":
                round(
                    detection_quality,
                    6,
                ),

            "geometry_quality":
                round(
                    geometry_quality,
                    6,
                ),

            "image_width":
                self.image_width,

            "image_height":
                self.image_height,
        }


# ============================================================
# ROS main
# ============================================================


def main(
    args: list[str] | None = None,
) -> None:
    rclpy.init(
        args=args
    )

    node = YoloLineAnalyzer()

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
