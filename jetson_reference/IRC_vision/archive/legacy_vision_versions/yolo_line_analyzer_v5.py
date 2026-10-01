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
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


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


# ============================================================
# Main ROS 2 node
# ============================================================


class YoloLineAnalyzer(Node):
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

    def __init__(self) -> None:
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
            "/camera/camera/color/image_raw",
        )

        self.declare_parameter(
            "output_topic",
            "/vision/line_info",
        )

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
            0.05,
        )

        self.declare_parameter(
            "roi_x_max_ratio",
            0.95,
        )

        self.declare_parameter(
            "roi_y_min_ratio",
            0.05,
        )

        self.declare_parameter(
            "roi_y_max_ratio",
            0.97,
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
        # Segment filtering
        # ====================================================

        # Very small vertical differences are dangerous because
        # perspective can cause image-space angles to become
        # extremely exaggerated.
        self.declare_parameter(
            "min_segment_dy_px",
            35.0,
        )

        # ====================================================
        # Immediate heading estimation
        # ====================================================

        # Number of nearest reliable segments used to calculate
        # the immediate heading.
        self.declare_parameter(
            "heading_segment_count",
            2,
        )

        # Weight decay:
        #
        # segment 1 -> 1.0
        # segment 2 -> 0.35
        # segment 3 -> 0.1225
        self.declare_parameter(
            "heading_weight_decay",
            0.35,
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

        # ====================================================
        # Camera resolution
        # ====================================================

        # Default until first Image message arrives.
        self.image_width = 1280
        self.image_height = 720

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

        image_topic = str(
            self.get_parameter(
                "image_topic"
            ).value
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

        self.detection_subscription = (
            self.create_subscription(
                String,
                detections_topic,
                self._detections_callback,
                10,
            )
        )

        self.image_subscription = (
            self.create_subscription(
                Image,
                image_topic,
                self._image_callback,
                10,
            )
        )

        # ====================================================
        # Startup log
        # ====================================================

        self.get_logger().info(
            f"Subscribing detections: {detections_topic}"
        )

        self.get_logger().info(
            f"Subscribing image info: {image_topic}"
        )

        self.get_logger().info(
            f"Publishing line info: {output_topic}"
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

    def _image_callback(
        self,
        message: Image,
    ) -> None:
        """Update actual RealSense image resolution."""

        if message.width > 0 and message.height > 0:
            self.image_width = int(
                message.width
            )

            self.image_height = int(
                message.height
            )

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

            points = (
                self._remove_near_duplicates(
                    points
                )
            )

            result = self._analyze_line(
                points
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

        image_center_x = (
            self.image_width / 2.0
        )

        lateral_offset_px = (
            predicted_line_x
            - image_center_x
        )

        lateral_offset_norm = (
            lateral_offset_px
            / image_center_x
            if (
                image_center_x > 0
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
