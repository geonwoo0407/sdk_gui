#!/usr/bin/env python3
"""
YOLO26 line geometry analyzer.

Input:
    /vision/detections
        std_msgs/String JSON from yolo26_detector

Output:
    /vision/line_info
        std_msgs/String JSON

Design goal:
    This node does NOT decide robot motion.

    It only provides condition information such as:
    - line center points
    - lateral offset
    - current heading error
    - future path heading
    - turn strength
    - turn consistency
    - detection quality
    - geometry quality
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


@dataclass(frozen=True)
class LinePoint:
    """One YOLO line detection center."""

    x: float
    y: float
    confidence: float


@dataclass(frozen=True)
class FittedLine:
    """
    Image-space fitted path.

    Model:
        x = slope * y + intercept

    Coordinate convention:
        x increases to image right
        y increases downward
    """

    slope: float
    intercept: float
    angle_deg: float
    rmse_px: float
    point_count: int
    y_span_px: float


class YoloLineAnalyzer(Node):
    """Analyze YOLO line detections for navigation geometry."""

    def __init__(self) -> None:
        super().__init__("yolo_line_analyzer")

        # =====================================================
        # ROS topics
        # =====================================================

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

        # =====================================================
        # Detection filtering
        # =====================================================

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

        # =====================================================
        # ROI
        # =====================================================

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

        # =====================================================
        # Point filtering
        # =====================================================

        self.declare_parameter(
            "min_point_distance_px",
            10.0,
        )

        self.declare_parameter(
            "max_points",
            30,
        )

        # =====================================================
        # Segment filtering
        # =====================================================

        # Segments with smaller vertical spacing than this are
        # unstable because perspective makes dy very small.
        self.declare_parameter(
            "min_segment_dy_px",
            35.0,
        )
        # =====================================================
        # Immediate heading estimation
        # =====================================================

        # 현재 heading 계산에 사용할 가장 가까운 신뢰 segment 개수
        self.declare_parameter(
            "heading_segment_count",
            2,
        )

        # 가까운 segment부터 가중치를 감소시키는 비율
        # 1번째 segment: 1.0
        # 2번째 segment: 0.35
        # 3번째 segment: 0.1225 ...
        self.declare_parameter(
            "heading_weight_decay",
            0.35,
        )

        # heading 안정성 평가 기준
        # 가까운 segment들의 각도 표준편차가 이 값 이상이면
        # heading quality가 낮아진다.
        self.declare_parameter(
            "bad_heading_std_deg",
            25.0,

        )
        # =====================================================
        # Robust line fitting
        # =====================================================

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

        # =====================================================
        # Near / far regions
        # =====================================================

        self.declare_parameter(
            "near_region_ratio",
            0.55,
        )

        self.declare_parameter(
            "far_region_ratio",
            0.45,
        )

        # =====================================================
        # Offset
        # =====================================================

        self.declare_parameter(
            "offset_eval_y_ratio",
            0.82,
        )

        # =====================================================
        # Quality
        # =====================================================

        self.declare_parameter(
            "good_rmse_px",
            12.0,
        )

        self.declare_parameter(
            "bad_rmse_px",
            80.0,
        )

        # =====================================================
        # Load parameters
        # =====================================================

        self.line_class_name = str(
            self.get_parameter("line_class_name").value
        )

        self.min_confidence = float(
            self.get_parameter("min_confidence").value
        )

        self.min_bbox_width_px = int(
            self.get_parameter("min_bbox_width_px").value
        )

        self.min_bbox_height_px = int(
            self.get_parameter("min_bbox_height_px").value
        )

        self.roi_x_min_ratio = float(
            self.get_parameter("roi_x_min_ratio").value
        )

        self.roi_x_max_ratio = float(
            self.get_parameter("roi_x_max_ratio").value
        )

        self.roi_y_min_ratio = float(
            self.get_parameter("roi_y_min_ratio").value
        )

        self.roi_y_max_ratio = float(
            self.get_parameter("roi_y_max_ratio").value
        )

        self.min_point_distance_px = float(
            self.get_parameter("min_point_distance_px").value
        )

        self.max_points = int(
            self.get_parameter("max_points").value
        )

        self.min_segment_dy_px = float(
            self.get_parameter("min_segment_dy_px").value
        )

        self.heading_segment_count = int(
            self.get_parameter("heading_segment_count").value
        )

        self.heading_weight_decay = float(
            self.get_parameter("heading_weight_decay").value
        )

        self.bad_heading_std_deg = float(
            self.get_parameter("bad_heading_std_deg").value
        )

        self.min_points_for_fit = int(
            self.get_parameter("min_points_for_fit").value
        )

        self.min_fit_y_span_px = float(
            self.get_parameter("min_fit_y_span_px").value
        )

        self.outlier_residual_px = float(
            self.get_parameter("outlier_residual_px").value
        )

        self.max_fit_iterations = int(
            self.get_parameter("max_fit_iterations").value
        )

        self.near_region_ratio = float(
            self.get_parameter("near_region_ratio").value
        )

        self.far_region_ratio = float(
            self.get_parameter("far_region_ratio").value
        )

        self.offset_eval_y_ratio = float(
            self.get_parameter("offset_eval_y_ratio").value
        )

        self.good_rmse_px = float(
            self.get_parameter("good_rmse_px").value
        )

        self.bad_rmse_px = float(
            self.get_parameter("bad_rmse_px").value
        )

        # Default resolution until first Image message arrives.
        self.image_width = 1280
        self.image_height = 720

        detections_topic = str(
            self.get_parameter("detections_topic").value
        )

        image_topic = str(
            self.get_parameter("image_topic").value
        )

        output_topic = str(
            self.get_parameter("output_topic").value
        )

        # =====================================================
        # ROS
        # =====================================================

        self.publisher = self.create_publisher(
            String,
            output_topic,
            10,
        )

        self.detection_subscription = self.create_subscription(
            String,
            detections_topic,
            self._detections_callback,
            10,
        )

        self.image_subscription = self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            10,
        )

        self.get_logger().info(
            f"Subscribing detections: {detections_topic}"
        )

        self.get_logger().info(
            f"Subscribing image info: {image_topic}"
        )

        self.get_logger().info(
            f"Publishing line info: {output_topic}"
        )

    # =========================================================
    # ROS callbacks
    # =========================================================

    def _image_callback(self, message: Image) -> None:
        """Update actual camera resolution."""

        if message.width > 0 and message.height > 0:
            self.image_width = int(message.width)
            self.image_height = int(message.height)

    def _detections_callback(self, message: String) -> None:
        """Process one YOLO detection message."""

        started = time.perf_counter()

        try:
            payload = json.loads(message.data)

            detections = payload.get(
                "detections",
                [],
            )

            if not isinstance(detections, list):
                raise ValueError(
                    "'detections' must be a list"
                )

            points = self._extract_line_points(
                detections
            )

            points = self._remove_near_duplicates(
                points
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

            self.publisher.publish(output)

        except (
            json.JSONDecodeError,
            TypeError,
            ValueError,
            KeyError,
        ) as exc:

            self.get_logger().warning(
                f"Invalid detection message: "
                f"{type(exc).__name__}: {exc}"
            )

        except Exception as exc:

            self.get_logger().error(
                f"Line analysis failed: "
                f"{type(exc).__name__}: {exc}"
            )

    # =========================================================
    # Detection extraction
    # =========================================================

    def _extract_line_points(
        self,
        detections: list[dict[str, Any]],
    ) -> list[LinePoint]:

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

            if confidence < self.min_confidence:
                continue

            bbox = detection.get("bbox")
            center = detection.get("center")

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

            bbox_width = right - left
            bbox_height = bottom - top

            if bbox_width < self.min_bbox_width_px:
                continue

            if bbox_height < self.min_bbox_height_px:
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

        # Near → Far
        points.sort(
            key=lambda point: point.y,
            reverse=True,
        )

        return points[: self.max_points]

    # =========================================================
    # Duplicate filtering
    # =========================================================

    def _remove_near_duplicates(
        self,
        points: list[LinePoint],
    ) -> list[LinePoint]:

        if len(points) <= 1:
            return points

        by_confidence = sorted(
            points,
            key=lambda point: point.confidence,
            reverse=True,
        )

        accepted: list[LinePoint] = []

        for candidate in by_confidence:

            duplicate = False

            for existing in accepted:

                distance = math.hypot(
                    candidate.x - existing.x,
                    candidate.y - existing.y,
                )

                if distance < self.min_point_distance_px:
                    duplicate = True
                    break

            if not duplicate:
                accepted.append(candidate)

        accepted.sort(
            key=lambda point: point.y,
            reverse=True,
        )

        return accepted

    # =========================================================
    # Segment geometry
    # =========================================================

    def _calculate_segment_angles(
        self,
        points: list[LinePoint],
    ) -> tuple[list[float], list[float]]:

        """
        Calculate reliable angles between consecutive centers.

        Returns:
            angles:
                Reliable segment angles only.

            vertical_spans:
                dy of each accepted segment.

        Very small dy segments are ignored because perspective
        can greatly exaggerate their image-space angles.
        """

        angles: list[float] = []
        vertical_spans: list[float] = []

        if len(points) < 2:
            return angles, vertical_spans

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

            if dy < self.min_segment_dy_px:
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

        def _calculate_immediate_heading(
            self,
            segment_angles: list[float],
            segment_vertical_spans: list[float],
        ) -> dict[str, float | int | None]:
            """
            Estimate the immediate path heading from the nearest reliable segments.

            The first reliable segment receives the largest weight.
            Farther segments receive exponentially smaller weights.

            Example:
                angles = [10.0, 30.0, 60.0]
                decay = 0.35

                weights:
                    1.0
                    0.35
                    0.1225

            By default only the nearest `heading_segment_count`
            reliable segments are used.
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
                max(1, self.heading_segment_count),
            )

            selected_angles = np.asarray(
                segment_angles[:count],
                dtype=np.float64,
            )

            selected_spans = np.asarray(
                segment_vertical_spans[:count],
                dtype=np.float64,
            )

            # Near segments have stronger influence.
            distance_weights = np.asarray(
                [
                    self.heading_weight_decay ** index
                    for index in range(count)
                ],
                dtype=np.float64,
            )

            # Larger vertical span generally gives more stable image geometry.
            if np.max(selected_spans) > 1e-9:
                span_weights = (
                    selected_spans
                    / np.max(selected_spans)
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
                np.sum(weights)
            )

            if weight_sum <= 1e-9:
                weights = np.ones(
                    count,
                    dtype=np.float64,
                )

                weight_sum = float(count)

            heading_deg = float(
                np.sum(
                    selected_angles * weights
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
                max(variance, 0.0)
            )

            # More usable segments increase confidence.
            count_score = min(
                count
                / max(1, self.heading_segment_count),
                1.0,
            )

            # Low standard deviation means nearby headings agree.
            if self.bad_heading_std_deg <= 1e-9:
                stability_score = 1.0
            else:
                stability_score = 1.0 - min(
                    stability_deg
                    / self.bad_heading_std_deg,
                    1.0,
                )

            quality = (
                count_score * 0.40
                + stability_score * 0.60
            )

            return {
                "heading_deg": heading_deg,
                "segment_count_used": count,
                "stability_deg": stability_deg,
                "quality": float(
                    np.clip(
                        quality,
                        0.0,
                        1.0,
                    )
                ),
            }

    # =========================================================
    # Robust fitting
    # =========================================================

    def _robust_fit(
        self,
        points: list[LinePoint],
        require_y_span: bool = True,
    ) -> FittedLine | None:

        if len(points) < self.min_points_for_fit:
            return None

        y_span = (
            max(point.y for point in points)
            - min(point.y for point in points)
        )

        if (
            require_y_span
            and y_span < self.min_fit_y_span_px
        ):
            return None

        working = list(points)

        for _ in range(
            max(
                1,
                self.max_fit_iterations,
            )
        ):

            if len(working) < self.min_points_for_fit:
                return None

            y_values = np.asarray(
                [
                    point.y
                    for point in working
                ],
                dtype=np.float64,
            )

            x_values = np.asarray(
                [
                    point.x
                    for point in working
                ],
                dtype=np.float64,
            )

            weights = np.asarray(
                [
                    max(
                        point.confidence,
                        1e-3,
                    )
                    for point in working
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

            coefficients, *_ = np.linalg.lstsq(
                weighted_design,
                weighted_x,
                rcond=None,
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

            inlier_mask = (
                residuals
                <= self.outlier_residual_px
            )

            if np.all(inlier_mask):
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

            if len(filtered) < self.min_points_for_fit:
                break

            working = filtered

        if len(working) < self.min_points_for_fit:
            return None

        y_values = np.asarray(
            [
                point.y
                for point in working
            ],
            dtype=np.float64,
        )

        x_values = np.asarray(
            [
                point.x
                for point in working
            ],
            dtype=np.float64,
        )

        weights = np.asarray(
            [
                max(
                    point.confidence,
                    1e-3,
                )
                for point in working
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

        coefficients, *_ = np.linalg.lstsq(
            weighted_design,
            weighted_x,
            rcond=None,
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

        rmse = float(
            np.sqrt(
                np.mean(
                    np.square(
                        x_values
                        - predicted_x
                    )
                )
            )
        )

        y_span = float(
            np.max(y_values)
            - np.min(y_values)
        )

        # Positive:
        # farther part of path is toward image right.
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

    # =========================================================
    # Near / far
    # =========================================================

    def _split_near_far(
        self,
        points: list[LinePoint],
    ) -> tuple[
        list[LinePoint],
        list[LinePoint],
    ]:

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
            if point.y >= near_boundary
        ]

        far_points = [
            point
            for point in points
            if point.y <= far_boundary
        ]

        minimum = self.min_points_for_fit

        if len(near_points) < minimum:

            count = max(
                minimum,
                len(points) // 2,
            )

            near_points = points[:count]

        if len(far_points) < minimum:

            count = max(
                minimum,
                len(points) // 2,
            )

            far_points = points[-count:]

        return (
            near_points,
            far_points,
        )

    # =========================================================
    # Turn statistics
    # =========================================================

    @staticmethod
    def _calculate_angle_change_statistics(
        segment_angles: list[float],
    ) -> dict[str, float | None]:

        if len(segment_angles) < 2:
            return {
                "angle_change_mean_deg": None,
                "angle_change_max_deg": None,
                "turn_consistency": None,
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

        if total_change_strength <= 1e-9:

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
            "angle_change_mean_deg": mean_change,
            "angle_change_max_deg": max_change,
            "turn_consistency": float(
                np.clip(
                    consistency,
                    0.0,
                    1.0,
                )
            ),
        }

    # =========================================================
    # Quality
    # =========================================================

    @staticmethod
    def _calculate_detection_quality(
        points: list[LinePoint],
    ) -> float:

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
        points: list[LinePoint],
        fit: FittedLine | None,
        segment_count: int,
    ) -> float:

        if not points or fit is None:
            return 0.0

        if fit.rmse_px <= self.good_rmse_px:
            fit_score = 1.0

        elif fit.rmse_px >= self.bad_rmse_px:
            fit_score = 0.0

        else:
            fit_score = 1.0 - (
                (
                    fit.rmse_px
                    - self.good_rmse_px
                )
                /
                (
                    self.bad_rmse_px
                    - self.good_rmse_px
                )
            )

        segment_score = min(
            segment_count / 5.0,
            1.0,
        )

        y_span_score = min(
            fit.y_span_px / 300.0,
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

    # =========================================================
    # Results
    # =========================================================

    def _empty_result(
        self,
    ) -> dict[str, Any]:

        return {
            "detected": False,
            "line_count": 0,

            "center_points_px": [],

            "segment_angles_deg": [],
            "usable_segment_count": 0,

            "lateral_offset_px": None,
            "lateral_offset_norm": None,

            "heading_error_deg": None,

            "near_heading_deg": None,
            "near_fit_heading_deg": None,

            "heading_segment_count_used": 0,
            "heading_stability_deg": None,
            "heading_quality": 0.0,

            "far_heading_deg": None,
            "turn_angle_deg": None,

            "angle_change_mean_deg": None,
            "angle_change_max_deg": None,
            "turn_consistency": None,

            "mean_confidence": 0.0,

            "fit_rmse_px": None,

            "detection_quality": 0.0,
            "geometry_quality": 0.0,

            "image_width": self.image_width,
            "image_height": self.image_height,
        }

    def _analyze_line(
        self,
        points: list[LinePoint],
    ) -> dict[str, Any]:

        if len(points) < self.min_points_for_fit:
            return self._empty_result()

        full_fit = self._robust_fit(
            points,
            require_y_span=False,
        )

        if full_fit is None:
            return self._empty_result()

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

        # -----------------------------------------------------
        # Reliable segment geometry
        # -----------------------------------------------------

        (
            segment_angles,
            segment_vertical_spans,
        ) = self._calculate_segment_angles(
            points
        )

        # -----------------------------------------------------
        # Immediate heading
        # -----------------------------------------------------

        heading_result = (
            self._calculate_immediate_heading(
                segment_angles,
                segment_vertical_spans,
            )
        )

        immediate_heading = (
            heading_result["heading_deg"]
        )

        near_fit_heading = (
            near_fit.angle_deg
            if near_fit is not None
            else None
        )

        # Preferred current heading:
        # 1. nearest reliable segments
        # 2. near-region fit
        # 3. full fit
        if immediate_heading is not None:

            heading_error = float(
                immediate_heading
            )

        elif near_fit is not None:

            heading_error = (
                near_fit.angle_deg
            )

        else:

            heading_error = (
                full_fit.angle_deg
            )

        # For backward compatibility,
        # near_heading_deg means immediate local heading.
        near_heading = heading_error

        # Far heading is only published when far geometry has
        # enough vertical span.
        far_heading = (
            far_fit.angle_deg
            if far_fit is not None
            else None
        )

        turn_angle = (
            far_heading - near_heading
            if far_heading is not None
            else None
        )

        # -----------------------------------------------------
        # Lateral offset near robot
        # -----------------------------------------------------

        eval_y = (
            self.image_height
            * self.offset_eval_y_ratio
        )

        offset_fit = (
            near_fit
            if near_fit is not None
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
            if image_center_x > 0
            else 0.0
        )

        # -----------------------------------------------------
        # Reliable segment angles
        # -----------------------------------------------------

        angle_stats = (
            self._calculate_angle_change_statistics(
                segment_angles
            )
        )

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
                points,
                full_fit,
                len(segment_angles),
            )
        )

        return {
            "detected": True,

            "line_count": len(points),

            "center_points_px": [
                [
                    int(round(point.x)),
                    int(round(point.y)),
                ]
                for point in points
            ],

            "segment_angles_deg":
                segment_angles,

            "segment_vertical_spans_px": [
                round(value, 3)
                for value in segment_vertical_spans
            ],

            "usable_segment_count":
                len(segment_angles),

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
                    if near_fit_heading is not None
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
                    if heading_result[
                        "stability_deg"
                    ]
                    is not None
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
                    if far_heading is not None
                    else None
                ),

            "turn_angle_deg":
                (
                    round(
                        turn_angle,
                        3,
                    )
                    if turn_angle is not None
                    else None
                ),

            "angle_change_mean_deg":
                (
                    round(
                        angle_stats[
                            "angle_change_mean_deg"
                        ],
                        3,
                    )
                    if angle_stats[
                        "angle_change_mean_deg"
                    ]
                    is not None
                    else None
                ),

            "angle_change_max_deg":
                (
                    round(
                        angle_stats[
                            "angle_change_max_deg"
                        ],
                        3,
                    )
                    if angle_stats[
                        "angle_change_max_deg"
                    ]
                    is not None
                    else None
                ),

            "turn_consistency":
                (
                    round(
                        angle_stats[
                            "turn_consistency"
                        ],
                        6,
                    )
                    if angle_stats[
                        "turn_consistency"
                    ]
                    is not None
                    else None
                ),

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
