#!/usr/bin/env python3
"""
YOLO26 line detection analyzer.

Input:
    /vision/detections  (std_msgs/String, JSON)

Output:
    /vision/line_info   (std_msgs/String, JSON)

This node does NOT decide robot motion.
It only extracts navigation-related condition information from line detections.
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
    """One detected line center."""

    x: float
    y: float
    confidence: float


@dataclass(frozen=True)
class FittedLine:
    """
    Fitted path model:

        x = slope * y + intercept

    Image coordinate:
        x → right
        y → down
    """

    slope: float
    intercept: float
    angle_deg: float
    rmse_px: float
    point_count: int


class YoloLineAnalyzer(Node):
    """Analyze YOLO line detections."""

    def __init__(self) -> None:
        super().__init__("yolo_line_analyzer")

        # -----------------------------
        # ROS topics
        # -----------------------------
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

        # -----------------------------
        # Detection filtering
        # -----------------------------
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

        # -----------------------------
        # ROI
        # -----------------------------
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

        # -----------------------------
        # Point filtering
        # -----------------------------
        self.declare_parameter(
            "min_point_distance_px",
            10.0,
        )

        self.declare_parameter(
            "max_points",
            30,
        )

        # -----------------------------
        # Robust line fitting
        # -----------------------------
        self.declare_parameter(
            "min_points_for_fit",
            2,
        )

        self.declare_parameter(
            "outlier_residual_px",
            50.0,
        )

        self.declare_parameter(
            "max_fit_iterations",
            3,
        )

        # -----------------------------
        # Near / far division
        # -----------------------------
        self.declare_parameter(
            "near_region_ratio",
            0.55,
        )

        self.declare_parameter(
            "far_region_ratio",
            0.45,
        )

        # -----------------------------
        # Offset evaluation
        # -----------------------------
        self.declare_parameter(
            "offset_eval_y_ratio",
            0.82,
        )

        # -----------------------------
        # Quality score
        # -----------------------------
        self.declare_parameter(
            "good_rmse_px",
            12.0,
        )

        self.declare_parameter(
            "bad_rmse_px",
            80.0,
        )

        # -----------------------------
        # Load parameters
        # -----------------------------
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

        self.min_points_for_fit = int(
            self.get_parameter("min_points_for_fit").value
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

        # Default until first image arrives.
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

        # -----------------------------
        # ROS publisher/subscribers
        # -----------------------------
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
        """Save actual RealSense image resolution."""

        if message.width > 0 and message.height > 0:
            self.image_width = int(message.width)
            self.image_height = int(message.height)

    def _detections_callback(self, message: String) -> None:
        """Process one YOLO detection message."""

        started = time.perf_counter()

        try:
            payload = json.loads(message.data)

            detections = payload.get("detections", [])

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
                (time.perf_counter() - started) * 1000.0,
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
            width * self.roi_x_min_ratio
        )

        roi_right = (
            width * self.roi_x_max_ratio
        )

        roi_top = (
            height * self.roi_y_min_ratio
        )

        roi_bottom = (
            height * self.roi_y_max_ratio
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

            too_close = False

            for existing in accepted:

                distance = math.hypot(
                    candidate.x - existing.x,
                    candidate.y - existing.y,
                )

                if (
                    distance
                    < self.min_point_distance_px
                ):
                    too_close = True
                    break

            if not too_close:
                accepted.append(candidate)

        accepted.sort(
            key=lambda point: point.y,
            reverse=True,
        )

        return accepted

    # =========================================================
    # Segment angles
    # =========================================================

    @staticmethod
    def _calculate_segment_angles(
        points: list[LinePoint],
    ) -> list[float]:
        """
        Calculate angle between consecutive points.

        Positive:
            Farther point is toward image right.

        Negative:
            Farther point is toward image left.
        """

        angles: list[float] = []

        if len(points) < 2:
            return angles

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

            if dy <= 1e-6:
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

        return angles

    # =========================================================
    # Robust line fitting
    # =========================================================

    def _robust_fit(
        self,
        points: list[LinePoint],
    ) -> FittedLine | None:

        if (
            len(points)
            < self.min_points_for_fit
        ):
            return None

        working = list(points)

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

            y_values = np.array(
                [
                    point.y
                    for point in working
                ],
                dtype=np.float64,
            )

            x_values = np.array(
                [
                    point.x
                    for point in working
                ],
                dtype=np.float64,
            )

            weights = np.array(
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
                slope * y_values
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

        y_values = np.array(
            [
                point.y
                for point in working
            ],
            dtype=np.float64,
        )

        x_values = np.array(
            [
                point.x
                for point in working
            ],
            dtype=np.float64,
        )

        weights = np.array(
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
            slope * y_values
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

        # Important:
        # y increases downward in image coordinates.
        # We define positive angle when the line moves
        # toward image right as it goes farther.
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
        )

    # =========================================================
    # Near / far regions
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

        minimum = (
            self.min_points_for_fit
        )

        # Fallback using ordered points.
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
    # Curvature
    # =========================================================

    @staticmethod
    def _calculate_curvature(
        segment_angles: list[float],
    ) -> float | None:

        if len(segment_angles) < 2:
            return None

        angle_array = np.asarray(
            segment_angles,
            dtype=np.float64,
        )

        angle_changes = np.diff(
            angle_array
        )

        curvature = float(
            np.mean(
                np.abs(
                    angle_changes
                )
            )
        )

        return curvature

    # =========================================================
    # Quality score
    # =========================================================

    def _calculate_quality(
        self,
        points: list[LinePoint],
        fit: FittedLine | None,
    ) -> float:

        if not points or fit is None:
            return 0.0

        count_score = min(
            len(points) / 6.0,
            1.0,
        )

        mean_confidence = float(
            np.mean(
                [
                    point.confidence
                    for point in points
                ]
            )
        )

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

        quality = (
            count_score * 0.25
            + mean_confidence * 0.40
            + fit_score * 0.35
        )

        return float(
            np.clip(
                quality,
                0.0,
                1.0,
            )
        )

    # =========================================================
    # Result creation
    # =========================================================

    def _empty_result(
        self,
    ) -> dict[str, Any]:

        return {
            "detected": False,
            "line_count": 0,
            "center_points_px": [],
            "segment_angles_deg": [],
            "lateral_offset_px": None,
            "lateral_offset_norm": None,
            "heading_error_deg": None,
            "near_heading_deg": None,
            "far_heading_deg": None,
            "turn_angle_deg": None,
            "curvature_deg": None,
            "mean_confidence": 0.0,
            "fit_rmse_px": None,
            "quality": 0.0,
            "image_width": self.image_width,
            "image_height": self.image_height,
        }

    def _analyze_line(
        self,
        points: list[LinePoint],
    ) -> dict[str, Any]:

        if (
            len(points)
            < self.min_points_for_fit
        ):
            return self._empty_result()

        full_fit = self._robust_fit(
            points
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
            near_points
        )

        far_fit = self._robust_fit(
            far_points
        )

        near_heading = (
            near_fit.angle_deg
            if near_fit is not None
            else full_fit.angle_deg
        )

        far_heading = (
            far_fit.angle_deg
            if far_fit is not None
            else full_fit.angle_deg
        )

        heading_error = (
            near_heading
        )

        turn_angle = (
            far_heading
            - near_heading
        )

        # Evaluate line x-position near the robot.
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

        segment_angles = (
            self._calculate_segment_angles(
                points
            )
        )

        curvature = (
            self._calculate_curvature(
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

        quality = (
            self._calculate_quality(
                points,
                full_fit,
            )
        )

        return {
            "detected": True,

            "line_count": len(points),

            "center_points_px": [
                [
                    int(
                        round(point.x)
                    ),
                    int(
                        round(point.y)
                    ),
                ]
                for point in points
            ],

            "segment_angles_deg":
                segment_angles,

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

            "far_heading_deg":
                round(
                    far_heading,
                    3,
                ),

            "turn_angle_deg":
                round(
                    turn_angle,
                    3,
                ),

            "curvature_deg":
                (
                    round(
                        curvature,
                        3,
                    )
                    if curvature is not None
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

            "quality":
                round(
                    quality,
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
