#!/usr/bin/env python3
"""Lightweight RGB-D visual odometry test node for RealSense D435i.

This node is an experiment, not a full SLAM system.

It tracks ordinary image features between RGB frames, samples aligned depth,
estimates a 3D rigid transform with RANSAC, rejects dynamic/outlier features,
and publishes a coarse relative odometry estimate.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import time
from typing import Any

from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


@dataclass(frozen=True)
class VisualOdomInfo:
    """JSON payload published on /vision/visual_odom."""

    tracking_ok: bool
    update_used: bool
    x_m: float
    y_m: float
    yaw_deg: float
    total_distance_m: float
    delta_forward_m: float
    delta_lateral_m: float
    delta_yaw_deg: float
    feature_count: int
    tracked_count: int
    depth_pair_count: int
    inlier_count: int
    inlier_ratio: float
    confidence: float
    camera_info_ready: bool
    depth_ready: bool
    dynamic_rejection: str
    note: str
    processing_ms: float


class RgbdVisualOdometry(Node):
    """Estimate short-term camera motion from RGB-D feature tracking.

    The idea is:
    - extract static-looking image features
    - track them with optical flow
    - use aligned depth to convert matched pixels into 3D points
    - estimate previous->current 3D transform with RANSAC
    - reject frames when too few inliers or motion is implausible

    Dynamic objects are handled as outliers. If too many tracked points move in
    a way that disagrees with the static scene, the frame is not used.
    """

    def __init__(self) -> None:
        super().__init__("rgbd_visual_odometry")

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic",
            "/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/camera/color/camera_info",
        )
        self.declare_parameter("output_topic", "/vision/visual_odom")
        self.declare_parameter("max_fps", 15.0)
        self.declare_parameter("display", True)
        self.declare_parameter("roi_y_min_ratio", 0.18)
        self.declare_parameter("roi_y_max_ratio", 0.98)
        self.declare_parameter("max_corners", 450)
        self.declare_parameter("quality_level", 0.01)
        self.declare_parameter("min_distance_px", 8.0)
        self.declare_parameter("block_size", 7)
        self.declare_parameter("min_tracked_points", 40)
        self.declare_parameter("min_depth_pairs", 20)
        self.declare_parameter("ransac_threshold_m", 0.045)
        self.declare_parameter("min_inlier_count", 14)
        self.declare_parameter("min_inlier_ratio", 0.45)
        self.declare_parameter("min_depth_m", 0.20)
        self.declare_parameter("max_depth_m", 4.0)
        self.declare_parameter("max_depth_delta_m", 0.45)
        self.declare_parameter("max_translation_per_frame_m", 0.25)
        self.declare_parameter("max_yaw_per_frame_deg", 18.0)
        self.declare_parameter("translation_scale", 1.0)
        self.declare_parameter("redetect_interval_frames", 8)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
        )
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.max_fps = float(self.get_parameter("max_fps").value)
        self.display = bool(self.get_parameter("display").value)
        self.roi_y_min_ratio = float(
            self.get_parameter("roi_y_min_ratio").value
        )
        self.roi_y_max_ratio = float(
            self.get_parameter("roi_y_max_ratio").value
        )
        self.max_corners = int(self.get_parameter("max_corners").value)
        self.quality_level = float(self.get_parameter("quality_level").value)
        self.min_distance_px = float(
            self.get_parameter("min_distance_px").value
        )
        self.block_size = int(self.get_parameter("block_size").value)
        self.min_tracked_points = int(
            self.get_parameter("min_tracked_points").value
        )
        self.min_depth_pairs = int(self.get_parameter("min_depth_pairs").value)
        self.ransac_threshold_m = float(
            self.get_parameter("ransac_threshold_m").value
        )
        self.min_inlier_count = int(
            self.get_parameter("min_inlier_count").value
        )
        self.min_inlier_ratio = float(
            self.get_parameter("min_inlier_ratio").value
        )
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.max_depth_delta_m = float(
            self.get_parameter("max_depth_delta_m").value
        )
        self.max_translation_per_frame_m = float(
            self.get_parameter("max_translation_per_frame_m").value
        )
        self.max_yaw_per_frame_deg = float(
            self.get_parameter("max_yaw_per_frame_deg").value
        )
        self.translation_scale = float(
            self.get_parameter("translation_scale").value
        )
        self.redetect_interval_frames = max(
            1, int(self.get_parameter("redetect_interval_frames").value)
        )

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(String, self.output_topic, 10)
        self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.depth_topic,
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._camera_info_callback, 10
        )

        self.latest_depth: np.ndarray | None = None
        self.latest_depth_time: float | None = None
        self.fx: float | None = None
        self.fy: float | None = None
        self.cx: float | None = None
        self.cy: float | None = None

        self.prev_gray: np.ndarray | None = None
        self.prev_depth: np.ndarray | None = None
        self.prev_pts: np.ndarray | None = None
        self.frame_index = 0
        self.last_process_time = 0.0

        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = 0.0
        self.total_distance_m = 0.0

        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(
                cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                30,
                0.01,
            ),
        )

        self.get_logger().info(f"Subscribing image: {self.image_topic}")
        self.get_logger().info(
            f"Subscribing aligned depth: {self.depth_topic}"
        )
        self.get_logger().info(
            f"Subscribing camera info: {self.camera_info_topic}"
        )
        self.get_logger().info(f"Publishing visual odom: {self.output_topic}")

    def _camera_info_callback(self, message: CameraInfo) -> None:
        if len(message.k) >= 6:
            self.fx = float(message.k[0])
            self.fy = float(message.k[4])
            self.cx = float(message.k[2])
            self.cy = float(message.k[5])

    def _depth_callback(self, message: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(
                message, desired_encoding="passthrough"
            )
            self.latest_depth = np.asarray(depth)
            self.latest_depth_time = time.monotonic()
        except Exception as exc:
            self.get_logger().warning(f"Could not read depth image: {exc}")

    def _depth_ready(self) -> bool:
        return (
            self.latest_depth is not None
            and self.latest_depth_time is not None
        )

    def _depth_to_meters(self, depth_value: Any) -> float:
        value = float(depth_value)
        if (
            self.latest_depth is not None
            and self.latest_depth.dtype == np.uint16
        ):
            value *= 0.001
        return value

    def _ensure_intrinsics(self, width: int, height: int) -> bool:
        if self.fx and self.fy and self.cx is not None and self.cy is not None:
            return True

        # Fallback only for testing. CameraInfo is strongly preferred.
        self.fx = max(width * 0.70, 1.0)
        self.fy = max(width * 0.70, 1.0)
        self.cx = width / 2.0
        self.cy = height / 2.0
        return False

    def _make_feature_mask(self, gray: np.ndarray) -> np.ndarray:
        height, width = gray.shape[:2]
        mask = np.zeros_like(gray, dtype=np.uint8)
        y1 = int(np.clip(self.roi_y_min_ratio, 0.0, 1.0) * height)
        y2 = int(np.clip(self.roi_y_max_ratio, 0.0, 1.0) * height)
        if y2 <= y1:
            y1, y2 = 0, height
        mask[y1:y2, :] = 255
        return mask

    def _detect_features(self, gray: np.ndarray) -> np.ndarray | None:
        mask = self._make_feature_mask(gray)
        points = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=self.max_corners,
            qualityLevel=self.quality_level,
            minDistance=self.min_distance_px,
            mask=mask,
            blockSize=self.block_size,
        )
        if points is None:
            return None
        return points.astype(np.float32)

    def _sample_depth(
        self, depth: np.ndarray, point: np.ndarray
    ) -> float | None:
        x = int(round(float(point[0])))
        y = int(round(float(point[1])))
        height, width = depth.shape[:2]
        if not (0 <= x < width and 0 <= y < height):
            return None

        radius = 2
        x1 = max(0, x - radius)
        x2 = min(width, x + radius + 1)
        y1 = max(0, y - radius)
        y2 = min(height, y + radius + 1)
        crop = depth[y1:y2, x1:x2]
        if crop.size == 0:
            return None

        crop_m = (
            crop.astype(np.float32) * 0.001
            if crop.dtype == np.uint16
            else crop.astype(np.float32)
        )
        valid = crop_m[
            np.isfinite(crop_m)
            & (crop_m >= self.min_depth_m)
            & (crop_m <= self.max_depth_m)
        ]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _backproject(self, point: np.ndarray, depth_m: float) -> np.ndarray:
        assert self.fx is not None and self.fy is not None
        assert self.cx is not None and self.cy is not None
        x_px = float(point[0])
        y_px = float(point[1])
        x = (x_px - self.cx) * depth_m / self.fx
        y = (y_px - self.cy) * depth_m / self.fy
        z = depth_m
        return np.array([x, y, z], dtype=np.float32)

    def _build_3d_pairs(
        self,
        prev_pts: np.ndarray,
        curr_pts: np.ndarray,
        prev_depth: np.ndarray,
        curr_depth: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        prev_3d: list[np.ndarray] = []
        curr_3d: list[np.ndarray] = []

        for prev_pt, curr_pt in zip(prev_pts, curr_pts):
            prev_d = self._sample_depth(prev_depth, prev_pt)
            curr_d = self._sample_depth(curr_depth, curr_pt)
            if prev_d is None or curr_d is None:
                continue
            if abs(prev_d - curr_d) > self.max_depth_delta_m:
                continue
            prev_3d.append(self._backproject(prev_pt, prev_d))
            curr_3d.append(self._backproject(curr_pt, curr_d))

        if not prev_3d:
            return np.empty((0, 3), dtype=np.float32), np.empty(
                (0, 3), dtype=np.float32
            )
        return np.asarray(prev_3d, dtype=np.float32), np.asarray(
            curr_3d, dtype=np.float32
        )

    @staticmethod
    def _yaw_from_rotation(rotation: np.ndarray) -> float:
        # Approximate yaw around the camera vertical axis. Full orientation
        # should come from a real SLAM/VIO stack.
        return -math.atan2(float(rotation[0, 2]), float(rotation[2, 2]))

    def _publish(self, info: VisualOdomInfo) -> None:
        message = String()
        message.data = json.dumps(
            asdict(info), ensure_ascii=True, separators=(",", ":")
        )
        self.publisher.publish(message)

    def _blank_info(
        self, note: str, processing_ms: float = 0.0
    ) -> VisualOdomInfo:
        return VisualOdomInfo(
            tracking_ok=False,
            update_used=False,
            x_m=round(self.x_m, 4),
            y_m=round(self.y_m, 4),
            yaw_deg=round(math.degrees(self.yaw_rad), 3),
            total_distance_m=round(self.total_distance_m, 4),
            delta_forward_m=0.0,
            delta_lateral_m=0.0,
            delta_yaw_deg=0.0,
            feature_count=(
                int(self.prev_pts.shape[0]) if self.prev_pts is not None else 0
            ),
            tracked_count=0,
            depth_pair_count=0,
            inlier_count=0,
            inlier_ratio=0.0,
            confidence=0.0,
            camera_info_ready=self.fx is not None,
            depth_ready=self._depth_ready(),
            dynamic_rejection="not_enough_data",
            note=note,
            processing_ms=round(processing_ms, 3),
        )

    def _update_pose(
        self, forward_m: float, lateral_m: float, yaw_delta_rad: float
    ) -> None:
        self.yaw_rad += yaw_delta_rad
        self.yaw_rad = math.atan2(
            math.sin(self.yaw_rad), math.cos(self.yaw_rad)
        )

        # Map convention for this experimental node: +y is forward at yaw=0,
        # +x is right/lateral. This is not the final IRC field coordinate yet.
        cos_yaw = math.cos(self.yaw_rad)
        sin_yaw = math.sin(self.yaw_rad)
        self.x_m += lateral_m * cos_yaw + forward_m * sin_yaw
        self.y_m += forward_m * cos_yaw - lateral_m * sin_yaw
        self.total_distance_m += math.hypot(forward_m, lateral_m)

    def _draw_debug(
        self,
        image: np.ndarray,
        prev_pts: np.ndarray | None,
        curr_pts: np.ndarray | None,
        inlier_mask: np.ndarray | None,
        info: VisualOdomInfo,
    ) -> None:
        if not self.display:
            return

        debug = image.copy()
        if prev_pts is not None and curr_pts is not None:
            mask = (
                inlier_mask.flatten().astype(bool)
                if inlier_mask is not None
                else np.zeros(len(curr_pts), dtype=bool)
            )
            for index, (prev_pt, curr_pt) in enumerate(
                zip(prev_pts, curr_pts)
            ):
                color = (
                    (0, 255, 0)
                    if index < len(mask) and mask[index]
                    else (0, 0, 255)
                )
                p1 = (int(prev_pt[0]), int(prev_pt[1]))
                p2 = (int(curr_pt[0]), int(curr_pt[1]))
                cv2.line(debug, p1, p2, color, 1, cv2.LINE_AA)
                cv2.circle(debug, p2, 2, color, -1, cv2.LINE_AA)

        lines = [
            f"RGB-D VO | {info.note}",
            f"x={info.x_m:.2f} y={info.y_m:.2f} yaw={info.yaw_deg:.1f}",
            f"dF={info.delta_forward_m:.3f} "
            f"dL={info.delta_lateral_m:.3f} "
            f"dYaw={info.delta_yaw_deg:.2f}",
            f"inliers={info.inlier_count}/{info.depth_pair_count} "
            f"conf={info.confidence:.2f}",
        ]
        for row, text in enumerate(lines):
            cv2.putText(
                debug,
                text,
                (12, 28 + row * 26),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
        cv2.imshow("RGBD Visual Odometry Debug", debug)
        cv2.waitKey(1)

    def _image_callback(self, message: Image) -> None:
        now = time.monotonic()
        if (
            self.max_fps > 0
            and now - self.last_process_time < 1.0 / self.max_fps
        ):
            return
        self.last_process_time = now
        started = time.perf_counter()

        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        except Exception as exc:
            self.get_logger().warning(f"Could not read color image: {exc}")
            return

        height, width = gray.shape[:2]
        camera_info_ready = self._ensure_intrinsics(width, height)
        curr_depth = (
            self.latest_depth.copy() if self.latest_depth is not None else None
        )

        if curr_depth is None:
            self.prev_gray = gray
            self.prev_depth = None
            self.prev_pts = self._detect_features(gray)
            elapsed = (time.perf_counter() - started) * 1000.0
            self._publish(
                self._blank_info("waiting_for_aligned_depth", elapsed)
            )
            return

        if (
            self.prev_gray is None
            or self.prev_depth is None
            or self.prev_pts is None
            or len(self.prev_pts) < self.min_tracked_points
        ):
            self.prev_gray = gray
            self.prev_depth = curr_depth
            self.prev_pts = self._detect_features(gray)
            elapsed = (time.perf_counter() - started) * 1000.0
            self._publish(
                self._blank_info("initialized_or_redetected_features", elapsed)
            )
            return

        next_pts, status, _error = cv2.calcOpticalFlowPyrLK(
            self.prev_gray,
            gray,
            self.prev_pts,
            None,
            **self.lk_params,
        )
        if next_pts is None or status is None:
            self.prev_gray = gray
            self.prev_depth = curr_depth
            self.prev_pts = self._detect_features(gray)
            elapsed = (time.perf_counter() - started) * 1000.0
            self._publish(self._blank_info("optical_flow_failed", elapsed))
            return

        good_prev = self.prev_pts[status.flatten() == 1].reshape(-1, 2)
        good_curr = next_pts[status.flatten() == 1].reshape(-1, 2)
        tracked_count = len(good_curr)

        if tracked_count < self.min_tracked_points:
            self.prev_gray = gray
            self.prev_depth = curr_depth
            self.prev_pts = self._detect_features(gray)
            elapsed = (time.perf_counter() - started) * 1000.0
            info = self._blank_info("too_few_tracked_features", elapsed)
            self._publish(info)
            self._draw_debug(image, good_prev, good_curr, None, info)
            return

        prev_3d, curr_3d = self._build_3d_pairs(
            good_prev, good_curr, self.prev_depth, curr_depth
        )
        depth_pair_count = len(prev_3d)
        if depth_pair_count < self.min_depth_pairs:
            self.prev_gray = gray
            self.prev_depth = curr_depth
            self.prev_pts = self._detect_features(gray)
            elapsed = (time.perf_counter() - started) * 1000.0
            info = self._blank_info("too_few_valid_depth_pairs", elapsed)
            (
                object.__setattr__(info, "tracked_count", tracked_count)
                if False
                else None
            )
            self._publish(info)
            self._draw_debug(image, good_prev, good_curr, None, info)
            return

        retval, transform, inliers = cv2.estimateAffine3D(
            prev_3d,
            curr_3d,
            ransacThreshold=self.ransac_threshold_m,
            confidence=0.99,
        )

        if retval == 0 or transform is None or inliers is None:
            self.prev_gray = gray
            self.prev_depth = curr_depth
            self.prev_pts = self._detect_features(gray)
            elapsed = (time.perf_counter() - started) * 1000.0
            info = self._blank_info("ransac_transform_failed", elapsed)
            self._publish(info)
            self._draw_debug(image, good_prev, good_curr, None, info)
            return

        inlier_count = int(np.count_nonzero(inliers))
        inlier_ratio = inlier_count / max(depth_pair_count, 1)
        rotation = transform[:3, :3]
        translation = transform[:3, 3]

        # Static scene transform maps previous camera coordinates to current
        # camera coordinates. Camera motion is the inverse translation.
        camera_motion = -rotation.T @ translation
        lateral_m = float(camera_motion[0]) * self.translation_scale
        forward_m = float(camera_motion[2]) * self.translation_scale
        yaw_delta_rad = self._yaw_from_rotation(rotation)

        translation_norm = math.hypot(forward_m, lateral_m)
        yaw_delta_deg = math.degrees(yaw_delta_rad)
        plausible = (
            inlier_count >= self.min_inlier_count
            and inlier_ratio >= self.min_inlier_ratio
            and translation_norm <= self.max_translation_per_frame_m
            and abs(yaw_delta_deg) <= self.max_yaw_per_frame_deg
        )

        note = "vo_update_used" if plausible else "vo_update_rejected"
        if plausible:
            self._update_pose(forward_m, lateral_m, yaw_delta_rad)
        else:
            forward_m = 0.0
            lateral_m = 0.0
            yaw_delta_rad = 0.0
            yaw_delta_deg = 0.0

        confidence = min(
            1.0, max(0.0, inlier_ratio * min(inlier_count / 80.0, 1.0))
        )
        elapsed = (time.perf_counter() - started) * 1000.0
        info = VisualOdomInfo(
            tracking_ok=True,
            update_used=plausible,
            x_m=round(self.x_m, 4),
            y_m=round(self.y_m, 4),
            yaw_deg=round(math.degrees(self.yaw_rad), 3),
            total_distance_m=round(self.total_distance_m, 4),
            delta_forward_m=round(forward_m, 4),
            delta_lateral_m=round(lateral_m, 4),
            delta_yaw_deg=round(yaw_delta_deg, 3),
            feature_count=(
                int(self.prev_pts.shape[0]) if self.prev_pts is not None else 0
            ),
            tracked_count=tracked_count,
            depth_pair_count=depth_pair_count,
            inlier_count=inlier_count,
            inlier_ratio=round(inlier_ratio, 4),
            confidence=round(confidence, 4),
            camera_info_ready=camera_info_ready,
            depth_ready=True,
            dynamic_rejection="ransac_inliers_only",
            note=note,
            processing_ms=round(elapsed, 3),
        )
        self._publish(info)
        self._draw_debug(image, good_prev, good_curr, inliers, info)

        self.frame_index += 1
        if (
            self.frame_index % self.redetect_interval_frames == 0
            or inlier_count < self.min_tracked_points
        ):
            self.prev_pts = self._detect_features(gray)
        else:
            self.prev_pts = good_curr.reshape(-1, 1, 2).astype(np.float32)
        self.prev_gray = gray
        self.prev_depth = curr_depth

    def destroy_node(self) -> bool:
        if self.display:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: RgbdVisualOdometry | None = None
    try:
        node = RgbdVisualOdometry()
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
