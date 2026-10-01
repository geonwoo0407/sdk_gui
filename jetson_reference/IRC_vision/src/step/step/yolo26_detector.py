#!/usr/bin/env python3
"""ROS 2 node for YOLO26 object detection on a RealSense color topic."""

from __future__ import annotations

import ast
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any

import cv2
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from cv_bridge import CvBridge
import numpy as np
try:
    import onnxruntime as ort
except ImportError:  # TensorRT .engine production does not require ONNX.
    ort = None
import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from step.depth_frame_cache import image_stamp_ns
from step.line_navigation_planner import numbered_turn_motion_metadata
from step.tensorrt_backend import TensorRTBackend


def _default_model_path() -> str:
    """Return the installed model, with a source-tree development fallback."""
    try:
        package_share = Path(get_package_share_directory("step"))
        return str(package_share / "models" / "best.onnx")
    except PackageNotFoundError:
        source_root = Path(__file__).resolve().parents[1]
        return str(source_root / "models" / "best.onnx")


DEFAULT_MODEL_PATH = _default_model_path()
DEFAULT_CLASS_NAMES = [
    "line",
    "ball",
    "goal",
    "backboard",
    "hurdle",
    "grab",
]
DISPLAY_WINDOW_NAME = "YOLO26 RealSense Detection"


@dataclass(frozen=True)
class CameraControlSpec:
    """Mapping from one OpenCV trackbar to a RealSense ROS parameter."""

    label: str
    parameter_name: str
    minimum: int
    maximum: int
    default: int
    value_type: str = "integer"

    @property
    def maximum_position(self) -> int:
        return (
            self.maximum - self.minimum
            if self.minimum < 0
            else self.maximum
        )

    def position_from_value(self, value: int) -> int:
        position = value - self.minimum if self.minimum < 0 else value
        return max(0, min(self.maximum_position, position))

    def value_from_position(self, position: int) -> bool | int | float:
        value = self.minimum + position if self.minimum < 0 else position
        value = max(self.minimum, min(self.maximum, value))
        if self.value_type == "boolean":
            return bool(value)
        if self.value_type == "double":
            return float(value)
        return int(value)


@dataclass(frozen=True)
class Detection:
    """One object detection in original-image pixel coordinates."""

    class_id: int
    class_name: str
    confidence: float
    bbox: list[int]
    center: list[int]


@dataclass(frozen=True)
class LetterboxInfo:
    """Geometry used to map model coordinates back to the source image."""

    scale: float
    pad_x: float
    pad_y: float


class Yolo26Detector(Node):
    """Run a YOLO26 ONNX model on incoming ROS color images."""

    def __init__(self) -> None:
        super().__init__("yolo26_detector")

        self.declare_parameter("model_path", DEFAULT_MODEL_PATH)
        self.declare_parameter(
            "image_topic", "/camera/color/image_raw"
        )
        self.declare_parameter("detections_topic", "/vision/detections")
        self.declare_parameter(
            "annotated_image_topic", "/vision/detections/image"
        )
        self.declare_parameter("line_info_topic", "/vision/line_info")
        self.declare_parameter("line_info_timeout_sec", 0.8)
        self.declare_parameter("show_line_metrics", True)
        self.declare_parameter("ball_info_topic", "/vision/ball_info")
        self.declare_parameter("ball_info_timeout_sec", 0.8)
        self.declare_parameter("show_ball_metrics", True)
        self.declare_parameter("ball_tracking_range_m", 1.5)
        self.declare_parameter("ball_control_range_m", 1.5)
        self.declare_parameter("goal_info_topic", "/vision/goal_info")
        self.declare_parameter("goal_info_timeout_sec", 0.8)
        self.declare_parameter("show_goal_metrics", True)
        self.declare_parameter("goal_tracking_range_m", 1.0)
        self.declare_parameter("goal_control_range_m", 0.5)
        self.declare_parameter("hurdle_info_topic", "/vision/hurdle_info")
        self.declare_parameter("hurdle_info_timeout_sec", 0.8)
        self.declare_parameter("show_hurdle_metrics", True)
        self.declare_parameter("hurdle_tracking_range_m", 1.0)
        self.declare_parameter("hurdle_control_range_m", 1.0)
        self.declare_parameter(
            "motion_command_topic",
            "/navigation/motion_command",
        )
        self.declare_parameter("motion_command_timeout_sec", 0.8)
        self.declare_parameter(
            "decision_debug_topic",
            "/navigation/decision_debug",
        )
        self.declare_parameter("decision_debug_timeout_sec", 0.5)
        self.declare_parameter("overlay_max_stamp_delta_sec", 0.05)
        self.declare_parameter("metrics_mode", "auto")
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("ball_confidence_threshold", 0.20)
        # Hurdle false positives can preempt every mission phase.  Keep the
        # raw detector threshold aligned with the analyzer/controller safety
        # threshold instead of publishing weak hurdle boxes.
        self.declare_parameter("hurdle_confidence_threshold", 0.60)
        self.declare_parameter("max_detections", 300)
        self.declare_parameter("max_fps", 15.0)
        self.declare_parameter("device", "auto")
        self.declare_parameter("display", True)
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("show_camera_controls", True)
        self.declare_parameter("camera_node_name", "/camera/camera")
        self.declare_parameter("camera_control_apply_interval_sec", 0.15)

        self.model_path = Path(
            self.get_parameter("model_path").value
        ).expanduser()
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.ball_confidence_threshold = float(
            self.get_parameter("ball_confidence_threshold").value
        )
        self.hurdle_confidence_threshold = float(
            self.get_parameter("hurdle_confidence_threshold").value
        )
        self.max_detections = int(
            self.get_parameter("max_detections").value
        )
        self.max_fps = float(self.get_parameter("max_fps").value)
        self.display = bool(self.get_parameter("display").value)
        self.publish_annotated_image = bool(
            self.get_parameter("publish_annotated_image").value
        )
        self.show_camera_controls = bool(
            self.get_parameter("show_camera_controls").value
        )
        self.camera_node_name = str(
            self.get_parameter("camera_node_name").value
        )
        self.camera_control_apply_interval_sec = max(
            0.05,
            float(
                self.get_parameter(
                    "camera_control_apply_interval_sec"
                ).value
            ),
        )
        self.line_info_timeout_sec = max(
            0.1,
            float(self.get_parameter("line_info_timeout_sec").value),
        )
        self.show_line_metrics = bool(
            self.get_parameter("show_line_metrics").value
        )
        self.ball_info_timeout_sec = max(
            0.1,
            float(self.get_parameter("ball_info_timeout_sec").value),
        )
        self.show_ball_metrics = bool(
            self.get_parameter("show_ball_metrics").value
        )
        self.ball_tracking_range_m = float(
            self.get_parameter("ball_tracking_range_m").value
        )
        self.ball_control_range_m = float(
            self.get_parameter("ball_control_range_m").value
        )
        self.goal_info_timeout_sec = max(
            0.1,
            float(self.get_parameter("goal_info_timeout_sec").value),
        )
        self.show_goal_metrics = bool(
            self.get_parameter("show_goal_metrics").value
        )
        self.goal_tracking_range_m = float(
            self.get_parameter("goal_tracking_range_m").value
        )
        self.goal_control_range_m = float(
            self.get_parameter("goal_control_range_m").value
        )
        self.hurdle_info_timeout_sec = max(
            0.1,
            float(self.get_parameter("hurdle_info_timeout_sec").value),
        )
        self.show_hurdle_metrics = bool(
            self.get_parameter("show_hurdle_metrics").value
        )
        self.hurdle_tracking_range_m = float(
            self.get_parameter("hurdle_tracking_range_m").value
        )
        self.hurdle_control_range_m = float(
            self.get_parameter("hurdle_control_range_m").value
        )
        self.motion_command_timeout_sec = max(
            0.1,
            float(self.get_parameter("motion_command_timeout_sec").value),
        )
        self.decision_debug_timeout_sec = max(
            0.1,
            float(self.get_parameter("decision_debug_timeout_sec").value),
        )
        self.overlay_max_stamp_delta_sec = max(
            0.0,
            float(
                self.get_parameter("overlay_max_stamp_delta_sec").value
            ),
        )
        self.metrics_mode = str(
            self.get_parameter("metrics_mode").value
        ).strip().lower()
        if self.metrics_mode not in {
            "auto",
            "line",
            "ball",
            "goal",
            "hurdle",
        }:
            raise ValueError(
                "metrics_mode must be auto, line, ball, goal, or hurdle"
            )

        if not self.model_path.is_file():
            raise FileNotFoundError(
                f"YOLO26 model was not found: {self.model_path}"
            )
        if not 0.0 <= self.confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold must be between 0 and 1")
        if not 0.0 <= self.ball_confidence_threshold <= 1.0:
            raise ValueError(
                "ball_confidence_threshold must be between 0 and 1"
            )
        if not 0.0 <= self.hurdle_confidence_threshold <= 1.0:
            raise ValueError(
                "hurdle_confidence_threshold must be between 0 and 1"
            )

        self.bridge = CvBridge()
        self.session: ort.InferenceSession | None = None
        self.tensorrt_backend: TensorRTBackend | None = None
        requested_device = str(self.get_parameter("device").value)
        self.requested_device = requested_device
        self.onnx_class_names_metadata: str | None = None
        model_suffix = self.model_path.suffix.lower()
        if model_suffix == ".onnx":
            self.backend_name = "ONNX Runtime"
            self.session, self.active_provider = self._create_session(
                requested_device
            )
            self.input_name = self.session.get_inputs()[0].name
            input_shape = self.session.get_inputs()[0].shape
            self.onnx_class_names_metadata = (
                self.session.get_modelmeta().custom_metadata_map.get("names")
            )
            self.class_names = self._read_class_names()
        elif model_suffix == ".engine":
            device = requested_device.strip().lower()
            if device not in {"auto", "tensorrt", "cuda", "cpu"}:
                raise ValueError("device must be auto, tensorrt, cuda, or cpu")
            if device == "cpu":
                self.get_logger().warning(
                    "device=cpu is ignored for a TensorRT engine; using GPU"
                )
            self.backend_name = "TensorRT engine"
            self.tensorrt_backend = TensorRTBackend(self.model_path)
            self.active_provider = "TensorRT"
            self.input_name = self.tensorrt_backend.input_name
            input_shape = self.tensorrt_backend.input_shape
            self.class_names = DEFAULT_CLASS_NAMES.copy()
        else:
            raise ValueError(
                "model_path must end in .onnx or .engine; "
                f"got {self.model_path}"
            )
        self.input_height = self._fixed_dimension(input_shape[2], 640)
        self.input_width = self._fixed_dimension(input_shape[3], 640)

        detections_topic = str(
            self.get_parameter("detections_topic").value
        )
        annotated_topic = str(
            self.get_parameter("annotated_image_topic").value
        )
        image_topic = str(self.get_parameter("image_topic").value)
        line_info_topic = str(
            self.get_parameter("line_info_topic").value
        )
        ball_info_topic = str(
            self.get_parameter("ball_info_topic").value
        )
        goal_info_topic = str(
            self.get_parameter("goal_info_topic").value
        )
        hurdle_info_topic = str(
            self.get_parameter("hurdle_info_topic").value
        )
        motion_command_topic = str(
            self.get_parameter("motion_command_topic").value
        )
        decision_debug_topic = str(
            self.get_parameter("decision_debug_topic").value
        )

        self.detections_publisher = self.create_publisher(
            String, detections_topic, 10
        )
        self.annotated_publisher = self.create_publisher(
            Image, annotated_topic, qos_profile_sensor_data
        )
        latest_image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            latest_image_qos,
        )
        self.create_subscription(
            String,
            line_info_topic,
            self._line_info_callback,
            10,
        )
        self.create_subscription(
            String,
            ball_info_topic,
            self._ball_info_callback,
            10,
        )
        self.create_subscription(
            String,
            goal_info_topic,
            self._goal_info_callback,
            10,
        )
        self.create_subscription(
            String,
            hurdle_info_topic,
            self._hurdle_info_callback,
            10,
        )
        self.create_subscription(
            String,
            motion_command_topic,
            self._motion_command_callback,
            10,
        )
        self.create_subscription(
            String,
            decision_debug_topic,
            self._decision_debug_callback,
            10,
        )

        self.last_inference_time = 0.0
        self.smoothed_fps = 0.0
        self.processing = False
        self.inference_timing_samples_ms: list[float] = []
        self.inference_timing_window_started = time.monotonic()
        self.inference_timing_last_log = self.inference_timing_window_started
        self.latest_line_info: dict[str, Any] | None = None
        self.latest_line_info_time: float | None = None
        self.latest_ball_info: dict[str, Any] | None = None
        self.latest_ball_info_time: float | None = None
        self.latest_goal_info: dict[str, Any] | None = None
        self.latest_goal_info_time: float | None = None
        self.latest_hurdle_info: dict[str, Any] | None = None
        self.latest_hurdle_info_time: float | None = None
        self.latest_motion_command: dict[str, Any] | None = None
        self.latest_motion_command_time: float | None = None
        self.latest_decision_debug: dict[str, Any] | None = None
        self.latest_decision_debug_time: float | None = None
        self._overlay_rgb_stamp_ns: int | None = None
        self._ball_info_stamp_delta_ms: float | None = None

        self.camera_control_specs = self._camera_control_specs()
        self.camera_control_values = {
            spec.parameter_name: spec.value_from_position(
                spec.position_from_value(spec.default)
            )
            for spec in self.camera_control_specs
        }
        self.camera_control_regions: dict[
            str, tuple[CameraControlSpec, str, int, int, int, int]
        ] = {}
        self.camera_control_dragging: str | None = None
        self.pending_camera_controls: dict[str, bool | int | float] = {}
        self.camera_control_future = None
        camera_service = (
            self.camera_node_name.rstrip("/") + "/set_parameters"
        )
        self.camera_control_client = self.create_client(
            SetParameters,
            camera_service,
        )
        self.camera_control_timer = self.create_timer(
            self.camera_control_apply_interval_sec,
            self._apply_pending_camera_controls,
        )
        if self.display and self.show_camera_controls:
            try:
                self._create_camera_control_panel()
            except cv2.error as exc:
                self.get_logger().warning(
                    "OpenCV display unavailable; continuing with detector "
                    f"display disabled: {exc}"
                )
                self.display = False
                self.show_camera_controls = False

        available_providers = (
            ort.get_available_providers() if ort is not None else []
        )
        session_providers = (
            self.session.get_providers() if self.session is not None else []
        )
        requested_device_normalized = requested_device.strip().lower()
        cpu_fallback = bool(
            model_suffix == ".onnx"
            and requested_device_normalized != "cpu"
            and self.active_provider == "CPUExecutionProvider"
        )
        self.get_logger().info(
            "[YOLO_RUNTIME] "
            f"model={self.model_path.resolve()} "
            f"sha256={self._file_sha256(self.model_path)} "
            f"requested_device={requested_device_normalized} "
            f"available_providers={available_providers} "
            f"session_providers={session_providers} "
            f"active_provider={self.active_provider} "
            f"cpu_fallback={str(cpu_fallback).lower()} "
            f"onnx_names_metadata={self.onnx_class_names_metadata!r}"
        )
        self.get_logger().info(f"Backend: {self.backend_name}")
        self.get_logger().info(
            f"Input: {self.input_width}x{self.input_height}"
        )
        self.get_logger().info(f"Classes: {self.class_names}")
        self.get_logger().info(f"Subscribing: {image_topic}")
        self.get_logger().info(f"Line metrics: {line_info_topic}")
        self.get_logger().info(f"Ball metrics: {ball_info_topic}")
        self.get_logger().info(f"Goal metrics: {goal_info_topic}")
        self.get_logger().info(f"Hurdle metrics: {hurdle_info_topic}")
        self.get_logger().info(f"Motion decision: {motion_command_topic}")
        self.get_logger().info(f"Decision debug: {decision_debug_topic}")
        if self.display and self.show_camera_controls:
            self.get_logger().info(
                f"Camera sliders: {self.camera_node_name}"
            )
        self.get_logger().info(f"Publishing: {detections_topic}")

    @staticmethod
    def _camera_control_specs() -> tuple[CameraControlSpec, ...]:
        """Return the live color controls exposed by the D435i driver."""
        return (
            CameraControlSpec(
                "Auto Exposure",
                "rgb_camera.enable_auto_exposure",
                0,
                1,
                1,
                "boolean",
            ),
            CameraControlSpec(
                "Exposure",
                "rgb_camera.exposure",
                1,
                10000,
                156,
            ),
            CameraControlSpec(
                "Gain",
                "rgb_camera.gain",
                0,
                128,
                64,
            ),
            CameraControlSpec(
                "Brightness",
                "rgb_camera.brightness",
                -64,
                64,
                0,
            ),
            CameraControlSpec(
                "Contrast",
                "rgb_camera.contrast",
                0,
                100,
                50,
            ),
            CameraControlSpec(
                "Saturation",
                "rgb_camera.saturation",
                0,
                100,
                64,
            ),
            CameraControlSpec(
                "Sharpness",
                "rgb_camera.sharpness",
                0,
                100,
                50,
            ),
            CameraControlSpec(
                "Auto White Balance",
                "rgb_camera.enable_auto_white_balance",
                0,
                1,
                1,
                "boolean",
            ),
            CameraControlSpec(
                "White Balance",
                "rgb_camera.white_balance",
                2800,
                6500,
                4000,
                "double",
            ),
            CameraControlSpec(
                "Power Line",
                "rgb_camera.power_line_frequency",
                0,
                3,
                3,
            ),
        )

    def _create_camera_control_panel(self) -> None:
        """Create the YOLO window and attach the compact panel mouse handler."""
        cv2.namedWindow(DISPLAY_WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(
            DISPLAY_WINDOW_NAME,
            self._camera_control_mouse_callback,
        )

    def _set_camera_control_value(
        self,
        spec: CameraControlSpec,
        value: bool | int | float,
        *,
        manual: bool = True,
    ) -> None:
        """Update the panel state and queue one RealSense parameter."""
        if spec.value_type == "boolean":
            converted: bool | int | float = bool(value)
        else:
            numeric = max(spec.minimum, min(spec.maximum, float(value)))
            converted = float(numeric) if spec.value_type == "double" else int(
                round(numeric)
            )
        self.camera_control_values[spec.parameter_name] = converted
        self.pending_camera_controls[spec.parameter_name] = converted

        if not manual:
            return
        if spec.parameter_name in {
            "rgb_camera.exposure",
            "rgb_camera.gain",
        }:
            self._disable_auto_control("rgb_camera.enable_auto_exposure")
        elif spec.parameter_name == "rgb_camera.white_balance":
            self._disable_auto_control(
                "rgb_camera.enable_auto_white_balance"
            )

    def _disable_auto_control(self, parameter_name: str) -> None:
        """Switch the matching auto mode off after a manual adjustment."""
        self.camera_control_values[parameter_name] = False
        self.pending_camera_controls[parameter_name] = False

    def _camera_control_mouse_callback(
        self,
        event: int,
        x: int,
        y: int,
        flags: int,
        _userdata: Any,
    ) -> None:
        """Handle compact sliders, toggle buttons, and power-line choices."""
        if event == cv2.EVENT_LBUTTONDOWN:
            for name, region in self.camera_control_regions.items():
                spec, kind, x1, y1, x2, y2 = region
                if x1 <= x <= x2 and y1 <= y <= y2:
                    if kind == "toggle":
                        current = bool(self.camera_control_values[name])
                        self._set_camera_control_value(spec, not current)
                    elif kind == "segments":
                        width = max(1, x2 - x1 + 1)
                        choice = min(3, max(0, int(4 * (x - x1) / width)))
                        self._set_camera_control_value(spec, choice)
                    else:
                        self.camera_control_dragging = name
                        self._update_camera_slider(spec, x, x1, x2)
                    return
        elif event == cv2.EVENT_MOUSEMOVE:
            if not (flags & cv2.EVENT_FLAG_LBUTTON):
                return
            name = self.camera_control_dragging
            if name is None or name not in self.camera_control_regions:
                return
            spec, kind, x1, _y1, x2, _y2 = self.camera_control_regions[name]
            if kind == "slider":
                self._update_camera_slider(spec, x, x1, x2)
        elif event == cv2.EVENT_LBUTTONUP:
            self.camera_control_dragging = None

    def _update_camera_slider(
        self,
        spec: CameraControlSpec,
        x: int,
        x1: int,
        x2: int,
    ) -> None:
        """Convert a mouse position on a compact slider to its real value."""
        ratio = (max(x1, min(x2, x)) - x1) / max(1, x2 - x1)
        value = spec.minimum + ratio * (spec.maximum - spec.minimum)
        self._set_camera_control_value(spec, value)

    def _apply_pending_camera_controls(self) -> None:
        """Send coalesced slider updates to the RealSense node."""
        if self.camera_control_future is not None:
            if not self.camera_control_future.done():
                return
            try:
                response = self.camera_control_future.result()
                failures = [
                    result.reason
                    for result in response.results
                    if not result.successful
                ]
                if failures:
                    self.get_logger().warning(
                        "Camera control rejected: " + "; ".join(failures)
                    )
            except Exception as exc:
                self.get_logger().warning(
                    f"Camera control request failed: {exc}"
                )
            self.camera_control_future = None

        if not self.pending_camera_controls:
            return
        if not self.camera_control_client.service_is_ready():
            return

        updates = self.pending_camera_controls
        self.pending_camera_controls = {}
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name=name, value=value).to_parameter_msg()
            for name, value in updates.items()
        ]
        self.camera_control_future = self.camera_control_client.call_async(
            request
        )

    def _reset_camera_controls(self) -> None:
        """Restore the documented D435i color defaults after pressing R."""
        defaults: dict[str, bool | int | float] = {}
        for spec in self.camera_control_specs:
            position = spec.position_from_value(spec.default)
            value = spec.value_from_position(position)
            defaults[spec.parameter_name] = value
            self.camera_control_values[spec.parameter_name] = value
        auto_names = {
            "rgb_camera.enable_auto_exposure",
            "rgb_camera.enable_auto_white_balance",
        }
        for name, value in defaults.items():
            if name not in auto_names:
                self.pending_camera_controls[name] = value
        for name in auto_names:
            self.pending_camera_controls[name] = defaults[name]
        self.get_logger().info("Camera color controls reset to defaults")

    def _draw_camera_control_panel(self, image: np.ndarray) -> np.ndarray:
        """Append a compact two-column camera control panel to the image."""
        panel_height = 188
        height, width = image.shape[:2]
        canvas = np.full(
            (height + panel_height, width, 3),
            (28, 31, 36),
            dtype=np.uint8,
        )
        canvas[:height, :width] = image
        cv2.line(canvas, (0, height), (width, height), (70, 75, 82), 1)
        cv2.putText(
            canvas,
            "CAMERA CONTROLS",
            (16, height + 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (235, 235, 235),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            "drag sliders  |  click buttons  |  R reset",
            (205, height + 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (155, 160, 168),
            1,
            cv2.LINE_AA,
        )

        specs = {spec.parameter_name: spec for spec in self.camera_control_specs}
        columns = (
            (
                "rgb_camera.enable_auto_exposure",
                "rgb_camera.exposure",
                "rgb_camera.gain",
                "rgb_camera.brightness",
                "rgb_camera.contrast",
            ),
            (
                "rgb_camera.enable_auto_white_balance",
                "rgb_camera.white_balance",
                "rgb_camera.saturation",
                "rgb_camera.sharpness",
                "rgb_camera.power_line_frequency",
            ),
        )
        self.camera_control_regions = {}
        column_width = width // 2
        for column_index, names in enumerate(columns):
            column_x = column_index * column_width
            label_x = column_x + 16
            control_x1 = column_x + min(190, max(145, column_width // 3))
            control_x2 = column_x + column_width - 24
            for row_index, name in enumerate(names):
                spec = specs[name]
                center_y = height + 48 + row_index * 28
                value = self.camera_control_values[name]
                cv2.putText(
                    canvas,
                    spec.label,
                    (label_x, center_y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.44,
                    (215, 218, 222),
                    1,
                    cv2.LINE_AA,
                )
                if spec.value_type == "boolean":
                    self._draw_camera_toggle(
                        canvas,
                        spec,
                        bool(value),
                        control_x1,
                        center_y,
                    )
                elif name == "rgb_camera.power_line_frequency":
                    self._draw_power_line_choices(
                        canvas,
                        spec,
                        int(value),
                        control_x1,
                        control_x2,
                        center_y,
                    )
                else:
                    self._draw_camera_slider(
                        canvas,
                        spec,
                        float(value),
                        control_x1,
                        control_x2,
                        center_y,
                    )
        return canvas

    def _draw_camera_toggle(
        self,
        image: np.ndarray,
        spec: CameraControlSpec,
        enabled: bool,
        x: int,
        center_y: int,
    ) -> None:
        """Draw one clickable ON/OFF button."""
        x2 = x + 82
        y1, y2 = center_y - 10, center_y + 11
        color = (50, 165, 70) if enabled else (65, 70, 78)
        cv2.rectangle(image, (x, y1), (x2, y2), color, -1)
        cv2.rectangle(image, (x, y1), (x2, y2), (115, 120, 128), 1)
        text = "ON" if enabled else "OFF"
        cv2.putText(
            image,
            text,
            (x + 27, center_y + 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        self.camera_control_regions[spec.parameter_name] = (
            spec,
            "toggle",
            x,
            y1,
            x2,
            y2,
        )

    def _draw_camera_slider(
        self,
        image: np.ndarray,
        spec: CameraControlSpec,
        value: float,
        x1: int,
        x2: int,
        center_y: int,
    ) -> None:
        """Draw one normalized compact slider and its real numeric value."""
        value_width = 62
        rail_x2 = max(x1 + 40, x2 - value_width)
        ratio = (value - spec.minimum) / max(1, spec.maximum - spec.minimum)
        thumb_x = int(round(x1 + ratio * (rail_x2 - x1)))
        cv2.line(image, (x1, center_y), (rail_x2, center_y), (92, 98, 108), 4)
        cv2.line(image, (x1, center_y), (thumb_x, center_y), (0, 190, 255), 4)
        cv2.circle(image, (thumb_x, center_y), 7, (245, 245, 245), -1)
        cv2.circle(image, (thumb_x, center_y), 7, (70, 75, 82), 1)
        shown_value = int(round(value))
        cv2.putText(
            image,
            str(shown_value),
            (rail_x2 + 13, center_y + 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (235, 235, 235),
            1,
            cv2.LINE_AA,
        )
        self.camera_control_regions[spec.parameter_name] = (
            spec,
            "slider",
            x1,
            center_y - 11,
            rail_x2,
            center_y + 11,
        )

    def _draw_power_line_choices(
        self,
        image: np.ndarray,
        spec: CameraControlSpec,
        selected: int,
        x1: int,
        x2: int,
        center_y: int,
    ) -> None:
        """Draw OFF/50/60/AUTO as four compact selectable buttons."""
        labels = ("OFF", "50", "60", "AUTO")
        total_width = max(120, x2 - x1)
        segment_width = total_width / 4.0
        y1, y2 = center_y - 10, center_y + 11
        for index, label in enumerate(labels):
            left = int(round(x1 + index * segment_width))
            right = int(round(x1 + (index + 1) * segment_width))
            color = (0, 150, 215) if index == selected else (55, 60, 68)
            cv2.rectangle(image, (left, y1), (right, y2), color, -1)
            cv2.rectangle(image, (left, y1), (right, y2), (105, 110, 118), 1)
            text_x = left + max(4, (right - left - len(label) * 8) // 2)
            cv2.putText(
                image,
                label,
                (text_x, center_y + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        self.camera_control_regions[spec.parameter_name] = (
            spec,
            "segments",
            x1,
            y1,
            x1 + total_width,
            y2,
        )

    def _line_info_callback(self, message: String) -> None:
        """Store the latest analyzed line geometry for the display."""
        payload = self._read_json_object(message, "line info")
        if payload is None:
            return
        self.latest_line_info = payload
        self.latest_line_info_time = time.monotonic()

    def _fresh_line_info(self) -> dict[str, Any] | None:
        """Return fresh line information, or None after its timeout."""
        if self.latest_line_info is None or self.latest_line_info_time is None:
            return None
        age = time.monotonic() - self.latest_line_info_time
        if age > self.line_info_timeout_sec:
            return None
        return self.latest_line_info

    def _motion_command_callback(self, message: String) -> None:
        """Store the command actually selected by motion_decision_node."""
        payload = self._read_json_object(message, "motion command")
        if payload is None:
            return
        self.latest_motion_command = payload
        self.latest_motion_command_time = time.monotonic()

    def _fresh_motion_command(self) -> dict[str, Any] | None:
        if (
            self.latest_motion_command is None
            or self.latest_motion_command_time is None
        ):
            return None
        age = time.monotonic() - self.latest_motion_command_time
        if age > self.motion_command_timeout_sec:
            return None
        return self.latest_motion_command

    def _decision_debug_callback(self, message: String) -> None:
        """Store decision state used only for the local debug overlay."""
        payload = self._read_json_object(message, "decision debug")
        if payload is None:
            return
        self.latest_decision_debug = payload
        self.latest_decision_debug_time = time.monotonic()

    def _fresh_decision_debug(self) -> dict[str, Any] | None:
        """Return recent decision state, avoiding a stale grasp banner."""
        if (
            self.latest_decision_debug is None
            or self.latest_decision_debug_time is None
        ):
            return None
        age = time.monotonic() - self.latest_decision_debug_time
        if age > self.decision_debug_timeout_sec:
            return None
        return self.latest_decision_debug

    def _read_json_object(
        self,
        message: String,
        label: str,
    ) -> dict[str, Any] | None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError(f"{label} must be a JSON object")
            return payload
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Invalid {label}: {exc}")
            return None

    def _ball_info_callback(self, message: String) -> None:
        """Store the latest analyzed ball geometry for the display."""
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("ball info must be a JSON object")
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Invalid ball info: {exc}")
            return
        self.latest_ball_info = payload
        self.latest_ball_info_time = time.monotonic()

    def _recent_ball_info(self) -> dict[str, Any] | None:
        """Return receipt-fresh ball status without matching RGB geometry."""
        if (
            self.latest_ball_info is None
            or self.latest_ball_info_time is None
        ):
            return None
        age = time.monotonic() - self.latest_ball_info_time
        if age > self.ball_info_timeout_sec:
            return None
        return self.latest_ball_info

    def _fresh_ball_info(self) -> dict[str, Any] | None:
        """Return ball information fresh in both receipt and RGB time."""
        self._ball_info_stamp_delta_ms = None
        recent_info = self._recent_ball_info()
        if recent_info is None:
            return None
        expected_stamp_ns = getattr(self, "_overlay_rgb_stamp_ns", None)
        if expected_stamp_ns is not None:
            try:
                info_stamp_ns = int(recent_info["rgb_stamp_ns"])
            except (KeyError, TypeError, ValueError):
                self._ball_info_stamp_delta_ms = None
                return None
            delta_ms = abs(info_stamp_ns - expected_stamp_ns) / 1_000_000.0
            self._ball_info_stamp_delta_ms = delta_ms
            max_delta_ms = (
                getattr(self, "overlay_max_stamp_delta_sec", 0.05)
                * 1000.0
            )
            if delta_ms > max_delta_ms:
                return None
        return recent_info

    def _goal_info_callback(self, message: String) -> None:
        """Store the latest analyzed goal geometry for the display."""
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("goal info must be a JSON object")
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Invalid goal info: {exc}")
            return
        self.latest_goal_info = payload
        self.latest_goal_info_time = time.monotonic()

    def _fresh_goal_info(self) -> dict[str, Any] | None:
        """Return fresh goal information, or None after its timeout."""
        if (
            self.latest_goal_info is None
            or self.latest_goal_info_time is None
        ):
            return None
        age = time.monotonic() - self.latest_goal_info_time
        if age > self.goal_info_timeout_sec:
            return None
        return self.latest_goal_info

    def _hurdle_info_callback(self, message: String) -> None:
        """Store the latest analyzed hurdle geometry for the display."""
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("hurdle info must be a JSON object")
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Invalid hurdle info: {exc}")
            return
        self.latest_hurdle_info = payload
        self.latest_hurdle_info_time = time.monotonic()

    def _fresh_hurdle_info(self) -> dict[str, Any] | None:
        """Return fresh hurdle information, or None after its timeout."""
        if (
            self.latest_hurdle_info is None
            or self.latest_hurdle_info_time is None
        ):
            return None
        age = time.monotonic() - self.latest_hurdle_info_time
        if age > self.hurdle_info_timeout_sec:
            return None
        return self.latest_hurdle_info

    @staticmethod
    def _fixed_dimension(value: Any, fallback: int) -> int:
        return int(value) if isinstance(value, int) and value > 0 else fallback

    @staticmethod
    def _file_sha256(path: Path) -> str:
        """Hash one model once at startup without loading it in one copy."""
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    def _record_inference_timing(
        self,
        latency_ms: float,
        now_monotonic: float,
    ) -> None:
        """Emit one lightweight two-second inference timing summary."""
        self.inference_timing_samples_ms.append(latency_ms)
        elapsed = now_monotonic - self.inference_timing_last_log
        if elapsed < 2.0:
            return
        samples = sorted(self.inference_timing_samples_ms)
        sample_count = len(samples)
        p50 = samples[max(0, (sample_count + 1) // 2 - 1)]
        p95 = samples[max(0, math.ceil(sample_count * 0.95) - 1)]
        window_duration = max(
            now_monotonic - self.inference_timing_window_started,
            1e-6,
        )
        self.get_logger().info(
            "[YOLO_INFERENCE_TIMING] "
            f"samples={sample_count} "
            f"avg_ms={sum(samples) / sample_count:.3f} "
            f"p50_ms={p50:.3f} "
            f"p95_ms={p95:.3f} "
            f"max_ms={samples[-1]:.3f} "
            f"effective_fps={sample_count / window_duration:.3f} "
            f"window_sec={window_duration:.3f}"
        )
        self.inference_timing_samples_ms.clear()
        self.inference_timing_window_started = now_monotonic
        self.inference_timing_last_log = now_monotonic

    def _create_session(
        self, requested_device: str
    ) -> tuple[ort.InferenceSession, str]:
        if ort is None:
            raise RuntimeError(
                "ONNX Runtime is required only for an .onnx model"
            )
        available = ort.get_available_providers()
        device = requested_device.strip().lower()
        provider_preferences = {
            "auto": [
                "TensorrtExecutionProvider",
                "CUDAExecutionProvider",
                "CPUExecutionProvider",
            ],
            "tensorrt": [
                "TensorrtExecutionProvider",
                "CUDAExecutionProvider",
                "CPUExecutionProvider",
            ],
            "cuda": ["CUDAExecutionProvider", "CPUExecutionProvider"],
            "cpu": ["CPUExecutionProvider"],
        }
        if device not in provider_preferences:
            raise ValueError("device must be auto, tensorrt, cuda, or cpu")

        providers = [
            provider
            for provider in provider_preferences[device]
            if provider in available
        ]
        if not providers:
            raise RuntimeError(
                f"No usable ONNX Runtime provider. Available: {available}"
            )

        try:
            session = ort.InferenceSession(
                str(self.model_path), providers=providers
            )
        except Exception as exc:
            if "CPUExecutionProvider" not in available or device == "cpu":
                raise RuntimeError(
                    f"Could not load YOLO26 model: {exc}"
                ) from exc
            self.get_logger().warning(
                "GPU provider initialization failed; falling back to CPU: "
                f"{exc}"
            )
            session = ort.InferenceSession(
                str(self.model_path), providers=["CPUExecutionProvider"]
            )

        active_provider = session.get_providers()[0]
        requested_gpu = device in {"cuda", "tensorrt"}
        if requested_gpu and active_provider == "CPUExecutionProvider":
            self.get_logger().warning(
                f"Requested {device}, but ONNX Runtime is using CPU"
            )
        return session, active_provider

    def _read_class_names(self) -> list[str]:
        if self.session is None:
            return DEFAULT_CLASS_NAMES.copy()
        metadata = self.session.get_modelmeta().custom_metadata_map
        raw_names = metadata.get("names")
        if raw_names:
            try:
                names = ast.literal_eval(raw_names)
                if isinstance(names, dict):
                    return [str(names[index]) for index in sorted(names)]
                if isinstance(names, (list, tuple)):
                    return [str(name) for name in names]
            except (SyntaxError, ValueError, KeyError, TypeError):
                self.get_logger().warning(
                    "Could not parse class names from ONNX metadata"
                )
        return DEFAULT_CLASS_NAMES.copy()

    def _letterbox(
        self, image: np.ndarray
    ) -> tuple[np.ndarray, LetterboxInfo]:
        height, width = image.shape[:2]
        scale = min(self.input_width / width, self.input_height / height)
        resized_width = max(1, int(round(width * scale)))
        resized_height = max(1, int(round(height * scale)))
        resized = cv2.resize(
            image,
            (resized_width, resized_height),
            interpolation=cv2.INTER_LINEAR,
        )

        pad_width = self.input_width - resized_width
        pad_height = self.input_height - resized_height
        left = pad_width // 2
        right = pad_width - left
        top = pad_height // 2
        bottom = pad_height - top
        padded = cv2.copyMakeBorder(
            resized,
            top,
            bottom,
            left,
            right,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )
        return padded, LetterboxInfo(scale, float(left), float(top))

    def _preprocess(
        self, image: np.ndarray
    ) -> tuple[np.ndarray, LetterboxInfo]:
        padded, info = self._letterbox(image)
        rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
        blob = rgb.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        return np.ascontiguousarray(blob), info

    def _postprocess(
        self,
        output: np.ndarray,
        info: LetterboxInfo,
        image_shape: tuple[int, ...],
    ) -> list[Detection]:
        predictions = (
            np.squeeze(output, axis=0) if output.ndim == 3 else output
        )
        if predictions.ndim != 2 or predictions.shape[1] < 6:
            raise RuntimeError(
                f"Unexpected YOLO26 output shape: {output.shape}"
            )

        height, width = image_shape[:2]
        detections: list[Detection] = []
        for prediction in predictions[: self.max_detections]:
            confidence = float(prediction[4])
            class_id = int(round(float(prediction[5])))
            if not 0 <= class_id < len(self.class_names):
                self.get_logger().warning(
                    f"Ignoring invalid class id: {class_id}"
                )
                continue
            class_name = self.class_names[class_id]
            class_thresholds = {
                "ball": self.ball_confidence_threshold,
                "hurdle": self.hurdle_confidence_threshold,
            }
            confidence_threshold = class_thresholds.get(
                class_name,
                self.confidence_threshold,
            )
            if confidence < confidence_threshold:
                continue

            x1, y1, x2, y2 = (float(value) for value in prediction[:4])
            x1 = (x1 - info.pad_x) / info.scale
            x2 = (x2 - info.pad_x) / info.scale
            y1 = (y1 - info.pad_y) / info.scale
            y2 = (y2 - info.pad_y) / info.scale

            left = int(np.clip(round(x1), 0, width - 1))
            top = int(np.clip(round(y1), 0, height - 1))
            right = int(np.clip(round(x2), 0, width - 1))
            bottom = int(np.clip(round(y2), 0, height - 1))
            if right <= left or bottom <= top:
                continue

            detections.append(
                Detection(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=confidence,
                    bbox=[left, top, right, bottom],
                    center=[(left + right) // 2, (top + bottom) // 2],
                )
            )
        return detections

    def _run_inference(self, blob: np.ndarray) -> np.ndarray:
        """Run the selected backend while keeping one output contract."""
        if self.tensorrt_backend is not None:
            return self.tensorrt_backend.infer(blob)
        if self.session is None:
            raise RuntimeError("No inference backend is initialized")
        outputs = self.session.run(None, {self.input_name: blob})
        if not outputs:
            raise RuntimeError("ONNX Runtime returned no outputs")
        return outputs[0]

    @staticmethod
    def _color_for_class(class_id: int) -> tuple[int, int, int]:
        colors = [
            (255, 255, 0),
            (0, 140, 255),
            (0, 255, 0),
            (255, 0, 255),
            (0, 255, 255),
        ]
        return colors[class_id % len(colors)]

    @staticmethod
    def _number(data: dict[str, Any], key: str) -> float | None:
        """Read one finite numeric display value from ball information."""
        value = data.get(key)
        if value is None or isinstance(value, bool):
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if np.isfinite(number) else None

    @staticmethod
    def _confirmation_badge(info: dict[str, Any] | None) -> str:
        """Return a short RAW/CANDIDATE/CONFIRMED display badge."""
        if info is None:
            return "RAW"
        if bool(info.get("confirmation_confirmed", False)):
            return "CONFIRMED"
        if not bool(info.get("raw_detected", False)):
            return "RAW"
        hits = int(info.get("confirmation_hits", 0))
        required = int(info.get("confirmation_required_hits", 1))
        return f"CANDIDATE {hits}/{required}"

    def _object_range_status(
        self,
        class_name: str,
        ball_info: dict[str, Any] | None,
        goal_info: dict[str, Any] | None,
        hurdle_info: dict[str, Any] | None,
    ) -> tuple[bool, bool, float | None]:
        """Return display visibility, control readiness, and target depth."""
        # A raw RGB ball detection must remain visible independently of depth.
        # ``ball_info`` only supplies confirmation and optional range metadata;
        # valid in-range depth is still required before motion control starts.
        if class_name == "ball":
            if ball_info is None:
                return True, False, None
            depth = self._number(ball_info, "depth_m")
            depth_valid = bool(ball_info.get("depth_valid", False))
            control_ready = bool(
                ball_info.get("detected", False)
                and depth_valid
                and depth is not None
                and depth <= self.ball_control_range_m
            )
            return (
                True,
                control_ready,
                depth if depth_valid else None,
            )

        if class_name == "hurdle":
            if hurdle_info is None:
                return True, False, None
            depth = self._number(hurdle_info, "depth_m")
            depth_valid = bool(hurdle_info.get("depth_valid", False))
            control_ready = bool(
                hurdle_info.get("detected", False)
                and depth_valid
                and depth is not None
                and depth <= self.hurdle_control_range_m
            )
            return (
                True,
                control_ready,
                depth if depth_valid else None,
            )

        settings = {
            "goal": (
                goal_info,
                self.goal_tracking_range_m,
                self.goal_control_range_m,
            ),
            "backboard": (
                goal_info,
                self.goal_tracking_range_m,
                self.goal_control_range_m,
            ),
        }
        setting = settings.get(class_name)
        if setting is None:
            return True, True, None
        info, tracking_range, control_range = setting
        if (
            info is None
            or not bool(info.get("detected", False))
            or not bool(info.get("depth_valid", False))
        ):
            return False, False, None
        depth = self._number(info, "depth_m")
        if depth is None or depth > tracking_range:
            return False, False, depth
        return True, depth <= control_range, depth

    @staticmethod
    def _metric_text(
        value: float | None,
        unit: str,
        digits: int = 2,
        signed: bool = False,
    ) -> str:
        """Format one optional numeric metric for the image panel."""
        if value is None:
            return "N/A"
        sign = "+" if signed else ""
        return f"{value:{sign}.{digits}f}{unit}"

    @staticmethod
    def _action_banner(
        source: str,
        action: str,
    ) -> tuple[str, tuple[int, int, int]] | None:
        """Map one selected planner command to a large screen banner."""
        normalized_source = source.strip().lower()
        normalized_action = action.strip().upper()
        line_labels = {
            "STRAIGHT": "STRAIGHT",
            "LEFT": "LEFT",
            "RIGHT": "RIGHT",
            "RECOVER_LEFT_TURN_LEFT": "RECOVER LEFT / TURN LEFT",
            "RECOVER_LEFT_TURN_RIGHT": "RECOVER LEFT / TURN RIGHT",
            "RECOVER_RIGHT_TURN_LEFT": "RECOVER RIGHT / TURN LEFT",
            "RECOVER_RIGHT_TURN_RIGHT": "RECOVER RIGHT / TURN RIGHT",
            "STOP": "STOP",
        }
        ball_labels = {
            "PICKUP_NOW": "PICK UP BALL",
            "TURN_LEFT": "BALL TURN LEFT",
            "TURN_RIGHT": "BALL TURN RIGHT",
            "APPROACH": "BALL APPROACH",
            "SLOW_APPROACH": "BALL SLOW APPROACH",
            "FINE_FORWARD_STEP": "BALL FINE STEP",
            "BALL_LOST_STOP": "BALL STOP",
            "RECOVER_TURN_LEFT": "FIND BALL LEFT",
            "RECOVER_TURN_RIGHT": "FIND BALL RIGHT",
            "STOP": "BALL STOP",
        }
        goal_labels = {
            "TURN_LEFT": "GOAL TURN LEFT",
            "TURN_RIGHT": "GOAL TURN RIGHT",
            "RETREAT_GOAL": "GOAL RETREAT",
            "SHOT": "SHOOT",
            "WAIT": "GOAL WAIT",
            "WAIT_SCORE_CONFIRMATION": "GOAL HOLD",
        }
        hurdle_labels = {
            "TURN_LEFT": "HURDLE TURN LEFT",
            "TURN_RIGHT": "HURDLE TURN RIGHT",
            "ALIGN_LEFT": "HURDLE PARALLEL LEFT",
            "ALIGN_RIGHT": "HURDLE PARALLEL RIGHT",
            "GO": "HURDLE GO",
            "WAIT": "HURDLE WAIT",
            "WAIT_GO_CONFIRMATION": "HURDLE HOLD",
        }

        def straight_label(prefix: str) -> str | None:
            if normalized_action == "STRAIGHT":
                return f"{prefix}STRAIGHT"
            if not normalized_action.startswith("STRAIGHT_"):
                return None
            level_text = normalized_action.rsplit("_", 1)[1]
            if not level_text.isdigit() or not 0 <= int(level_text) <= 5:
                return None
            return f"{prefix}STRAIGHT {int(level_text)}"

        if normalized_source == "line":
            label = straight_label("") or line_labels.get(normalized_action)
            if label is None:
                base_action, separator, suffix_text = (
                    normalized_action.rpartition("_")
                )
                base_label = line_labels.get(base_action)
                suffix_angles = (
                    {2: 15, 4: 30, 6: 45, 8: 60, 10: 75, 13: 90}
                    if base_action.endswith("TURN_LEFT")
                    else {4: 15, 6: 30, 8: 45, 10: 60, 12: 75, 15: 90}
                )
                if separator and base_label is not None and suffix_text.isdigit():
                    angle = suffix_angles.get(int(suffix_text))
                    if angle is not None:
                        label = f"{base_label} ({angle} DEG)"
            return (label, (130, 105, 0)) if label is not None else None
        if normalized_source == "ball":
            label = straight_label("BALL / ") or ball_labels.get(
                normalized_action
            )
            turn_motion, _, turn_angle_deg = numbered_turn_motion_metadata(
                normalized_action
            )
            if label is None and turn_motion is not None:
                turn_direction = (
                    "LEFT" if turn_angle_deg is not None and turn_angle_deg < 0
                    else "RIGHT"
                )
                label = (
                    f"BALL TURN {turn_direction} "
                    f"({abs(int(turn_angle_deg or 0))} DEG)"
                )
            if label is None:
                return None
            color = (
                (0, 120, 0)
                if normalized_action == "PICKUP_NOW"
                else (0, 105, 190)
            )
            return label, color
        if normalized_source == "goal":
            label = straight_label("GOAL / ") or goal_labels.get(
                normalized_action
            )
            return (label, (120, 90, 155)) if label is not None else None
        if normalized_source == "hurdle":
            label = straight_label("HURDLE / ") or hurdle_labels.get(
                normalized_action
            )
            return (label, (0, 125, 190)) if label is not None else None
        return None

    @staticmethod
    def _draw_action_banner(
        image: np.ndarray,
        text: str,
        background_color: tuple[int, int, int],
    ) -> None:
        """Draw one centered command banner while fitting long labels."""
        height, width = image.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        thickness = 4
        font_scale = 1.35
        max_text_width = max(120, width - 48)
        while font_scale > 0.65:
            (text_width, text_height), _ = cv2.getTextSize(
                text,
                font,
                font_scale,
                thickness,
            )
            if text_width <= max_text_width:
                break
            font_scale -= 0.05
        text_x = max(12, (width - text_width) // 2)
        text_y = min(76, max(text_height + 18, height - 12))
        cv2.rectangle(
            image,
            (text_x - 18, text_y - text_height - 14),
            (text_x + text_width + 18, text_y + 14),
            background_color,
            -1,
        )
        cv2.putText(
            image,
            text,
            (text_x, text_y),
            font,
            font_scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )

    def _draw_ball_metrics(self, image: np.ndarray) -> None:
        """Draw analyzed ball distance and alignment data in-place."""
        if not self.show_ball_metrics:
            return

        height, width = image.shape[:2]
        info = self._fresh_ball_info()
        recent_info = self._recent_ball_info()
        calibrated_center_x = (
            self._number(info, "robot_center_x_px")
            if info is not None
            else None
        )
        center_x = int(
            np.clip(
                round(calibrated_center_x)
                if calibrated_center_x is not None
                else width // 2,
                0,
                width - 1,
            )
        )
        center_color = (0, 255, 255)
        camera_center_x = width // 2
        camera_center_y = height // 2
        cv2.drawMarker(
            image,
            (camera_center_x, camera_center_y),
            (180, 180, 180),
            cv2.MARKER_CROSS,
            18,
            1,
            cv2.LINE_AA,
        )
        cv2.line(
            image,
            (center_x, max(45, int(height * 0.42))),
            (center_x, height - 1),
            center_color,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            "ROBOT CENTER",
            (min(center_x + 10, width - 180), max(45, int(height * 0.42))),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            center_color,
            1,
            cv2.LINE_AA,
        )

        decision = self._fresh_motion_command()
        if decision is None:
            planner_action = "NO COMMAND"
        else:
            planner_source = str(decision.get("source", "none")).upper()
            selected_action = str(decision.get("action", "WAIT")).upper()
            planner_action = f"{planner_source} / {selected_action}"
        detected = bool(info and info.get("detected", False))
        if detected and info is not None:
            target_x = self._number(info, "center_x")
            target_y = self._number(info, "center_y")
            if target_x is not None and target_y is not None:
                point_x = int(np.clip(round(target_x), 0, width - 1))
                point_y = int(np.clip(round(target_y), 0, height - 1))
                cv2.line(
                    image,
                    (camera_center_x, point_y),
                    (point_x, point_y),
                    (0, 140, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.line(
                    image,
                    (point_x, height - 1),
                    (point_x, point_y),
                    (255, 160, 0),
                    2,
                    cv2.LINE_AA,
                )
                offset_x = self._number(
                    info,
                    "camera_center_offset_x_px",
                )
                offset_text = self._metric_text(
                    offset_x,
                    "px",
                    digits=0,
                    signed=True,
                )
                bottom_distance = self._number(
                    info,
                    "bottom_distance_px",
                )
                bottom_text = self._metric_text(
                    bottom_distance,
                    "px",
                    digits=0,
                )
                text_x = min(camera_center_x, point_x) + 8
                text_y = max(22, point_y - 10)
                cv2.putText(
                    image,
                    f"center dx {offset_text}",
                    (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 140, 255),
                    2,
                    cv2.LINE_AA,
                )
                vertical_text_y = max(22, (height - 1 + point_y) // 2)
                cv2.putText(
                    image,
                    f"bottom dy {bottom_text}",
                    (min(point_x + 8, max(12, width - 190)), vertical_text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 160, 0),
                    2,
                    cv2.LINE_AA,
                )

        panel_width = min(390, max(250, width - 24))
        panel_height = 505
        panel_x = max(12, width - panel_width - 12)
        panel_y = 44
        panel_bottom = min(height - 8, panel_y + panel_height)
        overlay = image.copy()
        cv2.rectangle(
            overlay,
            (panel_x, panel_y),
            (panel_x + panel_width, panel_bottom),
            (20, 20, 20),
            -1,
        )
        image[panel_y:panel_bottom, panel_x:panel_x + panel_width] = (
            cv2.addWeighted(
                overlay[panel_y:panel_bottom, panel_x:panel_x + panel_width],
                0.72,
                image[panel_y:panel_bottom, panel_x:panel_x + panel_width],
                0.28,
                0.0,
            )
        )

        if info is None:
            stamp_delta = getattr(self, "_ball_info_stamp_delta_ms", None)
            analyzer_badge = self._confirmation_badge(recent_info)
            rows = [
                "BALL METRICS",
                f"Planner     : {planner_action}",
                (
                    f"Frame sync  : STALE ({stamp_delta:.1f} ms)"
                    if stamp_delta is not None
                    else "NO MATCHED BALL INFO"
                ),
                f"Analyzer    : {analyzer_badge}",
            ]
            if recent_info is not None:
                rows.append(
                    f"Analyzer state: {recent_info.get('state', 'UNKNOWN')}"
                )
        elif not detected:
            state = str(info.get("state", "SEARCH"))
            rows = [
                "BALL METRICS",
                f"Planner     : {planner_action}",
                f"State       : {state}",
            ]
        else:
            depth = self._number(info, "depth_m")
            offset_px = self._number(info, "offset_x_px")
            offset_norm = self._number(info, "offset_x_norm")
            camera_center_offset_px = self._number(
                info,
                "camera_center_offset_x_px",
            )
            bottom_distance_px = self._number(info, "bottom_distance_px")
            head_down_requested = bool(
                info.get("head_down_requested", False)
            )
            bearing = self._number(info, "bearing_deg")
            steering_angle = self._number(info, "steering_angle_deg")
            lateral = self._number(info, "lateral_offset_m")
            direction = str(info.get("horizontal_direction", "UNKNOWN"))
            state = str(info.get("state", "UNKNOWN"))
            camera_ready = bool(info.get("camera_info_ready", False))
            depth_source = str(info.get("depth_source", "invalid")).upper()
            sync_delta = self._number(info, "depth_sync_delta_ms")
            depth_width = self._number(info, "depth_width")
            depth_height = self._number(info, "depth_height")
            rgb_width = self._number(info, "image_width")
            rgb_height = self._number(info, "image_height")
            sample_count = self._number(info, "depth_sample_count")
            rows = [
                "BALL METRICS",
                f"Planner     : {planner_action}",
                f"State       : {state} / {direction}",
                "Depth Z     : " + self._metric_text(depth, "m"),
                f"Depth source: {depth_source}",
                "RGB-Depth dT: " + self._metric_text(sync_delta, "ms", 1),
                "Depth res   : "
                + (
                    f"{int(depth_width)}x{int(depth_height)}"
                    if depth_width is not None and depth_height is not None
                    else "N/A"
                ),
                "RGB res     : "
                + (
                    f"{int(rgb_width)}x{int(rgb_height)}"
                    if rgb_width is not None and rgb_height is not None
                    else "N/A"
                ),
                "Valid pixels: "
                + self._metric_text(sample_count, "", 0),
                "Robot dx    : "
                + self._metric_text(offset_px, "px", 0, signed=True),
                "Camera dx   : "
                + self._metric_text(
                    camera_center_offset_px,
                    "px",
                    0,
                    signed=True,
                ),
                "Bottom dy   : "
                + self._metric_text(bottom_distance_px, "px", 0),
                f"Head down   : {'ON' if head_down_requested else 'OFF'}",
                "Offset norm : "
                + self._metric_text(offset_norm, "", 3, signed=True),
                "Bearing     : "
                + self._metric_text(bearing, "deg", 1, signed=True),
                "Path angle  : "
                + self._metric_text(
                    steering_angle,
                    "deg",
                    1,
                    signed=True,
                ),
                "Lateral X   : "
                + self._metric_text(lateral, "m", 3, signed=True),
                f"Camera info : {'OK' if camera_ready else 'MISSING'}",
            ]

        for index, row in enumerate(rows):
            is_title = index == 0
            color = (0, 200, 255) if is_title else (245, 245, 245)
            cv2.putText(
                image,
                row,
                (panel_x + 12, panel_y + 25 + index * 23),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.56 if is_title else 0.5,
                color,
                2 if is_title else 1,
                cv2.LINE_AA,
            )

        decision_source = (
            str(decision.get("source", "")) if decision is not None else ""
        )
        decision_action = (
            str(decision.get("action", "")) if decision is not None else ""
        )
        banner = self._action_banner(decision_source, decision_action)
        if banner is None and detected and info is not None:
            if bool(info.get("pickup_now")):
                banner = ("PICK UP BALL", (0, 120, 0))
        if banner is not None:
            self._draw_action_banner(image, banner[0], banner[1])

    def _active_metrics_mode(self) -> str:
        """Choose the panel that matches the planner actually in use."""
        if self.metrics_mode != "auto":
            return self.metrics_mode

        decision = self._fresh_motion_command()
        if decision is not None:
            selected = str(decision.get("source", "")).strip().lower()
            enabled = {
                "line": self.show_line_metrics,
                "ball": self.show_ball_metrics,
                "goal": self.show_goal_metrics,
                "hurdle": self.show_hurdle_metrics,
            }
            if selected in enabled and enabled[selected]:
                return selected

        line_info = self._fresh_line_info()
        if (
            self.show_line_metrics
            and line_info
            and bool(line_info.get("detected", False))
        ):
            return "line"

        choices = (
            ("hurdle", self.show_hurdle_metrics, self._fresh_hurdle_info()),
            ("goal", self.show_goal_metrics, self._fresh_goal_info()),
            ("ball", self.show_ball_metrics, self._fresh_ball_info()),
        )
        for mode, enabled, info in choices:
            if enabled and info and bool(info.get("detected", False)):
                return mode
        return "line" if self.show_line_metrics else "ball"

    def _draw_line_metrics(self, image: np.ndarray) -> None:
        """Draw line geometry and the command chosen by the unified planner."""
        if not self.show_line_metrics:
            return

        height, width = image.shape[:2]
        decision = self._fresh_motion_command()
        info = self._fresh_line_info()
        self._draw_line_path_geometry(image, info)
        panel_width = min(410, max(270, width - 24))
        panel_height = 334
        panel_x = max(12, width - panel_width - 12)
        panel_y = 44
        panel_bottom = min(height - 8, panel_y + panel_height)
        overlay = image.copy()
        cv2.rectangle(
            overlay,
            (panel_x, panel_y),
            (panel_x + panel_width, panel_bottom),
            (20, 20, 20),
            -1,
        )
        panel_slice = np.s_[
            panel_y:panel_bottom,
            panel_x:panel_x + panel_width,
        ]
        image[panel_slice] = cv2.addWeighted(
            overlay[panel_slice],
            0.72,
            image[panel_slice],
            0.28,
            0.0,
        )

        planner_source = (
            str(decision.get("source", "none")).upper()
            if decision is not None
            else "WAITING"
        )
        planner_action = (
            str(decision.get("action", "WAIT")).upper()
            if decision is not None
            else "WAIT"
        )
        detected = bool(info and info.get("detected", False))
        if not detected or info is None:
            rows = [
                "LINE METRICS",
                f"Planner     : {planner_source} / {planner_action}",
                "State       : SEARCH",
            ]
        else:
            heading = self._number(info, "filtered_heading_error_deg")
            if heading is None:
                heading = self._number(info, "heading_error_deg")
            offset = self._number(info, "filtered_lateral_offset_norm")
            if offset is None:
                offset = self._number(info, "lateral_offset_norm")
            turn = self._number(info, "turn_angle_deg")
            if turn is None:
                turn = self._number(info, "path_turn_delta_deg")
            qualities = [
                self._number(info, "heading_quality"),
                self._number(info, "geometry_quality"),
                self._number(info, "detection_quality"),
            ]
            valid_qualities = [
                value for value in qualities if value is not None
            ]
            quality = min(valid_qualities) if valid_qualities else None
            rows = [
                "LINE METRICS",
                f"Planner     : {planner_source} / {planner_action}",
                "State       : TRACKING",
                "Heading     : "
                + self._metric_text(heading, "deg", 1, signed=True),
                "Offset norm : "
                + self._metric_text(offset, "", 3, signed=True),
                "Turn preview: "
                + self._metric_text(turn, "deg", 1, signed=True),
                "Quality     : " + self._metric_text(quality, "", 3),
            ]
            if bool(info.get("corner_preview_confirmed", False)):
                corner_direction = str(
                    info.get("corner_direction", "UNKNOWN")
                ).upper()
                corner_distance = self._number(
                    info,
                    "corner_start_distance_m",
                )
                corner_motion = str(
                    info.get("corner_approach_motion", "STRAIGHT")
                ).upper()
                rows.extend(
                    [
                        f"Corner      : {corner_direction}",
                        "Corner dist : "
                        + self._metric_text(corner_distance, "m", 2),
                        f"Approach    : {corner_motion}",
                    ]
                )

        for index, row in enumerate(rows):
            title = index == 0
            cv2.putText(
                image,
                row,
                (panel_x + 12, panel_y + 25 + index * 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.56 if title else 0.5,
                (255, 255, 0) if title else (245, 245, 245),
                2 if title else 1,
                cv2.LINE_AA,
            )

        banner = self._action_banner(planner_source, planner_action)
        source_command = (
            decision.get("source_command", {})
            if decision is not None
            else {}
        )
        if (
            planner_source == "LINE"
            and isinstance(source_command, dict)
            and bool(source_command.get("corner_prepare", False))
        ):
            corner_direction = str(
                source_command.get("corner_direction", "TURN")
            ).upper()
            approach_motion = str(
                source_command.get("approach_motion", "STRAIGHT")
            ).upper()
            banner = (
                f"{corner_direction} AHEAD / {approach_motion}",
                (0, 165, 255),
            )
        if banner is not None:
            self._draw_action_banner(image, banner[0], banner[1])

    def _draw_line_path_geometry(
        self,
        image: np.ndarray,
        info: dict[str, Any] | None,
    ) -> None:
        """Draw the near-to-far line path used by the line planner."""
        if info is None or not bool(info.get("detected", False)):
            return

        height, width = image.shape[:2]
        calibrated_center_x = self._number(info, "robot_center_x_px")
        center_x = int(
            np.clip(
                round(calibrated_center_x)
                if calibrated_center_x is not None
                else width // 2,
                0,
                width - 1,
            )
        )
        cv2.line(
            image,
            (center_x, 0),
            (center_x, height),
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            "ROBOT CENTER",
            (center_x + 10, 52),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )

        points = self._parse_line_center_points(
            info.get("center_points_px", [])
        )
        if len(points) >= 2:
            polyline = np.asarray(points, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(
                image,
                [polyline],
                False,
                (0, 0, 0),
                8,
                cv2.LINE_AA,
            )
            cv2.polylines(
                image,
                [polyline],
                False,
                (0, 255, 0),
                4,
                cv2.LINE_AA,
            )

        for index, point in enumerate(points):
            radius = 10 if index == 0 else 7
            color = (0, 0, 255) if index == 0 else (0, 255, 255)
            cv2.circle(image, point, radius + 2, (0, 0, 0), -1)
            cv2.circle(image, point, radius, color, -1, cv2.LINE_AA)
            cv2.putText(
                image,
                str(index),
                (point[0] + 11, point[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        raw_corner_point = info.get("corner_start_point_px")
        if (
            bool(info.get("corner_preview_confirmed", False))
            and isinstance(raw_corner_point, list)
            and len(raw_corner_point) == 2
        ):
            try:
                corner_point = (
                    int(round(float(raw_corner_point[0]))),
                    int(round(float(raw_corner_point[1]))),
                )
            except (TypeError, ValueError):
                corner_point = None
            if corner_point is not None:
                cv2.circle(
                    image,
                    corner_point,
                    15,
                    (0, 165, 255),
                    4,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    image,
                    "TURN START",
                    (corner_point[0] + 18, corner_point[1] - 14),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.62,
                    (0, 165, 255),
                    2,
                    cv2.LINE_AA,
                )

        if points:
            near = points[0]
            far = points[-1]
            cv2.putText(
                image,
                "NEAR",
                (near[0] + 14, min(height - 10, near[1] + 26)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                image,
                "FAR",
                (far[0] + 14, max(20, far[1] - 14)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )
            self._draw_line_heading_arrow(image, info, near)

        offset_px = self._number(info, "lateral_offset_px")
        if offset_px is not None:
            eval_y = int(height * 0.82)
            line_x = int(np.clip(center_x + offset_px, 0, width - 1))
            cv2.line(
                image,
                (center_x, eval_y),
                (line_x, eval_y),
                (0, 165, 255),
                5,
                cv2.LINE_AA,
            )
            cv2.circle(
                image,
                (line_x, eval_y),
                9,
                (0, 165, 255),
                -1,
                cv2.LINE_AA,
            )
            cv2.putText(
                image,
                f"OFFSET {offset_px:+.1f}px",
                (min(center_x, line_x), max(25, eval_y - 14)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (0, 165, 255),
                2,
                cv2.LINE_AA,
            )

    @staticmethod
    def _parse_line_center_points(raw_points: Any) -> list[tuple[int, int]]:
        """Parse the analyzer's already ordered near-to-far points."""
        if not isinstance(raw_points, list):
            return []
        points: list[tuple[int, int]] = []
        for raw_point in raw_points:
            if not isinstance(raw_point, list) or len(raw_point) != 2:
                continue
            try:
                points.append(
                    (
                        int(round(float(raw_point[0]))),
                        int(round(float(raw_point[1]))),
                    )
                )
            except (TypeError, ValueError):
                continue
        return points

    def _draw_line_heading_arrow(
        self,
        image: np.ndarray,
        info: dict[str, Any],
        near: tuple[int, int],
    ) -> None:
        """Draw the filtered local heading from the nearest path point."""
        heading = self._number(info, "filtered_heading_error_deg")
        if heading is None:
            heading = self._number(info, "heading_error_deg")
        if heading is None:
            return
        arrow_length = max(90, min(180, int(image.shape[0] * 0.25)))
        angle_rad = np.deg2rad(heading)
        end = (
            int(round(near[0] + arrow_length * np.sin(angle_rad))),
            int(round(near[1] - arrow_length * np.cos(angle_rad))),
        )
        cv2.arrowedLine(
            image,
            near,
            end,
            (255, 0, 255),
            5,
            cv2.LINE_AA,
            tipLength=0.15,
        )
        cv2.putText(
            image,
            f"HEADING {heading:+.1f}deg",
            (end[0] + 8, max(22, end[1])),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 0, 255),
            2,
            cv2.LINE_AA,
        )

    def _draw_goal_metrics(self, image: np.ndarray) -> None:
        """Draw analyzed goal distance and scoring data in-place."""
        height, width = image.shape[:2]
        center_x = width // 2
        cv2.line(
            image,
            (center_x, max(45, int(height * 0.32))),
            (center_x, height - 1),
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )

        info = self._fresh_goal_info()
        decision = self._fresh_motion_command()
        if decision is None:
            planner_action = "NO COMMAND"
        else:
            planner_source = str(decision.get("source", "none")).upper()
            selected_action = str(decision.get("action", "WAIT")).upper()
            planner_action = f"{planner_source} / {selected_action}"
        detected = bool(info and info.get("detected", False))
        if detected and info is not None:
            target_x = self._number(info, "center_x")
            target_y = self._number(info, "center_y")
            if target_x is not None and target_y is not None:
                point_x = int(np.clip(round(target_x), 0, width - 1))
                point_y = int(np.clip(round(target_y), 0, height - 1))
                cv2.arrowedLine(
                    image,
                    (center_x, point_y),
                    (point_x, point_y),
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                    tipLength=0.12,
                )
                offset_x = self._number(info, "offset_x_px")
                offset_text = self._metric_text(
                    offset_x,
                    "px",
                    digits=0,
                    signed=True,
                )
                text_x = min(center_x, point_x) + 8
                cv2.putText(
                    image,
                    f"dx {offset_text}",
                    (text_x, max(22, point_y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

        panel_width = min(390, max(250, width - 24))
        panel_height = 334
        panel_x = max(12, width - panel_width - 12)
        panel_y = 44
        panel_bottom = min(height - 8, panel_y + panel_height)
        overlay = image.copy()
        cv2.rectangle(
            overlay,
            (panel_x, panel_y),
            (panel_x + panel_width, panel_bottom),
            (20, 20, 20),
            -1,
        )
        image[panel_y:panel_bottom, panel_x:panel_x + panel_width] = (
            cv2.addWeighted(
                overlay[panel_y:panel_bottom, panel_x:panel_x + panel_width],
                0.72,
                image[panel_y:panel_bottom, panel_x:panel_x + panel_width],
                0.28,
                0.0,
            )
        )

        if info is None:
            rows = [
                "GOAL METRICS",
                f"Planner     : {planner_action}",
                "NO GOAL INFO",
            ]
        elif not detected:
            state = str(info.get("state", "SEARCH"))
            rows = [
                "GOAL METRICS",
                f"Planner     : {planner_action}",
                f"State       : {state}",
            ]
        else:
            depth = self._number(info, "depth_m")
            offset_px = self._number(info, "offset_x_px")
            offset_norm = self._number(info, "offset_x_norm")
            bearing = self._number(info, "bearing_deg")
            lateral = self._number(info, "lateral_offset_m")
            sample_count = self._number(info, "depth_sample_count")
            aim_source = str(info.get("aim_source", "goal")).upper()
            direction = str(info.get("horizontal_direction", "UNKNOWN"))
            state = str(info.get("state", "UNKNOWN"))
            rows = [
                "GOAL METRICS",
                f"Planner     : {planner_action}",
                f"State       : {state} / {direction}",
                f"Aim source  : {aim_source}",
                "Depth Z     : " + self._metric_text(depth, "m"),
                "Offset X    : "
                + self._metric_text(offset_px, "px", 0, signed=True),
                "Offset norm : "
                + self._metric_text(offset_norm, "", 3, signed=True),
                "Bearing     : "
                + self._metric_text(bearing, "deg", 1, signed=True),
                "Lateral X   : "
                + self._metric_text(lateral, "m", 3, signed=True),
                "Depth points: "
                + self._metric_text(sample_count, "", 0),
                f"Shot now    : {'YES' if info.get('score_now') else 'NO'}",
            ]

        for index, row in enumerate(rows):
            is_title = index == 0
            color = (0, 255, 0) if is_title else (245, 245, 245)
            cv2.putText(
                image,
                row,
                (panel_x + 12, panel_y + 25 + index * 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.56 if is_title else 0.5,
                color,
                2 if is_title else 1,
                cv2.LINE_AA,
            )

        if decision is not None:
            banner = self._action_banner(
                str(decision.get("source", "")),
                str(decision.get("action", "")),
            )
            if banner is not None and str(
                decision.get("action", "")
            ).upper() != "SHOT":
                self._draw_action_banner(image, banner[0], banner[1])

        if detected and info is not None and bool(info.get("score_now")):
            score_text = "SHOT"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1.0
            thickness = 3
            (text_width, text_height), _ = cv2.getTextSize(
                score_text,
                font,
                font_scale,
                thickness,
            )
            text_x = max(12, (width - text_width) // 2)
            text_y = 72
            cv2.rectangle(
                image,
                (text_x - 14, text_y - text_height - 14),
                (text_x + text_width + 14, text_y + 14),
                (0, 120, 0),
                -1,
            )
            cv2.putText(
                image,
                score_text,
                (text_x, text_y),
                font,
                font_scale,
                (255, 255, 255),
                thickness,
                cv2.LINE_AA,
            )

    def _draw_hurdle_metrics(self, image: np.ndarray) -> None:
        """Draw hurdle alignment, depth, width, and jump readiness."""
        if not self.show_hurdle_metrics:
            return

        height, width = image.shape[:2]
        info = self._fresh_hurdle_info()
        decision = self._fresh_motion_command()
        detected = bool(info and info.get("detected", False))

        self._draw_line_path_geometry(image, self._fresh_line_info())
        source_command: dict[str, Any] = {}
        if decision is not None:
            raw_source_command = decision.get("source_command", {})
            if isinstance(raw_source_command, dict):
                source_command = raw_source_command
        self._draw_hurdle_path_reference(image, source_command)

        panel_width = min(410, max(260, width - 24))
        panel_height = 358
        panel_x = max(12, width - panel_width - 12)
        panel_y = 44
        panel_bottom = min(height - 8, panel_y + panel_height)
        overlay = image.copy()
        cv2.rectangle(
            overlay,
            (panel_x, panel_y),
            (panel_x + panel_width, panel_bottom),
            (20, 20, 20),
            -1,
        )
        panel_slice = np.s_[
            panel_y:panel_bottom,
            panel_x:panel_x + panel_width,
        ]
        image[panel_slice] = cv2.addWeighted(
            overlay[panel_slice],
            0.72,
            image[panel_slice],
            0.28,
            0.0,
        )

        if info is None:
            rows = ["HURDLE METRICS", "NO HURDLE INFO"]
        elif not detected:
            state = str(info.get("state", "SEARCH"))
            rows = ["HURDLE METRICS", f"State       : {state}"]
        else:
            state = str(info.get("state", "UNKNOWN"))
            path_source = str(
                source_command.get("path_reference_source", "none")
            ).upper()
            rows = [
                "HURDLE METRICS",
                f"State       : {state}",
                f"Path ref    : {path_source}",
                "Path offset : "
                + self._metric_text(
                    self._number(source_command, "path_offset_x_norm"),
                    "",
                    3,
                    signed=True,
                ),
                "Depth Z     : "
                + self._metric_text(self._number(info, "depth_m"), "m"),
                "Left depth  : "
                + self._metric_text(
                    self._number(info, "left_depth_m"),
                    "m",
                ),
                "Right depth : "
                + self._metric_text(
                    self._number(info, "right_depth_m"),
                    "m",
                ),
                "Width est.  : "
                + self._metric_text(
                    self._number(info, "estimated_width_m"),
                    "m",
                ),
                "Parallel err: "
                + self._metric_text(
                    self._number(info, "hurdle_angle_deg"),
                    "deg",
                    1,
                    signed=True,
                ),
                f"Parallel    : {'YES' if info.get('is_parallel') else 'NO'}",
                "Depth points: "
                + str(int(info.get("depth_sample_count", 0))),
                f"Go now      : {'YES' if info.get('go_now') else 'NO'}",
            ]

        for index, row in enumerate(rows):
            title = index == 0
            cv2.putText(
                image,
                row,
                (panel_x + 12, panel_y + 25 + index * 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.56 if title else 0.5,
                (0, 255, 255) if title else (245, 245, 245),
                2 if title else 1,
                cv2.LINE_AA,
            )

        if decision is not None:
            banner = self._action_banner(
                str(decision.get("source", "")),
                str(decision.get("action", "")),
            )
            if banner is not None and str(
                decision.get("action", "")
            ).upper() != "GO":
                self._draw_action_banner(image, banner[0], banner[1])

        if detected and info is not None and bool(info.get("go_now")):
            go_text = "GO!"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1.4
            thickness = 4
            (text_width, text_height), _ = cv2.getTextSize(
                go_text,
                font,
                font_scale,
                thickness,
            )
            text_x = max(12, (width - text_width) // 2)
            text_y = 76
            cv2.rectangle(
                image,
                (text_x - 18, text_y - text_height - 14),
                (text_x + text_width + 18, text_y + 14),
                (0, 140, 0),
                -1,
            )
            cv2.putText(
                image,
                go_text,
                (text_x, text_y),
                font,
                font_scale,
                (255, 255, 255),
                thickness,
                cv2.LINE_AA,
            )

    @staticmethod
    def _draw_hurdle_path_reference(
        image: np.ndarray,
        source_command: dict[str, Any],
    ) -> None:
        """Draw the inferred line bridge and hurdle approach reference."""
        if not bool(source_command.get("path_reference_valid", False)):
            return
        raw_support = source_command.get("path_support_points_px", [])
        raw_segment = source_command.get("path_bridge_segment_px")
        raw_point = source_command.get("path_reference_point_px")
        if not isinstance(raw_segment, list) or len(raw_segment) != 2:
            return
        if not isinstance(raw_point, list) or len(raw_point) != 2:
            return
        try:
            segment = [
                (int(round(float(point[0]))), int(round(float(point[1]))))
                for point in raw_segment
                if isinstance(point, list) and len(point) == 2
            ]
            reference = (
                int(round(float(raw_point[0]))),
                int(round(float(raw_point[1]))),
            )
        except (TypeError, ValueError):
            return
        if len(segment) != 2:
            return
        support: list[tuple[int, int]] = []
        if isinstance(raw_support, list):
            try:
                support = [
                    (
                        int(round(float(point[0]))),
                        int(round(float(point[1]))),
                    )
                    for point in raw_support
                    if isinstance(point, list) and len(point) == 2
                ]
            except (TypeError, ValueError):
                support = []
        if len(support) >= 2:
            polyline = np.asarray(support, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(
                image,
                [polyline],
                False,
                (255, 255, 0),
                3,
                cv2.LINE_AA,
            )
        cv2.line(
            image,
            segment[0],
            segment[1],
            (255, 255, 0),
            5,
            cv2.LINE_AA,
        )
        cv2.drawMarker(
            image,
            reference,
            (0, 165, 255),
            cv2.MARKER_CROSS,
            30,
            5,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            "HURDLE PATH TARGET",
            (reference[0] + 16, max(24, reference[1] - 14)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 165, 255),
            2,
            cv2.LINE_AA,
        )

    def _draw_detections(
        self, image: np.ndarray, detections: list[Detection]
    ) -> np.ndarray:
        annotated = image.copy()
        metrics_mode = self._active_metrics_mode()
        decision = self._fresh_motion_command()
        ball_info = self._fresh_ball_info()
        recent_ball_info = self._recent_ball_info()
        goal_info = self._fresh_goal_info()
        hurdle_info = self._fresh_hurdle_info()
        line_info = self._fresh_line_info()
        ball_detected = bool(ball_info and ball_info.get("detected", False))
        goal_detected = bool(goal_info and goal_info.get("detected", False))
        decision_source = (
            str(decision.get("source", "")).lower()
            if decision is not None
            else ""
        )
        decision_action = (
            str(decision.get("action", "")).upper()
            if decision is not None
            else ""
        )
        ball_recovery = (
            metrics_mode == "ball"
            and decision_source == "ball"
            and not ball_detected
            and (
                decision_action == "BALL_LOST_STOP"
                or decision_action.startswith("RECOVER_TURN_")
            )
        )
        goal_recovery = (
            metrics_mode == "goal"
            and decision_source == "goal"
            and not goal_detected
            and (
                decision_action == "GOAL_LOST_STOP"
                or decision_action.startswith("RECOVER_GOAL_TURN_")
            )
        )
        show_recovery_line = (
            (ball_recovery or goal_recovery)
            and line_info is not None
            and bool(line_info.get("detected", False))
        )
        for detection in detections:
            if (
                metrics_mode == "line" or show_recovery_line
            ) and detection.class_name == "line":
                continue
            visible, control_ready, object_depth = self._object_range_status(
                detection.class_name,
                ball_info,
                goal_info,
                hurdle_info,
            )
            if not visible:
                continue
            left, top, right, bottom = detection.bbox
            color = self._color_for_class(detection.class_id)
            label = f"{detection.class_name} {detection.confidence:.2f}"
            if object_depth is not None and not control_ready:
                label = (
                    f"{detection.class_name} TRACK ONLY "
                    f"{object_depth:.2f}m"
                )
            confirmation_info = {
                "line": line_info,
                "ball": ball_info,
                "goal": goal_info,
                "backboard": goal_info,
                "hurdle": hurdle_info,
            }.get(detection.class_name)
            depth = (
                self._number(confirmation_info, "depth_m")
                if confirmation_info is not None
                else None
            )
            if depth is not None:
                label = f"{label} | DEPTH {depth:.2f}m"
            if detection.class_name == "ball" and confirmation_info is None:
                badge = (
                    "INFO STALE"
                    if recent_ball_info is not None
                    else "RAW YOLO"
                )
            else:
                badge = self._confirmation_badge(confirmation_info)
            label = f"{label} | {badge}"
            cv2.rectangle(annotated, (left, top), (right, bottom), color, 2)
            cv2.circle(annotated, tuple(detection.center), 4, color, -1)
            cv2.putText(
                annotated,
                label,
                (left, max(20, top - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )
        if metrics_mode == "line":
            self._draw_line_metrics(annotated)
        elif metrics_mode == "hurdle":
            self._draw_hurdle_metrics(annotated)
        elif metrics_mode == "goal":
            if show_recovery_line and line_info is not None:
                self._draw_line_path_geometry(annotated, line_info)
            self._draw_goal_metrics(annotated)
        else:
            if show_recovery_line and line_info is not None:
                self._draw_line_path_geometry(annotated, line_info)
            self._draw_ball_metrics(annotated)
        cv2.putText(
            annotated,
            f"YOLO26 | {self.active_provider} | {self.smoothed_fps:.1f} FPS",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        self._draw_grasp_verification_status(annotated)
        return annotated

    @staticmethod
    def _grasp_verification_banner(
        decision_debug: dict[str, Any] | None,
    ) -> tuple[str, tuple[int, int, int]] | None:
        """Translate the mission node's active grasp result into a banner."""
        if not isinstance(decision_debug, dict):
            return None
        verification = decision_debug.get("grasp_verification")
        if not isinstance(verification, dict) or not bool(
            verification.get("active", False)
        ):
            return None

        result = str(verification.get("result", "UNKNOWN")).upper()
        if result == "GRABBED":
            return "GRASP CHECK: GRABBED", (0, 255, 0)
        if result == "NOT_GRABBED":
            return "GRASP CHECK: NOT GRABBED", (0, 0, 255)
        return "GRASP CHECK: WAITING", (0, 165, 255)

    def _draw_grasp_verification_status(self, image: np.ndarray) -> None:
        """Show what motion_decision_node recognizes during grasp checking."""
        banner = self._grasp_verification_banner(
            self._fresh_decision_debug()
        )
        if banner is None:
            return

        text, color = banner
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.8
        thickness = 2
        (text_width, text_height), baseline = cv2.getTextSize(
            text, font, font_scale, thickness
        )
        center_x = image.shape[1] // 2
        text_x = max(8, center_x - text_width // 2)
        text_y = 64
        cv2.rectangle(
            image,
            (text_x - 8, text_y - text_height - 8),
            (text_x + text_width + 8, text_y + baseline + 8),
            (0, 0, 0),
            -1,
        )
        cv2.rectangle(
            image,
            (text_x - 8, text_y - text_height - 8),
            (text_x + text_width + 8, text_y + baseline + 8),
            color,
            2,
        )
        cv2.putText(
            image,
            text,
            (text_x, text_y),
            font,
            font_scale,
            color,
            thickness,
            cv2.LINE_AA,
        )

    def _publish_detections(
        self, message: Image, detections: list[Detection]
    ) -> None:
        payload = {
            "stamp": {
                "sec": int(message.header.stamp.sec),
                "nanosec": int(message.header.stamp.nanosec),
            },
            "frame_id": message.header.frame_id,
            "image_width": int(message.width),
            "image_height": int(message.height),
            "detections": [asdict(detection) for detection in detections],
        }
        output = String()
        output.data = json.dumps(
            payload, ensure_ascii=True, separators=(",", ":")
        )
        self.detections_publisher.publish(output)

    def _annotated_image_has_subscriber(self) -> bool:
        """Avoid copying a full RGB frame when only the local window uses it."""
        return bool(
            self.publish_annotated_image
            and self.annotated_publisher.get_subscription_count() > 0
        )

    def _image_callback(self, message: Image) -> None:
        now = time.monotonic()
        minimum_interval = 1.0 / self.max_fps if self.max_fps > 0 else 0.0
        waiting_for_interval = (
            now - self.last_inference_time < minimum_interval
        )
        if self.processing or waiting_for_interval:
            return

        # Limit callback start times, rather than adding the FPS interval after
        # every completed inference. Otherwise processing time and the interval
        # are both charged to each frame, making the real display FPS too low.
        self.last_inference_time = now
        self.processing = True
        started = time.perf_counter()
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            blob, info = self._preprocess(image)
            inference_started = time.perf_counter()
            output = self._run_inference(blob)
            inference_latency_ms = (
                time.perf_counter() - inference_started
            ) * 1000.0
            self._record_inference_timing(
                inference_latency_ms,
                time.monotonic(),
            )
            detections = self._postprocess(output, info, image.shape)
            elapsed = max(time.perf_counter() - started, 1e-6)
            current_fps = 1.0 / elapsed
            self.smoothed_fps = (
                current_fps
                if self.smoothed_fps == 0.0
                else self.smoothed_fps * 0.9 + current_fps * 0.1
            )

            self._publish_detections(message, detections)
            publish_annotated = self._annotated_image_has_subscriber()
            if publish_annotated or self.display:
                self._overlay_rgb_stamp_ns = image_stamp_ns(message)
                try:
                    annotated = self._draw_detections(image, detections)
                finally:
                    self._overlay_rgb_stamp_ns = None

                if publish_annotated:
                    annotated_message = self.bridge.cv2_to_imgmsg(
                        annotated, encoding="bgr8"
                    )
                    annotated_message.header = message.header
                    self.annotated_publisher.publish(annotated_message)

                if self.display:
                    display_image = annotated
                    if self.show_camera_controls:
                        display_image = self._draw_camera_control_panel(
                            annotated
                        )
                    cv2.imshow(DISPLAY_WINDOW_NAME, display_image)
                    key = cv2.waitKey(1) & 0xFF
                    if (
                        self.show_camera_controls
                        and key in {ord("r"), ord("R")}
                    ):
                        self._reset_camera_controls()
        except Exception as exc:
            self.get_logger().error(f"YOLO26 inference failed: {exc}")
        finally:
            self.processing = False

    def destroy_node(self) -> bool:
        if self.tensorrt_backend is not None:
            self.tensorrt_backend.close()
        if self.display:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: Yolo26Detector | None = None
    try:
        node = Yolo26Detector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        rclpy.logging.get_logger("yolo26_detector").fatal(str(exc))
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
