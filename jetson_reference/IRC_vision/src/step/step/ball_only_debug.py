#!/usr/bin/env python3
"""Run the production ONNX model on camera frames and inspect only balls."""

from __future__ import annotations

import ast
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Any

import cv2
from cv_bridge import CvBridge
import numpy as np
try:
    import onnxruntime as ort
except ImportError:
    ort = None
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .yolo26_detector import DEFAULT_CLASS_NAMES
from .yolo26_detector import DEFAULT_MODEL_PATH
from .yolo26_detector import LetterboxInfo


WINDOW_NAME = "BALL ONLY YOLO DEBUG"


@dataclass(frozen=True)
class RawCandidate:
    """One unfiltered end-to-end YOLO output row."""

    class_id: int
    confidence: float
    box: tuple[float, float, float, float]


def select_raw_candidates(
    output: np.ndarray,
    ball_class_id: int,
    max_candidates: int = 300,
) -> tuple[RawCandidate | None, RawCandidate | None]:
    """Return the strongest raw ball and strongest candidate of any class."""
    predictions = (
        np.squeeze(output, axis=0) if output.ndim == 3 else output
    )
    if predictions.ndim != 2 or predictions.shape[1] < 6:
        raise RuntimeError(f"Unexpected YOLO output shape: {output.shape}")

    strongest_ball: RawCandidate | None = None
    strongest_any: RawCandidate | None = None
    for prediction in predictions[:max(1, int(max_candidates))]:
        confidence = float(prediction[4])
        raw_class_id = float(prediction[5])
        if not math.isfinite(confidence) or not math.isfinite(raw_class_id):
            continue
        candidate = RawCandidate(
            class_id=int(round(raw_class_id)),
            confidence=confidence,
            box=tuple(float(value) for value in prediction[:4]),
        )
        if (
            strongest_any is None
            or candidate.confidence > strongest_any.confidence
        ):
            strongest_any = candidate
        if candidate.class_id == ball_class_id and (
            strongest_ball is None
            or candidate.confidence > strongest_ball.confidence
        ):
            strongest_ball = candidate
    return strongest_ball, strongest_any


class BallOnlyDebug(Node):
    """Display raw ball confidence without analyzers or mission control."""

    def __init__(self) -> None:
        super().__init__("ball_only_debug")
        self.declare_parameter("model_path", DEFAULT_MODEL_PATH)
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter(
            "debug_topic",
            "/vision/debug/ball_only",
        )
        self.declare_parameter("device", "cpu")
        self.declare_parameter("comparison_threshold", 0.20)
        self.declare_parameter("max_candidates", 300)
        self.declare_parameter("max_fps", 15.0)
        self.declare_parameter("display", True)

        self.model_path = Path(
            str(self.get_parameter("model_path").value)
        ).expanduser()
        self.comparison_threshold = float(
            self.get_parameter("comparison_threshold").value
        )
        if not 0.0 <= self.comparison_threshold <= 1.0:
            raise ValueError("comparison_threshold must be between 0 and 1")
        self.max_candidates = max(
            1,
            int(self.get_parameter("max_candidates").value),
        )
        self.max_fps = max(0.1, float(self.get_parameter("max_fps").value))
        self.display = bool(self.get_parameter("display").value)
        if not self.model_path.is_file():
            raise FileNotFoundError(f"YOLO model not found: {self.model_path}")
        if self.model_path.suffix.lower() != ".onnx":
            raise ValueError("ball_only_debug requires an ONNX model")

        self.session = self._create_session(
            str(self.get_parameter("device").value)
        )
        input_meta = self.session.get_inputs()[0]
        self.input_name = input_meta.name
        self.input_height = self._fixed_dimension(input_meta.shape[2], 640)
        self.input_width = self._fixed_dimension(input_meta.shape[3], 640)
        self.class_names = self._read_class_names()
        try:
            self.ball_class_id = self.class_names.index("ball")
        except ValueError as exc:
            raise RuntimeError(
                f"Model classes do not contain 'ball': {self.class_names}"
            ) from exc

        self.bridge = CvBridge()
        self.last_inference_time = 0.0
        self.last_log_time = 0.0
        self.smoothed_fps = 0.0
        debug_topic = str(self.get_parameter("debug_topic").value)
        image_topic = str(self.get_parameter("image_topic").value)
        self.publisher = self.create_publisher(String, debug_topic, 10)
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

        self.get_logger().info(f"Model: {self.model_path}")
        self.get_logger().info(f"Provider: {self.session.get_providers()[0]}")
        self.get_logger().info(f"Classes: {self.class_names}")
        self.get_logger().info(f"Ball class id: {self.ball_class_id}")
        self.get_logger().info(f"Subscribing: {image_topic}")
        self.get_logger().info(
            f"Comparison threshold: {self.comparison_threshold:.3f}"
        )

    @staticmethod
    def _fixed_dimension(value: Any, fallback: int) -> int:
        """Return a fixed positive model dimension."""
        return int(value) if isinstance(value, int) and value > 0 else fallback

    def _create_session(self, requested_device: str) -> ort.InferenceSession:
        """Create an ONNX Runtime session using a requested provider."""
        if ort is None:
            raise RuntimeError("ball_only_debug requires ONNX Runtime")
        available = ort.get_available_providers()
        device = requested_device.strip().lower()
        preferences = {
            "auto": [
                "CUDAExecutionProvider",
                "CPUExecutionProvider",
            ],
            "cuda": [
                "CUDAExecutionProvider",
                "CPUExecutionProvider",
            ],
            "cpu": ["CPUExecutionProvider"],
        }
        if device not in preferences:
            raise ValueError("device must be auto, cuda, or cpu")
        providers = [
            provider
            for provider in preferences[device]
            if provider in available
        ]
        if not providers:
            raise RuntimeError(f"No usable ONNX provider: {available}")
        return ort.InferenceSession(str(self.model_path), providers=providers)

    def _read_class_names(self) -> list[str]:
        """Read ordered class names from ONNX metadata."""
        raw_names = self.session.get_modelmeta().custom_metadata_map.get(
            "names"
        )
        if raw_names:
            try:
                names = ast.literal_eval(raw_names)
                if isinstance(names, dict):
                    return [str(names[index]) for index in sorted(names)]
                if isinstance(names, (list, tuple)):
                    return [str(name) for name in names]
            except (SyntaxError, ValueError, KeyError, TypeError):
                pass
        return DEFAULT_CLASS_NAMES.copy()

    def _preprocess(
        self,
        image: np.ndarray,
    ) -> tuple[np.ndarray, LetterboxInfo]:
        """Apply the production detector's letterbox and RGB conversion."""
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
        rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
        blob = rgb.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        return (
            np.ascontiguousarray(blob),
            LetterboxInfo(scale, float(left), float(top)),
        )

    @staticmethod
    def _source_box(
        candidate: RawCandidate,
        info: LetterboxInfo,
        image: np.ndarray,
    ) -> tuple[int, int, int, int] | None:
        """Map one model-space box to source-image coordinates."""
        height, width = image.shape[:2]
        x1, y1, x2, y2 = candidate.box
        left = int(np.clip(round((x1 - info.pad_x) / info.scale), 0, width - 1))
        right = int(np.clip(round((x2 - info.pad_x) / info.scale), 0, width - 1))
        top = int(np.clip(round((y1 - info.pad_y) / info.scale), 0, height - 1))
        bottom = int(np.clip(round((y2 - info.pad_y) / info.scale), 0, height - 1))
        if right <= left or bottom <= top:
            return None
        return left, top, right, bottom

    def _publish_debug(
        self,
        ball: RawCandidate | None,
        strongest: RawCandidate | None,
        bbox: tuple[int, int, int, int] | None,
    ) -> None:
        """Publish raw scores without influencing the production pipeline."""
        top_class = None
        if strongest is not None and 0 <= strongest.class_id < len(
            self.class_names
        ):
            top_class = self.class_names[strongest.class_id]
        payload = {
            "raw_ball_confidence": (
                round(ball.confidence, 6) if ball is not None else None
            ),
            "comparison_threshold": self.comparison_threshold,
            "passes_threshold": bool(
                ball is not None
                and ball.confidence >= self.comparison_threshold
            ),
            "ball_bbox": list(bbox) if bbox is not None else None,
            "strongest_class": top_class,
            "strongest_confidence": (
                round(strongest.confidence, 6)
                if strongest is not None
                else None
            ),
        }
        output = String()
        output.data = json.dumps(payload, separators=(",", ":"))
        self.publisher.publish(output)

    def _draw(
        self,
        image: np.ndarray,
        ball: RawCandidate | None,
        strongest: RawCandidate | None,
        bbox: tuple[int, int, int, int] | None,
    ) -> np.ndarray:
        """Draw raw ball score and strongest model output."""
        annotated = image.copy()
        ball_score = ball.confidence if ball is not None else None
        passes = bool(
            ball_score is not None
            and ball_score >= self.comparison_threshold
        )
        color = (0, 200, 0) if passes else (0, 165, 255)
        if bbox is not None and ball_score is not None and ball_score > 0.0:
            left, top, right, bottom = bbox
            cv2.rectangle(annotated, (left, top), (right, bottom), color, 3)
            cv2.putText(
                annotated,
                f"RAW BALL {ball_score:.4f}",
                (left, max(25, top - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                color,
                2,
                cv2.LINE_AA,
            )

        score_text = "NONE" if ball_score is None else f"{ball_score:.6f}"
        status = "PASS" if passes else "BELOW / NONE"
        top_name = "NONE"
        top_score = 0.0
        if strongest is not None:
            top_score = strongest.confidence
            if 0 <= strongest.class_id < len(self.class_names):
                top_name = self.class_names[strongest.class_id]
            else:
                top_name = f"CLASS_{strongest.class_id}"
        rows = [
            "BALL-ONLY RAW YOLO",
            f"RAW BALL SCORE : {score_text}",
            f"THRESHOLD      : {self.comparison_threshold:.3f}",
            f"RESULT         : {status}",
            f"TOP ANY        : {top_name} {top_score:.6f}",
            f"INFERENCE FPS  : {self.smoothed_fps:.1f}",
            "No analyzer / planner / mission control",
        ]
        overlay = annotated.copy()
        cv2.rectangle(overlay, (12, 12), (650, 230), (20, 20, 20), -1)
        annotated[12:230, 12:650] = cv2.addWeighted(
            overlay[12:230, 12:650],
            0.75,
            annotated[12:230, 12:650],
            0.25,
            0.0,
        )
        for index, row in enumerate(rows):
            cv2.putText(
                annotated,
                row,
                (28, 45 + index * 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255) if index == 0 else (245, 245, 245),
                2 if index == 0 else 1,
                cv2.LINE_AA,
            )
        return annotated

    def _image_callback(self, message: Image) -> None:
        """Run one independent raw ball inference."""
        now = time.monotonic()
        if now - self.last_inference_time < 1.0 / self.max_fps:
            return
        self.last_inference_time = now
        started = time.perf_counter()
        try:
            image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )
            blob, info = self._preprocess(image)
            output = self.session.run(None, {self.input_name: blob})[0]
            ball, strongest = select_raw_candidates(
                output,
                self.ball_class_id,
                self.max_candidates,
            )
            bbox = (
                self._source_box(ball, info, image)
                if ball is not None
                else None
            )
            elapsed = max(time.perf_counter() - started, 1e-6)
            current_fps = 1.0 / elapsed
            self.smoothed_fps = (
                current_fps
                if self.smoothed_fps == 0.0
                else self.smoothed_fps * 0.9 + current_fps * 0.1
            )
            self._publish_debug(ball, strongest, bbox)
            if now - self.last_log_time >= 1.0:
                score = ball.confidence if ball is not None else None
                self.get_logger().info(
                    "raw_ball_confidence="
                    f"{score if score is not None else 'NONE'}"
                )
                self.last_log_time = now
            if self.display:
                cv2.imshow(
                    WINDOW_NAME,
                    self._draw(image, ball, strongest, bbox),
                )
                key = cv2.waitKey(1) & 0xFF
                if key in {ord("q"), 27}:
                    rclpy.shutdown()
        except Exception as exc:
            self.get_logger().error(
                f"Ball-only inference failed: {type(exc).__name__}: {exc}"
            )

    def destroy_node(self) -> bool:
        """Close the diagnostic window before destroying the ROS node."""
        if self.display:
            try:
                cv2.destroyWindow(WINDOW_NAME)
            except cv2.error:
                pass
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the independent ball-only diagnostic node."""
    rclpy.init(args=args)
    node = BallOnlyDebug()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
