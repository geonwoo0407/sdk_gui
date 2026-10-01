#!/usr/bin/env python3
"""Estimate a coarse IRC mission state from vision topics."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@dataclass(frozen=True)
class MissionZone:
    """One coarse zone in the planned IRC course."""

    name: str
    mission: str
    expected_objects: list[str]
    description: str


@dataclass
class MissionState:
    """State published for the future algorithm node."""

    zone: str
    mission: str
    confidence: float
    expected_objects: list[str]
    line_detected: bool
    line_quality: float
    heading_error_deg: float | None
    lateral_offset_norm: float | None
    missed_line_frames: int
    last_seen_objects: list[str]
    state_age_sec: float
    note: str


COURSE_ZONES = [
    MissionZone(
        name="START",
        mission="wait_or_start",
        expected_objects=["line"],
        description="Initial robot position.",
    ),
    MissionZone(
        name="WALK_TO_BALL_A",
        mission="follow_line_to_ball_a",
        expected_objects=["line", "hurdle"],
        description="Walk along the line from start toward the first ball.",
    ),
    MissionZone(
        name="PICK_BALL_A",
        mission="pick_ball_a",
        expected_objects=["ball", "line"],
        description="Pick the first fixed ball.",
    ),
    MissionZone(
        name="SCORE_GOAL_A",
        mission="score_goal_a",
        expected_objects=["goal", "backboard"],
        description="Score the first ball into the nearby goal.",
    ),
    MissionZone(
        name="WALK_TO_BALL_B",
        mission="follow_line_to_ball_b",
        expected_objects=["line", "ball"],
        description="Walk along the line toward the second ball.",
    ),
    MissionZone(
        name="PICK_BALL_B",
        mission="pick_ball_b",
        expected_objects=["ball", "line"],
        description="Pick the second fixed ball.",
    ),
    MissionZone(
        name="SCORE_GOAL_B",
        mission="score_goal_b",
        expected_objects=["goal", "backboard"],
        description="Score the second ball into the nearby goal.",
    ),
    MissionZone(
        name="WALK_TO_FINISH",
        mission="follow_line_to_finish",
        expected_objects=["line"],
        description="Follow the line toward the finish area.",
    ),
    MissionZone(
        name="FINISH",
        mission="finish",
        expected_objects=[],
        description="End of the course.",
    ),
]


class MissionStateEstimator(Node):
    """Publish a simple zone/FSM state from line and object detections."""

    def __init__(self) -> None:
        super().__init__("mission_state_estimator")

        self.declare_parameter("line_info_topic", "/vision/line_info")
        self.declare_parameter("detections_topic", "/vision/detections")
        self.declare_parameter("mission_state_topic", "/vision/mission_state")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("line_timeout_sec", 0.7)
        self.declare_parameter("object_timeout_sec", 0.7)
        self.declare_parameter("min_object_confidence", 0.35)
        self.declare_parameter("line_quality_threshold", 0.45)
        self.declare_parameter("line_recovery_missed_frames", 4)
        self.declare_parameter("start_hold_sec", 5.0)

        self.line_timeout_sec = float(
            self.get_parameter("line_timeout_sec").value
        )
        self.object_timeout_sec = float(
            self.get_parameter("object_timeout_sec").value
        )
        self.min_object_confidence = float(
            self.get_parameter("min_object_confidence").value
        )
        self.line_quality_threshold = float(
            self.get_parameter("line_quality_threshold").value
        )
        self.line_recovery_missed_frames = int(
            self.get_parameter("line_recovery_missed_frames").value
        )
        self.start_hold_sec = float(
            self.get_parameter("start_hold_sec").value
        )

        line_info_topic = str(
            self.get_parameter("line_info_topic").value
        )
        detections_topic = str(
            self.get_parameter("detections_topic").value
        )
        mission_state_topic = str(
            self.get_parameter("mission_state_topic").value
        )

        self.publisher = self.create_publisher(
            String,
            mission_state_topic,
            10,
        )

        self.line_subscription = self.create_subscription(
            String,
            line_info_topic,
            self._line_info_callback,
            10,
        )

        self.detection_subscription = self.create_subscription(
            String,
            detections_topic,
            self._detections_callback,
            10,
        )

        publish_rate_hz = max(
            1.0,
            float(
                self.get_parameter("publish_rate_hz").value
            ),
        )

        self.timer = self.create_timer(
            1.0 / publish_rate_hz,
            self._publish_state,
        )

        self.latest_line_info: dict[str, Any] | None = None
        self.latest_line_time: float | None = None
        self.latest_objects: dict[str, dict[str, Any]] = {}
        self.latest_object_time: float | None = None
        self.current_zone_index = 1
        self.started_at = time.monotonic()

        self.get_logger().info(f"Subscribing line info: {line_info_topic}")
        self.get_logger().info(f"Subscribing detections: {detections_topic}")
        self.get_logger().info(f"Publishing mission state: {mission_state_topic}")

    def _line_info_callback(
        self,
        message: String,
    ) -> None:
        """Store latest line analysis JSON."""

        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("line_info JSON must be an object")
            self.latest_line_info = payload
            self.latest_line_time = time.monotonic()

        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                f"Invalid line_info message: {type(exc).__name__}: {exc}"
            )

    def _detections_callback(
        self,
        message: String,
    ) -> None:
        """Store the strongest recent detection per object class."""

        try:
            payload = json.loads(message.data)
            detections = payload.get("detections", [])
            if not isinstance(detections, list):
                raise ValueError("'detections' must be a list")

            objects: dict[str, dict[str, Any]] = {}

            for detection in detections:
                if not isinstance(detection, dict):
                    continue

                class_name = str(
                    detection.get("class_name", "")
                )

                if class_name == "line":
                    continue

                confidence = float(
                    detection.get("confidence", 0.0)
                )

                if confidence < self.min_object_confidence:
                    continue

                previous = objects.get(class_name)
                if (
                    previous is None
                    or confidence > float(previous.get("confidence", 0.0))
                ):
                    objects[class_name] = detection

            self.latest_objects = objects
            self.latest_object_time = time.monotonic()

        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                f"Invalid detection message: {type(exc).__name__}: {exc}"
            )

    def _line_is_fresh(self) -> bool:
        if self.latest_line_time is None:
            return False
        return (
            time.monotonic()
            - self.latest_line_time
        ) <= self.line_timeout_sec

    def _objects_are_fresh(self) -> bool:
        if self.latest_object_time is None:
            return False
        return (
            time.monotonic()
            - self.latest_object_time
        ) <= self.object_timeout_sec

    def _get_line_value(
        self,
        key: str,
        default: Any = None,
    ) -> Any:
        if not self._line_is_fresh() or self.latest_line_info is None:
            return default
        return self.latest_line_info.get(key, default)

    def _estimate_zone(self) -> tuple[str, str, list[str], float, str]:
        """Choose a coarse mission state from available vision signals."""

        line_detected = bool(
            self._get_line_value("detected", False)
        )

        heading_quality = float(
            self._get_line_value("heading_quality", 0.0) or 0.0
        )

        geometry_quality = float(
            self._get_line_value("geometry_quality", 0.0) or 0.0
        )

        detection_quality = float(
            self._get_line_value("detection_quality", 0.0) or 0.0
        )

        line_quality = min(
            heading_quality,
            geometry_quality,
            detection_quality,
        )

        missed_line_frames = int(
            self._get_line_value("missed_line_frames", 0) or 0
        )

        visible_objects = (
            set(self.latest_objects)
            if self._objects_are_fresh()
            else set()
        )

        elapsed = time.monotonic() - self.started_at

        if elapsed < self.start_hold_sec:
            zone = "START"
            mission = "wait_or_start"
            expected = ["line"]
            confidence = 0.70
            note = "Initial start-cell hold."

        elif "goal" in visible_objects or "backboard" in visible_objects:
            zone = "SCORE_GOAL_A"
            mission = "score_goal_a"
            expected = ["goal", "backboard", "line"]
            confidence = 0.80
            note = "Goal/backboard landmark is visible."

        elif "ball" in visible_objects:
            zone = "PICK_BALL_A"
            mission = "pick_ball_a"
            expected = ["ball", "goal", "line"]
            confidence = 0.78
            note = "Ball is visible."

        elif "hurdle" in visible_objects:
            zone = "WALK_TO_BALL_A"
            mission = "follow_line_to_ball_a"
            expected = ["hurdle", "line"]
            confidence = 0.75
            note = "Movable obstacle is visible; keep following line."

        elif (
            line_detected
            and line_quality >= self.line_quality_threshold
        ):
            zone = "WALK_TO_BALL_A"
            mission = "follow_line_to_ball_a"
            expected = ["line", "ball", "goal"]
            confidence = max(
                0.35,
                min(
                    line_quality,
                    1.0,
                ),
            )
            note = "Reliable line is visible."

        elif missed_line_frames >= self.line_recovery_missed_frames:
            zone = "LINE_RECOVERY"
            mission = "recover_line"
            expected = ["line"]
            confidence = 0.35
            note = "Line has been missing for several frames."

        else:
            zone = "UNKNOWN"
            mission = "hold_or_scan"
            expected = ["line"]
            confidence = 0.20
            note = "Vision state is not stable enough."

        return (
            zone,
            mission,
            expected,
            confidence,
            note,
        )

    def _publish_state(self) -> None:
        """Publish one mission-state JSON message."""

        (
            zone,
            mission,
            expected_objects,
            confidence,
            note,
        ) = self._estimate_zone()

        line_detected = bool(
            self._get_line_value("detected", False)
        )

        heading_quality = float(
            self._get_line_value("heading_quality", 0.0) or 0.0
        )

        geometry_quality = float(
            self._get_line_value("geometry_quality", 0.0) or 0.0
        )

        detection_quality = float(
            self._get_line_value("detection_quality", 0.0) or 0.0
        )

        line_quality = min(
            heading_quality,
            geometry_quality,
            detection_quality,
        )

        visible_objects = (
            sorted(self.latest_objects)
            if self._objects_are_fresh()
            else []
        )

        state = MissionState(
            zone=zone,
            mission=mission,
            confidence=round(confidence, 3),
            expected_objects=expected_objects,
            line_detected=line_detected,
            line_quality=round(line_quality, 3),
            heading_error_deg=self._get_line_value(
                "filtered_heading_error_deg"
            ),
            lateral_offset_norm=self._get_line_value(
                "filtered_lateral_offset_norm"
            ),
            missed_line_frames=int(
                self._get_line_value("missed_line_frames", 0) or 0
            ),
            last_seen_objects=visible_objects,
            state_age_sec=round(
                time.monotonic()
                - self.started_at,
                3,
            ),
            note=note,
        )

        message = String()
        message.data = json.dumps(
            asdict(state),
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self.publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    node = MissionStateEstimator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
