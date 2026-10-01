#!/usr/bin/env python3
"""Bridge navigation JSON commands to the C++ motion executor."""

from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .safety_interlock import RECOVERABLE_MOTOR_ERROR_CODES


LEFT_RECOVERY_MOTION_IDS = {
    2: "line_turn_left_4",
    4: "line_recovery_left_4",
    5: "line_recovery_left_5",
    6: "line_recovery_left_6",
    7: "line_recovery_left_7",
    8: "line_recovery_left_8",
    10: "line_turn_left_12",
    13: "line_turn_left_15",
}
RIGHT_RECOVERY_MOTION_IDS = {
    4: "line_recovery_right_4",
    5: "line_recovery_right_5",
    6: "line_recovery_right_6",
    7: "line_recovery_right_7",
    8: "line_recovery_right_8",
    10: "line_turn_right_8",
    12: "line_turn_right_10",
    15: "line_turn_right_large",
}


class MotionCommandBridgeNode(Node):
    """Translate supported navigation actions into SDK executor requests."""

    DWELL_MARKER = "__NON_BLOCKING_DWELL__"
    PICKUP_INITIAL_ALIGN_DWELL_MARKER = (
        "__BALL_PICKUP_INITIAL_ALIGN_DWELL__"
    )
    PICKUP_INITIAL_ALIGN_MARKER = "__BALL_PICKUP_INITIAL_ALIGN_CHECK__"
    FINE_ALIGN_MARKER = "__BALL_PICKUP_FINE_ALIGN_CHECK__"
    POST_BACKWARD_ALIGN_MARKER = (
        "__BALL_PICKUP_POST_BACKWARD_ALIGN_CHECK__"
    )
    PICKUP_POSITIONING_LOSS_LATCH_ACTION = (
        "BALL_PICKUP_POSITIONING_LOSS_LATCH"
    )
    PICKUP_FIXED_SEQUENCE_FIRST_MOTION = "pickup_pre_backward_camera_down"
    PICKUP_GRASP_CHECK_MOTION_ID = "pickup_grasp_check_pose"
    PICKUP_DWELL_SEC = 3.0
    PICKUP_INITIAL_ALIGN_ACTIONS = frozenset(
        {
            "BALL_PICKUP_INITIAL_ALIGN_CONTINUE",
            "BALL_PICKUP_INITIAL_CRAB_LEFT",
            "BALL_PICKUP_INITIAL_CRAB_RIGHT",
            *{
                f"BALL_PICKUP_CAMERA_DOWN_TURN_{direction}_{count}"
                for direction in ("LEFT", "RIGHT")
                for count in range(1, 10)
            },
        }
    )
    PICKUP_CAMERA_DOWN_TURN_MOTION_IDS = {
        **{
            f"BALL_PICKUP_CAMERA_DOWN_TURN_LEFT_{count}": (
                f"pickup_camera_down_turn_left_{count}"
            )
            for count in range(1, 10)
        },
        **{
            f"BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_{count}": (
                f"pickup_camera_down_turn_right_{count}"
            )
            for count in range(1, 10)
        },
    }
    PICKUP_FINE_ALIGN_ACTIONS = frozenset(
        {
            "BALL_PICKUP_FINE_ALIGN_CONTINUE",
            "BALL_PICKUP_FINE_FORWARD",
            "BALL_PICKUP_CRAB_RIGHT",
            "BALL_PICKUP_CRAB_LEFT",
        }
    )
    PICKUP_FINE_ALIGN_MOTION_IDS = {
        "BALL_PICKUP_FINE_FORWARD": "pickup_fine_forward_0",
        "BALL_PICKUP_CRAB_RIGHT": "pickup_crab_right_0",
        "BALL_PICKUP_CRAB_LEFT": "pickup_crab_left_0",
    }
    PICKUP_POST_BACKWARD_ALIGN_ACTIONS = frozenset(
        {
            "BALL_PICKUP_POST_BACKWARD_ALIGN_CONTINUE",
            "BALL_PICKUP_POST_BACKWARD_CRAB_RIGHT",
            "BALL_PICKUP_POST_BACKWARD_CRAB_LEFT",
            *{
                f"BALL_PICKUP_POST_BACKWARD_TURN_{direction}_{count}"
                for direction in ("LEFT", "RIGHT")
                for count in range(1, 10)
            },
        }
    )
    ATOMIC_SEQUENCE_ACTIONS = frozenset(
        {"PICKUP_NOW", "POST_BALL_GOAL_TRANSITION"}
    )

    PICKUP_CAMERA_DOWN_MOTION_IDS = {
        "STRAIGHT_0": "pickup_fine_forward_0",
        "STRAIGHT_2": "ball_camera_down_forward_4",
    }
    PICKUP_MOTION_TAIL = (
        "pickup_pre_backward_camera_down",
        "pickup",
        PICKUP_GRASP_CHECK_MOTION_ID,
        "pickup_retreat_3",
    )
    PICKUP_NO_BALL_MOTION_TAIL = (
        "pickup_fine_forward_0",
        "pickup_pre_backward_camera_down",
        POST_BACKWARD_ALIGN_MARKER,
        "pickup",
        PICKUP_GRASP_CHECK_MOTION_ID,
        "pickup_retreat_3",
    )
    POST_BALL_GOAL_TRANSITION_SEQUENCE = (
        "post_ball_forward_4",
        "post_ball_forward_8",
        "post_ball_camera_90",
    )
    ACTION_TO_MOTION_ID = {
        "STRAIGHT": "line_forward_6",
        "STRAIGHT_1": "line_forward_2",
        "STRAIGHT_2": "line_forward_4",
        "STRAIGHT_3": "line_forward_6",
        "STRAIGHT_4": "line_forward_8",
        "STRAIGHT_5": "line_forward_10",
        "APPROACH": "forward",
        "LEFT": "line_turn_left_15",
        "RIGHT": "line_turn_right_large",
        "POST_BALL_GOAL_TRANSITION": "post_ball_forward_4",
        "BALL_FINE_FORWARD_8": "ball_general_fine_forward_8",
        "GOAL_CAMERA_90_FORWARD": "goal_camera_90_forward_6",
        "GOAL_CAMERA_90_FORWARD_1": "goal_camera_90_forward_2",
        "GOAL_CAMERA_90_FORWARD_2": "goal_camera_90_forward_4",
        "GOAL_CAMERA_90_FORWARD_4": "goal_camera_90_forward_8",
        "GOAL_CAMERA90_CRAB_RIGHT": "goal_camera_90_crab_right",
        "GOAL_CAMERA90_CRAB_LEFT": "goal_camera_90_crab_left",
        **{
            f"POST_BALL_LINE_TURN_RIGHT_{count}": (
                f"post_ball_line_turn_right_{count}"
            )
            for count in range(1, 10)
        },
        **{
            f"POST_BALL_LINE_TURN_LEFT_{count}": (
                f"post_ball_line_turn_left_{count}"
            )
            for count in (2, 3, 4, 6)
        },
        **{
            f"BALL_APPROACH_TURN_RIGHT_{count}": (
                f"post_ball_line_turn_right_{count}"
            )
            for count in range(1, 10)
        },
        **{
            f"BALL_APPROACH_TURN_LEFT_{count}": (
                f"post_ball_line_turn_left_{count}"
            )
            for count in (2, 3, 4, 5, 6)
        },
        **{
            f"GOAL_CAMERA90_TURN_RIGHT_{count}": (
                f"goal_camera_90_turn_right_{count}"
            )
            for count in range(1, 10)
        },
        **{
            f"GOAL_CAMERA90_TURN_LEFT_{count}": (
                f"goal_camera_90_turn_left_{count}"
            )
            for count in range(1, 7)
        },
        "SHOT": "goal_shot",
        "GO": "hurdle",
        **{
            f"RECOVER_{line_side}_TURN_LEFT_{suffix}": motion_id
            for line_side in ("LEFT", "RIGHT")
            for suffix, motion_id in LEFT_RECOVERY_MOTION_IDS.items()
        },
        **{
            f"RECOVER_{line_side}_TURN_RIGHT_{suffix}": motion_id
            for line_side in ("LEFT", "RIGHT")
            for suffix, motion_id in RIGHT_RECOVERY_MOTION_IDS.items()
        },
        "TURN_LEFT": "stationary_turn_left",
        "TURN_RIGHT": "stationary_turn_right",
        "ALIGN_LEFT": "stationary_turn_left",
        "ALIGN_RIGHT": "stationary_turn_right",
    }
    TERMINAL_STATUSES = {
        "SUCCEEDED",
        "FAILED",
        "TIMEOUT",
        "CANCELLED",
        "REJECTED",
    }
    DEFAULT_TIMEOUT_MS = 12000

    def __init__(self) -> None:
        """Initialize bridge state, publishers, and subscriptions."""
        super().__init__("motion_command_bridge")

        self.last_sent_command_id: int | None = None
        self.motion_in_progress = False
        self.active_command_id: int | None = None
        self.active_event_id: int | None = None
        self.active_action: str | None = None
        self.active_request_id: int | None = None
        self.active_motion_id: str | None = None
        self.active_timeout_ms: int | None = None
        self.active_pickup_sequence: tuple[str, ...] = ()
        self.active_sequence_index = 0
        self.active_dwell_until: float | None = None
        self.pickup_initial_align_dwell_until: float | None = None
        self.pickup_initial_align_waiting = False
        self.pickup_initial_align_correction_active = False
        self.pickup_fine_align_waiting = False
        self.pickup_fine_align_correction_active = False
        self.pickup_post_backward_align_waiting = False
        self.pickup_post_backward_align_correction_active = False
        self.pickup_checkpoint_after_dwell: str | None = None
        self.pickup_positioning_loss_pending = False
        self.pickup_fixed_sequence_started = False
        self.pickup_fine_positioning_complete = False
        self.pickup_positioning_dwell_motion_id: str | None = None
        self.queued_command_id: int | None = None
        self.queued_event_id: int | None = None
        self.queued_action: str | None = None
        self.queued_request_id: int | None = None
        self.queued_motion_id: str | None = None
        self.queued_timeout_ms: int | None = None
        self.queued_request_deferred = False
        self.queued_pickup_sequence: tuple[str, ...] = ()

        self.navigation_subscription = self.create_subscription(
            String,
            "/navigation/motion_command",
            self.navigation_command_callback,
            10,
        )
        self.executor_status_subscription = self.create_subscription(
            String,
            "/motion/executor/status",
            self.executor_status_callback,
            10,
        )
        self.executor_request_publisher = self.create_publisher(
            String,
            "/motion/executor/request",
            10,
        )
        self.motion_status_publisher = self.create_publisher(
            String,
            "/motion/status",
            10,
        )
        self.dwell_timer = self.create_timer(
            0.05,
            self._check_atomic_dwell,
        )

        self.get_logger().info(
            "Motion command bridge ready: /navigation/motion_command -> "
            "/motion/executor/request, /motion/executor/status -> "
            "/motion/status"
        )

    @staticmethod
    def _is_integer(value: Any) -> bool:
        """Return true only for JSON integers, excluding booleans."""
        return isinstance(value, int) and not isinstance(value, bool)

    @classmethod
    def motion_id_for_action(cls, action: str) -> str | None:
        """Return a catalog-backed motion ID for one navigation action."""
        return cls.ACTION_TO_MOTION_ID.get(action)

    @classmethod
    def timeout_ms_from_payload(cls, payload: dict[str, Any]) -> int:
        """Read a positive timeout or use the safe default."""
        source_command = payload.get("source_command")
        candidates = []
        if isinstance(source_command, dict):
            candidates.append(source_command.get("timeout_ms"))
        candidates.append(payload.get("timeout_ms"))

        for candidate in candidates:
            if cls._is_integer(candidate) and candidate > 0:
                return candidate
        return cls.DEFAULT_TIMEOUT_MS

    @classmethod
    def _pickup_motion_sequence(
        cls,
        payload: dict[str, Any],
    ) -> tuple[str, ...] | None:
        """Build one catalog-backed sequence from planner and mission state."""
        source_command = payload.get("source_command")
        mission_progress = payload.get("mission_progress")
        if not isinstance(source_command, dict) or not isinstance(
            mission_progress, dict
        ):
            return None

        completed = mission_progress.get("pickups_completed")
        if (
            isinstance(completed, bool)
            or not isinstance(completed, int)
            or completed not in {0, 1}
        ):
            return None

        if completed == 0:
            final_stages = (
                "pickup_first_turn_right_9",
                cls.DWELL_MARKER,
                "pickup_first_to_right_back_camera_45",
            )
        else:
            final_stages = (
                "stationary_turn_left",
                cls.DWELL_MARKER,
            )
        return (*cls.PICKUP_MOTION_TAIL, *final_stages)

    def publish_motion_status(
        self,
        *,
        status: str,
        command_id: int | None,
        event_id: int | None,
        request_id: int | None,
        motion_id: str | None,
        action: str | None,
        error_code: str = "",
        message: str = "",
        completed_motion_id: str | None = None,
        verification_window_complete: bool | None = None,
    ) -> None:
        """Publish normalized status while preserving executor fields."""
        payload = {
            "status": status,
            "command_id": command_id,
            "event_id": event_id,
            "request_id": request_id,
            "motion_id": motion_id,
            "error_code": error_code,
            "message": message,
            "action": action,
            "source_node": "motion_command_bridge_node",
            "motion_in_progress": self.motion_in_progress,
        }
        if completed_motion_id is not None:
            payload["completed_motion_id"] = completed_motion_id
        if verification_window_complete is not None:
            payload["verification_window_complete"] = (
                verification_window_complete
            )
        output = String()
        output.data = json.dumps(
            payload,
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self.motion_status_publisher.publish(output)

    def _publish_local_rejection(
        self,
        *,
        status: str,
        command_id: int,
        event_id: int | None,
        action: str,
        error_code: str,
        message: str,
    ) -> None:
        """Report a command that is not forwarded to the executor."""
        self.publish_motion_status(
            status=status,
            command_id=command_id,
            event_id=event_id,
            request_id=command_id,
            motion_id=self.motion_id_for_action(action),
            action=action,
            error_code=error_code,
            message=message,
        )

    def _enter_pickup_fine_align_checkpoint(self) -> None:
        """Pause the atomic pickup while retaining its command ownership."""
        self.active_motion_id = self.FINE_ALIGN_MARKER
        self.pickup_fine_align_waiting = True
        self.pickup_fine_align_correction_active = False
        self.publish_motion_status(
            status="RUNNING",
            command_id=self.active_command_id,
            event_id=self.active_event_id,
            request_id=self.active_request_id,
            motion_id=self.FINE_ALIGN_MARKER,
            action=self.active_action,
            message="waiting for fresh BALL pickup fine alignment",
        )

    def _enter_pickup_initial_align_checkpoint(self) -> None:
        """Request a fresh Ball frame while retaining pickup ownership."""
        self.active_motion_id = self.PICKUP_INITIAL_ALIGN_MARKER
        self.pickup_initial_align_waiting = True
        self.pickup_initial_align_correction_active = False
        self.publish_motion_status(
            status="RUNNING",
            command_id=self.active_command_id,
            event_id=self.active_event_id,
            request_id=self.active_request_id,
            motion_id=self.PICKUP_INITIAL_ALIGN_MARKER,
            action=self.active_action,
            message="waiting for fresh BALL pickup heading alignment",
        )

    def _enter_pickup_post_backward_align_checkpoint(self) -> None:
        """Align the camera-down robot after the no-Ball fallback."""
        self.active_motion_id = self.POST_BACKWARD_ALIGN_MARKER
        self.pickup_post_backward_align_waiting = True
        self.pickup_post_backward_align_correction_active = False
        self.publish_motion_status(
            status="RUNNING",
            command_id=self.active_command_id,
            event_id=self.active_event_id,
            request_id=self.active_request_id,
            motion_id=self.POST_BACKWARD_ALIGN_MARKER,
            action=self.active_action,
            message="waiting for BALL alignment after pickup backward motion",
        )

    @classmethod
    def _with_pickup_motion_dwells(
        cls,
        sequence: tuple[str, ...],
    ) -> tuple[str, ...]:
        """Insert one non-blocking dwell after every catalog motion."""
        expanded: list[str] = []
        for index, stage in enumerate(sequence):
            expanded.append(stage)
            next_stage = (
                sequence[index + 1]
                if index + 1 < len(sequence)
                else None
            )
            if (
                not stage.startswith("__")
                and next_stage != cls.DWELL_MARKER
            ):
                expanded.append(cls.DWELL_MARKER)
        return tuple(expanded)

    def _start_pickup_checkpoint_dwell(self, checkpoint: str) -> None:
        """Wait three seconds after a correction before checking Vision."""
        self.pickup_initial_align_correction_active = False
        self.pickup_fine_align_correction_active = False
        self.pickup_post_backward_align_correction_active = False
        self.pickup_positioning_dwell_motion_id = self.active_motion_id
        self.active_motion_id = self.DWELL_MARKER
        self.active_dwell_until = time.monotonic() + self.PICKUP_DWELL_SEC
        self.pickup_checkpoint_after_dwell = checkpoint
        self.publish_motion_status(
            status="RUNNING",
            command_id=self.active_command_id,
            event_id=self.active_event_id,
            request_id=self.active_request_id,
            motion_id=self.DWELL_MARKER,
            action=self.active_action,
            message="pickup correction non-blocking dwell active",
        )
        self.get_logger().info(
            "Pickup correction dwell started: "
            f"{self.PICKUP_DWELL_SEC:.1f}s before {checkpoint}"
        )

    def _start_pickup_initial_align_dwell(self) -> None:
        """Hold still for three seconds before checking pickup heading."""
        self.active_motion_id = self.PICKUP_INITIAL_ALIGN_DWELL_MARKER
        self.pickup_initial_align_dwell_until = (
            time.monotonic() + self.PICKUP_DWELL_SEC
        )
        self.pickup_initial_align_waiting = False
        self.pickup_initial_align_correction_active = False
        self.publish_motion_status(
            status="RUNNING",
            command_id=self.active_command_id,
            event_id=self.active_event_id,
            request_id=self.active_request_id,
            motion_id=self.PICKUP_INITIAL_ALIGN_DWELL_MARKER,
            action=self.active_action,
            message="holding still before BALL pickup heading alignment",
        )

    def _handle_pickup_initial_align_command(
        self,
        *,
        action: str,
        command_id: int,
        event_id: int | None,
        payload: dict[str, Any],
    ) -> None:
        """Apply one fresh heading decision inside the pickup checkpoint."""
        parent_command_id = payload.get("active_special_command_id")
        parent_event_id = payload.get("active_special_event_id")
        checkpoint_matches = bool(
            self.motion_in_progress
            and self.active_action == "PICKUP_NOW"
            and self.pickup_initial_align_waiting
            and parent_command_id == self.active_command_id
            and parent_event_id == self.active_event_id
        )
        if not checkpoint_matches:
            self._publish_local_rejection(
                status="REJECTED",
                command_id=command_id,
                event_id=event_id,
                action=action,
                error_code="PICKUP_INITIAL_ALIGNMENT_NOT_ACTIVE",
                message="pickup initial-alignment checkpoint is not active",
            )
            return

        source_command = payload.get("source_command")

        self.last_sent_command_id = command_id
        self.pickup_initial_align_waiting = False
        if action == "BALL_PICKUP_INITIAL_ALIGN_CONTINUE":
            use_no_ball_pickup_path = bool(
                isinstance(source_command, dict)
                and source_command.get("use_no_ball_pickup_path") is True
            )
            approach_motion = (
                source_command.get("pickup_approach_motion")
                if isinstance(source_command, dict)
                else None
            )
            first_motion = self.PICKUP_CAMERA_DOWN_MOTION_IDS.get(
                approach_motion
            )
            if first_motion is None:
                self.pickup_initial_align_waiting = True
                self._publish_local_rejection(
                    status="REJECTED",
                    command_id=command_id,
                    event_id=event_id,
                    action=action,
                    error_code="UNSUPPORTED_PICKUP_DISTANCE_APPROACH",
                    message=(
                        "fresh pickup depth did not select a supported motion"
                    ),
                )
                return
            if use_no_ball_pickup_path:
                if (
                    self.active_pickup_sequence[
                        :len(self.PICKUP_MOTION_TAIL)
                    ] != self.PICKUP_MOTION_TAIL
                ):
                    self.pickup_initial_align_waiting = True
                    self._publish_local_rejection(
                        status="REJECTED",
                        command_id=command_id,
                        event_id=event_id,
                        action=action,
                        error_code="INVALID_PICKUP_NO_BALL_SEQUENCE",
                        message=(
                            "pickup sequence does not contain the expected "
                            "motion tail"
                        ),
                    )
                    return
                final_stages = self.active_pickup_sequence[
                    len(self.PICKUP_MOTION_TAIL):
                ]
                next_sequence = (
                    *self.PICKUP_NO_BALL_MOTION_TAIL,
                    *final_stages,
                )
                self.active_pickup_sequence = self._with_pickup_motion_dwells(
                    next_sequence
                )
                self.active_sequence_index = 0
                self.active_motion_id = first_motion
                self._publish_executor_request(
                    action=self.active_action,
                    command_id=self.active_command_id,
                    event_id=self.active_event_id,
                    request_id=self.active_request_id,
                    motion_id=first_motion,
                    timeout_ms=(
                        self.active_timeout_ms or self.DEFAULT_TIMEOUT_MS
                    ),
                )
                return

            if (
                approach_motion == "STRAIGHT_0"
                and self.pickup_fine_positioning_complete
            ):
                self._enter_pickup_fine_align_checkpoint()
                return

            self.pickup_fine_positioning_complete = bool(
                approach_motion == "STRAIGHT_0"
            )
            self.pickup_initial_align_correction_active = True
            self.active_motion_id = first_motion
            self._publish_executor_request(
                action=self.active_action,
                command_id=self.active_command_id,
                event_id=self.active_event_id,
                request_id=self.active_request_id,
                motion_id=first_motion,
                timeout_ms=self.active_timeout_ms or self.DEFAULT_TIMEOUT_MS,
            )
            return

        if action.startswith("BALL_PICKUP_INITIAL_CRAB_"):
            fine_action = action.replace("INITIAL_", "", 1)
            motion_id = self.PICKUP_FINE_ALIGN_MOTION_IDS[fine_action]
        else:
            motion_id = self.PICKUP_CAMERA_DOWN_TURN_MOTION_IDS[action]
        self.pickup_initial_align_correction_active = True
        self.active_motion_id = motion_id
        self._publish_executor_request(
            action=self.active_action,
            command_id=self.active_command_id,
            event_id=self.active_event_id,
            request_id=self.active_request_id,
            motion_id=motion_id,
            timeout_ms=self.active_timeout_ms or self.DEFAULT_TIMEOUT_MS,
        )

    def _handle_pickup_fine_align_command(
        self,
        *,
        action: str,
        command_id: int,
        event_id: int | None,
        payload: dict[str, Any],
    ) -> None:
        """Apply one Vision decision inside the active pickup checkpoint."""
        parent_command_id = payload.get("active_special_command_id")
        parent_event_id = payload.get("active_special_event_id")
        checkpoint_matches = bool(
            self.motion_in_progress
            and self.active_action == "PICKUP_NOW"
            and self.pickup_fine_align_waiting
            and parent_command_id == self.active_command_id
            and parent_event_id == self.active_event_id
        )
        if not checkpoint_matches:
            self._publish_local_rejection(
                status="REJECTED",
                command_id=command_id,
                event_id=event_id,
                action=action,
                error_code="PICKUP_FINE_ALIGNMENT_NOT_ACTIVE",
                message="pickup fine-alignment checkpoint is not active",
            )
            return

        self.last_sent_command_id = command_id
        self.pickup_fine_align_waiting = False
        if action == "BALL_PICKUP_FINE_ALIGN_CONTINUE":
            self.pickup_fixed_sequence_started = True
            self.active_pickup_sequence = self._with_pickup_motion_dwells(
                self.active_pickup_sequence
            )
            self.active_sequence_index = -1
            if not self._start_next_pickup_motion():
                self.pickup_fixed_sequence_started = False
                self.pickup_fine_align_waiting = True
                self._publish_local_rejection(
                    status="REJECTED",
                    command_id=command_id,
                    event_id=event_id,
                    action=action,
                    error_code="PICKUP_FINE_ALIGNMENT_RESUME_FAILED",
                    message="pickup sequence could not resume after alignment",
                )
            return

        motion_id = self.PICKUP_FINE_ALIGN_MOTION_IDS[action]
        # Intentionally keep no crab retry counter. Each completed correction
        # returns to the fresh pixel checkpoint until Vision reports centered.
        self.pickup_fine_align_correction_active = True
        self.active_motion_id = motion_id
        self._publish_executor_request(
            action=self.active_action,
            command_id=self.active_command_id,
            event_id=self.active_event_id,
            request_id=self.active_request_id,
            motion_id=motion_id,
            timeout_ms=self.active_timeout_ms or self.DEFAULT_TIMEOUT_MS,
        )

    def _enter_pickup_positioning_loss_reacquisition(
        self,
        checkpoint: str | None,
    ) -> None:
        """Resume pickup positioning at the existing camera-down checkpoint."""
        if checkpoint == self.FINE_ALIGN_MARKER:
            remaining_index = self.active_sequence_index
            if remaining_index >= 0:
                self.active_pickup_sequence = self.active_pickup_sequence[
                    remaining_index:
                ]
        elif checkpoint != self.PICKUP_INITIAL_ALIGN_MARKER:
            remaining_index = self.active_sequence_index + 1
            self.active_pickup_sequence = self.active_pickup_sequence[
                remaining_index:
            ]
        self.active_sequence_index = 0
        self.pickup_positioning_loss_pending = False
        self.pickup_fixed_sequence_started = False
        if (
            checkpoint == self.FINE_ALIGN_MARKER
            or self.pickup_fine_positioning_complete
        ):
            self._enter_pickup_fine_align_checkpoint()
        else:
            self._enter_pickup_initial_align_checkpoint()

    def _handle_pickup_positioning_loss_latch(
        self,
        *,
        command_id: int,
        event_id: int | None,
        payload: dict[str, Any],
    ) -> None:
        """Latch Vision loss without interrupting the active pickup motion."""
        parent_command_id = payload.get("active_special_command_id")
        parent_event_id = payload.get("active_special_event_id")
        source_command = payload.get("source_command")
        source_motion_id = (
            source_command.get("pickup_positioning_motion_id")
            if isinstance(source_command, dict)
            else None
        )
        actual_motion_running = bool(
            self.active_dwell_until is None
            and self.pickup_initial_align_dwell_until is None
            and isinstance(self.active_motion_id, str)
            and not self.active_motion_id.startswith("__")
        )
        request_matches = bool(
            self.motion_in_progress
            and self.active_action == "PICKUP_NOW"
            and not self.pickup_fixed_sequence_started
            and parent_command_id == self.active_command_id
            and parent_event_id == self.active_event_id
            and isinstance(source_command, dict)
            and source_command.get("pickup_positioning_ball_lost") is True
            and (
                actual_motion_running
                or self.pickup_fine_align_waiting
                or (
                    self.active_dwell_until is not None
                    and source_motion_id
                    == self.pickup_positioning_dwell_motion_id
                )
            )
        )
        if not request_matches:
            self._publish_local_rejection(
                status="REJECTED",
                command_id=command_id,
                event_id=event_id,
                action=self.PICKUP_POSITIONING_LOSS_LATCH_ACTION,
                error_code="PICKUP_POSITIONING_LOSS_LATCH_NOT_ACTIVE",
                message="pickup positioning motion is not active",
            )
            return

        self.last_sent_command_id = command_id
        self.pickup_positioning_loss_pending = True
        if self.pickup_fine_align_waiting:
            self.pickup_fine_align_waiting = False
            self._enter_pickup_positioning_loss_reacquisition(
                self.FINE_ALIGN_MARKER
            )

    def _handle_pickup_post_backward_align_command(
        self,
        *,
        action: str,
        command_id: int,
        event_id: int | None,
        payload: dict[str, Any],
    ) -> None:
        """Apply camera-down heading or lateral pickup alignment."""
        parent_command_id = payload.get("active_special_command_id")
        parent_event_id = payload.get("active_special_event_id")
        checkpoint_matches = bool(
            self.motion_in_progress
            and self.active_action == "PICKUP_NOW"
            and self.pickup_post_backward_align_waiting
            and parent_command_id == self.active_command_id
            and parent_event_id == self.active_event_id
        )
        if not checkpoint_matches:
            self._publish_local_rejection(
                status="REJECTED",
                command_id=command_id,
                event_id=event_id,
                action=action,
                error_code="PICKUP_POST_BACKWARD_ALIGNMENT_NOT_ACTIVE",
                message="post-backward pickup alignment is not active",
            )
            return

        self.last_sent_command_id = command_id
        self.pickup_post_backward_align_waiting = False
        if action == "BALL_PICKUP_POST_BACKWARD_ALIGN_CONTINUE":
            if not self._start_next_pickup_motion():
                self.pickup_post_backward_align_waiting = True
                self._publish_local_rejection(
                    status="REJECTED",
                    command_id=command_id,
                    event_id=event_id,
                    action=action,
                    error_code="PICKUP_POST_BACKWARD_ALIGNMENT_RESUME_FAILED",
                    message="pickup sequence could not resume after alignment",
                )
            return

        if action.startswith("BALL_PICKUP_POST_BACKWARD_TURN_"):
            initial_action = action.replace(
                "BALL_PICKUP_POST_BACKWARD_TURN_",
                "BALL_PICKUP_CAMERA_DOWN_TURN_",
                1,
            )
            motion_id = self.PICKUP_CAMERA_DOWN_TURN_MOTION_IDS[initial_action]
        else:
            fine_action = action.replace(
                "BALL_PICKUP_POST_BACKWARD_CRAB_",
                "BALL_PICKUP_CRAB_",
                1,
            )
            motion_id = self.PICKUP_FINE_ALIGN_MOTION_IDS[fine_action]
        self.pickup_post_backward_align_correction_active = True
        self.active_motion_id = motion_id
        self._publish_executor_request(
            action=self.active_action,
            command_id=self.active_command_id,
            event_id=self.active_event_id,
            request_id=self.active_request_id,
            motion_id=motion_id,
            timeout_ms=self.active_timeout_ms or self.DEFAULT_TIMEOUT_MS,
        )

    def _publish_executor_request(
        self,
        *,
        action: str,
        command_id: int,
        event_id: int | None,
        request_id: int,
        motion_id: str,
        timeout_ms: int,
    ) -> None:
        """Publish one validated request to the C++ executor."""
        request_payload = {
            "action": action,
            "command_id": command_id,
            "event_id": event_id,
            "request_id": request_id,
            "motion_id": motion_id,
            "timeout_ms": timeout_ms,
        }
        request_message = String()
        request_message.data = json.dumps(
            request_payload,
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self.executor_request_publisher.publish(request_message)

    def navigation_command_callback(self, msg: String) -> None:
        """Validate and translate one navigation command."""
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as exc:
            self.get_logger().warning(
                f"Invalid /navigation/motion_command JSON: {exc}"
            )
            return

        if not isinstance(payload, dict):
            self.get_logger().warning(
                "Invalid /navigation/motion_command: "
                "JSON root must be an object"
            )
            return

        action = payload.get("action")
        command_id = payload.get("command_id")
        event_id = payload.get("event_id")
        if payload.get("valid") is not True:
            self.get_logger().info("Command ignored: valid is not true")
            return
        if not self._is_integer(command_id):
            self.get_logger().warning(
                "Command ignored: command_id is missing or not an integer"
            )
            return
        if event_id is not None and not self._is_integer(event_id):
            self.get_logger().warning(
                "Command ignored: event_id is not an integer or null"
            )
            return
        if not isinstance(action, str) or not action:
            self.get_logger().warning(
                "Command ignored: action is missing or invalid"
            )
            return

        if action == self.PICKUP_POSITIONING_LOSS_LATCH_ACTION:
            self._handle_pickup_positioning_loss_latch(
                command_id=command_id,
                event_id=event_id,
                payload=payload,
            )
            return

        if action in self.PICKUP_FINE_ALIGN_ACTIONS:
            self._handle_pickup_fine_align_command(
                action=action,
                command_id=command_id,
                event_id=event_id,
                payload=payload,
            )
            return
        if action in self.PICKUP_POST_BACKWARD_ALIGN_ACTIONS:
            self._handle_pickup_post_backward_align_command(
                action=action,
                command_id=command_id,
                event_id=event_id,
                payload=payload,
            )
            return
        if action in self.PICKUP_INITIAL_ALIGN_ACTIONS:
            self._handle_pickup_initial_align_command(
                action=action,
                command_id=command_id,
                event_id=event_id,
                payload=payload,
            )
            return

        if command_id == self.last_sent_command_id:
            self._publish_local_rejection(
                status="REJECTED",
                command_id=command_id,
                event_id=event_id,
                action=action,
                error_code="DUPLICATE_COMMAND_ID",
                message="command_id was already sent to the executor",
            )
            return
        if (
            self.motion_in_progress
            and self.active_action in self.ATOMIC_SEQUENCE_ACTIONS
        ):
            self._publish_local_rejection(
                status="REJECTED",
                command_id=command_id,
                event_id=event_id,
                action=action,
                error_code="ATOMIC_SEQUENCE_LOCKED",
                message="an atomic BALL sequence owns the motion executor",
            )
            return
        if self.motion_in_progress and self.queued_request_id is not None:
            self._publish_local_rejection(
                status="REJECTED",
                command_id=command_id,
                event_id=event_id,
                action=action,
                error_code="QUEUE_FULL",
                message="one next motion is already queued",
            )
            return

        pickup_sequence: tuple[str, ...] = ()
        if action == "PICKUP_NOW":
            built_sequence = self._pickup_motion_sequence(payload)
            if built_sequence is None:
                self._publish_local_rejection(
                    status="UNSUPPORTED",
                    command_id=command_id,
                    event_id=event_id,
                    action=action,
                    error_code="UNSUPPORTED_PICKUP_SEQUENCE",
                    message=(
                        "pickup sequence requires ball mission index 0 or 1"
                    ),
                )
                return
            pickup_sequence = built_sequence
            motion_id = self.PICKUP_INITIAL_ALIGN_MARKER
        elif action == "POST_BALL_GOAL_TRANSITION":
            pickup_sequence = self.POST_BALL_GOAL_TRANSITION_SEQUENCE
            motion_id = pickup_sequence[0]
        else:
            motion_id = self.motion_id_for_action(action)
        if motion_id is None:
            goal_left_unavailable = action.startswith(
                "GOAL_CAMERA90_TURN_LEFT_"
            )
            self._publish_local_rejection(
                status="UNSUPPORTED",
                command_id=command_id,
                event_id=event_id,
                action=action,
                error_code=(
                    "GOAL_CAMERA90_LEFT_TURN_NOT_AVAILABLE"
                    if goal_left_unavailable
                    else "UNSUPPORTED_ACTION"
                ),
                message=(
                    "camera-90 goal left-turn motion is not in the catalog"
                    if goal_left_unavailable
                    else "action has no configured motion alias"
                ),
            )
            return

        request_id = command_id
        timeout_ms = self.timeout_ms_from_payload(payload)
        defer_until_active_finishes = bool(
            self.motion_in_progress and action == "PICKUP_NOW"
        )
        starts_with_initial_align_checkpoint = action == "PICKUP_NOW"
        if (
            not defer_until_active_finishes
            and not starts_with_initial_align_checkpoint
        ):
            self._publish_executor_request(
                action=action,
                command_id=command_id,
                event_id=event_id,
                request_id=request_id,
                motion_id=motion_id,
                timeout_ms=timeout_ms,
            )

        self.last_sent_command_id = command_id
        if self.motion_in_progress:
            self.queued_command_id = command_id
            self.queued_event_id = event_id
            self.queued_action = action
            self.queued_request_id = request_id
            self.queued_motion_id = motion_id
            self.queued_timeout_ms = timeout_ms
            self.queued_request_deferred = defer_until_active_finishes
            self.queued_pickup_sequence = pickup_sequence
        else:
            self.motion_in_progress = True
            self.active_command_id = command_id
            self.active_event_id = event_id
            self.active_action = action
            self.active_request_id = request_id
            self.active_motion_id = motion_id
            self.active_timeout_ms = timeout_ms
            self.active_pickup_sequence = pickup_sequence
            self.active_sequence_index = 0
            self.active_dwell_until = None
            self.pickup_positioning_loss_pending = False
            self.pickup_fixed_sequence_started = False
            self.pickup_fine_positioning_complete = False
            self.pickup_positioning_dwell_motion_id = None
            if starts_with_initial_align_checkpoint:
                self.active_sequence_index = -1
                self._start_pickup_initial_align_dwell()

    def _start_next_pickup_motion(self) -> bool:
        """Advance one atomic BALL sequence after a successful stage."""
        if self.active_action not in self.ATOMIC_SEQUENCE_ACTIONS:
            return False
        if self.active_dwell_until is not None:
            return True
        next_index = self.active_sequence_index + 1
        if next_index >= len(self.active_pickup_sequence):
            return False

        next_motion_id = self.active_pickup_sequence[next_index]
        self.active_sequence_index = next_index
        if next_motion_id == self.FINE_ALIGN_MARKER:
            self._enter_pickup_fine_align_checkpoint()
            return True
        if next_motion_id == self.POST_BACKWARD_ALIGN_MARKER:
            self._enter_pickup_post_backward_align_checkpoint()
            return True
        if next_motion_id == self.DWELL_MARKER:
            if self.active_action == "PICKUP_NOW":
                self.pickup_positioning_dwell_motion_id = (
                    self.active_motion_id
                )
            self.active_dwell_until = time.monotonic() + self.PICKUP_DWELL_SEC
            if self.active_action == "PICKUP_NOW":
                self.publish_motion_status(
                    status="RUNNING",
                    command_id=self.active_command_id,
                    event_id=self.active_event_id,
                    request_id=self.active_request_id,
                    motion_id=self.DWELL_MARKER,
                    action=self.active_action,
                    message="pickup sequence non-blocking dwell active",
                    completed_motion_id=(
                        self.pickup_positioning_dwell_motion_id
                    ),
                    verification_window_complete=False,
                )
            self.get_logger().info(
                f"Atomic sequence dwell started: {self.PICKUP_DWELL_SEC:.1f}s"
            )
            return True
        if (
            self.active_action == "PICKUP_NOW"
            and next_motion_id == self.PICKUP_FIXED_SEQUENCE_FIRST_MOTION
        ):
            self.pickup_fixed_sequence_started = True
        self.active_motion_id = next_motion_id
        self._publish_executor_request(
            action=self.active_action,
            command_id=self.active_command_id,
            event_id=self.active_event_id,
            request_id=self.active_request_id,
            motion_id=next_motion_id,
            timeout_ms=self.active_timeout_ms or self.DEFAULT_TIMEOUT_MS,
        )
        self.get_logger().info(
            f"Pickup sequence advanced to {next_motion_id}"
        )
        return True

    def _check_atomic_dwell(self, now: float | None = None) -> None:
        """Advance a dwell stage without blocking ROS callbacks."""
        if (
            self.active_dwell_until is None
            and self.pickup_initial_align_dwell_until is None
        ):
            return
        current_time = time.monotonic() if now is None else now
        if self.pickup_initial_align_dwell_until is not None:
            if current_time < self.pickup_initial_align_dwell_until:
                return
            self.pickup_initial_align_dwell_until = None
            self._enter_pickup_initial_align_checkpoint()
            return

        assert self.active_dwell_until is not None
        if current_time < self.active_dwell_until:
            return

        self.active_dwell_until = None
        completed_motion_id = self.pickup_positioning_dwell_motion_id
        if (
            self.active_action == "PICKUP_NOW"
            and completed_motion_id == self.PICKUP_GRASP_CHECK_MOTION_ID
        ):
            self.publish_motion_status(
                status="RUNNING",
                command_id=self.active_command_id,
                event_id=self.active_event_id,
                request_id=self.active_request_id,
                motion_id=self.DWELL_MARKER,
                action=self.active_action,
                message="grasp verification dwell completed",
                completed_motion_id=completed_motion_id,
                verification_window_complete=True,
            )
        self.pickup_positioning_dwell_motion_id = None
        self.pickup_fine_align_waiting = False
        self.pickup_fine_align_correction_active = False
        self.pickup_post_backward_align_waiting = False
        self.pickup_post_backward_align_correction_active = False
        checkpoint = self.pickup_checkpoint_after_dwell
        self.pickup_checkpoint_after_dwell = None
        if self.pickup_positioning_loss_pending:
            self._enter_pickup_positioning_loss_reacquisition(checkpoint)
            return
        if checkpoint == self.PICKUP_INITIAL_ALIGN_MARKER:
            self._enter_pickup_initial_align_checkpoint()
            return
        if checkpoint == self.FINE_ALIGN_MARKER:
            self._enter_pickup_fine_align_checkpoint()
            return
        if checkpoint == self.POST_BACKWARD_ALIGN_MARKER:
            self._enter_pickup_post_backward_align_checkpoint()
            return
        if self._start_next_pickup_motion():
            return

        command_id = self.active_command_id
        event_id = self.active_event_id
        request_id = self.active_request_id
        motion_id = self.active_motion_id
        action = self.active_action
        self._clear_active_request()
        self._promote_queued_request()
        self.publish_motion_status(
            status="SUCCEEDED",
            command_id=command_id,
            event_id=event_id,
            request_id=request_id,
            motion_id=motion_id,
            action=action,
            message="atomic sequence completed after non-blocking dwell",
        )

    def _valid_executor_status(self, payload: dict[str, Any]) -> bool:
        """Validate fields emitted by the C++ executor."""
        status = payload.get("status")
        command_id = payload.get("command_id")
        event_id = payload.get("event_id")
        request_id = payload.get("request_id")
        motion_id = payload.get("motion_id")
        error_code = payload.get("error_code")
        message = payload.get("message")
        return (
            status in {"RUNNING", *self.TERMINAL_STATUSES}
            and (command_id is None or self._is_integer(command_id))
            and (event_id is None or self._is_integer(event_id))
            and self._is_integer(request_id)
            and isinstance(motion_id, str)
            and isinstance(error_code, str)
            and isinstance(message, str)
        )

    def _clear_active_request(self) -> None:
        """Release the bridge after a terminal executor status."""
        self.motion_in_progress = False
        self.active_command_id = None
        self.active_event_id = None
        self.active_action = None
        self.active_request_id = None
        self.active_motion_id = None
        self.active_timeout_ms = None
        self.active_pickup_sequence = ()
        self.active_sequence_index = 0
        self.active_dwell_until = None
        self.pickup_initial_align_dwell_until = None
        self.pickup_initial_align_waiting = False
        self.pickup_initial_align_correction_active = False
        self.pickup_fine_align_waiting = False
        self.pickup_fine_align_correction_active = False
        self.pickup_post_backward_align_waiting = False
        self.pickup_post_backward_align_correction_active = False
        self.pickup_checkpoint_after_dwell = None
        self.pickup_positioning_loss_pending = False
        self.pickup_fixed_sequence_started = False
        self.pickup_fine_positioning_complete = False
        self.pickup_positioning_dwell_motion_id = None

    def _clear_queued_request(self) -> None:
        """Discard one queued request and any associated pickup sequence."""
        self.queued_command_id = None
        self.queued_event_id = None
        self.queued_action = None
        self.queued_request_id = None
        self.queued_motion_id = None
        self.queued_timeout_ms = None
        self.queued_request_deferred = False
        self.queued_pickup_sequence = ()

    def _promote_queued_request(self) -> None:
        self.active_command_id = self.queued_command_id
        self.active_event_id = self.queued_event_id
        self.active_action = self.queued_action
        self.active_request_id = self.queued_request_id
        self.active_motion_id = self.queued_motion_id
        self.active_timeout_ms = self.queued_timeout_ms
        self.active_pickup_sequence = self.queued_pickup_sequence
        self.active_sequence_index = 0
        self.active_dwell_until = None
        self.pickup_initial_align_dwell_until = None
        self.pickup_positioning_loss_pending = False
        self.pickup_fixed_sequence_started = False
        self.pickup_fine_positioning_complete = False
        self.pickup_positioning_dwell_motion_id = None
        queued_request_deferred = self.queued_request_deferred
        self._clear_queued_request()
        self.motion_in_progress = self.active_request_id is not None
        if queued_request_deferred and self.motion_in_progress:
            if self.active_action == "PICKUP_NOW":
                self._start_pickup_initial_align_dwell()
            else:
                self._publish_executor_request(
                    action=self.active_action,
                    command_id=self.active_command_id,
                    event_id=self.active_event_id,
                    request_id=self.active_request_id,
                    motion_id=self.active_motion_id,
                    timeout_ms=(
                        self.active_timeout_ms or self.DEFAULT_TIMEOUT_MS
                    ),
                )

    def executor_status_callback(self, msg: String) -> None:
        """Forward matching executor status and release terminal requests."""
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as exc:
            self.get_logger().warning(
                f"Invalid /motion/executor/status JSON: {exc}"
            )
            return
        if (
            not isinstance(payload, dict)
            or not self._valid_executor_status(payload)
        ):
            self.get_logger().warning(
                "Invalid /motion/executor/status fields"
            )
            return
        if not self.motion_in_progress:
            self.get_logger().info(
                "Executor status ignored: bridge has no active request"
            )
            return
        is_active = payload["request_id"] == self.active_request_id
        is_queued = payload["request_id"] == self.queued_request_id
        if not is_active and not is_queued:
            self.get_logger().warning(
                "Executor status ignored: request_id mismatch"
            )
            return

        action = self.active_action if is_active else self.queued_action
        if (
            is_active
            and action in self.ATOMIC_SEQUENCE_ACTIONS
            and payload["status"] == "FAILED"
            and payload["error_code"] in RECOVERABLE_MOTOR_ERROR_CODES
        ):
            self.get_logger().warning(
                "Recoverable motor fault during atomic sequence; "
                "continuing with the next stage: "
                f"motion_id={payload['motion_id']}, "
                f"error_code={payload['error_code']}"
            )
            payload["status"] = "SUCCEEDED"
        if (
            is_active
            and action in self.ATOMIC_SEQUENCE_ACTIONS
            and payload["motion_id"] != self.active_motion_id
        ):
            self.get_logger().warning(
                "Executor status ignored: pickup motion_id mismatch"
            )
            return
        if (
            is_active
            and action == "PICKUP_NOW"
            and self.pickup_initial_align_correction_active
            and payload["status"] == "SUCCEEDED"
        ):
            self._start_pickup_checkpoint_dwell(
                self.FINE_ALIGN_MARKER
                if self.pickup_fine_positioning_complete
                else self.PICKUP_INITIAL_ALIGN_MARKER
            )
            return
        if (
            is_active
            and action == "PICKUP_NOW"
            and self.pickup_fine_align_correction_active
            and payload["status"] == "SUCCEEDED"
        ):
            self._start_pickup_checkpoint_dwell(
                self.FINE_ALIGN_MARKER
            )
            return
        if (
            is_active
            and action == "PICKUP_NOW"
            and self.pickup_post_backward_align_correction_active
            and payload["status"] == "SUCCEEDED"
        ):
            self._start_pickup_checkpoint_dwell(
                self.POST_BACKWARD_ALIGN_MARKER
            )
            return
        if (
            is_active
            and payload["status"] == "SUCCEEDED"
            and self._start_next_pickup_motion()
        ):
            return
        if is_queued and payload["status"] in self.TERMINAL_STATUSES:
            self._clear_queued_request()
        elif is_active and payload["status"] in self.TERMINAL_STATUSES:
            self._clear_active_request()
            if payload["status"] == "SUCCEEDED":
                self._promote_queued_request()
            else:
                self._clear_queued_request()

        self.publish_motion_status(
            status=payload["status"],
            command_id=payload["command_id"],
            event_id=payload["event_id"],
            request_id=payload["request_id"],
            motion_id=payload["motion_id"],
            action=action,
            error_code=payload["error_code"],
            message=payload["message"],
        )


def main(args: list[str] | None = None) -> None:
    """Run the bridge node."""
    rclpy.init(args=args)
    node = MotionCommandBridgeNode()
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
