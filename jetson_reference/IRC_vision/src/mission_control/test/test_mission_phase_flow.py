"""End-to-end mission phase flow tests without ROS graph execution."""

import json

import pytest
from std_msgs.msg import String

from mission_control.legacy_motion_executor_adapter import (
    build_executor_request,
    map_action_to_motion_id,
)
from mission_control.legacy_motion_status_adapter import (
    convert_executor_status,
)
from mission_control.executor_heartbeat_watchdog import (
    ExecutorHeartbeatWatchdog,
)
from mission_control.mission_phase_manager import MissionPhaseManager
from mission_control.motion_command_gate import GeneralMotionCommandGate
from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_decision_planner import MotionDecisionPlanner
from mission_control.motion_executor_core import (
    ExecutorState,
    MotionExecutionResult,
)
from mission_control.motion_executor_node import (
    ExecutionPublicationState,
    parse_motion_request,
)
from mission_control.safety_interlock import SafetyInterlock


class CapturePublisher:
    """Collect JSON String messages in publication order."""

    def __init__(self):
        self.messages = []
        self.subscription_count = 1

    def publish(self, message):
        self.messages.append(json.loads(message.data))

    def get_subscription_count(self):
        return self.subscription_count


class FakeLogger:
    """Capture warnings while keeping flow tests quiet."""

    def __init__(self):
        self.warnings = []

    def info(self, _message):
        pass

    def warning(self, message):
        self.warnings.append(message)


class MissionFlowHarness:
    """Run real node decision methods against deterministic in-memory inputs."""

    SOURCES = MotionDecisionNode.SOURCES
    PRE_MOTION_SETTLE_ACTIONS = MotionDecisionNode.PRE_MOTION_SETTLE_ACTIONS
    SPECIAL_ACTIONS = MotionDecisionNode.SPECIAL_ACTIONS
    SPECIAL_ACTION_SOURCES = MotionDecisionNode.SPECIAL_ACTION_SOURCES
    PICKUP_INITIAL_ALIGN_MARKER = (
        MotionDecisionNode.PICKUP_INITIAL_ALIGN_MARKER
    )
    PICKUP_INITIAL_ALIGN_ACTIONS = (
        MotionDecisionNode.PICKUP_INITIAL_ALIGN_ACTIONS
    )
    PICKUP_FINE_ALIGN_MARKER = MotionDecisionNode.PICKUP_FINE_ALIGN_MARKER
    PICKUP_FINE_ALIGN_ACTIONS = MotionDecisionNode.PICKUP_FINE_ALIGN_ACTIONS
    PICKUP_POST_BACKWARD_ALIGN_MARKER = (
        MotionDecisionNode.PICKUP_POST_BACKWARD_ALIGN_MARKER
    )
    PICKUP_POST_BACKWARD_ALIGN_ACTIONS = (
        MotionDecisionNode.PICKUP_POST_BACKWARD_ALIGN_ACTIONS
    )
    PICKUP_POSITIONING_LOSS_LATCH_ACTION = (
        MotionDecisionNode.PICKUP_POSITIONING_LOSS_LATCH_ACTION
    )
    PICKUP_FIXED_SEQUENCE_FIRST_MOTION = (
        MotionDecisionNode.PICKUP_FIXED_SEQUENCE_FIRST_MOTION
    )
    BALL_POST_MOTION_DWELL_SEC = 0.0
    BALL_RAW_CONFIRMATION_RELEASE_SEC = (
        MotionDecisionNode.BALL_RAW_CONFIRMATION_RELEASE_SEC
    )
    SPECIAL_FAILURE_REASONS = (
        MotionDecisionNode.SPECIAL_FAILURE_REASONS
    )

    mission_phase = MotionDecisionNode.mission_phase
    required_pickups = MotionDecisionNode.required_pickups
    required_shots = MotionDecisionNode.required_shots
    required_ball_sections = MotionDecisionNode.required_ball_sections
    pickups_completed = MotionDecisionNode.pickups_completed
    shots_completed = MotionDecisionNode.shots_completed
    ball_sections_processed = MotionDecisionNode.ball_sections_processed
    finish_enabled = MotionDecisionNode.finish_enabled
    mission_complete = MotionDecisionNode.mission_complete
    active_special_action = MotionDecisionNode.active_special_action
    active_special_command_id = MotionDecisionNode.active_special_command_id
    special_motion_running = MotionDecisionNode.special_motion_running

    def __init__(
        self,
        phase="AUTO",
        required_ball_sections=2,
        max_pickup_failures=3,
        max_shot_failures=3,
        max_go_failures=3,
    ):
        self.phase_manager = MissionPhaseManager(
            initial_phase=phase,
            required_pickups=2,
            required_shots=2,
            required_ball_sections=required_ball_sections,
            max_pickup_failures=max_pickup_failures,
            max_shot_failures=max_shot_failures,
            max_go_failures=max_go_failures,
        )
        self.planner = MotionDecisionPlanner()
        self.general_motion_gate = GeneralMotionCommandGate()
        self.safety_interlock = SafetyInterlock()
        self.executor_heartbeat_watchdog = ExecutorHeartbeatWatchdog(
            started_at=0.0,
            startup_grace_sec=5.0,
            timeout_sec=2.0,
        )
        self.executor_heartbeat_watchdog.observe(
            sequence=0,
            observed_at=0.0,
        )
        self.executor_auto_ready = True
        self.executor_ready_requires_fresh_vision = False
        self.executor_active = False
        self.last_published_vision_stamp = {}
        self.active_general_source = None
        self.ball_approach_entry_pending = False
        self.ball_approach_alignment_pending = False
        self.ball_lost_during_motion_pending = False
        self.ball_last_visible_approach_info = None
        self.ball_last_visible_line_info = None
        self.ball_pickup_entry_pending = False
        self.ball_post_motion_dwell_until = None
        self.ball_confirmation_pending_latched = False
        self.ball_confirmation_last_raw_at = None
        self.active_line_motion_action = None
        self.active_line_motion_started_at = None
        self.active_line_motion_duration_sec = 0.0
        self.active_line_motion_target_frames = 0
        self.active_line_motion_frames = []
        self.line_timeout_recovery_active = False
        self.line_timeout_recovery_frames = []
        self.publisher = CapturePublisher()
        self._command_publisher_ready = None
        self.logger = FakeLogger()
        self.observations = {
            source: None for source in self.SOURCES
        }
        self.command_id = 0
        self.event_id = 0
        self.pre_motion_settle_sec = 0.0
        self.pre_motion_settle_source = None
        self.pre_motion_settle_action = None
        self.pre_motion_settle_started_at = None
        self.terminal_latch = None
        self.terminal_action_armed = {
            source: True
            for source in self.SPECIAL_ACTION_SOURCES.values()
        }
        self.active_special_event_id = None
        self.active_special_dynamics_command = None
        self.pickup_initial_align_waiting = False
        self.pickup_fine_align_waiting = False
        self.pickup_post_backward_align_waiting = False
        self.pickup_positioning_motion_running = False
        self.pickup_positioning_motion_id = None
        self.pickup_positioning_ball_seen_during_motion = False
        self.pickup_positioning_ball_lost_pending = False
        self.pickup_positioning_loss_latch_sent = False
        self.pickup_fixed_sequence_started = False
        self.finish_min_confidence = 0.70
        self.previous_publish_time = 0.0
        self.last_candidate_decision = None
        self.last_selected_decision = None

    def get_logger(self):
        return self.logger

    def _fresh_observations(self, _now):
        return dict(self.observations), {
            source: 0.0 if info is not None else None
            for source, info in self.observations.items()
        }

    def _rearm_absent_terminal_targets(self, observations):
        MotionDecisionNode._rearm_absent_terminal_targets(
            self, observations
        )

    def _select_mission_decision(self, observations, dt_sec):
        return MotionDecisionNode._select_mission_decision(
            self, observations, dt_sec
        )

    def _suppress_duplicate_terminal_action(self, decision):
        return MotionDecisionNode._suppress_duplicate_terminal_action(
            self, decision
        )

    def _suppress_exhausted_special_action(self, decision):
        return MotionDecisionNode._suppress_exhausted_special_action(
            self, decision
        )

    def _mission_progress(self):
        return MotionDecisionNode._mission_progress(self)

    def _latch_critical_executor_fault(self, **kwargs):
        return MotionDecisionNode._latch_critical_executor_fault(self, **kwargs)

    def _command_publisher_has_subscriber(self):
        return MotionDecisionNode._command_publisher_has_subscriber(self)

    def _reset_pre_motion_settle(self):
        return MotionDecisionNode._reset_pre_motion_settle(self)

    def _pre_motion_settle_ready(self, decision, now):
        return MotionDecisionNode._pre_motion_settle_ready(
            self,
            decision,
            now,
        )

    def publish_vision(self, **observations):
        self.observations.update(observations)
        self.general_motion_gate.on_new_vision_input()
        before = len(self.publisher.messages)
        MotionDecisionNode._publish_decision(self)
        return self.publisher.messages[before:]

    def send_status(self, action, command_id, status, motion_id=None):
        message = String()
        event_id = (
            self.active_special_event_id
            if command_id == self.active_special_command_id
            else None
        )
        message.data = json.dumps(
            {
                "action": action,
                "command_id": command_id,
                "event_id": event_id,
                "dynamics_command": None,
                "status": status,
                "motion_id": motion_id,
            }
        )
        MotionDecisionNode._motion_status_callback(self, message)

    def send_phase(self, phase):
        message = String()
        message.data = phase
        MotionDecisionNode._phase_callback(self, message)


def line_info(heading=0.0, offset=0.0):
    return {
        "detected": True,
        "filtered_heading_error_deg": heading,
        "filtered_lateral_offset_norm": offset,
        "heading_quality": 0.95,
        "geometry_quality": 0.95,
        "detection_quality": 0.95,
        "turn_angle_deg": 0.0,
        "turn_consistency": 1.0,
    }


def pickup_ready_ball():
    return {
        "detected": True,
        "confidence": 0.95,
        "bearing_deg": 0.0,
        "offset_x_px": 0,
        "offset_x_norm": 0.0,
        "camera_center_offset_x_px": 0,
        "bottom_distance_px": 500,
        "pickup_x_tolerance_norm": 0.08,
        "depth_m": 0.40,
        "ground_distance_m": 0.15,
        "distance_m": 0.40,
        "depth_valid": True,
        "depth_age_sec": 0.05,
        "pickup_ready": True,
        "pickup_now": True,
        "is_in_pickup_window": True,
    }


def score_ready_goal():
    return {
        "detected": True,
        "confidence": 0.95,
        "bearing_deg": 0.0,
        "offset_x_norm": 0.0,
        "depth_m": 0.25,
        "ground_distance_m": 0.25,
        "distance_m": 0.25,
        "depth_valid": True,
        "score_now": True,
    }


def go_ready_hurdle():
    return {
        "detected": True,
        "confirmation_confirmed": True,
        "confidence": 0.95,
        "depth_m": 0.10,
        "distance_m": 0.10,
        "depth_valid": True,
        "ground_gap_m": 0.10,
        "camera_bottom_gap_m": 0.05,
        "hurdle_angle_deg": 0.0,
        "go_now": True,
    }


def confirmed_finish():
    return {
        "detected": True,
        "confirmed": True,
        "confidence": 0.95,
    }


def approaching_ball():
    info = pickup_ready_ball()
    info.update(
        {
            "depth_m": 0.80,
            "ground_distance_m": 0.80,
            "distance_m": 0.80,
            "pickup_ready": False,
            "pickup_now": False,
        }
    )
    return info


def approaching_goal():
    info = score_ready_goal()
    info.update(
        {
            "depth_m": 0.49,
            "ground_distance_m": 0.49,
            "distance_m": 0.49,
            "score_now": False,
        }
    )
    return info


def aligning_goal():
    info = score_ready_goal()
    info.update(
        {
            "bearing_deg": 8.0,
            "offset_x_norm": 0.30,
            "score_now": False,
        }
    )
    return info


def approaching_hurdle():
    info = go_ready_hurdle()
    info.update(
        {
            "depth_m": 0.35,
            "distance_m": 0.35,
            "ground_gap_m": 0.35,
            "camera_bottom_gap_m": 0.20,
            "go_now": False,
        }
    )
    return info


def publish_special(harness, source, observation, action):
    published = harness.publish_vision(**{source: observation})
    matching = [
        payload for payload in published if payload["action"] == action
    ]
    assert len(matching) == 1
    payload = matching[0]
    assert payload["command_id"] == harness.active_special_command_id
    assert payload["sdk_motion_requested"] is True
    return payload


def mark_next_ball_grabbed(harness):
    """Seed the verified precondition for existing SHOT flow tests."""
    ball_index = harness.phase_manager.ball_sections_processed + 1
    harness.phase_manager.ball_grasp_results[ball_index] = "GRABBED"


def complete_active(harness, action, command_id, terminal="SUCCEEDED"):
    harness.send_status(action, command_id, "RUNNING")
    harness.send_status(action, command_id, terminal)


def release_general(harness, payload):
    harness.send_status(
        payload["action"],
        payload["command_id"],
        "RUNNING",
    )
    harness.send_status(
        payload["action"],
        payload["command_id"],
        "SUCCEEDED",
    )


def test_ball_pickup_entry_waits_for_general_motion_dwell(monkeypatch):
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    harness.BALL_POST_MOTION_DWELL_SEC = 3.0
    now = [10.0]
    monkeypatch.setattr(
        "mission_control.motion_decision_node.time.monotonic",
        lambda: now[0],
    )
    approach = harness.publish_vision(ball=approaching_ball())[-1]
    assert approach["action"] == "STRAIGHT_2"

    close_ball = pickup_ready_ball()
    close_ball.update(
        {
            "depth_m": 0.56,
            "distance_m": 0.56,
            "pickup_ready": False,
            "pickup_now": False,
        }
    )
    MotionDecisionNode._latch_ball_pickup_entry(harness, close_ball)

    assert harness.ball_pickup_entry_pending is True
    release_general(harness, approach)

    assert harness.ball_post_motion_dwell_until == pytest.approx(13.0)
    now[0] = 12.999
    assert harness.publish_vision(ball={"detected": False}) == []
    now[0] = 13.0
    assert harness.publish_vision(ball={"detected": False}) == []
    now[0] = 13.1
    published = harness.publish_vision(ball=close_ball)
    assert published[-1]["action"] == "PICKUP_NOW"
    assert published[-1]["reason"] == (
        "ball_pickup_entry_latched_during_motion"
    )
    assert harness.active_special_action == "PICKUP_NOW"
    assert harness.ball_pickup_entry_pending is False

    harness.send_status(
        "PICKUP_NOW",
        published[-1]["command_id"],
        "RUNNING",
        motion_id=MotionDecisionNode.PICKUP_INITIAL_ALIGN_MARKER,
    )
    assert harness.pickup_initial_align_waiting is True
    assert harness.observations["ball"] is None

    waiting = harness.publish_vision(line=line_info())[-1]
    assert waiting["action"] == "WAIT"
    assert waiting["source"] == "ball"

    right_ball = pickup_ready_ball()
    right_ball["steering_angle_deg"] = 26.0
    right_ball["offset_x_norm"] = 0.2
    alignment = harness.publish_vision(ball=right_ball)[-1]
    assert alignment["action"] == (
        "BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_3"
    )


def test_ball_loss_recovery_turn_is_followed_by_approach_not_another_turn(
    monkeypatch,
):
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    harness.BALL_POST_MOTION_DWELL_SEC = 3.0
    now = [10.0]
    monkeypatch.setattr(
        "mission_control.motion_decision_node.time.monotonic",
        lambda: now[0],
    )
    visible_ball = approaching_ball()
    visible_ball["steering_angle_deg"] = -31.0
    remembered_line = line_info(offset=-0.24)
    approach = harness.publish_vision(
        ball=visible_ball,
        line=remembered_line,
    )[-1]
    harness.ball_last_visible_line_info = remembered_line

    MotionDecisionNode._track_ball_loss_during_motion(
        harness,
        visible_ball,
    )
    lost_ball = {"detected": False, "raw_detected": False}
    MotionDecisionNode._track_ball_loss_during_motion(harness, lost_ball)
    release_general(harness, approach)

    assert harness.ball_post_motion_dwell_until == pytest.approx(13.0)
    now[0] = 12.999
    assert harness.publish_vision(ball=lost_ball) == []
    now[0] = 13.0
    assert harness.publish_vision(ball=lost_ball) == []
    now[0] = 13.1
    recovery = harness.publish_vision(ball=lost_ball)[-1]
    assert recovery["action"] == "BALL_APPROACH_TURN_LEFT_6"

    now[0] = 14.0
    release_general(harness, recovery)
    assert harness.ball_post_motion_dwell_until is None
    assert harness.ball_approach_alignment_pending is False
    misaligned_ball = approaching_ball()
    misaligned_ball["steering_angle_deg"] = 26.0
    fresh_decision = harness.publish_vision(ball=misaligned_ball)[-1]
    assert fresh_decision["action"] == "STRAIGHT_2"


def test_ball_motion_without_pickup_entry_keeps_three_second_dwell():
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    harness.BALL_POST_MOTION_DWELL_SEC = 3.0
    approach = harness.publish_vision(ball=approaching_ball())[-1]

    release_general(harness, approach)

    assert harness.ball_pickup_entry_pending is False
    assert harness.ball_post_motion_dwell_until is not None
    assert harness.publish_vision(ball=approaching_ball()) == []


def test_idle_ball_pickup_entry_crossing_is_latched_without_general_dwell():
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    close_ball = pickup_ready_ball()
    close_ball.update(
        {
            "depth_m": 0.56,
            "distance_m": 0.56,
            "pickup_ready": False,
            "pickup_now": False,
        }
    )

    MotionDecisionNode._latch_ball_pickup_entry(harness, close_ball)

    assert harness.ball_pickup_entry_pending is True
    assert harness.ball_post_motion_dwell_until is None
    published = harness.publish_vision(ball=close_ball)[-1]
    assert published["action"] == "PICKUP_NOW"


def test_failed_ball_motion_discards_latched_pickup_entry():
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    approach = harness.publish_vision(ball=approaching_ball())[-1]
    close_ball = pickup_ready_ball()
    close_ball["depth_m"] = 0.56
    close_ball["distance_m"] = 0.56
    MotionDecisionNode._latch_ball_pickup_entry(harness, close_ball)

    harness.send_status(
        approach["action"],
        approach["command_id"],
        "RUNNING",
    )
    harness.send_status(
        approach["action"],
        approach["command_id"],
        "FAILED",
    )

    assert harness.ball_pickup_entry_pending is False


def test_pickup_fine_alignment_rechecks_fresh_ball_under_atomic_lock():
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    pickup = publish_special(
        harness,
        "ball",
        pickup_ready_ball(),
        "PICKUP_NOW",
    )

    harness.send_status(
        "PICKUP_NOW",
        pickup["command_id"],
        "RUNNING",
        motion_id=MotionDecisionNode.PICKUP_FINE_ALIGN_MARKER,
    )
    assert harness.observations["ball"] is None
    assert harness.pickup_fine_align_waiting is True
    assert harness.active_special_command_id == pickup["command_id"]

    waiting = harness.publish_vision(
        line=line_info(),
        goal=score_ready_goal(),
        hurdle=go_ready_hurdle(),
    )[-1]
    assert waiting["action"] == "WAIT"
    assert waiting["source"] == "ball"

    right_ball = pickup_ready_ball()
    right_ball.update(
        {
            "offset_x_norm": 0.2,
            "offset_x_px": 51,
            "pickup_ready": False,
            "is_in_pickup_window": False,
        }
    )
    right = harness.publish_vision(ball=right_ball)[-1]
    assert right["action"] == "BALL_PICKUP_CRAB_RIGHT"
    assert right["active_special_command_id"] == pickup["command_id"]
    assert harness.pickup_fine_align_waiting is False

    harness.send_status(
        "PICKUP_NOW",
        pickup["command_id"],
        "RUNNING",
        motion_id=MotionDecisionNode.PICKUP_FINE_ALIGN_MARKER,
    )
    assert harness.observations["ball"] is None

    centered = harness.publish_vision(ball=pickup_ready_ball())[-1]
    assert centered["action"] == "BALL_PICKUP_FINE_ALIGN_CONTINUE"
    assert centered["active_special_command_id"] == pickup["command_id"]
    assert harness.active_special_action == "PICKUP_NOW"


def test_pickup_positioning_loss_returns_to_fresh_camera_down_alignment():
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    pickup = publish_special(
        harness,
        "ball",
        pickup_ready_ball(),
        "PICKUP_NOW",
    )
    harness.pickup_positioning_motion_running = True
    harness.pickup_positioning_motion_id = "ball_camera_down_forward_2"
    harness.pickup_positioning_ball_seen_during_motion = True
    lost_ball = {"detected": False, "raw_detected": False}
    MotionDecisionNode._track_pickup_positioning_ball_loss(
        harness,
        lost_ball,
    )

    latch = harness.publish_vision(ball=lost_ball)[-1]
    assert latch["action"] == harness.PICKUP_POSITIONING_LOSS_LATCH_ACTION
    assert latch["sdk_motion_requested"] is False
    assert latch["active_special_command_id"] == pickup["command_id"]

    harness.send_status(
        "PICKUP_NOW",
        pickup["command_id"],
        "RUNNING",
        motion_id=MotionDecisionNode.PICKUP_INITIAL_ALIGN_MARKER,
    )
    assert harness.pickup_initial_align_waiting is True
    waiting = harness.publish_vision(ball=lost_ball)[-1]
    assert waiting["action"] == "WAIT"
    assert waiting["sdk_motion_requested"] is False

    left_ball = pickup_ready_ball()
    left_ball["steering_angle_deg"] = -26.0
    left_ball["offset_x_norm"] = -0.2
    turn = harness.publish_vision(ball=left_ball)[-1]
    assert turn["action"] == "BALL_PICKUP_CAMERA_DOWN_TURN_LEFT_3"
    assert harness.pickup_positioning_ball_lost_pending is False


def test_pickup_initial_alignment_requires_fresh_ball_and_depth():
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    pickup = publish_special(
        harness,
        "ball",
        pickup_ready_ball(),
        "PICKUP_NOW",
    )

    harness.send_status(
        "PICKUP_NOW",
        pickup["command_id"],
        "RUNNING",
        motion_id=MotionDecisionNode.PICKUP_INITIAL_ALIGN_MARKER,
    )
    assert harness.observations["ball"] is None
    assert harness.pickup_initial_align_waiting is True
    assert harness.active_special_command_id == pickup["command_id"]

    waiting = harness.publish_vision(line=line_info())[-1]
    assert waiting["action"] == "WAIT"
    assert waiting["source"] == "ball"

    right_ball = pickup_ready_ball()
    right_ball.update(
        {
            "steering_angle_deg": 24.0,
            "bearing_deg": -20.0,
            "offset_x_norm": 0.2,
            "pickup_ready": False,
        }
    )
    right = harness.publish_vision(ball=right_ball)[-1]
    assert right["action"] == "BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_2"
    assert right["active_special_command_id"] == pickup["command_id"]
    assert harness.pickup_initial_align_waiting is False

    harness.send_status(
        "PICKUP_NOW",
        pickup["command_id"],
        "RUNNING",
        motion_id=MotionDecisionNode.PICKUP_INITIAL_ALIGN_MARKER,
    )
    centered = pickup_ready_ball()
    centered.update(
        {"depth_m": 0.55, "distance_m": 0.44, "pickup_ready": False}
    )
    approach = harness.publish_vision(ball=centered)[-1]
    assert approach["action"] == "BALL_PICKUP_INITIAL_ALIGN_CONTINUE"
    assert approach["source_command"]["pickup_approach_motion"] == "STRAIGHT_0"
    assert approach["source_command"]["distance_m"] == 0.44


def test_full_course_mock_flow_without_ros_graph():
    harness = MissionFlowHarness(required_ball_sections=1)

    straight = harness.publish_vision(line=line_info())
    assert [item["action"] for item in straight] == ["STRAIGHT"]
    repeated = harness.publish_vision(line=line_info())
    assert repeated == []
    release_general(harness, straight[0])

    recovery = harness.publish_vision(line=line_info(offset=-0.24))
    assert [item["action"] for item in recovery] == ["STRAIGHT"]
    assert recovery[0]["valid"] is True
    assert recovery[0]["reason"] == "line_tracking"
    assert map_action_to_motion_id(recovery[0]["action"]) == "forward"
    assert harness.general_motion_gate.locked is True
    release_general(harness, recovery[0])

    large = harness.publish_vision(line=line_info(offset=0.35))
    assert [item["action"] for item in large] == ["STRAIGHT"]
    assert large[0]["valid"] is True
    assert map_action_to_motion_id(large[0]["action"]) == "forward"
    release_general(harness, large[0])

    approach_ball = harness.publish_vision(
        line=line_info(),
        ball=approaching_ball(),
    )
    assert approach_ball[-1]["source"] == "ball"
    assert approach_ball[-1]["action"] == "STRAIGHT_2"
    release_general(harness, approach_ball[-1])

    pickup = publish_special(
        harness,
        "ball",
        pickup_ready_ball(),
        "PICKUP_NOW",
    )
    pickup_id = pickup["command_id"]

    harness.send_status("PICKUP_NOW", pickup_id + 10, "RUNNING")
    harness.send_status("SHOT", pickup_id, "RUNNING")
    assert harness.active_special_command_id == pickup_id
    assert harness.pickups_completed == 0

    harness.send_status("PICKUP_NOW", pickup_id, "RUNNING")
    assert harness.special_motion_running is True
    locked = harness.publish_vision(
        line=line_info(offset=0.35),
        ball=None,
        hurdle=None,
    )
    assert locked[-1]["action"] == "WAIT"
    assert locked[-1]["requires_ack"] is False
    assert harness.active_special_command_id == pickup_id

    harness.send_status("PICKUP_NOW", pickup_id, "SUCCEEDED")
    harness.send_status("PICKUP_NOW", pickup_id, "SUCCEEDED")
    assert harness.pickups_completed == 1
    assert harness.mission_phase == "POST_BALL_LINE_ALIGN"
    assert harness.active_special_command_id is None

    line_aligned = harness.publish_vision(line=line_info())
    assert line_aligned[-1]["action"] == "POST_BALL_GOAL_TRANSITION"
    assert harness.mission_phase == "POST_BALL_GOAL_TRANSITION"

    transition = line_aligned[-1]
    complete_active(
        harness,
        "POST_BALL_GOAL_TRANSITION",
        transition["command_id"],
    )
    assert harness.mission_phase == "GOAL_APPROACH"

    goal_approach = harness.publish_vision(
        ball=None,
        hurdle=None,
        goal=approaching_goal(),
    )
    assert goal_approach[-1]["action"] == "GOAL_CAMERA_90_FORWARD"
    release_general(harness, goal_approach[-1])
    goal_align = harness.publish_vision(goal=aligning_goal())
    assert goal_align[-1]["action"] == "GOAL_CAMERA90_TURN_RIGHT_1"
    harness.send_status(
        "GOAL_CAMERA90_TURN_RIGHT_1",
        goal_align[-1]["command_id"],
        "UNSUPPORTED",
    )

    mark_next_ball_grabbed(harness)
    shot = publish_special(
        harness,
        "goal",
        score_ready_goal(),
        "SHOT",
    )
    shot_id = shot["command_id"]
    harness.send_status("SHOT", shot_id, "RUNNING")
    shot_locked = harness.publish_vision(
        line=line_info(),
        goal=score_ready_goal(),
        hurdle=go_ready_hurdle(),
    )
    assert shot_locked[-1]["action"] == "WAIT"
    harness.send_status("SHOT", shot_id, "SUCCEEDED")

    assert harness.shots_completed == 1
    assert harness.ball_sections_processed == 1
    assert harness.finish_enabled is True
    assert harness.mission_phase == "LINE_TRACK"
    assert harness.mission_complete is False

    hurdle_approach = harness.publish_vision(
        ball=None,
        goal=None,
        hurdle=approaching_hurdle(),
    )
    assert hurdle_approach[-1]["source"] == "hurdle"
    assert hurdle_approach[-1]["action"] == "STRAIGHT_2"
    release_general(harness, hurdle_approach[-1])

    go = publish_special(
        harness,
        "hurdle",
        go_ready_hurdle(),
        "GO",
    )
    go_id = go["command_id"]
    assert go["action"] == "GO"
    assert map_action_to_motion_id("GO") == "forward"
    assert go["action"] != "STRAIGHT"

    harness.send_status("GO", go_id, "RUNNING")
    go_locked = harness.publish_vision(
        line=line_info(),
        hurdle=go_ready_hurdle(),
    )
    assert go_locked[-1]["action"] == "WAIT"
    harness.send_status("GO", go_id, "SUCCEEDED")
    assert harness.mission_phase == "LINE_TRACK"

    harness.planner._clear_ball_tracking()
    harness.planner._clear_goal_tracking()
    final_line = harness.publish_vision(
        line=line_info(),
        ball=None,
        goal=None,
        hurdle=None,
        finish=confirmed_finish(),
    )
    assert final_line[-1]["action"] == "STRAIGHT"
    assert final_line[-1]["phase"] == "LINE_TRACK"
    assert harness.mission_phase == "LINE_TRACK"
    assert harness.mission_complete is False
    assert all(
        item["action"] != "CROSS_FINISH"
        for item in harness.publisher.messages
    )
    assert all(
        item["phase"] != "FINISHED"
        for item in harness.publisher.messages
    )


@pytest.mark.parametrize(
    ("action", "phase", "source", "observation"),
    [
        ("PICKUP_NOW", "BALL_APPROACH", "ball", pickup_ready_ball()),
        ("SHOT", "GOAL_APPROACH", "goal", score_ready_goal()),
        ("GO", "HURDLE_APPROACH", "hurdle", go_ready_hurdle()),
    ],
)
@pytest.mark.parametrize("terminal", ["FAILED", "TIMEOUT"])
def test_special_failure_or_timeout_returns_to_safe_approach_phase(
    action,
    phase,
    source,
    observation,
    terminal,
):
    harness = MissionFlowHarness(phase=phase)
    if action == "SHOT":
        mark_next_ball_grabbed(harness)
    command = publish_special(
        harness,
        source,
        observation,
        action,
    )
    complete_active(
        harness,
        action,
        command["command_id"],
        terminal,
    )

    expected_phase = {
        "PICKUP_NOW": "BALL_APPROACH",
        "SHOT": "GOAL_APPROACH",
        "GO": "HURDLE_APPROACH",
    }[action]
    assert harness.mission_phase == expected_phase
    assert harness.ball_sections_processed == 0
    assert harness.active_special_command_id is None


@pytest.mark.parametrize(
    (
        "action",
        "phase",
        "source",
        "observation",
        "reason",
    ),
    [
        (
            "PICKUP_NOW",
            "BALL_APPROACH",
            "ball",
            pickup_ready_ball(),
            "pickup_failures_exhausted",
        ),
        (
            "SHOT",
            "GOAL_APPROACH",
            "goal",
            score_ready_goal(),
            "shot_failures_exhausted",
        ),
        (
            "GO",
            "HURDLE_APPROACH",
            "hurdle",
            go_ready_hurdle(),
            "go_failures_exhausted",
        ),
    ],
)
def test_special_failure_limit_blocks_reexecution_after_target_reappears(
    action,
    phase,
    source,
    observation,
    reason,
):
    harness = MissionFlowHarness(
        phase=phase,
        max_pickup_failures=1,
        max_shot_failures=1,
        max_go_failures=1,
    )
    if action == "SHOT":
        mark_next_ball_grabbed(harness)

    first = publish_special(
        harness,
        source,
        observation,
        action,
    )

    complete_active(
        harness,
        action,
        first["command_id"],
        "FAILED",
    )

    assert harness.phase_manager.special_action_exhausted(action)

    if source == "ball":
        harness.planner._clear_ball_tracking()
    elif source == "goal":
        harness.planner._clear_goal_tracking()

    harness.publish_vision(
        **{source: None}
    )

    assert harness.terminal_action_armed[source] is True

    reappeared = harness.publish_vision(
        **{source: observation}
    )

    decision = reappeared[-1]

    assert decision["action"] == "WAIT"
    assert decision["valid"] is False
    assert decision["reason"] == reason
    assert harness.active_special_command_id is None

    special_commands = [
        item
        for item in harness.publisher.messages
        if item["action"] == action
    ]
    assert len(special_commands) == 1


def test_full_flow_line_loss_stops_until_reacquisition():
    planner = MotionDecisionPlanner()
    observations = {
        source: None for source in MotionDecisionNode.SOURCES
    }

    observations["line"] = line_info(offset=-0.35)
    large = planner.plan("LINE_TRACK", observations, 0.1)
    assert large.action == "STRAIGHT"

    observations["line"] = None
    first_missing = planner.plan("LINE_TRACK", observations, 0.1)
    recovered = planner.plan("LINE_TRACK", observations, 0.1)
    assert first_missing.action == "STOP"
    assert recovered.action == "STOP"

    observations["line"] = line_info()
    reacquired = planner.plan("LINE_TRACK", observations, 0.1)
    assert reacquired.action == "STRAIGHT"

    observations["line"] = line_info(offset=0.35)
    planner.plan("LINE_TRACK", observations, 0.1)
    observations["line"] = None
    repeated_missing = [
        planner.plan("LINE_TRACK", observations, 0.1)
        for _ in range(4)
    ]
    assert all(item.action == "STOP" for item in repeated_missing)
    assert all(
        item.reason == "waiting_for_line_info"
        for item in repeated_missing
    )


@pytest.mark.parametrize(
    "phase",
    ["AUTO", "BALL_APPROACH", "LINE_TRACK"],
)
def test_hurdle_selection_does_not_depend_on_course_phase(phase):
    harness = MissionFlowHarness(phase=phase)

    published = harness.publish_vision(
        line=line_info(),
        hurdle=go_ready_hurdle(),
    )
    commands = [item for item in published if item["action"] == "GO"]
    assert len(commands) == 1
    command = commands[0]

    assert command["phase"] == phase
    assert command["source"] == "hurdle"


def test_go_ready_confirmed_hurdle_beats_pickup_ready_ball():
    harness = MissionFlowHarness()

    published = harness.publish_vision(
        line=line_info(),
        ball=pickup_ready_ball(),
        hurdle=go_ready_hurdle(),
    )

    assert published[-1]["source"] == "hurdle"
    assert published[-1]["action"] == "GO"
    assert harness.active_special_action == "GO"


def test_confirmed_approaching_hurdle_beats_approaching_ball():
    harness = MissionFlowHarness()

    published = harness.publish_vision(
        line=line_info(),
        ball=approaching_ball(),
        hurdle=approaching_hurdle(),
    )

    assert published[-1]["source"] == "hurdle"
    assert published[-1]["action"] == "STRAIGHT_2"


def test_goal_approach_resumes_after_hurdle_go_and_ignores_new_ball():
    harness = MissionFlowHarness(phase="GOAL_APPROACH")
    command = publish_special(
        harness,
        "hurdle",
        go_ready_hurdle(),
        "GO",
    )

    harness.send_status("GO", command["command_id"], "RUNNING")
    locked = harness.publish_vision(
        line=line_info(),
        ball=pickup_ready_ball(),
        goal=score_ready_goal(),
        hurdle=go_ready_hurdle(),
    )
    assert locked[-1]["action"] == "WAIT"

    harness.send_status("GO", command["command_id"], "SUCCEEDED")
    assert harness.mission_phase == "GOAL_APPROACH"

    mark_next_ball_grabbed(harness)
    resumed = harness.publish_vision(
        ball=pickup_ready_ball(),
        goal=score_ready_goal(),
        hurdle=None,
    )
    assert resumed[-1]["source"] == "goal"
    assert resumed[-1]["action"] == "SHOT"


def test_hurdle_can_appear_after_shot_without_fixed_order():
    harness = MissionFlowHarness(
        phase="GOAL_APPROACH",
        required_ball_sections=1,
    )
    mark_next_ball_grabbed(harness)
    shot = publish_special(
        harness,
        "goal",
        score_ready_goal(),
        "SHOT",
    )
    complete_active(harness, "SHOT", shot["command_id"])
    assert harness.mission_phase == "LINE_TRACK"

    hurdle = publish_special(
        harness,
        "hurdle",
        go_ready_hurdle(),
        "GO",
    )

    assert hurdle["source"] == "hurdle"
    assert hurdle["command_id"] > shot["command_id"]


def test_missing_or_stale_hurdle_preserves_active_hurdle_lock():
    harness = MissionFlowHarness()
    no_hurdle = harness.publish_vision(
        line=line_info(),
        hurdle=None,
    )
    assert no_hurdle[-1]["action"] == "STRAIGHT"
    release_general(harness, no_hurdle[-1])

    seen = harness.publish_vision(
        line=line_info(),
        hurdle=approaching_hurdle(),
    )
    assert seen[-1]["source"] == "hurdle"
    assert seen[-1]["action"] == "STRAIGHT_2"
    release_general(harness, seen[-1])

    missing = harness.publish_vision(hurdle=None)
    assert missing[-1]["source"] == "hurdle"
    assert missing[-1]["action"] == "WAIT"
    assert missing[-1]["reason"] == "waiting_for_hurdle_info"

    harness.latest_info = {
        source: None for source in MotionDecisionNode.SOURCES
    }
    harness.latest_time = {
        source: None for source in MotionDecisionNode.SOURCES
    }
    harness.timeouts = {
        source: 0.5 for source in MotionDecisionNode.SOURCES
    }
    harness.latest_info["line"] = line_info()
    harness.latest_time["line"] = 1.5
    harness.latest_info["hurdle"] = go_ready_hurdle()
    harness.latest_time["hurdle"] = 1.0
    harness.terminal_action_armed["hurdle"] = False

    observations, ages = MotionDecisionNode._fresh_observations(
        harness,
        now=1.6,
    )
    MotionDecisionNode._rearm_absent_terminal_targets(
        harness,
        observations,
    )
    decision = harness.planner.plan("AUTO", observations, 0.1)

    assert observations["hurdle"] is None
    assert ages["hurdle"] == pytest.approx(0.6)
    assert decision.source == "hurdle"
    assert decision.action == "WAIT"
    assert decision.reason == "waiting_for_hurdle_info"
    assert harness.terminal_action_armed["hurdle"] is True


@pytest.mark.parametrize(
    ("overrides", "reason"),
    [
        (
            {"confirmation_confirmed": False},
            "hurdle_confirmation_pending",
        ),
        (
            {"depth_valid": False},
            "hurdle_depth_invalid_wait",
        ),
    ],
)
def test_unconfirmed_or_invalid_depth_hurdle_does_not_preempt_ball(
    overrides,
    reason,
):
    harness = MissionFlowHarness()
    hurdle = go_ready_hurdle()
    hurdle.update(overrides)

    published = harness.publish_vision(
        line=line_info(),
        ball=pickup_ready_ball(),
        hurdle=hurdle,
    )
    decision = published[-1]

    assert decision["source"] == "ball"
    assert decision["action"] == "PICKUP_NOW"
    assert decision["valid"] is True
    assert harness.active_special_action == "PICKUP_NOW"


def test_unconfirmed_hurdle_disappearance_resumes_ball_mission():
    harness = MissionFlowHarness()
    hurdle = go_ready_hurdle()
    hurdle["confirmation_confirmed"] = False

    held = harness.publish_vision(
        ball=approaching_ball(),
        hurdle=hurdle,
    )
    assert held[-1]["source"] == "ball"
    assert held[-1]["action"] == "STRAIGHT_2"
    release_general(harness, held[-1])

    resumed = harness.publish_vision(hurdle=None)
    assert resumed[-1]["source"] == "ball"
    assert resumed[-1]["action"] == "STRAIGHT_2"


def test_unconfirmed_hurdle_can_become_confirmed_and_take_priority():
    harness = MissionFlowHarness()
    hurdle = approaching_hurdle()
    hurdle["confirmation_confirmed"] = False

    held = harness.publish_vision(
        line=line_info(),
        ball=approaching_ball(),
        hurdle=hurdle,
    )
    assert held[-1]["source"] == "ball"
    assert held[-1]["action"] == "STRAIGHT_2"
    release_general(harness, held[-1])

    hurdle["confirmation_confirmed"] = True
    selected = harness.publish_vision(hurdle=hurdle)
    assert selected[-1]["source"] == "hurdle"
    assert selected[-1]["action"] == "STRAIGHT_2"


def test_absent_hurdle_keeps_ball_priority_over_line():
    harness = MissionFlowHarness()

    published = harness.publish_vision(
        line=line_info(),
        ball=approaching_ball(),
        hurdle=None,
    )

    assert published[-1]["source"] == "ball"
    assert published[-1]["action"] == "STRAIGHT_2"


def test_go_running_and_terminal_suppress_same_hurdle_observation():
    harness = MissionFlowHarness()
    command = publish_special(
        harness,
        "hurdle",
        go_ready_hurdle(),
        "GO",
    )
    command_id = command["command_id"]

    harness.send_status("GO", command_id, "RUNNING")
    running = harness.publish_vision(hurdle=go_ready_hurdle())
    assert running[-1]["action"] == "WAIT"
    assert running[-1]["reason"] == (
        "mission_locked_waiting_for_motion_status"
    )

    harness.send_status("GO", command_id, "SUCCEEDED")
    retained = harness.publish_vision(hurdle=go_ready_hurdle())
    assert retained[-1]["action"] == "WAIT"
    assert retained[-1]["reason"] == (
        "duplicate_terminal_action_suppressed"
    )
    assert harness.active_special_command_id is None
    assert harness.terminal_action_armed["hurdle"] is False
    assert not any(
        item["action"] == "GO"
        and item["command_id"] > command_id
        for item in harness.publisher.messages
    )


def test_completed_hurdle_mission_lock_waits_when_hurdle_disappears():
    harness = MissionFlowHarness()
    hurdle = go_ready_hurdle()
    first = publish_special(harness, "hurdle", hurdle, "GO")
    complete_active(harness, "GO", first["command_id"])

    retained = harness.publish_vision(hurdle=hurdle)
    assert retained[-1]["action"] == "WAIT"

    resumed = harness.publish_vision(
        ball=pickup_ready_ball(),
        hurdle=None,
    )
    assert resumed[-1]["source"] == "hurdle"
    assert resumed[-1]["action"] == "WAIT"
    assert resumed[-1]["reason"] == "waiting_for_hurdle_info"


def test_hurdle_can_reappear_at_a_different_position_after_absence():
    harness = MissionFlowHarness()
    first_hurdle = go_ready_hurdle()
    first_hurdle.update({"offset_x_norm": -0.70, "depth_m": 0.10})
    first = publish_special(harness, "hurdle", first_hurdle, "GO")
    complete_active(harness, "GO", first["command_id"])

    retained = harness.publish_vision(hurdle=first_hurdle)
    assert retained[-1]["action"] == "WAIT"

    absent = harness.publish_vision(hurdle=None)
    assert absent[-1]["action"] == "WAIT"
    assert harness.terminal_action_armed["hurdle"] is True

    second_hurdle = go_ready_hurdle()
    second_hurdle.update({"offset_x_norm": 0.75, "depth_m": 0.12})
    second = publish_special(harness, "hurdle", second_hurdle, "GO")

    assert second["command_id"] > first["command_id"]
    assert harness.observations["hurdle"]["offset_x_norm"] == 0.75
    assert second["source_command"]["depth_m"] == 0.12


def test_pickup_success_publishes_once_and_advances_to_goal():
    harness = MissionFlowHarness()
    command = publish_special(
        harness,
        "ball",
        pickup_ready_ball(),
        "PICKUP_NOW",
    )
    command_id = command["command_id"]
    repeated = harness.publish_vision(ball=pickup_ready_ball())

    assert not any(
        payload["action"] == "PICKUP_NOW" for payload in repeated
    )
    assert harness.active_special_action == "PICKUP_NOW"
    assert harness.pickups_completed == 0

    harness.send_status("PICKUP_NOW", command_id, "RUNNING")
    assert harness.mission_phase == "AUTO"
    assert harness.pickups_completed == 0
    assert harness.special_motion_running is True

    harness.send_status("PICKUP_NOW", command_id, "SUCCEEDED")
    assert harness.active_special_action is None
    assert harness.active_special_command_id is None
    assert harness.pickups_completed == 1
    assert harness.mission_phase == "POST_BALL_LINE_ALIGN"


def test_first_ball_line_alignment_is_closed_loop_before_forward_transition():
    harness = MissionFlowHarness()
    pickup = publish_special(
        harness,
        "ball",
        pickup_ready_ball(),
        "PICKUP_NOW",
    )
    complete_active(harness, "PICKUP_NOW", pickup["command_id"])
    assert harness.mission_phase == "POST_BALL_LINE_ALIGN"

    missing = harness.publish_vision(line=None)
    assert missing[-1]["action"] == "WAIT"
    assert harness.mission_phase == "POST_BALL_LINE_ALIGN"

    correction = harness.publish_vision(line=line_info(heading=34.0))[-1]
    assert correction["action"] == "POST_BALL_LINE_TURN_RIGHT_3"
    assert correction["source_command"]["turn_count"] == 3
    release_general(harness, correction)

    published_count = len(harness.publisher.messages)
    MotionDecisionNode._publish_decision(harness)
    assert len(harness.publisher.messages) == published_count
    assert harness.mission_phase == "POST_BALL_LINE_ALIGN"

    aligned = harness.publish_vision(line=line_info(heading=0.0))[-1]
    assert aligned["action"] == "POST_BALL_GOAL_TRANSITION"
    assert harness.mission_phase == "POST_BALL_GOAL_TRANSITION"


def test_post_ball_line_turn_timeout_does_not_enter_line_lost_recovery():
    harness = MissionFlowHarness(phase="POST_BALL_LINE_ALIGN")
    correction = harness.publish_vision(line=line_info(heading=34.0))[-1]

    harness.send_status(
        correction["action"],
        correction["command_id"],
        "RUNNING",
    )
    harness.send_status(
        correction["action"],
        correction["command_id"],
        "TIMEOUT",
    )

    assert harness.mission_phase == "POST_BALL_LINE_ALIGN"
    assert harness.line_timeout_recovery_active is False


def test_goal_camera90_turn_rechecks_fresh_goal_before_forward():
    harness = MissionFlowHarness(phase="GOAL_APPROACH")

    correction = harness.publish_vision(
        goal={
            **approaching_goal(),
            "depth_m": 0.5,
            "ground_distance_m": 0.5,
            "distance_m": 0.5,
            "bearing_deg": 34.0,
            "offset_x_norm": 0.3,
        }
    )[-1]
    assert correction["action"] == "GOAL_CAMERA90_TURN_RIGHT_3"
    release_general(harness, correction)

    published_count = len(harness.publisher.messages)
    MotionDecisionNode._publish_decision(harness)
    assert len(harness.publisher.messages) == published_count

    forward = harness.publish_vision(
        goal={
            **approaching_goal(),
            "depth_m": 0.5,
            "ground_distance_m": 0.5,
            "distance_m": 0.5,
            "bearing_deg": 0.0,
            "offset_x_norm": 0.0,
        }
    )[-1]
    assert forward["action"] == "GOAL_CAMERA_90_FORWARD"


def test_goal_camera90_crab_rechecks_full_priority_with_fresh_goal():
    harness = MissionFlowHarness(phase="GOAL_APPROACH")

    crab = harness.publish_vision(
        goal={
            **approaching_goal(),
            "depth_m": 0.49,
            "ground_distance_m": 0.49,
            "distance_m": 0.49,
            "bearing_deg": 0.0,
            "offset_x_norm": 0.2,
        }
    )[-1]
    assert crab["action"] == "GOAL_CAMERA90_CRAB_RIGHT"
    release_general(harness, crab)

    published_count = len(harness.publisher.messages)
    MotionDecisionNode._publish_decision(harness)
    assert len(harness.publisher.messages) == published_count

    yaw = harness.publish_vision(
        goal={
            **approaching_goal(),
            "depth_m": 0.49,
            "ground_distance_m": 0.49,
            "distance_m": 0.49,
            "bearing_deg": -24.0,
            "offset_x_norm": 0.2,
        }
    )[-1]
    assert yaw["action"] == "GOAL_CAMERA90_TURN_LEFT_2"


def test_goal_camera90_forward_rechecks_depth_bucket_from_fresh_goal():
    harness = MissionFlowHarness(phase="GOAL_APPROACH")
    far = {
        **approaching_goal(),
        "depth_m": 0.65,
        "ground_distance_m": 0.65,
        "distance_m": 0.65,
        "bearing_deg": 0.0,
        "offset_x_norm": 0.0,
    }

    first = harness.publish_vision(goal=far)[-1]
    assert first["action"] == "GOAL_CAMERA_90_FORWARD_4"
    release_general(harness, first)

    published_count = len(harness.publisher.messages)
    MotionDecisionNode._publish_decision(harness)
    assert len(harness.publisher.messages) == published_count

    closer = dict(far)
    closer["depth_m"] = 0.50
    closer["ground_distance_m"] = 0.50
    closer["distance_m"] = 0.50
    second = harness.publish_vision(goal=closer)[-1]
    assert second["action"] == "GOAL_CAMERA_90_FORWARD"


def test_stale_wrong_and_duplicate_pickup_statuses_are_ignored():
    harness = MissionFlowHarness()
    command = publish_special(
        harness,
        "ball",
        pickup_ready_ball(),
        "PICKUP_NOW",
    )
    command_id = command["command_id"]

    harness.send_status("PICKUP_NOW", command_id + 1, "SUCCEEDED")
    harness.send_status("SHOT", command_id, "SUCCEEDED")
    assert harness.pickups_completed == 0
    assert harness.active_special_command_id == command_id

    complete_active(harness, "PICKUP_NOW", command_id)
    harness.send_status("PICKUP_NOW", command_id, "SUCCEEDED")
    assert harness.pickups_completed == 1
    assert harness.mission_phase == "POST_BALL_LINE_ALIGN"


def test_shot_success_updates_one_section_and_returns_to_auto():
    harness = MissionFlowHarness(
        phase="GOAL_APPROACH",
        required_ball_sections=2,
    )
    mark_next_ball_grabbed(harness)
    command = publish_special(
        harness,
        "goal",
        score_ready_goal(),
        "SHOT",
    )
    complete_active(harness, "SHOT", command["command_id"])

    assert harness.shots_completed == 1
    assert harness.ball_sections_processed == 1
    assert harness.finish_enabled is False
    assert harness.mission_phase == "AUTO"


def test_last_shot_enables_finish_flag_and_continues_line_driving():
    harness = MissionFlowHarness(
        phase="GOAL_APPROACH",
        required_ball_sections=2,
    )
    harness.phase_manager.ball_sections_processed = 1
    mark_next_ball_grabbed(harness)
    command = publish_special(
        harness,
        "goal",
        score_ready_goal(),
        "SHOT",
    )
    complete_active(harness, "SHOT", command["command_id"])

    assert harness.ball_sections_processed == 2
    assert harness.finish_enabled is True
    assert harness.mission_phase == "LINE_TRACK"

    # Model the goal becoming stale after the existing recovery window.
    harness.planner._clear_goal_tracking()
    line_commands = harness.publish_vision(
        line=line_info(),
        goal=None,
    )
    assert any(
        command["action"] == "STRAIGHT" for command in line_commands
    )
    assert all(command["phase"] == "LINE_TRACK" for command in line_commands)
    assert all(
        command["action"] != "CROSS_FINISH"
        for command in line_commands
    )


def test_final_line_track_ignores_new_ball_after_all_sections_complete():
    harness = MissionFlowHarness()

    harness.phase_manager.ball_sections_processed = (
        harness.phase_manager.required_ball_sections
    )
    harness.phase_manager.finish_enabled = True
    harness.phase_manager.set_phase("LINE_TRACK")

    published = harness.publish_vision(
        line=line_info(),
        ball=pickup_ready_ball(),
        goal=None,
        hurdle=None,
    )

    decision = published[-1]

    assert decision["phase"] == "LINE_TRACK"
    assert decision["source"] == "line"
    assert decision["action"] == "STRAIGHT"


def test_final_line_track_still_prioritizes_confirmed_hurdle():
    harness = MissionFlowHarness()

    harness.phase_manager.ball_sections_processed = (
        harness.phase_manager.required_ball_sections
    )
    harness.phase_manager.finish_enabled = True
    harness.phase_manager.set_phase("LINE_TRACK")

    published = harness.publish_vision(
        line=line_info(),
        ball=pickup_ready_ball(),
        goal=None,
        hurdle=approaching_hurdle(),
    )

    decision = published[-1]

    assert decision["phase"] == "LINE_TRACK"
    assert decision["source"] == "hurdle"
    assert decision["action"] == "STRAIGHT_2"



def test_final_line_track_still_holds_for_unconfirmed_hurdle():
    harness = MissionFlowHarness()

    harness.phase_manager.ball_sections_processed = (
        harness.phase_manager.required_ball_sections
    )
    harness.phase_manager.finish_enabled = True
    harness.phase_manager.set_phase("LINE_TRACK")

    hurdle = approaching_hurdle()
    hurdle["confirmation_confirmed"] = False

    published = harness.publish_vision(
        line=line_info(),
        ball=pickup_ready_ball(),
        goal=None,
        hurdle=hurdle,
    )

    decision = published[-1]

    assert decision["source"] == "line"
    assert decision["action"] == "STRAIGHT"



def test_go_round_trip_preserves_action_and_origin_phase():
    harness = MissionFlowHarness(phase="HURDLE_APPROACH")
    before = harness.phase_manager.snapshot()
    command = publish_special(
        harness,
        "hurdle",
        go_ready_hurdle(),
        "GO",
    )
    command_id = command["command_id"]

    request_dict = build_executor_request(
        request_id=7,
        motion_id="forward",
        command_id=command_id,
        action="GO",
    )
    request = parse_motion_request(json.dumps(request_dict))
    publication = ExecutionPublicationState()
    publication.begin(request)
    running = convert_executor_status(publication.running_payload())
    terminal = publication.terminal_payload(
        MotionExecutionResult(
            motion_id="forward",
            final_status=ExecutorState.SUCCEEDED,
            success=True,
            error_code="NONE",
            message="done",
        )
    )
    succeeded = convert_executor_status(terminal)

    assert running["action"] == "GO"
    assert succeeded["action"] == "GO"
    assert running["command_id"] == command_id
    assert succeeded["command_id"] == command_id

    harness.send_status(
        running["action"], running["command_id"], running["status"]
    )
    assert harness.special_motion_running is True
    assert harness.phase_manager.current_phase == "HURDLE_APPROACH"
    harness.send_status(
        succeeded["action"],
        succeeded["command_id"],
        succeeded["status"],
    )

    assert harness.mission_phase == "HURDLE_APPROACH"
    assert harness.active_special_action is None
    assert harness.pickups_completed == before["pickups_completed"]
    assert harness.shots_completed == before["shots_completed"]
    assert (
        harness.ball_sections_processed
        == before["ball_sections_processed"]
    )


def test_manual_cross_finish_status_compatibility_still_enters_finished():
    harness = MissionFlowHarness(
        phase="WALK_TO_FINISH",
        required_ball_sections=0,
    )
    assert harness.phase_manager.start_special_action("CROSS_FINISH", 10)
    complete_active(harness, "CROSS_FINISH", 10)

    assert harness.mission_complete is True
    assert harness.mission_phase == "FINISHED"
    after_finish = harness.publish_vision(
        line=line_info(),
        ball=pickup_ready_ball(),
        goal=score_ready_goal(),
        hurdle=None,
        finish=None,
    )
    assert after_finish
    assert all(
        payload["action"] not in {"STRAIGHT", "LEFT", "RIGHT"}
        for payload in after_finish
    )
    assert after_finish[-1]["action"] == "STOP"


@pytest.mark.parametrize("status", ["FAILED", "TIMEOUT"])
@pytest.mark.parametrize(
    (
        "action",
        "initial_phase",
        "expected_phase",
        "expected_sections",
    ),
    [
        ("PICKUP_NOW", "BALL_APPROACH", "BALL_APPROACH", 0),
        ("SHOT", "GOAL_APPROACH", "GOAL_APPROACH", 0),
        ("GO", "HURDLE_APPROACH", "HURDLE_APPROACH", 0),
        (
            "CROSS_FINISH",
            "WALK_TO_FINISH",
            "WALK_TO_FINISH",
            0,
        ),
    ],
)
def test_special_failure_and_timeout_follow_manager_policy(
    status,
    action,
    initial_phase,
    expected_phase,
    expected_sections,
):
    harness = MissionFlowHarness(
        phase=initial_phase,
        required_ball_sections=2,
    )
    assert harness.phase_manager.start_special_action(action, 10)
    complete_active(harness, action, 10, status)

    assert harness.mission_phase == expected_phase
    assert harness.ball_sections_processed == expected_sections
    assert harness.active_special_command_id is None


def test_general_gate_blocks_overlap_and_releases_after_terminal():
    harness = MissionFlowHarness()
    first = harness.publish_vision(line=line_info())
    assert len(first) == 1
    assert first[0]["action"] == "STRAIGHT"
    command_id = first[0]["command_id"]

    repeated = harness.publish_vision(line=line_info())
    assert repeated == []
    assert harness.general_motion_gate.locked is True

    harness.send_status("STRAIGHT", command_id, "RUNNING")
    harness.send_status("STRAIGHT", command_id, "SUCCEEDED")
    next_command = harness.publish_vision(line=line_info())
    assert len(next_command) == 1
    assert next_command[0]["action"] == "STRAIGHT"
    assert next_command[0]["command_id"] > command_id
    assert all(
        payload.get("error_code") != "REJECTED_BUSY"
        for payload in harness.publisher.messages
    )


def test_ball_stationary_turn_requires_fresh_ball_before_next_approach():
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    harness.BALL_POST_MOTION_DWELL_SEC = 0.0
    off_center = approaching_ball()
    off_center.update(
        {
            "depth_m": 0.70,
            "ground_distance_m": 0.70,
            "distance_m": 0.70,
            "bearing_deg": 20.0,
        }
    )

    approach = harness.publish_vision(ball=off_center)
    assert len(approach) == 1
    assert approach[0]["action"] == "STRAIGHT_2"
    command_id = approach[0]["command_id"]

    harness.send_status(
        approach[0]["action"],
        command_id,
        "RUNNING",
    )
    harness.send_status(
        approach[0]["action"],
        command_id,
        "SUCCEEDED",
    )

    before = len(harness.publisher.messages)
    MotionDecisionNode._publish_decision(harness)
    assert len(harness.publisher.messages) == before

    turn = harness.publish_vision(ball=off_center)
    assert len(turn) == 1
    assert turn[0]["action"] == "BALL_APPROACH_TURN_RIGHT_2"
    release_general(harness, turn[0])

    before = len(harness.publisher.messages)
    MotionDecisionNode._publish_decision(harness)
    assert len(harness.publisher.messages) == before

    centered = dict(off_center)
    centered["depth_m"] = 0.60
    centered["ground_distance_m"] = 0.60
    centered["distance_m"] = 0.60
    centered["bearing_deg"] = 0.0
    next_command = harness.publish_vision(ball=centered)
    assert len(next_command) == 1
    assert next_command[0]["action"] == "STRAIGHT_2"


def test_second_ball_uses_same_straight_distance_policy():
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    harness.phase_manager.pickups_completed = 1
    ball = approaching_ball()
    ball.update(
        {
            "depth_m": 0.70,
            "ground_distance_m": 0.70,
            "distance_m": 0.70,
            "bearing_deg": -20.0,
        }
    )

    command = harness.publish_vision(ball=ball)

    assert len(command) == 1
    assert command[0]["action"] == "STRAIGHT_2"


def test_failed_first_pickup_keeps_straight_distance_policy():
    harness = MissionFlowHarness(phase="BALL_APPROACH")
    pickup = publish_special(
        harness,
        "ball",
        pickup_ready_ball(),
        "PICKUP_NOW",
    )
    harness.send_status("PICKUP_NOW", pickup["command_id"], "RUNNING")
    harness.send_status("PICKUP_NOW", pickup["command_id"], "FAILED")
    assert harness.pickups_completed == 0

    ball = approaching_ball()
    ball.update(
        {
            "depth_m": 0.70,
            "ground_distance_m": 0.70,
            "distance_m": 0.70,
            "bearing_deg": 20.0,
        }
    )
    command = harness.publish_vision(ball=ball)

    assert len(command) == 1
    assert command[0]["action"] == "STRAIGHT_2"


def test_active_special_uses_temporary_lock_without_changing_manager_phase():
    harness = MissionFlowHarness(phase="HURDLE_APPROACH")
    command = publish_special(
        harness,
        "hurdle",
        go_ready_hurdle(),
        "GO",
    )
    locked = harness.publish_vision(hurdle=go_ready_hurdle())

    assert harness.phase_manager.current_phase == "HURDLE_APPROACH"
    assert locked[-1]["phase"] == "HURDLE_APPROACH_LOCK"
    assert locked[-1]["action"] == "WAIT"

    complete_active(harness, "GO", command["command_id"])
    assert harness.phase_manager.current_phase == "HURDLE_APPROACH"


def test_external_phase_override_accepts_only_valid_idle_phase():
    harness = MissionFlowHarness(phase="AUTO")
    harness.send_phase('{"phase": "goal_approach"}')
    assert harness.phase_manager.current_phase == "GOAL_APPROACH"

    harness.send_phase("")
    harness.send_phase("UNKNOWN")
    assert harness.phase_manager.current_phase == "GOAL_APPROACH"
