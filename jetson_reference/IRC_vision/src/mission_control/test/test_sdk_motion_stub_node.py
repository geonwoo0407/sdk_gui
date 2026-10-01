#!/usr/bin/env python3
"""Tests for the temporary SDK motion stub."""

import json

import pytest

rclpy = pytest.importorskip("rclpy")

from std_msgs.msg import String  # noqa: E402

from mission_control.sdk_motion_stub_node import SdkMotionStubNode  # noqa: E402


@pytest.fixture
def node():
    """Create and destroy one SDK stub node."""
    if not rclpy.ok():
        rclpy.init()

    test_node = SdkMotionStubNode()
    yield test_node

    test_node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


def make_message(payload: dict) -> String:
    """Create one JSON String message."""
    message = String()
    message.data = json.dumps(payload)
    return message


def test_invalid_command_is_ignored(node):
    """Commands with valid=false must not start execution."""
    message = make_message(
        {
            "valid": False,
            "action": "SHOT",
            "command_id": 1,
            "event_id": 1,
            "sdk_motion_requested": True,
        }
    )

    node.command_callback(message)

    assert node.active_payload is None


def test_wait_command_is_ignored(node):
    """WAIT must not start SDK execution."""
    message = make_message(
        {
            "valid": True,
            "action": "WAIT",
            "command_id": 2,
            "event_id": 2,
            "sdk_motion_requested": False,
        }
    )

    node.command_callback(message)

    assert node.active_payload is None


def test_normal_walking_command_does_not_lock(node):
    """Continuous walking commands are logged but not locked."""
    message = make_message(
        {
            "valid": True,
            "action": "STRAIGHT",
            "command_id": 3,
            "event_id": 3,
            "sdk_motion_requested": False,
        }
    )

    node.command_callback(message)

    assert node.active_payload is None


def test_special_motion_starts(node):
    """A special motion must enter the active state."""
    message = make_message(
        {
            "valid": True,
            "action": "PICKUP_NOW",
            "command_id": 4,
            "event_id": 4,
            "sdk_motion_requested": True,
        }
    )

    node.command_callback(message)

    assert node.active_payload is not None
    assert node.active_payload["action"] == "PICKUP_NOW"


def test_special_motion_completes(node):
    """Completing a special motion must clear the active state."""
    message = make_message(
        {
            "valid": True,
            "action": "SHOT",
            "command_id": 5,
            "event_id": 5,
            "sdk_motion_requested": True,
        }
    )

    node.command_callback(message)
    assert node.active_payload is not None

    node.complete_active_motion()

    assert node.active_payload is None


def test_duplicate_special_motion_is_not_started_again(node):
    """The same completed special event must not restart."""
    payload = {
        "valid": True,
        "action": "GO",
        "command_id": 6,
        "event_id": 6,
        "sdk_motion_requested": True,
    }

    message = make_message(payload)

    node.command_callback(message)
    assert node.active_payload is not None

    node.complete_active_motion()
    assert node.active_payload is None

    node.command_callback(message)

    assert node.active_payload is None


def test_cross_finish_publishes_running_then_succeeded(node):
    """Treat CROSS_FINISH like the other acknowledged special motions."""
    statuses = []

    def capture_status(*, status, payload, reason):
        statuses.append((status, payload['action'], reason))

    node.publish_status = capture_status
    message = make_message(
        {
            "valid": True,
            "action": "CROSS_FINISH",
            "command_id": 7,
            "event_id": 7,
            "sdk_motion_requested": True,
        }
    )

    node.command_callback(message)
    assert node.active_payload is not None

    node.complete_active_motion()

    assert statuses == [
        ("RUNNING", "CROSS_FINISH", "sdk_stub_started"),
        ("SUCCEEDED", "CROSS_FINISH", "sdk_stub_completed"),
    ]

    node.command_callback(message)

    assert node.active_payload is None
