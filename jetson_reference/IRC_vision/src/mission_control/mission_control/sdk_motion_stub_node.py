#!/usr/bin/env python3
"""
Temporary SDK motion executor.

This node does not control motors. It receives abstract navigation commands,
prints the SDK action that would be executed, and simulates completion for
special one-shot motions.
"""

import json
from typing import Any

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class SdkMotionStubNode(Node):
    """Placeholder for the future STEP SDK motion executor."""

    SPECIAL_ACTIONS = {
        'PICKUP_NOW',
        'SHOT',
        'GO',
        'CROSS_FINISH',
    }

    def __init__(self) -> None:
        """Initialize the SDK motion stub node."""
        super().__init__('sdk_motion_stub_node')

        self.declare_parameter(
            'command_topic',
            '/navigation/motion_command',
        )
        self.declare_parameter(
            'status_topic',
            '/motion/status',
        )
        self.declare_parameter(
            'completion_delay_sec',
            1.0,
        )

        command_topic = str(
            self.get_parameter('command_topic').value
        )
        status_topic = str(
            self.get_parameter('status_topic').value
        )

        self.status_publisher = self.create_publisher(
            String,
            status_topic,
            10,
        )

        self.create_subscription(
            String,
            command_topic,
            self.command_callback,
            10,
        )

        self.active_payload: dict[str, Any] | None = None
        self.completion_timer = None
        self.last_special_key: tuple[Any, Any, Any] | None = None

        self.get_logger().info(
            'SDK motion stub ready: '
            f'{command_topic} -> simulated SDK execution -> {status_topic}'
        )
        self.get_logger().warning(
            'SDK STUB MODE: no motor or Dynamixel command will be sent'
        )

    def publish_status(
        self,
        *,
        status: str,
        payload: dict[str, Any],
        reason: str,
    ) -> None:
        """Publish one JSON motion status message."""
        status_payload = {
            'status': status,
            'command_id': payload.get('command_id'),
            'event_id': payload.get('event_id'),
            'action': payload.get('action'),
            'dynamics_command': None,
            'reason': reason,
            'executor': 'sdk_stub',
        }

        message = String()
        message.data = json.dumps(
            status_payload,
            ensure_ascii=True,
            separators=(',', ':'),
        )
        self.status_publisher.publish(message)

        self.get_logger().info(
            'SDK stub status: '
            f'status={status}, '
            f"action={status_payload['action']}, "
            f"command_id={status_payload['command_id']}, "
            f"event_id={status_payload['event_id']}"
        )

    def command_callback(self, message: String) -> None:
        """Receive an abstract motion command."""
        try:
            payload = json.loads(message.data)

            if not isinstance(payload, dict):
                raise ValueError('JSON must be an object')

        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                f'Invalid navigation command: {type(exc).__name__}: {exc}'
            )
            return

        if payload.get('valid') is not True:
            return

        action = str(payload.get('action', '')).strip().upper()

        if not action or action == 'WAIT':
            return

        self.get_logger().info(
            'SDK_STUB received: '
            f'action={action}, '
            f"command_id={payload.get('command_id')}, "
            f"event_id={payload.get('event_id')}"
        )

        # Walking and alignment actions are only logged for now.
        # The real SDK node will replace this section with SDK calls.
        if action not in self.SPECIAL_ACTIONS:
            self.get_logger().info(
                f'SDK_STUB execute placeholder: {action}'
            )
            return

        special_key = (
            payload.get('event_id'),
            payload.get('command_id'),
            action,
        )

        # Prevent the same repeatedly published terminal command from
        # starting again after it has already completed.
        if special_key == self.last_special_key:
            return

        if self.active_payload is not None:
            self.get_logger().warning(
                'SDK_STUB busy: '
                f'ignored action={action}'
            )
            return

        self.active_payload = payload
        self.last_special_key = special_key

        self.publish_status(
            status='RUNNING',
            payload=payload,
            reason='sdk_stub_started',
        )

        delay = max(
            0.05,
            float(
                self.get_parameter(
                    'completion_delay_sec'
                ).value
            ),
        )

        self.completion_timer = self.create_timer(
            delay,
            self.complete_active_motion,
        )

    def complete_active_motion(self) -> None:
        """Simulate successful completion of the active SDK motion."""
        if self.completion_timer is not None:
            self.completion_timer.cancel()
            self.destroy_timer(self.completion_timer)
            self.completion_timer = None

        if self.active_payload is None:
            return

        payload = self.active_payload
        self.active_payload = None

        self.publish_status(
            status='SUCCEEDED',
            payload=payload,
            reason='sdk_stub_completed',
        )


def main(args=None) -> None:
    """Run the temporary SDK executor."""
    rclpy.init(args=args)
    node = SdkMotionStubNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
