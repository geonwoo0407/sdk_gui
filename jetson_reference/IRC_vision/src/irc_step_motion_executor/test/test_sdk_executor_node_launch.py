import json
import time
import unittest

import launch
import launch.actions
import launch.events
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from launch_ros.actions import Node
import rclpy
from std_msgs.msg import String


def generate_test_description():
    executor = Node(
        package="irc_step_motion_executor",
        executable="sdk_motion_executor",
        name="sdk_motion_executor_launch_test",
        output="screen",
        parameters=[{
            "poll_period_ms": 20,
            "running_polls": 5,
            "settling_polls": 1,
            "force_start_failure": False,
            "force_backend_failure": False,
            "heartbeat_period_ms": 20,
        }],
    )
    return (
        launch.LaunchDescription([
            executor,
            launch_testing.actions.ReadyToTest(),
            launch_testing.util.KeepAliveProc(),
            launch.actions.TimerAction(
                period=5.0,
                actions=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown(
                        reason="simulated executor test timeout guard"
                    )
                )],
            ),
        ]),
        {"executor": executor},
    )


class TestSdkExecutorTopics(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("sdk_executor_contract_test")
        cls.request_publisher = cls.node.create_publisher(
            String, "/motion/executor/request", 10
        )
        cls.cancel_publisher = cls.node.create_publisher(
            String, "/motion/executor/cancel", 10
        )
        cls.ball_info_publisher = cls.node.create_publisher(
            String, "/vision/ball_info", 10
        )
        cls.statuses = []
        cls.heartbeats = []
        cls.subscription = cls.node.create_subscription(
            String,
            "/motion/executor/status",
            lambda message: cls.statuses.append(json.loads(message.data)),
            10,
        )
        cls.heartbeat_subscription = cls.node.create_subscription(
            String,
            "/motion/executor/heartbeat",
            lambda message: cls.heartbeats.append(json.loads(message.data)),
            10,
        )
        cls._wait_for_graph()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _wait_for_graph(cls):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if (
                cls.request_publisher.get_subscription_count() > 0
                and cls.cancel_publisher.get_subscription_count() > 0
                and cls.ball_info_publisher.get_subscription_count() > 0
            ):
                return
        raise AssertionError("executor topic discovery timed out")

    def _publish(self, publisher, payload):
        message = String()
        message.data = json.dumps(payload)
        publisher.publish(message)

    def _request(
        self,
        request_id,
        motion_id,
        action,
        command_id=17,
        event_id=29,
        timeout_ms=1000,
    ):
        self._publish(self.request_publisher, {
            "action": action,
            "command_id": command_id,
            "event_id": event_id,
            "request_id": request_id,
            "motion_id": motion_id,
            "timeout_ms": timeout_ms,
        })

    def _cancel(self, request_id):
        self._publish(self.cancel_publisher, {"request_id": request_id})

    def _wait_for(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            matching = [status for status in self.statuses if predicate(status)]
            if matching:
                return matching[-1]
        raise AssertionError(
            f"status timeout; received={self.statuses!r}"
        )

    def test_simulated_topic_contract(self):
        self._request(101, "forward", "STRAIGHT", 1001, 2001)
        succeeded = self._wait_for(
            lambda value:
            value["request_id"] == 101
            and value["status"] == "SUCCEEDED"
        )
        forward_statuses = [
            value for value in self.statuses
            if value["request_id"] == 101
        ]
        self.assertEqual(succeeded["action"], "STRAIGHT")
        self.assertEqual(succeeded["command_id"], 1001)
        self.assertEqual(succeeded["event_id"], 2001)
        self.assertTrue(any(
            value["status"] == "RUNNING"
            for value in forward_statuses
        ))
        self.assertTrue(any(
            value["status"] == "RUNNING"
            and "settling" in value["message"]
            for value in forward_statuses
        ))

        self._request(
            102, "pickup", "PICKUP_NOW", None, None
        )
        null_terminal = self._wait_for(
            lambda value:
            value["request_id"] == 102
            and value["status"] == "SUCCEEDED"
        )
        self.assertIsNone(null_terminal["command_id"])
        self.assertIsNone(null_terminal["event_id"])
        self.assertEqual(null_terminal["motion_id"], "pickup")

        self._request(103, "turn_left", "TURN_LEFT")
        unsupported = self._wait_for(
            lambda value:
            value["request_id"] == 103
            and value["error_code"] == "INVALID_MOTION"
        )
        self.assertEqual(unsupported["status"], "REJECTED")

        self._request(104, "forward", "STRAIGHT")
        self._request(105, "hurdle", "GO")
        busy = self._wait_for(
            lambda value:
            value["request_id"] == 105
            and value["error_code"] == "BUSY"
        )
        self.assertEqual(busy["status"], "REJECTED")
        self._wait_for(
            lambda value:
            value["request_id"] == 104
            and value["status"] == "SUCCEEDED"
        )

        self._request(106, "forward", "STRAIGHT")
        self._wait_for(
            lambda value:
            value["request_id"] == 106
            and value["status"] == "RUNNING"
        )
        self._cancel(999)
        stale = self._wait_for(
            lambda value:
            value["request_id"] == 106
            and value["error_code"] == "STALE_REQUEST"
        )
        self.assertEqual(stale["status"], "REJECTED")
        self._wait_for(
            lambda value:
            value["request_id"] == 106
            and value["status"] == "SUCCEEDED"
        )
        self.assertFalse(any(
            value["request_id"] == 106
            and value["status"] == "CANCELLED"
            for value in self.statuses
        ))

        self._request(107, "forward", "STRAIGHT")
        self._wait_for(
            lambda value:
            value["request_id"] == 107
            and value["status"] == "RUNNING"
        )
        self._cancel(107)
        cancelled = self._wait_for(
            lambda value:
            value["request_id"] == 107
            and value["status"] == "CANCELLED"
        )
        self.assertEqual(cancelled["action"], "STRAIGHT")

        self._request(
            108, "forward", "STRAIGHT", 1008, 2008, timeout_ms=1
        )
        timed_out = self._wait_for(
            lambda value:
            value["request_id"] == 108
            and value["error_code"] == "TIMEOUT"
        )
        self.assertEqual(timed_out["status"], "FAILED")
        self.assertEqual(timed_out["command_id"], 1008)
        self.assertEqual(timed_out["event_id"], 2008)

    def test_ball_bottom_trigger_latches_override_until_pickup_starts(self):
        self._request(201, "forward", "STRAIGHT", 1201, 2201)
        self._wait_for(
            lambda value:
            value["request_id"] == 201
            and value["status"] == "RUNNING"
        )
        self._publish(self.ball_info_publisher, {
            "detected": True,
            "head_down_requested": True,
        })
        active = self._wait_for_heartbeat(
            lambda value: value.get("ball_head_override_active") is True
        )
        self.assertEqual(active["ball_head_override_deg"], -60.0)

        self._wait_for(
            lambda value:
            value["request_id"] == 201
            and value["status"] == "SUCCEEDED"
        )
        held_after_forward = self._wait_for_heartbeat(
            lambda value:
            value["sequence"] > active["sequence"]
            and value.get("ball_head_override_active") is True
        )

        self._request(202, "hurdle", "GO", 1202, 2202)
        self._wait_for(
            lambda value:
            value["request_id"] == 202
            and value["status"] == "RUNNING"
        )
        self._wait_for_heartbeat(
            lambda value:
            value["sequence"] > held_after_forward["sequence"]
            and value.get("ball_head_override_active") is True
        )
        self._wait_for(
            lambda value:
            value["request_id"] == 202
            and value["status"] == "SUCCEEDED"
        )

        self._request(203, "pickup", "PICKUP_NOW", 1203, 2203)
        self._wait_for(
            lambda value:
            value["request_id"] == 203
            and value["status"] == "RUNNING"
        )
        self._wait_for_heartbeat(
            lambda value:
            value["sequence"] > held_after_forward["sequence"]
            and value.get("ball_head_override_active") is False
        )
        self._wait_for(
            lambda value:
            value["request_id"] == 203
            and value["status"] == "SUCCEEDED"
        )

    def test_simulated_executor_publishes_repeated_liveness_heartbeats(self):
        first = self._wait_for_heartbeat_count(2)
        self.assertEqual(first["backend_type"], "simulated")
        self.assertIsInstance(first["active"], bool)
        self.assertIs(first["auto_ready"], True)
        self.assertIsInstance(first["ball_head_override_active"], bool)
        sequences = [heartbeat["sequence"] for heartbeat in self.heartbeats]
        self.assertEqual(sequences, sorted(sequences))
        self.assertEqual(len(sequences), len(set(sequences)))

    def _wait_for_heartbeat_count(self, count, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if len(self.heartbeats) >= count:
                return self.heartbeats[-1]
        raise AssertionError(
            f"heartbeat timeout; received={self.heartbeats!r}"
        )

    def _wait_for_heartbeat(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            matching = [value for value in self.heartbeats if predicate(value)]
            if matching:
                return matching[-1]
        raise AssertionError(
            f"heartbeat predicate timeout; received={self.heartbeats!r}"
        )


@launch_testing.post_shutdown_test()
class TestSdkExecutorShutdown(unittest.TestCase):

    def test_process_exited_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
