import unittest

import launch
import launch.actions
import launch.events
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from launch_ros.actions import Node


@launch_testing.parametrize(
    "backend_type", ["unknown", "robot_motion_player"]
)
def generate_test_description(backend_type):
    executor = Node(
        package="irc_step_motion_executor",
        executable="sdk_motion_executor",
        name=f"sdk_executor_reject_{backend_type}",
        output="screen",
        parameters=[{"backend_type": backend_type}],
    )
    return (
        launch.LaunchDescription([
            executor,
            launch_testing.util.KeepAliveProc(),
            launch_testing.actions.ReadyToTest(),
            launch.actions.TimerAction(
                period=2.0,
                actions=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown(
                        reason="backend rejection test timeout guard"
                    )
                )],
            ),
        ]),
        {
            "backend_type": backend_type,
            "executor": executor,
        },
    )


class TestRejectedBackendSelection(unittest.TestCase):

    def test_node_reports_error_and_exits(
        self, backend_type, executor, proc_info, proc_output
    ):
        expected = (
            "UNSUPPORTED_BACKEND_TYPE"
            if backend_type == "unknown"
            else "ROBOT_HARDWARE_NOT_ENABLED"
        )
        proc_output.assertWaitFor(
            expected, process=executor, timeout=10
        )
        proc_info.assertWaitForShutdown(process=executor, timeout=10)


@launch_testing.post_shutdown_test()
class TestRejectedBackendExitCode(unittest.TestCase):

    def test_exit_code_is_failure(self, executor, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[1], process=executor
        )
