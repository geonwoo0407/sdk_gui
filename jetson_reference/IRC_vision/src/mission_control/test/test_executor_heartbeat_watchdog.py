"""Tests for the ROS-independent executor heartbeat watchdog."""

from mission_control.executor_heartbeat_watchdog import (
    EXECUTOR_HEARTBEAT_TIMEOUT,
)
from mission_control.executor_heartbeat_watchdog import (
    ExecutorHeartbeatWatchdog,
)
from mission_control.executor_heartbeat_watchdog import HeartbeatStartupDelay


def watchdog():
    return ExecutorHeartbeatWatchdog(
        started_at=10.0,
        startup_grace_sec=5.0,
        timeout_sec=2.0,
    )


def test_startup_grace_does_not_report_missing_executor_early():
    monitor = watchdog()
    assert monitor.check(10.0) is None
    assert monitor.check(15.0) is None


def test_never_seen_executor_reports_noncritical_startup_delay_once():
    monitor = watchdog()
    timeout = monitor.check(15.001)
    assert isinstance(timeout, HeartbeatStartupDelay)
    assert monitor.check(20.0) is None


def test_periodic_heartbeat_prevents_timeout():
    monitor = watchdog()
    monitor.observe(sequence=0, observed_at=12.0)
    assert monitor.check(13.9) is None
    monitor.observe(sequence=1, observed_at=13.9)
    assert monitor.check(15.8) is None
    assert monitor.executor_seen
    assert monitor.last_sequence == 1


def test_seen_executor_times_out_after_heartbeat_loss():
    monitor = watchdog()
    monitor.observe(sequence=4, observed_at=12.0)
    assert monitor.check(14.0) is None
    timeout = monitor.check(14.001)
    assert timeout is not None
    assert timeout.executor_seen is True


def test_late_first_heartbeat_enables_runtime_timeout_detection():
    monitor = watchdog()
    assert isinstance(monitor.check(16.0), HeartbeatStartupDelay)
    monitor.observe(sequence=0, observed_at=17.0)
    timeout = monitor.check(19.001)
    assert timeout is not None
    assert timeout.error_code == EXECUTOR_HEARTBEAT_TIMEOUT
    assert timeout.executor_seen is True
