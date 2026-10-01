import builtins
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "mission_control"))

from mock_motion_player import (  # noqa: E402
    MockRobotMotionPlayer,
    MotionError,
    StartResult,
)
from motion_player_factory import create_motion_player  # noqa: E402
from motion_player_protocol import MotionPlayerProtocol  # noqa: E402
from sdk_motion_player_placeholder import (  # noqa: E402
    SdkMotionPlayerPlaceholder,
)


def test_mock_backend_returns_mock_player():
    player = create_motion_player("mock")
    assert isinstance(player, MockRobotMotionPlayer)
    assert isinstance(player, MotionPlayerProtocol)


def test_mock_failure_configuration_is_forwarded():
    player = create_motion_player(
        "mock",
        mock_fail_after_updates=1,
        mock_failure_code="POSITION_TIMEOUT",
    )
    assert player.start("forward") is StartResult.ACCEPTED
    player.update()
    assert player.result() is MotionError.POSITION_TIMEOUT
    assert player.lastError() == (
        "test-only injected mock failure: POSITION_TIMEOUT"
    )


def test_sdk_backend_returns_safe_placeholder():
    player = create_motion_player("sdk")
    assert isinstance(player, SdkMotionPlayerPlaceholder)
    assert isinstance(player, MotionPlayerProtocol)
    assert player.hardwareReady() is False
    assert player.start("forward") is StartResult.HARDWARE_NOT_READY
    assert player.lastError() == "SDK backend is not connected yet"


def test_sdk_placeholder_does_not_access_files_or_hardware(monkeypatch):
    def fail_open(*args, **kwargs):
        del args, kwargs
        raise AssertionError("placeholder must not open files or devices")

    monkeypatch.setattr(builtins, "open", fail_open)
    player = create_motion_player(
        "sdk",
        mock_fail_after_updates=0,
        mock_failure_code="COMMUNICATION_ERROR",
    )
    player.update()
    player.cancel()
    assert player.hardwareReady() is False


def test_unsupported_backend_raises_value_error():
    with pytest.raises(ValueError, match="unsupported motion player backend"):
        create_motion_player("hardware")
