"""Create safe Motion Executor player backends without hardware access."""

try:
    from .mock_motion_player import MockRobotMotionPlayer, MotionError
    from .motion_player_protocol import MotionPlayerProtocol
    from .sdk_motion_player_placeholder import SdkMotionPlayerPlaceholder
except ImportError:  # Allows direct unit-test imports.
    from mock_motion_player import MockRobotMotionPlayer, MotionError
    from motion_player_protocol import MotionPlayerProtocol
    from sdk_motion_player_placeholder import SdkMotionPlayerPlaceholder


SUPPORTED_PLAYER_BACKENDS = frozenset({"mock", "sdk"})


def create_motion_player(
    backend: str,
    *,
    mock_fail_after_updates: int = -1,
    mock_failure_code: str = "COMMUNICATION_ERROR",
) -> MotionPlayerProtocol:
    if backend == "mock":
        failure_error = MotionError.__members__.get(
            mock_failure_code, MotionError.INTERNAL_ERROR
        )
        return MockRobotMotionPlayer(
            fail_after_updates=mock_fail_after_updates,
            failure_error=failure_error,
            failure_message=(
                "test-only injected mock failure: "
                f"{failure_error.name}"
            ),
        )

    if backend == "sdk":
        # This is intentionally a safe placeholder, not an SDK import.
        return SdkMotionPlayerPlaceholder()

    raise ValueError(
        f"unsupported motion player backend: {backend!r}; "
        f"expected one of {sorted(SUPPORTED_PLAYER_BACKENDS)}"
    )
