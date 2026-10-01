"""Tests for the unified aligned-depth frame cache."""

import threading

import numpy as np

from step.depth_frame_cache import DepthFrameCache
from step.depth_frame_cache import DepthFrameConsumer


class _Consumer(DepthFrameConsumer):
    pass


def test_shared_consumers_read_the_same_depth_frame():
    """One cache update is immediately visible to every analyzer consumer."""
    cache = DepthFrameCache()
    first = _Consumer()
    second = _Consumer()
    first.depth_cache = cache
    second.depth_cache = cache
    image = np.full((2, 3), 900, dtype=np.uint16)

    cache.update(image, width=3, height=2, received_at=12.5)

    assert first.latest_depth_image is image
    assert second.latest_depth_image is image
    assert first.latest_depth_time == 12.5
    assert second.latest_image_width == 3
    assert second.latest_image_height == 2


def test_consumer_properties_still_support_lightweight_unit_helpers():
    """Analyzer tests may populate depth fields without constructing ROS."""
    consumer = _Consumer()
    image = np.ones((1, 1), dtype=np.uint16)

    consumer.latest_depth_image = image
    consumer.latest_depth_time = 3.0

    assert consumer.latest_depth_image is image
    assert consumer.latest_depth_time == 3.0


def test_cache_update_is_serialized_by_the_shared_lock():
    """Do not publish a partial frame while another user holds the lock."""
    cache = DepthFrameCache()
    image = np.full((2, 2), 1200, dtype=np.uint16)
    update_started = threading.Event()
    update_finished = threading.Event()

    def update_cache() -> None:
        update_started.set()
        cache.update(image, width=2, height=2, received_at=8.0)
        update_finished.set()

    with cache.lock:
        worker = threading.Thread(target=update_cache)
        worker.start()
        assert update_started.wait(timeout=1.0)
        assert not update_finished.is_set()

    worker.join(timeout=1.0)
    assert not worker.is_alive()
    assert update_finished.is_set()
    assert cache.image is image
    assert cache.received_at == 8.0
    assert cache.width == 2
    assert cache.height == 2


def test_nearest_selects_depth_by_ros_timestamp():
    """Match delayed detections to the closest buffered depth frame."""
    cache = DepthFrameCache(max_frames=6)
    first = np.full((2, 2), 400, dtype=np.uint16)
    second = np.full((2, 2), 900, dtype=np.uint16)
    cache.update(
        first,
        width=2,
        height=2,
        stamp_ns=1_000_000_000,
        received_at=10.0,
    )
    cache.update(
        second,
        width=2,
        height=2,
        stamp_ns=1_040_000_000,
        received_at=10.04,
    )

    frame, delta_sec = cache.nearest(1_035_000_000)

    assert frame is not None
    assert frame.image is second
    assert frame.stamp_ns == 1_040_000_000
    assert delta_sec == 0.005


def test_ring_discards_oldest_depth_frame():
    """Keep memory bounded while retaining recent timestamp history."""
    cache = DepthFrameCache(max_frames=2)
    for index in range(3):
        cache.update(
            np.full((1, 1), index, dtype=np.uint16),
            width=1,
            height=1,
            stamp_ns=index,
            received_at=float(index),
        )

    assert [frame.stamp_ns for frame in cache.frames] == [1, 2]
