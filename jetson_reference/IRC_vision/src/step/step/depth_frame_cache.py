"""Share the newest aligned-depth frame between vision analyzers."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading
import time

from cv_bridge import CvBridge
import numpy as np
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image


def image_stamp_ns(message: Image) -> int:
    """Convert one ROS image header timestamp to integer nanoseconds."""
    return (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )


@dataclass(frozen=True)
class DepthFrame:
    """One immutable depth image and the metadata needed for synchronization."""

    image: np.ndarray
    stamp_ns: int | None
    received_at: float
    width: int
    height: int


class DepthFrameCache:
    """Thread-safe timestamp ring plus latest-frame compatibility fields."""

    def __init__(self, max_frames: int = 30) -> None:
        self.lock = threading.RLock()
        self.frames: deque[DepthFrame] = deque(
            maxlen=max(2, int(max_frames))
        )
        self.image: np.ndarray | None = None
        self.stamp_ns: int | None = None
        self.received_at: float | None = None
        self.width: int | None = None
        self.height: int | None = None

    def update(
        self,
        image: np.ndarray,
        *,
        width: int,
        height: int,
        stamp_ns: int | None = None,
        received_at: float | None = None,
    ) -> None:
        """Atomically append a frame and update compatibility fields."""
        received_at = (
            time.monotonic() if received_at is None else received_at
        )
        frame = DepthFrame(
            image=image,
            stamp_ns=stamp_ns,
            received_at=received_at,
            width=width,
            height=height,
        )
        with self.lock:
            self.frames.append(frame)
            self.image = image
            self.stamp_ns = stamp_ns
            self.received_at = received_at
            self.width = width
            self.height = height

    def nearest(
        self,
        stamp_ns: int,
    ) -> tuple[DepthFrame | None, float | None]:
        """Return the frame closest to ``stamp_ns`` and its delta in seconds."""
        with self.lock:
            candidates = [
                frame for frame in self.frames if frame.stamp_ns is not None
            ]
            if not candidates:
                return None, None
            frame = min(
                candidates,
                key=lambda item: abs(int(item.stamp_ns) - stamp_ns),
            )
        delta_sec = abs(int(frame.stamp_ns) - stamp_ns) / 1_000_000_000.0
        return frame, delta_sec

    def latest(self) -> DepthFrame | None:
        """Return the newest complete frame snapshot."""
        with self.lock:
            if self.frames:
                return self.frames[-1]
            if self.image is None or self.received_at is None:
                return None
            return DepthFrame(
                image=self.image,
                stamp_ns=self.stamp_ns,
                received_at=self.received_at,
                width=int(self.width or self.image.shape[1]),
                height=int(self.height or self.image.shape[0]),
            )


class DepthFrameConsumer:
    """Compatibility properties for analyzers backed by a depth cache."""

    depth_cache: DepthFrameCache

    def _get_depth_cache(self) -> DepthFrameCache:
        """Lazily support analyzer helpers constructed without ROS setup."""
        cache = getattr(self, "depth_cache", None)
        if cache is None:
            cache = DepthFrameCache()
            self.depth_cache = cache
        return cache

    @property
    def latest_depth_image(self) -> np.ndarray | None:
        cache = self._get_depth_cache()
        with cache.lock:
            return cache.image

    @latest_depth_image.setter
    def latest_depth_image(self, value: np.ndarray | None) -> None:
        cache = self._get_depth_cache()
        with cache.lock:
            cache.image = value

    @property
    def latest_depth_time(self) -> float | None:
        cache = self._get_depth_cache()
        with cache.lock:
            return cache.received_at

    @latest_depth_time.setter
    def latest_depth_time(self, value: float | None) -> None:
        cache = self._get_depth_cache()
        with cache.lock:
            cache.received_at = value

    @property
    def latest_image_width(self) -> int | None:
        cache = self._get_depth_cache()
        with cache.lock:
            return cache.width

    @latest_image_width.setter
    def latest_image_width(self, value: int | None) -> None:
        cache = self._get_depth_cache()
        with cache.lock:
            cache.width = value

    @property
    def latest_image_height(self) -> int | None:
        cache = self._get_depth_cache()
        with cache.lock:
            return cache.height

    @latest_image_height.setter
    def latest_image_height(self, value: int | None) -> None:
        cache = self._get_depth_cache()
        with cache.lock:
            cache.height = value

    def _store_depth_message(
        self,
        message: Image,
        bridge: CvBridge,
    ) -> None:
        depth = bridge.imgmsg_to_cv2(
            message,
            desired_encoding="passthrough",
        )
        self.depth_cache.update(
            np.asarray(depth),
            width=int(message.width),
            height=int(message.height),
            stamp_ns=image_stamp_ns(message),
        )


class SharedDepthSubscriber(Node):
    """Convert aligned depth once for all analyzers in the unified process."""

    def __init__(self, topic: str, cache: DepthFrameCache) -> None:
        super().__init__("vision_depth_cache")
        self._cache = cache
        self._bridge = CvBridge()
        self._callback_group = MutuallyExclusiveCallbackGroup()
        latest_depth_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Image,
            topic,
            self._depth_callback,
            latest_depth_qos,
            callback_group=self._callback_group,
        )
        self.get_logger().info(f"Sharing aligned depth: {topic}")

    def _depth_callback(self, message: Image) -> None:
        try:
            depth = self._bridge.imgmsg_to_cv2(
                message,
                desired_encoding="passthrough",
            )
            self._cache.update(
                np.asarray(depth),
                width=int(message.width),
                height=int(message.height),
                stamp_ns=image_stamp_ns(message),
            )
        except Exception as exc:
            self.get_logger().warning(f"Could not read depth image: {exc}")
