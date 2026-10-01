#!/usr/bin/env python3
"""Run existing line, ball, goal, and hurdle analyzers in one process."""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .ball_analyzer import BallAnalyzer
from .depth_frame_cache import DepthFrameCache
from .depth_frame_cache import SharedDepthSubscriber
from .goal_analyzer import GoalAnalyzer
from .hurdle_analyzer import HurdleAnalyzer
from .yolo_line_analyzer import YoloLineAnalyzer


def _create_analyzers() -> list[Node]:
    """Construct analyzers sharing one converted aligned-depth frame."""
    depth_cache = DepthFrameCache()
    analyzers = [
        YoloLineAnalyzer(depth_cache=depth_cache),
        BallAnalyzer(depth_cache=depth_cache),
        GoalAnalyzer(depth_cache=depth_cache),
        HurdleAnalyzer(depth_cache=depth_cache),
    ]
    depth_topic = str(analyzers[0].get_parameter("depth_topic").value)
    return [SharedDepthSubscriber(depth_topic, depth_cache), *analyzers]


def main(args: list[str] | None = None) -> None:
    """Compose all vision analyzers into one executable process."""
    rclpy.init(args=args)
    nodes: list[Node] = []
    # One worker receives depth while the remaining workers consume the newest
    # detections. Four workers avoid oversubscribing the six-core Jetson CPU.
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        nodes = _create_analyzers()
        for node in nodes:
            executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
