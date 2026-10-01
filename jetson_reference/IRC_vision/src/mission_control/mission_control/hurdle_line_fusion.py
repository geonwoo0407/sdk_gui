"""Fuse route-line geometry with a hurdle without entering line mode."""

from __future__ import annotations

import math
from typing import Any


def _number(data: dict[str, Any] | None, key: str) -> float | None:
    if data is None:
        return None
    value = data.get(key)
    if value is None or isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _points(raw_points: Any) -> list[tuple[float, float]]:
    if not isinstance(raw_points, list):
        return []
    parsed: list[tuple[float, float]] = []
    for raw in raw_points:
        if not isinstance(raw, list) or len(raw) != 2:
            continue
        try:
            x = float(raw[0])
            y = float(raw[1])
        except (TypeError, ValueError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            parsed.append((x, y))
    return parsed


def empty_hurdle_path_reference(reason: str) -> dict[str, Any]:
    """Return a stable empty fusion payload."""
    return {
        "path_reference_valid": False,
        "path_reference_reason": reason,
        "path_reference_source": "none",
        "path_reference_point_px": None,
        "path_offset_x_px": None,
        "path_offset_x_norm": None,
        "path_direction": "UNKNOWN",
        "path_bridge_segment_px": None,
        "path_support_points_px": [],
        "path_fit_quality": 0.0,
        "path_fit_point_count": 0,
        "line_mode_allowed": False,
    }


def build_hurdle_path_reference(
    hurdle_info: dict[str, Any] | None,
    line_info: dict[str, Any] | None,
) -> dict[str, Any]:
    """
    Estimate where the route line meets the hurdle's lower image edge.

    The result is hurdle-owned geometry.  It must never be interpreted as
    permission to select the line planner while the hurdle mission is active.
    """
    if hurdle_info is None or not bool(hurdle_info.get("detected", False)):
        return empty_hurdle_path_reference("hurdle_not_detected")
    if line_info is None or not bool(line_info.get("detected", False)):
        return empty_hurdle_path_reference("line_geometry_unavailable")

    raw_bbox = hurdle_info.get("bbox")
    if not isinstance(raw_bbox, list) or len(raw_bbox) != 4:
        return empty_hurdle_path_reference("invalid_hurdle_bbox")
    try:
        left, top, right, bottom = (float(value) for value in raw_bbox)
    except (TypeError, ValueError):
        return empty_hurdle_path_reference("invalid_hurdle_bbox")
    if not all(math.isfinite(value) for value in (left, top, right, bottom)):
        return empty_hurdle_path_reference("invalid_hurdle_bbox")
    if right <= left or bottom <= top:
        return empty_hurdle_path_reference("invalid_hurdle_bbox")

    image_width = _number(hurdle_info, "image_width")
    if image_width is None:
        image_width = _number(line_info, "image_width")
    image_height = _number(hurdle_info, "image_height")
    if image_height is None:
        image_height = _number(line_info, "image_height")
    if image_width is None or image_width <= 1.0:
        return empty_hurdle_path_reference("invalid_image_width")
    if image_height is None or image_height <= 1.0:
        return empty_hurdle_path_reference("invalid_image_height")

    route_points = _points(line_info.get("center_points_px"))
    rejected_points = _points(line_info.get("path_rejected_points_px"))
    outside = [
        point
        for point in route_points
        if not (
            left <= point[0] <= right
            and top <= point[1] <= bottom
        )
    ]
    used_rejected_fallback = False
    if len(outside) < 2:
        outside.extend(
            point
            for point in rejected_points
            if not (
                left <= point[0] <= right
                and top <= point[1] <= bottom
            )
        )
        used_rejected_fallback = True
        if len(outside) < 3:
            return empty_hurdle_path_reference(
                "insufficient_unoccluded_line_points"
            )

    target_y = min(max(bottom, 0.0), image_height - 1.0)
    local_points = sorted(
        outside,
        key=lambda point: abs(point[1] - target_y),
    )[:5]
    y_span = max(point[1] for point in local_points) - min(
        point[1] for point in local_points
    )
    if y_span < 8.0:
        return empty_hurdle_path_reference("line_points_have_no_y_span")

    mean_y = sum(point[1] for point in local_points) / len(local_points)
    mean_x = sum(point[0] for point in local_points) / len(local_points)
    denominator = sum((point[1] - mean_y) ** 2 for point in local_points)
    if denominator <= 1e-6:
        return empty_hurdle_path_reference("unstable_line_fit")
    slope = sum(
        (point[1] - mean_y) * (point[0] - mean_x)
        for point in local_points
    ) / denominator
    intercept = mean_x - slope * mean_y

    residual = math.sqrt(
        sum(
            (point[0] - (slope * point[1] + intercept)) ** 2
            for point in local_points
        )
        / len(local_points)
    )
    fit_quality = max(0.0, 1.0 - residual / max(image_width * 0.08, 1.0))
    minimum_fit_quality = 0.60 if used_rejected_fallback else 0.35
    if fit_quality < minimum_fit_quality:
        return empty_hurdle_path_reference("low_line_fit_quality")

    support_points = list(route_points)
    support_tolerance_px = max(30.0, image_width * 0.06)
    support_points.extend(
        point
        for point in rejected_points
        if abs(point[0] - (slope * point[1] + intercept))
        <= support_tolerance_px
    )
    support_points = sorted(set(support_points), key=lambda point: -point[1])

    target_x = min(max(slope * target_y + intercept, 0.0), image_width - 1.0)
    bridge_top_y = min(max(top, 0.0), image_height - 1.0)
    bridge_bottom_y = target_y
    bridge_top_x = min(
        max(slope * bridge_top_y + intercept, 0.0),
        image_width - 1.0,
    )
    bridge_bottom_x = target_x

    robot_center_x = _number(line_info, "robot_center_x_px")
    if robot_center_x is None:
        robot_center_x = image_width / 2.0
    offset_px = target_x - robot_center_x
    offset_norm = offset_px / max(image_width / 2.0, 1.0)
    if offset_norm < -0.02:
        direction = "LEFT"
    elif offset_norm > 0.02:
        direction = "RIGHT"
    else:
        direction = "CENTER"

    return {
        "path_reference_valid": True,
        "path_reference_reason": "line_hurdle_lower_edge_intersection",
        "path_reference_source": "fresh",
        "path_reference_point_px": [
            int(round(target_x)),
            int(round(target_y)),
        ],
        "path_offset_x_px": round(offset_px, 3),
        "path_offset_x_norm": round(offset_norm, 6),
        "path_direction": direction,
        "path_bridge_segment_px": [
            [int(round(bridge_bottom_x)), int(round(bridge_bottom_y))],
            [int(round(bridge_top_x)), int(round(bridge_top_y))],
        ],
        "path_support_points_px": [
            [int(round(point[0])), int(round(point[1]))]
            for point in support_points
        ],
        "path_fit_quality": round(fit_quality, 4),
        "path_fit_point_count": len(local_points),
        "line_mode_allowed": False,
    }
