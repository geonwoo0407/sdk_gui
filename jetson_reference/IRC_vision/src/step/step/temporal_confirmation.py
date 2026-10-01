#!/usr/bin/env python3
"""Reusable temporal and spatial confirmation for vision detections."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math


def depth_is_within_range(
    depth_valid: bool,
    depth_m: float | None,
    maximum_depth_m: float,
) -> bool:
    """Return whether a fresh object depth is inside an inclusive gate."""
    if not depth_valid or depth_m is None or isinstance(depth_m, bool):
        return False
    try:
        depth = float(depth_m)
        maximum = float(maximum_depth_m)
    except (TypeError, ValueError):
        return False
    return bool(
        math.isfinite(depth)
        and math.isfinite(maximum)
        and depth > 0.0
        and maximum > 0.0
        and depth <= maximum
    )


@dataclass(frozen=True)
class ConfirmationResult:
    """State returned after processing one video frame."""

    raw_detected: bool
    confirmed: bool
    matched_previous: bool
    hit_count: int
    required_hits: int
    window_size: int
    missed_frames: int

    def as_dict(self, prefix: str = "confirmation") -> dict[str, object]:
        """Return JSON-friendly diagnostic fields."""
        raw_key = (
            "raw_detected"
            if prefix == "confirmation"
            else f"{prefix}_raw_detected"
        )
        return {
            raw_key: self.raw_detected,
            f"{prefix}_confirmed": self.confirmed,
            f"{prefix}_matched_previous": self.matched_previous,
            f"{prefix}_hits": self.hit_count,
            f"{prefix}_required_hits": self.required_hits,
            f"{prefix}_window_size": self.window_size,
            f"{prefix}_missed_frames": self.missed_frames,
        }


class TemporalConfirmationFilter:
    """Confirm repeated, spatially consistent detections in a frame window.

    The filter can also confirm a plain boolean condition by omitting ``bbox``.
    A confirmed track tolerates a small number of misses internally, but the
    returned ``confirmed`` flag is false on a frame with no current detection.
    This prevents a motion planner from acting on stale image geometry while
    allowing a reacquired target to recover without starting from zero.
    """

    def __init__(
        self,
        *,
        window_size: int,
        required_hits: int,
        max_missed_frames: int = 2,
        max_center_shift_norm: float = 0.18,
        min_area_ratio: float = 0.40,
        spatial_matching: bool = True,
    ) -> None:
        self.window_size = max(1, int(window_size))
        self.required_hits = max(
            1,
            min(self.window_size, int(required_hits)),
        )
        self.max_missed_frames = max(0, int(max_missed_frames))
        self.max_center_shift_norm = max(0.0, float(max_center_shift_norm))
        self.min_area_ratio = min(1.0, max(0.0, float(min_area_ratio)))
        self.spatial_matching = bool(spatial_matching)

        self._hits: deque[bool] = deque(maxlen=self.window_size)
        self._last_bbox: tuple[float, float, float, float] | None = None
        self._missed_frames = 0
        self._track_confirmed = False

    def reset(self) -> None:
        """Forget the current candidate and all temporal history."""
        self._hits.clear()
        self._last_bbox = None
        self._missed_frames = 0
        self._track_confirmed = False

    def update(
        self,
        detected: bool,
        *,
        bbox: list[int] | tuple[int, int, int, int] | None = None,
        image_width: int | None = None,
        image_height: int | None = None,
    ) -> ConfirmationResult:
        """Process one frame and return the current confirmation state."""
        current_bbox = self._normalize_bbox(bbox)
        matched = bool(detected)
        if detected and self.spatial_matching:
            matched = current_bbox is not None and self._matches_previous(
                current_bbox,
                image_width,
                image_height,
            )
            if not matched:
                self.reset()
                matched = current_bbox is not None

        if detected and matched:
            self._hits.append(True)
            self._missed_frames = 0
            if current_bbox is not None:
                self._last_bbox = current_bbox
            if sum(self._hits) >= self.required_hits:
                self._track_confirmed = True
        else:
            self._hits.append(False)
            self._missed_frames += 1
            if self._missed_frames > self.max_missed_frames:
                self.reset()

        confirmed_now = bool(
            detected and matched and self._track_confirmed
        )
        return ConfirmationResult(
            raw_detected=bool(detected),
            confirmed=confirmed_now,
            matched_previous=matched,
            hit_count=sum(self._hits),
            required_hits=self.required_hits,
            window_size=self.window_size,
            missed_frames=self._missed_frames,
        )

    @staticmethod
    def _normalize_bbox(
        bbox: list[int] | tuple[int, int, int, int] | None,
    ) -> tuple[float, float, float, float] | None:
        if bbox is None or len(bbox) != 4:
            return None
        left, top, right, bottom = (float(value) for value in bbox)
        if not all(math.isfinite(value) for value in (left, top, right, bottom)):
            return None
        if right <= left or bottom <= top:
            return None
        return left, top, right, bottom

    def _matches_previous(
        self,
        bbox: tuple[float, float, float, float],
        image_width: int | None,
        image_height: int | None,
    ) -> bool:
        if self._last_bbox is None:
            return True
        left, top, right, bottom = bbox
        old_left, old_top, old_right, old_bottom = self._last_bbox
        center_x = (left + right) / 2.0
        center_y = (top + bottom) / 2.0
        old_center_x = (old_left + old_right) / 2.0
        old_center_y = (old_top + old_bottom) / 2.0
        diagonal = math.hypot(
            max(1, int(image_width or 0)),
            max(1, int(image_height or 0)),
        )
        center_shift = math.hypot(
            center_x - old_center_x,
            center_y - old_center_y,
        ) / diagonal
        area = (right - left) * (bottom - top)
        old_area = (old_right - old_left) * (old_bottom - old_top)
        area_ratio = min(area, old_area) / max(area, old_area, 1.0)
        return (
            center_shift <= self.max_center_shift_norm
            and area_ratio >= self.min_area_ratio
        )
