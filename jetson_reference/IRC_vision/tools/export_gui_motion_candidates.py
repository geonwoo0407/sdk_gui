#!/usr/bin/env python3
"""Export selected GUI sequences into a non-operational motion catalog."""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Iterable


DEFAULT_SEQUENCE_NAMES = (
    "공잡기",
    "공잡기 리그랩까지",
    "좌회전1",
    "우회전",
)
EXPECTED_MOTOR_IDS = {str(motor_id) for motor_id in range(23)}


class ExportError(ValueError):
    """Raised when candidate export input violates the loader contract."""


def _load_json(path: Path) -> dict[str, Any]:
    try:
        with path.open(encoding="utf-8") as stream:
            value = json.load(stream)
    except (OSError, json.JSONDecodeError) as exc:
        raise ExportError(f"cannot read JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ExportError(f"JSON root must be an object: {path}")
    return value


def _require_number(value: Any, label: str, *, positive: bool = False) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ExportError(f"{label} must be a number")
    if not math.isfinite(float(value)):
        raise ExportError(f"{label} must be finite")
    if (positive and value <= 0) or (not positive and value < 0):
        qualifier = "positive" if positive else "non-negative"
        raise ExportError(f"{label} must be {qualifier}")


def _require_finite_number(value: Any, label: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ExportError(f"{label} must be a number")
    if not math.isfinite(float(value)):
        raise ExportError(f"{label} must be finite")


def validate_loader_motion(motion: dict[str, Any]) -> None:
    """Validate the fields consumed by the C++ MotionLibrary loader."""
    name = motion.get("name")
    if not isinstance(name, str) or not name.strip():
        raise ExportError("motion name must be a non-empty string")

    _require_number(motion.get("max_seq_ms"), f"{name}.max_seq_ms")
    repeat_count = motion.get("repeat_count", 1)
    if isinstance(repeat_count, bool) or not isinstance(repeat_count, int):
        raise ExportError(f"{name}.repeat_count must be an integer")
    if repeat_count < 1:
        raise ExportError(f"{name}.repeat_count must be positive")
    _require_number(
        motion.get("playback_speed", 1.0),
        f"{name}.playback_speed",
        positive=True,
    )

    frames = motion.get("frames")
    if not isinstance(frames, list) or not frames:
        raise ExportError(f"{name}.frames must be a non-empty list")
    previous_end = 0
    for index, frame in enumerate(frames):
        label = f"{name}.frames[{index}]"
        if not isinstance(frame, dict):
            raise ExportError(f"{label} must be an object")
        if not isinstance(frame.get("name"), str) or not frame["name"]:
            raise ExportError(f"{label}.name must be a non-empty string")
        _require_number(frame.get("start_ms"), f"{label}.start_ms")
        _require_number(frame.get("time_ms"), f"{label}.time_ms", positive=True)
        if frame["start_ms"] < previous_end:
            raise ExportError(f"{label} overlaps the previous frame")
        previous_end = frame["start_ms"] + frame["time_ms"]

        angles = frame.get("angles")
        if not isinstance(angles, dict) or set(angles) != EXPECTED_MOTOR_IDS:
            raise ExportError(f"{label}.angles must contain exactly motor IDs 0-22")
        for motor_id, angle in angles.items():
            _require_finite_number(angle, f"{label}.angles[{motor_id}]")

        torques = frame.get("torques")
        if not isinstance(torques, dict) or set(torques) != EXPECTED_MOTOR_IDS:
            raise ExportError(f"{label}.torques must contain exactly motor IDs 0-22")
        if any(not isinstance(state, bool) for state in torques.values()):
            raise ExportError(f"{label}.torques values must be boolean")

    if motion["max_seq_ms"] < previous_end:
        raise ExportError(f"{name}.max_seq_ms is shorter than its frames")


def _export_sequence(sequence: dict[str, Any]) -> dict[str, Any]:
    frames = copy.deepcopy(sequence.get("frames"))
    if not isinstance(frames, list) or not frames:
        name = sequence.get("name", "<unnamed>")
        raise ExportError(f"GUI sequence has no frames: {name}")
    motion = {
        "name": sequence.get("name"),
        "max_seq_ms": sequence.get("max_seq_ms"),
        "repeat_count": sequence.get("repeat_count", 1),
        "playback_speed": sequence.get("playback_speed", 1.0),
        "repeatable": bool(sequence.get("repeatable", True)),
        "start_pose": frames[0].get("name", ""),
        "end_pose": frames[-1].get("name", ""),
        "completion": copy.deepcopy(sequence.get("completion", {})),
        "frames": frames,
    }
    validate_loader_motion(motion)
    return motion


def build_candidate_catalog(
    gui_state: dict[str, Any],
    official_catalog: dict[str, Any],
    sequence_names: Iterable[str] = DEFAULT_SEQUENCE_NAMES,
) -> dict[str, Any]:
    """Return the official catalog plus selected, validated GUI sequences."""
    official_motions = official_catalog.get("motions")
    if not isinstance(official_motions, list):
        raise ExportError("official catalog must contain a motions list")
    for motion in official_motions:
        validate_loader_motion(motion)

    names = [motion.get("name") for motion in official_motions]
    if len(names) != len(set(names)):
        raise ExportError("official catalog contains duplicate motion names")

    saved_sequences = gui_state.get("saved_sequences")
    if not isinstance(saved_sequences, list):
        raise ExportError("GUI state must contain a saved_sequences list")
    sequence_by_name: dict[str, dict[str, Any]] = {}
    for sequence in saved_sequences:
        if not isinstance(sequence, dict) or not isinstance(sequence.get("name"), str):
            raise ExportError("GUI saved sequence has an invalid name")
        name = sequence["name"]
        if name in sequence_by_name:
            raise ExportError(f"duplicate GUI sequence name: {name}")
        sequence_by_name[name] = sequence

    requested_names = list(sequence_names)
    if len(requested_names) != len(set(requested_names)):
        raise ExportError("requested sequence names contain duplicates")

    result_motions = copy.deepcopy(official_motions)
    occupied_names = set(names)
    for name in requested_names:
        if name not in sequence_by_name:
            raise ExportError(f"requested GUI sequence not found: {name}")
        if name in occupied_names:
            raise ExportError(f"motion name collision: {name}")
        result_motions.append(_export_sequence(sequence_by_name[name]))
        occupied_names.add(name)

    result = {
        "version": official_catalog.get("version", 1),
        "motions": result_motions,
    }
    validate_catalog(result)
    return result


def validate_catalog(catalog: dict[str, Any]) -> None:
    motions = catalog.get("motions")
    if not isinstance(motions, list):
        raise ExportError("catalog must contain a motions list")
    names: set[str] = set()
    for motion in motions:
        if not isinstance(motion, dict):
            raise ExportError("each motion must be an object")
        validate_loader_motion(motion)
        if motion["name"] in names:
            raise ExportError(f"duplicate motion name: {motion['name']}")
        names.add(motion["name"])


def export_candidates(
    gui_state_path: Path,
    official_catalog_path: Path,
    output_path: Path,
    sequence_names: Iterable[str] = DEFAULT_SEQUENCE_NAMES,
) -> dict[str, Any]:
    """Build and atomically write a candidate catalog without overwriting inputs."""
    resolved_output = output_path.resolve()
    input_paths = {gui_state_path.resolve(), official_catalog_path.resolve()}
    if resolved_output in input_paths:
        raise ExportError("output path must not overwrite an input file")

    catalog = build_candidate_catalog(
        _load_json(gui_state_path),
        _load_json(official_catalog_path),
        sequence_names,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            "w",
            encoding="utf-8",
            dir=output_path.parent,
            prefix=f".{output_path.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary_name = stream.name
            json.dump(catalog, stream, ensure_ascii=False, indent=4)
            stream.write("\n")
        os.replace(temporary_name, output_path)
    finally:
        if temporary_name is not None:
            Path(temporary_name).unlink(missing_ok=True)
    return catalog


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("gui_state", type=Path, help="sdk_gui_state.json path")
    parser.add_argument("official_catalog", type=Path, help="robot_motions.json path")
    parser.add_argument("output", type=Path, help="candidate catalog output path")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    try:
        catalog = export_candidates(
            args.gui_state,
            args.official_catalog,
            args.output,
        )
    except ExportError as exc:
        raise SystemExit(f"export failed: {exc}") from exc
    print(f"exported {len(catalog['motions'])} motions to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
