#!/usr/bin/env python3
"""Validate an SDK motion JSON catalog without accessing robot hardware."""

import argparse
import json
from pathlib import Path
import sys

import yaml


EXPECTED_MOTOR_IDS = set(range(23))


def _motor_ids(values, location: str, errors: list[str]) -> None:
    if not isinstance(values, dict):
        errors.append(f"{location}: must be an object")
        return
    parsed = set()
    for raw_id in values:
        try:
            motor_id = int(raw_id)
        except (TypeError, ValueError):
            errors.append(f"{location}: invalid motor ID {raw_id!r}")
            continue
        if str(motor_id) != str(raw_id):
            errors.append(f"{location}: invalid motor ID {raw_id!r}")
            continue
        parsed.add(motor_id)
    missing = sorted(EXPECTED_MOTOR_IDS - parsed)
    extra = sorted(parsed - EXPECTED_MOTOR_IDS)
    if missing or extra:
        errors.append(
            f"{location}: motor IDs differ; missing={missing}, extra={extra}"
        )


def validate_catalog(catalog: object, aliases: object) -> list[str]:
    """Return all catalog and alias consistency errors."""
    errors = []
    if not isinstance(catalog, dict):
        return ["catalog root must be an object"]
    motions = catalog.get("motions")
    if not isinstance(motions, list):
        return ["catalog must contain a top-level motions array"]

    names = set()
    for motion_index, motion in enumerate(motions):
        if not isinstance(motion, dict):
            errors.append(f"motion[{motion_index}]: must be an object")
            continue
        name = motion.get("name")
        location = f"motion[{motion_index}]({name!r})"
        if not isinstance(name, str) or not name:
            errors.append(f"{location}: name must be a non-empty string")
        elif name in names:
            errors.append(f"{location}: duplicate motion name")
        else:
            names.add(name)
        for field in ("completion", "start_pose", "end_pose"):
            if field not in motion:
                errors.append(f"{location}: missing {field}")
        frames = motion.get("frames")
        if not isinstance(frames, list):
            errors.append(f"{location}: frames must be an array")
            continue
        for frame_index, frame in enumerate(frames):
            frame_name = frame.get("name") if isinstance(frame, dict) else None
            frame_location = (
                f"{location}/frame[{frame_index}]({frame_name!r})"
            )
            if not isinstance(frame, dict):
                errors.append(f"{frame_location}: must be an object")
                continue
            _motor_ids(frame.get("angles"), f"{frame_location}/angles", errors)
            _motor_ids(
                frame.get("torques"),
                f"{frame_location}/torques",
                errors,
            )

    if not isinstance(aliases, dict):
        errors.append("motion_aliases must be a YAML mapping")
    else:
        for alias, target in aliases.items():
            if target not in names:
                errors.append(
                    f"alias {alias!r} targets missing motion {target!r}"
                )
    return errors


def main() -> int:
    """Load paths, report all validation errors, and return a shell status."""
    parser = argparse.ArgumentParser()
    parser.add_argument("motion_json_path", type=Path)
    parser.add_argument("--aliases", required=True, type=Path)
    args = parser.parse_args()
    try:
        with args.motion_json_path.open(encoding="utf-8") as stream:
            catalog = json.load(stream)
        with args.aliases.open(encoding="utf-8") as stream:
            alias_root = yaml.safe_load(stream)
    except (OSError, json.JSONDecodeError, yaml.YAMLError) as exc:
        print(f"catalog validation failed: {exc}", file=sys.stderr)
        return 1
    aliases = (
        alias_root.get("motion_aliases")
        if isinstance(alias_root, dict)
        else None
    )
    errors = validate_catalog(catalog, aliases)
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(
        f"OK: {len(catalog['motions'])} motions; aliases resolve; "
        "all frame motor IDs are exactly 0..22"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
