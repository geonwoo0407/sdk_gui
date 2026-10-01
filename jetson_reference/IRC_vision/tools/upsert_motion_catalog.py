#!/usr/bin/env python3
"""UPSERT aliased motion objects without deleting unrelated runtime entries."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile

import yaml


def _sha256(path: Path) -> str:
    """Return the SHA256 digest of one file."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_catalog(path: Path) -> dict:
    """Load and minimally validate one motion catalog."""
    payload = json.loads(path.read_text(encoding="utf-8"))
    motions = payload.get("motions")
    if not isinstance(motions, list):
        raise ValueError(f"{path}: motions must be a list")
    if any(not isinstance(item, dict) or not item.get("name") for item in motions):
        raise ValueError(f"{path}: every motion must have a non-empty name")
    return payload


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--runtime", type=Path, required=True)
    parser.add_argument("--aliases", type=Path, required=True)
    parser.add_argument("--backup-dir", type=Path, default=Path("/tmp"))
    return parser.parse_args()


def main() -> int:
    """Back up runtime and UPSERT only exact names referenced by aliases."""
    args = _parse_args()
    source_payload = _load_catalog(args.source)
    runtime_payload = _load_catalog(args.runtime)
    aliases_payload = yaml.safe_load(args.aliases.read_text(encoding="utf-8"))
    aliases = aliases_payload.get("motion_aliases", {})
    if not isinstance(aliases, dict) or not aliases:
        raise ValueError("motion_aliases must be a non-empty mapping")

    required_names = set(aliases.values())
    source_counts = Counter(item["name"] for item in source_payload["motions"])
    runtime_counts = Counter(item["name"] for item in runtime_payload["motions"])
    missing = sorted(name for name in required_names if source_counts[name] == 0)
    source_duplicates = {
        name: source_counts[name]
        for name in sorted(required_names)
        if source_counts[name] > 1
    }
    runtime_duplicates = {
        name: runtime_counts[name]
        for name in sorted(required_names)
        if runtime_counts[name] > 1
    }
    if missing or source_duplicates or runtime_duplicates:
        raise ValueError(
            "preflight failed: "
            f"missing={missing}, source_duplicates={source_duplicates}, "
            f"runtime_duplicates={runtime_duplicates}"
        )

    source_by_name = {
        item["name"]: item for item in source_payload["motions"]
    }
    motions = list(runtime_payload["motions"])
    runtime_positions = {
        item["name"]: index for index, item in enumerate(motions)
    }
    added = []
    replaced = []
    identical = []
    for name in sorted(required_names):
        source_motion = source_by_name[name]
        position = runtime_positions.get(name)
        if position is None:
            runtime_positions[name] = len(motions)
            motions.append(source_motion)
            added.append(name)
        elif motions[position] == source_motion:
            identical.append(name)
        else:
            motions[position] = source_motion
            replaced.append(name)

    args.backup_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    backup = args.backup_dir / f"robot_motions.before-upsert-{timestamp}.json"
    before_sha = _sha256(args.runtime)
    shutil.copy2(args.runtime, backup)
    backup_sha = _sha256(backup)
    if backup_sha != before_sha:
        raise RuntimeError("backup SHA256 does not match runtime")

    merged_payload = dict(runtime_payload)
    merged_payload["motions"] = motions
    descriptor, temporary_name = tempfile.mkstemp(
        prefix="robot_motions.upsert.",
        suffix=".json",
        dir=args.runtime.parent,
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(merged_payload, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, args.runtime)
    finally:
        if os.path.exists(temporary_name):
            os.unlink(temporary_name)

    merged = _load_catalog(args.runtime)
    merged_counts = Counter(item["name"] for item in merged["motions"])
    invalid_counts = {
        name: merged_counts[name]
        for name in sorted(required_names)
        if merged_counts[name] != 1
    }
    if invalid_counts:
        raise RuntimeError(f"postflight alias counts invalid: {invalid_counts}")

    print(f"backup={backup}")
    print(f"before_sha256={before_sha}")
    print(f"backup_sha256={backup_sha}")
    print(f"after_sha256={_sha256(args.runtime)}")
    print(f"motions={len(runtime_payload['motions'])}->{len(merged['motions'])}")
    print(f"added={len(added)} replaced={len(replaced)} identical={len(identical)}")
    print("active_alias_missing=0 active_alias_duplicates=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
