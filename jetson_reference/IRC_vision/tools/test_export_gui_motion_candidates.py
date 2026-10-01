import copy
import importlib.util
import json
from pathlib import Path

import pytest


MODULE_PATH = Path(__file__).with_name("export_gui_motion_candidates.py")
SPEC = importlib.util.spec_from_file_location("export_gui_motion_candidates", MODULE_PATH)
exporter = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(exporter)


def frame(name="pose", start_ms=0, time_ms=100):
    return {
        "frame_id": f"id-{name}",
        "source_frame_id": f"source-{name}",
        "name": name,
        "time_ms": time_ms,
        "angles": {str(motor_id): float(motor_id - 11) for motor_id in range(23)},
        "torques": {str(motor_id): True for motor_id in range(23)},
        "is_important": False,
        "start_ms": start_ms,
    }


def motion(name):
    return {
        "name": name,
        "max_seq_ms": 1000,
        "repeat_count": 1,
        "playback_speed": 1.0,
        "frames": [frame(name)],
    }


def sequence(name, frame_count, repeat_count=1, playback_speed=1.0):
    frames = [frame(f"{name}-{index}", index * 100) for index in range(frame_count)]
    return {
        "sequence_id": f"sequence-{name}",
        "name": name,
        "max_seq_ms": 8000,
        "repeat_count": repeat_count,
        "playback_speed": playback_speed,
        "repeatable": True,
        "completion": {
            "position_tolerance_deg": 3.0,
            "settle_duration_ms": 80,
            "settle_timeout_ms": 700,
        },
        "frames": frames,
    }


@pytest.fixture
def inputs():
    official = {"version": 1, "motions": [motion(f"official-{i}") for i in range(6)]}
    gui = {
        "saved_sequences": [
            sequence("공잡기", 6),
            sequence("공잡기 리그랩까지", 10),
            sequence("좌회전1", 6, repeat_count=5),
            sequence("우회전", 4, repeat_count=3, playback_speed=0.9),
        ]
    }
    return gui, official


def test_exports_exactly_four_candidates_with_loader_compatible_schema(inputs):
    gui, official = inputs
    result = exporter.build_candidate_catalog(gui, official)
    assert len(result["motions"]) == 10
    candidates = {item["name"]: item for item in result["motions"][6:]}
    assert {name: len(item["frames"]) for name, item in candidates.items()} == {
        "공잡기": 6,
        "공잡기 리그랩까지": 10,
        "좌회전1": 6,
        "우회전": 4,
    }
    assert candidates["좌회전1"]["repeat_count"] == 5
    assert candidates["우회전"]["playback_speed"] == 0.9
    exporter.validate_catalog(json.loads(json.dumps(result)))


def test_missing_sequence_has_clear_error(inputs):
    gui, official = inputs
    with pytest.raises(exporter.ExportError, match="requested GUI sequence not found: 없음"):
        exporter.build_candidate_catalog(gui, official, ["없음"])


def test_duplicate_motion_name_collision_is_rejected(inputs):
    gui, official = inputs
    official["motions"].append(motion("공잡기"))
    with pytest.raises(exporter.ExportError, match="motion name collision: 공잡기"):
        exporter.build_candidate_catalog(gui, official, ["공잡기"])


@pytest.mark.parametrize("field", ["angles", "torques"])
def test_missing_motor_id_is_rejected(inputs, field):
    gui, official = inputs
    del gui["saved_sequences"][0]["frames"][0][field]["22"]
    with pytest.raises(exporter.ExportError, match="motor IDs 0-22"):
        exporter.build_candidate_catalog(gui, official, ["공잡기"])


@pytest.mark.parametrize(("field", "value"), [("start_ms", -1), ("time_ms", 0)])
def test_invalid_frame_time_is_rejected(inputs, field, value):
    gui, official = inputs
    gui["saved_sequences"][0]["frames"][0][field] = value
    with pytest.raises(exporter.ExportError, match=field):
        exporter.build_candidate_catalog(gui, official, ["공잡기"])


@pytest.mark.parametrize("input_name", ["sdk_gui_state.json", "robot_motions.json"])
def test_output_cannot_overwrite_either_input(inputs, tmp_path, input_name):
    gui, official = inputs
    gui_path = tmp_path / "sdk_gui_state.json"
    official_path = tmp_path / "robot_motions.json"
    gui_path.write_text(json.dumps(gui), encoding="utf-8")
    official_path.write_text(json.dumps(official), encoding="utf-8")
    protected_path = tmp_path / input_name
    before = protected_path.read_bytes()
    with pytest.raises(exporter.ExportError, match="must not overwrite"):
        exporter.export_candidates(gui_path, official_path, protected_path)
    assert protected_path.read_bytes() == before


def test_first_and_last_vectors_and_metadata_are_preserved(inputs):
    gui, official = inputs
    source = copy.deepcopy(gui["saved_sequences"][0])
    result = exporter.build_candidate_catalog(gui, official, ["공잡기"])
    candidate = result["motions"][-1]
    assert candidate["frames"][0]["angles"] == source["frames"][0]["angles"]
    assert candidate["frames"][-1]["angles"] == source["frames"][-1]["angles"]
    assert candidate["frames"] == source["frames"]
    assert candidate["completion"] == source["completion"]
