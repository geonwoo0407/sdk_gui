import json
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "mission_control"))

from legacy_motion_executor_adapter import (  # noqa: E402
    build_executor_request,
    map_action_to_motion_id,
    parse_legacy_motion_command,
)
from legacy_motion_status_adapter import convert_executor_status  # noqa: E402

VECTORS_PATH = (
    PACKAGE_ROOT.parent
    / "irc_step_motion_executor"
    / "test"
    / "legacy_catalog_contract_vectors.json"
)
VECTORS = json.loads(VECTORS_PATH.read_text(encoding="utf-8"))
REQUEST_FIELDS = {
    "action",
    "command_id",
    "event_id",
    "request_id",
    "motion_id",
}
STATUS_FIELDS = REQUEST_FIELDS | {"status", "error_code", "message"}


@pytest.mark.parametrize("vector", VECTORS, ids=lambda item: item["name"])
def test_legacy_adapter_matches_shared_request_vectors(vector):
    command = parse_legacy_motion_command(vector["legacy"])
    motion_id = map_action_to_motion_id(command.action)
    request = build_executor_request(
        vector["request"]["request_id"],
        motion_id,
        command.command_id,
        command.action,
        command.event_id,
    )

    assert request == vector["request"]
    assert REQUEST_FIELDS <= request.keys()
    assert isinstance(request["action"], str)
    assert isinstance(request["request_id"], int)
    assert not isinstance(request["request_id"], bool)
    assert isinstance(request["motion_id"], str)
    for field in ("command_id", "event_id"):
        assert request[field] is None or (
            isinstance(request[field], int)
            and not isinstance(request[field], bool)
        )


@pytest.mark.parametrize("vector", VECTORS, ids=lambda item: item["name"])
def test_catalog_status_shape_round_trips_through_status_adapter(vector):
    request = vector["request"]
    catalog_status = {
        field: request[field] for field in REQUEST_FIELDS
    }
    catalog_status.update(
        status="REJECTED",
        error_code=vector["expected_error_code"],
        message="catalog-only contract test",
    )

    assert STATUS_FIELDS == catalog_status.keys()
    converted = convert_executor_status(json.dumps(catalog_status))
    for field in (
        "status",
        "action",
        "command_id",
        "event_id",
        "request_id",
        "motion_id",
        "error_code",
        "message",
    ):
        assert converted[field] == catalog_status[field]


def test_null_event_id_is_present_but_missing_event_id_is_detectable():
    request = build_executor_request(
        9, "forward_short", 109, "FINE_FORWARD_STEP", None
    )
    assert "event_id" in request
    assert request["event_id"] is None

    missing_event_id = dict(request)
    del missing_event_id["event_id"]
    assert "event_id" not in missing_event_id


def test_status_adapter_accepts_serialized_catalog_null_fields():
    catalog_status = {
        "status": "REJECTED",
        "action": "SLOW_APPROACH",
        "command_id": None,
        "event_id": None,
        "request_id": 10,
        "motion_id": "forward_short",
        "error_code": "HARDWARE_NOT_READY",
        "message": "catalog-only mode",
    }
    converted = convert_executor_status(json.dumps(catalog_status))
    assert converted["status"] == "REJECTED"
    assert converted["action"] == "SLOW_APPROACH"
    assert converted["command_id"] is None
    assert converted["event_id"] is None
