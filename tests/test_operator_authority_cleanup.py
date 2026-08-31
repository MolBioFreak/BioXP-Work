from __future__ import annotations

import asyncio
import hashlib
import json
import sqlite3
from pathlib import Path

import pytest
from fastapi import FastAPI, HTTPException
from fastapi.testclient import TestClient

import bioxp.operator_controls as operator_controls
import bioxp.operator_receipt_store as receipt_store


def _create_legacy_history(path: Path) -> None:
    connection = sqlite3.connect(path)
    connection.executescript(
        """
        CREATE TABLE operator_plane_commands (
            command_id TEXT PRIMARY KEY,
            stream_sequence INTEGER NOT NULL,
            method_id TEXT,
            method_sequence INTEGER,
            action_id TEXT NOT NULL,
            requested_json TEXT NOT NULL,
            effective_json TEXT NOT NULL,
            status TEXT NOT NULL,
            version INTEGER NOT NULL,
            ownership_generation INTEGER NOT NULL,
            queued_at REAL NOT NULL,
            dispatched_at REAL,
            finished_at REAL,
            source_noop INTEGER NOT NULL DEFAULT 0,
            source_noop_reason TEXT,
            remote_acknowledged INTEGER NOT NULL DEFAULT 0,
            controller_acknowledged INTEGER NOT NULL DEFAULT 0,
            physical_effect_verified INTEGER NOT NULL DEFAULT 0,
            terminal_json TEXT
        );
        CREATE TABLE operator_plane_methods (
            method_id TEXT PRIMARY KEY,
            name TEXT NOT NULL,
            digest TEXT NOT NULL,
            failure_policy TEXT NOT NULL,
            status TEXT NOT NULL,
            version INTEGER NOT NULL,
            ownership_generation INTEGER NOT NULL,
            expanded_count INTEGER NOT NULL,
            first_stream_sequence INTEGER,
            last_stream_sequence INTEGER,
            queued_at REAL NOT NULL,
            updated_at REAL NOT NULL
        );
        CREATE TABLE operator_plane_transitions (
            transition_sequence INTEGER PRIMARY KEY,
            command_id TEXT,
            method_id TEXT,
            state TEXT NOT NULL,
            payload_json TEXT,
            created_at REAL NOT NULL
        );
        CREATE TABLE serial206_movement_commands (
            command_id TEXT PRIMARY KEY,
            sequence INTEGER NOT NULL,
            state TEXT NOT NULL,
            state_version INTEGER NOT NULL,
            expected_board_epochs_json TEXT NOT NULL,
            terminal_receipt_id TEXT
        );
        CREATE TABLE serial206_command_resources (
            command_id TEXT NOT NULL,
            resource_key TEXT NOT NULL
        );
        """
    )
    connection.execute(
        """
        INSERT INTO operator_plane_commands(
            command_id,stream_sequence,method_id,method_sequence,action_id,
            requested_json,effective_json,status,version,ownership_generation,
            queued_at,dispatched_at,finished_at,terminal_json
        ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?)
        """,
        (
            "legacy-running",
            7,
            "legacy-method",
            1,
            "oem.y.move_steps",
            json.dumps({"steps": 10}),
            json.dumps({"steps": 10}),
            "dispatched",
            3,
            11,
            100.0,
            101.0,
            None,
            None,
        ),
    )
    connection.execute(
        "INSERT INTO serial206_movement_commands VALUES(?,?,?,?,?,?)",
        ("legacy-running", 7, "dispatched", 3, "{}", None),
    )
    connection.execute(
        "INSERT INTO operator_plane_methods VALUES(?,?,?,?,?,?,?,?,?,?,?,?)",
        ("legacy-method", "legacy move", "digest", "fail_fast", "running", 2, 11, 1, 7, 7, 100.0, 101.0),
    )
    connection.execute(
        "INSERT INTO operator_plane_transitions VALUES(?,?,?,?,?,?)",
        (1, "legacy-running", "legacy-method", "dispatched", "{}", 101.0),
    )
    connection.commit()
    connection.close()


def test_legacy_history_reader_is_read_only_and_projects_nonterminal_as_ambiguous(tmp_path: Path) -> None:
    database_path = tmp_path / "bioxp_runtime.db"
    _create_legacy_history(database_path)
    before = hashlib.sha256(database_path.read_bytes()).hexdigest()

    reader_type = getattr(receipt_store, "OperatorHistoryReader", None)
    assert reader_type is not None
    reader = reader_type(tmp_path)
    command = reader.get_command("legacy-running")
    method = reader.get_method("legacy-method")
    detail = reader.command_detail_v2("legacy-running")
    reader.close()

    assert command is not None
    assert command["source"] == "legacy_operator_plane"
    assert command["status"] == "ambiguous"
    assert command["recovery_required"] is True
    assert command["stored_status"] == "dispatched"
    assert detail is not None and detail["source"] == "legacy_operator_plane"
    assert method is not None
    assert method["source"] == "legacy_operator_plane"
    assert method["status"] == "ambiguous"
    assert method["recovery_required"] is True
    assert hashlib.sha256(database_path.read_bytes()).hexdigest() == before


def test_legacy_reader_excludes_current_command_identity_from_history(tmp_path: Path) -> None:
    _create_legacy_history(tmp_path / "bioxp_runtime.db")
    reader_type = getattr(receipt_store, "OperatorHistoryReader", None)
    assert reader_type is not None
    reader = reader_type(tmp_path)
    assert reader.list_commands(limit=10, exclude_command_ids={"legacy-running"}) == []
    reader.close()


def test_operator_receipt_lookup_is_current_first_on_legacy_identity_collision(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _create_legacy_history(tmp_path / "bioxp_runtime.db")
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    app = FastAPI()
    operator_controls.install_operator_control_plane(app)
    current = {
        "schema_version": "bioxp.operator_action_receipt.v1",
        "command_id": "legacy-running",
        "idempotency_key": "current-collision-key",
        "action_id": "query.status",
        "status": "completed",
        "ownership_generation": 1,
        "started_at": "200.0",
        "finished_at": "201.0",
        "controller_acknowledged": False,
        "physical_effect_verified": False,
        "response": {"http_status": 200, "body": {"source": "current"}},
        "stage_receipts": [],
    }
    app.state.operator_receipt_store.put(current)

    with TestClient(app) as client:
        selected = client.get("/operator/actions/receipts/legacy-running")
        history = client.get("/operator/actions/history", params={"limit": 20})

    assert selected.status_code == 200
    assert selected.json()["status"] == "completed"
    matches = [
        row for row in history.json()["receipts"]
        if row["command_id"] == "legacy-running"
    ]
    assert len(matches) == 1
    assert matches[0]["status"] == "completed"


def test_legacy_reader_fails_closed_on_unknown_table_schema(tmp_path: Path) -> None:
    connection = sqlite3.connect(tmp_path / "bioxp_runtime.db")
    connection.execute("CREATE TABLE operator_plane_commands(command_id TEXT PRIMARY KEY, mystery TEXT)")
    connection.commit()
    connection.close()

    reader_type = getattr(receipt_store, "OperatorHistoryReader", None)
    assert reader_type is not None
    reader = reader_type(tmp_path)
    with pytest.raises(RuntimeError, match="unsupported legacy operator history schema"):
        reader.get_command("anything")
    reader.close()


def test_only_aggregate_abort_is_catalogued_and_legacy_z_abort_route_is_excluded() -> None:
    app = FastAPI()

    @app.post("/motion/oem/x/abort", operation_id="aggregate_abort_provider")
    async def aggregate_abort_provider():
        return {"ok": True}

    @app.post("/motion/oem/z/abort", operation_id="legacy_z_abort")
    async def legacy_z_abort():
        return {"ok": False}

    actions, _dispatch = operator_controls._build_catalog(app)
    abort_ids = {str(row["action_id"]) for row in actions if "abort" in str(row["action_id"])}
    assert abort_ids == {"oem.abort_all"}


def test_duplicate_authority_module_and_successive_queue_are_deleted() -> None:
    package_root = Path(operator_controls.__file__).parent
    assert not (package_root / "operator_command_plane.py").exists()
    assert not hasattr(operator_controls, "_SuccessiveMoveQueue")
    source = Path(operator_controls.__file__).read_text(encoding="utf-8")
    assert "operator_command_plane" not in source
    assert "_successive_move_queue" not in source
    assert '"successive_move_queue"' not in source
    assert '"command_queue"' not in source
    assert "oem.z.abort" not in source


def test_legacy_z_abort_route_is_a_tombstone_naming_aggregate_identity(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    from bioxp import api

    with pytest.raises(HTTPException) as captured:
        asyncio.run(api.motion_oem_z_abort())

    assert captured.value.status_code == 410
    assert captured.value.detail["replacement_action_id"] == "oem.abort_all"
