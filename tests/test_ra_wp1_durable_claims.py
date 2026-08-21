from __future__ import annotations

import asyncio
import sqlite3
from pathlib import Path

import pytest

import bioxp.runtime_audit_store as runtime_store_module
from bioxp.operator_receipt_store import OperatorReceiptStore, runtime_state_root
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.services.pipette_service import _run_transport_call


def _claim_payload(*, key: str = "idem-1", volume: float = 10.0) -> dict:
    return {
        "command_id": f"cmd-{key}",
        "idempotency_key": key,
        "action_id": "pipette.aspirate",
        "operation": "aspirate",
        "entrypoint_id": "direct.liquid.aspirate",
        "caller_class": "operator",
        "control_class": "physical_liquid_command",
        "ownership_generation": 4,
        "source_identity": {"revision": "test"},
        "requested_inputs": {"volume_ul": volume},
        "status": "reserved",
    }


def test_runtime_root_rejects_conflicting_environment_overrides(tmp_path, monkeypatch):
    canonical = tmp_path / "canonical"
    legacy = tmp_path / "legacy"
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(canonical))
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(legacy))

    with pytest.raises(ValueError, match="conflicting BioXP runtime roots"):
        runtime_state_root()


def test_legacy_pipette_jsonl_root_never_selects_a_sqlite_root(tmp_path, monkeypatch):
    canonical = tmp_path / "canonical"
    legacy_jsonl = tmp_path / "legacy-jsonl"
    monkeypatch.delenv("BIOXP_OEM_RUNTIME_STATE_ROOT", raising=False)
    monkeypatch.delenv("BIOXP_OEM_RUNTIME_ROOT", raising=False)
    monkeypatch.setenv("BIOXP_PIPETTE_RECEIPT_ROOT", str(legacy_jsonl))
    monkeypatch.setattr(runtime_store_module, "CANONICAL_RUNTIME_ROOT", canonical)

    assert runtime_state_root() == canonical.resolve()
    assert not (legacy_jsonl / "bioxp_runtime.db").exists()


def test_wp1_schema_is_versioned_and_append_only(tmp_path):
    store = OperatorReceiptStore(tmp_path)

    assert store.connection.execute("PRAGMA journal_mode").fetchone()[0] == "wal"
    assert store.connection.execute("PRAGMA synchronous").fetchone()[0] == 2
    assert store.connection.execute("PRAGMA foreign_keys").fetchone()[0] == 1
    assert store.connection.execute("PRAGMA user_version").fetchone()[0] >= 2

    tables = {
        row[0]
        for row in store.connection.execute(
            "SELECT name FROM sqlite_master WHERE type='table'"
        )
    }
    assert {
        "runtime_schema_migrations",
        "runtime_store_identity",
        "runtime_migration_receipts",
        "pipette_operations",
    } <= tables

    store.connection.execute(
        "INSERT INTO operator_commands(command_id,idempotency_key,action_id,status,ownership_generation,started_at,receipt_json,updated_at) "
        "VALUES('cmd-1','key-1','query.status','reserved',1,'now','{}',1)"
    )
    store.connection.execute(
        "INSERT INTO operator_transitions(command_id,state,observed_at) VALUES('cmd-1','reserved',1)"
    )
    with pytest.raises(sqlite3.IntegrityError, match="append-only"):
        store.connection.execute(
            "UPDATE operator_transitions SET state='completed' WHERE command_id='cmd-1'"
        )
    with pytest.raises(sqlite3.IntegrityError, match="append-only"):
        store.connection.execute("DELETE FROM operator_transitions WHERE command_id='cmd-1'")
    with pytest.raises(sqlite3.IntegrityError):
        store.connection.execute("DELETE FROM operator_commands WHERE command_id='cmd-1'")


def test_claim_replay_requires_same_canonical_payload_and_persists_identity(tmp_path):
    store = OperatorReceiptStore(tmp_path)
    first, created = store.claim(_claim_payload())
    assert created is True
    assert first["status"] == "reserved"

    row = store.connection.execute(
        "SELECT command_id, canonical_request_sha256, operation, entrypoint_id, caller_class, control_class "
        "FROM operator_commands WHERE command_id=?",
        (first["command_id"],),
    ).fetchone()
    assert row["canonical_request_sha256"]
    assert row["operation"] == "aspirate"
    assert row["entrypoint_id"] == "direct.liquid.aspirate"
    assert row["caller_class"] == "operator"
    assert row["control_class"] == "physical_liquid_command"

    replay, replay_created = store.claim(_claim_payload())
    assert replay_created is False
    assert replay["command_id"] == first["command_id"]

    with pytest.raises(ValueError, match="idempotency key conflict"):
        store.claim(_claim_payload(volume=11.0))


def test_pipette_receipts_use_the_canonical_sqlite_authority(tmp_path):
    store = PipetteReceiptStore(tmp_path)
    assert store.path == tmp_path / "bioxp_runtime.db"
    assert store.receipts_path is None
    claim, created = store.claim(
        operation="status",
        requested_inputs={},
        entrypoint_id="direct.liquid.status",
        caller_class="query",
        control_class="hardware_query",
        idempotency_key="status-key",
    )
    assert created is True
    assert claim["status"] == "reserved"
    assert store.connection.execute(
        "SELECT COUNT(*) FROM pipette_operations WHERE command_id=?",
        (claim["command_id"],),
    ).fetchone()[0] == 1


def test_oem_runtime_store_uses_the_same_schema_authority(tmp_path):
    operator = OperatorReceiptStore(tmp_path)
    runtime = OEMRuntimeStore(tmp_path)

    assert runtime.root == operator.root == tmp_path.resolve()
    assert runtime._db.execute("PRAGMA user_version").fetchone()[0] >= 2
    assert runtime._db.execute(
        "SELECT database_path FROM runtime_store_identity WHERE identity_id=1"
    ).fetchone()[0] == str((tmp_path / "bioxp_runtime.db").resolve())
    assert runtime._db.execute(
        "SELECT name FROM sqlite_master WHERE type='table' AND name='serial206_receipts'"
    ).fetchone() is not None


def test_service_claims_before_get_transport_and_operation():
    events: list[str] = []

    class TrackingStore:
        def claim(self, **kwargs):
            events.append("claim")
            return {"command_id": "cmd-service", "status": "reserved"}, True

        def record(self, **kwargs):
            events.append("record")
            return {"receipt_id": "receipt-service", "truth": {}, "source_identity": {}}

    def get_transport():
        events.append("get_transport")
        return object()

    async def run_blocking(_label, operation, *, timeout_s):
        del timeout_s
        return operation()

    result = asyncio.run(
        _run_transport_call(
            "Pipette status",
            timeout_s=1,
            get_transport=get_transport,
            run_blocking=run_blocking,
            operation=lambda _transport: (events.append("operation") or {"ok": True}),
            operation_name="status",
            receipt_store=TrackingStore(),
        )
    )

    assert result["receipt_id"] == "receipt-service"
    assert events == ["claim", "get_transport", "operation", "record"]
