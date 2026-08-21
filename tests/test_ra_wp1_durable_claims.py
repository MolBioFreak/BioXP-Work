from __future__ import annotations

import asyncio
import sqlite3
from pathlib import Path

import pytest

import bioxp.runtime_audit_store as runtime_store_module
from bioxp.operator_receipt_store import OperatorReceiptStore, runtime_state_root
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.pipette.receipts import PipetteReceiptError, PipetteReceiptStore
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
    claim_kwargs: dict[str, object] = {}
    record_kwargs: dict[str, object] = {}

    class TrackingStore:
        def claim(self, **kwargs):
            claim_kwargs.update(kwargs)
            events.append("claim")
            return {"command_id": "cmd-service", "status": "reserved"}, True

        def record(self, **kwargs):
            record_kwargs.update(kwargs)
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
            operation=lambda _transport: (
                events.append("operation")
                or {"ok": True, "provenance": {"transaction_id": "txn-service"}}
            ),
            operation_name="status",
            receipt_store=TrackingStore(),
        )
    )

    assert result["receipt_id"] == "receipt-service"
    assert isinstance(claim_kwargs.get("callback_session_id"), str)
    assert claim_kwargs["callback_session_id"].startswith("pipette-callback:")
    assert record_kwargs["runtime_binding"]["callback_session_id"] == claim_kwargs["callback_session_id"]
    assert record_kwargs["result"]["callback_session_id"] == claim_kwargs["callback_session_id"]
    assert record_kwargs["result"]["provenance"]["callback_session_id"] == claim_kwargs["callback_session_id"]
    assert events == ["claim", "get_transport", "operation", "record"]


def test_pipette_claim_persists_all_typed_correlation_fields(tmp_path):
    store = PipetteReceiptStore(tmp_path)
    claim, created = store.claim(
        operation="aspirate",
        requested_inputs={"volume_ul": 10.0},
        entrypoint_id="protocol.pipette.aspirate",
        caller_class="protocol",
        control_class="physical_liquid_command",
        idempotency_key="typed-correlation",
        connection_generation=8,
        protocol_job_id="protocol-job-1",
        protocol_action_id="action-1",
        lifecycle_stage_id="stage-1",
        callback_session_id="callback-1",
        runtime_binding={"owner": "test"},
    )

    assert created is True
    row = store.connection.execute(
        """
        SELECT connection_generation, protocol_job_id, protocol_action_id,
               lifecycle_stage_id, callback_session_id
        FROM pipette_operations WHERE pipette_operation_id=?
        """,
        (claim["pipette_operation_id"],),
    ).fetchone()
    assert tuple(row) == (8, "protocol-job-1", "action-1", "stage-1", "callback-1")


def test_pipette_replay_repairs_missing_typed_child(tmp_path):
    store = PipetteReceiptStore(tmp_path)
    payload = _claim_payload(key="missing-child")
    first, created = store._audit_database.claim(payload, pipette=False)
    assert created is True
    assert store.connection.execute(
        "SELECT COUNT(*) FROM pipette_operations WHERE command_id=?",
        (first["command_id"],),
    ).fetchone()[0] == 0

    replay, replay_created = store.claim(
        operation="aspirate",
        requested_inputs=payload["requested_inputs"],
        entrypoint_id=payload["entrypoint_id"],
        caller_class=payload["caller_class"],
        control_class=payload["control_class"],
        idempotency_key=payload["idempotency_key"],
        command_id=payload["command_id"],
        ownership_generation=payload["ownership_generation"],
        runtime_binding=payload["source_identity"],
    )

    assert replay_created is False
    assert replay["pipette_operation_id"]
    assert store.connection.execute(
        "SELECT COUNT(*) FROM pipette_operations WHERE command_id=?",
        (first["command_id"],),
    ).fetchone()[0] == 1


def test_receipt_without_controller_ack_is_not_acknowledged(tmp_path):
    store = PipetteReceiptStore(tmp_path)
    claim, _ = store.claim(
        operation="aspirate",
        requested_inputs={"volume_ul": 10.0},
        entrypoint_id="direct.liquid.aspirate",
        caller_class="operator",
        control_class="physical_liquid_command",
        idempotency_key="tx-only-status",
    )
    store.record(
        operation="aspirate",
        requested_inputs={"volume_ul": 10.0},
        result={
            "ok": True,
            "delivery_verified": True,
            "controller_acknowledged": False,
            "completion_verified": False,
            "outcome": "tx_only",
        },
        command_id=claim["command_id"],
        pipette_operation_id=claim["pipette_operation_id"],
    )

    row = store.connection.execute(
        "SELECT status, controller_acknowledged FROM pipette_operations WHERE pipette_operation_id=?",
        (claim["pipette_operation_id"],),
    ).fetchone()
    assert row["status"] == "dispatched"
    assert row["controller_acknowledged"] == 0


def test_transport_exception_closes_claim_with_failure_receipt(tmp_path):
    store = PipetteReceiptStore(tmp_path)

    def get_transport():
        raise RuntimeError("transport unavailable")

    async def run_blocking(_label, operation, *, timeout_s):
        del timeout_s
        return operation()

    with pytest.raises(Exception):
        asyncio.run(
            _run_transport_call(
                "Pipette status",
                timeout_s=1,
                get_transport=get_transport,
                run_blocking=run_blocking,
                operation=lambda _transport: {"ok": True},
                operation_name="status",
                receipt_store=store,
            )
        )

    row = store.connection.execute(
        "SELECT status, failure_code FROM pipette_operations ORDER BY created_at DESC LIMIT 1"
    ).fetchone()
    assert row["status"] == "failed"
    assert row["failure_code"] == "transport_exception"


def test_normalization_failure_closes_reserved_claim(tmp_path):
    store = PipetteReceiptStore(tmp_path)
    claim, _ = store.claim(
        operation="readback",
        requested_inputs={},
        entrypoint_id="direct.liquid.readback",
        caller_class="direct_api",
        control_class="hardware_query",
        idempotency_key="normalize-failure",
    )

    with pytest.raises(PipetteReceiptError, match="not-an-int"):
        store.record(
            operation="readback",
            requested_inputs={},
            result={"ok": True, "channels": [{"channel": "not-an-int"}]},
            command_id=claim["command_id"],
            pipette_operation_id=claim["pipette_operation_id"],
        )

    row = store.connection.execute(
        "SELECT status,failure_code,outcome FROM pipette_operations WHERE pipette_operation_id=?",
        (claim["pipette_operation_id"],),
    ).fetchone()
    assert row["status"] == "failed"
    assert row["failure_code"] == "pipette_result_normalization_failed"
    assert row["outcome"] == "normalization_failed"


def test_serial206_pipette_lifecycle_calls_audit_runner_before_transport():
    from src.bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter

    class Transport:
        def initialize(self, _command):
            return {"ok": True}

    adapter = Serial206ProductionPrimitiveAdapter(
        object(),
        Transport(),
        authority_provider=lambda: {},
        generation_provider=lambda: 0,
    )
    calls = []

    def runner(operation_name, operation, *, requested_inputs, lifecycle_stage_id):
        calls.append((operation_name, requested_inputs, lifecycle_stage_id))
        assert operation(adapter.pipette_transport)["ok"] is True
        return {"ok": True, "receipt_id": "receipt-lifecycle"}

    adapter.pipette_audit_runner = runner
    result = adapter.initiate_pipette_group()

    assert result["receipt_id"] == "receipt-lifecycle"
    assert calls == [
        ("initialize", {"pressure_profile": "1R", "prime_volume_ul": None}, "serial206.initiate_pipette_group")
    ]


def test_coordinator_persists_explicit_connection_generation_fallback(tmp_path):
    store = PipetteReceiptStore(tmp_path)

    def get_transport():
        return object()

    async def run_blocking(_label, operation, *, timeout_s):
        del timeout_s
        return operation()

    asyncio.run(
        _run_transport_call(
            "Pipette status",
            timeout_s=1,
            get_transport=get_transport,
            run_blocking=run_blocking,
            operation=lambda _transport: {"ok": True, "delivery_verified": True},
            operation_name="status",
            receipt_store=store,
        )
    )

    row = store.connection.execute(
        "SELECT connection_generation FROM pipette_operations ORDER BY created_at DESC LIMIT 1"
    ).fetchone()
    assert row["connection_generation"] == 0
