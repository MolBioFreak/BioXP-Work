from __future__ import annotations

import base64
import hashlib
import json
import sqlite3
import time
from pathlib import Path

import pytest

from bioxp.operator_receipt_store import OperatorReceiptStore
from bioxp.pipette.receipts import PipetteReceiptError, PipetteReceiptStore
from bioxp.oem_runtime_store import OEMRuntimeStore
import bioxp.pipette.receipts as pipette_receipts_module


@pytest.fixture(autouse=True)
def verified_pipette_authority(monkeypatch):
    monkeypatch.setattr(pipette_receipts_module, "current_release_identity", lambda: {
        "verified": True,
        "release_id": "test-release",
        "source": {"manifest_sha256": "1" * 64, "aggregate_sha256": "2" * 64},
    })
    monkeypatch.setattr(pipette_receipts_module, "current_authority_identity", lambda: {
        "evidence_lock_identity_verified": True,
        "evidence_lock_sha256": "3" * 64,
    })
    monkeypatch.setattr(pipette_receipts_module, "current_registry_sha256", lambda: "4" * 64)


def _prepare_runtime(root: Path) -> None:
    owner = OEMRuntimeStore(root)
    owner.close()


def _operator_store(root: Path) -> OperatorReceiptStore:
    _prepare_runtime(root)
    return OperatorReceiptStore(root)


def _pipette_store(root: Path) -> PipetteReceiptStore:
    _prepare_runtime(root)
    return PipetteReceiptStore(root)


def legacy_pipette_receipt(
    receipt_id: str,
    *,
    created_at: str,
    operation: str,
    channel: int,
) -> dict:
    return {
        "schema": "bioxp.pipette.receipt.v1",
        "receipt_id": receipt_id,
        "created_at": created_at,
        "operation": operation,
        "requested_inputs": {"channel": channel},
        "effective_inputs": {"channel": channel},
        "result": {"ok": True, "outcome": "completion", "channel": channel},
        "truth": {
            "delivery_verified": True,
            "controller_acknowledged": True,
            "completion_verified": True,
            "hardware_precondition_verified": False,
            "hardware_postcondition_verified": False,
            "physical_effect_verified": False,
            "physical_effect_claim_suppressed": True,
        },
        "runtime_binding": {"source": "legacy"},
        "source_identity": {
            "source_sha256": {"pipette_receipts": "a" * 64},
            "registry_sha256": "b" * 64,
            "evidence_authority": {"kind": "test-fixture"},
            "authority_verified": False,
        },
    }


def test_evidence_expiry_is_two_phase_and_uses_only_persisted_retention_authority(tmp_path, monkeypatch):
    store = _operator_store(tmp_path)
    monkeypatch.setattr(store, "_five_calendar_year_deadline", lambda _now: 0.0)
    receipt = {
        "command_id": "cmd-evidence-1",
        "idempotency_key": "idem-evidence-1",
        "action_id": "pipette.aspirate",
        "status": "completed",
        "started_at": time.time(),
        "finished_at": time.time(),
        "duration_ms": 10.0,
        "controller_acknowledged": True,
        "physical_effect_verified": False,
        "ownership_generation": 3,
        "response": {"large": "payload", "value": 7},
        "stage_receipts": [{"stage": "completion", "ok": True}],
        "evidence_retention_deadline": 0.0,
        "legal_hold": True,
    }
    compact = store.put(receipt)
    relpath = compact["evidence_relpath"]
    evidence_path = store.root / relpath
    assert evidence_path.exists()

    artifact = store.connection.execute(
        "SELECT evidence_artifact_id,sha256,byte_count,active_relpath,expiry_state FROM runtime_evidence_objects"
    ).fetchone()
    assert artifact is not None
    digest = artifact["sha256"]
    size = artifact["byte_count"]
    assert digest == hashlib.sha256(evidence_path.read_bytes()).hexdigest()
    assert size == evidence_path.stat().st_size
    assert artifact["active_relpath"] == relpath
    assert artifact["expiry_state"] == "active"

    persisted = store.connection.execute(
        "SELECT retention_deadline,legal_hold FROM runtime_evidence_objects WHERE evidence_artifact_id=?",
        (artifact["evidence_artifact_id"],),
    ).fetchone()
    persisted_deadline = persisted["retention_deadline"]
    assert persisted_deadline == 0.0
    assert persisted["legal_hold"] == 0
    with pytest.raises(TypeError):
        store.expire_evidence("cmd-evidence-1", retention_deadline=0.0, now=1.0)
    result = store.expire_evidence("cmd-evidence-1")
    assert result["state"] == "expired"
    assert not evidence_path.exists()

    row = store.connection.execute(
        "SELECT evidence_relpath,evidence_sha256,evidence_bytes,evidence_state FROM operator_commands WHERE command_id=?",
        ("cmd-evidence-1",),
    ).fetchone()
    assert row["evidence_relpath"] is None
    assert row["evidence_sha256"] == digest
    assert row["evidence_bytes"] == size
    assert row["evidence_state"] == "expired"
    events = [
        row[0]
        for row in store.connection.execute(
            "SELECT event_kind FROM runtime_evidence_events WHERE evidence_artifact_id=? ORDER BY event_id",
            (artifact["evidence_artifact_id"],),
        ).fetchall()
    ]
    assert events == ["published", "expiry_pending", "deleted", "expired"]


def test_evidence_expiry_rejects_symlink_substitution_without_unlinking_target(tmp_path, monkeypatch):
    store = _operator_store(tmp_path)
    monkeypatch.setattr(store, "_five_calendar_year_deadline", lambda _now: 0.0)
    selected = {
        "command_id": "cmd-evidence-symlink",
        "idempotency_key": "idem-evidence-symlink",
        "action_id": "pipette.status",
        "status": "completed",
        "started_at": time.time(),
        "finished_at": time.time(),
        "ownership_generation": 3,
        "response": {"value": "retained"},
        "stage_receipts": [],
    }
    compact = store.put(selected)
    evidence_path = store.root / compact["evidence_relpath"]
    outside = tmp_path.parent / "outside-retention-target.json"
    outside.write_text("outside", encoding="utf-8")
    evidence_path.unlink()
    evidence_path.symlink_to(outside)

    with pytest.raises(RuntimeError, match="evidence expiry integrity failure"):
        store.expire_evidence(selected["command_id"])

    assert outside.read_text(encoding="utf-8") == "outside"
    assert evidence_path.is_symlink()
    artifact = store.connection.execute(
        "SELECT expiry_state FROM runtime_evidence_objects WHERE command_id=?",
        (selected["command_id"],),
    ).fetchone()
    assert artifact["expiry_state"] == "integrity_failed"


def test_pipette_jsonl_migration_is_digest_bound_and_idempotent(tmp_path):
    store = _pipette_store(tmp_path)
    legacy_rows = [
        legacy_pipette_receipt(
            "legacy-pipette-1",
            created_at="2026-08-20T00:00:00Z",
            operation="status",
            channel=0,
        ),
        legacy_pipette_receipt(
            "legacy-pipette-2",
            created_at="2026-08-20T00:00:01Z",
            operation="pressure",
            channel=1,
        ),
    ]
    raw = ("\n".join(json.dumps(row, sort_keys=True) for row in legacy_rows) + "\n").encode()
    store._legacy_path.write_bytes(raw)

    first = store.migrate_legacy_jsonl()
    assert first["status"] == "completed"
    assert first["imported_count"] == 2
    assert first["source_sha256"] == hashlib.sha256(raw).hexdigest()
    archive_path = store.root / first["archive_relpath"]
    assert archive_path.read_bytes() == raw
    assert not store._legacy_path.exists()
    assert store.connection.execute("SELECT COUNT(*) FROM pipette_operations").fetchone()[0] == 2

    second = store.migrate_legacy_jsonl()
    assert second["status"] == "already_imported"
    assert second["imported_count"] == 0
    assert store.connection.execute("SELECT COUNT(*) FROM pipette_operations").fetchone()[0] == 2
    migration = store.connection.execute(
        "SELECT archive_relpath FROM runtime_migration_receipts"
    ).fetchone()
    assert migration["archive_relpath"] == first["archive_relpath"]
    retirement = store.connection.execute(
        "SELECT retired_at,retirement_sha256 FROM runtime_migration_retirements"
    ).fetchone()
    assert retirement["retired_at"] is not None
    assert retirement["retirement_sha256"] == first["source_sha256"]
    assert archive_path.stat().st_mode & 0o777 == 0o400


def test_pipette_jsonl_quarantine_binds_exact_line_bytes_and_typed_reason(tmp_path):
    store = _pipette_store(tmp_path)
    valid_line = (
        json.dumps(
            legacy_pipette_receipt(
                "legacy-valid",
                created_at="2026-08-20T00:00:00Z",
                operation="status",
                channel=0,
            ),
            sort_keys=True,
        ).encode("utf-8")
        + b"\n"
    )
    invalid_line = b'{"schema":"bioxp.pipette.receipt.v1",broken}\r\n'
    raw = valid_line + invalid_line
    store._legacy_path.write_bytes(raw)

    migrated = store.migrate_legacy_jsonl()

    assert migrated["status"] == "completed_with_quarantine"
    assert migrated["source_count"] == 2
    assert migrated["imported_count"] == 1
    assert migrated["quarantined_count"] == 1
    quarantine_path = store.root / migrated["quarantine_relpath"]
    quarantine_bytes = quarantine_path.read_bytes()
    assert hashlib.sha256(quarantine_bytes).hexdigest() == migrated["quarantine_sha256"]
    quarantine = json.loads(quarantine_bytes)
    assert quarantine["line_number"] == 2
    assert quarantine["line_sha256"] == hashlib.sha256(invalid_line).hexdigest()
    assert quarantine["line_bytes"] == len(invalid_line)
    assert base64.b64decode(quarantine["raw_base64"]) == invalid_line
    assert quarantine["reason_code"] == "invalid_json"
    assert quarantine_path.stat().st_mode & 0o777 == 0o400
    marker = store.connection.execute(
        "SELECT source_count,imported_count,quarantined_count FROM runtime_migration_receipts"
    ).fetchone()
    assert tuple(marker) == (2, 1, 1)

    restarted = store.migrate_legacy_jsonl()
    assert restarted["status"] == "already_imported"
    assert restarted["quarantine_relpath"] == migrated["quarantine_relpath"]
    assert restarted["quarantine_sha256"] == migrated["quarantine_sha256"]


def test_quarantine_publication_failure_precedes_all_database_mutation(tmp_path, monkeypatch):
    store = _pipette_store(tmp_path)
    valid_line = (
        json.dumps(
            legacy_pipette_receipt(
                "legacy-before-invalid",
                created_at="2026-08-20T00:00:00Z",
                operation="status",
                channel=0,
            ),
            sort_keys=True,
        ).encode("utf-8")
        + b"\n"
    )
    raw = valid_line + b"not-json\n"
    store._legacy_path.write_bytes(raw)
    real_write = store._write_immutable_migration_artifact

    def fail_quarantine(path, data, *, expected_sha256):
        if "quarantine" in path.parts:
            raise OSError("injected quarantine publication failure")
        return real_write(path, data, expected_sha256=expected_sha256)

    monkeypatch.setattr(store, "_write_immutable_migration_artifact", fail_quarantine)
    with pytest.raises(OSError, match="injected quarantine publication failure"):
        store.migrate_legacy_jsonl()

    assert store._legacy_path.read_bytes() == raw
    assert store.connection.execute("SELECT COUNT(*) FROM operator_commands").fetchone()[0] == 0
    assert store.connection.execute("SELECT COUNT(*) FROM pipette_operations").fetchone()[0] == 0
    assert store.connection.execute("SELECT COUNT(*) FROM runtime_migration_receipts").fetchone()[0] == 0
    assert store.connection.execute("SELECT COUNT(*) FROM runtime_migration_retirements").fetchone()[0] == 0


def test_pipette_jsonl_import_and_reconciliation_roll_back_as_one_transaction(tmp_path, monkeypatch):
    store = _pipette_store(tmp_path)
    rows = [
        legacy_pipette_receipt(
            "legacy-atomic-1",
            created_at="2026-08-20T00:00:00Z",
            operation="status",
            channel=0,
        ),
        legacy_pipette_receipt(
            "legacy-atomic-2",
            created_at="2026-08-20T00:00:01Z",
            operation="pressure",
            channel=1,
        ),
    ]
    raw = ("\n".join(json.dumps(row, sort_keys=True) for row in rows) + "\n").encode("utf-8")
    store._legacy_path.write_bytes(raw)
    real_finalize = store._audit_database.finalize_claim
    finalized = 0

    def fail_second_finalize(**kwargs):
        nonlocal finalized
        real_finalize(**kwargs)
        finalized += 1
        if finalized == 2:
            raise RuntimeError("injected second-receipt failure")

    monkeypatch.setattr(store._audit_database, "finalize_claim", fail_second_finalize)
    with pytest.raises(RuntimeError, match="injected second-receipt failure"):
        store.migrate_legacy_jsonl()

    assert store._legacy_path.read_bytes() == raw
    for table in (
        "operator_commands",
        "operator_transitions",
        "pipette_operations",
        "pipette_channel_observations",
        "pipette_transport_exchanges",
        "runtime_events",
        "runtime_migration_receipts",
        "runtime_migration_evidence",
        "runtime_migration_retirements",
    ):
        assert store.connection.execute(f"SELECT COUNT(*) FROM {table}").fetchone()[0] == 0


def test_retirement_authority_is_durable_before_unlink_and_resume_is_idempotent(tmp_path, monkeypatch):
    store = _pipette_store(tmp_path)
    row = legacy_pipette_receipt(
        "legacy-retirement-window",
        created_at="2026-08-20T00:00:00Z",
        operation="status",
        channel=0,
    )
    raw = (json.dumps(row, sort_keys=True) + "\n").encode("utf-8")
    store._legacy_path.write_bytes(raw)
    real_unlink = Path.unlink
    authority_seen: list[tuple[str, str]] = []

    def fail_source_unlink(path, *args, **kwargs):
        if path == store._legacy_path:
            retirement = store.connection.execute(
                "SELECT source_digest,archive_relpath FROM runtime_migration_retirements"
            ).fetchone()
            assert retirement is not None
            authority_seen.append((retirement["source_digest"], retirement["archive_relpath"]))
            raise OSError("injected unlink crash window")
        return real_unlink(path, *args, **kwargs)

    monkeypatch.setattr(Path, "unlink", fail_source_unlink)
    with pytest.raises(OSError, match="injected unlink crash window"):
        store.migrate_legacy_jsonl()

    source_sha256 = hashlib.sha256(raw).hexdigest()
    assert authority_seen == [(source_sha256, f"archive/receipts.jsonl.{source_sha256}")]
    assert store._legacy_path.read_bytes() == raw
    assert store.connection.execute("SELECT COUNT(*) FROM pipette_operations").fetchone()[0] == 1
    assert store.connection.execute("SELECT COUNT(*) FROM runtime_migration_receipts").fetchone()[0] == 1
    assert store.connection.execute("SELECT COUNT(*) FROM runtime_migration_retirements").fetchone()[0] == 1

    monkeypatch.setattr(Path, "unlink", real_unlink)
    resumed = store.migrate_legacy_jsonl()
    assert resumed["status"] == "completed"
    assert resumed["imported_count"] == 0
    assert resumed["duplicate_count"] == 1
    assert not store._legacy_path.exists()
    assert store.connection.execute("SELECT COUNT(*) FROM pipette_operations").fetchone()[0] == 1
    assert store.connection.execute("SELECT COUNT(*) FROM runtime_migration_retirements").fetchone()[0] == 1


def test_atomic_migration_resumes_valid_v1_marker_before_retirement(tmp_path):
    store = _pipette_store(tmp_path)
    legacy = legacy_pipette_receipt(
        "legacy-v1-marker",
        created_at="2026-08-20T00:00:00Z",
        operation="status",
        channel=0,
    )
    raw = (json.dumps(legacy, sort_keys=True) + "\n").encode("utf-8")
    store._legacy_path.write_bytes(raw)
    source_sha256 = hashlib.sha256(raw).hexdigest()
    command_id = "legacy.pipette.legacy-v1-marker"
    claim, created = store._audit_database.claim(
        {
            "command_id": command_id,
            "pipette_operation_id": command_id,
            "idempotency_key": "legacy-pipette:legacy-v1-marker",
            "action_id": "pipette.status",
            "operation": "status",
            "entrypoint_id": "migration.pipette_jsonl.v1",
            "caller_class": "legacy_migration",
            "control_class": "historical_import",
            "ownership_generation": 0,
            "connection_generation": 0,
            "source_identity": {
                "legacy_receipt_id": "legacy-v1-marker",
                "source_sha256": source_sha256,
            },
            "requested_inputs": {"channel": 0},
        },
        pipette=True,
    )
    assert created is True
    store._audit_database.finalize_claim(
        command_id=command_id,
        pipette_operation_id=claim["pipette_operation_id"],
        expected_status=claim["status"],
        status="completed",
        outcome="completion",
        failure_code=None,
        result={**legacy["result"], **legacy["truth"]},
        effective_inputs=legacy["effective_inputs"],
        receipt_json=json.dumps(legacy, sort_keys=True),
    )
    archive_relpath = f"archive/receipts.jsonl.{source_sha256}"
    store.connection.execute(
        """
        INSERT INTO runtime_migration_receipts(
            migration_id,source_kind,source_digest,source_count,imported_count,
            quarantined_count,status,archive_relpath,created_at
        ) VALUES(?,?,?,?,?,?,?,?,?)
        """,
        (
            f"pipette-jsonl:{source_sha256}",
            "pipette_receipts_jsonl",
            source_sha256,
            1,
            1,
            0,
            "completed",
            archive_relpath,
            time.time(),
        ),
    )

    resumed = store.migrate_legacy_jsonl()

    assert resumed["status"] == "completed"
    assert resumed["imported_count"] == 0
    assert resumed["duplicate_count"] == 1
    assert not store._legacy_path.exists()
    assert store.connection.execute("SELECT COUNT(*) FROM pipette_operations").fetchone()[0] == 1
    retirement = store.connection.execute(
        "SELECT source_digest,archive_relpath FROM runtime_migration_retirements"
    ).fetchone()
    assert tuple(retirement) == (source_sha256, archive_relpath)


def test_retired_v1_migration_attests_historical_empty_outcome_without_rewrite(tmp_path):
    store = _pipette_store(tmp_path)
    legacy = legacy_pipette_receipt(
        "retired-v1-empty-outcome",
        created_at="2026-08-20T00:00:00Z",
        operation="status",
        channel=0,
    )
    legacy["result"]["outcome"] = ""
    raw = (json.dumps(legacy, sort_keys=True) + "\n").encode("utf-8")
    source_sha256 = hashlib.sha256(raw).hexdigest()
    migration_id = f"pipette-jsonl:{source_sha256}"
    archive_relpath = f"archive/receipts.jsonl.{source_sha256}"
    command_id = "legacy.pipette.retired-v1-empty-outcome"
    source_identity = {
        "legacy_receipt_id": "retired-v1-empty-outcome",
        "source_sha256": source_sha256,
    }
    claim, created = store._audit_database.claim(
        {
            "command_id": command_id,
            "pipette_operation_id": command_id,
            "idempotency_key": "legacy-pipette:retired-v1-empty-outcome",
            "action_id": "pipette.status",
            "operation": "status",
            "entrypoint_id": "migration.pipette_jsonl.v1",
            "caller_class": "legacy_migration",
            "control_class": "historical_import",
            "ownership_generation": 0,
            "connection_generation": 0,
            "source_identity": source_identity,
            "requested_inputs": {"channel": 0},
        },
        pipette=True,
    )
    assert created is True
    store._audit_database.finalize_claim(
        command_id=command_id,
        pipette_operation_id=claim["pipette_operation_id"],
        expected_status=claim["status"],
        status="completed",
        outcome="completed",
        failure_code=None,
        result={**legacy["result"], **legacy["truth"]},
        effective_inputs=legacy["effective_inputs"],
        receipt_json=json.dumps(legacy, sort_keys=True),
    )
    store._write_immutable_migration_artifact(
        store.root / archive_relpath,
        raw,
        expected_sha256=source_sha256,
    )
    backup_root = store.root / "backups" / "pipette-audit-pre-migration-retired-v1"
    backup_root.mkdir(parents=True)
    backup_database = backup_root / "bioxp_runtime.db"
    backup_connection = sqlite3.connect(backup_database)
    store.connection.backup(backup_connection)
    backup_connection.close()
    backup_receipts = backup_root / "receipts.jsonl"
    backup_receipts.write_bytes(raw)
    runtime_backup_alias = Path("/var/lib/bioxp-oem-runtime/backups") / backup_root.name
    sums = (
        f"{hashlib.sha256(backup_database.read_bytes()).hexdigest()}  "
        f"{runtime_backup_alias / backup_database.name}\n"
        f"{source_sha256}  {runtime_backup_alias / backup_receipts.name}\n"
    )
    (backup_root / "SHA256SUMS").write_text(sums, encoding="utf-8")
    backup_relpath = backup_root.relative_to(store.root).as_posix()
    now = time.time()
    store.connection.execute(
        """
        INSERT INTO runtime_migration_receipts(
            migration_id,source_kind,source_digest,source_count,imported_count,
            duplicate_count,quarantined_count,status,archive_relpath,created_at
        ) VALUES(?,?,?,?,?,?,?,?,?,?)
        """,
        (
            migration_id,
            "pipette_receipts_jsonl",
            source_sha256,
            1,
            1,
            0,
            0,
            "completed",
            archive_relpath,
            now,
        ),
    )
    store.connection.execute(
        """
        INSERT INTO runtime_migration_retirements(
            migration_id,source_digest,archive_relpath,retired_at,retirement_sha256
        ) VALUES(?,?,?,?,?)
        """,
        (migration_id, source_sha256, archive_relpath, now, source_sha256),
    )
    store._ensure_migration_evidence(
        migration_id=migration_id,
        source_path=str(store._legacy_path),
        source_digest=source_sha256,
        source_bytes=len(raw),
        source_count=1,
        imported_count=1,
        duplicate_count=0,
        quarantined_count=0,
        backup_relpath=backup_relpath,
        archive_relpath=archive_relpath,
    )

    restarted = store.migrate_legacy_jsonl()

    assert restarted["status"] == "already_imported"
    assert restarted["duplicate_count"] == 1
    assert restarted["quarantined_count"] == 0
    assert (store.root / archive_relpath).read_bytes() == raw
    persisted = store.connection.execute(
        "SELECT outcome,source_identity_json FROM pipette_operations WHERE command_id=?",
        (command_id,),
    ).fetchone()
    assert persisted["outcome"] == "completed"
    assert json.loads(persisted["source_identity_json"]) == source_identity


def test_record_without_typed_ids_uses_sqlite_and_does_not_append_jsonl(tmp_path):
    store = _pipette_store(tmp_path)

    receipt = store.record(
        operation="status",
        requested_inputs={"channel": 0},
        result={"ok": True, "outcome": "completion", "channel": 0},
        runtime_binding={"owner": "legacy_callsite"},
    )

    assert receipt["receipt_id"]
    assert not store._legacy_path.exists()
    callback = store.connection.execute(
        "SELECT callback_session_id FROM pipette_operations"
    ).fetchone()["callback_session_id"]
    assert isinstance(callback, str) and callback.startswith("pipette-callback:")
    assert store.connection.execute("SELECT COUNT(*) FROM pipette_operations").fetchone()[0] == 1


def test_pipette_store_selects_existing_legacy_pipette_layout(tmp_path):
    legacy = tmp_path / "pipette" / "receipts.jsonl"
    legacy.parent.mkdir()
    legacy.write_text("{\"receipt_id\":\"legacy\"}\n", encoding="utf-8")

    store = _pipette_store(tmp_path)

    assert store._legacy_path == legacy


def test_active_jsonl_is_not_a_read_authority(tmp_path):
    store = _pipette_store(tmp_path)
    store._legacy_path.write_text('{"receipt_id":"legacy"}\n', encoding="utf-8")

    with pytest.raises(PipetteReceiptError, match="requires JSONL migration"):
        store.read()
