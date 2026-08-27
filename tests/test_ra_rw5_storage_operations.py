from __future__ import annotations

import inspect
import json
import shutil
import time

import pytest
from bioxp.operator_receipt_store import OperatorReceiptStore
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.storage_operations import (
    StorageEvidenceError,
    audit_health_report,
    capacity_report,
    checkpoint_database,
    create_backup_unit,
    restore_drill,
    verify_backup_unit,
)


def _operator_store(root) -> OperatorReceiptStore:
    owner = OEMRuntimeStore(root)
    owner.close()
    return OperatorReceiptStore(root)


def _operator_receipt(command_id: str) -> dict:
    return {
        "command_id": command_id,
        "idempotency_key": f"idempotency-{command_id}",
        "action_id": "query.status",
        "status": "completed",
        "started_at": str(time.time()),
        "finished_at": str(time.time()),
        "ownership_generation": 1,
        "controller_acknowledged": False,
        "physical_effect_verified": False,
        "response": {"status": "read-only", "payload": "evidence"},
        "stage_receipts": [],
    }


def test_backup_publishes_new_sealed_units_and_source_only_isolated_restore(tmp_path):
    store = _operator_store(tmp_path)
    stored = store.put(_operator_receipt("storage-command-1"))
    assert stored["evidence_artifact_id"]

    first = create_backup_unit(
        tmp_path,
        label="rw5-test-backup",
        phase="test",
        source_kind="test",
        source_digest="test-digest",
    )
    second = create_backup_unit(
        tmp_path,
        label="rw5-test-backup",
        phase="test",
        source_kind="test",
        source_digest="test-digest",
    )

    assert first["status"] == "verified"
    assert first["backup_id"] != second["backup_id"]
    first_unit = tmp_path / first["unit_relpath"]
    second_unit = tmp_path / second["unit_relpath"]
    assert first_unit.is_dir()
    assert second_unit.is_dir()
    assert not list((tmp_path / "backups").glob(".*.staging"))
    assert not (first_unit / "backup-receipt.json").exists()
    external_receipt = tmp_path / "backups" / "receipts" / f"{first['backup_id']}.json"
    assert json.loads(external_receipt.read_text(encoding="utf-8"))["manifest_sha256"] == first["manifest_sha256"]

    verified = verify_backup_unit(first_unit)
    assert verified["status"] == "source_verified"
    restored = restore_drill(first_unit)
    assert restored["status"] == "source_verified"
    assert restored["closure"] == "source_only"
    assert restored["execution_performed"] is False
    assert restored["service_started"] is False
    assert restored["hardware_touched"] is False
    assert restored["production_database_modified"] is False
    assert (tmp_path / restored["target_relpath"] / "bioxp_runtime.db").is_file()

    checkpoint = checkpoint_database(tmp_path / "bioxp_runtime.db")
    assert checkpoint["status"] == "verified"
    capacity = capacity_report(tmp_path, allocation_bytes=1024 * 1024 * 1024)
    assert capacity["status"] == "insufficient_evidence"
    assert capacity["representative_sample"] is False
    assert "command_sample_below_100" in capacity["insufficient_evidence_reasons"]

    corrupt = tmp_path / "backups" / "rw5-corrupt"
    shutil.copytree(first_unit, corrupt)
    corrupt_db = corrupt / "bioxp_runtime.db"
    corrupt_db.write_bytes(corrupt_db.read_bytes() + b"corrupt")
    with pytest.raises(StorageEvidenceError):
        verify_backup_unit(corrupt)

    store.connection.close()


def test_backup_fails_on_unbound_or_missing_evidence_instead_of_trusting_a_list(tmp_path):
    store = _operator_store(tmp_path)
    stored = store.put(_operator_receipt("storage-command-closure"))
    unbound = tmp_path / "operator_evidence" / "unbound.json"
    unbound.write_text("{}", encoding="utf-8")

    with pytest.raises(StorageEvidenceError, match="closure mismatch"):
        create_backup_unit(tmp_path, label="unbound", phase="test")

    unbound.unlink()
    evidence = tmp_path / stored["evidence_relpath"]
    evidence.unlink()
    with pytest.raises(StorageEvidenceError, match="closure mismatch"):
        create_backup_unit(tmp_path, label="missing", phase="test")


def test_backup_requires_exact_schema_migration_and_trigger_identity(tmp_path):
    store = _operator_store(tmp_path)
    store.put(_operator_receipt("storage-command-schema"))
    store.connection.execute("DROP TRIGGER runtime_events_append_only_delete")

    with pytest.raises(StorageEvidenceError, match="schema/migration/trigger"):
        create_backup_unit(tmp_path, label="schema-mismatch", phase="test")


def test_health_returns_degraded_without_checkpoint_backup_or_representative_writer_receipts(tmp_path):
    store = _operator_store(tmp_path)
    health = audit_health_report(
        tmp_path,
        connection=store.connection,
        writer_health={"status": "ok", "queue_depth": 0},
    )

    assert health["status"] == "degraded"
    assert "checkpoint_freshness_or_identity" in health["degraded_reasons"]
    assert "backup_freshness_or_integrity" in health["degraded_reasons"]
    assert health["checks"]["schema"]["status"] == "ok"
    assert health["checks"]["integrity"]["status"] == "ok"
    assert health["checks"]["foreign_keys"]["status"] == "ok"
    assert health["physical_admission_gate_added"] is False


def test_api_lifespan_runs_pipette_migration_before_normal_startup(monkeypatch, tmp_path):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    from bioxp import api

    source = inspect.getsource(api.lifespan)
    assert "runtime_state_root()" in source
    assert "migrate_runtime_database_v2(migration_connection, runtime_root)" in source
    assert "_pipette_receipts.migrate_legacy_jsonl()" in source
    assert "_pipette_receipts.reconcile_nonterminal_claims()" in source
    assert "app.state.pipette_migration" in source
    assert "app.state.runtime_reconciliation" in source
    assert "/tmp/bioxp-oem-runtime" not in source
