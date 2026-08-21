from __future__ import annotations

import inspect
import json

from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.storage_operations import (
    capacity_report,
    create_backup_unit,
    restore_drill,
    verify_backup_unit,
)


def test_backup_restore_and_capacity_evidence_are_database_and_evidence_bound(tmp_path):
    store = PipetteReceiptStore(tmp_path)
    store.record(
        operation="status",
        requested_inputs={"channel": 0},
        result={"ok": True, "outcome": "completion", "channel": 0},
        runtime_binding={"owner": "storage-test"},
    )
    evidence = tmp_path / "operator_evidence" / "run" / "status.json"
    evidence.parent.mkdir(parents=True)
    evidence.write_text(json.dumps({"status": "read-only"}), encoding="utf-8")

    receipt = create_backup_unit(
        tmp_path,
        label="rw5-test-backup",
        phase="test",
        source_kind="test",
        source_digest="test-digest",
    )

    assert receipt["status"] == "verified"
    unit = tmp_path / "backups" / "rw5-test-backup"
    assert verify_backup_unit(unit)["status"] == "verified"
    restored = restore_drill(unit)
    assert restored["status"] == "verified"
    assert restored["restored_file_count"] == 1

    capacity = capacity_report(tmp_path, allocation_bytes=1024 * 1024 * 1024)
    assert capacity["status"] == "pass"
    assert capacity["observed"]["command_count"] == 1
    assert capacity["five_year_projection"]["total_bytes"] <= capacity["threshold_bytes"]

    store.connection.close()


def test_api_lifespan_runs_pipette_migration_before_normal_startup(monkeypatch, tmp_path):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    from bioxp import api

    source = inspect.getsource(api.lifespan)
    assert "runtime_state_root()" in source
    assert "_pipette_receipts.migrate_legacy_jsonl()" in source
    assert "app.state.pipette_migration" in source
    assert "/tmp/bioxp-oem-runtime" not in source
