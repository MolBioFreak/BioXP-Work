from __future__ import annotations

import hashlib
import json
import time

from bioxp.operator_receipt_store import OperatorReceiptStore
from bioxp.pipette.receipts import PipetteReceiptStore


def test_evidence_expiry_is_two_phase_and_retains_digest_metadata(tmp_path):
    store = OperatorReceiptStore(tmp_path)
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

    result = store.expire_evidence("cmd-evidence-1", retention_deadline=0.0, now=1.0)
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


def test_pipette_jsonl_migration_is_digest_bound_and_idempotent(tmp_path):
    store = PipetteReceiptStore(tmp_path)
    legacy_rows = [
        {
            "receipt_id": "legacy-pipette-1",
            "created_at": "2026-08-20T00:00:00Z",
            "operation": "status",
            "requested_inputs": {"channel": 0},
            "effective_inputs": {"channel": 0},
            "result": {"ok": True, "outcome": "completion", "channel": 0},
            "runtime_binding": {"source": "legacy"},
        },
        {
            "receipt_id": "legacy-pipette-2",
            "created_at": "2026-08-20T00:00:01Z",
            "operation": "pressure",
            "requested_inputs": {"channel": 1},
            "effective_inputs": {"channel": 1},
            "result": {"ok": True, "outcome": "completion", "channel": 1},
            "runtime_binding": {"source": "legacy"},
        },
    ]
    raw = ("\n".join(json.dumps(row, sort_keys=True) for row in legacy_rows) + "\n").encode()
    store._legacy_path.write_bytes(raw)

    first = store.migrate_legacy_jsonl()
    assert first["status"] == "completed"
    assert first["imported_count"] == 2
    assert first["source_sha256"] == hashlib.sha256(raw).hexdigest()
    assert store._legacy_path.read_bytes() == raw
    assert store.connection.execute("SELECT COUNT(*) FROM pipette_operations").fetchone()[0] == 2

    second = store.migrate_legacy_jsonl()
    assert second["status"] == "already_imported"
    assert second["imported_count"] == 0
    assert store.connection.execute("SELECT COUNT(*) FROM pipette_operations").fetchone()[0] == 2
    assert store.connection.execute("SELECT COUNT(*) FROM runtime_migration_receipts").fetchone()[0] == 1
