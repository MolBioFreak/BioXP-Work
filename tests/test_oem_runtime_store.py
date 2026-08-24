import json
import sqlite3
import threading
import time
from pathlib import Path

import pytest

from src.bioxp.oem_runtime_store import OEMRuntimeStore
from src.bioxp import oem_runtime_store as runtime_store_subject
from src.bioxp.oem_runtime_types import OEMRuntimeSnapshot
from src.bioxp.lifecycle_state import CanonicalLifecycleOwner


def test_runtime_store_writes_state_and_journals_with_sequence(tmp_path, monkeypatch):
    lifecycle = CanonicalLifecycleOwner()
    lifecycle.transition("waiting", reason="test_waiting")
    monkeypatch.setattr("src.bioxp.lifecycle_state.lifecycle_state", lifecycle)

    store = OEMRuntimeStore(tmp_path)
    state = store.write_state(OEMRuntimeSnapshot())
    event = store.append_event({"event_type": "door"})
    hist = store.append_command_history({"command": {"name": "PrepareToRunJob"}})
    assert state["sequence"] < event["sequence"] < hist["sequence"]
    assert state["runtime_state"] == "waiting"
    saved = store.read_state()
    assert saved is not None
    assert saved["runtime_state"] == "waiting"
    assert saved["operation_state"] == "waiting"
    assert store.read_journal("event_journal.jsonl")[0]["event_type"] == "door"


def test_runtime_recovery_flags_active_command(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    payload = OEMRuntimeSnapshot().to_dict()
    payload["worker"]["state"] = "running"
    payload["worker"]["active_command"] = {"name": "PrepareToRunJob"}
    store.write_state(payload)
    recovered = store.recover_state()
    assert recovered["recovery_required"] is True
    assert recovered["recovery"] == "active_command"


def test_serial206_multi_stream_receipts_commit_atomically(tmp_path: Path) -> None:
    store = OEMRuntimeStore(tmp_path)
    stored = store.append_serial206_receipts_atomic((
        ("x", {"command_id": "xyz-current", "receipt_id": "xyz-current", "status": "completed"}),
        ("z", {"command_id": "xyz-current", "receipt_id": "xyz-current", "status": "completed"}),
    ))
    assert len(stored) == 2
    assert store.read_serial206_receipt("x", "xyz-current") is not None
    assert store.read_serial206_receipt("z", "xyz-current") is not None


def test_serial206_multi_stream_receipts_roll_back_if_second_stream_fails(tmp_path: Path) -> None:
    store = OEMRuntimeStore(tmp_path)
    store._db.execute(
        """
        CREATE TRIGGER reject_z_multi_stream_receipt
        BEFORE INSERT ON serial206_receipts
        WHEN NEW.stream='z'
        BEGIN
            SELECT RAISE(ABORT, 'injected z receipt failure');
        END
        """
    )
    with pytest.raises(sqlite3.DatabaseError, match="injected z receipt failure"):
        store.append_serial206_receipts_atomic((
            ("x", {"command_id": "xyz-rollback", "receipt_id": "xyz-rollback", "status": "completed"}),
            ("z", {"command_id": "xyz-rollback", "receipt_id": "xyz-rollback", "status": "completed"}),
        ))
    assert store.read_serial206_receipt("x", "xyz-rollback") is None
    assert store.read_serial206_receipt("z", "xyz-rollback") is None


def test_serial206_receipt_metadata_is_retained_indefinitely_per_stream(tmp_path: Path) -> None:
    store = OEMRuntimeStore(tmp_path)
    for index in range(129):
        store.append_serial206_receipt("z", {
            "command_id": f"z-{index}",
            "idempotency_key": f"key-{index}",
            "status": "completed",
            "finished_at": float(index + 1),
        })

    count = store._db.execute(
        "SELECT COUNT(*) FROM serial206_receipts WHERE stream='z'"
    ).fetchone()[0]
    assert count == 129
    assert store.read_serial206_receipt("z", "z-0") is not None
    assert store.read_serial206_receipt("z", "z-128") is not None


def test_serial206_insert_does_not_prune_metadata(tmp_path: Path) -> None:
    store = OEMRuntimeStore(tmp_path)
    for index in range(128):
        store.append_serial206_receipt("z", {
            "command_id": f"atomic-{index}",
            "idempotency_key": f"atomic-key-{index}",
            "status": "completed",
            "finished_at": float(index + 1),
        })
    store._db.execute(
        """
        CREATE TRIGGER reject_serial206_prune
        BEFORE DELETE ON serial206_receipts
        BEGIN
            SELECT RAISE(ABORT, 'injected prune failure');
        END
        """
    )

    store.append_serial206_receipt("z", {
        "command_id": "atomic-128",
        "idempotency_key": "atomic-key-128",
        "status": "completed",
        "finished_at": 129.0,
    })

    assert store._db.execute(
        "SELECT COUNT(*) FROM serial206_receipts WHERE stream='z'"
    ).fetchone()[0] == 129
    assert store.read_serial206_receipt("z", "atomic-128") is not None


def test_serial206_nonreplayable_interrupt_keys_are_not_unique_or_queryable(tmp_path: Path) -> None:
    store = OEMRuntimeStore(tmp_path)
    for command_id, intent in (("stop-1", "stop"), ("stop-2", "stop"), ("abort-1", "abort")):
        stored = store.append_serial206_receipt("z", {
            "command_id": command_id,
            "idempotency_key": "repeated-interrupt-key",
            "idempotency_replay_enabled": False,
            "intent": intent,
            "status": "completed",
            "finished_at": float(len(command_id)),
        })
        assert stored["idempotency_replay_enabled"] is False

    assert store._db.execute(
        "SELECT COUNT(*) FROM serial206_receipts WHERE stream='z' AND idempotency_key=?",
        ("repeated-interrupt-key",),
    ).fetchone()[0] == 3
    assert store.read_serial206_receipt_by_idempotency("z", "repeated-interrupt-key") is None


def test_serial206_current_state_keeps_one_receipt_and_sql_keeps_all(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    receipts = [
        {
            "command_id": f"z-{index}",
            "idempotency_key": f"z-key-{index}",
            "status": "completed",
            "finished_at": float(index + 1),
        }
        for index in range(3)
    ]
    for row in receipts:
        store.append_serial206_receipt("z", row)
    state = {
        "movement_ledger": {},
        "used_approvals": {},
        "initialize_motion_ledger": {},
        "z_lifecycle": {"receipts": receipts},
        "x_lifecycle": {"receipts": []},
    }

    returned = store.write_oem_serial206_initialization_state(state)
    stored = store.read_oem_serial206_initialization_state()

    assert returned["z_lifecycle"]["receipts"] == receipts
    assert stored["z_lifecycle"]["receipts"] == [receipts[-1]]
    assert stored["z_lifecycle"]["receipts_omitted_to_sqlite"] == 2
    assert len(store.list_serial206_receipts("z")) == 3
