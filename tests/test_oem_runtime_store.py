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


def test_serial206_embedded_receipts_import_once_and_remain_queryable(tmp_path):
    source = {
        "z_lifecycle": {
            "receipts": [
                {
                    "command_id": "z-old",
                    "idempotency_key": "z-key-old",
                    "status": "completed",
                    "finished_at": 9.0,
                },
                {
                    "command_id": "z-new",
                    "idempotency_key": "z-key-new",
                    "status": "completed",
                    "finished_at": 10.0,
                },
            ]
        },
        "x_lifecycle": {"receipts": []},
    }
    state_path = tmp_path / "serial206_oem_initialization_state.json"
    state_path.write_text(json.dumps(source), encoding="utf-8")
    original = state_path.read_bytes()

    store = OEMRuntimeStore(tmp_path)

    assert state_path.read_bytes() == original
    assert store.read_serial206_receipt("z", "z-old")["idempotency_key"] == "z-key-old"
    assert store.read_serial206_receipt_by_idempotency("z", "z-key-new")["command_id"] == "z-new"
    assert [row["command_id"] for row in store.list_serial206_receipts("z")] == ["z-old", "z-new"]
    marker = json.loads(
        store._db.execute(
            "SELECT value FROM runtime_metadata WHERE key='serial206_embedded_receipt_import_v1'"
        ).fetchone()[0]
    )
    assert marker["source_retained"] is True
    assert marker["imported"] == 2


def test_serial206_embedded_compact_placeholders_stay_in_json_not_sql_indexes(tmp_path):
    compact_receipt = {
        "command_id": "z-compact",
        "idempotency_key": {"omitted": "item_limit"},
        "intent": "stop",
        "status": {"omitted": "item_limit"},
        "finished_at": {"omitted": "item_limit"},
    }
    (tmp_path / "serial206_oem_initialization_state.json").write_text(
        json.dumps({"z_lifecycle": {"receipts": [compact_receipt]}}),
        encoding="utf-8",
    )

    store = OEMRuntimeStore(tmp_path)

    stored = store.read_serial206_receipt("z", "z-compact")
    assert stored is not None
    assert stored["idempotency_key"] == {"omitted": "item_limit"}
    assert stored["status"] == {"omitted": "item_limit"}
    indexed = store._db.execute(
        """
        SELECT idempotency_key,idempotency_replay_enabled,status,observed_at
        FROM serial206_receipts WHERE stream='z' AND command_id='z-compact'
        """
    ).fetchone()
    assert indexed["idempotency_key"] is None
    assert indexed["idempotency_replay_enabled"] == 0
    assert indexed["status"] is None
    assert isinstance(indexed["observed_at"], float)


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


def test_serial206_interrupt_uses_zero_wait_fallback_and_imports_on_restart(tmp_path: Path) -> None:
    store = OEMRuntimeStore(tmp_path)
    blocker = sqlite3.connect(tmp_path / "bioxp_runtime.db", isolation_level=None)
    blocker.execute("BEGIN IMMEDIATE")
    receipt = {
        "command_id": "z-stop-fallback",
        "idempotency_key": "repeated-stop-key",
        "idempotency_replay_enabled": False,
        "intent": "stop",
        "status": "completed",
        "finished_at": 10.0,
    }

    started = time.perf_counter()
    stored = store.append_serial206_interrupt_receipt("z", receipt)
    elapsed = time.perf_counter() - started
    assert elapsed < 0.5
    assert stored["persistence_fallback"]["kind"] == "serial206_interrupt_jsonl"
    assert store.serial206_interrupt_fallback_path.exists()
    assert store.read_serial206_receipt("z", "z-stop-fallback") is None

    blocker.execute("ROLLBACK")
    blocker.close()
    restarted = OEMRuntimeStore(tmp_path)
    imported = restarted.read_serial206_receipt("z", "z-stop-fallback")
    assert imported is not None
    assert imported["persistence_fallback"]["kind"] == "serial206_interrupt_jsonl"
    assert not restarted.serial206_interrupt_fallback_path.exists()
    assert list(tmp_path.glob("serial206_interrupt_fallback.imported.*.jsonl"))


def test_serial206_fallback_rotation_does_not_lose_concurrent_append(tmp_path: Path, monkeypatch) -> None:
    first = OEMRuntimeStore(tmp_path)
    before = {"command_id": "serial-before", "intent": "stop", "status": "completed"}
    during = {"command_id": "serial-during", "intent": "stop", "status": "completed"}
    first.append_serial206_interrupt_fallback("x", before, reason="test")
    original_replace = runtime_store_subject.os.replace
    writer_threads: list[threading.Thread] = []
    triggered = False

    def replace_with_concurrent_writer(source, destination):
        nonlocal triggered
        if Path(source) == first.serial206_interrupt_fallback_path and ".pending." in Path(destination).name and not triggered:
            triggered = True
            thread = threading.Thread(
                target=lambda: first.append_serial206_interrupt_fallback("x", during, reason="test"),
                daemon=True,
            )
            writer_threads.append(thread)
            thread.start()
            time.sleep(0.02)
            assert thread.is_alive()
        return original_replace(source, destination)

    monkeypatch.setattr(runtime_store_subject.os, "replace", replace_with_concurrent_writer)
    second = OEMRuntimeStore(tmp_path)
    writer_threads[0].join(timeout=2)
    assert not writer_threads[0].is_alive()
    assert second.read_serial206_receipt("x", "serial-before") is not None
    assert second.read_serial206_receipt("x", "serial-during") is None

    third = OEMRuntimeStore(tmp_path)
    assert third.read_serial206_receipt("x", "serial-during") is not None


def test_serial206_archive_cleanup_fails_closed_on_unlink_error(tmp_path: Path, monkeypatch) -> None:
    for index in range(10):
        (tmp_path / f"serial206_interrupt_fallback.imported.{index}.jsonl").write_text("", encoding="utf-8")
    original_unlink = Path.unlink

    def fail_stale_unlink(path, *args, **kwargs):
        if path.name.startswith("serial206_interrupt_fallback.imported."):
            raise OSError("injected archive unlink failure")
        return original_unlink(path, *args, **kwargs)

    monkeypatch.setattr(Path, "unlink", fail_stale_unlink)
    with pytest.raises(OSError, match="injected archive unlink failure"):
        OEMRuntimeStore(tmp_path)


def test_legacy_nonreplayable_receipts_with_reused_command_id_import_separately(tmp_path):
    legacy = {
        "z_lifecycle": {
            "receipts": [
                {
                    "command_id": "legacy-reused-stop",
                    "intent": "stop",
                    "idempotency_replay_enabled": False,
                    "status": "completed",
                    "started_at": 1.0,
                },
                {
                    "command_id": "legacy-reused-stop",
                    "intent": "stop",
                    "idempotency_replay_enabled": False,
                    "status": "completed",
                    "started_at": 2.0,
                },
            ]
        },
        "x_lifecycle": {"receipts": []},
    }
    (tmp_path / "serial206_oem_initialization_state.json").write_text(json.dumps(legacy))

    store = OEMRuntimeStore(tmp_path)
    rows = [
        row for row in store.list_serial206_receipts("z", 10)
        if row.get("command_id") == "legacy-reused-stop"
    ]

    assert len(rows) == 2
    assert len({row["receipt_id"] for row in rows}) == 2


def test_serial206_embedded_receipt_corruption_fails_without_marker(tmp_path):
    state_path = tmp_path / "serial206_oem_initialization_state.json"
    state_path.write_text("{broken", encoding="utf-8")

    with pytest.raises(RuntimeError, match="embedded serial-206 receipt import failed"):
        OEMRuntimeStore(tmp_path)

    import sqlite3

    connection = sqlite3.connect(tmp_path / "bioxp_runtime.db")
    assert connection.execute(
        "SELECT COUNT(*) FROM runtime_metadata WHERE key='serial206_embedded_receipt_import_v1'"
    ).fetchone()[0] == 0


def test_serial206_terminal_update_replaces_row_and_numeric_time_orders_correctly(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    store.append_serial206_receipt(
        "z",
        {
            "command_id": "z-command",
            "idempotency_key": "z-key",
            "status": "queued",
            "started_at": 9.0,
        },
    )
    store.append_serial206_receipt(
        "z",
        {
            "command_id": "z-command",
            "idempotency_key": "z-key",
            "status": "completed",
            "started_at": 9.0,
            "finished_at": 10.0,
            "terminal_state": {"position_steps": 0, "speed_steps_s": 0},
        },
    )
    store.append_serial206_receipt(
        "z",
        {
            "command_id": "z-earlier",
            "idempotency_key": "z-key-earlier",
            "status": "completed",
            "finished_at": 9.5,
        },
    )

    rows = store.list_serial206_receipts("z")
    assert [row["command_id"] for row in rows] == ["z-earlier", "z-command"]
    assert store.read_serial206_receipt("z", "z-command")["status"] == "completed"
    assert store._db.execute(
        "SELECT COUNT(*) FROM serial206_receipts WHERE stream='z' AND command_id='z-command'"
    ).fetchone()[0] == 1


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
