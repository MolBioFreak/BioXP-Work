from __future__ import annotations

import json
import sqlite3
import threading
import time
from pathlib import Path

import pytest

from bioxp.operator_receipt_store import OperatorReceiptStore, runtime_state_root
from bioxp.oem_runtime_store import OEMRuntimeStore


def _operator_store(root: Path) -> OperatorReceiptStore:
    owner = OEMRuntimeStore(root)
    owner.close()
    store = OperatorReceiptStore(root)
    store.converge_startup_state()
    return store


def receipt(
    command_id: str,
    *,
    key: str | None = None,
    status: str = "completed",
    started_at: str = "100.0",
    response: object | None = None,
) -> dict:
    return {
        "schema_version": "bioxp.operator_action_receipt.v1",
        "command_id": command_id,
        "idempotency_key": key or f"key-{command_id}",
        "action_id": "query.status",
        "status": status,
        "ownership_generation": 7,
        "started_at": started_at,
        "finished_at": "101.0" if status == "completed" else None,
        "controller_acknowledged": False,
        "physical_effect_verified": False,
        "response": response,
        "stage_receipts": [],
    }


def test_runtime_root_rejects_conflicting_state_roots(tmp_path, monkeypatch):
    canonical = tmp_path / "canonical"
    legacy = tmp_path / "legacy"
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(canonical))
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(legacy))

    with pytest.raises(ValueError, match="conflicting BioXP runtime roots"):
        runtime_state_root()


def test_legacy_import_is_transactional_compact_and_keeps_source(tmp_path):
    large_body = {"payload": "x" * 100_000, "nested": {"terminal": True}}
    legacy_path = tmp_path / "operator_action_receipts.json"
    legacy_path.write_text(
        json.dumps(
            {
                "receipts": [
                    receipt("legacy-1", response={"http_status": 200, "body": large_body}),
                    receipt("legacy-2", response={"http_status": 200, "body": {"ok": True}}),
                ]
            }
        ),
        encoding="utf-8",
    )
    original = legacy_path.read_bytes()

    store = _operator_store(tmp_path)

    assert legacy_path.read_bytes() == original
    assert store.connection.execute("PRAGMA journal_mode").fetchone()[0] == "wal"
    assert store.connection.execute("PRAGMA foreign_keys").fetchone()[0] == 1
    assert store.connection.execute("PRAGMA synchronous").fetchone()[0] == 2
    assert store.connection.execute("PRAGMA foreign_key_check").fetchall() == []
    assert store.connection.execute("SELECT COUNT(*) FROM operator_commands").fetchone()[0] == 2
    compact = store.by_command("legacy-1", include_evidence=False)
    assert compact is not None
    assert compact["response"] == {"http_status": 200, "body": {}}
    assert compact["stage_receipts"] == []
    assert store.connection.execute("SELECT COUNT(*) FROM runtime_evidence_objects").fetchone()[0] == 2
    detailed = store.by_command("legacy-1", include_evidence=True)
    assert detailed is not None
    assert detailed["response"]["body"] == large_body
    marker = json.loads(
        store.connection.execute(
            "SELECT value FROM runtime_metadata WHERE key='operator_receipt_legacy_import_v1'"
        ).fetchone()[0]
    )
    assert marker["source_retained"] is True
    assert marker["imported_receipts"] == 2


def test_malformed_legacy_import_fails_without_success_marker(tmp_path):
    legacy_path = tmp_path / "operator_action_receipts.json"
    legacy_path.write_text("{broken", encoding="utf-8")

    with pytest.raises(RuntimeError, match="legacy operator receipt import failed"):
        _operator_store(tmp_path)

    import sqlite3

    connection = sqlite3.connect(tmp_path / "bioxp_runtime.db")
    assert connection.execute(
        "SELECT COUNT(*) FROM runtime_metadata WHERE key='operator_receipt_legacy_import_v1'"
    ).fetchone()[0] == 0
    assert legacy_path.read_text(encoding="utf-8") == "{broken"


def test_atomic_claim_allows_one_cross_connection_owner(tmp_path):
    first = _operator_store(tmp_path)
    second = _operator_store(tmp_path)
    barrier = threading.Barrier(2)
    outcomes: list[tuple[str, bool, str]] = []

    def claim(store: OperatorReceiptStore, command_id: str) -> None:
        barrier.wait(timeout=3)
        row, created = store.claim(receipt(command_id, key="shared-key", status="queued"))
        outcomes.append((command_id, created, row["command_id"]))

    threads = [
        threading.Thread(target=claim, args=(first, "claim-a")),
        threading.Thread(target=claim, args=(second, "claim-b")),
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=5)

    assert all(not thread.is_alive() for thread in threads)
    assert sorted(created for _, created, _ in outcomes) == [False, True]
    winning_ids = {returned_id for _, _, returned_id in outcomes}
    assert len(winning_ids) == 1
    assert first.connection.execute(
        "SELECT COUNT(*) FROM operator_commands WHERE idempotency_key='shared-key'"
    ).fetchone()[0] == 1


def test_terminal_update_moves_command_to_newest_and_keeps_transitions(tmp_path):
    store = _operator_store(tmp_path)
    store.claim(receipt("first", status="queued"))
    store.claim(receipt("second", status="queued"))
    store.put(
        receipt("first", status="completed", response={"http_status": 200, "body": {"ok": True}}),
        _expected_status="reserved",
    )

    assert store.list()[0]["command_id"] == "first"
    transitions = store.connection.execute(
        "SELECT state FROM operator_transitions WHERE command_id='first' ORDER BY transition_id"
    ).fetchall()
    assert [row[0] for row in transitions] == ["reserved", "completed"]


@pytest.mark.parametrize(
    "terminal_status",
    [
        "completed",
        "failed",
        "rejected",
        "ambiguous",
        "outcome_unknown",
        "cancelled",
        "reconciliation_required",
    ],
)
def test_ordinary_put_cannot_mutate_any_terminal_receipt(tmp_path, terminal_status):
    store = _operator_store(tmp_path)
    command_id = f"terminal-{terminal_status}"
    store.put(
        receipt(
            command_id,
            status=terminal_status,
            response={"http_status": 200, "body": {"value": "first"}},
        )
    )
    before = store.connection.execute(
        "SELECT status,receipt_json,response_summary_json,updated_at FROM operator_commands WHERE command_id=?",
        (command_id,),
    ).fetchone()

    with pytest.raises(RuntimeError, match="cannot mutate terminal state"):
        store.put(
            receipt(
                command_id,
                status=terminal_status,
                response={"http_status": 200, "body": {"value": "stale"}},
            ),
            _expected_status=terminal_status,
        )

    after = store.connection.execute(
        "SELECT status,receipt_json,response_summary_json,updated_at FROM operator_commands WHERE command_id=?",
        (command_id,),
    ).fetchone()
    assert tuple(after) == tuple(before)


def test_retention_keeps_all_command_metadata_and_evidence_until_lifecycle_expiry(tmp_path: Path) -> None:
    store = _operator_store(tmp_path)
    first_evidence: Path | None = None

    for index in range(513):
        store.put(receipt(
            f"bounded-{index}",
            status="completed",
            started_at=str(index),
            response={"http_status": 200, "body": {"ok": True, "index": index}},
        ))
        if index == 0:
            selected = store.connection.execute(
                "SELECT evidence_relpath FROM operator_commands WHERE command_id='bounded-0'"
            ).fetchone()
            first_evidence = tmp_path / selected[0]

    assert store.connection.execute("SELECT COUNT(*) FROM operator_commands").fetchone()[0] == 513
    assert store.by_command("bounded-0") is not None
    assert first_evidence is not None
    assert first_evidence.exists()
    assert store.list(200)[0]["command_id"] == "bounded-512"


def test_interrupt_receipt_uses_zero_wait_fallback_and_imports_on_restart(tmp_path: Path) -> None:
    store = _operator_store(tmp_path)
    blocker = sqlite3.connect(tmp_path / "bioxp_runtime.db", isolation_level=None)
    blocker.execute("BEGIN IMMEDIATE")
    interrupt = receipt(
        "stop-fallback",
        key="same-stop-key",
        status="completed",
        response={"http_status": 200, "body": {"ok": True}},
    )
    interrupt.update({
        "action_id": "oem.z.stop",
        "safety_class": "stop",
        "idempotency_replay_enabled": False,
    })

    started = time.perf_counter()
    stored = store.put_interrupt(interrupt)
    elapsed = time.perf_counter() - started
    assert elapsed < 0.5
    assert stored["persistence_fallback"]["kind"] == "operator_interrupt_jsonl"
    assert store.interrupt_fallback_path.exists()
    assert store.by_command("stop-fallback") is None

    blocker.execute("ROLLBACK")
    blocker.close()
    restarted = _operator_store(tmp_path)
    imported = restarted.by_command("stop-fallback")
    assert imported is not None
    assert imported["status"] == "completed"
    assert imported["persistence_fallback"]["kind"] == "operator_interrupt_jsonl"
    assert not restarted.interrupt_fallback_path.exists()
    assert list(tmp_path.glob("operator_interrupt_fallback.imported.*.jsonl"))


def test_failed_sql_update_cannot_mutate_committed_evidence_and_restart_removes_orphan(
    tmp_path: Path,
    monkeypatch,
) -> None:
    store = _operator_store(tmp_path)
    store.put(receipt("immutable", response={"http_status": 200, "body": {"value": "first"}}))
    committed = store.connection.execute(
        "SELECT evidence_relpath,evidence_sha256 FROM operator_commands WHERE command_id='immutable'"
    ).fetchone()
    committed_path = tmp_path / committed["evidence_relpath"]
    committed_bytes = committed_path.read_bytes()
    original_upsert = store._upsert

    def fail_upsert(*_args, **_kwargs):
        raise sqlite3.OperationalError("injected rollback")

    monkeypatch.setattr(store, "_upsert", fail_upsert)
    with pytest.raises(sqlite3.OperationalError, match="injected rollback"):
        store.put(receipt("immutable", response={"http_status": 200, "body": {"value": "second"}}))

    monkeypatch.setattr(store, "_upsert", original_upsert)
    detailed = store.by_command("immutable", include_evidence=True)
    assert detailed is not None
    assert detailed["response"]["body"]["value"] == "first"
    assert committed_path.read_bytes() == committed_bytes
    assert len(list(store.evidence_root.rglob("immutable.*.json"))) == 2

    store.connection.close()
    restarted = _operator_store(tmp_path)
    assert len(list(restarted.evidence_root.rglob("immutable.*.json"))) == 1
    assert restarted.by_command("immutable", include_evidence=True)["response"]["body"]["value"] == "first"
    orphan = restarted.connection.execute(
        """
        SELECT evidence_artifact_id,active_relpath,expiry_state,sha256,byte_count
        FROM runtime_evidence_objects WHERE evidence_artifact_id LIKE 'orphan:%'
        """
    ).fetchone()
    assert orphan is not None
    assert orphan["active_relpath"] is None
    assert orphan["expiry_state"] == "orphan_cleaned"
    orphan_events = [
        row[0]
        for row in restarted.connection.execute(
            "SELECT event_kind FROM runtime_evidence_events WHERE evidence_artifact_id=? ORDER BY event_id",
            (orphan["evidence_artifact_id"],),
        ).fetchall()
    ]
    assert orphan_events == ["orphan_classified", "orphan_deleted", "orphan_cleaned"]


def test_interrupt_evidence_write_failure_uses_fsynced_fallback(tmp_path: Path, monkeypatch) -> None:
    store = _operator_store(tmp_path)
    interrupt = receipt(
        "stop-evidence-fallback",
        key="reused-stop-key",
        response={"http_status": 200, "body": {"ok": True}},
    )
    interrupt.update({"action_id": "oem.z.stop", "idempotency_replay_enabled": False})
    monkeypatch.setattr(
        store,
        "_persist_evidence",
        lambda _receipt: (_ for _ in ()).throw(OSError("evidence disk unavailable")),
    )

    stored = store.put_interrupt(interrupt)

    assert stored["persistence_fallback"]["kind"] == "operator_interrupt_jsonl"
    assert "evidence disk unavailable" in stored["persistence_fallback"]["reason"]
    assert store.interrupt_fallback_path.exists()


def test_existing_database_migrates_replay_column_before_replacing_unique_index(
    tmp_path: Path,
) -> None:
    database = sqlite3.connect(tmp_path / "bioxp_runtime.db")
    database.executescript(
        """
        CREATE TABLE operator_commands (
            sequence INTEGER PRIMARY KEY AUTOINCREMENT,
            command_id TEXT NOT NULL UNIQUE,
            idempotency_key TEXT NOT NULL,
            action_id TEXT NOT NULL,
            status TEXT NOT NULL,
            safety_class TEXT,
            ownership_generation INTEGER NOT NULL,
            started_at TEXT NOT NULL,
            finished_at TEXT,
            duration_ms REAL,
            controller_acknowledged INTEGER NOT NULL DEFAULT 0,
            physical_effect_verified INTEGER NOT NULL DEFAULT 0,
            receipt_json TEXT NOT NULL,
            response_summary_json TEXT,
            evidence_relpath TEXT,
            evidence_sha256 TEXT,
            evidence_bytes INTEGER,
            updated_at REAL NOT NULL
        );
        CREATE UNIQUE INDEX operator_commands_idempotency_key_idx
            ON operator_commands(idempotency_key);
        """
    )
    actions = (
        "oem.z.stop",
        "oem.z.abort",
        "oem.x.stop",
        "oem.abort_all",
        "meta.emergency_stop",
        "query.status",
    )
    for index, action_id in enumerate(actions):
        selected = receipt(f"existing-{index}", key=f"existing-key-{index}")
        selected["action_id"] = action_id
        database.execute(
            """
            INSERT INTO operator_commands(
                command_id,idempotency_key,action_id,status,safety_class,
                ownership_generation,started_at,finished_at,duration_ms,
                controller_acknowledged,physical_effect_verified,receipt_json,
                response_summary_json,evidence_relpath,evidence_sha256,evidence_bytes,updated_at
            ) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
            """,
            (
                selected["command_id"],
                selected["idempotency_key"],
                action_id,
                selected["status"],
                None,
                selected["ownership_generation"],
                selected["started_at"],
                selected["finished_at"],
                None,
                0,
                0,
                json.dumps(selected),
                None,
                None,
                None,
                None,
                float(index),
            ),
        )
    database.commit()
    database.close()

    store = _operator_store(tmp_path)

    columns = {
        row["name"]: row
        for row in store.connection.execute("PRAGMA table_info(operator_commands)")
    }
    assert columns["idempotency_replay_enabled"]["notnull"] == 1
    assert columns["idempotency_replay_enabled"]["dflt_value"] == "1"
    replay_by_action = dict(store.connection.execute(
        "SELECT action_id,idempotency_replay_enabled FROM operator_commands"
    ).fetchall())
    assert replay_by_action == {
        "oem.z.stop": 0,
        "oem.z.abort": 0,
        "oem.x.stop": 0,
        "oem.abort_all": 0,
        "meta.emergency_stop": 0,
        "query.status": 1,
    }
    index_sql = store.connection.execute(
        "SELECT sql FROM sqlite_master WHERE name='operator_commands_replay_key_idx'"
    ).fetchone()[0]
    assert "WHERE idempotency_replay_enabled=1" in index_sql

    repeated = receipt("repeated-x-stop", key="existing-key-2")
    repeated["action_id"] = "oem.x.stop"
    store.put_interrupt(repeated)
    assert store.connection.execute(
        "SELECT COUNT(*) FROM operator_commands WHERE idempotency_key='existing-key-2'"
    ).fetchone()[0] == 2

    store.connection.close()
    restarted = _operator_store(tmp_path)
    assert restarted.connection.execute(
        "SELECT COUNT(*) FROM operator_commands WHERE idempotency_key='existing-key-2'"
    ).fetchone()[0] == 2
    assert restarted.connection.execute(
        "SELECT idempotency_replay_enabled FROM operator_commands WHERE command_id='repeated-x-stop'"
    ).fetchone()[0] == 0


def test_legacy_repeated_safety_keys_import_as_nonreplayable(tmp_path: Path) -> None:
    first = receipt("legacy-stop-1", key="same-stop-key")
    first["action_id"] = "oem.z.stop"
    first.pop("idempotency_replay_enabled", None)
    second = receipt("legacy-stop-2", key="same-stop-key")
    second["action_id"] = "oem.z.stop"
    second.pop("idempotency_replay_enabled", None)
    (tmp_path / "operator_action_receipts.json").write_text(
        json.dumps({"receipts": [first, second]}),
        encoding="utf-8",
    )

    store = _operator_store(tmp_path)

    assert store.connection.execute(
        "SELECT COUNT(*) FROM operator_commands WHERE idempotency_key='same-stop-key'"
    ).fetchone()[0] == 2
    assert store.by_idempotency("same-stop-key") is None
    assert store.by_command("legacy-stop-1")["idempotency_replay_enabled"] is False


def test_startup_gc_waits_for_inflight_evidence_publication(tmp_path: Path, monkeypatch) -> None:
    first = _operator_store(tmp_path)
    created = threading.Event()
    release = threading.Event()
    real_persist = first._persist_evidence

    def paused_persist(selected_receipt):
        evidence = real_persist(selected_receipt)
        created.set()
        assert release.wait(timeout=2)
        return evidence

    monkeypatch.setattr(first, "_persist_evidence", paused_persist)
    row = receipt(
        "concurrent-evidence",
        response={"http_status": 200, "body": {"payload": "x" * 20_000}},
    )
    put_thread = threading.Thread(target=lambda: first.put(row), daemon=True)
    put_thread.start()
    assert created.wait(timeout=2)

    opened: list[OperatorReceiptStore] = []
    startup_thread = threading.Thread(
        target=lambda: opened.append(_operator_store(tmp_path)),
        daemon=True,
    )
    startup_thread.start()
    time.sleep(0.05)
    assert startup_thread.is_alive()
    release.set()
    put_thread.join(timeout=2)
    startup_thread.join(timeout=2)
    assert not put_thread.is_alive()
    assert not startup_thread.is_alive()

    detailed = opened[0].by_command("concurrent-evidence", include_evidence=True)
    assert detailed is not None
    assert detailed["response"]["body"]["payload"] == "x" * 20_000


def test_delayed_cleanup_preserves_evidence_reused_by_later_commit(tmp_path: Path, monkeypatch) -> None:
    first = _operator_store(tmp_path)
    second = _operator_store(tmp_path)
    original = receipt(
        "reuse-evidence",
        response={"http_status": 200, "body": {"payload": "x" * 20_000}},
    )
    first.put(original)
    original_path = next(first.evidence_root.rglob("reuse-evidence.*.json"))
    cleanup_started = threading.Event()
    release_cleanup = threading.Event()
    real_cleanup = first._remove_pruned_evidence

    def delayed_cleanup(relpaths, *, nonblocking=False):
        cleanup_started.set()
        assert release_cleanup.wait(timeout=2)
        return real_cleanup(relpaths, nonblocking=nonblocking)

    monkeypatch.setattr(first, "_remove_pruned_evidence", delayed_cleanup)
    replacement = receipt(
        "reuse-evidence",
        response={"http_status": 200, "body": {"payload": "y" * 20_000}},
    )
    update_thread = threading.Thread(
        target=lambda: first.reconcile(replacement, expected_status="completed"),
        daemon=True,
    )
    update_thread.start()
    assert cleanup_started.wait(timeout=2)

    second.reconcile(original, expected_status="completed")
    release_cleanup.set()
    update_thread.join(timeout=2)
    assert not update_thread.is_alive()
    assert original_path.exists()
    detailed = second.by_command("reuse-evidence", include_evidence=True)
    assert detailed is not None
    assert detailed["response"]["body"]["payload"] == "x" * 20_000


def test_retention_never_deletes_an_inflight_idempotency_claim(tmp_path: Path) -> None:
    store = _operator_store(tmp_path)
    active = receipt("active-claim", status="queued")
    claimed, created = store.claim(active)
    assert created is True
    assert claimed["command_id"] == "active-claim"

    for index in range(600):
        store.put(receipt(f"terminal-{index}", status="completed"))

    retained = store.by_idempotency("key-active-claim")
    assert retained is not None
    assert retained["status"] == "reserved"
    assert store.connection.execute("SELECT COUNT(*) FROM operator_commands").fetchone()[0] == 601


def test_interrupt_fallback_rotation_does_not_lose_concurrent_append(tmp_path: Path, monkeypatch) -> None:
    import bioxp.operator_receipt_store as store_subject

    first = _operator_store(tmp_path)
    first.append_interrupt_fallback(receipt("fallback-before"), reason="test")
    original_replace = store_subject.os.replace
    writer_threads: list[threading.Thread] = []
    triggered = False

    def replace_with_concurrent_writer(source, destination):
        nonlocal triggered
        if Path(source) == first.interrupt_fallback_path and ".pending." in Path(destination).name and not triggered:
            triggered = True
            thread = threading.Thread(
                target=lambda: first.append_interrupt_fallback(receipt("fallback-during"), reason="test"),
                daemon=True,
            )
            writer_threads.append(thread)
            thread.start()
            time.sleep(0.02)
            assert thread.is_alive()
        return original_replace(source, destination)

    monkeypatch.setattr(store_subject.os, "replace", replace_with_concurrent_writer)
    second = _operator_store(tmp_path)
    writer_threads[0].join(timeout=2)
    assert not writer_threads[0].is_alive()
    assert second.by_command("fallback-before") is not None
    assert second.by_command("fallback-during") is None

    third = _operator_store(tmp_path)
    assert third.by_command("fallback-during") is not None


def test_unclassified_evidence_cannot_enter_direct_cleanup(tmp_path: Path) -> None:
    store = _operator_store(tmp_path)
    store.put(receipt(
        "cleanup-propagates",
        status="completed",
        response={"http_status": 200, "body": {"payload": "x" * 9000}},
    ))
    relpath = "operator_evidence/orphan-cleanup.json"
    evidence_path = tmp_path / relpath
    evidence_path.parent.mkdir(parents=True, exist_ok=True)
    evidence_path.write_text("{}", encoding="utf-8")
    store._remove_pruned_evidence([relpath])
    assert evidence_path.exists()
    assert store.connection.execute(
        "SELECT 1 FROM runtime_evidence_objects WHERE active_relpath=?",
        (relpath,),
    ).fetchone() is None
