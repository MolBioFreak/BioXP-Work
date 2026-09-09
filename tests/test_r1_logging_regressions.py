"""R1 disposable-store tests. No API/provider/USB/runtime singleton imports."""
import fcntl
import json
import os
import sqlite3
import subprocess
import sys
import threading
import time

import pytest

from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.operator_receipt_store import OperatorReceiptStore
from bioxp.command_exchange_observer import ExchangeObserver


@pytest.fixture
def store(tmp_path):
    owner = OEMRuntimeStore(tmp_path)
    owner.close()
    result = OperatorReceiptStore(tmp_path)
    yield result
    result.connection.close()


def receipt(identity="stop-1"):
    return {"command_id": identity, "idempotency_key": identity,
            "action_id": "oem.x.stop", "status": "completed",
            "ownership_generation": 1, "started_at": "100.0", "finished_at": "101.0",
            "controller_acknowledged": False, "physical_effect_verified": False,
            "response": {"http_status": 200, "body": {"raw_rx": "7e7d5e", "raw_tx": "010203",
                         "source_return": 0, "completion_disposition": "unverified"}},
            "stage_receipts": [{"stage": "double-stop", "raw_rx": "abcdef", "ack": None}]}


def test_interrupt_fallback_retains_full_response_and_stages_through_restart(store):
    original = receipt()
    store.append_interrupt_fallback(original, reason="test_db_busy")
    # Public result may be compact; the durable recovery bytes must not be.
    raw = json.loads(store.interrupt_fallback_path.read_text())
    assert raw["response"] == original["response"]
    assert raw["stage_receipts"] == original["stage_receipts"]
    store._import_interrupt_fallback()
    got = store.by_command(original["command_id"], include_evidence=True)
    assert got["response"] == original["response"]
    assert got["stage_receipts"] == original["stage_receipts"]
    store._import_interrupt_fallback()
    assert store.connection.execute("SELECT COUNT(*) FROM operator_commands").fetchone()[0] == 1


def test_interrupt_lifecycle_contention_does_not_wait_on_ordinary_storage(store):
    # A real separate flock owner, not a mocked SQLite connection.
    fd = os.open(store.root / "runtime-storage.lifecycle.lock", os.O_RDWR)
    fcntl.flock(fd, fcntl.LOCK_EX)
    done = threading.Event()
    results, errors = [], []
    def run():
        try:
            results.append(store.put_interrupt(receipt()))
        except BaseException as exc:
            errors.append(exc)
        finally:
            done.set()
    worker = threading.Thread(target=run)
    worker.start()
    try:
        finished_while_contended = done.wait(1.0)
    finally:
        fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)
        worker.join(5)
    assert not worker.is_alive()
    assert not errors
    assert finished_while_contended, "interrupt persistence waited for ordinary lifecycle lease"
    assert results[0]["persistence_fallback"]["reason"] == "runtime_lifecycle_busy"


def test_interrupt_never_rolls_back_reentrant_ordinary_transaction(store):
    store.connection.execute("BEGIN IMMEDIATE")
    store.connection.execute("INSERT INTO runtime_metadata(key,value,updated_at) VALUES('r1-uncommitted','kept',100)")
    try:
        result = store.put_interrupt(receipt())
        assert store.connection.in_transaction
        assert store.connection.execute("SELECT value FROM runtime_metadata WHERE key='r1-uncommitted'").fetchone()[0] == "kept"
        assert result["persistence_fallback"]["reason"] == "sqlite_transaction_active"
    finally:
        store.connection.rollback()


def test_interrupt_preserves_connection_busy_timeout(store):
    store.connection.execute("PRAGMA busy_timeout=71")
    store.put_interrupt(receipt())
    assert store.connection.execute("PRAGMA busy_timeout").fetchone()[0] == 71


def test_transport_identity_collision_is_not_silently_dropped(store):
    store.put(receipt())
    original = {"exchange_id": "e1", "command_id": "stop-1", "raw_rx": "01"}
    store.merge_transport_evidence("stop-1", {"transport_exchanges": [original]})
    with pytest.raises(ValueError, match="identity collision"):
        store.merge_transport_evidence("stop-1", {"transport_exchanges": [{**original, "raw_rx": "02"}]})
    retained = store.by_command("stop-1")["transport_exchanges"]
    assert retained == [original]


def test_real_sqlite_busy_falls_back_then_reconciles(store):
    other = sqlite3.connect(store.path)
    other.execute("BEGIN IMMEDIATE")
    try:
        result = store.put_interrupt(receipt())
        assert "persistence_fallback" in result
    finally:
        other.rollback()
        other.close()
    store._import_interrupt_fallback()
    assert store.by_command("stop-1")["response"] == receipt()["response"]
    assert store.connection.execute("PRAGMA integrity_check").fetchone()[0] == "ok"
    assert store.connection.execute("PRAGMA foreign_key_check").fetchall() == []


def test_disk_full_failure_is_visible_and_existing_claim_survives(store, monkeypatch):
    store.put(receipt())
    def full(*args, **kwargs):
        raise OSError(28, "No space left on device")
    monkeypatch.setattr(os, "write", full)
    with pytest.raises(OSError, match="No space left"):
        store.append_interrupt_fallback(receipt("stop-2"), reason="test")
    assert store.by_command("stop-1") is not None
    assert store.by_command("stop-2") is None


def test_observer_writer_failure_is_visible_but_not_durable():
    def fail(_):
        raise sqlite3.OperationalError("database or disk is full")
    owner = ExchangeObserver("command", fail)
    owner.append({"exchange_id": "e", "transaction_id": "t"})
    owner.flush()
    snapshot = owner.snapshot()
    assert len(snapshot["transport_exchanges"]) == 1
    assert snapshot["transport_retention_errors"][0]["class"] == "OperationalError"
    # This verifies only an in-memory error channel, NOT crash-safe ingress.


def test_sqlite_full_propagates_original_error_without_claim(store):
    count = store.connection.execute("PRAGMA page_count").fetchone()[0]
    store.connection.execute(f"PRAGMA max_page_count={count}")
    attempt = {**receipt("full"), "inputs": {"payload": "x" * 4_000_000}}
    with pytest.raises(sqlite3.OperationalError, match="full"):
        store.put(attempt)
    assert not store.connection.in_transaction
    assert store.by_command("full") is None
    assert store.connection.execute("PRAGMA integrity_check").fetchone()[0] == "ok"


def test_committed_receipt_and_recovery_journal_survive_process_exit(tmp_path):
    owner = OEMRuntimeStore(tmp_path)
    owner.close()
    code = """
import json, os, sys
from bioxp.operator_receipt_store import OperatorReceiptStore
s = OperatorReceiptStore(sys.argv[1])
a, b = json.loads(sys.argv[2])
s.put(a)
s.append_interrupt_fallback(b, reason='simulated_writer_unavailable')
os._exit(0)
"""
    first, second = receipt("committed"), receipt("journal")
    subprocess.run([sys.executable, "-c", code, str(tmp_path), json.dumps([first, second])], check=True, timeout=15)
    restarted = OperatorReceiptStore(tmp_path)
    try:
        restarted.converge_startup_state()
        assert restarted.by_command("committed")["response"] == first["response"]
        assert restarted.by_command("journal")["response"] == second["response"]
        assert restarted.connection.execute("PRAGMA integrity_check").fetchone()[0] == "ok"
    finally:
        restarted.connection.close()


def test_pagination_and_large_checkpoint_deduplicate_without_loss(store):
    for i in range(7):
        store.put(receipt(f"page-{i}"))
    all_rows, cursor = [], None
    while True:
        page = store.list(3, before_sequence=cursor)
        if not page:
            break
        all_rows.extend(page)
        # Store accepts a sequence cursor but does not project it publicly.
        # Exercise store pagination with the DB cursor; API pagination is a
        # separate unresolved contract, not claimed by this test.
        cursor = store.connection.execute(
            "SELECT sequence FROM operator_commands WHERE command_id=?",
            (page[-1]["command_id"],),
        ).fetchone()[0]
    assert len({r["command_id"] for r in all_rows}) == len(all_rows) == 7
    exchanges = [{"exchange_id": f"e{i}", "command_id": "page-0", "reader_generation": 1,
                  "receive_sequence": i, "raw_rx": "ab" * 64} for i in range(3000)]
    store.merge_transport_evidence("page-0", {"transport_exchanges": exchanges})
    store.merge_transport_evidence("page-0", {"transport_exchanges": exchanges})
    assert store.by_command("page-0")["transport_exchanges"] == exchanges


def test_failed_checkpoint_retries_and_late_evidence_preserve_terminal_state(store):
    store.put(receipt())
    attempts = []
    def sink(evidence):
        attempts.append(evidence)
        if len(attempts) == 1:
            raise sqlite3.OperationalError("database is locked")
        store.merge_transport_evidence("stop-1", evidence)
    observer = ExchangeObserver("stop-1", sink)
    observer.append({"exchange_id": "early", "transaction_id": "t1"})
    observer.flush()
    observer.append({"exchange_id": "late", "transaction_id": "t2"})
    observer.flush()
    result = store.by_command("stop-1")
    assert result["status"] == "completed"
    assert [r["exchange_id"] for r in result["transport_exchanges"]] == ["early", "late"]
    assert result["transport_retention_errors"][0]["class"] == "OperationalError"
