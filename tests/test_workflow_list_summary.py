"""Compact discovery from the real SQLite reader; no execution or hardware."""
import json
import sqlite3
import threading

import pytest
from bioxp import operator_command_plane as plane
from bioxp.services.protocol_service import ProtocolOperatorBundleStore, list_protocol_jobs, get_protocol_job


def reader(path):
    # Only reader-owned columns: a fixture DB, not a replacement schema/migration.
    conn = sqlite3.connect(path, check_same_thread=False)
    conn.row_factory = sqlite3.Row
    conn.executescript("""
        CREATE TABLE operator_commands(sequence INTEGER PRIMARY KEY, command_id TEXT,
          command_kind TEXT, status TEXT, idempotency_key TEXT, ownership_generation INTEGER,
          requested_inputs_json TEXT, receipt_json TEXT);
        CREATE TABLE operator_plane_commands(command_id TEXT, version INTEGER);
    """)
    store = plane.OperatorCommandStore.__new__(plane.OperatorCommandStore)
    store.connection, store._lock = conn, threading.RLock()
    return store


def insert(store, payload, sequence, receipt=None):
    cmd = payload.get("command") or {}
    store.connection.execute("INSERT INTO operator_commands VALUES(?,?,?,?,?,?,?,?)", (
        sequence, payload["job_id"], "protocol_workflow", payload["status"],
        cmd.get("idempotency_key", "key-" + payload["job_id"]), cmd.get("ownership_generation", 7),
        json.dumps({"bundle": payload}), json.dumps(payload) if receipt is None else receipt))
    store.connection.execute("INSERT INTO operator_plane_commands VALUES(?,?)", (
        payload["job_id"], cmd.get("state_version", 1)))
    store.connection.commit()


def payload(job="job-1", dry_run=False):
    return dict(job_id=job, status="completed", created_at="created", updated_at="updated",
        protocol={"document": {"protocol_id": "test", "large": "x" * 10000}, "source_type": "native"},
        execution={"dry_run": dry_run}, operator={"pending_review": None})


def project(full):
    return dict(job_id=full["job_id"], status=full["status"],
        dry_run=full["execution"]["dry_run"], protocol_id=full["protocol"]["document"]["protocol_id"],
        source_type=full["protocol"]["source_type"], created_at=full["created_at"],
        updated_at=full["updated_at"], pending_review=full["operator"]["pending_review"], command=full["command"])


@pytest.mark.parametrize("receipt", [None, "{}", " { } ", "null", "broken", ""])
@pytest.mark.parametrize("dry_run", [False, True])
def test_projection_fallback_boolean_identity_and_one_select(tmp_path, monkeypatch, receipt, dry_run):
    store = reader(tmp_path / "reader.db")
    insert(store, payload(dry_run=dry_run), 1, receipt)
    expected = project(store.get_workflow("job-1"))
    queries, decoded = [], []
    store.connection.set_trace_callback(queries.append)
    original = plane._json_load
    def counted(value, *args):
        decoded.append(len(value.encode()))
        return original(value, *args)
    monkeypatch.setattr(plane, "_json_load", counted)
    monkeypatch.setattr(store, "get_workflow", lambda *a: pytest.fail("N+1 full read"))
    assert store.list_workflow_summaries() == [expected]
    assert len(queries) == 1
    assert sum(decoded) < 200
    store.connection.close()


def test_order_limit_historical_merge_and_full_detail(tmp_path):
    store = reader(tmp_path / "reader.db")
    historical = ProtocolOperatorBundleStore(tmp_path / "history")
    for index in range(4):
        insert(store, payload(f"job-{index}"), index)
    historical.save(payload("old-job", True))
    historical.save(payload("job-3"))
    full = list_protocol_jobs(limit=6, store=historical, command_store=store)
    compact = list_protocol_jobs(limit=6, summary=True, store=historical, command_store=store)
    assert [x["job_id"] for x in compact] == [x["job_id"] for x in full] == ["job-3", "job-2", "job-1", "job-0", "old-job"]
    assert compact[:4] == [project(x) for x in full[:4]]
    assert compact[4] == full[4]
    assert list_protocol_jobs(limit=2, summary=True, store=historical, command_store=store) == compact[:2]
    class NoHistoryRead:
        def list(self, **kwargs):
            pytest.fail("full canonical page must not read historical bundle files")
    assert list_protocol_jobs(limit=2, summary=True, store=NoHistoryRead(), command_store=store) == compact[:2]
    selected = get_protocol_job(compact[0]["job_id"], store=historical, command_store=store)
    assert selected == full[0]
    assert len(selected["protocol"]["document"]["large"]) == 10000
    assert get_protocol_job("old-job", store=historical, command_store=store)["execution"]["dry_run"] is True
    store.connection.close()


def test_real_schema_admission_and_api_summary(tmp_path, monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient
    from bioxp import api
    from bioxp.oem_runtime_store import OEMRuntimeStore
    OEMRuntimeStore(tmp_path / "runtime")
    store = plane.OperatorCommandStore(tmp_path / "runtime")
    try:
        full = store.admit_workflow(command_id="job-1", idempotency_key="key-1",
            plan_fingerprint="fingerprint", requested_inputs={"bundle": payload()},
            ownership_generation=7, resources=[], board_epochs={})
        historical = ProtocolOperatorBundleStore(tmp_path / "history")
        monkeypatch.setattr(api, "_protocol_command_store", lambda: store)
        original_list = list_protocol_jobs
        monkeypatch.setattr(api, "list_protocol_jobs", lambda **kw: original_list(store=historical, **kw))
        # Mount the actual handler without robot app startup/hardware.
        app = FastAPI()
        app.add_api_route("/protocol/jobs", api.protocol_jobs)
        client = TestClient(app)
        response = client.get("/protocol/jobs?summary=true")
        assert response.status_code == 200
        assert response.json() == {"rows": [project(full)]}
        assert client.get("/protocol/jobs").json() == {"rows": [full]}
        assert client.get("/protocol/jobs?limit=0").status_code == 422
    finally:
        store.stop()


def test_receipt_is_whole_snapshot_not_field_coalescing(tmp_path):
    store = reader(tmp_path / "reader.db")
    insert(store, payload(), 1, json.dumps({"updated_at": "new"}))
    row = store.list_workflow_summaries()[0]
    assert row["updated_at"] == "new" and row["dry_run"] is None and row["protocol_id"] is None
    store.connection.close()
