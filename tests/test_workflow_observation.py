"""Current selected-job projection, using actual captured bytes when supplied."""
import copy
import json
import os
from pathlib import Path

import pytest
from bioxp import operator_command_plane as plane
from bioxp.services.protocol_service import get_protocol_job
from test_workflow_list_summary import reader, insert, payload


def project(full):
    runtime = full.get("execution", {}).get("runtime_state", {})
    operator = full.get("operator", {})
    return {"schema_version": "bioxp.protocol_job_observation.v1",
        "job_id": full["job_id"], "status": full["status"],
        "command": {k: v for k, v in full["command"].items() if k not in {"idempotency_key", "status_path"}},
        "execution": {**({"dry_run": full["execution"]["dry_run"]} if "dry_run" in full.get("execution", {}) else {}),
            "runtime_state": {k: runtime[k] for k in ("workflow",) if k in runtime}},
        "operator": {k: operator[k] for k in ("pending_review",) if k in operator}}


@pytest.mark.parametrize("receipt", [None, "{}", " { } ", "null", "broken", ""])
def test_whole_receipt_fallback_and_one_projected_select(tmp_path, monkeypatch, receipt):
    store = reader(tmp_path / "reader.db")
    body = payload()
    body["execution"]["runtime_state"] = {"workflow": None}
    insert(store, body, 1, receipt)
    expected = project(store.get_workflow(body["job_id"]))
    statements, decoded = [], []
    store.connection.set_trace_callback(statements.append)
    load = plane._json_load
    def counted(value, *args):
        decoded.append(len(value.encode()))
        return load(value, *args)
    monkeypatch.setattr(plane, "_json_load", counted)
    monkeypatch.setattr(store, "get_workflow", lambda *a: pytest.fail("full hydration"))
    assert get_protocol_job(body["job_id"], observation=True, command_store=store) == expected
    assert len(statements) == 1 and sum(decoded) < 100
    store.connection.close()


def test_omission_is_not_null_and_no_admission_field_coalescing(tmp_path):
    store = reader(tmp_path / "reader.db")
    insert(store, payload(), 1, json.dumps({"execution": {"dry_run": True}, "operator": {}}))
    observed = store.get_workflow_observation("job-1")
    assert observed["operator"] == {}
    assert observed["execution"] == {"dry_run": True, "runtime_state": {}}
    assert observed == project(store.get_workflow("job-1"))
    assert store.get_workflow_observation("missing") is None
    store.connection.execute("UPDATE operator_commands SET receipt_json=?", (json.dumps({"operator": {"pending_review": None}}),))
    assert store.get_workflow_observation("job-1") == project(store.get_workflow("job-1"))
    store.connection.close()


def test_actual_retained_database_readonly(tmp_path, monkeypatch):
    import sqlite3
    import threading
    backup = os.environ.get("BIOXP_RETAINED_DB")
    capture = os.environ.get("BIOXP_SELECTED_JOB_CAPTURE")
    if not backup or not capture:
        pytest.skip("requires readonly retained DB backup and actual selected capture")
    original = json.loads(Path(capture).read_bytes())
    store = plane.OperatorCommandStore.__new__(plane.OperatorCommandStore)
    store.connection = sqlite3.connect(f"file:{backup}?mode=ro", uri=True)
    store.connection.row_factory = sqlite3.Row
    store._lock = threading.RLock()
    try:
        full = store.get_workflow(original["job_id"])
        assert full == original
        decoded, statements = [], []
        load = plane._json_load
        monkeypatch.setattr(plane, "_json_load", lambda value, *args: (decoded.append(len(value.encode())), load(value, *args))[1])
        store.connection.set_trace_callback(statements.append)
        monkeypatch.setattr(store, "get_workflow", lambda *a: pytest.fail("full reader called"))
        for _ in range(30):
            assert store.get_workflow_observation(original["job_id"]) == project(full)
        assert len(statements) == 30
        assert max(decoded) < len(Path(capture).read_bytes()) * .05
    finally:
        store.connection.close()


def test_actual_writers_and_http_negotiation(tmp_path, monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient
    from bioxp import api
    from bioxp.oem_runtime_store import OEMRuntimeStore
    prep = OEMRuntimeStore(tmp_path / "runtime")
    prep.close()
    store = plane.OperatorCommandStore(tmp_path / "runtime")
    store.bind_workflow_dispatcher(lambda claimed: None)  # inert executor
    body = payload()
    body["execution"]["runtime_state"] = {"workflow": {"command_id": "job-1", "phase": "queued"}}
    try:
        store.admit_workflow(command_id="job-1", idempotency_key="key-1", plan_fingerprint="plan",
            requested_inputs={"bundle": body}, ownership_generation=7, resources=[], board_epochs={})
        monkeypatch.setattr(api, "_protocol_command_store", lambda: store)
        app = FastAPI()
        app.add_api_route("/protocol/jobs/{job_id}", api.protocol_job_detail)
        client = TestClient(app)
        def check():
            full = client.get("/protocol/jobs/job-1")
            observed = client.get("/protocol/jobs/job-1?observation=true")
            assert full.status_code == observed.status_code == 200
            assert observed.json() == project(full.json())
            assert "large" in full.json()["protocol"]["document"]
            return observed.json()
        assert check()["status"] == "queued"
        store.claim_next()
        assert check()["status"] == "dispatched"
        workflow = body["execution"]["runtime_state"]["workflow"]
        workflow.update(phase="waiting", gate="review", source_occurrence_id="review:1")
        body["operator"]["pending_review"] = {"stage_id": "stage:1", "action_id": "review:1", "reason": None}
        store.publish_workflow("job-1", payload=body)
        assert check()["operator"]["pending_review"]["action_id"] == "review:1"
        def control(cid, request):
            workflow.update(last_control_id=cid, reached_control_id=cid, requested_control=None, phase="executing", gate=None)
            body["operator"]["pending_review"] = None
            store.publish_workflow("job-1", payload=body)
            return workflow
        store.bind_workflow_controls("job-1", control)
        store.control_workflow("job-1", request={"action": "review", "command_id": "job-1",
            "expected_ownership_generation": 7, "idempotency_key": "review-key"})
        assert check()["operator"]["pending_review"] is None
        store.finish_workflow("job-1", status="completed", payload=body, lifecycle_settled=False)
        assert check()["execution"]["runtime_state"]["workflow"]["held_reason"] == "workflow_settlement_unknown"
    finally:
        store.stop()


def test_actual_capture_idle_and_changing_minutes(tmp_path, monkeypatch):
    capture = os.environ.get("BIOXP_SELECTED_JOB_CAPTURE")
    if not capture:
        pytest.skip("set BIOXP_SELECTED_JOB_CAPTURE to the actual selected-job HTTP body")
    raw = Path(capture).read_bytes()
    original = json.loads(raw)
    store = reader(tmp_path / "capture-replay.db")
    insert(store, original, 1)
    original_read = store.get_workflow(original["job_id"])
    assert original_read == original
    load = plane._json_load
    decoded = []
    monkeypatch.setattr(plane, "_json_load", lambda value, *args: (decoded.append(len(value.encode())), load(value, *args))[1])
    observations = {}
    for scenario in ("idle", "changing"):
        rows = []
        for tick in range(30):
            full = copy.deepcopy(original)
            if scenario == "changing":
                workflow = full["execution"]["runtime_state"]["workflow"]
                # Replay changed native receipt fields while deliberately keeping
                # the retained command version fixed. No live control is called.
                workflow["source_occurrence_id"] = f"occurrence:{tick}"
                workflow["requested_control"] = {"action": "pause", "mode": "ordinary"} if tick % 2 else None
                workflow["last_control_id"] = f"control:{tick}"
                workflow["reached_control_id"] = None if tick % 2 else f"control:{tick}"
                workflow["gate"] = "review" if tick % 3 == 0 else None
                workflow["gate_id"] = f"gate:{tick}" if tick % 3 == 0 else None
                full["operator"]["pending_review"] = {"stage_id": f"stage:{tick}", "action_id": None, "reason": "review"} if tick % 3 == 0 else None
                if tick < 15:
                    full["status"] = "dispatched"
                    full["command"].update(status="dispatched", terminal=False)
                    workflow["phase"] = "waiting" if tick % 3 == 0 else "executing"
                    workflow["held_reason"] = None
                else:
                    workflow["held_reason"] = "workflow_owner_lost" if tick % 2 else "workflow_settlement_unknown"
            store.connection.execute("UPDATE operator_commands SET status=?,receipt_json=? WHERE command_id=?",
                (full["status"], json.dumps(full), full["job_id"]))
            result = store.get_workflow_observation(full["job_id"])
            assert result == project(full)
            rows.append(result)
        sizes = [len(json.dumps(row, separators=(",", ":")).encode()) for row in rows]
        observations[scenario] = {"polls": len(rows), "full_bytes": len(raw)*len(rows),
            "observation_bytes": sum(sizes), "min_bytes": min(sizes), "max_bytes": max(sizes),
            "reduction_percent": 100*(1-sum(sizes)/(len(raw)*len(rows))), "rows": rows}
        assert observations[scenario]["reduction_percent"] >= 95
    assert store.get_workflow(original["job_id"])["execution"]["runtime_state"]["source_model"] == original["execution"]["runtime_state"]["source_model"]
    output = os.environ.get("BIOXP_OBSERVATION_EVIDENCE")
    if output:
        Path(output).write_text(json.dumps({"baseline_bytes": len(raw), "job_id": original["job_id"],
            "cold_observation_bytes": len(json.dumps(project(original), separators=(",", ":")).encode()),
            "python_decoded_bytes": decoded, "windows": observations}, indent=2))
    store.connection.close()
