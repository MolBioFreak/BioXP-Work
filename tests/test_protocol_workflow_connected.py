"""Connected HTTP -> service -> dispatcher -> executor -> SQLite, native leaves doubled.

These tests qualify host orchestration only, never physical or thermal support.
"""
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained


def request_payload(key, actions=None):
    return {
        "source_type": "native", "dry_run": False, "idempotency_key": key,
        "document": {
            "protocol_id": "offline-host-integration",
            "stages": [{"stage_id": "one", "actions": actions or [
                {"action_id": "first", "kind": "led", "params": {"witness": "original"}},
                {"action_id": "second", "kind": "led", "params": {"witness": "second"}},
            ]}],
        },
        "live_execution": {
            "live_execution_ack": True, "operator_id": "offline-test",
            "physical_console_verified": True, "deck_manifest": {"fixture": "no-hardware"},
            "artifact_refs": ["offline-test-only"],
        },
    }


def mount_protocol_routes(app):
    from bioxp import api
    app.add_api_route("/protocol/execute", api.protocol_execute, methods=["POST"])
    app.add_api_route("/protocol/jobs", api.protocol_jobs, methods=["GET"])
    app.add_api_route("/protocol/jobs/{job_id}", api.protocol_job_detail, methods=["GET"])
    app.add_api_route("/protocol/jobs/{job_id}/review", api.protocol_job_review, methods=["POST"])


def await_job(client, job_id, predicate, timeout=8):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        response = client.get("/protocol/jobs/" + job_id)
        assert response.status_code == 200, response.text
        last = response.json()
        if predicate(last):
            return last
        time.sleep(.01)
    pytest.fail(f"canonical job did not reach expected state: {last}")


def test_http_custody_returns_before_leaf_and_replays_without_current_support(installed_retained, monkeypatch, tmp_path):
    from bioxp import api
    from bioxp.protocols.models import ProtocolActionKind
    app, provider, primitive, references, root = installed_retained
    monkeypatch.setenv("BIOXP_PROTOCOL_JOBS_ROOT", str(tmp_path / "artifacts"))
    entered, release = threading.Event(), threading.Event()
    calls = []

    def leaf(action, state):
        from bioxp.runtime_audit_store import workflow_claim_context
        calls.append((action.action_id, action.params["witness"], workflow_claim_context()))
        if action.action_id == "first":
            entered.set()
            assert release.wait(12)
        return {"ok": True, "fixture_only": True, "physical_operation": False}

    monkeypatch.setattr(api, "_protocol_live_handlers", lambda: {ProtocolActionKind.LED: leaf})
    mount_protocol_routes(app)
    client = TestClient(app)
    payload = request_payload("connected-original")
    # Prepared logical inventory is exercised through the real writer/readers;
    # these fixture values are not physical readiness or generated science.
    well = {"content": "fixture", "volume": 12.5, "capacity": 50.0,
            "empty": False, "zone_index": 2}
    tray = {"tray_id": "fixture-tray", "location": 5, "wells": [well],
            "tip_type": 50, "tray_empty": False, "strip_color": "fixture-color"}
    payload["document"]["metadata"] = {"source_model": {
        "fluid_name": "fixture", "tip_zone_index": 2, "old_tip_well": "A1",
        "trays": {"5": tray}, "strips": [tray], "tip_trays": [tray],
        "pressure_baseline": [1.0, 2.0, 3.0, 4.0],
        "pressure_history": [[float(i)] * 5 for i in range(4)],
    }}
    response = client.post("/protocol/execute", json=payload)
    assert response.status_code == 202, response.text
    admitted = response.json()
    job_id = admitted["job_id"]
    assert admitted["command"]["command_id"] == job_id
    assert admitted["command"]["status"] == "queued"
    assert calls == []
    app.state.operator_command_plane.start()
    try:
        assert entered.wait(6)
        running = client.get("/protocol/jobs/" + job_id)
        assert running.status_code == 200, running.text
        assert running.json()["command"]["status"] == "dispatched"
        monkeypatch.setattr(api, "_protocol_live_handlers", lambda: {})
        replay = client.post("/protocol/execute", json=payload)
        assert replay.status_code == 202, replay.text
        assert replay.json()["job_id"] == job_id
        assert len(calls) == 1
    finally:
        release.set()
    done = await_job(client, job_id, lambda value: value["command"]["terminal"])
    assert done["command"]["status"] == "completed", done
    assert [row[:2] for row in calls] == [("first", "original"), ("second", "second")]
    assert all(row[2]["parent_command_id"] == job_id and row[2]["parent_attempt"] for row in calls)
    assert [row[2]["source_occurrence_id"] for row in calls] == ["first", "second"]
    assert done["execution"]["runtime_state"]["workflow"]["phase"] == "terminal"
    replay = client.post("/protocol/execute", json=payload)
    assert replay.status_code == 200 and replay.json()["job_id"] == job_id, replay.text
    changed = copy.deepcopy(payload)
    changed["document"]["stages"][0]["actions"][0]["params"]["witness"] = "changed"
    conflict = client.post("/protocol/execute", json=changed)
    assert conflict.status_code == 409, conflict.text
    assert len(calls) == 2
    listed = client.get("/protocol/jobs").json()["rows"]
    assert sum(row["job_id"] == job_id for row in listed) == 1
    app.state.operator_command_plane.stop()
    code = (
        "import json,sys;from bioxp.operator_command_plane import OperatorCommandStore;"
        "s=OperatorCommandStore(sys.argv[1]);print(json.dumps(s.get_workflow(sys.argv[2])));s.stop()"
    )
    reopened = json.loads(subprocess.check_output([sys.executable, "-c", code, str(root), job_id], text=True, timeout=25))
    assert reopened["command"] == done["command"]
    assert reopened["execution"]["runtime_state"] == done["execution"]["runtime_state"]
    if os.environ.get("BIOXP_WORKFLOW_EXPORT"):
        Path(os.environ["BIOXP_WORKFLOW_EXPORT"]).write_text(json.dumps({
            "fixture_only": True, "physical_acceptance": False,
            "request": payload, "accepted": admitted, "running": running.json(),
            "completed": done, "fresh_process": reopened,
        }, indent=2))


def test_review_keeps_same_parent_and_original_ack_does_not_reexecute(installed_retained, monkeypatch, tmp_path):
    from bioxp import api
    from bioxp.protocols.models import ProtocolActionKind
    app, *_ = installed_retained
    monkeypatch.setenv("BIOXP_PROTOCOL_JOBS_ROOT", str(tmp_path / "artifacts"))
    calls = []
    def leaf(action, state):
        calls.append(action.action_id)
        return {"ok": True, "fixture_only": True}
    monkeypatch.setattr(api, "_protocol_live_handlers", lambda: {ProtocolActionKind.LED: leaf})
    mount_protocol_routes(app)
    client = TestClient(app)
    payload = request_payload("review-parent", actions=[
        {"action_id": "first", "kind": "led", "review_required": True},
        {"action_id": "second", "kind": "led"},
    ])
    response = client.post("/protocol/execute", json=payload)
    assert response.status_code == 202, response.text
    admitted = response.json()
    job_id = admitted["job_id"]
    app.state.operator_command_plane.start()
    paused = await_job(client, job_id, lambda row: row["execution"]["runtime_state"]["workflow"]["gate"] == "review")
    assert paused["command"]["status"] == "dispatched" and not paused["command"]["terminal"]
    assert calls == ["first"]
    body = {
        "idempotency_key": "review-original-ack", "command_id": job_id,
        "expected_ownership_generation": paused["command"]["ownership_generation"],
        "stage_id": "one", "action_id": "first", "reviewer": "offline-test",
    }
    wrong = {**body, "idempotency_key": "wrong-review", "action_id": "second"}
    rejected = client.post("/protocol/jobs/" + job_id + "/review", json=wrong)
    assert rejected.status_code == 409, rejected.text
    assert calls == ["first"]
    accepted = client.post("/protocol/jobs/" + job_id + "/review", json=body)
    assert accepted.status_code == 200, accepted.text
    done = await_job(client, job_id, lambda row: row["command"]["terminal"])
    assert done["command"]["status"] == "completed", done
    assert calls == ["first", "second"]
    replay = client.post("/protocol/jobs/" + job_id + "/review", json=body)
    assert replay.status_code == 200 and replay.json()["job_id"] == job_id, replay.text
    assert calls == ["first", "second"]


def test_late_unbound_action_is_rejected_before_any_native_leaf(installed_retained, monkeypatch, tmp_path):
    from bioxp import api
    from bioxp.protocols.models import ProtocolActionKind
    app, *_ = installed_retained
    monkeypatch.setenv("BIOXP_PROTOCOL_JOBS_ROOT", str(tmp_path / "artifacts"))
    calls = []
    monkeypatch.setattr(api, "_protocol_live_handlers", lambda: {
        ProtocolActionKind.LED: lambda action, state: calls.append(action.action_id) or {"ok": True}
    })
    mount_protocol_routes(app)
    client = TestClient(app)
    app.state.operator_command_plane.start()
    payload = request_payload("late-unbound", actions=[
        {"action_id": "first", "kind": "led"},
        {"action_id": "last", "kind": "wait", "params": {}},
    ])
    response = client.post("/protocol/execute", json=payload)
    assert response.status_code in (400, 409), response.text
    assert calls == []
    assert app.state.operator_command_plane.store.list_workflows() == []
