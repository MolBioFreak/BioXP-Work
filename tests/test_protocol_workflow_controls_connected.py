"""HTTP controls through the real retained dispatcher; lifecycle leaves doubled.

Qualifies host control/custody interfaces only, not thermal/native support.
"""
import json
import os
from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_protocol_workflow_connected import mount_protocol_routes, await_job, request_payload


@pytest.fixture
def gated_workflow(installed_retained, monkeypatch, tmp_path):
    from bioxp import api
    from bioxp.protocols.executor import ProtocolExecutor
    from bioxp.protocols.models import ProtocolDocument
    app, provider, *_ = installed_retained
    from bioxp.services.protocol_service import ProtocolBindings
    monkeypatch.setenv("BIOXP_PROTOCOL_JOBS_ROOT", str(tmp_path / "artifacts"))
    trace = []
    def bindings(bundle, **kwargs):
        doc = ProtocolDocument.from_payload(bundle["protocol"]["document"])
        def native(action, state):
            trace.append(action.oem_opcode)
            return {"ok": True, "fixture_only": True, "physical_effect_verified": False}
        def hook(name):
            def call(state):
                trace.append(name)
                if name == "script_finally":
                    return kwargs["source_executor"]().finalize_source_host(state)
                return {"ok": True, "fixture_only": True, "physical_effect_verified": False}
            return call
        return ProtocolBindings({}, {"led": native}, {
            name: hook(name) for name in ProtocolExecutor.required_lifecycle(doc)
        }, source_script_begin=lambda state: provider.wp8_source_script_begin(command_id=state.workflow.command_id),
            source_script_returned=lambda state: provider.wp8_source_script_returned(command_id=state.workflow.command_id))
    monkeypatch.setattr(api, "_protocol_bindings", bindings)
    mount_protocol_routes(app)
    app.add_api_route("/protocol/jobs/{job_id}/control", api.protocol_job_control, methods=["POST"])
    client = TestClient(app)
    payload = request_payload("connected-gate", [
        {"action_id": "delay", "kind": "oem_operation", "oem_opcode": "delaypoint",
         "source_occurrence_id": "source:delay", "params": {"arguments": []}},
        {"action_id": "after", "kind": "oem_operation", "oem_opcode": "led",
         "source_occurrence_id": "source:after", "params": {"arguments": ["0", "0", "0"]}},
    ])
    payload["document"]["metadata"] = {"input_mode": "oem_prepared", "delayed_start": True,
        "source_model": {"logical_tip_present": False, "carried_plate_present": False, "allow_to_stop": True,
            "tip_trays": [{"tray_id": str(i), "location": 7+i,
                "wells": [{"content": None, "volume": 0, "capacity": 0, "empty": True} for _ in range(96)]}
                for i in range(4)]}}
    # Incomplete preparation cannot reserve custody or execute a prefix.
    incomplete = json.loads(json.dumps(payload))
    incomplete["idempotency_key"] = "missing-tip-inventory"
    incomplete["document"]["metadata"]["source_model"]["tip_trays"][3]["wells"] = []
    denied = client.post("/protocol/execute", json=incomplete)
    assert denied.status_code == 409 and "captured tip-tray wells" in denied.text
    assert trace == []
    assert app.state.operator_command_plane.store.list_workflows() == []
    accepted = client.post("/protocol/execute", json=payload)
    assert accepted.status_code == 202, accepted.text
    job_id = accepted.json()["job_id"]
    app.state.operator_command_plane.start()
    gate = await_job(client, job_id, lambda j: j["execution"]["runtime_state"]["workflow"]["gate"] == "delaypoint")
    yield client, app.state.operator_command_plane.store, payload, gate, trace
    # A failing assertion must not strand the actual owner at an unreleased gate.
    row = client.get("/protocol/jobs/" + job_id).json()
    if not row["command"]["terminal"]:
        client.post("/protocol/jobs/" + job_id + "/control", json={
            "command_id": job_id, "expected_ownership_generation": row["command"]["ownership_generation"],
            "idempotency_key": "fixture-teardown", "action": "abort"})
        await_job(client, job_id, lambda j: j["command"]["terminal"])


def control_body(job, key, **variant):
    return {"command_id": job["job_id"], "expected_ownership_generation": job["command"]["ownership_generation"],
            "idempotency_key": key, **variant}


def export_control(outcome, job):
    if os.environ.get("BIOXP_WORKFLOW_EXPORT"):
        Path(os.environ["BIOXP_WORKFLOW_EXPORT"] + "." + outcome + ".json").write_text(json.dumps({
            "fixture_only": True, "physical_acceptance": False, "job": job}, indent=2))


def test_delayed_continue_reconciles_original_control_and_keeps_other_claims_out(gated_workflow):
    client, store, payload, gate, trace = gated_workflow
    job_id = gate["job_id"]
    assert gate["command"]["status"] == "dispatched" and not gate["command"]["terminal"]
    with pytest.raises(ValueError, match="workflow_busy"):
        with store.normal_mutation_scope(resources=("thermal",)):
            pytest.fail("external mutation entered a held workflow")
    wrong = control_body(gate, "wrong-gate", action="continue", gate="delaypoint", gate_id="old")
    assert client.post("/protocol/jobs/" + job_id + "/control", json=wrong).status_code == 409
    body = control_body(gate, "continue-once", action="continue", gate="delaypoint", gate_id="source:delay")
    first = client.post("/protocol/jobs/" + job_id + "/control", json=body)
    assert first.status_code == 200, first.text
    done = await_job(client, job_id, lambda j: j["command"]["terminal"])
    assert done["command"]["status"] == "completed", done
    replay = client.post("/protocol/jobs/" + job_id + "/control", json=body)
    assert replay.status_code == 200 and replay.json() == first.json(), replay.text
    assert trace.count("led") == 1
    assert trace[-4:] == ["epilogue_sweep", "epilogue_lid", "epilogue_park", "script_finally"]
    export_control("delay-gate", gate)
    export_control("delay-completed", done)


def test_cooperative_abort_at_delay_does_not_continue_or_replay(gated_workflow):
    client, store, payload, gate, trace = gated_workflow
    job_id = gate["job_id"]
    body = control_body(gate, "abort-once", action="abort")
    response = client.post("/protocol/jobs/" + job_id + "/control", json=body)
    assert response.status_code == 200, response.text
    done = await_job(client, job_id, lambda j: j["command"]["terminal"])
    assert done["command"]["status"] == "interrupted", done
    assert "led" not in trace and "epilogue_park" not in trace
    assert trace.count("abort_true_prefix") == trace.count("cleanup") == 1
    assert client.post("/protocol/jobs/" + job_id + "/control", json=body).json() == response.json()
    assert client.post("/protocol/execute", json=payload).json()["job_id"] == job_id
    assert "led" not in trace
    export_control("aborted", done)


def test_affected_stop_diverts_held_owner_without_cleanup_or_new_native_entry(gated_workflow):
    client, store, payload, gate, trace = gated_workflow
    prior = list(trace)
    # Exercise the actual canonical epoch notification. Native Stop delivery has
    # independent regressions; this fixture does not pretend to send one.
    with store._transaction() as conn:
        conn.execute("UPDATE operator_plane_safety SET x_epoch=x_epoch+1")
    store._notify_workflow_interrupt()
    done = await_job(client, gate["job_id"], lambda j: j["command"]["terminal"])
    assert done["command"]["status"] == "interrupted", done
    assert trace == prior + ["script_finally"]
    export_control("stopped", done)
    assert client.post("/protocol/execute", json=payload).json()["job_id"] == gate["job_id"]
    assert trace == prior + ["script_finally"]
