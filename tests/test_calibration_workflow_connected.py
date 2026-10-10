"""HTTP/native dispatcher/SQLite receiving; only physical thermal leaves replaced."""
import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_protocol_workflow_connected import mount_protocol_routes, request_payload, await_job
from tests.test_calibration_measurements import prompt, reference, pipette


def export(name, payload):
    root = os.environ.get("NON_OEM_NATIVE_EXPORT_DIR")
    if root:
        path = Path(root)
        path.mkdir(parents=True, exist_ok=True)
        (path / (name + ".json")).write_text(json.dumps(payload, indent=2))


@pytest.mark.parametrize("module,bank", [("rc","rc"),("oc","oc"),("tc","nest"),("tc","lid"),("tc","pedestal")])
def test_thermal_prompt_real_handlers_same_job_and_store_reopen(installed_retained, monkeypatch, tmp_path, module, bank):
    from bioxp import api
    from bioxp.operator_command_plane import OperatorCommandStore
    from bioxp.protocols.runtime_state import ProtocolRuntimeState
    app, provider, primitives, references, root = installed_retained
    monkeypatch.setenv("BIOXP_PROTOCOL_JOBS_ROOT", str(tmp_path / "artifacts"))
    calls = []
    last_target = [None]
    # Fixture-only physical leaves. Real API binding, thermal mechanism, timers,
    # executor, controls and durable job all remain connected.
    def set_temp(selected, target, **kw):
        calls.append((selected, target, kw))
        last_target[0] = target
        return {"ok": True, "fixture_only": True, "authored_target": target}
    tester = SimpleNamespace(THERMAL_BANK_NEST=0, THERMAL_BANK_LID=1,
        CHILLER_BANK_RC=0, CHILLER_BANK_OC=1,
        chiller_activate=lambda: calls.append("activate"),
        chiller_set_target_temp=set_temp, thermal_set_target_temp=set_temp,
        thermal_set_ped_temp=lambda target, **kw: set_temp("pedestal", target, **kw),
        thermal_read_temp_gp=lambda selected: {"ok": True, "temp_c":last_target[0] - .1, "fixture_only":True})
    monkeypatch.setattr(api, "_get_tester", lambda: tester)
    mount_protocol_routes(app)
    app.add_api_route("/protocol/calibration/schema",api.protocol_calibration_schema,methods=["GET"])
    app.add_api_route("/protocol/calibration/analyze",api.protocol_calibration_analyze,methods=["POST"])
    client = TestClient(app)
    samples, actions = [], []
    for index, target in enumerate((31.25, 37.0)):
        pid = f"probe{index}"
        actions.append({"action_id":f"heat{index}", "kind":"chiller_setpoint" if module != "tc" else "thermal_setpoint",
                        "params":{"bank":bank,"target_temp_c":target}})
        if bank == "nest":
            actions[-1]["kind"]="thermal_hold"
            actions[-1]["params"].update(duration_s=0,start="attainment",tolerance_c=.2,timeout_s=2)
        actions.extend([{"action_id":f"timer{index}","kind":"timer_start","params":{"timer_id":f"t{index}","seconds":0}},
                        {"action_id":f"wait{index}","kind":"timer_wait","params":{"timer_id":f"t{index}"}}])
        p = prompt(pid,module)
        p["params"]["measurement_form"]["fields"][0].update(target=str(target),commanded=str(target),zone=bank)
        actions.append(p)
        for repeat in range(2):
            samples.append({"point_id":pid,"module":module,"zone":bank,"target_c":str(target),"commanded_c":str(target),
                            "measured_c":reference(pid,repeat),
                            **({"controller_reported_c":{"stage_id":"one","action_id":f"heat{index}",
                               "source_occurrence_id":f"heat{index}","output":"result","path":["last_temperature","temp_c"]}}
                               if bank == "nest" else {})})
    actions.append({"action_id":"analysis","kind":"calibration","params":{"operation":"analyze","analysis":{"kind":"thermal","samples":samples}}})
    request = request_payload(f"measurement-{module}-{bank}", actions)
    response = client.post("/protocol/execute",json=request)
    assert response.status_code == 202,response.text
    job=response.json()["job_id"]
    app.state.operator_command_plane.start()
    helds, acknowledgments = [], []
    for index in range(2):
        paused=await_job(client,job,lambda r:r["operator"].get("pending_review",{}).get("action_id") == f"probe{index}" if r["operator"].get("pending_review") else False)
        helds.append(paused)
        pending=paused["operator"]["pending_review"]
        # Reopening a reader while the same executor owns the held job neither
        # replays heating nor invents a new recovery executor.
        reopened=OperatorCommandStore(root)
        try:
            raw=reopened.get_workflow(job)
            assert raw["operator"]["pending_review"] == pending
            ProtocolRuntimeState.from_payload(raw["execution"]["runtime_state"])
        finally:
            reopened.stop()
        body={"command_id":job,"idempotency_key":f"measurement-ack-{index}",
              "expected_ownership_generation":paused["command"]["ownership_generation"],
              "stage_id":"one","action_id":f"probe{index}","source_occurrence_id":f"repeat/probe{index}",
              "reviewer":"external-probe-fixture","measurements":{"probe":["30.9000",31.1] if index == 0 else ["36.9000",37.1]}}
        wrong=client.post(f"/protocol/jobs/{job}/review",json={**body,"idempotency_key":f"wrong-{index}","source_occurrence_id":"wrong"})
        assert wrong.status_code == 409,wrong.text
        if module == "rc" and index == 0:
            store=app.state.operator_command_plane.store
            original_publish=store.publish_workflow
            def fail_response_write(command_id, *, payload):
                if payload["execution"]["runtime_state"]["measurement_reviews"]:
                    raise OSError("injected scratch response persistence failure")
                return original_publish(command_id,payload=payload)
            monkeypatch.setattr(store,"publish_workflow",fail_response_write)
            with TestClient(app,raise_server_exceptions=False) as failing_client:
                failed=failing_client.post(f"/protocol/jobs/{job}/review",json={**body,"idempotency_key":"write-failed"})
                assert failed.status_code == 500
            monkeypatch.setattr(store,"publish_workflow",original_publish)
            still_held=client.get(f"/protocol/jobs/{job}").json()
            assert still_held["operator"]["pending_review"] == pending
            assert still_held["execution"]["runtime_state"]["measurement_reviews"] == []
            assert sum(isinstance(c,tuple) for c in calls) == 1
        acknowledgment=client.post(f"/protocol/jobs/{job}/review",json=body)
        assert acknowledgment.status_code == 200,acknowledgment.text
        acknowledgments.append(body)
    done=await_job(client,job,lambda r:r["command"]["terminal"])
    assert done["command"]["status"] == "completed",done
    rows=done["execution"]["runtime_state"]["action_results"]
    analysis=rows[-1]["analysis"]
    assert analysis["points"][0]["bias"] == "-0.2500"
    assert analysis["thermal_application"] == "source_write_semantics_unresolved"
    assert analysis["correction"] is None and analysis["applied"] is False
    assert analysis["resolved_samples"][0]["measured_c"] == "30.9000"
    if bank == "nest":
        assert analysis["resolved_samples"][0]["controller_reported_c"] == 31.15
    assert sum(isinstance(c,tuple) for c in calls) == 2
    for body in acknowledgments:
        replay=client.post(f"/protocol/jobs/{job}/review",json=body)
        assert replay.status_code == 200,replay.text
    assert sum(isinstance(c,tuple) for c in calls) == 2
    app.state.operator_command_plane.stop()
    reopened=OperatorCommandStore(root)
    try:
        loaded=reopened.get_workflow(job)
        assert loaded["execution"]["runtime_state"] == done["execution"]["runtime_state"]
        assert len(loaded["operator"]["reviews"]) == 2
    finally:
        reopened.stop()
    schema=client.get("/protocol/calibration/schema")
    assert schema.status_code == 200,schema.text
    api_result=client.post("/protocol/calibration/analyze",json=pipette("exact_point_offset"))
    assert api_result.status_code == 200,api_result.text
    export(f"thermal-{module}-{bank}",{"request":request,"held":helds,"review_requests":acknowledgments,"completed":done,"reopened":loaded,
                                           "schema":schema.json(),"analysis_api":api_result.json(),"physical_leaf_calls":calls})
