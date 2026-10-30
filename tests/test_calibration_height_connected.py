"""Custom calibration scope keeps OEM per-save and full-restore semantics."""
from dataclasses import replace

import pytest
from fastapi.testclient import TestClient
from tests.deck_scoped_rig import retained_rig
from tests.deck_scoped_installed import installed_retained
from tests.test_protocol_workflow_connected import request_payload, mount_protocol_routes, await_job
from tests.test_calibration_measurements import prompt, reference
from tests.test_calibration_workflow_connected import export
from oem_machine_bundle_test_support import bind_serial206_oem_snapshot


def calc(identity, measured, plate="RC"):
    return {"action_id":identity,"source_occurrence_id":"repeat/"+identity,"kind":"calibration",
            "params":{"operation":"height_calculate","scope_id":"calibration","plate":plate,"measured_z":measured}}


def apply(identity, previous):
    return {"action_id":identity,"kind":"calibration","params":{"operation":"height_apply","calculation":{
        "stage_id":"one","action_id":previous,"source_occurrence_id":"repeat/"+previous,
        "output":"result","path":["height_adjustment"]}}}


@pytest.mark.parametrize("machine_calibrated,prior",[(False,False),(True,False),(True,True)])
@pytest.mark.parametrize("partial",[False,True])
def test_composed_height_full_before_partial_save_later_edit_restore(installed_retained,monkeypatch,tmp_path,machine_calibrated,prior,partial):
    from bioxp import api
    from bioxp.oem_runtime_store import OEMRuntimeStore
    from bioxp.oem_calibration_settings import CalibrationSettingsService, CalibrationSettingsPatch, load_saved_calibration
    app,provider,primitives,references,root=installed_retained
    monkeypatch.setenv("BIOXP_PROTOCOL_JOBS_ROOT", str(tmp_path / "artifacts"))
    snapshot=bind_serial206_oem_snapshot(monkeypatch)
    snapshot=replace(snapshot,fields={**snapshot.fields,"machine.calibrated":replace(snapshot.fields["machine.calibrated"],value=machine_calibrated)})
    settings_store=OEMRuntimeStore(root)
    settings=CalibrationSettingsService(settings_store,snapshot)
    monkeypatch.setattr(api.app.state,"calibration_settings",settings,raising=False)
    if prior:
        settings.save(CalibrationSettingsPatch.model_validate({"positions":[{"name":"LOC_TC","zLow":7777}]}))
    before=settings.read()
    mount_protocol_routes(app)
    app.add_api_route("/motion/oem/calibration_settings/runs/{run_id}",api.calibration_run_read,methods=["GET"])
    app.add_api_route("/motion/oem/calibration_settings/runs/{run_id}/decision",api.calibration_run_decision,methods=["POST"])
    client=TestClient(app)
    p=prompt("height","pipette")
    p["params"]["measurement_form"]["fields"][0].update(unit="steps",target=None,commanded=None)
    actions=[p,calc("calc0",reference("height",0)),apply("apply0","calc0"),
             calc("calc1",reference("height",1),"STRIP"),apply("apply1","calc1")]
    if partial:
        actions += [calc("unavailable",reference("never-entered")), {"kind":"note","action_id":"not-entered"}]
    else:
        actions += [{"kind":"pause_review","action_id":"decision-note","pause_message":"Optional authored decision review"},
                    {"kind":"calibration","action_id":"restore","params":{"operation":"height_decision","scope_id":"calibration","decision":"restore"}}]
    request=request_payload(f"height-{machine_calibrated}-{prior}-{partial}",actions)
    response=client.post("/protocol/execute",json=request)
    assert response.status_code==202,response.text
    job=response.json()["job_id"]
    app.state.operator_command_plane.start()
    held=await_job(client,job,lambda r:bool(r["operator"].get("pending_review")))
    body={"command_id":job,"idempotency_key":"height-ack","expected_ownership_generation":held["command"]["ownership_generation"],
          "stage_id":"one","action_id":"height","source_occurrence_id":"repeat/height","measurements":{"probe":["88000",88100]}}
    response=client.post(f"/protocol/jobs/{job}/review",json=body)
    assert response.status_code==200,response.text
    reached=await_job(client,job,lambda r:r["command"]["terminal"] if partial else (r["operator"].get("pending_review") or {}).get("action_id")=="decision-note")
    state=reached["execution"]["runtime_state"]
    run_id=state["calibration_scopes"]["calibration"]
    run=settings.read_run(run_id)
    assert run["job_id"]==job
    assert run["before"]["revision_id"]==before["saved_revision_id"]
    assert len(run["measurements"])==2
    assert run["measurements"][0]["saved_revision_id"] != run["measurements"][1]["saved_revision_id"]
    assert [r["measured_raw_z"] for r in run["measurements"]]==[88000,88100]
    assert all(r["calculation_reference"]["source_occurrence_id"]==f"repeat/calc{i}" for i,r in enumerate(run["measurements"]))
    assert len(run["measurements"][1]["calculated_z_lows"])==4
    if partial:
        assert reached["command"]["status"]!="completed"
        assert state["action_results"][-1]["unavailable_operand_or_calculation"]
        assert state["action_results"][-1]["action_id"]=="unavailable"
        assert run["body_completed"] is False
    # Apply remains durable BEFORE a decision; later unrelated edits are neither
    # silently retained on restore nor used as a conflict/refusal gate.
    settings.save(CalibrationSettingsPatch.model_validate({"positions":[{"name":"LOC_OC","x":123456}]}))
    after_later=settings.read()
    reopened_store=OEMRuntimeStore(root)
    reopened=CalibrationSettingsService(reopened_store,load_saved_calibration(snapshot,reopened_store))
    monkeypatch.setattr(api.app.state,"calibration_settings",reopened)
    assert reopened.read_run(run_id)["after"]==run["after"]
    if partial:
        decision=client.post(f"/motion/oem/calibration_settings/runs/{run_id}/decision",json={"decision":"restore"})
        assert decision.status_code==200,decision.text
    else:
        accepted=client.post(f"/protocol/jobs/{job}/review",json={"command_id":job,"idempotency_key":"decision-ack",
            "expected_ownership_generation":reached["command"]["ownership_generation"],"stage_id":"one","action_id":"decision-note","note":"ordinary"})
        assert accepted.status_code==200,accepted.text
        reached=await_job(client,job,lambda r:r["command"]["terminal"])
        assert reached["command"]["status"]=="completed",reached
    final=client.get(f"/motion/oem/calibration_settings/runs/{run_id}").json()
    assert final["before"]==run["before"] and final["after"]==run["after"]
    if machine_calibrated:
        assert final["decision"]=="restore"
        assert reopened.read()["saved_revision_id"]==before["saved_revision_id"]
        assert reopened.read()["active_positions"]==before["active_positions"]
    else:
        assert final["decision"] is None and final["decision_status"]=="no_previous_values"
        assert reopened.read()["saved_revision_id"]==after_later["saved_revision_id"]
    export(f"height-{machine_calibrated}-{prior}-{partial}",{"request":request,"held":held,"response":body,"completed":reached,
        "before":before,"run_before_decision":run,"later_edit":after_later,"final_run":final,"settings_after":reopened.read()})
    app.state.operator_command_plane.stop()
    reopened_store.close()
    settings_store.close()
