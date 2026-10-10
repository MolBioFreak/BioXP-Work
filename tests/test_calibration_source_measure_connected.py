"""Actual native measure -> run-local calculate -> apply (inert board leaves)."""
from contextlib import nullcontext
import socket
import time

import pytest
from tests.test_oem_pipette_calibration import rig
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_calibration_workflow_connected import export
from tests.test_deck_tip_query_publication import bind_collection_test_identity
from oem_machine_bundle_test_support import bind_serial206_oem_snapshot
from bioxp.oem_compat.position_table import load_bound_oem_position_table as REAL_TABLE

_SOCKET = socket.socket


def test_source_measurement_connected_to_adjustment_and_durable_save(rig, monkeypatch, tmp_path):
    from bioxp import oem_machine_bundle as bundle
    from bioxp.oem_calibration_settings import CalibrationSettingsService
    from bioxp.oem_runtime_store import OEMRuntimeStore
    from bioxp.pipette.receipts import PipetteReceiptStore
    from bioxp.manual_pipetting import bind_manual_physical_handler
    from bioxp.oem_deck_movement import make_wp8_operation_executor
    from bioxp.operator_command_plane import OperatorCommandPlane, OperatorCommandStore
    from bioxp.operator_controls import _workflow_terminal_result
    from bioxp.protocols.compiler import compile_native_protocol
    from bioxp.protocols.executor import ProtocolExecutor
    from bioxp.calibration_workflow import calibration_handler
    from fastapi import FastAPI
    monkeypatch.setattr(socket,"socket",lambda family=socket.AF_INET,*a,**kw:
        _SOCKET(family,*a,**kw) if family==socket.AF_UNIX else pytest.fail("hardware/network socket"))
    bind_collection_test_identity(monkeypatch)
    snapshot=bind_serial206_oem_snapshot(monkeypatch)
    from bioxp.oem_compat import position_table
    fixture_table=position_table.load_bound_oem_position_table()
    monkeypatch.setattr(position_table,"load_bound_oem_position_table",lambda snapshot=None:
                        fixture_table if snapshot is None else REAL_TABLE(snapshot))
    store,p=rig.store,rig.provider
    stamps=p.deck_owner_authority_stamps()
    store.bind_deck_owner_authority_reader(lambda:stamps,scope=nullcontext)
    store.bind_workflow_dispatcher(lambda _:None)
    settings_store=OEMRuntimeStore(tmp_path)
    settings=CalibrationSettingsService(settings_store,snapshot)
    receipt_store=PipetteReceiptStore(tmp_path)
    for driver in rig.drivers:
        driver.stamp=time.monotonic()+.1
    # Real source measure at the fixture's current RC well, repeated twice.
    actions=[]
    for index in range(2):
        measurement=f"measure{index}"
        calc=f"calculate{index}"
        ref={"stage_id":"one","action_id":measurement,"source_occurrence_id":f"loop/{measurement}",
             "output":"result","path":["position_steps"]}
        actions.extend([
            {"kind":"pipette_manual_physical","action_id":measurement,"source_occurrence_id":f"loop/{measurement}","params":{"operation":"measure_fluid_height","speed":300}},
            {"kind":"calibration","action_id":calc,"source_occurrence_id":f"loop/{calc}","params":{"operation":"height_calculate","scope_id":"source-height","plate":"RC","measured_z":ref}},
            {"kind":"calibration","action_id":f"apply{index}","params":{"operation":"height_apply","calculation":{
                "stage_id":"one","action_id":calc,"source_occurrence_id":f"loop/{calc}","output":"result","path":["height_adjustment"]}}}])
    doc=compile_native_protocol({"protocol_id":"source-height","stages":[{"stage_id":"one","actions":actions}]})
    store.admit_workflow(command_id="measurement-parent",idempotency_key="measurement-parent",plan_fingerprint="fixture",
        requested_inputs={"bundle":{"protocol":{"document":doc.to_payload()}}},ownership_generation=1,
        resources=("axis:x","axis:y","axis:z","pipette"),board_epochs={})
    assert store.claim_next()["command_id"]=="measurement-parent"
    assert store._renew_owner(lease_seconds=120)
    native=make_wp8_operation_executor(provider_getter=lambda:p,command_store=store)
    admission={"ownership_generation":1,"serial206_initialization_provider":{
        "x_authority":{"current_board_lifecycle_generation":1},"board4_authority":{"active_board_epoch":1}}}
    plane=OperatorCommandPlane.__new__(OperatorCommandPlane)
    plane.app=FastAPI()
    plane.store=store
    plane.machine_state_provider=lambda:admission
    plane.app.state.oem_deck_provider=p
    plane.app.state.oem_wp8_operation_executor=native
    children=[]
    def execute(plan,action,state):
        admitted=store.admit_internal_wp8_operation(plan["operation"],inputs={},state=admission,
            idempotency_key=action.action_id,prepared_plan=plan)
        child=store.claim_next()
        assert child["command_id"]==admitted["command_id"]
        children.append(child["command_id"])
        plane._dispatch_one(child)
        return _workflow_terminal_result(store.get_command(child["command_id"]))
    manual=bind_manual_physical_handler(command_store=store,execute_plan=execute,
        require_motion_ready=lambda:None,provider_getter=lambda:p,receipt_store_getter=lambda:receipt_store)
    def publish(state):
        store.publish_workflow("measurement-parent",payload={"protocol":{"document":doc.to_payload()},"execution":{"runtime_state":state.to_payload()}})
    runtime=ProtocolExecutor(dry_run=False,job_id="measurement-parent",
        handlers={"pipette_manual_physical":manual,"calibration":calibration_handler(lambda:settings)},
        before_native_entry=lambda identity,state:store.assert_workflow_current("measurement-parent"),on_state_change=publish).execute(doc)
    export("source-height-before-assert", {"runtime":runtime.to_payload(),"children":[store.get_command(cid) for cid in children]})
    assert runtime.completed,runtime.to_payload()
    rows=runtime.action_results
    assert [rows[i]["position_steps"] for i in (0,3)]==[88000,88000]
    assert [rows[i]["height_adjustment"]["measured_raw_z"] for i in (1,4)]==[88000,88000]
    assert len(children)==2 and len(set(children))==2
    run_id=runtime.calibration_scopes["source-height"]
    run=settings.read_run(run_id)
    assert len(run["measurements"])==2
    assert run["measurements"][0]["saved_revision_id"]!=run["measurements"][1]["saved_revision_id"]
    assert settings.read()["saved_revision_id"]==rows[-1]["saved_revision_id"]
    terminal=store.finish_workflow("measurement-parent",status="completed",payload={"execution":{"runtime_state":runtime.to_payload()}},lifecycle_settled=True)
    assert terminal["command"]["status"]=="completed"
    reopened=OperatorCommandStore(tmp_path)
    try:
        assert reopened.get_workflow("measurement-parent")["execution"]["runtime_state"]==terminal["execution"]["runtime_state"]
        assert reopened.get_workflow("measurement-parent")["execution"]["runtime_state"]["calibration_scopes"]==runtime.calibration_scopes
    finally:
        reopened.stop()
    claims=receipt_store.connection.execute("SELECT command_id FROM pipette_operations").fetchall()
    assert len(claims)==6 and len({r[0] for r in claims})==6
    export("source-height",{"document":doc.to_payload(),"runtime":runtime.to_payload(),"run":run,"terminal":terminal,
                            "children":[store.get_command(cid) for cid in children],"receipt_count":len(claims)})
    receipt_store._audit_database.close()
    settings_store.close()


@pytest.mark.parametrize("termination",["abort","addressed_stop"])
def test_measurement_hold_preserves_prior_values_on_termination(installed_retained,monkeypatch,tmp_path,termination):
    from bioxp import api
    from tests.test_protocol_workflow_connected import request_payload,mount_protocol_routes,await_job
    from tests.test_calibration_measurements import prompt
    from fastapi.testclient import TestClient
    app,*_=installed_retained
    mount_protocol_routes(app)
    app.add_api_route("/protocol/jobs/{job_id}/control",api.protocol_job_control,methods=["POST"])
    client=TestClient(app)
    request=request_payload("interrupted-measurement",[prompt("p0",repetitions=1),prompt("p1",repetitions=1),{"kind":"note","action_id":"not-entered"}])
    response=client.post("/protocol/execute",json=request)
    assert response.status_code==202,response.text
    job=response.json()["job_id"]
    app.state.operator_command_plane.start()
    held=await_job(client,job,lambda r:bool(r["operator"].get("pending_review")))
    ack=client.post(f"/protocol/jobs/{job}/review",json={"command_id":job,"idempotency_key":"first-reading",
        "expected_ownership_generation":held["command"]["ownership_generation"],"stage_id":"one","action_id":"p0",
        "source_occurrence_id":"repeat/p0","measurements":{"probe":["3.0000"]}})
    assert ack.status_code==200,ack.text
    held=await_job(client,job,lambda r:(r["operator"].get("pending_review") or {}).get("action_id")=="p1")
    if termination=="abort":
        result=client.post(f"/protocol/jobs/{job}/control",json={"command_id":job,"idempotency_key":"abort-held",
            "expected_ownership_generation":held["command"]["ownership_generation"],"action":"abort"})
        assert result.status_code==200,result.text
    else:
        # Existing addressed Stop notification into this same executor; no
        # Stop transport is sent by the offline fixture.
        callback=app.state.operator_command_plane.store._workflow_controls[job]
        callback("fixture-addressed-stop",{"action":"_addressed_stop"})
    done=await_job(client,job,lambda r:r["command"]["terminal"])
    state=done["execution"]["runtime_state"]
    assert done["command"]["status"]!="completed"
    assert [r["action_id"] for r in state["action_results"]]==["p0","p1"]
    assert len(state["measurement_reviews"])==1
    assert state["measurement_reviews"][0]["measurements"]=={"probe":["3.0000"]}
    export("measurement-"+termination,{"request":request,"held":held,"terminal":done})
    app.state.operator_command_plane.stop()
