"""Native calibration worker under one claimed command; hardware transport replaced."""
import asyncio
import socket
from contextlib import nullcontext
from dataclasses import replace
from types import SimpleNamespace

import pytest

from tests.test_oem_pipette_calibration import rig
from bioxp.oem_calibration_settings import CalibrationSettingsService
from bioxp.oem_deck_movement import compile_finite_plate_operation, make_wp8_operation_executor
from bioxp.oem_job_preparation import construct_new_machine_source_model
from bioxp.oem_compat.position_table import load_bound_oem_position_table as original_table
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.manual_pipetting import bind_manual_physical_handler, compile_manual_pipetting
from bioxp.protocols.runtime_state import ProtocolRuntimeState
from bioxp.services.pipette_service import run_pipette_operation
from oem_machine_bundle_test_support import bind_serial206_oem_snapshot

_SOCKET = socket.socket


def test_calibration_compiles_one_owner_and_rejects_untyped_arguments():
    from bioxp.manual_pipetting import compile_manual_pipetting, manual_physical_plan
    from bioxp.protocols.models import ProtocolActionKind
    doc = compile_manual_pipetting({"protocol_id": "cal", "steps": [{"operation": "source_calwith_fluid"}]})
    action = doc.stages[0].actions[0]
    assert action.kind == ProtocolActionKind.PIPETTE_MANUAL_PHYSICAL
    assert [c["operation"] for c in manual_physical_plan(action.params)["children"]] == ["sourceCalwithFluid"]
    with pytest.raises(ValueError):
        manual_physical_plan({"operation": "source_calwith_fluid", "comparison_choice": True})


@pytest.mark.parametrize("inject_failure", [False, True, "finalization"])
@pytest.mark.parametrize("machine_calibrated", [True, False])
def test_native_calibration_five_saves_and_partial_failure(rig, monkeypatch, tmp_path, inject_failure, machine_calibrated):
    from tests.test_deck_tip_query_publication import bind_collection_test_identity
    bind_collection_test_identity(monkeypatch)
    monkeypatch.setattr(socket, "socket", lambda family=socket.AF_INET, *a, **kw:
        _SOCKET(family, *a, **kw) if family == socket.AF_UNIX else pytest.fail("network socket"))
    snapshot = bind_serial206_oem_snapshot(monkeypatch)
    snapshot = replace(snapshot, fields={**snapshot.fields, "machine.calibrated":
        replace(snapshot.fields["machine.calibrated"], value=machine_calibrated)})
    # Exercise the actual active geometry through every provider consumer.
    from bioxp import oem_machine_bundle as bundle
    bundle.apply_owned_calibration_snapshot(snapshot)
    monkeypatch.setattr("bioxp.oem_compat.position_table.load_bound_oem_position_table", original_table)
    monkeypatch.setattr("bioxp.oem_serial206_initialization.load_bound_oem_position_table", original_table)
    monkeypatch.setattr("bioxp.oem_serial206_initialization.load_oem_parity_config",
                        lambda _: SimpleNamespace(blockers=[], values={}))
    p = rig.provider
    rig.store.bind_deck_owner_authority_reader(p.deck_owner_authority_stamps, scope=nullcontext)
    from bioxp.oem_deck_movement import OEM_PLATE_NAME_ORDINALS
    from bioxp.oem_compat.pathing import LOCATION_ID_TO_NAME
    names = {value: key for key, value in LOCATION_ID_TO_NAME.items()}
    def facts():
        semantic = rig.store.deck_semantic_state()
        return {**semantic, "current_location": names[semantic["current_location"]],
            "plate_locations": {OEM_PLATE_NAME_ORDINALS[key]: names[value]
                for key, value in semantic["movable_plate_locations"].items()
                if key in OEM_PLATE_NAME_ORDINALS},
            "thermal_door_open": True, "gripper_version": 1}
    p.mov_execution_machine_state = facts
    p._offset_deck_semantic_state = lambda **_: facts()
    p._wp8_execution_fence_checker = lambda *a, **kw: None
    p._oem_pipette_rgb_writer = lambda *a: {"ok": True}
    monkeypatch.setattr(rig.native, "motor_wait_stopped", lambda *a, **kw: {"ok": True, "stopped": True}, raising=False)
    p._manual_pipette_source_state = SimpleNamespace(source_model=construct_new_machine_source_model())
    p._manual_pipette_source_settings = {"LogPressure": False, "CheckForStaticTipLoss": False}
    p._manual_calibration_settings = CalibrationSettingsService(OEMRuntimeStore(tmp_path), snapshot,
        publish=bundle.apply_owned_calibration_snapshot)
    for tray in p._manual_pipette_source_state.source_model.tip_trays:
        tray.tip_type = 201
        tray.location = 6
    from bioxp.serial206_y_provider import Serial206YProvider
    p.primitives.bind_y_provider(Serial206YProvider(rig.native,
        state_store=p._manual_calibration_settings.store, generation_provider=lambda: 1))
    p._read_constructed_tip_tray = lambda i: {"tip_type": "T50" if i == 0 else "T200",
        "location": 7 + i, "construction_id": "fixture", "tip_available": True,
        "occupancy": [True] * 96}
    p.bind_tip_tray_state_reader(rig.store.tip_tray_state)
    p.bind_tip_tray_state_publisher(rig.store.publish_tip_tray_transition)
    receipts = PipetteReceiptStore(tmp_path)
    p.bind_pipette_collection_state_reader(lambda: {"tip_exists": False})
    rig.store.publish_deck_owner_state(source_operation="sourceUnlatch",
        source_command_id="cal-parent:fixture-latch", updates={"latch_closed": False},
        ownership_generation=1, board_epoch_4=1, board_epoch_5=1)
    rig.store.publish_deck_owner_state(source_operation="updatePlateLocation",
        source_command_id="cal-parent:fixture-plate", updates={
            "movable_plate_locations": {"POOL_PLATE": "LOC_P_TC"}},
        ownership_generation=1, board_epoch_4=1, board_epoch_5=1)
    rig.store.publish_deck_owner_state(source_operation="updateThermalDoorOpen",
        source_command_id="cal-parent:fixture-door", updates={"thermal_door_open": True},
        ownership_generation=1, board_epoch_4=1, board_epoch_5=1)
    p.wp8_update_thermal_door_open("updateThermalDoorOpen", {"value": True},
        command_id="cal-parent:door", child_order=0, plan_digest="fixture")
    async def inline(label, body, *, timeout_s):
        return body()
    def receipt(name, call, command_id, identity, inputs):
        return asyncio.run(run_pipette_operation(name, call, get_transport=lambda: rig.group,
            run_blocking=inline, receipt_store=receipts,
            requested_inputs={"source_occurrence_id": identity["source_identity"]},
            runtime_binding={"idempotency_key": identity["source_identity"],
                "entrypoint_id": "protocol.pipette_manual_physical", "caller_class": "protocol_manual",
                "parent_operator_command_id": command_id}))
    p._manual_pipette_receipt_runner = receipt
    rig.store.bind_workflow_dispatcher(lambda claimed: None)
    rig.store.admit_workflow(command_id="cal-parent", idempotency_key="cal-parent",
        plan_fingerprint="cal", requested_inputs={"bundle": {"execution": {"runtime_state": {}}}}, ownership_generation=1,
        resources=("axis:x", "axis:y", "axis:z", "pipette"), board_epochs={})
    rig.store.claim_next()
    assert rig.store._renew_owner(lease_seconds=120.0)
    p.publish_tip_tray_transition(tray_id=0, transition="construct",
        operation_id="cal-parent:construct", command_id="cal-parent",
        provenance={"source_operation": "ClassMachineStatus.constructor"})
    for transport in rig.group._transports:
        transport._tip_loaded = False
    rig.group._tip_type = 201
    calls = []
    ejected_at = {}
    for driver in rig.drivers:
        def query(d=driver):
            calls.append(d.channel)
            loaded = len(calls) > 4 and len(rig.native.moves) > ejected_at.get(d.channel, -1)
            return {"ok": True, "semantic_ok": True, "tip_loaded": loaded,
                    "source_return_completed": True, "source_return": int(loaded),
                    "source_tip_loaded": loaded, "hardware_truth_level": "hardware_query"}
        def eject(d=driver, **kwargs):
            ejected_at[d.channel] = len(rig.native.moves)
            return d.issue("eject", **kwargs)
        monkeypatch.setattr(driver, "query_tip_status", query)
        monkeypatch.setattr(driver, "pipette_eject_tip", eject, raising=False)
        monkeypatch.setattr(driver, "aspirate", lambda volume, d=driver, **kw:
            d.issue("aspirate", volume=volume, **kw), raising=False)
        monkeypatch.setattr(driver, "dispense", lambda volume, d=driver, **kw:
            d.issue("dispense_liquid", volume=volume, **kw), raising=False)
    if inject_failure == "finalization":
        original_acc = p.primitives.z_set_max_acc
        def fail_final_acc(value):
            if value == 576:
                raise RuntimeError("transport final acceleration failure")
            return original_acc(value)
        monkeypatch.setattr(p.primitives, "z_set_max_acc", fail_final_acc)
    if inject_failure is True:
        saved = []
        service = p._manual_calibration_settings
        original_save = service.save
        def saving(*args, **kwargs):
            result = original_save(*args, **kwargs)
            saved.append(result["saved_revision_id"])
            return result
        monkeypatch.setattr(service, "save", saving)
        original_wait = rig.drivers[2].wait_pipette_command_completion
        def fail_third_scan(timeout, *, owner_token):
            result = original_wait(timeout, owner_token=owner_token)
            return {**result, "ok": False} if owner_token.startswith("BR") and len(saved) >= 2 else result
        monkeypatch.setattr(rig.drivers[2], "wait_pipette_command_completion", fail_third_scan)
    doc = compile_manual_pipetting({"protocol_id": "cal", "steps": [
        {"operation": "source_calwith_fluid"}]})
    runtime = ProtocolRuntimeState.from_document(doc, dry_run=False, job_id="cal-parent")
    monkeypatch.setattr("bioxp.runtime_state.get_active_oem_runtime_state_store", lambda: object())
    monkeypatch.setattr("bioxp.pipette.manual_settings.read_pipette_operation_settings",
        lambda _: {"runtime_values": {"LogPressure": False,
                                      "CheckForStaticTipLoss": False}})
    execute_wp8 = make_wp8_operation_executor(provider_getter=lambda: p, command_store=rig.store)
    admission = {"ownership_generation": 1, "serial206_initialization_provider": {
        "x_authority": {"current_board_lifecycle_generation": 1},
        "board4_authority": {"active_board_epoch": 1}}}
    from bioxp.operator_command_plane import OperatorCommandPlane
    from bioxp.operator_controls import _workflow_terminal_result
    from fastapi import FastAPI
    plane = OperatorCommandPlane.__new__(OperatorCommandPlane)
    plane.app = FastAPI()
    plane.store = rig.store
    plane.machine_state_provider = lambda: admission
    plane.app.state.oem_deck_provider = p
    plane.app.state.oem_wp8_operation_executor = execute_wp8
    def execute(compiled, action, state):
        admitted = rig.store.admit_internal_wp8_operation(compiled["operation"], inputs={},
            state=admission, idempotency_key=action.action_id, prepared_plan=compiled)
        claimed = rig.store.claim_next()
        assert claimed["command_id"] == admitted["command_id"]
        plane._dispatch_one(claimed)
        terminal = rig.store.get_command(claimed["command_id"])
        response = _workflow_terminal_result(terminal)
        assert response["pipette_result"] == terminal["terminal_evidence"]["response"]["pipette_result"]
        assert response["status"] == ("ambiguous" if inject_failure else "completed")
        import json
        assert len(json.dumps(terminal["terminal_evidence"])) < 131072
        canonical = rig.store.wp8_operation_evidence(claimed["command_id"])
        child = canonical["children"][0]
        child_result = json.loads(child["terminal_evidence_json"])["result"]
        assert child_result["measurements"] == response["pipette_result"]["measurements"]
        assert "source_events" not in child_result
        if not inject_failure:
            operation_result = json.loads(canonical["operation"]["terminal_result_json"])
            assert operation_result["pipette_result"] == response["pipette_result"]
            assert len(canonical["operation"]["terminal_result_json"]) < 131072
        outbox = rig.store.connection.execute(
            "SELECT payload_json FROM operator_plane_outbox WHERE command_id=? ORDER BY transition_sequence DESC LIMIT 1",
            (claimed["command_id"],)).fetchone()
        assert json.loads(outbox[0])["pipette_result"] == response["pipette_result"]
        return response
    handler = bind_manual_physical_handler(command_store=rig.store, execute_plan=execute,
        require_motion_ready=lambda: None, provider_getter=lambda: p,
        receipt_store_getter=lambda: receipts,
        calibration_settings_getter=lambda: p._manual_calibration_settings)
    from bioxp.protocols.executor import ProtocolExecutor
    from bioxp.protocols.models import ProtocolActionKind
    def publish(state):
        rig.store.publish_workflow("cal-parent", payload={"execution": {"runtime_state": state.to_payload()}})
    runtime = ProtocolExecutor(dry_run=False, job_id="cal-parent",
        before_native_entry=lambda identity, state: rig.store.assert_workflow_current("cal-parent"),
        on_state_change=publish,
        handlers={ProtocolActionKind.PIPETTE_MANUAL_PHYSICAL: handler}).execute(doc, state=runtime)
    result = next(row for row in runtime.action_results if row.get("action_id") == doc.stages[0].actions[0].action_id)
    retained = rig.store.get_workflow("cal-parent")["execution"]["runtime_state"]["action_results"]
    assert next(row for row in retained if "pipette_result" in row)["pipette_result"] == result["pipette_result"]
    assert result["calibration_persisted"] is True
    body = result["pipette_result"]
    assert body["kind"] == "source_calwith_fluid"
    assert body["ok"] is (not inject_failure), body
    assert [m["plate"] for m in body["measurements"]] == (
        ["TC", "MS"] if inject_failure is True else ["TC", "MS", "OC", "RC", "STRIP"])
    if inject_failure is True:
        assert body["error"] and body["outcome"] == ("incomplete" if machine_calibrated else "accepted_no_previous_values")
    else:
        assert body["body_completed"] and not body.get("error")
        assert not body["pending_restart"]
        assert p._manual_pipette_source_state.source_model.strips[1].strip_color == "X"
    assert len({m["saved_revision_id"] for m in body["measurements"]}) == len(body["measurements"])
    assert body["measurements"][0]["measured_raw_z"] == 88000
    assert body["saved_revision_id"] == body["measurements"][-1]["saved_revision_id"]
    assert body["comparison_choice"] is (None if machine_calibrated else True)
    if machine_calibrated:
        assert body["comparison_gap"]
    else:
        assert body["comparison_gap"] is None
        if not inject_failure:
            assert body["outcome"] == "accepted_no_previous_values"
    assert p._manual_calibration_settings.read()["saved_revision_id"] == body["saved_revision_id"]
    assert not p._manual_calibration_settings.read()["pending_restart"]
    if inject_failure == "finalization":
        assert body["finalization_error"] == "transport final acceleration failure"
        assert body["body_completed"] is True
    assert "source_events" not in body
    assert all("steps" not in m["scan"] for m in body["measurements"])
    keys = [row[0] for row in receipts.connection.execute("SELECT command_id FROM pipette_operations")]
    assert len(keys) == len(set(keys)) and len(keys) > 4
    assert rig.store.connection.execute("SELECT count(*) FROM operator_commands WHERE command_id=?",
        ("cal-parent",)).fetchone()[0] == 1
    model = p._manual_pipette_source_state.source_model
    assert [tray.tip_type for tray in model.tip_trays] == [50, 50, 50, 200, 50]
    assert [tray.location for tray in model.tip_trays] == [7, 8, 9, 10, 15]
    persisted = p._manual_calibration_settings.read_run(body["run_id"])
    assert persisted["body_completed"] is body["body_completed"]
    assert persisted["active_revision_id"] == body["saved_revision_id"]
    for measurement in body["measurements"]:
        for name, low in measurement["calculated_z_lows"].items():
            assert original_table().resolve(location_id=name).z_low == low
    p._manual_calibration_settings.store.close()
    receipts.connection.close()
