"""Offline diagnostic composition through real finite provider and SQLite owners."""
import socket
from contextlib import nullcontext
from types import SimpleNamespace

import pytest

from bioxp.manual_pipetting import (compile_manual_pipetting, manual_physical_plan,
                                    bind_manual_physical_handler)
from bioxp.oem_deck_movement import make_wp8_operation_executor
from bioxp.oem_job_preparation import construct_new_machine_source_model
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.protocols.runtime_state import ProtocolRuntimeState
from tests.test_oem_pipette_calibration import rig

_SOCKET = socket.socket


def test_typed_diagnostic_is_one_finite_child():
    doc = compile_manual_pipetting({"protocol_id": "diagnostic", "steps": [
        {"operation": "diagnostic_detect_fluid"}]})
    action = doc.stages[0].actions[0]
    assert action.kind.value == "pipette_manual_physical"
    plan = manual_physical_plan(action.params)
    assert [child["operation"] for child in plan["children"]] == ["sourceDiagnosticDetectFluid"]
    with pytest.raises(ValueError):
        manual_physical_plan({"operation": "diagnostic_detect_fluid", "skip_steps": 12})


def test_diagnostic_void_initiate_continues_pressure_sequence_after_wait_false(rig, monkeypatch):
    streams = []
    for driver in rig.drivers:
        monkeypatch.setattr(driver, "pipette_initiate_group", lambda d=driver: {
            **d.issue("WR"), "immediate_ack_received": True}, raising=False)
        monkeypatch.setattr(driver, "wait_pipette_initialization_completion",
                            lambda timeout: {"ok": False}, raising=False)
        monkeypatch.setattr(driver, "enable_pressure_stream",
                            lambda on, d=driver: (streams.append((d.channel, on)) or {"ok": True}), raising=False)
    result = rig.group.initiate_group_once_for_oem_detect_fluid()
    assert result["ok"] is False
    assert result["outcome"] == "group_completion_timeout_or_error"
    assert [row["result"]["ok"] for row in result["delayed_completions"]] == [False] * 4
    assert streams == [(channel, True) for channel in range(4)] + [(channel, False) for channel in range(4)]


@pytest.mark.parametrize("failure", [None, "full_inventory", "initiate_boolean_false", "second_catch", "first_release", "third_scan", "outer_exception"])
def test_diagnostic_real_provider_sqlite_and_transport_seams(rig, monkeypatch, tmp_path, failure):
    from tests.test_deck_tip_query_publication import bind_collection_test_identity
    bind_collection_test_identity(monkeypatch)
    monkeypatch.setattr(socket, "socket", lambda family=socket.AF_INET, *a, **kw:
        _SOCKET(family, *a, **kw) if family == socket.AF_UNIX else pytest.fail("network socket"))
    from oem_machine_bundle_test_support import bind_serial206_oem_snapshot
    bind_serial206_oem_snapshot(monkeypatch)
    monkeypatch.setattr("bioxp.oem_serial206_initialization.load_oem_parity_config",
                        lambda _: SimpleNamespace(blockers=[], values={}))
    p = rig.provider
    rig.store.bind_deck_owner_authority_reader(p.deck_owner_authority_stamps, scope=nullcontext)
    rig.native.motor_wait_stopped = lambda *a, **kw: {"ok": True, "stopped": True}
    from bioxp.oem_deck_movement import OEM_PLATE_NAME_ORDINALS
    from bioxp.oem_compat.pathing import LOCATION_ID_TO_NAME
    rig.store.publish_deck_owner_state(source_operation="updatePlateLocation", source_command_id="detect-parent:inventory",
        updates={"movable_plate_locations": {"POOL_PLATE": "LOC_P_TC",
            "OUTPUT_PLATE": "LOC_P_OC", "REAGENT_PLATE": "LOC_RC"}},
        **p.deck_owner_authority_stamps())
    names = {value: key for key, value in LOCATION_ID_TO_NAME.items()}
    def facts():
        semantic = rig.store.deck_semantic_state()
        return {**semantic, "current_location": names[semantic["current_location"]],
            "plate_locations": {OEM_PLATE_NAME_ORDINALS[key]: names[value]
                for key, value in semantic["movable_plate_locations"].items()
                if key in OEM_PLATE_NAME_ORDINALS},
            "thermal_door_open": False, "gripper_version": 1}
    p.mov_execution_machine_state = facts
    p._offset_deck_semantic_state = lambda **_: facts()
    p._wp8_execution_fence_checker = lambda *a, **kw: None
    p.bind_pipette_collection_state_reader(lambda: {"tip_exists": False})
    p.wp8_update_thermal_door_open("updateThermalDoorOpen", {"value": True},
        command_id="detect-parent:door", child_order=0, plan_digest="fixture")
    rig.store.publish_deck_owner_state(source_operation="sourceUnlatch",
        source_command_id="detect-parent:unlatch", updates={"latch_closed": False},
        **p.deck_owner_authority_stamps())
    p._oem_pipette_rgb_writer = lambda *a: {"ok": True}
    p._manual_pipette_source_state = SimpleNamespace(source_model=construct_new_machine_source_model())
    p._manual_pipette_source_settings = {"LogPressure": False, "CheckForStaticTipLoss": False}
    p._read_constructed_tip_tray = lambda i: {"tip_type": "T50" if i in (0, 1) or (i == 2 and failure == "third_scan") or failure == "full_inventory" else "T200",
        "location": [7, 8, 9, 10, 15][i], "construction_id": "fixture", "tip_available": True,
        "occupancy": [True] * 96}
    p.bind_tip_tray_state_reader(rig.store.tip_tray_state)
    p.bind_tip_tray_state_publisher(rig.store.publish_tip_tray_transition)
    receipts = PipetteReceiptStore(tmp_path)
    rig.store.bind_workflow_dispatcher(lambda claimed: None)
    rig.store.admit_workflow(command_id="detect-parent", idempotency_key="detect-parent",
        plan_fingerprint="diagnostic", requested_inputs={"bundle": {"execution": {"runtime_state": {}}}}, ownership_generation=1,
        resources=("axis:x", "axis:y", "axis:z", "pipette"), board_epochs={})
    rig.store.claim_next()
    # The production worker heartbeats this lease; the offline test runs inline.
    assert rig.store._renew_owner(lease_seconds=120.0)
    p.publish_tip_tray_transition(tray_id=0, transition="construct",
        operation_id="detect-parent:construct", command_id="detect-parent",
        provenance={"source_operation": "ClassMachineStatus.constructor"})
    p.publish_tip_tray_transition(tray_id=1, transition="construct",
        operation_id="detect-parent:construct:1", command_id="detect-parent",
        provenance={"source_operation": "ClassMachineStatus.constructor"})
    if failure == "full_inventory":
        # Diagnostic inventory is declared separately from calibration reset:
        # all four source-addressable trays contain T50 tips.
        for tray in p._manual_pipette_source_state.source_model.tip_trays:
            tray.tip_type = 50
            for well in tray.wells:
                well.empty = False
        for tray_id in (2, 3, 4):
            p.publish_tip_tray_transition(tray_id=tray_id, transition="construct",
                operation_id=f"detect-parent:construct:{tray_id}", command_id="detect-parent",
                provenance={"source_operation": "ClassMachineStatus.constructor"})
    for t in rig.group._transports:
        t._tip_loaded = False
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
        monkeypatch.setattr(driver, "pipette_initiate_group", lambda d=driver: {
            **d.issue("WR"), "immediate_ack_received": True}, raising=False)
        monkeypatch.setattr(driver, "wait_pipette_initialization_completion", lambda timeout: {"ok": True}, raising=False)
        monkeypatch.setattr(driver, "enable_pressure_stream", lambda on: {"ok": True}, raising=False)
    if failure == "second_catch":
        original_move = rig.native.motor_oem_move_absolute
        def fail_during_second_catch(board, target, *, motor, **kwargs):
            if (board, motor) == (4, 2) and target == 30350 and any(
                    move[:3] == ("g", 30350, True) for move in rig.native.moves):
                raise RuntimeError("injected_second_catch_native_failure")
            return original_move(board, target, motor=motor, **kwargs)
        rig.native.motor_oem_move_absolute = fail_during_second_catch
    if failure == "first_release":
        # Observe the composing adapter, replacing only the native IO beneath
        # its first release. The compiler/suppression policy remains real.
        active = []
        compile_execute = p._wp8_compile_and_execute
        def observed_finite(**kwargs):
            active.append(kwargs["operation"])
            try:
                return compile_execute(**kwargs)
            finally:
                active.pop()
        monkeypatch.setattr(p, "_wp8_compile_and_execute", observed_finite)
        native_move = rig.native.motor_oem_move_absolute
        injected = []
        def release_failure(*args, **kwargs):
            if active == ["release_plate"] and not injected:
                injected.append(True)
                raise LookupError("injected_first_release_native_failure")
            return native_move(*args, **kwargs)
        monkeypatch.setattr(rig.native, "motor_oem_move_absolute", release_failure)
    if failure == "outer_exception":
        def failed_initiate():
            raise LookupError("injected_initiate_outer_exception")
        monkeypatch.setattr(rig.drivers[0], "pipette_initiate_group", failed_initiate)
    if failure == "third_scan":
        p.publish_tip_tray_transition(tray_id=2, transition="construct",
            operation_id="detect-parent:construct:2", command_id="detect-parent",
            provenance={"source_operation": "ClassMachineStatus.constructor"})
        driver = rig.drivers[2]
        original_wait = driver.wait_pipette_command_completion
        press_waits = []
        rig.native.motor_wait_stopped = lambda *a, **kw: (press_waits.append(True) or {"ok": True, "stopped": True})
        def fail_third_scan(timeout, *, owner_token):
            result = original_wait(timeout, owner_token=owner_token)
            if owner_token.startswith("BR") and press_waits:
                return {**result, "ok": False}
            return result
        monkeypatch.setattr(driver, "wait_pipette_command_completion", fail_third_scan)
    if failure == "initiate_boolean_false":
        monkeypatch.setattr(rig.group, "initiate_group_once_for_oem_detect_fluid",
            lambda: {"ok": False, "cycle": "detectFluid.initiateGroup",
                     "outcome": "source_ignored_false_return"})

    doc = compile_manual_pipetting({"protocol_id": "detect", "steps": [
        {"operation": "diagnostic_detect_fluid"}]})
    runtime = ProtocolRuntimeState.from_document(doc, dry_run=False, job_id="detect-parent")
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
    child_ids = []
    def execute(compiled, action, state):
        admitted = rig.store.admit_internal_wp8_operation(compiled["operation"], inputs={},
            state=admission, idempotency_key=action.action_id, prepared_plan=compiled)
        child_id = admitted["command_id"]
        child_ids.append(child_id)
        claimed = rig.store.claim_next()
        assert claimed["command_id"] == child_id
        plane._dispatch_one(claimed)
        terminal = rig.store.get_command(child_id)
        import json
        assert len(json.dumps(terminal["terminal_evidence"])) < 131072
        return _workflow_terminal_result(terminal)
    handler = bind_manual_physical_handler(command_store=rig.store,
        execute_plan=execute,
        require_motion_ready=lambda: None, provider_getter=lambda: p,
        receipt_store_getter=lambda: receipts)
    from bioxp.protocols.executor import ProtocolExecutor
    from bioxp.protocols.models import ProtocolActionKind
    def publish(state):
        rig.store.publish_workflow("detect-parent", payload={"execution": {"runtime_state": state.to_payload()}})
    runtime = ProtocolExecutor(dry_run=False, job_id="detect-parent",
        before_native_entry=lambda identity, state: rig.store.assert_workflow_current("detect-parent"),
        on_state_change=publish,
        handlers={ProtocolActionKind.PIPETTE_MANUAL_PHYSICAL: handler}).execute(doc, state=runtime)
    result = next(row for row in runtime.action_results if row.get("action_id") == doc.stages[0].actions[0].action_id)
    retained = rig.store.get_workflow("detect-parent")["execution"]["runtime_state"]["action_results"]
    assert next(row for row in retained if "pipette_result" in row)["pipette_result"] == result["pipette_result"]
    body = result["pipette_result"]
    assert body["kind"] == "diagnostic_detect_fluid"
    assert body == result["receipt"]["terminal_evidence"]["pipette_result"]
    # Default inventory deliberately has only two T50 trays. Even the fully
    # stocked case must retain true source exhaustion through SQLite/workflow.
    assert body["completed"] is False
    assert result["calibration_persisted"] is False
    assert not any(e["operation"] == "park_gantry" for e in body["events"])
    operations = [e["operation"] for e in body["events"]]
    if failure == "full_inventory":
        # Source loadTips searches four trays, not the hotel. TC/MS/OC/RC
        # consume their 96 four-tip groups; STRIP needs six more. Do not turn
        # true stock exhaustion into an ignored Boolean or invent replenishment.
        assert [s["plate"] for s in body["scans"]] == ["TC", "MS", "OC", "RC"]
        assert operations[-1] == "zOffset:STRIP"
        assert body["error"] == "Tips are not available"
    elif failure == "outer_exception":
        assert "injected_initiate_outer_exception" in body["error"]
        assert not body["scans"]
        assert not any(op.startswith("zOffset:") for op in operations)
    else:
        assert [s["plate"] for s in body["scans"]] == ["TC", "MS"]
        assert operations[-1] == "zOffset:OC"
        if failure == "third_scan":
            assert body["error"] != "The required tip is not loaded!"
            assert "sourceMeasureFluidHeight" in str(body["events"][-1])
        else:
            assert body["error"] == "The required tip is not loaded!"
        if failure in {"second_catch", "first_release"}:
            failed_name = "catch_plate" if failure == "second_catch" else "release_plate"
            suppressed = [e for e in body["events"] if e["operation"] == failed_name
                          and e.get("result", {}).get("exception_suppressed")]
            assert len(suppressed) == 1
            assert suppressed[0]["result"]["ok"] is False
            assert f"injected_{failure}_native_failure" in str(suppressed[0]["errors"])
            assert body["scans"][1]["plate"] == "MS"
        if failure == "initiate_boolean_false":
            assert [e["result"]["ok"] for e in body["events"]
                    if e["operation"] == "initiateGroup"] == [False]
    assert receipts.read(limit=1000)
    keys = [row[0] for row in receipts.connection.execute("SELECT command_id FROM pipette_operations")]
    assert len(keys) == len(set(keys))
    ids = [e["source_identity"] for e in body["events"] if "source_identity" in e]
    assert len(ids) == len(set(ids))
    assert rig.store.wp8_operation_evidence(child_ids[0])["children"][0]["operation"] == "sourceDiagnosticDetectFluid"
