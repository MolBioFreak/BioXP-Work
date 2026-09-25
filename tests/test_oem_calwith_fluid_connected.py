"""Native calibration worker under one claimed command; hardware transport replaced."""
import asyncio
import socket
from types import SimpleNamespace

import pytest

from tests.test_oem_pipette_calibration import rig
from bioxp.oem_calibration_settings import CalibrationSettingsService
from bioxp.oem_deck_movement import compile_finite_plate_operation
from bioxp.oem_job_preparation import construct_new_machine_source_model
from bioxp.oem_compat.position_table import load_bound_oem_position_table as original_table
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.pipette.receipts import PipetteReceiptStore
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


def test_native_calibration_retains_per_plate_saves_on_nested_motion_failure(rig, monkeypatch, tmp_path):
    from tests.test_deck_tip_query_publication import bind_collection_test_identity
    bind_collection_test_identity(monkeypatch)
    monkeypatch.setattr(socket, "socket", lambda family=socket.AF_INET, *a, **kw:
        _SOCKET(family, *a, **kw) if family == socket.AF_UNIX else pytest.fail("network socket"))
    snapshot = bind_serial206_oem_snapshot(monkeypatch)
    # The motion fixture's zero-argument PositionTable shortcut is not the
    # calibration service's snapshot projection. Preserve both callers.
    from bioxp.oem_compat import position_table
    motion_table = position_table.load_bound_oem_position_table()
    monkeypatch.setattr(position_table, "load_bound_oem_position_table",
                        lambda bound=None: original_table(bound) if bound is not None else motion_table)
    monkeypatch.setattr("bioxp.oem_serial206_initialization.load_oem_parity_config",
                        lambda _: SimpleNamespace(blockers=[], values={"GripperVersion": 1}))
    p = rig.provider
    p.mov_execution_machine_state = lambda: type(p).mov_execution_machine_state(p)
    p._wp8_execution_fence_checker = lambda *a, **kw: None
    p._oem_pipette_rgb_writer = lambda *a: {"ok": True}
    monkeypatch.setattr(rig.native, "motor_wait_stopped", lambda *a, **kw: {"ok": True, "stopped": True}, raising=False)
    p._manual_pipette_source_state = SimpleNamespace(source_model=construct_new_machine_source_model())
    p._manual_pipette_source_settings = {"LogPressure": False, "CheckForStaticTipLoss": False}
    p._manual_calibration_settings = CalibrationSettingsService(OEMRuntimeStore(tmp_path), snapshot)
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
    rig.store.admit_workflow(command_id="cal-parent", idempotency_key="cal-parent",
        plan_fingerprint="cal", requested_inputs={}, ownership_generation=1,
        resources=("axis:x", "axis:y", "axis:z", "pipette"), board_epochs={})
    rig.store.claim_next()
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
    plan = compile_finite_plate_operation("source_calwith_fluid", source_leaf_available=True)
    child = {**plan["children"][0], "_delivery_identity": {"source_identity": "cal-parent:source"}}
    body = p.execute_wp8_child(child, command_id="cal-parent", child_order=0,
                               plan_digest=plan["plan_digest"])
    assert body["ok"] is False and body["outcome"] == "incomplete"
    assert body["error"], body
    assert [m["plate"] for m in body["measurements"]] == ["TC", "MS"]
    assert len({m["saved_revision_id"] for m in body["measurements"]}) == 2
    assert body["measurements"][0]["measured_raw_z"] == 88000
    assert body["saved_revision_id"] == body["measurements"][-1]["saved_revision_id"]
    assert body["comparison_choice"] is None and body["comparison_gap"]
    assert p._manual_calibration_settings.read()["saved_revision_id"] == body["saved_revision_id"]
    assert p._manual_calibration_settings.read()["pending_restart"]
    assert [e["operation"] for e in body["source_events"]][:9] == [
        "setMaxAcc(z):outer", "setFileName", *(["pipette_tip_transition"] * 5),
        "resetStatus", "press_plates"]
    keys = [row[0] for row in receipts.connection.execute("SELECT command_id FROM pipette_operations")]
    assert len(keys) == len(set(keys)) and len(keys) > 4
    assert rig.store.connection.execute("SELECT count(*) FROM operator_commands WHERE command_id=?",
        ("cal-parent",)).fetchone()[0] == 1
    p._manual_calibration_settings.store.close()
    receipts.connection.close()
