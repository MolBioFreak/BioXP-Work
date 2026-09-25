"""Finite loadTips provider with real compiler, child adapters and transport-replaced IO."""
import pytest
import asyncio
import socket
from types import SimpleNamespace
_SOCKET = socket.socket
from tests.test_oem_pipette_calibration import rig
from bioxp.oem_deck_movement import compile_finite_plate_operation


def test_load_tips_missing_owner_does_not_return_success(rig):
    p = rig.provider
    plan = compile_finite_plate_operation("pipette_load_tips", source_leaf_available=True,
                                          tip_type=50, force_new_tip=True)
    p._wp8_execution_fence_checker = lambda *args, **kwargs: None
    with pytest.raises(RuntimeError, match="pipette_receipt_runner"):
        p._wp8_execute_nested_plan(plan=plan, command_id="offline-load",
            owner_identity={"source_identity": "offline-load"})


@pytest.mark.parametrize("rgb_false,initial_loaded,lost_z_once,camera_loss",
    [(rgb, loaded, lost, False) for rgb in (False, True)
        for loaded in (False, True) for lost in (False, True)] +
    [(False, False, False, True)])
def test_source_force_new_tip_provider_connected(rig, monkeypatch, tmp_path,
                                                 rgb_false, initial_loaded, lost_z_once, camera_loss):
    p = rig.provider
    from bioxp.pipette.receipts import PipetteReceiptStore
    from bioxp.services.pipette_service import run_pipette_operation
    from tests.test_deck_tip_query_publication import bind_collection_test_identity
    bind_collection_test_identity(monkeypatch)

    def local_socket(family=socket.AF_INET, *args, **kwargs):
        if family != socket.AF_UNIX:
            pytest.fail("network/hardware socket attempted")
        return _SOCKET(family, *args, **kwargs)
    monkeypatch.setattr(socket, "socket", local_socket)
    receipt_store = PipetteReceiptStore(tmp_path)
    async def inline(label, body, *, timeout_s):
        return body()
    def receipt(name, call, command_id, identity, inputs):
        return asyncio.run(run_pipette_operation(name, call,
            get_transport=lambda: rig.group, run_blocking=inline,
            receipt_store=receipt_store,
            requested_inputs={"source_occurrence_id": identity["source_identity"]},
            runtime_binding={"idempotency_key": identity["source_identity"],
                "entrypoint_id": "protocol.pipette_manual_physical", "caller_class": "protocol_manual",
                "parent_operator_command_id": command_id}))
    monkeypatch.setattr("bioxp.oem_machine_bundle.get_active_oem_machine_snapshot", lambda:
        SimpleNamespace(operation_parameters={"Mode": "WebMode"},
            fields={"machine.camera_installed": SimpleNamespace(value=camera_loss)}, camera_calibrated=camera_loss))
    monkeypatch.setattr("bioxp.oem_serial206_initialization.load_oem_parity_config",
                        lambda _: SimpleNamespace(blockers=[], values={}))
    p._wp8_execution_fence_checker = lambda *args, **kwargs: None
    p._manual_pipette_receipt_runner = receipt
    frames = []
    if camera_loss:
        from tests.test_pipette_check_tips_connected import image
        frames = [image(1), image()]
        settings = {"CheckForStaticTipLoss": True, "CameraXOffset": 0,
                    "CameraYOffset": 0, "InspectionSettings": {"ClungTips": {
                        "Exposure": 1000, "Gain": 1000, "LED1": False, "LED2": True,
                        "LED3": False, "Parameters": {"threshold": 100}}}}
        p.bind_oem_cover_inspection_callbacks(settings=lambda: settings,
            capture=lambda **_: {"frame": frames.pop(0), "capture_evidence": {"fixture": True}},
            save=lambda **_: {"artifact_saved": True, "fixture": True},
            led=lambda **_: None, rgb=lambda *_: None, barcode=lambda _: "")
        p.primitives.tester.motor_x_move_relative_strict = lambda steps: {"ok": True, "steps": steps}
        p.primitives.tester.motor_y_move_relative_strict = lambda steps: {"ok": True, "steps": steps}
        p.sleep = lambda _: None
    p._oem_pipette_rgb_writer = lambda *rgb: {"ok": not rgb_false}
    p._read_constructed_tip_tray = lambda i: {"tip_type": "T50" if i == 0 else "T200",
        "location": 7 + i, "construction_id": "fixture", "tip_available": True, "occupancy": [True] * 96}
    p.bind_tip_tray_state_reader(rig.store.tip_tray_state)
    p.bind_tip_tray_state_publisher(rig.store.publish_tip_tray_transition)
    rig.store.admit_workflow(command_id="offline-load", idempotency_key="offline-load",
        plan_fingerprint="load-tips", requested_inputs={}, ownership_generation=1,
        resources=("axis:x", "axis:y", "axis:z", "pipette"), board_epochs={})
    rig.store.claim_next()
    p.publish_tip_tray_transition(tray_id=0, transition="construct",
        operation_id="offline-load:construct", command_id="offline-load",
        provenance={"source_operation": "ClassMachineStatus.constructor"})
    # The source queries existing tips first, optionally ejects, then picks up.
    for t in rig.group._transports:
        t._tip_loaded = initial_loaded
    rig.group._tip_type = 50 if initial_loaded else 201
    calls = []
    ejected_at = {}
    for driver in rig.drivers:
        def query(d=driver):
            calls.append(d.channel)
            loaded = (len(calls) <= 8 or len(calls) > 12) if initial_loaded else len(calls) > 4
            if camera_loss and d.channel in ejected_at:
                loaded = len(rig.native.moves) > ejected_at[d.channel]
            d.events.append(("query", d.channel))
            return {"ok": True, "semantic_ok": True, "tip_loaded": loaded,
                "source_return_completed": True, "source_return": int(loaded),
                "source_tip_loaded": loaded, "hardware_truth_level": "hardware_query"}
        monkeypatch.setattr(driver, "query_tip_status", query)
        def eject(d=driver):
            ejected_at[d.channel] = len(rig.native.moves)
            d.events.append(("eject", d.channel))
            return {"ok": True, "outcome": "completed"}
        monkeypatch.setattr(driver, "pipette_eject_tip", eject, raising=False)
    homes = []
    if lost_z_once:
        original_home = rig.native.motor_oem_move_z_home
        def lost_home(*args, **kwargs):
            result = original_home(*args, **kwargs)
            homes.append(len(homes))
            return {**result, "source_return_code": 301 if len(homes) == 2 else 0}
        monkeypatch.setattr(rig.native, "motor_oem_move_z_home", lost_home)
    plan = compile_finite_plate_operation("pipette_load_tips", source_leaf_available=True,
                                          tip_type=50, force_new_tip=True)
    try:
        result = p._wp8_execute_nested_plan(plan=plan, command_id="offline-load",
            owner_identity={"source_identity": "offline-load"})
    except Exception as exc:
        if not camera_loss:
            pytest.fail(repr(getattr(exc, "evidence", exc)))
        failures = getattr(exc, "evidence", {}).get("failure_evidence", [])
        assert len(failures) == 1, failures
        body = failures[0]["result"]
    else:
        assert result["ok"] is True, result
        body = result["source_children"][0]["result"]
    assert "source_return" in body, {"body_keys": list(body), "body": str(body)[:700]}
    assert body["source_return"] is not camera_loss
    assert body["selected_tray"] == 0 and body["selected_group"] == 0
    assert calls[:8] == [0, 1, 2, 3] * 2
    assert len([event for event in rig.drivers[0].events if event[0] == "eject"]) == (4 if initial_loaded else 0) + (4 if camera_loss else 0)
    assert body["steps"][1]["result"]["source_children"][0]["ok"] is not rgb_false
    journal = receipt_store.read(limit=100)
    assert len(journal) == (4 if initial_loaded else 3) + int(lost_z_once) + int(camera_loss)
    assert len({row["command_id"] for row in receipt_store.connection.execute(
        "SELECT command_id FROM pipette_operations")}) == (4 if initial_loaded else 3) + int(lost_z_once) + int(camera_loss)
    assert rig.store.tip_tray_state(0)["occupancy"][0] is False
    names = [step["operation"] for step in body["steps"]]
    if initial_loaded:
        assert names.index("ejectAllTips") < names.index("MoveZHome.before")
    assert names.index("MoveZHome.before") < names.index("scriptmoveTo.tray") < names.index("lowerPipette") < names.index("MoveZHome.after") < names.index("removeTip.loaded")
    if lost_z_once:
        assert names.count("lowerPipette") == names.count("MoveZHome.after") == 2
        assert len(homes) >= 3
    if camera_loss:
        inspection = next(step["result"] for step in body["steps"] if step["operation"] == "checkTips")
        child = inspection["source_children"][0]["result"]
        assert child["source_return"] is False and child["inspection_completed"] is True
        assert len(child["captures"]) == 2 and not frames
        assert rig.store.tip_tray_state(0)["occupancy"][36] is False
        assert names.index("checkTips") < names.index("ejectAllTips.inspection") < names.index("setZaxisCurrentmax")
