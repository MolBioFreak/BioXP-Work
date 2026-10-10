"""Application coordination through actual native owners; physical IO is inert."""
import copy
import json
import os
from pathlib import Path
import pytest
from bioxp import api
from bioxp.pipette.cavro_application import ApplicationRequest, compile_application, capability_catalog
from bioxp.pipette.cavro_liquid import Recipe, compile_liquid_recipe
from bioxp.pipette.cavro_geometry import resolve_surface_move, shared_head_evidence
from bioxp.pipette.receipts import PipetteReceiptStore
from tests.test_cavro_application import rig as wire_rig, request, stroke, run
from tests.test_cavro_liquid import recipe
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_method_runtime_connected import mount, submit, action, control
from tests.test_protocol_workflow_connected import await_job
from tests.test_oem_pipette_calibration import MotionNative
from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter


def surface(offset=100, detected=True):
    return {"operation": "surface_move", "intent": "immersion",
        "reference": {"kind": "detected", "detection_id": "source", "penetration_correction_steps": -20}
            if detected else {"kind": "authored", "reference_id": "fixture", "position_steps": 40000},
        "displacement": {"unit": "steps", "offset_steps": offset, "speed_native": 300}}


def search(mode=1):
    return {"operation": "plld", "channels": [0, 1], "timeout_ms": 100,
        "start_mode": mode, "detection_id": "source", "start_steps": 40000,
        "search_target_steps": 60000, "search_speed_native": 300, "z_motor_current": 17}


def tracked(detected=True):
    return {**stroke(volume=22.5), "operation": "tracked_aspirate", "target_liquid_ul": 20,
        "trajectory": surface(200, detected), "z_motor_current": 17, "z_timeout_ms": 100}


def export(name, value):
    root = os.environ.get("NON_OEM_NATIVE_EXPORT_DIR") or os.environ.get("CAVRO_EVIDENCE_ROOT")
    if root:
        Path(root).mkdir(exist_ok=True, parents=True)
        Path(root, name + ".json").write_text(json.dumps(value, indent=2))


def test_exact_geometry_and_no_invented_scale():
    op = surface()
    out = resolve_surface_move(op, {"source": {"final_position_steps": 1000}})
    assert out["target_steps"] == 1080 and not out["physical_surface_verified"]
    op = surface(detected=False)
    op["displacement"] = {"unit": "mm", "depth_mm": 1.25, "speed_mm_s": 2,
        "tip_offset_mm": .25, "calibration": {"calibration_id": "authored-test-only",
            "steps_per_mm": 100, "native_speed_per_mm_s": 30, "positive_steps_direction": "up"}}
    out = resolve_surface_move(op, {})
    assert out["target_steps"] == 39900 and out["speed_native"] == 60
    op["displacement"]["depth_mm"] = .001
    assert compile_application(request(op))["issues"]
    op["reference"] = surface()["reference"]
    assert compile_application(request(op))["issues"]  # known scale error, no search needed
    track = tracked()
    track["trajectory"] = op
    assert compile_application(request(track))["issues"]
    with pytest.raises(ValueError, match="operand unavailable"):
        resolve_surface_move(surface(), {})


def test_b_modes_and_stream_no_hidden_nvram():
    for mode, wire in [(0, "BR"), (1, "B1R")]:
        assert compile_application(request(search(mode)))["operations"][0]["wire_commands"][0]["ascii"] == wire
    old = search(); old.pop("start_mode")
    assert compile_application(request(old))["operations"][0]["wire_commands"][0]["ascii"] == "BR"
    for invalid in (True, 1.0, "1"):
        assert compile_application(request(search(invalid)))["issues"]
    out = compile_application(request({"operation": "settings", "channels": [0], "timeout_ms": 30,
        "values": {"pressure_streaming": True, "slope": [0, 41]}}, stroke(speed=1001)))
    assert not out["issues"] and len(out["warnings"]) == 2
    assert out["operations"][0]["wire_commands"][0]["ascii"] == "o0,1R"


def test_selected_correction_once_phase_truth_and_order():
    raw = recipe()
    raw.pop("commanded_aspiration_ul")
    raw["selected_correction"] = {"calibration_id": "fixture", "revision": "r1", "context": {"tip": "T10"},
        "function": {"kind": "points", "points": [{"target_ul": 20, "commanded_ul": 22.75}]}}
    raw["leading_air"] = None
    raw["phase_settings"] = {"leading_air": {"plld_threshold_adc": 20}}
    raw["aspiration_tracking"] = {k: v for k, v in tracked(False).items()
        if k in {"trajectory", "z_motor_current", "z_timeout_ms"}}
    before = copy.deepcopy(raw)
    plan = compile_liquid_recipe(raw)
    assert not plan["issues"], plan
    assert raw == before
    assert plan["operations"][0]["operation"] == "tracked_aspirate"
    assert plan["operations"][0]["wire_commands"][1]["ascii"] == "P22.75,1R"
    ledger = plan["application"]["liquid_settings"]
    assert ledger["selected_correction"]["application_count"] == 1
    assert ledger["phase_settings_emission"]["leading_air"]["emitted"] is False
    assert not any("p0," in c["ascii"] for op in plan["operations"] for c in op.get("wire_commands", []))
    assert compile_application(plan["application"])["operations"] == plan["operations"]
    raw["commanded_aspiration_ul"] = 22.75
    assert compile_liquid_recipe(raw)["issues"]
    export("corrected-recipe", plan)


def test_real_codec_embedding_and_schema_export():
    from bioxp.pipette.cavro_commands import compile_device_command, compile_device_program
    command = {"command": "report", "selector": 57}
    program = {"mode": "execute", "commands": [{"command": "delay", "milliseconds": 20}]}
    out = compile_application(request(
        {"operation": "device_command", "channels": [2], "timeout_ms": 100, "command": command},
        {"operation": "device_program", "channels": [2], "timeout_ms": 100, "program": program}))
    assert not out["issues"], out
    assert out["operations"][0]["compiled_command"] == compile_device_command(command)
    assert out["operations"][1]["compiled_command"] == compile_device_program(program)
    export("application-schema", ApplicationRequest.model_json_schema())
    export("recipe-schema", Recipe.model_json_schema())
    export("application-catalog", capability_catalog())
    export("device-instructions", out)


@pytest.mark.parametrize("fault", [None, "device", "interrupt"])
def test_real_finite_factory_geometry_tracking_and_partial_store(wire_rig, installed_retained, monkeypatch, tmp_path, fault):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    provider = installed_retained[1]
    native = MotionNative()
    native.motor_wait_stopped = lambda *args, **kwargs: {"stopped": True, "fixture": "inert hardware completion"}
    adapter = Serial206ProductionPrimitiveAdapter(native, wire_rig.group,
        authority_provider=lambda: None, generation_provider=provider.generation_provider)
    provider.primitives = adapter
    receipts = PipetteReceiptStore(installed_retained[4])
    monkeypatch.setattr(api, "_pipette_receipts", receipts)
    monkeypatch.setattr(api, "_require_motion_route_ready", lambda: None)
    # Actual native motion and Cavro sends share a chronological physical-leaf
    # trace: Z starts while P's completion owner has not yet been waited on.
    trace = []
    move = native.motor_oem_move_absolute
    def move_leaf(board, position, **kwargs):
        trace.append(("z_send", position, kwargs.get("wait_for_stop")))
        return move(board, position, **kwargs)
    monkeypatch.setattr(native, "motor_oem_move_absolute", move_leaf)
    wait = wire_rig.drivers[0].bus.wait_pipette_completion
    def wait_leaf(channel, timeout, **kwargs):
        trace.append(("pump_wait", channel, list(wire_rig.wire)))
        return wait(channel, timeout, **kwargs)
    for driver in wire_rig.drivers:
        driver.bus.wait_pipette_completion = wait_leaf
    if fault:
        wire_rig.fault[fault] = True
    application = request(search(), surface(), tracked(), surface(-300), stroke("dispense", 20))
    authored = recipe()
    authored.pop("commanded_aspiration_ul")
    authored.update(channels=[0, 1], leading_air=None, trailing_air=None, before_liquid=[search(), surface()],
        after_liquid=[surface(-300)], dispense_segments=[{"volume_ul": 20, "speed_ul_s": 50}],
        final_empty_speed_ul_s=None,
        selected_correction={"calibration_id": "fixture", "revision": "r1", "context": {},
            "function": {"kind": "affine", "scale": 1, "offset_ul": 2.5}},
        aspiration_tracking={k: v for k, v in tracked().items()
            if k in {"trajectory", "z_motor_current", "z_timeout_ms"}})
    params = {"operation": "cavro_liquid_recipe", "recipe": authored}
    job = submit(app, client, [action("pipette_manual_physical", {"operation": "cavro_application",
        "application": application} if fault else params), action("note")], key="geometry-" + str(fault))
    done = await_job(client, job, lambda r: r["command"]["terminal"])
    rows = done["execution"]["runtime_state"]["action_results"]
    source = rows[0].get("pipette_result", {})
    export("geometry-" + str(fault), {"application": application, "result": done,
        "wire": wire_rig.wire, "trace": trace, "physical_acceptance": False})
    assert source, rows
    if fault:
        assert not source["ok"], source
        assert not any(w.startswith("D") for _, w in wire_rig.wire)
    else:
        assert source["ok"], source.get("error")
        assert done["command"]["status"] == "completed"
        assert (0, "B1R") in wire_rig.wire
        assert ("z", 88080, True) in native.moves
        assert ("z", 88180, False) in native.moves
        assert ("z", 87680, True) in native.moves
        start = next(i for i, t in enumerate(trace) if t[:2] == ("z_send", 88180))
        assert any(t[0] == "pump_wait" and (1, "P22.5,1R") in t[2] for t in trace[start + 1:])
        assert any(e["operation"] == "tracking_z_wait" for e in source["events"])
        assert source["liquid_settings"]["selected_correction"]["application_count"] == 1
    store = app.state.operator_command_plane.store
    ids = done["execution"]["runtime_state"]["workflow"]["child_command_ids"]
    finite = [cid for cid in ids if not cid.startswith("pipette_")]
    assert len(finite) == 1
    durable = store.wp8_operation_evidence(finite[0])
    assert durable["children"][0]["operation"] == "sourceCavroApplication"
    assert receipts.read(limit=100)
    assert client.get("/protocol/jobs/" + job).json()["execution"]["runtime_state"] == done["execution"]["runtime_state"]
    reopened = PipetteReceiptStore(installed_retained[4])
    assert reopened.read(limit=100) == receipts.read(limit=100)


@pytest.mark.parametrize("nominal", [10, 1000])
def test_explicit_profile_pickup_actual_finite_owner(wire_rig, installed_retained, monkeypatch, tmp_path, nominal):
    from bioxp.manual_pipetting import compile_manual_pipetting
    from bioxp import oem_machine_bundle
    registry = json.loads(Path("docs/specs/2026-07-23-oem-movement-method-source-binary-registry.json").read_text())
    lock = Path(registry["authority"]["evidence_lock_path"]).parent / "oem_machine_runtime_bundle_serial206" / "OEM_EVIDENCE_LOCK.json"
    snapshot = oem_machine_bundle.load_oem_machine_snapshot(lock, operator_label_serial="206")
    monkeypatch.setattr(oem_machine_bundle, "_active_snapshot", snapshot)
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    provider = installed_retained[1]
    native = MotionNative()
    provider.primitives = Serial206ProductionPrimitiveAdapter(native, wire_rig.group,
        authority_provider=lambda: None, generation_provider=provider.generation_provider)
    receipts = PipetteReceiptStore(installed_retained[4])
    monkeypatch.setattr(api, "_pipette_receipts", receipts)
    monkeypatch.setattr(api, "_require_motion_route_ready", lambda: None)
    # Replace only report wire IO. Keep original query parser, collection,
    # state publication, receipt runner and both SQLite stores connected.
    for driver in wire_rig.drivers:
        original = driver._send_pipette_command
        def report(command, *args, _original=original, **kwargs):
            if command == "?31":
                return {"ok": True, "ack": {"received": True, "data": [32, 96, 49]},
                    "query_response_correlated": True, "fixture": "inert tip report"}
            return _original(command, *args, **kwargs)
        monkeypatch.setattr(driver, "_send_pipette_command", report)
    pickup = {"operation": "profile_pickup", "profile": {"profile_id": f"test-T{nominal}",
        "nominal_volume_ul": nominal, "usable_capacity_ul": nominal + .5,
        "manufacturer": "test only", "sku": "explicit fixture", "filtered": False},
        "calibration": {"calibration_id": "test-rack-depth", "rack_id": "test-rack",
            "positioning": {"operation": "move", "location_id": 2, "well": "A1", "position_flag": 1},
            "pickup": surface(100, False), "retract": surface(-100, False),
            "channel_wells": {"0": "A1", "1": "B1"}, "provenance": "synthetic explicit authored coordinates"}}
    doc = compile_manual_pipetting({"protocol_id": "profile-test", "steps": [pickup]})
    assert doc.stages[0].actions[0].params["profile"]["nominal_volume_ul"] == nominal
    job = submit(app, client, [action("pipette_manual_physical", pickup)], key="profile-" + str(nominal))
    done = await_job(client, job, lambda r: r["command"]["terminal"])
    source = done["execution"]["runtime_state"]["action_results"][0].get("pipette_result", {})
    export("profile-" + str(nominal), {"request": pickup, "result": done, "moves": native.moves,
        "physical_acceptance": False})
    assert source.get("ok"), source.get("error", done)
    assert ("z", 40100, True) in native.moves
    assert ("z", 39900, True) in native.moves
    assert source["requested"]["operations"][0]["profile"]["nominal_volume_ul"] == nominal
    assert receipts.read(limit=100)


def test_actual_workflow_abort_during_tracked_pump_wait(wire_rig, installed_retained, monkeypatch, tmp_path):
    import threading
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    provider = installed_retained[1]
    native = MotionNative()
    provider.primitives = Serial206ProductionPrimitiveAdapter(native, wire_rig.group,
        authority_provider=lambda: None, generation_provider=provider.generation_provider)
    receipts = PipetteReceiptStore(installed_retained[4])
    monkeypatch.setattr(api, "_pipette_receipts", receipts)
    monkeypatch.setattr(api, "_require_motion_route_ready", lambda: None)
    entered, release = threading.Event(), threading.Event()
    original = wire_rig.drivers[0].bus.wait_pipette_completion
    def wait(channel, timeout, **kwargs):
        if any(w.startswith("P") for _, w in wire_rig.wire):
            entered.set()
            assert release.wait(10), "test fixture release was not signaled"
        return original(channel, timeout, **kwargs)
    for driver in wire_rig.drivers:
        driver.bus.wait_pipette_completion = wait
    job = submit(app, client, [action("pipette_manual_physical", {"operation": "cavro_application",
        "application": request(tracked(False), surface(-100, False), stroke("dispense", 20))})], key="tracked-abort")
    try:
        assert entered.wait(10), wire_rig.wire
        response = control(client, job, int(provider.generation_provider()), action="abort")
        assert response.status_code in (200, 202), response.text
    finally:
        release.set()
    done = await_job(client, job, lambda r: r["command"]["terminal"])
    assert done["command"]["status"] != "completed"
    assert not any(w.startswith("D") for _, w in wire_rig.wire)
    assert ("z", 39900, True) not in native.moves
    export("tracked-abort", {"result": done, "wire": wire_rig.wire, "moves": native.moves})
