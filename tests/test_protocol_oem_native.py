"""Offline native wrapper tests. All physical leaves are explicit doubles."""
from types import SimpleNamespace
import pytest

from bioxp.oem_deck_movement import (
    compile_finite_plate_operation, execute_finite_plate_operation,
    prepared_class_move_to_intent, compile_mov_execution,
)
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider as Provider
from bioxp.protocols.runtime_state import ProtocolRuntimeState


def result(**extra):
    return dict(ok=True, controller_command_acknowledged=True,
                controller_completion_verified=True, **extra)


@pytest.fixture
def park_rig(monkeypatch):
    trace = []
    p = object.__new__(Provider)
    state = dict(current_location_id="LOC_OC", current_well_id=0,
                 pseudo_z_home=65000, plate_on_gantry=None)
    p._deck_execution_semantics = lambda *a, **k: state
    p._park_collection_state = lambda: {"tip_exists": False}
    p._deck_gripper_confirmed = lambda: True
    p.sleep = lambda value: trace.append(("sleep", value))
    p.primitives = SimpleNamespace(
        oem_initialize_motion_scriptmove_to_waste=lambda **kw: (trace.append(("move", kw)) or result()),
        home_xy=lambda: (trace.append(("home", None)) or result(source_return={"x": 0, "y": 0})),
    )
    row = SimpleNamespace(base_coordinates={"y": 1})
    monkeypatch.setattr("bioxp.oem_serial206_initialization.load_bound_oem_position_table",
                        lambda: SimpleNamespace(resolve=lambda **kw: row))
    return p, state, trace


def test_script_park_true_source_order_and_false_unchanged(park_rig):
    p, _, trace = park_rig
    assert p.parkGantry(rehome=True)["ok"]
    assert [x[0] for x in trace] == ["move", "home", "move"]
    assert [x[1]["position_flag"] for x in trace if x[0] == "move"] == [0, 2]
    trace.clear()
    assert p.parkGantry()["ok"]
    assert [x[0] for x in trace] == ["move"]
    assert trace[0][1]["position_flag"] == 2


@pytest.mark.parametrize("axis,value,held", [("x", 100, False), ("x", -101, True), ("y", 101, True)])
def test_script_park_lost_step_source_boundary(park_rig, axis, value, held):
    p, _, trace = park_rig
    p.primitives.home_xy = lambda: result(source_return={"x": value if axis == "x" else 0, "y": value if axis == "y" else 0})
    r = p.parkGantry(rehome=True)
    assert bool(r.get("source_pause_scripts")) is held
    assert len(trace) == (1 if held else 2)
    if held:
        assert axis in r["source_error_event"]
        assert "source_location_update" not in r


def test_script_park_already_parked_never_homes(park_rig):
    p, state, trace = park_rig
    state["current_location_id"] = "LOC_PARK"
    assert p.parkGantry(rehome=True)["source_noop"]
    assert trace == []


def test_script_park_missing_source_return_not_zero(park_rig):
    p, _, trace = park_rig
    p.primitives.home_xy = lambda: result(source_return=None)
    with pytest.raises(RuntimeError, match="source_return_unavailable"):
        p.parkGantry(rehome=True)
    assert len(trace) == 1


def test_script_door_plan_uses_true_park_without_pipette_init():
    script = compile_finite_plate_operation("thermal_door", source_leaf_available=True,
        open=True, door_is_open=False, script_running=True)
    manual = compile_finite_plate_operation("thermal_door", source_leaf_available=True,
        open=True, door_is_open=False)
    assert script["children"][0]["arguments"] == {"rehome": True}
    assert manual["children"][0]["arguments"] == {"rehome": False}
    assert not any("pipette" in c["operation"].lower() for c in script["children"])
    def invoke(child):
        return result(door_open=False, door_closed=True) if child["operation"] == "readDoorSensors" else result()
    assert execute_finite_plate_operation(script, invoke)["source_return"] is False
    assert "source_return" not in execute_finite_plate_operation(manual, invoke)


def test_script_door_noop_source_true():
    plan = compile_finite_plate_operation("thermal_door", source_leaf_available=True,
        open=True, door_is_open=True, script_running=True)
    assert execute_finite_plate_operation(plan, lambda c: pytest.fail("native noop"))["source_return"] is True


def test_snapshot_preserves_move_model_sleep_before_null_image():
    plan = compile_finite_plate_operation("script_snapshot", source_leaf_available=True)
    assert [c["operation"] for c in plan["children"]] == ["scriptmoveTo", "updateLocation", "Sleep", "SnapshotImage"]
    assert plan["children"][0]["arguments"]["position_flag"] == 2
    assert plan["children"][2]["arguments"] == {"milliseconds": 2000}
    p = object.__new__(Provider)
    p.primitives = SimpleNamespace()
    r = p.wp8_snapshot_image("SnapshotImage", {"name": "aspirate_image"}, command_id="child", child_order=3, plan_digest="digest")
    assert r["source_noop"] == "m_frameGrabber_null"


def test_cutseal_source_children_and_default():
    plan = compile_finite_plate_operation("cut_seal", source_leaf_available=True,
        cut_x=12000, cut_z=40000, thermal_door_open=False, count=2)
    children = plan["children"]
    assert [c["operation"] for c in children[:4]] == ["doorOpen", "scriptmoveTo", "updateLocation", "CloseGripper"]
    assert [c["arguments"]["value"] for c in children if c["operation"] == "sourceMoveX"] == [19500, 17650, 7600, 5750]
    assert children[-1]["operation"] == "sendZandGripperHome"
    assert plan["parent_return_allows_background_pending"]
    with pytest.raises(ZeroDivisionError):
        compile_finite_plate_operation("cut_seal", source_leaf_available=True,
            cut_x=0, cut_z=0, thermal_door_open=True, count=0)


@pytest.mark.parametrize("count", [0, -1, 2])
def test_shakeoff_zero_and_negative_still_lower_and_home(count):
    plan = compile_finite_plate_operation("shakeoff", source_leaf_available=True,
        count=count, z_position=12345, current_location=6)
    names = [c["operation"] for c in plan["children"]]
    assert names[-2:] == ["sourceLowerPipette", "MoveZHome"]
    assert names[:-2] == ["Sleep", "sourceMoveZ", "sourceSetZAcc", "sourceMoveZ", "sourceRestoreZAcc"] * max(count, 0)


def native_rig():
    p = object.__new__(Provider)
    machine = dict(thermal_door_open=True, door_is_open=True, plate_locations={0: 23, 1: 21}, plate_location=23,
        gripper_version=1, output_plate_location=21, plate_on_gantry=0, current_tray=0)
    p.wp8_operation_machine_state = lambda op, args: dict(machine)
    p.mov_execution_machine_state = lambda: machine
    p.sleep = lambda value: None
    trace = []
    def execute(plan, action, state):
        trace.append(plan)
        return result(source_return=True, residual_state={"plate_on_gantry": None}, owned_children=())
    return p, trace, execute


def action(*args):
    return SimpleNamespace(params={"arguments": args})


def test_native_mapping_aliases_and_press_parse_default():
    p, trace, execute = native_rig()
    h = p.build_oem_native_handlers(settings={}, execute_plan=execute)
    assert h["catch"] is h["catchPlate"]
    assert h["release"] is h["releasePlate"]
    assert "led" not in h and "mov" not in h and "sp" not in h
    state = ProtocolRuntimeState(protocol_id="offline", dry_run=False)
    h["catch"](action("pool_plate"), state)
    assert state.source_model.allow_to_stop is False
    h["release"](action("POOL_PLATE", "pressplate"), state)
    assert state.source_model.allow_to_stop is True
    h["pressp"](action("not-an-enum"), state)
    assert any(c["operation"] == "moveZPress" for c in trace[-1]["children"])


@pytest.mark.parametrize("barcodes,held,reads", [(["BLACK", "BLACK"], False, 2), (["BLACK", "other"], True, 2), (["other"], False, 1), (["", ""], True, 2)])
def test_dopen_source_barcode_branch(barcodes, held, reads):
    p, _, execute = native_rig()
    calls = []
    def barcode():
        value = barcodes[len(calls)]
        calls.append(value)
        return value
    h = p.build_oem_native_handlers(settings={"DeckInspection": True, "StartMode": 1}, execute_plan=execute, barcode_reader=barcode)
    r = h["dopen"](action(), ProtocolRuntimeState(protocol_id="offline", dry_run=False))
    assert bool(r.get("source_error_hold")) is held
    assert len(calls) == reads


def test_selected_cv_refused_before_plan_and_disabled_snapshot_noop():
    p, trace, execute = native_rig()
    h = p.build_oem_native_handlers(settings={"DeckInspection": True, "StartMode": 1, "CheckSnapTips": False}, execute_plan=execute)
    with pytest.raises(RuntimeError, match="ReadBarcode"):
        h["dopen"](action(), ProtocolRuntimeState(protocol_id="offline", dry_run=False))
    assert h["snapshot"](action(), ProtocolRuntimeState(protocol_id="offline", dry_run=False))["source_noop"]
    assert trace == []


def test_rgb_clamp_cache_and_native_return_not_light_proof():
    p, _, execute = native_rig()
    calls = []
    h = p.build_oem_native_handlers(settings={}, execute_plan=execute,
        rgb_writer=lambda *rgb: (calls.append(rgb) or {"ok": False, "acks": {}}))
    r = h["led"](action("-1", "256", "7"), ProtocolRuntimeState(protocol_id="offline", dry_run=False))
    assert calls == [(0, 255, 7)] and p._oem_source_rgb == (0, 255, 7)
    assert r["ok"] is False and "hardware_postcondition_verified" not in r


def test_mov_capture_enum_type_old_well_and_piercing():
    i = prepared_class_move_to_intent({"m_destination": {"enum_type": "plateName", "value": "POOL_PLATE"},
        "m_well": "A1", "m_piersOption": {"m_piercoption": "r"}}, script_line=9)
    assert i.plate_name == 0 and i.continuation == "r" and i.well == "A1"
    old = prepared_class_move_to_intent({"m_oldWell": True}, script_line=9)
    plan = compile_mov_execution(old, {"old_location": 6, "old_well_text": "B2"})
    assert plan.source_branch == "old_well_terminal"
    with pytest.raises(ValueError, match="enum_type_required"):
        prepared_class_move_to_intent({"m_destination": 0}, script_line=1)


def test_lifecycle_deferred_enter_and_wake_are_distinct():
    p, plans, execute = native_rig()
    trace = []
    p.sleep = lambda value: trace.append(("sleep", value))
    h = p.build_oem_lifecycle_handlers(execute_plan=execute,
        unlatch=lambda state: (trace.append("unlatch") or result()),
        initial_check=lambda state: (trace.append("check") or result(source_return=False)),
        initialize_motors=lambda state: (trace.append("full_home") or result()),
        restore_door_model=lambda value, state: (trace.append(("door_model", value)) or result()),
        resume_temperature=lambda state: (trace.append("temperature") or result()))
    state = ProtocolRuntimeState(protocol_id="offline", dry_run=False)
    assert h["deferred_pause_enter"](state)["ok"]
    assert plans[-1]["children"][0]["arguments"] == {"rehome": False}
    assert trace == ["unlatch"]
    trace.clear()
    assert h["wake"](state)["source_wake_ready"]
    assert trace == ["check", "full_home", ("sleep", 0.04), ("door_model", False), "temperature"]
    assert "continue" not in h
    missing = p.build_oem_lifecycle_handlers(execute_plan=execute)
    assert "wake" not in missing and "script_prologue" not in missing


def test_lifecycle_run_job_prefix_hold_prevents_motion():
    p, plans, execute = native_rig()
    h = p.build_oem_lifecycle_handlers(execute_plan=execute,
        run_job_tip_prefix=lambda state: {"ok": False, "source_pause_scripts": True},
        confirm_gripper=lambda state: pytest.fail("prefix held"),
        home_gripper=lambda state: pytest.fail("prefix held"))
    assert h["run_job"](ProtocolRuntimeState(protocol_id="offline", dry_run=False))["source_pause_scripts"]
    assert plans == []


def test_lifecycle_prologue_three_baselines_and_image_then_return():
    p, _, execute = native_rig()
    trace = []
    h = p.build_oem_lifecycle_handlers(execute_plan=execute,
        pressure_baseline=lambda state: (trace.append("baseline") or result()),
        collect_critical_images=lambda state: (trace.append("images") or result()))
    assert h["script_prologue"](ProtocolRuntimeState(protocol_id="offline", dry_run=False))["ok"]
    assert trace == ["baseline"] * 3 + ["images"]


def test_critical_images_source_order_and_null_job():
    plan = compile_finite_plate_operation("critical_item_images", source_leaf_available=True,
        job_name="offline", camera_x_offset=0, camera_y_offset=0, camera_z_offset=0)
    children = plan["children"]
    assert children[0]["operation"] == "sourceImageGantryLoad"
    assert [c["arguments"]["location"] for c in children if c["operation"] == "sourceMoveTo"] == [0, 1, 3, 16, 11, 7, 8, 10, 9]
    assert 16 not in [c["arguments"]["destination"] for c in children if c["operation"] == "updateLocation"]
    assert [c["arguments"]["name"] for c in children if c["operation"] == "SnapshotImage"][-4:] == ["tip-tray-1", "tip-tray-2", "tip-tray-4", "tip-tray-3"]
    assert compile_finite_plate_operation("critical_item_images", source_leaf_available=True, job_name=None)["source_noop"]


def test_source_image_load_does_not_claim_carried_plate():
    p = object.__new__(Provider)
    calls = []
    p._wp8_publish_semantic = lambda **kw: (calls.append(kw) or result())
    assert p.wp8_source_image_gantry_load("sourceImageGantryLoad", {}, command_id="c", child_order=0, plan_digest="p")["ok"]
    assert calls[0]["updates"] == {"pseudo_z_home": 500}


@pytest.mark.parametrize("operation,args,expected", [
    ("sourceMoveX", {"value": 123}, ("x", 123)),
    ("sourceMoveZ", {"value": 456}, ("z", 456)),
    ("sourceSetZAcc", {"value": 500}, ("acc", 500)),
    ("sourceRestoreZAcc", {}, ("acc", None)),
    ("sourceLowerPipette", {"location": 6}, ("lower", "WASTE_BIN")),
])
def test_new_finite_leaf_bindings_use_existing_native_owners(operation, args, expected, monkeypatch):
    monkeypatch.setattr("bioxp.oem_serial206_initialization.load_bound_oem_position_table",
                        lambda: {6: {"location_id": "WASTE_BIN"}})
    p = object.__new__(Provider)
    trace = []
    p.primitives = SimpleNamespace(
        x_move_absolute=lambda **kw: (trace.append(("x", kw["position_steps"])) or result()),
        z_set_max_acc=lambda value=None: (trace.append(("acc", value)) or result()),
        z_pipette_position=lambda **kw: (trace.append(("lower", kw["location_id"])) or result()))
    p.moveZ = lambda value: (trace.append(("z", value)) or result())
    child = {"operation": operation, "order": 0, "arguments": args}
    r = p.execute_wp8_child(child, command_id="c", child_order=0, plan_digest="p")
    assert r["ok"] and trace == [expected]


def test_script_park_inherited_fence_prevents_home_after_approach(park_rig):
    p, _, trace = park_rig
    def check(boundary):
        if boundary == "script_park_home_xy":
            raise RuntimeError("original_parent_stopped")
    with pytest.raises(RuntimeError, match="original_parent_stopped"):
        p.parkGantry(rehome=True, before_native_entry=check)
    assert [r[0] for r in trace] == ["move"]


def test_mov_handler_uses_typed_intent_not_named_move():
    p, _, execute = native_rig()
    calls = []
    h = p.build_oem_native_handlers(settings={}, execute_plan=execute,
        execute_mov=lambda intent, a, s: (calls.append(intent) or result()))
    a = SimpleNamespace(source_key=17, params={"arguments": {
        "m_destination": {"enum_type": "locationID", "value": 6}, "m_well": None}})
    assert h["mov"](a, ProtocolRuntimeState(protocol_id="offline", dry_run=False))["ok"]
    assert calls[0].script_line == 17 and calls[0].location_id == 6


def test_lifecycle_critical_image_body_is_not_a_missing_callback():
    p, plans, execute = native_rig()
    h = p.build_oem_lifecycle_handlers(execute_plan=execute,
        settings={"JobName": "offline", "CameraXOffset": 0, "CameraYOffset": 0, "CameraZOffset": 0},
        pressure_baseline=lambda state: result())
    assert h["script_prologue"](ProtocolRuntimeState(protocol_id="offline", dry_run=False))["ok"]
    assert plans[-1]["operation"] == "critical_item_images"
