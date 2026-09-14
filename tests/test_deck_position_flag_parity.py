"""H-08: pinned OEM movExecution IL_01dd pushes zero, not zHigh mode.

Run with tests.z_stop_offline_guard inside the qualified private namespace.
Only source planning and recorded motor leaves are exercised, not live authority.
"""
import hashlib
import json
from types import SimpleNamespace

import pytest

from bioxp import oem_serial206_initialization as native
from bioxp.oem_deck_movement import ClassMoveToIntent, compile_mov_execution
from bioxp.oem_compat.position_table import load_bound_oem_position_table
from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot


@pytest.fixture
def rig(monkeypatch):
    snapshot = bind_serial206_oem_snapshot(monkeypatch)
    # Synthetic test label only, under private /dev + denied USB discovery.
    # Reload through the real validator rather than clearing parity blockers.
    from bioxp import oem_machine_bundle
    snapshot = oem_machine_bundle.load_oem_machine_snapshot(
        snapshot.bundle_root / "OEM_EVIDENCE_LOCK.json", operator_label_serial=206, require_operator_label=True)
    monkeypatch.setattr(oem_machine_bundle, "_active_snapshot", snapshot)
    table = load_bound_oem_position_table()
    primitive = object.__new__(native.Serial206ProductionPrimitiveAdapter)
    xyz = dict(x=1000, y=1000, z=70000)
    monkeypatch.setattr(primitive, "_read_axis_position", lambda axis: xyz[axis])
    provider = object.__new__(native.Serial206OemInitializationProvider)
    provider.primitives = primitive
    state = dict(current_location=0, tip_loaded=False, tip_dirty=False,
                 tip_location=-1, clean_path=False, pseudo_z_home=500,
                 plate_on_gantry=None, plate_locations={0: 1})
    monkeypatch.setattr(provider, "mov_execution_machine_state", lambda: dict(state))
    return provider, primitive, state, xyz, table


@pytest.mark.parametrize("destination", [1, 10])
@pytest.mark.parametrize("pseudo", [500, 65000])
@pytest.mark.parametrize("well_kind", ["default", "explicit", "material"])
@pytest.mark.parametrize("tip", [False, True])
def test_normal_compiler_provider_planner_dynamic_home(rig, destination, pseudo, well_kind, tip):
    provider, _, state, _, table = rig
    state.update(pseudo_z_home=pseudo, tip_loaded=tip, plate_locations={0: destination})
    options = {"well": "H12"} if well_kind == "explicit" else {"material": "sample"} if well_kind == "material" else {}
    calls = []
    def next_well(*args):
        calls.append(args)
        return 95
    compiled = compile_mov_execution(ClassMoveToIntent(1, plate_name=0, **options), state, get_next_well=next_well)
    plan = provider.preview_scriptmove_to(compiled.steps[0].arguments)["plan"]
    assert plan["target_coordinates"]["z"] == pseudo
    assert compiled.steps[0].arguments["positionflag"] == plan["positionflag"] == 0
    assert compiled.well_source == well_kind
    assert (plan["column"], plan["row"]) == ((0, 0) if well_kind == "default" else (11, 7))
    assert calls == ([(2, "sample", 0.0)] if well_kind == "material" else [])
    assert table.resolve(location_id="LOC_OC" if destination == 1 else "TECANRACK4").z_high == (59390 if destination == 1 else 89312)


@pytest.mark.parametrize("entry", ["provider", "primitive"])
@pytest.mark.parametrize("pseudo", [500, 65000])
def test_omitted_default_uses_source_zero(rig, entry, pseudo):
    provider, primitive, state, _, _ = rig
    state["pseudo_z_home"] = pseudo
    if entry == "provider":
        plan = provider.preview_scriptmove_to({"destination": 1})["plan"]
    else:
        plan = primitive.oem_preview_scriptmove_to(current_location=0, target_location=1,
            tip_loaded=False, tip_dirty=False, tip_location=-1, clean_path=False,
            pseudo_home_steps=pseudo, plate_on_gantry=None)["plan"]
    assert plan["positionflag"] == 0
    assert plan["target_coordinates"]["z"] == pseudo


@pytest.mark.parametrize("flag", [0, 1, -1, 2])
@pytest.mark.parametrize("spelling", ["positionflag", "position_flag"])
def test_explicit_modes_preserved_through_provider_execution(rig, monkeypatch, flag, spelling):
    provider, _, _, _, table = rig
    seen = []
    monkeypatch.setattr(native, "_execute_oem_steps_live", lambda steps, *a, **kw: seen.append(steps) or {})
    result = provider.scriptmoveTo(destination=1, **{spelling: flag})
    plan = result["plan"]
    target = table.resolve(location_id="LOC_OC")
    assert plan["positionflag"] == flag
    assert plan["target_coordinates"]["z"] == (500 if flag == 0 else target.z_high if flag == 1 else target.z_low)
    assert seen == [plan["steps"]]


def test_old_well_and_d_remain_distinct(rig):
    provider, _, state, _, table = rig
    intent = ClassMoveToIntent(1, plate_name=0, well="H12", continuation="d")
    normal = compile_mov_execution(intent, state)
    assert [s.arguments["positionflag"] for s in normal.steps if s.operation == "scriptmoveTo"] == [0, -1]
    continuation = provider.preview_scriptmove_to(normal.steps[-1].arguments)["plan"]
    assert continuation["target_coordinates"]["z"] == table.resolve(location_id="LOC_OC").z_low
    state.update(old_well=True, old_well_text="H12", old_location=1)
    old = compile_mov_execution(intent, state)
    assert len(old.steps) == 2
    assert old.steps[0].arguments["positionflag"] == 1
    assert provider.preview_scriptmove_to(old.steps[0].arguments)["plan"]["target_coordinates"]["z"] == 59390


@pytest.mark.parametrize("pseudo", [500, 65000])
@pytest.mark.parametrize("dirty", [False, True])
def test_same_xy_normal_and_continuation_request_distinction(rig, pseudo, dirty):
    provider, _, state, xyz, table = rig
    state.update(pseudo_z_home=pseudo, tip_dirty=dirty)
    xyz.update(table.resolve(location_id="LOC_OC").base_coordinates)
    compiled = compile_mov_execution(ClassMoveToIntent(1, plate_name=0, continuation="d"), state)
    for step, expected in [(compiled.steps[0], pseudo), (compiled.steps[-1], table.resolve(location_id="LOC_OC").z_low)]:
        plan = provider.preview_scriptmove_to(step.arguments)["plan"]
        assert plan["branch"] == "same_xy_move_z"
        assert [(s["op"], s["z"]) for s in plan["steps"]] == [("moveZ", expected)]


@pytest.mark.parametrize("requested,pseudo,effective", [(500, 500, 500), (500, 65000, 65000), (59390, 65000, 65000), (89312, 65000, 89312)])
def test_real_z_primitive_retains_source_floor(rig, monkeypatch, requested, pseudo, effective):
    _, primitive, _, _, _ = rig
    calls = []
    primitive.tester = SimpleNamespace(
        motor_set_axis_param=lambda *a, **kw: {"ok": True},
        motor_oem_move_absolute=lambda board, target, **kw: calls.append((board, target, kw)) or {"ok": True})
    monkeypatch.setattr(primitive, "_axis_profile", lambda axis: dict(board=4, motor=0, axis_max_steps=150000))
    result = primitive.oem_move_z(requested, pseudo_home_steps=pseudo)
    assert (result["requested"], result["effective"]) == (requested, effective)
    assert calls == [(4, effective, dict(motor=0, wait_for_stop=True, max_position=150000))]


def test_park_final_source_caller_keeps_flag_two(rig, monkeypatch):
    provider, primitive, _, _, table = rig
    semantics = dict(current_location_id="LOC_OC", current_well_id=0,
                     pseudo_z_home=500, tip_loaded=False, plate_on_gantry=None)
    monkeypatch.setattr(provider, "_deck_execution_semantics", lambda authority: semantics)
    calls = []
    def record(**kw):
        calls.append(kw)
        return {"ok": True, "controller_command_acknowledged": True,
                "controller_completion_verified": True}
    monkeypatch.setattr(primitive, "oem_initialize_motion_scriptmove_to_waste", record)
    provider.parkGantry()
    assert len(calls) == 1
    assert (calls[0]["target_location"], calls[0]["position_flag"]) == (28, 2)
    plan = provider.preview_scriptmove_to({"destination": 28, "positionflag": 2})["plan"]
    assert plan["target_coordinates"]["z"] == table.resolve(location_id="LOC_PARK").z_low == 114092


@pytest.mark.parametrize("parallel", [False, True])
def test_press_compiler_omitted_flag_provider_default(rig, parallel):
    from bioxp.oem_deck_movement import _compile_finite_plate_operation_unchecked
    provider, _, _, _, _ = rig
    compiled = _compile_finite_plate_operation_unchecked(
        "press_plate", source_leaf_available=True, plate=1,
        plate_locations={1: 1}, run_in_parallel=parallel)
    child = next(c for c in compiled["children"] if c["operation"] == "scriptmoveTo")
    assert "positionflag" not in child["arguments"]
    plan = provider.preview_scriptmove_to(child["arguments"])["plan"]
    assert plan["positionflag"] == 0
    assert plan["target_coordinates"]["z"] == 500


@pytest.mark.parametrize("location", [1, 6, 7, 8, 9, 10, 16])
@pytest.mark.parametrize("tip_location", [-1, 0, 1, 2, 3])
def test_representative_well_tip_exemptions_on_bound_map(rig, location, tip_location):
    from bioxp.oem_compat.pathing import LOCATION_ID_TO_NAME
    provider, _, state, _, table = rig
    state["tip_location"] = tip_location
    plan = provider.preview_scriptmove_to({"destination": location, "well": 95})["plan"]
    target = table.resolve(location_id=LOCATION_ID_TO_NAME[location])
    row = 7 if location in {6, 7, 8, 9, 10, 16} else 7 - (0 if tip_location == -1 else tip_location) * 2
    assert plan["target_coordinates"] == dict(
        x=target.base_coordinates["x"] + target.inc_factor * -2132 * 11,
        y=target.base_coordinates["y"] + target.inc_factor * 2132 * row, z=500)
    assert len(list(table.rows())) == 29


def test_existing_sealed_high_mode_plan_not_reinterpreted(rig, monkeypatch):
    provider, _, _, _, _ = rig
    plan = provider.preview_scriptmove_to({"destination": 1, "positionflag": 1})["plan"]
    digest = hashlib.sha256(json.dumps(plan, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
    def forbidden(*a, **kw):
        pytest.fail("changed plan reached execution")
    monkeypatch.setattr(native, "_execute_oem_steps_live", forbidden)
    with pytest.raises(RuntimeError, match="plan_authority_changed_before_first_tx"):
        provider.scriptmoveTo(destination=1, positionflag=0, expected_script_plan_digest=digest, source_plan=plan)
