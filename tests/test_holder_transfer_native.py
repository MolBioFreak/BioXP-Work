"""Nested holder Recipe -> application -> real finite provider, inert motor rig."""
import copy
import json
import pytest
import tests.z_stop_offline_guard  # noqa: F401
from bioxp.manual_pipetting import ManualMove, manual_position_plan
from bioxp.oem_calibration_settings import CalibrationSettingsPatch
from bioxp.oem_compat.position_table import load_bound_oem_position_table
from bioxp.oem_deck_catalog import DeckCatalog
from bioxp.oem_machine_bundle import get_active_oem_machine_snapshot
from bioxp.pipette.cavro_liquid import compile_liquid_recipe
from bioxp.pipette.cavro_application import compile_application, run_application_inline
from tests.test_bulk_holder_xy import save_owner, HOLDER_Z
from tests.test_movement_modes import connected, rig, cycle, bind_deck, z_key
from tests.test_liquid_detection import _recipe, _Rig


def position(**inputs):
    return {"operation": "position", "positioning": inputs}


def test_named_parser_keeps_legacy_move_contract():
    with pytest.raises(ValueError):
        ManualMove.model_validate({"operation": "move", "target": "LOC_TROUGH_MICRO_ROW_1", "camera_offset": False})
    assert manual_position_plan({"operation": "move", "location_id": 16, "well": "A1", "position_flag": 0})["operation"] == "manual_pipette_move"
    for extra in ({"x": 1}, {"well": "A1"}, {"position_flag": 2}, {"location_id": 16}):
        with pytest.raises(ValueError):
            manual_position_plan({"operation": "move", "target": "LOC_TROUGH_MICRO_ROW_1", "camera_offset": False, **extra})
    for target in ("LOC_TROUGH", "LOC_PARK", "LOC_TROUGH_MICRO_ROW_3"):
        with pytest.raises(ValueError):
            manual_position_plan({"operation": "move", "target": target, "camera_offset": False})


def prepare(connected, tmp_path, monkeypatch):
    http, provider, frames, fault, driver, positions = connected
    store, service, _ = save_owner(tmp_path, monkeypatch, get_active_oem_machine_snapshot())
    service.save(CalibrationSettingsPatch.model_validate({"bulk_reagent_holder": HOLDER_Z}))
    commands = bind_deck(http, provider)
    # The movement fixture emulates positions; add native SAP/GAP register storage
    # for the Z speed/current owners used by search (no product gate bypass).
    profile = driver._motion_oem_axis_profile("z")
    registers = {4: profile["speed"], 5: profile["acc"], 6: profile["run_current"], 205: profile["stall_guard"]}
    wire = driver.send_tmcl
    def exchange(board, command, typ, motor, value, **kw):
        reply = wire(board, command, typ, motor, value, **kw)
        if board == 4 and motor == 1:
            if command == 5:
                registers[typ] = value
            elif command == 6 and typ in registers:
                reply["value"] = registers[typ]
        return reply
    monkeypatch.setattr(driver, "send_tmcl", exchange)
    monkeypatch.setattr(driver, "send_tmcl_retry", exchange)
    # Existing pump simulator is inert; keep real provider/adapter motor execution.
    with monkeypatch.context() as patch:
        pumps = _Rig(patch, {0: 0.25, 1: 0.5, 2: 0.75})
    monkeypatch.setattr(provider.primitives, "pipette_transport", pumps.provider.primitives.pipette_transport)
    monkeypatch.setattr(provider, "_manual_pipette_receipt_runner", pumps.provider._manual_pipette_receipt_runner, raising=False)
    monkeypatch.setattr(provider, "_wp8_execution_fence_checker", lambda *a, **kw: None, raising=False)
    return provider, frames, positions, store, commands


def test_all_twenty_nested_named_positions_use_provider_and_publication(connected, tmp_path, monkeypatch):
    provider, frames, positions, store, commands = prepare(connected, tmp_path, monkeypatch)
    rows = [r for r in DeckCatalog.from_position_table(load_bound_oem_position_table()).rows() if "holder_row" in r]
    assert len(rows) == 20
    for n, row in enumerate(rows):
        zb, zm = z_key(provider)
        positions[zb, zm] = 120000
        frames.clear()
        request = {"implementation": "original-adp.application.v1", "operations": [position(operation="move", target=row["target"], camera_offset=False)]}
        compiled = compile_application(request)
        assert not compiled["issues"], compiled
        plan = compiled["operations"][0]["native_plan"]
        assert [c["operation"] for c in plan["children"]] == ["sourceForceToHighHome", "sourceNamedMoveTo"]
        result = run_application_inline(provider, request, command_id=f"holder-{n}", owner_identity={"source_identity": f"holder-{n}"})
        assert result["ok"], json.dumps(result, indent=2)
        assert provider.primitives._read_axis_position("x") == row["xy"]["x"]
        assert provider.primitives._read_axis_position("y") == row["xy"]["y"]
        assert [v for b,c,t,m,v in frames if c == 4 and (b,m)==(zb,zm)] == [500]
        semantic = commands.deck_semantic_state()
        assert semantic["current_location"] == "LOC_TROUGH"
        assert semantic["current_well"] == ((row.get("well_number") or 1) - 1)
    store.close()


@pytest.mark.parametrize("mode", ["by_level", "together", "one_at_a_time", "on_contact"])
def test_recipe_named_xy_row_search_lower_lift_through_finite_provider(connected, tmp_path, monkeypatch, mode):
    provider, frames, positions, store, commands = prepare(connected, tmp_path, monkeypatch)
    recipe = _recipe(location_id=16, holder_row="micro_row_1", mode=mode)
    recipe["before_liquid"] = [position(operation="move", target="LOC_TROUGH_MICRO_ROW_1", camera_offset=False)]
    recipe["after_liquid"] = [position(operation="lower", location_id=16, holder_row="micro_row_1"),
                             position(operation="lift", location_id=16, holder_row="micro_row_1", height_steps=None)]
    original = copy.deepcopy(recipe)
    compiled = compile_liquid_recipe(recipe)
    assert not compiled["issues"], compiled
    application = compiled["application"]
    native = compile_application(application)
    assert not native["issues"], native
    search = next(o for o in native["operations"] if o["operation"] == "liquid_search")
    assert search["holder_row"] == "micro_row_1"
    result = run_application_inline(provider, application, command_id="recipe", owner_identity={"source_identity": "recipe"})
    assert result["ok"], json.dumps(result, indent=2)
    assert recipe == original
    level = result["liquid_levels"][0]
    assert (level["start_steps"], level["floor_steps"]) == (500, 117984)
    assert all(level["channels"][str(c)]["found"] for c in recipe["channels"])
    assert any(e["operation"] == "liquid_search_z_stop" and e["status"] == "completed" for e in result["events"])
    zb, zm = z_key(provider)
    moves = [v for b,c,t,m,v in frames if c == 4 and (b,m)==(zb,zm)]
    assert moves[-2:] == [120000, 500]
    assert provider.primitives._read_axis_position("x") == 85555
    assert provider.primitives._read_axis_position("y") == 63278
    store.close()


@pytest.mark.parametrize("row", ["micro_row_2", "pcr_row_1", "pcr_row_2"])
def test_unmeasured_holder_search_never_uses_legacy_trough_z(connected, tmp_path, monkeypatch, row):
    provider, frames, positions, store, commands = prepare(connected, tmp_path, monkeypatch)
    application = compile_liquid_recipe(_recipe(location_id=16, holder_row=row))["application"]
    frames.clear()
    result = run_application_inline(provider, application, command_id="missing", owner_identity={"source_identity": "missing"})
    assert not result["ok"]
    assert "calibrated high and low" in result["error"]
    assert not any(c == 4 for b,c,t,m,v in frames)
    store.close()


def test_nested_named_move_obeys_existing_stop_fence(connected, tmp_path, monkeypatch):
    provider, frames, positions, store, commands = prepare(connected, tmp_path, monkeypatch)
    def stopped(command_id, *, boundary):
        if boundary == "before_nested_child_1":
            raise RuntimeError("operator Stop")
    monkeypatch.setattr(provider, "_wp8_execution_fence_checker", stopped)
    frames.clear()
    request = {"implementation": "original-adp.application.v1", "operations": [
        position(operation="move", target="LOC_TROUGH_MICRO_ROW_1", camera_offset=False),
        position(operation="lower", location_id=16, holder_row="micro_row_1")]}
    result = run_application_inline(provider, request, command_id="stop", owner_identity={"source_identity": "stop"})
    assert not result["ok"] and "operator Stop" in result["error"]
    assert not any(c == 4 for b,c,t,m,v in frames)
    assert commands.deck_semantic_state()["current_location"] is None
    store.close()
