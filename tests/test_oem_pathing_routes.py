from pathlib import Path
import asyncio


def _write_bundle(root: Path) -> None:
    root.mkdir(parents=True, exist_ok=True)
    rows = "".join([
        '<LOC_MS x="1000" y="1000" zLow="80000" zDelta="30000" inc_factor="1" />',
        '<LOC_OC x="5000" y="5000" zLow="80000" zDelta="30000" inc_factor="1" />',
        '<LOC_TC x="20000" y="10000" zLow="80000" zDelta="30000" inc_factor="1" />',
        '<LOC_RC x="30000" y="20000" zLow="80000" zDelta="30000" inc_factor="1" />',
        '<WASTE_BIN x="90000" y="93000" zLow="0" zDelta="0" inc_factor="0" />',
        '<TECANRACK1 x="2000" y="90000" zLow="75000" zDelta="20000" inc_factor="1" />',
        '<TECANRACK2 x="3000" y="90000" zLow="75000" zDelta="20000" inc_factor="1" />',
        '<TECANRACK3 x="4000" y="90000" zLow="75000" zDelta="20000" inc_factor="1" />',
        '<TECANRACK4 x="5000" y="90000" zLow="75000" zDelta="20000" inc_factor="1" />',
        '<LOC_STRIP1 x="40000" y="50000" zLow="70000" zDelta="10000" inc_factor="1" />',
        '<LOC_STRIP2 x="42000" y="52000" zLow="70000" zDelta="10000" inc_factor="1" />',
        '<LOC_STRIP3 x="44000" y="54000" zLow="70000" zDelta="10000" inc_factor="1" />',
        '<LOC_STRIP4 x="46000" y="56000" zLow="70000" zDelta="10000" inc_factor="1" />',
        '<LOC_TROUGH x="60000" y="60000" zLow="70000" zDelta="10000" inc_factor="1" />',
        '<LOC_RC_COVER x="65000" y="65000" zLow="70000" zDelta="10000" inc_factor="1" />',
    ])
    (root / "config.xml").write_text(f'<BioXPCommonLib><AxisLimits><X_limit minSteps="0" maxSteps="90263"/><Y_limit minSteps="0" maxSteps="102956"/></AxisLimits><PositionTable>{rows}</PositionTable></BioXPCommonLib>')
    for name in ("Operation_parameters.xml", "InspectionSettings.xml", "processtime.xml", "calreference.xml"):
        (root / name).write_text("<x />")


def _bind_test_position_table(monkeypatch) -> None:
    """Inject a test table at the route boundary; production still needs a bound snapshot."""
    from src.bioxp import oem_homing_routes
    from src.bioxp.oem_compat.position_table import PositionTable

    table = PositionTable.from_rows(
        [{"locationID": "LOC_MS", "x": 1000, "y": 1000, "zLow": 80000, "zDelta": 30000, "inc_factor": 1}],
        source="test-bound-position-table",
    )
    monkeypatch.setattr(oem_homing_routes, "load_bound_oem_position_table", lambda: table)
    monkeypatch.setattr(oem_homing_routes, "_require_bound_snapshot", lambda _root_dir: None)
    monkeypatch.setattr(
        oem_homing_routes,
        "find_oem_machine_config_bundle",
        lambda: {"config": {"axis_limits": {"x": {"max_steps": 90263}, "y": {"max_steps": 102956}}}},
    )


def test_default_parameters_route_is_read_only():
    from src.bioxp.oem_homing_routes import get_oem_pathing_default_parameters
    payload = asyncio.run(get_oem_pathing_default_parameters(tiploaded="TIP"))
    assert payload["pseudo_z_home"] == 500
    assert payload["motion_commanded"] is False
    assert payload["physical_motion"] is False


def test_scriptmove_plan_route_is_read_only_and_uses_path_planner(tmp_path, monkeypatch):
    from src.bioxp.oem_homing_routes import plan_oem_scriptmove_path
    _bind_test_position_table(monkeypatch)
    _write_bundle(tmp_path)
    monkeypatch.setenv("BIOXP_OEM_MACHINE_CONFIG_DIR", str(tmp_path))
    payload = asyncio.run(plan_oem_scriptmove_path(location_id="LOC_MS", current_loc="LOC_MS", current_x=0, current_y=0, current_z=0, column=2, row=3, positionflag=1, gripper_confirmed=True))
    assert payload["branch"] == "gripper_confirmed_no_tip_direct_moveTo"
    assert payload["target_coordinates"] == {"x": 1000 - 2132 * 2, "y": 1000 + 2132 * 3, "z": 50000}
    assert payload["opened_usb"] is False
    assert payload["motion_commanded"] is False
    assert payload["current_mutation_commanded"] is False



def test_scriptmove_execute_route_defaults_to_no_motion_preview(tmp_path, monkeypatch):
    from src.bioxp.oem_homing_routes import OemScriptMoveExecuteRequest, execute_oem_scriptmove_path
    _bind_test_position_table(monkeypatch)
    _write_bundle(tmp_path)
    monkeypatch.setenv("BIOXP_OEM_MACHINE_CONFIG_DIR", str(tmp_path))

    payload = asyncio.run(execute_oem_scriptmove_path(OemScriptMoveExecuteRequest(**{
        "location_id": "LOC_MS",
        "current_loc": "LOC_MS",
        "current_x": 0,
        "current_y": 0,
        "current_z": 0,
        "column": 2,
        "row": 3,
        "positionflag": 1,
        "gripper_confirmed": True,
    })))

    assert payload["ok"] is True
    assert payload["schema_version"] == "bioxp.oem_scriptmove_execution.v1"
    assert payload["mode"] == "dry_run"
    assert payload["motion_commanded"] is False
    assert payload["opened_usb"] is False
    assert payload["executor_status"] == "preview_only"
    assert payload["plan"]["branch"] == "gripper_confirmed_no_tip_direct_moveTo"
    assert payload["execution_steps"][0]["op"] == "moveTo"
    assert payload["execution_steps"][0]["would_call"] == "/motion/axis/absolute_sequence"


def test_scriptmove_execute_live_requires_explicit_ack(tmp_path, monkeypatch):
    from fastapi import HTTPException
    from src.bioxp.oem_homing_routes import OemScriptMoveExecuteRequest, execute_oem_scriptmove_path
    _bind_test_position_table(monkeypatch)
    _write_bundle(tmp_path)
    monkeypatch.setenv("BIOXP_OEM_MACHINE_CONFIG_DIR", str(tmp_path))

    try:
        asyncio.run(execute_oem_scriptmove_path(OemScriptMoveExecuteRequest(**{
            "mode": "live",
            "location_id": "LOC_MS",
            "current_loc": "LOC_MS",
            "gripper_confirmed": True,
        })))
    except HTTPException as exc:
        assert exc.status_code == 409
        assert "ack" in str(exc.detail).lower()
    else:
        raise AssertionError("live execution without ack should fail closed")
