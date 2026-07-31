from pathlib import Path


def _write_bundle(root: Path) -> None:
    root.mkdir(parents=True, exist_ok=True)
    (root / "config.xml").write_text(
        """
        <BioXPCommonLib>
          <AxisLimits><X_limit minSteps="0" maxSteps="90263"/><Y_limit minSteps="0" maxSteps="102956"/><Z_limit minSteps="0" maxSteps="160000"/><G_limit minSteps="0" maxSteps="15000"/></AxisLimits>
          <PositionTable>
            <LOC_MS x="26213" y="9241" zLow="83407" zDelta="37400" inc_factor="1" />
            <WASTE_BIN x="92049" y="93211" zLow="0" zDelta="0" inc_factor="0" />
          </PositionTable>
        </BioXPCommonLib>
        """.strip()
    )
    for name in ("Operation_parameters.xml", "InspectionSettings.xml", "processtime.xml", "calreference.xml"):
        (root / name).write_text("<x />")


def _bind_snapshot(root: Path, monkeypatch) -> None:
    from src.bioxp import oem_machine_bundle
    from src.bioxp.oem_config import parse_oem_machine_config_bundle
    from src.bioxp.oem_machine_bundle import OemMachineSnapshot

    parsed = parse_oem_machine_config_bundle(root)
    snapshot = OemMachineSnapshot(
        acquisition_id="test-acquisition",
        machine_serial=206,
        lock_sha256="0" * 64,
        bundle_root=root,
        records={},
        fields={},
        axis_limits=parsed["config"]["axis_limits"],
        position_table=tuple(parsed["config"]["position_table"]),
        config_sections={},
        operation_parameters={"Mode": "test", "DeckInspection": False, "CheckCamera": False},
        inspection_profile_name="Settings3200",
        inspection_profile={},
        calibration_comparison={},
        process_times={},
        mutable_seeds={},
        operator_label_matched=True,
    )
    monkeypatch.setattr(oem_machine_bundle, "_active_snapshot", snapshot)


def test_bound_position_table_uses_oem_z_high_and_increment_formula(tmp_path, monkeypatch):
    from src.bioxp.oem_compat.position_table import load_bound_oem_position_table

    _write_bundle(tmp_path)
    _bind_snapshot(tmp_path, monkeypatch)
    table = load_bound_oem_position_table()
    plan = table.compile_move_to("LOC_MS", column=2, row=3, high_pos=True)

    assert plan["planned_coordinates"] == {
        "x": 26213 + 1 * -2132 * 2,
        "y": 9241 + 1 * 2132 * 3,
        "z": 83407 - 37400,
    }
    low = table.compile_move_to("LOC_MS", column=0, row=0, high_pos=False)
    assert low["planned_coordinates"]["z"] == 83407


def test_bound_position_table_script_move_to_uses_oem_pseudo_z_and_tip_adjust(tmp_path, monkeypatch):
    from src.bioxp.oem_compat.position_table import load_bound_oem_position_table

    _write_bundle(tmp_path)
    _bind_snapshot(tmp_path, monkeypatch)
    table = load_bound_oem_position_table()
    plan = table.compile_script_move_to("LOC_MS", column=1, row=4, positionflag=0, tip_location=1)

    assert plan["planned_coordinates"] == {
        "x": 26213 - 2132,
        "y": 9241 + 2132 * 2,
        "z": 65000,
    }
    high = table.compile_script_move_to("LOC_MS", column=0, row=0, positionflag=1, tip_location=-1)
    assert high["planned_coordinates"]["z"] == 83407 - 37400


def test_position_table_routes_are_read_only(tmp_path, monkeypatch):
    from src.bioxp.oem_homing_routes import get_oem_position_table, plan_oem_position_table_move
    import asyncio

    _write_bundle(tmp_path)
    _bind_snapshot(tmp_path, monkeypatch)

    rows = asyncio.run(get_oem_position_table())
    assert rows["position_table_count"] == 2
    assert rows["opened_usb"] is False
    plan = asyncio.run(plan_oem_position_table_move(location_id="WASTE_BIN", mode="moveTo"))
    assert plan["planned_coordinates"] == {"x": 92049, "y": 93211, "z": 0}
    assert plan["physical_motion"] is False


def test_oem_position_table_ignores_unknown_plain_z_field():
    from src.bioxp.oem_compat.position_table import PositionTable

    table = PositionTable.from_rows([{"name": "PLAIN_Z", "x": 10, "y": 20, "z": 12345}])
    target = table.resolve(location_id="PLAIN_Z")

    assert target.z_low is None
    assert target.z_high is None
    assert target.z_delta is None
    assert target.base_coordinates["z"] == 0
    assert table.compile_move_to("PLAIN_Z")["planned_coordinates"]["z"] == 0


def test_oem_position_table_z_precedence_is_explicit_high_then_derived_then_low_default():
    from src.bioxp.oem_compat.position_table import PositionTable

    table = PositionTable.from_rows(
        [
            {"name": "EXPLICIT", "zLow": 80000, "zDelta": 30000, "zHigh": 41000},
            {"name": "DERIVED", "zLow": 80000, "zDelta": 30000},
            {"name": "LOW_ONLY", "zLow": 80000},
            {"name": "ZERO", "zLow": 0, "zDelta": 0},
        ]
    )

    assert table.resolve(location_id="EXPLICIT").base_coordinates["z"] == 41000
    assert table.resolve(location_id="DERIVED").base_coordinates["z"] == 50000
    assert table.resolve(location_id="LOW_ONLY").base_coordinates["z"] == 80000
    assert table.resolve(location_id="ZERO").base_coordinates["z"] == 0


def test_oem_position_table_rejects_malformed_recognized_z_fields():
    from src.bioxp.oem_compat.position_table import PositionTable

    import pytest

    with pytest.raises(ValueError):
        PositionTable.from_rows([{"name": "BAD", "zLow": "not-a-number"}])
