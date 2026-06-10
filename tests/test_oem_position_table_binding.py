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


def test_bound_position_table_uses_oem_z_high_and_increment_formula(tmp_path, monkeypatch):
    from src.bioxp.oem_compat.position_table import load_bound_oem_position_table

    _write_bundle(tmp_path)
    monkeypatch.setenv("BIOXP_OEM_MACHINE_CONFIG_DIR", str(tmp_path))
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
    monkeypatch.setenv("BIOXP_OEM_MACHINE_CONFIG_DIR", str(tmp_path))
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
    monkeypatch.setenv("BIOXP_OEM_MACHINE_CONFIG_DIR", str(tmp_path))

    rows = asyncio.run(get_oem_position_table())
    assert rows["position_table_count"] == 2
    assert rows["opened_usb"] is False
    plan = asyncio.run(plan_oem_position_table_move(location_id="WASTE_BIN", mode="moveTo"))
    assert plan["planned_coordinates"] == {"x": 92049, "y": 93211, "z": 0}
    assert plan["physical_motion"] is False
