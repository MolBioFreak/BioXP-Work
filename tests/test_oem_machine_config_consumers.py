
from pathlib import Path


def _write_real_shape_bundle(root: Path) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    cfg = root / "config.xml"
    cfg.write_text(
        """
        <BioXPCommonLib>
          <GenBot>
            <Config Version="3" GripperVersion="1" />
            <Calibration Calibrated="1" />
            <CameraInstalled Camera="1" Cameracalibrated="True" />
          </GenBot>
          <CalibrationFactors>
            <Offsets m_TCDoorStallGuardThreshold="6" m_TC_DOOR_VELOCITY="50" m_TC_DOOR_ACCELERATION="20" m_TC_DOOR_MAX_CURRENT="31" m_Z_MOTOR_MAX_CURRENT_DOWN="25" m_Z_MOTOR_MAX_CURRENT_UP="31" />
          </CalibrationFactors>
          <AxisLimits>
            <X_limit minSteps="0" maxSteps="90263" />
            <Y_limit minSteps="0" maxSteps="102956" />
            <Z_limit minSteps="0" maxSteps="160000" />
            <G_limit minSteps="0" maxSteps="15000" />
          </AxisLimits>
          <PositionTable><LOC_PARK x="1506" y="71" zLow="114092" zDelta="114092" inc_factor="0" /></PositionTable>
        </BioXPCommonLib>
        """.strip()
    )
    (root / "Operation_parameters.xml").write_text("<OperationParameters />")
    (root / "InspectionSettings.xml").write_text("<InspectionSettings />")
    (root / "processtime.xml").write_text("<processTime />")
    (root / "calreference.xml").write_text("<CalibrationReferences />")
    return cfg


def test_harmonized_motion_config_prefers_bound_original_ssd_machine_config(tmp_path, monkeypatch):
    from src.bioxp.oem_config import harmonized_motion_config

    _write_real_shape_bundle(tmp_path)
    monkeypatch.setenv("BIOXP_OEM_MACHINE_CONFIG_DIR", str(tmp_path))

    cfg = harmonized_motion_config()

    assert cfg["source"] == "original_ssd_machine_config"
    assert cfg["axis_limits"]["x"]["max_steps"] == 90263
    assert cfg["axis_limits"]["y"]["max_steps"] == 102956
    assert cfg["config_status"]["machine_calibrated"] is True
    assert cfg["config_status"]["diff_vs_source_defaults"]["summary"]["axis_limit_changes"] == 2


def test_parity_config_loader_maps_real_oem_attribute_shape(tmp_path):
    from bioxp.oem_parity_config import load_oem_parity_config

    cfg_xml = _write_real_shape_bundle(tmp_path)
    cfg = load_oem_parity_config(cfg_xml)

    assert cfg.machine_calibrated is True
    assert cfg.values["X_MOTOR_MAX_POSITION"] == 90263
    assert cfg.values["Y_MOTOR_MAX_POSITION"] == 102956
    assert cfg.values["Z_MOTOR_MAX_POSITION"] == 160000
    assert cfg.values["G_MOTOR_MAX_POSITION"] == 15000
    assert cfg.values["GripperVersion"] == 1
    assert cfg.values["CameraCalibrated"] is True
    assert cfg.values["Calibrated"] == 1
    assert cfg.values["TC_DOOR_VELOCITY"] == 50
    assert cfg.values["Z_MOTOR_MAX_CURRENT_UP"] == 31



def test_usb_homing_profiles_use_bound_machine_axis_limits_for_xyg(tmp_path, monkeypatch):
    from src.bioxp.usb_driver import BioXpTester

    _write_real_shape_bundle(tmp_path)
    monkeypatch.setenv("BIOXP_OEM_MACHINE_CONFIG_DIR", str(tmp_path))
    tester = BioXpTester.__new__(BioXpTester)

    x = tester._motion_oem_axis_profile("x", startup=True)
    y = tester._motion_oem_axis_profile("y", startup=True)
    g = tester._motion_oem_axis_profile("g", startup=True)
    z = tester._motion_oem_axis_profile("z", startup=True)

    assert x["home_search_max_abs_delta"] == 90263
    assert x["home_search_max_abs_delta_source"] == "original_ssd_machine_config"
    assert y["home_search_max_abs_delta"] == 102956
    assert y["home_search_max_abs_delta_source"] == "original_ssd_machine_config"
    assert g["home_search_max_abs_delta"] == 15000
    assert g["home_search_max_abs_delta_source"] == "original_ssd_machine_config"
    assert z["home_search_max_abs_delta"] == 160000
    assert z["home_search_max_abs_delta_source"] == "original_ssd_machine_config"
