
from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot


def test_harmonized_motion_config_prefers_bound_immutable_original_ssd_machine_config(monkeypatch):
    from bioxp.oem_config import harmonized_motion_config

    bind_serial206_oem_snapshot(monkeypatch)

    cfg = harmonized_motion_config()

    assert cfg["source"] == "immutable_oem_machine_snapshot"
    assert cfg["live_ready"] is True
    assert cfg["axis_limits"]["x"]["max_steps"] == 90263
    assert cfg["axis_limits"]["y"]["max_steps"] == 102956
    assert cfg["config_status"]["machine_calibrated"] is True
    assert cfg["config_status"]["diff_vs_source_defaults"]["summary"]["axis_limit_changes"] == 2


def test_parity_config_loader_maps_bound_immutable_oem_snapshot(monkeypatch):
    from bioxp.oem_parity_config import load_oem_parity_config

    snapshot = bind_serial206_oem_snapshot(monkeypatch)
    cfg = load_oem_parity_config(snapshot)

    assert cfg.calibration_source == "immutable_oem_machine_snapshot"
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
    assert cfg.blockers == ["operator_physical_label_not_matched"]



def test_usb_homing_profiles_use_bound_machine_axis_limits_for_xyg(monkeypatch):
    from bioxp.usb_driver import BioXpTester

    bind_serial206_oem_snapshot(monkeypatch)
    tester = BioXpTester.__new__(BioXpTester)

    x = tester._motion_oem_axis_profile("x", startup=True)
    y = tester._motion_oem_axis_profile("y", startup=True)
    g = tester._motion_oem_axis_profile("g", startup=True)
    z = tester._motion_oem_axis_profile("z", startup=True)

    assert x["home_search_max_abs_delta"] == 90263
    assert x["home_search_max_abs_delta_source"] == "immutable_oem_machine_snapshot"
    assert y["home_search_max_abs_delta"] == 102956
    assert y["home_search_max_abs_delta_source"] == "immutable_oem_machine_snapshot"
    assert g["home_search_max_abs_delta"] == 15000
    assert g["home_search_max_abs_delta_source"] == "immutable_oem_machine_snapshot"
    assert z["home_search_max_abs_delta"] == 160000
    assert z["home_search_max_abs_delta_source"] == "immutable_oem_machine_snapshot"
