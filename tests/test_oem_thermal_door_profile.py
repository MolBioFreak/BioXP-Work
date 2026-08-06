from pathlib import Path

from bioxp.usb_driver import BioXpTester
from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot


def test_thermal_door_static_profile_preserves_oem_serial_ge_10_defaults():
    door = BioXpTester.MOTOR_FUNCTION_PRESETS["door"]

    assert door["speed"] == 50
    assert door["home_speed"] == 50
    assert door["acc"] == 20
    assert door["run_current"] == 31
    assert door["standby_current"] == 10
    assert door["stall_guard"] == 6
    assert door["open_position"] == 16000
    assert door["close_position"] == 0
    assert door["disable_right"] is True
    assert door["disable_left"] is True


def test_thermal_door_runtime_profile_prefers_immutable_original_ssd_machine_calibration(monkeypatch):
    bind_serial206_oem_snapshot(monkeypatch)
    tester = object.__new__(BioXpTester)

    door = tester._motion_oem_axis_profile("door")

    assert door["open_position"] == 18500
    assert door["open_position_source"] == "immutable_oem_machine_snapshot"
    assert door["speed"] == 50
    assert door["acc"] == 20
    assert door["run_current"] == 31
    assert door["stall_guard"] == 6


def test_diagnostic_door_menu_no_longer_uses_ad_hoc_targets():
    source = Path("src/bioxp/usb_driver.py").read_text()

    assert "open_pos = 10750" not in source
    assert "close_pos = -7000" not in source
    assert 'open_pos = int(p.get("open_position", 16000))' in source
    assert 'close_pos = int(p.get("close_position", 0))' in source
