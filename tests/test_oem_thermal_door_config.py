from src.bioxp.oem_config import (
    OEM_CRITICAL_SOURCE_DEFAULTS,
    machine_config_diff,
    oem_thermal_door_defaults,
)


def test_thermal_door_defaults_serial_ge_10_match_oem_source():
    defaults = oem_thermal_door_defaults("3200")

    assert defaults == {
        "TCDoorOpen": 16000,
        "TC_DOOR_VELOCITY": 50,
        "TC_DOOR_ACCELERATION": 20,
        "TC_DOOR_MAX_CURRENT": 31,
        "TCDoorStallGuardThreshold": 6,
    }


def test_thermal_door_defaults_serial_lt_10_match_oem_source():
    defaults = oem_thermal_door_defaults("9")

    assert defaults["TCDoorOpen"] == 93000
    assert defaults["TC_DOOR_VELOCITY"] == 900
    assert defaults["TC_DOOR_ACCELERATION"] == 20
    assert defaults["TC_DOOR_MAX_CURRENT"] == 31
    assert defaults["TCDoorStallGuardThreshold"] == 6


def test_thermal_door_defaults_unknown_serial_fail_to_current_machine_class():
    assert oem_thermal_door_defaults(None)["TCDoorOpen"] == 16000
    assert oem_thermal_door_defaults("not-a-number")["TC_DOOR_VELOCITY"] == 50


def test_critical_defaults_include_tc_door_open():
    assert OEM_CRITICAL_SOURCE_DEFAULTS["TCDoorOpen"] == 16000


def test_machine_config_diff_reports_tc_door_open_override():
    diff = machine_config_diff({"offsets": {"m_TCDoorOpen": 12345}})

    changed = diff["critical_constants_changed_from_source_defaults"]
    assert changed["TCDoorOpen"] == {
        "machine_key": "m_TCDoorOpen",
        "source_default": 16000,
        "machine_config": 12345,
    }
