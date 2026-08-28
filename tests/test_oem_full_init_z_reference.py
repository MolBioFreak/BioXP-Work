from src.bioxp.usb_driver import BioXpTester
from support_oem_machine_bundle import serial_206_immutable_machine_bundle


def test_full_initialize_motors_meta_action_is_absent():
    tester = BioXpTester.__new__(BioXpTester)
    assert not hasattr(tester, "motor_oem_initialize_motors_full_sequence")


def test_z_oem_profile_is_literal_and_has_no_live_mask_substitution(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    monkeypatch.setattr(tester, "motor_function_preset", lambda key: {"board": 4, "motor": 1, "speed": 1791, "acc": 576, "standby_current": 10})
    monkeypatch.setattr(tester, "_machine_config_bundle", serial_206_immutable_machine_bundle)

    profile = tester._motion_oem_axis_profile("z", startup=True)

    assert (profile["speed"], profile["acc"], profile["home_speed"]) == (1791, 576, 1791)
    assert (profile["axis_min_steps"], profile["axis_max_steps"]) == (0, 160000)
    assert profile["oem_home_step"] == "MotorZ.axisSearchHome(speed=1791)"
    assert "disable_right" not in profile
    assert "disable_left" not in profile
    assert "positive_down_requires_right_mask" not in profile



def test_z_already_home_rejects_gap10_as_a_substitute_for_gap9(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    monkeypatch.setattr(tester, "motor_function_preset", lambda key: {"board": 4, "motor": 1, "speed": 1791, "acc": 576, "standby_current": 10})
    monkeypatch.setattr(tester, "_machine_config_bundle", serial_206_immutable_machine_bundle)
    monkeypatch.setattr(tester, "motor_get_position", lambda board, motor=0: {"ack": {"status": 100}, "position": 0})
    monkeypatch.setattr(tester, "motor_get_speed", lambda board, motor=0: {"ack": {"status": 100}, "speed": 0})
    monkeypatch.setattr(tester, "motor_query_home_switch", lambda board, motor=0: {"ack": {"status": 100}, "value": 0})
    monkeypatch.setattr(tester, "motor_get_switch_activity", lambda board, motor=0: {
        "right_state": 1,
        "right_raw_active": True,
        "right_disabled": True,
        "left_state": 0,
        "left_raw_active": False,
        "left_disabled": False,
    })

    result = tester.motor_oem_axis_already_home("z", tolerance_steps=2)

    assert result["ok"] is False
    assert result["predicate_active"] is False
    assert result["live_z_reference_active"] is False
    assert result["z_reference_contract"]["accepted"] is False
    assert "GAP10/right is diagnostic-only" in result["z_reference_contract"]["reason"]
