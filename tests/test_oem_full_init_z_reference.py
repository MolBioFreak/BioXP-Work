from src.bioxp.usb_driver import BioXpTester
from support_oem_machine_bundle import serial_206_immutable_machine_bundle


def test_full_initialize_motors_meta_action_remains_blocked_before_motion(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    calls = []

    monkeypatch.setattr(
        tester,
        "motor_oem_axis_already_home",
        lambda *args, **kwargs: (_ for _ in ()).throw(
            AssertionError("OEM initializeMotors must not replace axisSearchHome with a no-motion live predicate")
        ),
    )
    monkeypatch.setattr(
        tester,
        "motor_oem_move_z_to_reference",
        lambda **kwargs: (_ for _ in ()).throw(
            AssertionError("supervised coordinate recovery must not run inside OEM initializeMotors")
        ),
    )
    monkeypatch.setattr(
        tester,
        "motor_oem_axis_search_home",
        lambda axis, **kwargs: calls.append(("axis_search_home", axis, kwargs)) or {"ok": True},
    )
    monkeypatch.setattr(
        tester,
        "motor_oem_verify_z_clearance_for_xy",
        lambda **kwargs: calls.append(("z_clearance", kwargs)) or {"ok": False, "stop_after_z_only_for_test": True},
    )
    monkeypatch.setattr(
        tester,
        "_machine_config_axis_max",
        lambda axis, fallback: (160000, "immutable_oem_machine_snapshot"),
    )

    result = tester.motor_oem_initialize_motors_full_sequence(timeout_s=123)

    assert result["ok"] is False
    assert result["blocked"] is True
    assert result["blocked_reason"] == "literal_direct_oem_stage_rewrite_pending"
    assert result["physical_motion_commanded"] is False
    assert calls == []


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



def test_z_already_home_accepts_controller_zero_gap10_live_reference(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    monkeypatch.setattr(tester, "motor_function_preset", lambda key: {"board": 4, "motor": 1, "speed": 1791, "acc": 576, "standby_current": 10})
    monkeypatch.setattr(tester, "_machine_config_bundle", serial_206_immutable_machine_bundle)
    monkeypatch.setattr(tester, "motor_get_position", lambda board, motor=0: {"position": 0})
    monkeypatch.setattr(tester, "motor_get_speed", lambda board, motor=0: {"speed": 0})
    monkeypatch.setattr(tester, "motor_query_home_switch", lambda board, motor=0: {"value": 0})
    monkeypatch.setattr(tester, "motor_get_switch_activity", lambda board, motor=0: {
        "right_state": 1,
        "right_raw_active": True,
        "right_disabled": True,
        "left_state": 0,
        "left_raw_active": False,
        "left_disabled": False,
    })

    result = tester.motor_oem_axis_already_home("z", tolerance_steps=2)

    assert result["ok"] is True
    assert result["predicate_active"] is False
    assert result["live_z_reference_active"] is True
    assert result["z_reference_contract"]["accepted"] is True
