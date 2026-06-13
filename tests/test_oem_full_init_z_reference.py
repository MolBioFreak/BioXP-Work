from src.bioxp.usb_driver import BioXpTester


def test_full_initialize_motors_uses_z_reference_return_not_generic_z_home(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    calls = []

    def already_home(axis, tolerance_steps=2):
        calls.append(("already_home", axis, tolerance_steps))
        # first check: not at reference; second check after return: good
        if sum(1 for c in calls if c[0] == "already_home") == 1:
            return {"ok": False, "already_home": False, "position": {"position": -177378}}
        return {"ok": True, "already_home": True, "position": {"position": 0}}

    def forbidden_home_axis(axis, **kwargs):
        raise AssertionError(f"generic home_axis must not be used for Z full init: {axis} {kwargs}")

    monkeypatch.setattr(tester, "motor_oem_axis_already_home", already_home)
    monkeypatch.setattr(tester, "motor_oem_home_axis", forbidden_home_axis)
    monkeypatch.setattr(tester, "motor_oem_move_z_to_reference", lambda **kwargs: calls.append(("z_return", kwargs)) or {"ok": True, "target_position": 0})
    monkeypatch.setattr(tester, "motor_oem_verify_z_clearance_for_xy", lambda **kwargs: calls.append(("z_clearance", kwargs)) or {"ok": False, "stop_after_z_only_for_test": True})

    result = tester.motor_oem_initialize_motors_full_sequence(timeout_s=123)

    assert result["ok"] is False
    assert result["failed_at"] == "z_clearance_for_xy"
    assert "z_return_to_live_reference_0" in result["steps"]
    assert "z_axisSearchHome_1791_motion" not in result["steps"]
    assert any(c[0] == "z_return" and c[1]["target_position"] == 0 for c in calls)


def test_z_profile_requires_right_mask_for_positive_reference_recovery(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    monkeypatch.setattr(tester, "motor_function_preset", lambda key: {"board": 4, "motor": 1, "speed": 1791, "acc": 576, "standby_current": 10})
    monkeypatch.setattr(tester, "_machine_config_axis_max", lambda axis, fallback: (160000, "test"))

    profile = tester._motion_oem_axis_profile("z", startup=True)

    assert profile["positive_down_requires_right_mask"] is True
    assert "live Z reference recovery" in profile["oem_home_step"]



def test_z_already_home_accepts_controller_zero_gap10_live_reference(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    monkeypatch.setattr(tester, "motor_function_preset", lambda key: {"board": 4, "motor": 1, "speed": 1791, "acc": 576, "standby_current": 10})
    monkeypatch.setattr(tester, "_machine_config_axis_max", lambda axis, fallback: (160000, "test"))
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
