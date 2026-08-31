import pytest

from src.bioxp.usb_driver import BioXpTester


def _profile():
    return {
        "board": 4,
        "motor": 2,
        "speed": 600,
        "acc": 5,
        "run_current": 10,
        "standby_current": 10,
        "home_current": 31,
        "restore_current": 10,
        "stall_guard": 5,
        "rdiv": 6,
        "pdiv": 2,
        "home_speed": 600,
        "gripper_version": 1,
    }


def test_gv1_startup_profile_is_idle_safe_until_home_window(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    monkeypatch.setattr(
        tester,
        "motor_function_preset",
        lambda key: {"board": 4, "motor": 2, "speed": 600, "acc": 5, "standby_current": 10},
    )
    monkeypatch.setattr(tester, "_motion_oem_gripper_version", lambda: 1)
    monkeypatch.setattr(
        tester,
        "_machine_config_axis_max",
        lambda axis: (15000, "immutable_oem_machine_snapshot"),
    )

    profile = tester._motion_oem_axis_profile("g", startup=True)

    assert profile["run_current"] == 10
    assert profile["standby_current"] == 10
    assert profile["home_current"] == 31
    assert profile["restore_current"] == 10


@pytest.mark.parametrize("raise_during_home", [False, True])
def test_gv1_home_scopes_action_current_and_restores_on_every_exit(monkeypatch, raise_during_home):
    tester = BioXpTester.__new__(BioXpTester)
    events = []

    monkeypatch.setattr(tester, "_motion_oem_axis_profile", lambda axis, startup=False: _profile())
    monkeypatch.setattr(
        tester,
        "motor_prepare_axis",
        lambda board, motor=0, **kwargs: events.append(("prepare", kwargs["run_current"], kwargs["standby_current"])) or {"ok": True},
    )

    def home(*args, **kwargs):
        events.append(("home",))
        if raise_during_home:
            raise RuntimeError("simulated gripper home failure")
        return {"ok": True}

    monkeypatch.setattr(tester, "motor_oem_axis_search_home", home)
    monkeypatch.setattr(
        tester,
        "motor_restore_gripper_idle_current",
        lambda reason="": events.append(("restore", reason)) or {"ok": True, "run_current_param6": {"value": 10}},
    )

    if raise_during_home:
        with pytest.raises(RuntimeError, match="simulated gripper home failure"):
            tester.motor_oem_home_axis("g", startup=True)
    else:
        result = tester.motor_oem_home_axis("g", startup=True)
        assert result["home"]["ok"] is True

    assert events[0] == ("prepare", 31, 10)
    assert events[-1] == ("restore", "oem_home_axis_g_finally")
