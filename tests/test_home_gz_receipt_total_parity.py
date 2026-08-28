from src.bioxp.usb_driver import BioXpTester


def test_home_gz_normal_branch_fails_if_any_source_command_fails(monkeypatch):
    driver = object.__new__(BioXpTester)
    driver._motion_oem_axis_profile = lambda axis, startup=False: {  # type: ignore[method-assign]
        "board": 4,
        "motor": 1 if axis == "z" else 2,
        "axis_max_steps": 160000 if axis == "z" else 15000,
    }
    driver._oem_board_present = lambda board: True  # type: ignore[method-assign]
    driver._motion_oem_gripper_version = lambda: 0
    driver.motor_set_axis_param = lambda *args, **kwargs: {"ok": True}
    moves = iter(({"ok": False}, {"ok": True}))
    driver.motor_oem_move_absolute = lambda *args, **kwargs: next(moves)
    driver.motor_oem_go_home = lambda *args, **kwargs: {
        "ok": True,
        "source_return_code": 1,
    }
    monkeypatch.setattr("src.bioxp.usb_driver.time.sleep", lambda _seconds: None)

    result = driver.motor_oem_home_gz(
        delay_s=0,
        pseudo_z_home=65000,
        gripper_version=0,
        development_machine=True,
        timeout_s=30.0,
        caught_plate_x_home=lambda: {"ok": True},
    )

    assert result["ok"] is False
    assert result["branch"] == "normal_home_gz"
    assert result["failure"] == "home_gz_source_command_not_acknowledged"
