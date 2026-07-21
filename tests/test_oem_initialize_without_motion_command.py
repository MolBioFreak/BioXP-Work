from src.bioxp.usb_driver import BioXpTester


def test_live_parity_test_case_emits_literal_oem_sequence(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    sent = []
    sleeps = []
    profiles = {
        "x": {"board": 5, "motor": 0},
        "y": {"board": 4, "motor": 0},
        "z": {"board": 4, "motor": 1},
        "g": {"board": 4, "motor": 2},
        "door": {"board": 6, "motor": 0, "speed": 50, "acc": 20, "run_current": 31, "stall_guard": 6},
    }

    monkeypatch.setattr(tester, "_motion_oem_axis_profile", lambda axis: profiles[axis])
    monkeypatch.setattr(tester, "_machine_config_offset_int", lambda key, fallback: (31 if "CURRENT" in key else 3, "test"))
    monkeypatch.setattr(tester, "_motion_oem_gripper_version", lambda: 1)
    monkeypatch.setattr(tester, "_tmcl_success", lambda ack: ack and ack.get("status") == 100)
    monkeypatch.setattr(tester, "send_tmcl_retry", lambda *args, **kwargs: sent.append((args, kwargs)) or {"status": 100})

    import src.bioxp.usb_driver as driver
    monkeypatch.setattr(driver.time, "sleep", lambda seconds: sleeps.append(seconds))

    result = tester.oem_initialize_without_motion_test_case()

    assert result["ok"] is True
    assert result["test_case"] == "oem.initializeMotorsWithoutMotion.live_parity.v1"
    commands = [(args[0], args[1], args[2], args[3], args[4]) for args, _ in sent]
    # waitForBoard activates all four initially-uninitialized boards before
    # initializeMotorsWithoutMotion starts its heater/chiller sequence.
    assert commands[:4] == [
        (4, 64, 0, 0, 1), (5, 64, 0, 0, 1),
        (6, 64, 0, 0, 1), (7, 64, 0, 0, 1),
    ]
    assert commands[4:8] == [
        (6, 144, 0, 0, 0), (6, 144, 0, 0, 0),  # duplicate heater PWM off
        (7, 144, 0, 1, 0), (7, 144, 0, 0, 0),  # OC then RC
    ]
    assert (4, 6, 6, 1, 0) in commands  # OEM Z readMaxCurrent, no write/readback helper
    pdiv = commands.index((4, 5, 154, 2, 2))
    rdiv = commands.index((4, 5, 153, 2, 6))
    assert pdiv < rdiv
    assert (7, 9, 8, 1, -25) in commands
    assert (7, 9, 8, 0, -25) in commands
    assert commands[-3:] == [(5, 50, 0, 0, 1024), (5, 50, 0, 1, 1024), (5, 50, 0, 2, 1024)]
    assert sleeps[0] == 0.1
    assert 0.001 in sleeps and 0.002 in sleeps


def test_wait_for_board_matches_oem_31_poll_then_selective_activation(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    sent = []
    sleeps = []
    monkeypatch.setattr(tester, "send_tmcl_retry", lambda *args, **kwargs: sent.append(args) or {"status": 100})
    import src.bioxp.usb_driver as driver
    monkeypatch.setattr(driver.time, "sleep", lambda seconds: sleeps.append(seconds))

    result = tester.oem_wait_for_board()

    assert result["ok"] is True
    assert len([row for row in result["trace"] if "pending" in row]) == 32
    assert [args[0] for args in sent] == [4, 5, 6, 7]
    assert all(args[1:5] == (64, 0, 0, 1) for args in sent)
    assert sleeps.count(0.1) == 32
    assert sleeps.count(0.001) == 4
    assert sleeps.count(0.01) == 4


def test_oem_activation_chiller_accepts_status_2_but_other_boards_do_not(monkeypatch):
    tester = BioXpTester.__new__(BioXpTester)
    monkeypatch.setattr(tester, "send_tmcl_retry", lambda board, *args, **kwargs: {"status": 2})
    import src.bioxp.usb_driver as driver
    monkeypatch.setattr(driver.time, "sleep", lambda seconds: None)

    assert tester._oem_activate_board(7)["initialized"] is True
    assert tester._oem_activate_board(4)["initialized"] is False
