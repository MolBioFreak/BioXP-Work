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

    monkeypatch.setattr(tester, "query_only_transport_state", lambda: {"CAN_READY": True})
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
    assert commands[:4] == [
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

