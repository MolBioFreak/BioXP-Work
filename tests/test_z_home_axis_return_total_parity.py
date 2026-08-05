from src.bioxp.usb_driver import BioXpTester


ACK = {"status": 100}


def test_axis_search_home_propagates_oem_go_home_source_return_code():
    driver = object.__new__(BioXpTester)
    driver._motion_oem_axis_profile = lambda axis, startup=False: {  # type: ignore[method-assign]
        "board": 4,
        "motor": 1,
        "home_search_max_abs_delta": 160000,
    }
    driver.motor_set_home = lambda board, motor=0: {  # type: ignore[method-assign]
        "ok": True,
        "ack": dict(ACK),
        "readback": {"ack": dict(ACK), "value": 0},
    }
    driver.motor_query_home_switch = lambda board, motor=0: {  # type: ignore[method-assign]
        "ok": True,
        "ack": dict(ACK),
        "value": 0,
    }
    driver.motor_get_switch_activity = lambda board, motor=0: {"ok": True}  # type: ignore[method-assign]
    driver.motor_oem_go_home = lambda axis, **kwargs: {  # type: ignore[method-assign]
        "ok": True,
        "source_return_code": 92049,
        "home_after": {"ack": dict(ACK), "value": 1},
        "set_home": {"ok": True},
        "switch_transition": True,
        "false_home_guard": None,
    }

    result = driver.motor_oem_axis_search_home("z", speed=597)

    assert result["ok"] is True
    assert result["source_return_code"] == 92049
