from src.bioxp.usb_driver import BioXpTester


ACK = {"status": 100}


def _driver(speeds, positions):
    driver = object.__new__(BioXpTester)
    speed_values = list(speeds)
    position_values = list(positions)
    driver.motor_get_speed = lambda board, motor=0: {  # type: ignore[method-assign]
        "speed": speed_values.pop(0) if speed_values else 0,
        "ack": ACK,
    }
    driver.motor_get_position = lambda board, motor=0: {  # type: ignore[method-assign]
        "ok": True,
        "position": position_values.pop(0) if position_values else positions[-1],
        "ack": ACK,
    }
    return driver


def test_target_wait_does_not_accept_pre_acceleration_zero():
    driver = _driver([0, 0, 0, 250, 100, 0], [0, 10000])
    result = driver.motor_wait_stopped(
        4,
        motor=1,
        timeout_s=1.0,
        poll_s=0.02,
        require_seen_nonzero=True,
        target_position=10000,
    )
    assert result["stopped"] is True
    assert result["seen_nonzero"] is True
    assert result["last_position"] == 10000


def test_target_wait_accepts_verified_tiny_move_without_sampled_nonzero_speed():
    driver = _driver([0, 0, 0, 0], [0, 10000])
    result = driver.motor_wait_stopped(
        4,
        motor=1,
        timeout_s=1.0,
        poll_s=0.02,
        require_seen_nonzero=True,
        target_position=10000,
    )
    assert result["stopped"] is True
    assert result["seen_nonzero"] is False
    assert result["target_reached"] is True


def test_target_wait_rejects_persistent_pre_start_zero_at_wrong_position():
    driver = _driver([0], [0])
    result = driver.motor_wait_stopped(
        4,
        motor=1,
        timeout_s=0.3,
        poll_s=0.02,
        require_seen_nonzero=True,
        target_position=10000,
    )
    assert result["stopped"] is False
    assert result["ambiguous_no_motion"] is True
    assert result["last_position"] == 0


def test_target_wait_rejects_target_value_when_position_ack_is_invalid():
    driver = object.__new__(BioXpTester)
    driver.motor_get_speed = lambda board, motor=0: {"speed": 0, "ack": ACK}  # type: ignore[method-assign]
    driver.motor_get_position = lambda board, motor=0: {  # type: ignore[method-assign]
        "ok": True,
        "position": 10000,
        "ack": {"status": 2},
    }
    result = driver.motor_wait_stopped(
        4,
        motor=1,
        timeout_s=0.3,
        poll_s=0.02,
        require_seen_nonzero=True,
        target_position=10000,
    )
    assert result["stopped"] is False
    assert result["target_reached"] is False
