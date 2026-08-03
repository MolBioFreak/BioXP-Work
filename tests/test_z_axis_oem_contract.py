from __future__ import annotations

import pytest

from bioxp.usb_driver import BioXpTester


def _driver() -> BioXpTester:
    return object.__new__(BioXpTester)


def _ack() -> dict:
    return {"status": 100}


def test_oem_stop_is_two_unconditional_deliveries_not_transport_retry(monkeypatch):
    driver = _driver()
    calls = []

    def send(*args, **kwargs):
        calls.append((args, kwargs))
        return _ack()

    monkeypatch.setattr(driver, "_send_motor", send)
    result = driver.motor_stop(4, motor=1)

    assert len(calls) == 2
    assert all(call[0][1:4] == (3, 0, 1) for call in calls)
    assert all(call[1]["attempts"] == 1 for call in calls)
    assert result["oem_double_stop"] is True
    assert result["first_delivery"]["status"] == 100
    assert result["second_delivery"]["status"] == 100
    assert result["ok"] is True


@pytest.mark.parametrize(
    ("method_name", "args", "command"),
    (
        ("motor_move_relative", (4, 100), 4),
        ("motor_move_absolute", (4, 100), 4),
        ("motor_move_left", (4,), 2),
        ("motor_move_right", (4,), 1),
    ),
)
def test_physical_motion_commands_are_single_delivery(method_name, args, command, monkeypatch):
    driver = _driver()
    calls = []
    monkeypatch.setattr(driver, "motor_query_motor_stop", lambda *a, **k: {"ok": True, "ack": _ack()})

    def send(*send_args, **kwargs):
        calls.append((send_args, kwargs))
        return _ack()

    monkeypatch.setattr(driver, "_send_motor", send)
    result = getattr(driver, method_name)(*args, motor=1)

    assert result["ok"] is True
    assert len(calls) == 1
    assert calls[0][0][1] == command
    assert calls[0][1]["attempts"] == 1


@pytest.mark.parametrize("method_name,args", (("motor_move_relative", (4, 100)), ("motor_move_absolute", (4, 100))))
def test_relative_and_absolute_refuse_to_send_after_failed_oem_pre_stop(method_name, args, monkeypatch):
    driver = _driver()
    monkeypatch.setattr(driver, "motor_query_motor_stop", lambda *a, **k: {"ok": False, "ack": {"status": 2}})
    monkeypatch.setattr(
        driver,
        "_send_motor",
        lambda *a, **k: (_ for _ in ()).throw(AssertionError("motion command must not be delivered")),
    )

    result = getattr(driver, method_name)(*args, motor=1)

    assert result["ok"] is False
    assert result["command_sent"] is False
    assert result["failure"] == "oem_pre_move_stop_query_failed"


def test_z_no_motion_profile_requires_effective_oem_switch_masks_clear(monkeypatch):
    driver = _driver()
    driver._oem_no_motion_profiles_ready = {"z"}
    monkeypatch.setattr(
        driver,
        "_motion_oem_axis_profile",
        lambda axis: {
            "board": 4,
            "motor": 1,
            "speed": 1791,
            "acc": 576,
            "run_current": 31,
            "stall_guard": 3,
        },
    )
    observed = {4: 1791, 5: 576, 6: 31, 205: 3, 12: 0, 13: 1}
    monkeypatch.setattr(
        driver,
        "motor_get_axis_param",
        lambda board, param, *, motor=0: {"ok": True, "ack": _ack(), "value": observed[param]},
    )

    with pytest.raises(RuntimeError, match="param.*13|13.*expected"):
        driver.motor_oem_require_no_motion_profile("z")

    assert driver._oem_no_motion_profile_ready is False


def test_oem_z_interlock_verification_is_read_only(monkeypatch):
    driver = _driver()
    calls = []
    monkeypatch.setattr(driver, "motor_query_24v_sensor", lambda: calls.append(("query_24v",)) or {"ack": _ack(), "reply_valid": True, "sample_valid": True, "safety_valid": True, "oem_scalar": 0})
    monkeypatch.setattr(driver, "deck_io_query_type", lambda kind: calls.append(("query_io", kind)) or {"ack": _ack(), "value": 1, "ok": True})
    monkeypatch.setattr(driver, "reconnect", lambda: (_ for _ in ()).throw(AssertionError("must not reconnect")))
    monkeypatch.setattr(driver, "activate_boards", lambda **kwargs: (_ for _ in ()).throw(AssertionError("must not reactivate boards")))
    monkeypatch.setattr(driver, "motor_set_axis_param", lambda *a, **k: (_ for _ in ()).throw(AssertionError("must not rewrite motor parameters")))

    result = driver.motor_oem_verify_motion_interlock()

    assert result["ok"] is True
    assert result["mutation_commanded"] is False
    assert calls == [("query_24v",), ("query_io", 1), ("query_io", 3)]


def test_async_z_event_window_actively_waits_and_returns_decoded_motor_event(monkeypatch):
    driver = _driver()

    class Frame:
        def provenance(self):
            return {
                "raw": [0x7E, 0, 0, 0, 0, 8, 4, 128, 138, 0, 0, 0, 1, 1, 0, 0x7E],
                "received_at": 1.0,
            }

    class Router:
        def queue_snapshot(self, name):
            return [Frame()] if name == "valid_async" else []

    driver.novo_router = Router()
    result = driver._collect_bus_events_locked(duration_s=0.0, timeout_ms=1, max_events=10)

    assert result[0]["status"] == 128
    assert result[0]["board"] == 4
    assert result[0]["motor"] == 1


def test_clear_event_window_clears_router_async_queues(monkeypatch):
    driver = _driver()
    driver._bus_event_buffer = [{"status": 128}]
    calls = []

    class Router:
        def queue_clear(self, name):
            calls.append(name)
            return 1

    driver.novo_router = Router()
    result = driver.clear_bus_event_buffer()

    assert result["cleared"] == 1
    assert result["router_cleared"] == {"valid_async": 1, "unknown_async": 1}
    assert calls == ["valid_async", "unknown_async"]


def test_begin_event_window_returns_monotonic_cursor(monkeypatch):
    driver = _driver()
    driver._bus_event_sequence = 7
    driver._bus_event_buffer = [{"status": 128}]
    monkeypatch.setattr(driver, "clear_bus_event_buffer", lambda: {"cleared": 1, "router_cleared": {}})

    window = driver.begin_bus_event_window()

    assert window["after_sequence"] == 7
    assert window["cleared"] == 1


def test_wait_target_reached_ignores_stale_event_and_accepts_fresh_event(monkeypatch):
    driver = _driver()
    driver._oem_abort_generation = 0
    monkeypatch.setattr(driver, "oem_no24v_state", lambda: False)
    monkeypatch.setattr(
        driver,
        "collect_bus_events",
        lambda **kwargs: [
            {"board": 5, "motor": 0, "status": 128, "event_sequence": 7},
            {"board": 5, "motor": 0, "status": 128, "event_sequence": 8},
        ],
    )

    result = driver.motor_oem_wait_target_reached(
        5,
        motor=0,
        timeout_s=0.1,
        event_window={"after_sequence": 7},
    )

    assert result["ok"] is True
    assert result["event"]["event_sequence"] == 8


def test_wait_target_reached_rejects_fresh_stall_event(monkeypatch):
    driver = _driver()
    driver._oem_abort_generation = 0
    monkeypatch.setattr(driver, "oem_no24v_state", lambda: False)
    monkeypatch.setattr(
        driver,
        "collect_bus_events",
        lambda **kwargs: [{"board": 5, "motor": 0, "status": 130, "event_sequence": 8}],
    )

    result = driver.motor_oem_wait_target_reached(
        5,
        motor=0,
        timeout_s=0.1,
        event_window={"after_sequence": 7},
    )

    assert result["ok"] is False
    assert result["failure"] == "oem_moveToAbs_stall_event"


def test_move_z_home_is_distinct_source_method_at_1791_without_standby_write(monkeypatch):
    driver = _driver()
    calls = []
    monkeypatch.setattr(
        driver,
        "_motion_oem_axis_profile",
        lambda axis, startup=False: {
            "board": 4,
            "motor": 1,
            "run_current": 31,
            "home_search_max_abs_delta": 160000,
        },
    )
    monkeypatch.setattr(driver, "motor_oem_require_no_motion_profile", lambda axis: calls.append(("require_profile", axis)) or {"ok": True})
    monkeypatch.setattr(
        driver,
        "motor_set_axis_param",
        lambda board, param, value, *, motor=0: calls.append(("set", board, motor, param, value)) or {"ok": True, "ack": _ack(), "readback": {"value": value}},
    )
    monkeypatch.setattr(
        driver,
        "motor_get_axis_param",
        lambda board, param, *, motor=0: calls.append(("get", board, motor, param)) or {"ok": True, "ack": _ack(), "value": 31},
    )
    monkeypatch.setattr(
        driver,
        "motor_oem_go_home",
        lambda axis, **kwargs: calls.append(("go_home", axis, kwargs)) or {"ok": True, "home_predicate_confirmed": True},
    )

    result = driver.motor_oem_move_z_home(rehome=True, timeout_s=30.0)

    assert result["ok"] is True
    assert result["oem_method"] == "ClassControlInterface.MoveZHome"
    assert result["standby_current_param7_written"] is False
    assert calls == [
        ("require_profile", "z"),
        ("set", 4, 1, 6, 31),
        ("get", 4, 1, 6),
        ("go_home", "z", {"speed": 1791, "rehome": True, "timeout_s": 30.0, "max_search_abs_delta": None}),
    ]


def test_startup_z_home_uses_existing_literal_profile_without_extra_axis_prep(monkeypatch):
    driver = _driver()
    calls = []
    monkeypatch.setattr(
        driver,
        "_motion_oem_axis_profile",
        lambda axis, startup=False: {
            "board": 4,
            "motor": 1,
            "speed": 1791,
            "acc": 576,
            "home_speed": 1791,
            "run_current": 31,
            "stall_guard": 3,
            "home_search_max_abs_delta": 160000,
        },
    )
    monkeypatch.setattr(driver, "motor_oem_require_no_motion_profile", lambda axis: calls.append(("require_profile", axis)) or {"ok": True})
    monkeypatch.setattr(
        driver,
        "motor_prepare_axis",
        lambda *a, **k: (_ for _ in ()).throw(AssertionError("startup Z home must not rewrite the profile")),
    )
    monkeypatch.setattr(
        driver,
        "motor_oem_axis_search_home",
        lambda axis, **kwargs: calls.append(("axis_search_home", axis, kwargs)) or {"ok": True},
    )

    result = driver.motor_oem_home_axis("z", startup=True, speed=1791, timeout_s=20.0)

    assert result["prepare"]["source_exact_no_additional_axis_writes"] is True
    assert calls == [
        ("require_profile", "z"),
        ("axis_search_home", "z", {"speed": 1791, "timeout_s": 20.0, "max_search_abs_delta": 160000}),
    ]
