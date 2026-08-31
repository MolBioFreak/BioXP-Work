from __future__ import annotations

import pytest

from bioxp.usb_driver import BioXpTester


def _driver() -> BioXpTester:
    driver = object.__new__(BioXpTester)
    driver._oem_no_motion_profile_ready = True
    driver._oem_no_motion_profiles_ready = {"z"}
    driver._oem_board_lifecycle_generation = 1
    driver._oem_active_board_lifecycle_generation = 1
    driver._oem_no_motion_profile_generations = {"z": 1}
    driver._oem_no_motion_profile_fingerprints = {}
    driver._oem_board_activation_observer_provider = None
    return driver


def _ack() -> dict:
    return {"status": 100}


def test_any_command64_transition_invalidates_all_axis_profile_generations(monkeypatch):
    driver = _driver()
    driver._oem_no_motion_profiles_ready = {"x", "y", "z"}
    driver._oem_no_motion_profile_generations = {"x": 1, "y": 1, "z": 1}
    driver._oem_no_motion_profile_fingerprints = {"z": {"speed": 1791}}
    monkeypatch.setattr(
        driver,
        "send_tmcl_retry",
        lambda board, command, cmd_type, motor, value, **kwargs: {"status": 2 if board == 7 else 100},
    )

    rows = driver.deactivate_boards(expect_reply=True, fail_fast=True)

    assert set(rows) == {4, 5, 6, 7}
    assert driver._oem_no_motion_profile_ready is False
    assert driver._oem_no_motion_profiles_ready == set()
    assert driver._oem_no_motion_profile_generations == {}
    assert driver._oem_no_motion_profile_fingerprints == {}
    assert driver._oem_active_board_lifecycle_generation is None


def test_command64_observer_preserves_legacy_two_argument_callback_compatibility(monkeypatch):
    driver = _driver()
    observed = []
    driver._board_activation_observer = (
        lambda board, ack: observed.append((board, ack.get("status") if isinstance(ack, dict) else None))
    )
    monkeypatch.setattr(
        driver,
        "send_tmcl_retry",
        lambda board, command, cmd_type, motor, value, **kwargs: {"status": 2 if board == 7 else 100},
    )

    rows = driver.deactivate_boards(expect_reply=True, fail_fast=True)

    assert set(rows) == {4, 5, 6, 7}
    assert observed == [(4, 100), (5, 100), (6, 100), (7, 2)]


def test_board_generation_advances_only_after_complete_deactivate_activate_cycle():
    driver = _driver()
    driver._oem_board_lifecycle_generation = 7
    driver._oem_active_board_lifecycle_generation = None
    complete = {board: {"status": 2 if board == 7 else 100} for board in (4, 5, 6, 7)}

    result = driver.oem_begin_board_lifecycle_generation(
        deactivation=complete,
        activation=complete,
    )

    assert result["ok"] is True
    assert result["board_lifecycle_generation"] == 8
    assert driver._oem_active_board_lifecycle_generation == 8


def test_board_generation_does_not_advance_after_partial_cycle():
    driver = _driver()
    driver._oem_board_lifecycle_generation = 7
    driver._oem_active_board_lifecycle_generation = None
    incomplete = {4: _ack(), 5: _ack()}
    complete = {board: {"status": 2 if board == 7 else 100} for board in (4, 5, 6, 7)}

    result = driver.oem_begin_board_lifecycle_generation(
        deactivation=incomplete,
        activation=complete,
    )

    assert result["ok"] is False
    assert driver._oem_board_lifecycle_generation == 7
    assert driver._oem_active_board_lifecycle_generation is None


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
    monkeypatch.setattr(
        driver,
        "clear_bus_event_buffer",
        lambda: (_ for _ in ()).throw(AssertionError("OEM wait-latch reset must not purge queues")),
    )

    window = driver.begin_bus_event_window()

    assert window["after_sequence"] == 7
    assert window["oem_wait_latch_reset"] is True
    assert window["cleared"] == 0
    assert window["queue_purge_omitted_by_source"] is True


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
        ("set", 4, 1, 6, 31),
        ("get", 4, 1, 6),
        ("go_home", "z", {"speed": 1791, "rehome": True, "timeout_s": 30.0, "max_search_abs_delta": None}),
    ]


def test_z_already_home_never_promotes_gap10_right_switch_to_reference(monkeypatch):
    driver = _driver()
    monkeypatch.setattr(
        driver,
        "_motion_oem_axis_profile",
        lambda axis, startup=False: {"board": 4, "motor": 1},
    )
    monkeypatch.setattr(
        driver,
        "motor_get_position",
        lambda board, motor=0: {"ok": True, "ack": _ack(), "position": 0},
    )
    monkeypatch.setattr(
        driver,
        "motor_get_speed",
        lambda board, motor=0: {"ok": True, "ack": _ack(), "speed": 0},
    )
    monkeypatch.setattr(
        driver,
        "motor_query_home_switch",
        lambda board, motor=0: {"ok": True, "ack": _ack(), "value": 0},
    )
    monkeypatch.setattr(
        driver,
        "motor_get_switch_activity",
        lambda board, motor=0: {"left_state": 0, "right_state": 1},
    )

    result = driver.motor_oem_axis_already_home("z")

    assert result["ok"] is False
    assert result["already_home"] is False
    assert result["live_z_reference_active"] is False
    assert result["z_reference_contract"]["accepted"] is False
