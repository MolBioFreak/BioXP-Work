"""Source-selected retry proof using real driver/router/reader, fake USB only.

Waits record the unchanged production budget, but use deterministic offline
endpoint outcomes. No controller delivery or durable-owner acceptance claim.
"""
from contextlib import contextmanager
import queue
import threading

import pytest
import usb.core

from bioxp import novo_router as router_module
from bioxp.command_exchange_observer import exchange_scope
from bioxp.novo_router import NovoRouter
from bioxp.usb_driver import BioXpTester, novo_decode, novo_encode


@contextmanager
def motor_transport(script):
    incoming = queue.Queue()
    waits = []
    actions = iter(script)

    class Input:
        def read(self, size, timeout):
            try:
                return incoming.get(timeout=.005)
            except queue.Empty:
                raise TimeoutError("offline idle")

    class Wait:
        def __init__(self, event):
            self.event = event
            self.action = None

        def set(self):
            self.event.set()

        def is_set(self):
            return self.event.is_set()

        def wait(self, timeout):
            waits.append(timeout)
            if self.action == "timeout":
                return False
            if self.action == "malformed":
                # Wait for the real reader to classify skipped traffic.
                for _ in range(100):
                    if router.queue_snapshot("malformed"):
                        return False
                    threading.Event().wait(.001)
                raise AssertionError("reader did not classify malformed traffic")
            if self.action == "shutdown_false":
                router.shutdown()
                return False
            if self.action == "rebind_false":
                router._reader_generation += 1
                return False
            assert self.event.wait(.5), "offline response not delivered"
            return self.action != "late_false"

    class Output:
        def __init__(self):
            self.writes = []
            self.pending_ids = []

        def write(self, raw, timeout):
            self.writes.append((bytes(raw), timeout))
            action = next(actions)  # Extra writes fail rather than hang.
            pending = router._pending
            if action == "late":
                pending = self.last_pending
            if pending is not None:
                self.last_pending = pending
                self.pending_ids.append(id(pending))
                if not isinstance(pending.event, Wait):
                    pending.event = Wait(pending.event)
                pending.event.action = action
            if isinstance(action, Exception):
                raise action
            if action in {"empty", "signaled_malformed"}:
                pending.outcome = "malformed" if action == "signaled_malformed" else None
                pending.event.set()
            elif action in {"success", "error", "malformed", "late_false"}:
                decoded = novo_decode(bytes(raw))
                data = bytes([decoded[3], 2 if action == "error" else 100, decoded[5], 0, 0, 0, 7, 0])
                wire = bytes(novo_encode(bytes([0, 0, 0, 0, 8]) + data))
                if action == "malformed":
                    wire = wire[:-2] + bytes([wire[-2] ^ 1]) + wire[-1:]
                incoming.put(wire)
            return len(raw)

    endpoint = Output()
    router = NovoRouter(ep_in=Input(), ep_out=endpoint, decode=novo_decode)
    driver = object.__new__(BioXpTester)
    driver.novo_router = router
    driver._transport_lock = threading.RLock()
    driver._motor_noresp_streak = {5: 8}
    driver._motor_last_tx_ts = {}
    driver._record_usb_sniff_ledger = lambda *a, **kw: None
    driver.reconnect = lambda: pytest.fail("selected motor must not reconnect")
    router.start()
    try:
        with exchange_scope("existing-root") as owner:
            yield driver, router, endpoint, waits, owner
    finally:
        router.shutdown()


def rows(owner):
    return owner.snapshot()["transport_exchanges"]


@pytest.mark.parametrize("script,expected", [
    (["timeout", "success"], ["timeout", "response"]),
    (["timeout", "timeout"], ["timeout", "timeout"]),
    (["malformed", "timeout"], ["malformed", "malformed"]),
    (["success"], ["response"]),
    (["error"], ["status_error"]),
    (["empty"], ["signaled_empty"]),
    (["signaled_malformed"], ["signaled_empty"]),
])
def test_selected_gap_retains_each_actual_attempt(script, expected):
    with motor_transport(script) as (driver, router, endpoint, waits, owner):
        result = driver.motor_get_axis_param(5, 1)
        retained = rows(owner)
        assert len(endpoint.writes) == len(expected)
        assert [r["outcome"] for r in retained] == expected
        assert [r["attempt_ordinal"] for r in retained] == list(range(1, len(expected) + 1))
        assert len({r["transaction_id"] for r in retained}) == 1
        assert len({r["exchange_id"] for r in retained}) == len(expected)
        assert len(set(endpoint.pending_ids)) == 1
        assert all(r["owner_generation"] == router.reader_generation for r in retained)
        assert waits == [60.0] * len(expected)
        assert len(set(endpoint.writes)) == 1
        assert [r["wait_signaled"] for r in retained] == [a not in {"timeout", "malformed"} for a in script]
        if result["ack"]:
            assert result["ack"]["provenance"]["transaction_id"] == retained[0]["transaction_id"]


@pytest.mark.parametrize("ordinal", [1, 2])
@pytest.mark.parametrize("error", [usb.core.USBTimeoutError("offline timeout"), usb.core.USBError("offline pipe"), ValueError("offline unknown")])
def test_selected_write_exception_retained_before_conversion(ordinal, error):
    with motor_transport(["timeout"] * (ordinal - 1) + [error]) as (driver, router, endpoint, waits, owner):
        if isinstance(error, usb.core.USBError):
            assert driver.motor_get_axis_param(5, 1)["ack"] is None
        else:
            with pytest.raises(ValueError) as caught:
                driver.motor_get_axis_param(5, 1)
            assert caught.value is error
        retained = rows(owner)
        assert len(endpoint.writes) == len(retained) == ordinal
        assert retained[-1]["attempt_ordinal"] == ordinal
        assert retained[-1]["outcome"] == "write_exception"
        assert retained[-1]["wait_signaled"] is None
        assert retained[-1]["write_returned"] is False
        assert router._pending is None


@pytest.mark.parametrize("action", ["shutdown_false", "rebind_false"])
def test_invalidated_owner_does_not_resubmit(action):
    with motor_transport([action]) as (driver, router, endpoint, waits, owner):
        generation = router.reader_generation
        assert driver.motor_get_axis_param(5, 1)["ack"] is None
        assert len(endpoint.writes) == 1
        assert rows(owner)[0]["owner_generation"] == generation
        assert rows(owner)[0]["wait_signaled"] is False


@pytest.mark.parametrize("last", ["timeout", "error", "success"])
def test_generic_absolute_has_two_source_calls_and_effective_ack(last):
    with motor_transport(["success", "timeout", "timeout", "timeout", last]) as (driver, router, endpoint, waits, owner):
        result = driver.motor_move_absolute(5, 321)
        retained = rows(owner)
        assert len(endpoint.writes) == 5  # one pre-stop, four MVP attempts
        assert [r["expected_command"] for r in retained] == [138, 4, 4, 4, 4]
        assert [r["transport_call_ordinal"] for r in retained] == [1, 2, 2, 3, 3]
        assert [r["attempt_ordinal"] for r in retained] == [1, 1, 2, 1, 2]
        assert result["first_ack"] is None
        assert result["ack"] == result["retry_ack"]
        assert result["source_return_code"] == result["low_level_source_return_code"] == 0
        assert result["ok"] is (last == "success")
        assert result["command_sent"] is True


def test_cached_speed_keeps_independent_null_leaf_call():
    with motor_transport(["timeout", "timeout", "timeout", "success"]) as (driver, router, endpoint, waits, owner):
        driver._oem_speed_cache = {(5, 0): 91}
        result = driver.motor_get_speed(5)
        assert result["speed"] == 91 and result["speed_source"] == "oem_cached"
        assert len(endpoint.writes) == 4
        assert [r["transport_call_ordinal"] for r in rows(owner)] == [1, 1, 2, 2]


@pytest.mark.parametrize("script,call_ordinals", [
    (["success", "timeout", "timeout"], [1, 2, 2]),
    (["timeout"] * 4, [1, 1, 2, 2]),
])
def test_stop_keeps_two_unconditional_calls(script, call_ordinals):
    with motor_transport(script) as (driver, router, endpoint, waits, owner):
        result = driver.motor_oem_stop_exact(5)
        assert result["source_return_code"] == 0
        assert len(endpoint.writes) == len(script)
        assert [r["transport_call_ordinal"] for r in rows(owner)] == call_ordinals


def test_move_left_is_one_call_with_original_short_wait():
    with motor_transport(["timeout", "timeout"]) as (driver, router, endpoint, waits, owner):
        driver.motor_move_left(5)
        assert len(endpoint.writes) == 2
        assert waits == [1.0, 1.0]
        assert len({r["transaction_id"] for r in rows(owner)}) == 1


def test_default_raw_query_remains_nonselected():
    with motor_transport(["timeout"]) as (driver, router, endpoint, waits, owner):
        assert driver.query_only_tmcl(5, 6, 1, 0, 0) is None
        assert len(endpoint.writes) == 1


def test_false_wait_and_late_same_call_frame_are_both_retained():
    with motor_transport(["late_false", "late"]) as (driver, router, endpoint, waits, owner):
        result = driver.motor_get_axis_param(5, 1)
        first, second = rows(owner)
        assert len(endpoint.writes) == 2
        assert first["wait_signaled"] is False
        assert first["outcome"] == "timeout"
        assert first["response_present"] is True
        assert second["outcome"] == "response"
        assert second["response_attempt_attribution"] == "same_call_ambiguous"
        assert first["receive_sequence"] == second["receive_sequence"]
        assert result["ack"]["provenance"]["observed_rx_raw"] == first["observed_rx_raw"]
        assert result["ack"]["provenance"]["attempts"][0]["wait_signaled"] is False
        assert len(set(endpoint.pending_ids)) == 1


def test_observer_failure_cannot_suppress_source_retry(monkeypatch):
    def broken(row):
        raise RuntimeError("offline evidence failure")
    with motor_transport(["timeout", "success"]) as (driver, router, endpoint, waits, owner):
        monkeypatch.setattr(router_module, "publish_exchange", broken)
        assert driver.motor_get_axis_param(5, 1)["ack"]["status"] == 100
        assert len(endpoint.writes) == 2
        assert len(owner.snapshot()["transport_retention_errors"]) == 2


@pytest.mark.parametrize("operation", ["shutdown", "replace"])
def test_invalidation_after_first_retention_cannot_retry(monkeypatch, operation):
    original = router_module.publish_exchange
    with motor_transport(["timeout"]) as (driver, router, endpoint, waits, owner):
        def publish(row):
            original(row)
            if operation == "shutdown":
                router.shutdown()
            else:
                # Different call owner, same generation: no takeover by retry.
                router._pending = object()
        monkeypatch.setattr(router_module, "publish_exchange", publish)
        assert driver.motor_get_axis_param(5, 1)["ack"] is None
        assert len(endpoint.writes) == len(rows(owner)) == 1
        if operation == "replace":
            router._pending = None


@pytest.mark.parametrize("first", ["success", "error", "empty"])
def test_generic_absolute_nonnull_and_signaled_empty_leaf_topology(first):
    script = ["success", first] + (["error"] if first == "empty" else [])
    with motor_transport(script) as (driver, router, endpoint, waits, owner):
        result = driver.motor_move_absolute(5, 321)
        assert len(endpoint.writes) == len(script)
        assert result["source_return_code"] == (1 if first == "error" else 0)
        assert result["ok"] is (first == "success")
        assert all(r["attempt_ordinal"] == 1 for r in rows(owner))


def test_board_absolute_retains_four_mvp_writes_and_separate_preparation(monkeypatch):
    with motor_transport(["success", "success", "timeout", "timeout", "timeout", "error"]) as (driver, router, endpoint, waits, owner):
        # Only external machine configuration is faked. GAP, query-stop, MVP,
        # caches, event window, and receipt generation are production methods.
        monkeypatch.setattr(driver, "oem_no24v_state", lambda: False)
        monkeypatch.setattr(driver, "_oem_board_state", lambda: {5: True})
        monkeypatch.setattr(driver, "_motion_oem_axis_profile", lambda *a, **kw: {"axis_min_steps": 0, "axis_max_steps": 1000})
        result = driver.motor_oem_move_absolute(5, 321, wait_for_stop=False)
        retained = rows(owner)
        assert [r["expected_command"] for r in retained] == [6, 138, 4, 4, 4, 4]
        assert [r["transport_call_ordinal"] for r in retained] == [1, 2, 3, 3, 4, 4]
        assert result["low_level_source_return_code"] == 0
        assert result["ack"] is None and result["retry_ack"]["status"] == 2


@pytest.mark.parametrize("method,args,commands", [
    ("motor_move_relative", (5, 20), [138, 4, 4]),
    ("motor_set_home", (5,), [5, 5]),
    ("motor_set_axis_param", (5, 5, 20), [5, 5]),
    ("motor_get_position", (5,), [6, 6]),
    ("motor_get_axis_param", (5, 6), [6, 6]),
])
def test_non_leaf_retry_primitives_have_only_one_transport_call(method, args, commands):
    script = (["success"] if method == "motor_move_relative" else []) + ["timeout", "timeout"]
    with motor_transport(script) as (driver, router, endpoint, waits, owner):
        getattr(driver, method)(*args)
        retained = rows(owner)
        assert [r["expected_command"] for r in retained] == commands
        assert retained[-2]["transaction_id"] == retained[-1]["transaction_id"]


def configure_no_motion(driver, monkeypatch):
    monkeypatch.setattr(driver, "_motion_oem_axis_profile", lambda *a, **kw: {
        "board": 5, "motor": 0, "speed": 1700, "acc": 350, "run_current": 31, "stall_guard": 16})
    monkeypatch.setattr(driver, "_machine_config_offset_int", lambda key, default: (default, "offline fixture"))
    monkeypatch.setattr(driver, "_motion_oem_gripper_version", lambda: 1)


@pytest.mark.parametrize("component,motor_calls", [("x", 4), ("z", 5)])
def test_no_motion_selects_motor_sap_gap_but_not_global_commands(monkeypatch, component, motor_calls):
    # Z includes readMaxCurrent GAP; global has eleven nonmotor calls.
    with motor_transport(["timeout"] * (2 * motor_calls + 11)) as (driver, router, endpoint, waits, owner):
        configure_no_motion(driver, monkeypatch)
        driver.oem_initialize_without_motion_test_case(board_wait={"ok": True}, components=[component, "global"])
        retained = rows(owner)
        motor = [r for r in retained if r["expected_command"] in {5, 6}]
        other = [r for r in retained if r["expected_command"] not in {5, 6}]
        assert len(motor) == 2 * motor_calls and len(other) == 11
        assert [r["attempt_ordinal"] for r in motor] == [1, 2] * motor_calls
        assert all(r["attempt_ordinal"] == 1 for r in other)


@pytest.mark.parametrize("family", ["io", "thermal", "chiller"])
def test_nonmotor_default_loops_and_recovery_are_preserved(family):
    with motor_transport(["timeout", "timeout", "timeout"]) as (driver, router, endpoint, waits, owner):
        recoveries = []
        driver.reconnect = lambda: recoveries.append("fixture_only_no_hardware")
        if family == "io":
            driver.deck_io_query_type(0)
        else:
            setattr(driver, "_" + family + "_last_tx_ts", 0)
            setattr(driver, "_" + family + "_noresp_streak", 8)
            getattr(driver, "_send_" + family)(9, 8, 0, -25)
        assert len(endpoint.writes) == 3
        assert len(recoveries) == 1
        assert [r["attempt_ordinal"] for r in rows(owner)] == [1, 1, 1]
        assert len({r["transaction_id"] for r in rows(owner)}) == 3


def test_selected_wrapper_ignores_generic_loop_and_recovery_defaults():
    with motor_transport(["timeout", "timeout"]) as (driver, router, endpoint, waits, owner):
        assert driver._send_motor(5, 6, 1, 0, 0, attempts=7,
            ordinary_motor_retry=True) is None
        assert len(endpoint.writes) == 2
        assert len({r["transaction_id"] for r in rows(owner)}) == 1
        assert driver._motor_noresp_streak[5] == 9


def test_selected_tx_only_never_waits_or_retries():
    with motor_transport(["timeout"]) as (driver, router, endpoint, waits, owner):
        result = driver.send_tmcl(5, 5, 1, 0, 0, wait_reply=False, ordinary_motor_retry=True)
        assert result["status"] == 100
        assert len(endpoint.writes) == 1 and waits == []
        assert rows(owner)[0]["outcome"] == "tx_only"


@pytest.mark.parametrize("family", ["pipette", "canopen"])
def test_router_selector_does_not_enable_other_families(family):
    # This is a selector exclusion test, not pipette/CANopen wire parity.
    with motor_transport(["timeout"]) as (driver, router, endpoint, waits, owner):
        result = router.transact(bytes(driver._build_frame(5, 6, 1, 0, 0)),
            matcher=router.tmcl_matcher(board_id=5, command=6),
            matcher_name="offline-exclusion", timeout_s=60, write_timeout_ms=55,
            provenance={"command_family": family}, ordinary_motor_retry=True)
        assert result["ok"] is False
        assert len(endpoint.writes) == 1 and rows(owner) == []
