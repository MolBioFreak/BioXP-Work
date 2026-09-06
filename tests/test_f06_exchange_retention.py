"""Offline caller-local retention slice; not durable operator-owner acceptance."""
from contextlib import contextmanager
from contextvars import copy_context
import queue
import threading

import pytest
import usb.core

from bioxp import novo_router as router_module
from bioxp.novo_router import NovoRouter
from bioxp.usb_driver import BioXpTester, novo_decode, novo_encode


@contextmanager
def transport(mode="timeout", error=None):
    incoming = queue.Queue()

    class Input:
        def read(self, size, timeout):
            try:
                return incoming.get(timeout=.005)
            except queue.Empty:
                raise TimeoutError("offline idle endpoint")

    class Output:
        def __init__(self):
            self.writes = []

        def write(self, raw, timeout):
            self.writes.append(bytes(raw))
            if error is not None:
                raise error
            if mode == "empty":
                router._pending.event.set()
            elif mode in {"success", "status_error", "malformed"}:
                data = bytes([5, 2 if mode == "status_error" else 100, 6, 0, 0, 0, 7, 0])
                wire = bytes(novo_encode(bytes([0, 0, 0, 0, 8]) + data))
                if mode == "malformed":
                    wire = wire[:-2] + bytes([wire[-2] ^ 1]) + wire[-1:]
                incoming.put(wire)
            return len(raw)

    output = Output()
    router = NovoRouter(ep_in=Input(), ep_out=output, decode=novo_decode)
    driver = object.__new__(BioXpTester)
    driver.novo_router = router
    driver._transport_lock = threading.RLock()
    driver._record_usb_sniff_ledger = lambda *a, **kw: None
    router.start()
    try:
        yield driver, router, output
    finally:
        router.shutdown()


def send(driver):
    return driver.send_tmcl(5, 6, 1, 0, 0, read_timeout_ms=1, max_reads=1)


@pytest.mark.parametrize("mode,outcome,signaled", [
    ("timeout", "timeout", False),
    ("empty", "signaled_empty", True),
    ("success", "response", True),
    ("status_error", "status_error", True),
    ("malformed", "malformed", False),
])
def test_driver_reduction_retains_finalized_exchange(monkeypatch, mode, outcome, signaled):
    retained = []
    monkeypatch.setattr(router_module, "publish_exchange", retained.append, raising=False)
    with transport(mode) as (driver, router, endpoint):
        result = send(driver)
        assert len(endpoint.writes) == 1  # This slice does NOT restore the resend yet.
        assert (result is None) == (mode in {"timeout", "empty", "malformed"})
        assert len(retained) == 1, "failed/no-frame provenance was discarded by driver"
        row = retained[0]
        assert row["outcome"] == outcome
        assert row["wait_signaled"] is signaled
        assert row["attempt_ordinal"] == 1
        assert row["exchange_id"] == row["transaction_id"] + ":1"
        assert row["tx_raw"] == list(endpoint.writes[0])
        assert row["write_attempted"] is True and row["write_returned"] is True
        assert row["owner_generation"] == router.reader_generation
        assert row["tx_timestamp"] <= row["tx_write_completed_at"] <= row["finalized_at"]
        assert row["physical_effect_verified"] is False
        assert "frames" not in row and "skipped_frames" not in row
        if result is not None:
            assert row["observed_status"] == result["status"]
            assert row["transaction_id"] == result["provenance"]["transaction_id"]


@pytest.mark.parametrize("error", [usb.core.USBTimeoutError("offline timeout"), usb.core.USBError("offline pipe"), ValueError("offline write error")])
def test_write_exception_is_retained_before_none_or_original_rethrow(monkeypatch, error):
    retained = []
    monkeypatch.setattr(router_module, "publish_exchange", retained.append, raising=False)
    with transport(error=error) as (driver, router, endpoint):
        if isinstance(error, usb.core.USBError):
            assert send(driver) is None
        else:
            with pytest.raises(ValueError) as caught:
                send(driver)
            assert caught.value is error
        assert len(endpoint.writes) == 1
        assert router._pending is None
        assert len(retained) == 1, "write exception bypassed exchange finalization"
        row = retained[0]
        assert row["outcome"] == "write_exception"
        assert row["write_attempted"] is True and row["write_returned"] is False
        assert row["wait_signaled"] is None
        assert row["exception"]["class"] == type(error).__name__


def test_caller_scope_reuses_root_dedupes_and_snapshot_is_detached():
    from bioxp.command_exchange_observer import exchange_scope, publish_exchange
    with transport() as (driver, router, endpoint):
        with exchange_scope("existing-command") as owner:
            assert send(driver) is None
            with exchange_scope("existing-child") as child:
                assert child is owner
                assert send(driver) is None
            snapshot = owner.snapshot()
            rows = snapshot["transport_exchanges"]
            assert len(rows) == 2 and len(endpoint.writes) == 2
            assert {r["command_id"] for r in rows} == {"existing-command"}
            assert [r["transport_call_ordinal"] for r in rows] == [1, 2]
            assert len({r["transaction_id"] for r in rows}) == 2
            publish_exchange(rows[0])
            rows[0]["tx_raw"].clear()
            assert len(owner.snapshot()["transport_exchanges"]) == 2
            assert owner.snapshot()["transport_exchanges"][0]["tx_raw"]
        # No ambient sink leaks past reset.
        assert send(driver) is None
        assert len(owner.snapshot()["transport_exchanges"]) == 2


def test_copied_worker_context_outlives_scope_and_keeps_same_owner():
    from bioxp.command_exchange_observer import exchange_scope
    with transport() as (driver, router, endpoint):
        with exchange_scope("existing-command") as owner:
            worker_context = copy_context()
        thread = threading.Thread(target=lambda: worker_context.run(send, driver))
        thread.start()
        thread.join(timeout=3)
        assert not thread.is_alive()
        assert len(endpoint.writes) == 1
        assert owner.snapshot()["transport_exchanges"][0]["command_id"] == "existing-command"


def test_observer_and_sink_failures_do_not_mask_write_exception(monkeypatch):
    from bioxp.command_exchange_observer import exchange_scope
    def broken(*args):
        raise RuntimeError("offline evidence failure")
    original = ValueError("original wire failure")
    with transport(error=original) as (driver, router, endpoint):
        with exchange_scope("existing-command", sink=broken) as owner:
            monkeypatch.setattr(owner, "append", broken)
            with pytest.raises(ValueError) as caught:
                send(driver)
            assert caught.value is original
            owner.flush()
            assert len(owner.snapshot()["transport_retention_errors"]) == 2
        assert len(endpoint.writes) == 1


def test_caller_local_anonymous_scope_claims_no_durable_command():
    from bioxp.command_exchange_observer import exchange_scope
    with transport() as (driver, router, endpoint):
        with exchange_scope() as owner:
            assert send(driver) is None
        row = owner.snapshot()["transport_exchanges"][0]
        assert row["command_id"] is None
        assert row["trace_id"]
        assert row["durable_ownership_claimed"] is False
