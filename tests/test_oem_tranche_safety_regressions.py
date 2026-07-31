from __future__ import annotations

import asyncio
import threading

import pytest


def test_blocking_timeout_keeps_exclusion_until_worker_has_stopped():
    from fastapi import HTTPException
    from src.bioxp import api

    started = threading.Event()
    release = threading.Event()
    events: list[str] = []

    def worker():
        events.append("started")
        started.set()
        assert release.wait(2.0)
        events.append("stopped")
        return {"ok": True}

    async def scenario():
        task = asyncio.create_task(api._run_blocking("non-cancellable", worker, timeout_s=0.01))
        assert await asyncio.to_thread(started.wait, 1.0)
        with pytest.raises(HTTPException) as exc_info:
            await task
        assert exc_info.value.status_code == 504
        assert exc_info.value.detail["connection_transition_blocked_until_worker_exit"] is True

        # The timed-out HTTP waiter may finish, but the non-cancellable worker
        # must retain the tester lease. Prove the exclusion directly instead
        # of equating it with the lifetime of the response task.
        after_started = threading.Event()

        def after_worker():
            events.append("after")
            after_started.set()
            return {"ok": True}

        after_task = asyncio.create_task(api._run_blocking("after-timeout", after_worker, timeout_s=1.0))
        await asyncio.sleep(0.05)
        assert after_started.is_set() is False
        assert after_task.done() is False
        release.set()
        assert await after_task == {"ok": True}
        assert events == ["started", "stopped", "after"]

    asyncio.run(scenario())


def test_arbitrary_tmcl_success_does_not_invent_board_initialized_state():
    from src.bioxp.usb_driver import BioXpTester

    class Router:
        @staticmethod
        def tmcl_matcher(**_kwargs):
            return lambda _frame: True

        @staticmethod
        def transact(*_args, **_kwargs):
            return {"ok": True, "frames": [{"data": [4, 100, 6, 0, 0, 0, 1, 0], "raw": []}]}

    tester = BioXpTester.__new__(BioXpTester)
    tester.novo_router = Router()
    tester._oem_initialized_boards = set()
    tester._record_usb_sniff_ledger = lambda *_args, **_kwargs: None

    result = tester._send_tmcl_locked(4, 6, 9, 0, 0)

    assert result["status"] == 100
    assert tester._oem_initialized_boards == set()


def test_activate_board_owns_initialized_state_and_status_two_is_chiller_only(monkeypatch):
    from src.bioxp.usb_driver import BioXpTester

    tester = BioXpTester.__new__(BioXpTester)
    tester._oem_initialized_boards = set()
    tester.enable_motor_power = lambda: None
    monkeypatch.setattr("src.bioxp.usb_driver.time.sleep", lambda _seconds: None)

    replies = {
        4: {"status": 100},
        5: {"status": 2},
        6: None,
        7: {"status": 2},
    }
    tester.send_tmcl_retry = lambda board, *_args, **_kwargs: replies[int(board)]

    tester.activate_boards(expect_reply=True)

    assert tester._oem_initialized_boards == {4, 7}


def test_repeatable_initial_check_refuses_active_operation_without_hardware_calls():
    from src.bioxp.lifecycle_state import CanonicalLifecycleOwner, LifecycleStateError

    owner = CanonicalLifecycleOwner()
    owner.transport_changed(True, reason="test")
    owner.run_stage("constructor_pipette_stage", lambda: {"ok": True})
    owner.run_stage("initialization_without_motion", lambda: {"ok": True})
    owner.run_stage("initial_check", lambda: {"ok": True})
    owner.transition("running", reason="active_job")

    class Hardware:
        def __getattr__(self, name):
            raise AssertionError(f"hardware method must not be called: {name}")

    with pytest.raises(LifecycleStateError, match="active operation"):
        owner.run_initial_check(Hardware(), can_ready=lambda: True)

    projection = owner.projection()
    assert projection["operation_state"] == "running"
    assert projection["operation_reason"] == "active_job"
    assert projection["startup"]["stages"]["initial_check"]["state"] == "passed"


def test_q1_status_rejects_oem_error_collection_byte_after_status_header():
    from src.bioxp.can_driver import BioXpCanDriver

    driver = BioXpCanDriver.__new__(BioXpCanDriver)
    driver._send_pipette_command = lambda *_args, **_kwargs: {
        "ok": True,
        "ack": {"data": [0x20, 0x60, 0x20, 0x45]},
        "error": None,
    }

    result = driver.query_status()

    assert result["reply_received"] is True
    assert result["semantic_ok"] is True
    assert result["ok"] is False
    assert result["oem_error_code"] == 0x45
    assert result["oem_error_codes"] == [0x45]
