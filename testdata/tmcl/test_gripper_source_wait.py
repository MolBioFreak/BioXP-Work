"""Offline API custody tests; callbacks are explicit no-hardware doubles."""
import asyncio
import threading

import pytest
from fastapi import HTTPException

from bioxp import api


@pytest.mark.parametrize("route,primitive,expected", [
    ("motion_gripper_open", "gripper_open", None),
    ("motion_gripper_open_wide", "gripper_open_wide", None),
    ("motion_gripper_close", "gripper_close", 30.0),
])
def test_manual_route_uses_source_wait_and_retains_custody(monkeypatch, route, primitive, expected):
    began, release, next_began = threading.Event(), threading.Event(), threading.Event()
    tester = object()
    seen = []
    original = api._run_blocking

    def source(actual_tester, **kwargs):
        assert actual_tester is tester
        assert kwargs["timeout_s"] == 20.0
        began.set()
        assert release.wait(2), "offline test did not release source callback"
        return {"ok": True, "physical_effect_verified": False, "offline_double": True}

    async def trace(label, func, timeout_s=30.0):
        seen.append(timeout_s)
        return await original(label, func, timeout_s)

    monkeypatch.setattr(api, "_require_motion_route_ready", lambda: None)
    monkeypatch.setattr(api, "_get_tester", lambda: tester)
    monkeypatch.setattr(api, primitive, source)
    monkeypatch.setattr(api, "_run_blocking", trace)

    async def scenario():
        monkeypatch.setattr(api, "_tester_lock", asyncio.Lock())
        task = asyncio.create_task(getattr(api, route)())
        second = None
        try:
            assert await asyncio.to_thread(began.wait, 1)
            assert seen == [expected]
            assert api._tester_lock.locked()
            second = asyncio.create_task(original("next ordinary operation", next_began.set, 1))
            await asyncio.sleep(0.02)
            assert not next_began.is_set()
            release.set()
            result = await asyncio.wait_for(task, 1)
            assert result["ok"] and result["physical_effect_verified"] is False
            await asyncio.wait_for(second, 1)
            assert next_began.is_set()
        finally:
            release.set()
            await asyncio.gather(task, *([second] if second else []), return_exceptions=True)
    asyncio.run(scenario())


@pytest.mark.parametrize("cancel", [True, False], ids=["caller-cancel", "caller-deadline"])
def test_waiter_exit_retains_ordinary_custody_and_stop_lane(monkeypatch, cancel):
    began, release, stopped, next_began = [threading.Event() for _ in range(4)]

    def source():
        began.set()
        assert release.wait(2)
        return {"ok": True, "physical_effect_verified": False}

    async def scenario():
        monkeypatch.setattr(api, "_tester_lock", asyncio.Lock())
        monkeypatch.setattr(api, "_tester_transition_lock", asyncio.Lock())
        monkeypatch.setattr(api, "_get_tester", lambda: object())
        task = asyncio.create_task(api._run_blocking("source-owned recovery", source, None if cancel else 0.02))
        second = None
        try:
            assert await asyncio.to_thread(began.wait, 1)
            if cancel:
                task.cancel()
                with pytest.raises(asyncio.CancelledError):
                    await task
            else:
                with pytest.raises(HTTPException) as failure:
                    await task
                assert failure.value.status_code == 504
                assert failure.value.detail["retry_forbidden"] is True
                assert failure.value.detail["connection_transition_blocked_until_worker_exit"] is True
            assert api._tester_lock.locked()
            second = asyncio.create_task(api._run_blocking("next ordinary", next_began.set, 1))
            await api._run_safety_interrupt_blocking("offline addressed Stop", lambda _: stopped.set(), 1)
            assert stopped.is_set() and not next_began.is_set()
            release.set()
            await asyncio.wait_for(second, 1)
        finally:
            release.set()
            await asyncio.gather(task, *([second] if second else []), return_exceptions=True)
    asyncio.run(scenario())


def test_source_timeout_is_not_a_nonexistent_wrapper_deadline(monkeypatch):
    def source():
        raise TimeoutError("source failure")

    async def scenario():
        monkeypatch.setattr(api, "_tester_lock", asyncio.Lock())
        with pytest.raises(TimeoutError, match="source failure"):
            await api._run_blocking("source-owned wait", source, None)
        assert not api._tester_lock.locked()
    asyncio.run(scenario())
