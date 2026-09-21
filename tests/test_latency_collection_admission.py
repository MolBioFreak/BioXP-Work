"""Bound background refresh, preserve worker lifetime, narrow direct prerequisites."""
import asyncio
import threading
from contextlib import nullcontext

import pytest
from bioxp import api


def test_direct_motion_prerequisite_is_narrow_and_not_self_preempting(monkeypatch):
    seen = []
    def collect(domains, **kwargs):
        seen.append((domains, kwargs))
        return {'ok': True, 'published': True}
    async def blocking(label, fn, **kwargs): return fn()
    monkeypatch.setattr(api, '_run_blocking', blocking)
    monkeypatch.setattr(api, '_collect_and_publish_hardware_snapshot', collect)
    assert asyncio.run(api._collect_motion_admission_snapshot())['published']
    domains, options = seen[0]
    assert set(domains) == {'transport', 'boards', 'power', 'interlock', 'latch', 'axes', 'gripper'}
    assert options['warm_deck_authority'] is False
    assert not options.get('automatic', False)


def test_full_explicit_collection_contract_is_unchanged(monkeypatch):
    seen = []
    async def blocking(label, fn, **kwargs): return fn()
    monkeypatch.setattr(api, '_run_blocking', blocking)
    monkeypatch.setattr(api, '_collect_and_publish_hardware_snapshot',
                        lambda domains, **kw: seen.append((domains, kw)) or {'ok': True})
    asyncio.run(api.hardware_snapshot_collect())
    assert seen[0][0] == list(api.DEFAULT_HARDWARE_SNAPSHOT_DOMAINS)
    assert seen[0][1]['automatic'] is False


def test_duplicate_automatic_reads_do_not_build_a_waiting_queue(monkeypatch):
    async def exercise():
        monkeypatch.setattr(api, '_automatic_snapshot_pending', threading.Lock())
        monkeypatch.setattr(api, '_tester_lock', asyncio.Lock())
        monkeypatch.setattr(api.app.state, 'operator_normal_action_active', lambda: False, raising=False)
        entered, release = asyncio.Event(), asyncio.Event()
        calls = []
        async def blocking(label, fn, *, on_finished=None, **kwargs):
            calls.append(label)
            entered.set()
            await release.wait()
            if on_finished: on_finished()
            return {'ok': False, 'published': False}
        monkeypatch.setattr(api, '_run_blocking', blocking)
        first = asyncio.create_task(api.hardware_snapshot_collect({'automatic': True, 'domains': ['axes']}))
        await asyncio.wait_for(entered.wait(), 2)
        try:
            results = await asyncio.gather(*(api.hardware_snapshot_collect(
                {'automatic': True, 'domains': ['latch']}) for _ in range(20)))
            assert all(r == {'ok': False, 'published': False, 'reason': 'hardware_collection_in_progress'} for r in results)
            assert len(calls) == 1
        finally:
            release.set()
            await first
        assert not api._automatic_snapshot_pending.locked()
    asyncio.run(exercise())


@pytest.mark.parametrize('end', ['cancel', 'timeout', 'error', 'success'])
def test_completion_slot_follows_actual_worker_not_http_waiter(monkeypatch, end):
    async def exercise():
        monkeypatch.setattr(api, '_tester_lock', asyncio.Lock())
        monkeypatch.setattr(api, '_protocol_mutation_scope', lambda _: nullcontext())
        entered, release = asyncio.Event(), asyncio.Event()
        completions = []
        async def pool(fn):
            entered.set()
            await release.wait()
            return fn()
        monkeypatch.setattr(api, 'run_in_threadpool', pool)
        def work():
            if end == 'error': raise ValueError('injected')
            return 42
        task = asyncio.create_task(api._run_blocking('offline', work,
            timeout_s=.03 if end == 'timeout' else None,
            on_finished=lambda: completions.append('done')))
        await asyncio.wait_for(entered.wait(), 2)
        if end == 'cancel':
            task.cancel()
            with pytest.raises(asyncio.CancelledError): await task
        elif end == 'timeout':
            with pytest.raises(api.HTTPException): await task
        assert not completions
        release.set()
        if end == 'error':
            with pytest.raises(ValueError): await task
        elif end == 'success':
            assert await task == 42
        for _ in range(10): await asyncio.sleep(0)
        assert completions == ['done']
        assert not api._tester_lock.locked()
    asyncio.run(exercise())
