import asyncio
import threading

import pytest
from fastapi import HTTPException

from bioxp.operator_controls import _OperatorPollCache


def test_cancelled_cold_waiter_does_not_release_singleflight():
    cache = _OperatorPollCache()
    entered, release = threading.Event(), threading.Event()
    calls = []

    @cache.wrap
    async def first():
        calls.append('first')
        entered.set()
        release.wait(3)
        return {'upstream': 'same'}

    @cache.wrap
    async def other():
        calls.append('other')
        return {}

    async def run():
        waiter = asyncio.create_task(first())
        assert await asyncio.to_thread(entered.wait, 1)
        waiter.cancel()
        with pytest.raises(asyncio.CancelledError):
            await waiter
        for _ in range(50):
            with pytest.raises(HTTPException) as error:
                await other()
            assert error.value.status_code == 503
        assert calls == ['first']
        release.set()
        assert await first() == {'upstream': 'same'}
    try:
        asyncio.run(run())
    finally:
        release.set()
        cache.close()


def test_cached_projection_ages_at_exact_boundary(monkeypatch):
    cache = _OperatorPollCache()
    release = threading.Event()
    now = [100.0]
    monkeypatch.setattr('bioxp.operator_controls.time.monotonic', lambda: now[0])

    @cache.wrap
    async def catalog():
        release.wait(3)
        return {}

    key = ('catalog', None)
    cache._cache[key] = ({'generated_at': 25.0,
                          'actions': [{'enabled': True, 'interrupt': False},
                                      {'enabled': True, 'interrupt': True}],
                          'snapshot': {'snapshot_id': 'device-observation', 'observed_at': 25.0,
                                       'freshness': {'state': 'fresh', 'age_s': 0.0, 'fresh_for_s': 15.0}}}, 100.0)
    async def run():
        now[0] = 114.999
        before = await catalog()
        assert before['actions'][0]['enabled'] is True
        now[0] = 115.0
        after = await catalog()
        assert after['actions'][0]['enabled'] is False
        assert after['actions'][1]['enabled'] is True
        assert after['snapshot']['freshness']['state'] == 'stale'
        assert after['snapshot']['observed_at'] == 25.0
        assert after['generated_at'] == 25.0
        assert cache._cache[key][0]['actions'][0]['enabled'] is True
    try:
        asyncio.run(run())
    finally:
        release.set()
        cache.close()


def test_cold_timeout_and_shutdown_keep_worker_owned():
    import time
    cache = _OperatorPollCache()
    release = threading.Event()
    entered = threading.Event()
    calls = []
    @cache.wrap
    async def blocked():
        calls.append(1)
        entered.set()
        release.wait(3)
        return {}
    async def run():
        started = time.monotonic()
        with pytest.raises(HTTPException) as error:
            await blocked()
        assert error.value.detail == 'operator_poll_warming'
        assert time.monotonic() - started < 1
        assert entered.is_set()
        pending = cache._pending
        assert pending.running()
        cache.close()
        assert pending.running(), 'close cannot kill provider work'
        with pytest.raises(HTTPException) as closed:
            await blocked()
        assert closed.value.detail == 'operator_poll_closed'
        assert calls == [1]
        release.set()
        await asyncio.wrap_future(pending)
        assert pending.done()
    try:
        asyncio.run(run())
    finally:
        release.set()
        cache.close()


def test_hot_catalog_cannot_starve_requested_cold_dashboard():
    cache = _OperatorPollCache()
    release = threading.Event()
    calls = []
    @cache.wrap
    async def catalog():
        calls.append('catalog')
        if len(calls) > 1:
            release.wait(3)
        return {'catalog': True}
    @cache.wrap
    async def dashboard():
        calls.append('dashboard')
        return {'dashboard': True}
    async def run():
        await catalog()
        await catalog()  # blocked refresh owns the only worker
        with pytest.raises(HTTPException):
            await dashboard()
        release.set()
        await asyncio.wrap_future(cache._pending)
        for _ in range(100):
            await catalog()
        assert calls == ['catalog', 'catalog']
        assert await dashboard() == {'dashboard': True}
    try:
        asyncio.run(run())
    finally:
        release.set()
        cache.close()


def test_generation_change_never_publishes_old_enabled_controls():
    generation = [7]
    cache = _OperatorPollCache(lambda: generation[0])
    release = threading.Event()
    block = [False]
    @cache.wrap
    async def catalog():
        captured = generation[0]
        if block[0]:
            release.wait(3)
        return {'ownership_generation': captured, 'actions': [{'enabled': True}]}
    async def run():
        assert (await catalog())['ownership_generation'] == 7
        block[0] = True
        await catalog()
        generation[0] = 8
        with pytest.raises(HTTPException) as stale:
            await catalog()
        assert stale.value.status_code == 503
        release.set()
        try:
            await asyncio.wrap_future(cache._pending)
        except HTTPException:
            pass
        # A subsequent refresh can repair the projection, never return epoch7.
        for _ in range(3):
            try:
                row = await catalog()
            except HTTPException:
                await asyncio.wrap_future(cache._pending)
                continue
            assert row['ownership_generation'] == 8
            return
        pytest.fail('new generation failed to recover')
    try:
        asyncio.run(run())
    finally:
        release.set()
        cache.close()


def test_oversized_telemetry_has_explicit_error_and_full_detail_path():
    from bioxp.operator_controls import _bounded_telemetry
    with pytest.raises(HTTPException) as error:
        _bounded_telemetry({'serial206_initialization_provider': {
            'z_authority': {'last_observation': 'x' * (64 * 1024)}}})
    assert error.value.status_code == 503
    assert error.value.detail == {'error': 'telemetry_projection_exceeds_64kib',
                                  'detail_path': '/operator/dashboard'}
