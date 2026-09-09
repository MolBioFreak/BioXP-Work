"""Offline event barriers: control-loop reachability, not physical Stop."""
import asyncio
import ast
from pathlib import Path
from typing import Any, Mapping
import importlib.util
import json
import os
import sys
import threading
import time

import pytest
from httpx import ASGITransport, AsyncClient
from test_operator_controls import make_app
from bioxp import operator_controls


@pytest.mark.parametrize('path', ['/operator/v2/dashboard', '/operator/v2/control-catalog', '/operator/dashboard', '/operator/control-catalog'])
def test_poll_does_not_starve_stop(tmp_path, monkeypatch, path):
    entered, release = threading.Event(), threading.Event()
    blocked = False
    calls = 0
    import test_operator_controls as fixtures
    install = fixtures.install_operator_control_plane
    execution_lock = threading.Lock()
    holder_ready = threading.Event()

    def install_with_locked_projection(app, **kwargs):
        original = kwargs['serial206_initialization_state_provider']
        def provider():
            nonlocal calls
            calls += 1
            if blocked:
                entered.set()
            with execution_lock:
                return original()
        kwargs['serial206_initialization_state_provider'] = provider
        return install(app, **kwargs)

    monkeypatch.setattr(fixtures, 'install_operator_control_plane', install_with_locked_projection)
    app, dispatched = make_app(tmp_path, monkeypatch)

    def motion():
        with execution_lock:
            holder_ready.set()
            release.wait(3)  # independent watchdog makes the baseline terminate
    async def run():
        nonlocal blocked
        async with AsyncClient(transport=ASGITransport(app=app), base_url='http://offline') as client:
            assert (await client.get(path)).status_code == 200
            holder = threading.Thread(target=motion)
            holder.start()
            assert await asyncio.to_thread(holder_ready.wait, 1)
            blocked = True
            poll = asyncio.create_task(client.get(path))
            start = time.monotonic()
            # A real thread observes entry even when the baseline loop blocks.
            assert await asyncio.to_thread(entered.wait, 4)
            stop = asyncio.create_task(client.post('/operator/actions/oem.abort_all', json={
                'expected_generation': 7, 'idempotency_key': 'poll-stop', 'inputs': {}}))
            while ('abort_all', None) not in dispatched and time.monotonic() - start < 5:
                await asyncio.sleep(0)
            assert ('abort_all', None) in dispatched
            elapsed = time.monotonic() - start
            assert elapsed < 1, f'Stop intake/HTTP delayed {elapsed:.3f}s by poll'
            assert (await poll).status_code == 200
            poll_times = []
            async def timed_poll():
                begin = time.monotonic()
                response = await client.get(path)
                poll_times.append(time.monotonic() - begin)
                return response
            responses = await asyncio.gather(*(timed_poll() for _ in range(40)))
            assert max(poll_times) < 1
            print(json.dumps({'path': path, 'stop_source_dispatch_s': elapsed,
                              'cached_poll_s': sorted(poll_times)}))
            assert all(r.status_code == 200 for r in responses)
            # One poll refresh plus the separate post-TX reconciliation reader.
            assert calls <= 3, 'poll flood queued provider refresh work'
            release.set()
            assert (await stop).status_code == 200
            assert ('abort_all', None) in dispatched
    try:
        asyncio.run(run())
    finally:
        release.set()
        cache = getattr(app.state, 'operator_poll_cache', None)
        if cache:
            cache.close()
        app.state.operator_command_plane.stop()


@pytest.mark.parametrize('source_time,expected_state', [(100.0, 'stale'), (1e12, 'missing')])
def test_producer_telemetry_strict_model_and_upstream_age(tmp_path, monkeypatch, source_time, expected_state):
    models_path = os.environ.get('BIOXP_BMS_OPERATOR_MODELS')
    assert models_path, 'Set BIOXP_BMS_OPERATOR_MODELS to the actual strict consumer module'
    spec = importlib.util.spec_from_file_location('control_plane_bms_models', models_path)
    models = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = models
    spec.loader.exec_module(models)
    # Execute the actual pure unavailable pipette producer, without importing
    # api.py (which would construct the runtime/hardware owner).
    tree = ast.parse((Path(__file__).parents[1] / 'src/bioxp/api.py').read_text())
    node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == '_operator_pipette_status')
    namespace = {'_pipette_transport': None, 'Any': Any, 'Mapping': Mapping}
    exec(compile(ast.Module(body=[node], type_ignores=[]), '<pure-api-producer>', 'exec'), namespace)
    app, _ = make_app(tmp_path, monkeypatch, pipette_status_provider=namespace['_operator_pipette_status'])
    app.state.operator_test_serial206.clear()  # fixture has no complete provider authority
    original_project = app.state.operator_test_hardware.project
    def old_source(domain):
        projected = original_project(domain)
        projected['domains'][domain]['observed_unix'] = source_time
        if domain == 'axes':
            projected['domains'][domain]['observation'] = {'rows': {
                'x': {'status': {'position': {'value': 123}, 'speed': {'value': 0}}},
                'z': {'status': {'position': {'value': 456}, 'speed': {'value': 0}}},
            }}
        return projected
    monkeypatch.setattr(app.state.operator_test_hardware, 'project', old_source)
    async def run():
        async with AsyncClient(transport=ASGITransport(app=app), base_url='http://offline') as client:
            body = (await client.get('/operator/v2/dashboard')).json()
            assert body.get('telemetry') is not None
            parsed = models.OperatorDashboardV2.model_validate(body)
            assert parsed.telemetry.snapshot['collection_triggered'] is False
            assert parsed.telemetry.snapshot['observed_at'] == source_time
            assert parsed.telemetry.snapshot['freshness']['state'] == expected_state
            if expected_state == 'missing':
                assert parsed.telemetry.snapshot['clock_skew_detected'] is True
            assert [row.position_steps for row in parsed.telemetry.axes] == [123, 456]
            print(json.dumps({'dashboard_bytes': len(json.dumps(body).encode()),
                              'snapshot': parsed.telemetry.snapshot}))
            assert len(json.dumps(body).encode()) < 128 * 1024
            first = body['telemetry']['snapshot']
            later = (await client.get('/operator/v2/dashboard')).json()['telemetry']['snapshot']
            assert later['snapshot_id'] == first['snapshot_id']
            assert later['observed_at'] == first['observed_at']
    try:
        asyncio.run(run())
    finally:
        cache = getattr(app.state, 'operator_poll_cache', None)
        if cache:
            cache.close()
        app.state.operator_command_plane.stop()
