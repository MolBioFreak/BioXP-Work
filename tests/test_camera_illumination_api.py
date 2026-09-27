"""Offline operator routes through the real provider and SMI ioctl leaf."""
from concurrent.futures import ThreadPoolExecutor
import threading

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from tests.test_camera_oem_led_binding import led_rig  # shared syscall-only device rig
from bioxp import camera_provider as camera
from bioxp.vision import oem_camera_led as smi


def expected(p, values=None):
    values = values or {}
    return {"schema_version": "bioxp.camera_illumination.v1",
            "provider_generation": p.generation,
            "channels": [{"channel": c, "on": values.get(c)} for c in (1, 2, 3)],
            "state_source": "last_successful_command", "physical_effect_verified": False}


@pytest.fixture
def api_client(led_rig, monkeypatch):
    from bioxp import api
    p, s, _ = led_rig
    monkeypatch.setattr(api, '_camera_provider', p)
    def forbidden(*args, **kwargs):
        raise AssertionError('no capture, stream lifecycle, or motion access')
    for name in ('capture', 'capture_inspection', 'begin_stream', 'end_stream'):
        monkeypatch.setattr(p, name, forbidden)
    monkeypatch.setattr(api, '_get_tester', forbidden)
    # Real registered routes without global application startup/hardware workers.
    app = FastAPI()
    app.router.routes.extend(r for r in api.app.routes
                             if getattr(r, 'path', '').startswith('/camera/illumination'))
    with TestClient(app) as client:
        yield api, client, p, s


def test_passive_get_unknown_no_discovery(api_client, monkeypatch):
    _, client, p, s = api_client
    monkeypatch.setattr(p, 'discover', lambda: pytest.fail('passive discovery'))
    assert client.get('/camera/illumination/state').json() == expected(p)
    assert not s.opened and not s.seen


@pytest.mark.parametrize('channel', [1, 2, 3])
@pytest.mark.parametrize('on', [True, False])
def test_post_exact_contract_and_inspection_observed(api_client, channel, on):
    _, client, p, s = api_client
    response = client.post('/camera/illumination', json={'channel': channel, 'on': on})
    assert response.status_code == 200
    assert response.json() == {**expected(p, {channel: on}), 'ok': True,
                              'channel': channel, 'on': on, 'delivery_attempted': True}
    p.set_illumination(channel=channel, on=not on)
    before = list(s.seen)
    assert client.get('/camera/illumination/state').json() == expected(p, {channel: not on})
    assert s.seen == before
    assert len(s.opened) == 1 and not s.closed


@pytest.mark.parametrize('body', [{}, {'channel': 0, 'on': True}, {'channel': 4, 'on': False},
    {'channel': True, 'on': True}, {'channel': '1', 'on': True}, {'channel': 1.0, 'on': True},
    {'channel': 1, 'on': 1}, {'channel': 1, 'on': 'false'}, {'channel': 1, 'on': None},
    {'channel': 1, 'on': True, 'force': True}, None, []])
def test_invalid_body_no_io(api_client, body):
    _, client, _, s = api_client
    assert client.post('/camera/illumination', json=body).status_code == 422
    assert not s.opened and not s.seen


@pytest.mark.parametrize('method,path,kwargs', [
    ('GET', '/camera/illumination/state?channel=1', {}),
    ('GET', '/camera/illumination/state', {'json': {'force': True}}),
    ('POST', '/camera/illumination?force=true', {'json': {'channel': 2, 'on': True}}),
])
def test_unknown_fields_no_io(api_client, method, path, kwargs):
    _, client, _, s = api_client
    assert client.request(method, path, **kwargs).status_code == 422
    assert not s.opened and not s.seen


@pytest.mark.parametrize('offset', range(1, 7))
def test_initialization_failure_not_success_or_retried(api_client, offset):
    _, client, p, s = api_client
    s.fail = offset
    assert client.post('/camera/illumination', json={'channel': 2, 'on': True}).status_code == 503
    assert len(s.seen) == offset and s.closed == [71]
    assert client.get('/camera/illumination/state').json() == expected(p)


@pytest.mark.parametrize('channel,offset', [(1, 1), (1, 4), (2, 4), (3, 1), (3, 2)])
def test_set_failure_clears_all_commands_without_retry(api_client, channel, offset):
    _, client, p, s = api_client
    p.command_illumination(channel=1, on=True)
    before = len(s.seen)
    s.fail = before + offset
    assert client.post('/camera/illumination', json={'channel': channel, 'on': False}).status_code == 503
    assert len(s.seen) == before + offset and s.closed == [71]
    assert client.get('/camera/illumination/state').json() == expected(p)


def test_preview_same_fd_init_once_and_no_stream_change(led_rig, monkeypatch):
    p, s, _ = led_rig
    p.begin_stream('preview')
    generation = p.generation
    for name in ('capture', 'capture_inspection', 'begin_stream', 'end_stream'):
        monkeypatch.setattr(p, name, lambda *a, **k: pytest.fail('video operation'))
    for channel in (1, 2, 3):
        for on in (True, False):
            assert p.command_illumination(channel=channel, on=on)['ok']
    assert sum(q == smi.UVC_GET_LEN for q, _ in s.seen) == 1
    assert len(s.opened) == 1 and not s.closed
    assert p.generation == generation and p._stream_owner == 'preview' and p._stream_accepting


@pytest.mark.parametrize('transition', ['begin', 'invalidate', 'end', 'close', 'generation', 'binding'])
def test_cached_state_invalidation_has_no_led_side_effect(led_rig, transition):
    p, s, usb = led_rig
    if transition in ('invalidate', 'end'):
        p.begin_stream('preview')
    p.command_illumination(channel=2, on=True)
    before = list(s.seen)
    if transition == 'begin': p.begin_stream('preview')
    elif transition == 'invalidate': p.invalidate_stream('preview')
    elif transition == 'end': p.end_stream('preview')
    elif transition == 'close': p.close()
    elif transition == 'generation': p._generation += 1
    else:
        (usb / 'descriptors').write_bytes(b'')
        with pytest.raises(camera.CameraUnavailable):
            p.set_illumination(channel=1, on=True)
    assert p.illumination_state() == expected(p)
    assert s.seen == before


def test_api_preview_commands_keep_owner_and_fd(api_client):
    _, client, p, s = api_client
    camera.CameraProvider.begin_stream(p, 'preview')
    generation = p.generation
    for channel in (1, 2, 3):
        for on in (True, False):
            assert client.post('/camera/illumination', json={'channel': channel, 'on': on}).status_code == 200
    assert len(s.opened) == 1 and not s.closed
    assert sum(q == smi.UVC_GET_LEN for q, _ in s.seen) == 1
    assert p.generation == generation and p._stream_owner == 'preview' and p._stream_accepting


def test_operator_new_binding_reinitializes_without_old_command_values(led_rig):
    p, s, _ = led_rig
    p.command_illumination(channel=1, on=True)
    p.begin_stream('preview')
    result = p.command_illumination(channel=2, on=False)
    assert result == {**expected(p, {2: False}), 'ok': True,
                      'channel': 2, 'on': False, 'delivery_attempted': True}
    assert len(s.opened) == 2 and s.closed == [71]
    assert sum(q == smi.UVC_GET_LEN for q, _ in s.seen) == 2


def test_probe_then_operator_reinitializes_and_inspection_init_reused(led_rig):
    p, s, _ = led_rig
    p.initialize_illumination()
    before = len(s.seen)
    p.command_illumination(channel=3, on=True)
    assert len(s.seen) == before + 2
    p.probe_illumination()  # existing leaf's probe clears its DSP discovery
    p.command_illumination(channel=3, on=False)
    assert p.illumination_state() == expected(p, {3: False})


def test_concurrent_get_waits_for_existing_transaction_lock(led_rig, monkeypatch):
    p, _, _ = led_rig
    p.initialize_illumination()
    entered, release, reading = threading.Event(), threading.Event(), threading.Event()
    original = p._led_leaf.set_led
    def blocked(channel, on):
        entered.set()
        assert release.wait(5)
        original(channel, on)
    monkeypatch.setattr(p._led_leaf, 'set_led', blocked)
    def read():
        reading.set()
        return p.illumination_state()
    with ThreadPoolExecutor(2) as pool:
        write = pool.submit(p.command_illumination, channel=3, on=True)
        assert entered.wait(5)
        pending = pool.submit(read)
        assert reading.wait(5) and not pending.done()
        release.set()
        assert write.result()['ok']
        assert pending.result() == expected(p, {3: True})


@pytest.mark.parametrize('method,path', [('GET', '/camera/illumination/state'), ('POST', '/camera/illumination')])
def test_provider_replacement_never_publishes_stale_success(api_client, monkeypatch, method, path):
    api, client, p, _ = api_client
    original = api.run_in_threadpool
    async def replace(func, *args, **kwargs):
        result = await original(func, *args, **kwargs)
        api._camera_provider = camera.CameraProvider(generation=99)
        return result
    monkeypatch.setattr(api, 'run_in_threadpool', replace)
    kwargs = {'json': {'channel': 3, 'on': True}} if method == 'POST' else {}
    assert client.request(method, path, **kwargs).status_code == 503
    assert api._camera_provider.illumination_state() == expected(api._camera_provider)
