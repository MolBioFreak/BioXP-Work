"""Bounded real HOME reconciliation vs passive provider/runtime polling."""
import faulthandler
import os
import subprocess
import sys
import threading

import pytest
from fastapi.testclient import TestClient
from tests.test_deck_home_recovery import (
    homed_replacement, home_body, raw_command, stopped_failure, native_routes,
)
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_scoped_authority import retained_rig


def test_home_reconcile_poll_lock_order(homed_replacement, monkeypatch):
    if os.environ.get('DECK_RECONCILE_LOCK_CHILD') != '1':
        try:
            result = subprocess.run([sys.executable, '-m', 'pytest', '-p', 'no:cacheprovider',
                '-p', 'tests.z_stop_offline_guard', '-s', '-q',
                __file__ + '::test_home_reconcile_poll_lock_order'],
                env=dict(os.environ, DECK_RECONCILE_LOCK_CHILD='1'),
                capture_output=True, text=True, timeout=18)
        except subprocess.TimeoutExpired as exc:
            pytest.fail('bounded reconciliation deadlock:\n' + (exc.stderr or b'').decode())
        assert result.returncode == 0, result.stdout + result.stderr
        return
    faulthandler.dump_traceback_later(10)
    app, provider, primitive, refs, root, leaf, data = homed_replacement
    store = app.state.operator_command_plane.store
    before = raw_command(store, data['command_id'])
    request = home_body(provider)
    original = store.reconcile_deck_recovery
    original_acquire = provider._lock.acquire
    polled = []
    def reconcile(*args, **kwargs):
        start = threading.Event()
        held = threading.Event()
        attempted = threading.Event()
        owner = threading.get_ident()
        def acquire(*a, **k):
            if threading.get_ident() == owner and not start.is_set():
                start.set()
                assert held.wait(3)
                attempted.set()
            return original_acquire(*a, **k)
        monkeypatch.setattr(provider._lock, 'acquire', acquire)
        def poll():
            assert start.wait(3)
            with provider._lock:
                held.set()
                assert attempted.wait(3)
                with provider.projection_scope():
                    provider.state_store.read_oem_serial206_initialization_state()
                    polled.append(True)
        thread = threading.Thread(target=poll, name='offline-home-reconcile-poll')
        thread.start()
        result = original(*args, **kwargs)
        thread.join(3)
        assert not thread.is_alive()
        monkeypatch.setattr(provider._lock, 'acquire', original_acquire)
        return result
    monkeypatch.setattr(store, 'reconcile_deck_recovery', reconcile)
    response = TestClient(app).post('/operator/recovery/deck/' + data['command_id'] + '/reconcile', json=request)
    assert response.status_code == 200, response.text
    assert polled == [True]
    assert raw_command(store, data['command_id']) == before
    assert store.deck_recovery_blocker() is None
    assert leaf.moves == []
    faulthandler.cancel_dump_traceback_later()
