"""Queued named execution with real provider/native adapter and retained SQLite.

Only the declared controller leaf is doubled. The bounded child process makes
an opposite-order lock regression a failure rather than a hung suite.
"""
import faulthandler
import json
import os
import sqlite3
import subprocess
import sys
import threading
import time

import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig, qualify_test_references
from tests.test_deck_scoped_integration import installed_retained, catalog_action, catalog_payload
from tests.test_deck_near_terminal import NearUSB


@pytest.mark.parametrize('drift', [False, True], ids=['poll', 'owner-drift'])
def test_queued_populated_finalization_and_reopen(installed_retained, retained_rig, monkeypatch, drift):
    scenario = 'owner-drift' if drift else 'poll'
    if os.environ.get('DECK_TRANSACTION_CHILD') != scenario:
        try:
            result = subprocess.run([sys.executable, '-m', 'pytest', '-p', 'no:cacheprovider',
                '-p', 'tests.z_stop_offline_guard', '-s', '-q',
                f'{__file__}::test_queued_populated_finalization_and_reopen[{scenario}]'],
                env=dict(os.environ, DECK_TRANSACTION_CHILD=scenario), capture_output=True,
                text=True, timeout=35)
        except subprocess.TimeoutExpired as exc:
            pytest.fail('bounded queued finalization deadlock: ' + str(exc.stderr))
        assert result.returncode == 0, result.stdout + result.stderr
        return
    faulthandler.dump_traceback_later(25)
    from bioxp import api
    from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
    from bioxp.serial206_y_provider import Serial206YProvider
    app, provider, observations, references, root = installed_retained
    runtime = retained_rig[2]
    plane = app.state.operator_command_plane
    store = plane.store
    generation = int(provider.generation_provider())
    assert catalog_action(app)['enabled'] is True  # intent availability; executor still checks references
    qualify_test_references(references)
    assert api._collect_and_publish_hardware_snapshot(['axes', 'latch'],
        reason='isolated-transaction-qualification')['deck_authority']['enabled']
    leaf = NearUSB((0, 0))
    original_wait = leaf.motor_wait_target_reached
    def wait(*args, **kwargs):
        result = original_wait(*args, **kwargs)
        result['event']['owner_generation'] = generation
        return result
    monkeypatch.setattr(leaf, 'motor_wait_target_reached', wait)
    monkeypatch.setattr(leaf, 'begin_bus_event_window', lambda: {
        'after_sequence': 0, 'receive_owner': 'offline-usb', 'owner_generation': generation})
    adapter = Serial206ProductionPrimitiveAdapter(leaf, None, authority_provider=lambda: {},
        generation_provider=lambda: generation, reference_store=references)
    adapter.y_provider = Serial206YProvider(leaf, state_store=runtime,
        generation_provider=lambda: generation, reference_store=references)
    monkeypatch.setattr(observations, 'oem_move_to', adapter.oem_move_to)
    # Observe all live owner callbacks made by this queued path. Acquiring
    # the provider scope must precede the shared writer at each such callback.
    callbacks = []
    original_reader = store._deck_owner_authority_reader
    def reader():
        if store._lock._owner_thread_id == threading.get_ident():
            assert provider._lock._owner == threading.get_ident(), 'writer-held live callback lacks provider scope'
        callbacks.append(threading.current_thread().name)
        return original_reader()
    monkeypatch.setattr(store, '_deck_owner_authority_reader', reader)
    original_commit = store.commit_deck_success
    original_acquire = provider._lock.acquire
    attempting = threading.Event()
    finalizer_id = [None]
    polls = []
    def acquire(*args, **kwargs):
        if threading.get_ident() == finalizer_id[0]:
            attempting.set()
        return original_acquire(*args, **kwargs)
    monkeypatch.setattr(provider._lock, 'acquire', acquire)
    def commit(command_id, *args, **kwargs):
        assert command_id in store.live_command_worker_ids()
        with sqlite3.connect(root / 'bioxp_runtime.db') as db:
            stage = db.execute('SELECT terminal_state,terminal_evidence_json FROM '
                'operator_plane_deck_stages WHERE command_id=? AND stage_order=3',
                (command_id,)).fetchone()
        assert stage[0] == 'completed'
        assert json.loads(stage[1])['controller_completion_verified'] is True
        holding = threading.Event()
        attempting.clear()
        def poll():
            with provider._lock:
                holding.set()
                assert attempting.wait(4)
                with runtime.serial206_projection_scope():
                    runtime.read_oem_serial206_initialization_state()
                    store.command_detail_v2(command_id)
                    if drift:
                        monkeypatch.setattr(provider, 'generation_provider', lambda: generation + 1)
                polls.append(command_id)
        worker = threading.Thread(target=poll, daemon=True)
        worker.start()
        assert holding.wait(4)
        finalizer_id[0] = threading.get_ident()
        try:
            return original_commit(command_id, *args, **kwargs)
        finally:
            finalizer_id[0] = None
            worker.join(4)
            assert not worker.is_alive()
    monkeypatch.setattr(store, 'commit_deck_success', commit)
    client = TestClient(app)
    receipts = {}
    plane.start()
    for index, target in enumerate(['LOC_OC'] if drift else ['LOC_OC', 'LOC_MS']):
        # Explicit collection is the existing next-command freshness owner;
        # a cached disabled catalog is not admission authority.
        if index:
            assert api._collect_and_publish_hardware_snapshot(['axes', 'latch'],
                reason='isolated-next-command')['deck_authority']['enabled']
        action = catalog_action(app)
        response = client.post('/operator/v2/actions/oem.deck.move_to_location', json={
            'schema_version': 'bioxp.operator_action_request.v2',
            'idempotency_key': f'queued-transaction-{index}',
            'expected_ownership_generation': generation,
            'expected_board_epoch_by_board': action['expected_board_epoch_by_board'],
            'inputs': {'target': target, 'camera_offset': False}})
        assert response.status_code == 200, response.text
        cid = response.json()['command_id']
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            compact = client.get('/operator/v2/actions/receipts/' + cid).json()
            if compact['status'] not in {'queued', 'dispatched', 'issued_pending'}:
                break
            time.sleep(.01)
        detail = client.get('/operator/v2/actions/receipts/' + cid + '?detail=true').json()
        assert detail['status'] == ('ambiguous' if drift else 'completed'), json.dumps(detail, indent=2)
        assert detail['deck_movement']['semantic_state_committed'] is (not drift)
        assert detail['deck_movement']['controller_completion_verified'] is True
        assert detail['physical_effect_verified'] is False
        receipts[cid] = {'compact': compact, 'detail': detail}
        # Warm populated catalog/receipt consumers repeatedly, never re-submit.
        for _ in range(3):
            assert client.get('/operator/v2/actions/receipts/' + cid + '?detail=true').json() == detail
            assert len(catalog_payload(app)['actions']) > 0
    assert len(polls) == len(receipts)
    assert any(name.startswith('bioxp-operator-command-') for name in callbacks)
    # LOC_OC -> LOC_MS shares X; native near-axis branch moves Y only.
    assert len(leaf.moves) == (2 if drift else 3)
    plane.stop()
    script = ('import json,sys; from tests.test_deck_scoped_integration import fresh_process_receipts; '
        'print(json.dumps({cid:fresh_process_receipts(sys.argv[1],cid) for cid in sys.argv[2:]}))')
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', script,
        str(root), *receipts], text=True, timeout=12))
    assert reopened == receipts
    from bioxp.operator_command_plane import OperatorCommandStore
    restarted = OperatorCommandStore(root)
    try:
        assert restarted.claim_next() is None
        if drift:
            assert restarted.deck_recovery_blocker() is not None
    finally:
        restarted.stop()
    faulthandler.cancel_dump_traceback_later()
