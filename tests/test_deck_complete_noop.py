"""Actual queued source no-op, not synchronous executor-forced completion."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig, qualify_test_references
from tests.test_deck_scoped_integration import installed_retained, catalog_action
from tests.test_deck_near_terminal import NearUSB


def setup_native(installed, retained, monkeypatch, before=(0, 0), fault=None):
    from bioxp import api
    from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
    from bioxp.serial206_y_provider import Serial206YProvider
    app, provider, observations, references, root = installed
    api.serial206_oem_initialization_provider_status()
    qualify_test_references(references)
    generation = int(provider.generation_provider())
    leaf = NearUSB(before, fault)
    original_wait = leaf.motor_wait_target_reached
    waits = []
    def wait(*args, **kwargs):
        waits.append(args)
        result = original_wait(*args, **kwargs)
        if result.get('event') is not None:
            result['event']['owner_generation'] = generation
        return result
    monkeypatch.setattr(leaf, 'motor_wait_target_reached', wait)
    monkeypatch.setattr(leaf, 'begin_bus_event_window', lambda: {
        'after_sequence': 0, 'receive_owner': 'offline-usb', 'owner_generation': generation})
    adapter = Serial206ProductionPrimitiveAdapter(leaf, None, authority_provider=lambda: {},
        generation_provider=lambda: generation, reference_store=references)
    adapter.y_provider = Serial206YProvider(leaf, state_store=retained[2],
        generation_provider=lambda: generation, reference_store=references)
    raw = []
    def move(*args, **kwargs):
        result = adapter.oem_move_to(*args, **kwargs)
        raw.append(result)
        return result
    monkeypatch.setattr(observations, 'oem_move_to', move)
    assert api._collect_and_publish_hardware_snapshot(['axes', 'latch'],
        reason='offline-noop-qualification')['deck_authority']['enabled']
    assert catalog_action(app)['enabled']
    return leaf, waits, raw


def submit(app, client, target, provider):
    action = catalog_action(app)
    body = {'schema_version': 'bioxp.operator_action_request.v2',
        'idempotency_key': 'queued-noop-' + target,
        'expected_ownership_generation': int(provider.generation_provider()),
        'expected_board_epoch_by_board': action['expected_board_epoch_by_board'],
        'inputs': {'target': target, 'camera_offset': False}}
    response = client.post('/operator/v2/actions/oem.deck.move_to_location', json=body)
    assert response.status_code == 200, response.text
    return response.json()['command_id'], body


def terminal(client, cid):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        response = client.get('/operator/v2/actions/receipts/' + cid)
        assert response.status_code == 200, response.text
        if response.json()['terminal']:
            return client.get('/operator/v2/actions/receipts/' + cid + '?detail=true').json()
        time.sleep(.01)
    pytest.fail('queued command did not reach durable terminal')


def test_actual_pool_bsc_noop_then_distinct_move(installed_retained, retained_rig, monkeypatch):
    from bioxp import api
    app, provider, observations, references, root = installed_retained
    leaf, waits, raw = setup_native(installed_retained, retained_rig, monkeypatch)
    plane = app.state.operator_command_plane
    client = TestClient(app)
    plane.start()
    receipts = {}
    for target in ('LOC_P_TC', 'LOC_BSC', 'LOC_RC'):
        prior_moves, prior_waits = len(leaf.moves), len(waits)
        cid, body = submit(app, client, target, provider)
        detail = terminal(client, cid)
        # Save actual pre-fix receipt too; never replace evidence with expected data.
        output = os.environ.get('DECK_TEST_OUTPUT')
        if output:
            Path(output + '.' + target + '.json').write_text(json.dumps(detail, indent=2))
        assert detail['status'] == 'completed', json.dumps(detail, indent=2)
        assert plane.store.wait_for_command_workers([cid], timeout=2)
        deck = detail['deck_movement']
        assert deck['semantic_state_committed'] is True
        assert all(s['terminal_state'] == 'completed' for s in deck['stages'])
        assert detail['physical_effect_verified'] is False
        if target == 'LOC_BSC':
            assert len(leaf.moves) == prior_moves and len(waits) == prior_waits
            assert leaf.positions[(5, 0)] == 42518 and leaf.positions[(4, 0)] == 8779
            assert detail['completion_class'] == 'source_noop'
            assert detail['source_receipt']['source_noop'] is True
            assert detail['source_receipt']['source_noop_reason'] == 'already_at_target'
            assert deck['delivery_attempted'] is False
            assert deck['controller_command_acknowledged'] is False
            assert deck['controller_completion_verified'] is False
            assert detail['source_receipt']['controller_acknowledged'] is False
            op = raw[-1]['operations'][0]
            assert op['branch'] == 'source_noop' and op['source_noop_verified'] is True
            assert op['command_issued'] is False and op['target_event_128_observed'] is False
            assert op['physical_motion_commanded'] is False
            assert op['before'] == op['requested'] == op['after']
        else:
            assert len(leaf.moves) > prior_moves and len(waits) > prior_waits
            assert detail['completion_class'] == 'deck_terminal'
            assert deck['controller_completion_verified'] is True
        compact = client.get('/operator/v2/actions/receipts/' + cid).json()
        receipts[cid] = {'compact': compact, 'detail': detail}
        before = list(leaf.moves)
        replay = client.post('/operator/v2/actions/oem.deck.move_to_location', json=body)
        assert replay.status_code == 200 and replay.json()['command_id'] == cid
        assert leaf.moves == before
        assert api._collect_and_publish_hardware_snapshot(['axes', 'latch'],
            reason='offline-next-noop-refresh')['deck_authority']['enabled']
        for _ in range(3):
            for path in ('/operator/dashboard', '/operator/control-catalog',
                         '/operator/v2/dashboard', '/operator/v2/control-catalog'):
                result = client.get(path)
                assert result.status_code == 200, (path, result.text)
                pending = app.state.operator_poll_cache._pending
                if pending is not None:
                    pending.result(timeout=3)
            assert client.get('/operator/v2/actions/receipts/' + cid + '?detail=true').json() == detail
            legacy = client.get('/operator/commands/' + cid + '?detail=true')
            assert legacy.status_code == 200, legacy.text
            assert legacy.json()['status'] == 'completed'
        assert catalog_action(app)['enabled']
        assert client.get('/operator/v2/dashboard').json()['deck']['current_location'] == target
    assert len(receipts) == 3 and len(raw) == 3
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
        assert restarted.deck_recovery_blocker() is None
    finally:
        restarted.stop()


@pytest.mark.parametrize('fault', ['ack_only', 'missing_event', 'missing_ack', 'invalid_position', 'source_exception', 'partial_motor'])
def test_queued_missing_proof_still_fails(installed_retained, retained_rig, monkeypatch, fault):
    app, provider, observations, references, root = installed_retained
    leaf, waits, raw = setup_native(installed_retained, retained_rig, monkeypatch,
        before=(42518, 8779) if fault == 'invalid_position' else (0, 0), fault=fault)
    if fault == 'partial_motor':
        original_wait = leaf.motor_wait_target_reached
        def partial_wait(board, *args, **kwargs):
            result = original_wait(board, *args, **kwargs)
            if board == 5:
                result.update(ok=False, target_reached=False, event=None)
            return result
        monkeypatch.setattr(leaf, 'motor_wait_target_reached', partial_wait)
    if fault == 'source_exception':
        def fail(*args, **kwargs):
            raise RuntimeError('offline source primitive failure')
        monkeypatch.setattr(leaf, 'motor_oem_move_absolute', fail)
    app.state.operator_command_plane.start()
    client = TestClient(app)
    cid, _ = submit(app, client, 'LOC_BSC', provider)
    detail = terminal(client, cid)
    assert detail['status'] in {'failed', 'ambiguous'}, detail
    assert detail['completion_class'] != 'source_noop'
    assert detail['deck_movement']['semantic_state_committed'] is False
    assert detail['source_receipt']['source_noop'] is False
