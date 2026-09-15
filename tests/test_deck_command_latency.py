"""Named-start amplification: actual route/worker/SQLite, offline native leaves."""
import json
import os
from pathlib import Path
import subprocess
import sys
import threading

import pytest
from fastapi.testclient import TestClient
from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from bioxp.services.reference_service import MarkAxisReferencedCommand
from tests.test_deck_scoped_authority import retained_rig, qualify_test_references
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_automatic_refresh_owner import request, finish
from tests.test_deck_postmove_reference import USBLeaf


@pytest.mark.parametrize('fault', [None, 'latch', 'position', 'reference', 'generation', 'tip', 'interrupt', 'publication', 'board', 'owner'])
def test_named_start_two_samples_and_final_fences(installed_retained, monkeypatch, fault):
    from bioxp import api
    app, provider, observations, references, root = installed_retained
    api.serial206_oem_initialization_provider_status()
    qualify_test_references(references)
    store = app.state.operator_command_plane.store
    leaf = USBLeaf()
    axis_address = {'x': (5, 0), 'y': (4, 0), 'z': (4, 1)}
    monkeypatch.setattr(observations, '_read_axis_position', lambda axis: leaf.positions[axis_address[axis]])
    adapter = Serial206ProductionPrimitiveAdapter(leaf, None,
        authority_provider=lambda: {}, generation_provider=provider.generation_provider,
        reference_store=references)
    events = []
    executor = app.state.oem_deck_command_executor
    def execute(**kwargs):
        events.append(('executor_enter',))
        return executor(**kwargs)
    monkeypatch.setattr(app.state, 'oem_deck_command_executor', execute)
    original_sample = provider.deck_authority_snapshot
    def sample(**kwargs):
        value = original_sample(**kwargs)
        if threading.current_thread().name.startswith('bioxp-operator-command-'):
            events.append(('sample', value['captured_at'], value['machine_state_revision']))
        return value
    monkeypatch.setattr(provider, 'deck_authority_snapshot', sample)
    def move(*args, **kwargs):
        events.append(('native_move',))
        return adapter.oem_move_to(*args, **kwargs)
    monkeypatch.setattr(observations, 'oem_move_to', move)
    original_terminalize = store.terminalize_deck_stage
    def terminalize(command_id, step, **kwargs):
        result = original_terminalize(command_id, step, **kwargs)
        events.append(('stage', step.operation))
        if step.operation == 'check_machine_latch_closed':
            if fault == 'latch':
                monkeypatch.setattr(observations, 'query_latch', lambda: {'ok': True, 'value': 0})
            elif fault == 'position':
                monkeypatch.setattr(observations, '_read_axis_position', lambda axis: 1 if axis == 'x' else 0)
            elif fault == 'reference':
                references.mark_referenced(MarkAxisReferencedCommand('x', 0, source='independent offline replacement'))
            elif fault == 'generation':
                generation = provider.generation_provider()
                monkeypatch.setattr(provider, 'generation_provider', lambda: generation + 1)
            elif fault == 'tip':
                state = provider._load_state()
                state['machine_status']['tip_loaded'] = True
                provider._save_state(state)
            elif fault == 'interrupt':
                provider._x_interrupt_epoch += 1
            elif fault == 'owner':
                provider._deck_owner_id = 'independent-offline-provider'
            elif fault == 'board':
                state = provider._load_state()
                state['x_lifecycle']['board_lifecycle_generation'] += 1
                provider._save_state(state)
        return result
    monkeypatch.setattr(store, 'terminalize_deck_stage', terminalize)
    if fault == 'publication':
        original_persist = store.persist_deck_pseudo_home
        def foreign_publication(command_id, value, **kwargs):
            original_persist(command_id, value, **kwargs)
            return original_persist(command_id, value, **kwargs)
        monkeypatch.setattr(store, 'persist_deck_pseudo_home', foreign_publication)
    client = TestClient(app)
    response = client.post('/operator/v2/actions/oem.deck.move_to_location', json=request(provider, 'latency-' + str(fault)))
    assert response.status_code == 200, response.text
    command_id = response.json()['command_id']
    app.state.operator_command_plane.start()
    receipt = finish(client, command_id)
    assert store.wait_for_command_workers([command_id], timeout=2)
    if os.environ.get('DECK_TEST_OUTPUT'):
        Path(os.environ['DECK_TEST_OUTPUT'] + '.latency-' + str(fault) + '.json').write_text(
            json.dumps({'receipt': receipt, 'events': events, 'moves': leaf.moves}, indent=2))
    executor_events = events[events.index(('executor_enter',)) + 1:]
    if fault:
        assert receipt['status'] != 'completed', receipt
        assert not leaf.moves
        assert not any(e[0] == 'native_move' for e in events)
        assert store.deck_semantic_state()['current_location'] is None
        if fault == 'publication':
            assert sum(e[0] == 'sample' for e in executor_events) == 1
        return
    events = executor_events
    assert receipt['status'] == 'completed', receipt
    assert receipt['deck_movement']['semantic_state_committed'] is True
    assert receipt['deck_movement']['controller_completion_verified'] is True
    assert receipt['physical_effect_verified'] is False
    assert len(leaf.moves) == 2
    samples = [i for i, e in enumerate(events) if e[0] == 'sample']
    assert len(samples) == 2, events
    last_predicate = next(i for i,e in enumerate(events) if e == ('stage', 'check_machine_latch_closed'))
    first_move = next(i for i,e in enumerate(events) if e[0] == 'native_move')
    assert samples[0] < last_predicate < samples[-1] < first_move
    assert events[samples[-1]][2] == events[samples[0]][2] + 1
    # Warm next command starts with pseudo500 and a populated semantic owner.
    event_count = len(executor_events)
    revision = store.deck_semantic_state()['semantic_state_revision']
    body = request(provider, 'latency-repeat')
    body['inputs']['target'] = 'LOC_OC'
    response2 = client.post('/operator/v2/actions/oem.deck.move_to_location', json=body)
    assert response2.status_code == 200, response2.text
    receipt2 = finish(client, response2.json()['command_id'])
    assert receipt2['status'] == 'completed', receipt2
    assert store.wait_for_command_workers([response2.json()['command_id']], timeout=2)
    assert store.deck_semantic_state()['semantic_state_revision'] == revision + 2
    repeated = events[event_count:]
    repeated = repeated[repeated.index(('executor_enter',)) + 1:]
    assert sum(e[0] == 'sample' for e in repeated) == 2, repeated
    assert len(leaf.moves) == 4
    app.state.operator_command_plane.stop()
    script = ('import json,sys; from tests.test_deck_scoped_integration import fresh_process_receipts; '
              'print(json.dumps(fresh_process_receipts(sys.argv[1], sys.argv[2])))')
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', script, str(root), command_id], text=True))
    assert reopened['detail'] == receipt
    assert reopened['compact']['status'] == 'completed'
