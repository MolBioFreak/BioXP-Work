"""Offline normal dispatch: host reference evidence is not an OEM interlock.

Real ASGI/queue/provider/production adapter/SQLite; native transport is replaced.
No hardware calls, re-homing, reference repair or fabricated owner epochs.
"""
import copy
import json
import os
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_complete_noop import setup_native, terminal
from tests.test_cover_carry_release_connected import connected
from bioxp.services.reference_service import MarkAxisDesyncedCommand


def desync(references):
    for axis in ('x', 'y', 'z', 'g'):
        assert references.mark_desynced(MarkAxisDesyncedCommand(
            axis, reason='isolated stale host record, not physical acceptance'))['ok']
    return reference_truth(references)


def reference_truth(references):
    # Last-motion telemetry may legitimately change; reference truth must not.
    return {axis: {key: value for key, value in row.items()
                   if key not in {'updated_at', 'last_motion_kind'}}
            for axis, row in references.snapshot(('x', 'y', 'z', 'g'))['rows'].items()}


@pytest.mark.parametrize('reference_mode', ['desynced', 'missing_store', 'missing_rows'])
@pytest.mark.parametrize('caller_epochs', [{}, {'4': 1, '5': 1}])
def test_queued_named_move_dispatches_without_reference_or_display_epochs(
        installed_retained, retained_rig, monkeypatch, reference_mode, caller_epochs):
    app, provider, observations, references, root = installed_retained
    leaf, waits, raw = setup_native(installed_retained, retained_rig, monkeypatch)
    before = desync(references)
    if reference_mode == 'missing_store':
        provider.reference_store = None
    elif reference_mode == 'missing_rows':
        # Only the observation is absent; the canonical records are left intact.
        from types import SimpleNamespace
        provider.reference_store = SimpleNamespace(snapshot=lambda axes: {'rows': {}})
    plane = app.state.operator_command_plane
    plane.start()
    client = TestClient(app)
    body = {'schema_version': 'bioxp.operator_action_request.v2',
            'idempotency_key': 'offline-non-oem-reference',
            'expected_ownership_generation': int(provider.generation_provider()),
            'expected_board_epoch_by_board': caller_epochs,
            'inputs': {'target': 'WASTE_BIN', 'camera_offset': False}}
    response = client.post('/operator/v2/actions/oem.deck.move_to_location', json=body)
    assert response.status_code == 200, response.text
    cid = response.json()['command_id']
    receipt = terminal(client, cid)
    assert receipt['status'] == 'completed', receipt
    assert plane.store.wait_for_command_workers([cid], timeout=2)
    assert leaf.moves and raw and waits
    assert receipt['deck_movement']['semantic_state_committed'] is True
    assert reference_truth(references) == before
    stamps = provider.deck_owner_authority_stamps()
    assert receipt['expected_board_epoch_by_board'] == {
        '4': stamps['board_epoch_4'], '5': stamps['board_epoch_5']}
    assert receipt['physical_effect_verified'] is False
    moves = copy.deepcopy(leaf.moves)
    replay = client.post('/operator/v2/actions/oem.deck.move_to_location', json=body)
    assert replay.status_code == 200 and replay.json()['command_id'] == cid
    assert leaf.moves == moves

    if output := os.environ.get('NON_OEM_NATIVE_EXPORT_DIR'):
        catalog = client.get('/operator/v2/control-catalog')
        assert catalog.status_code == 200, catalog.text
        path = Path(output)
        path.mkdir(parents=True, exist_ok=True)
        (path / f'{reference_mode}-{len(caller_epochs)}.json').write_text(json.dumps({
            'catalog': catalog.json(), 'receipt': receipt,
            'reference_mode': reference_mode, 'caller_epochs': caller_epochs,
        }))
    conflict = client.post('/operator/v2/actions/oem.deck.move_to_location',
                           json={**body, 'inputs': {'target': 'LOC_OC', 'camera_offset': False}})
    assert conflict.status_code == 409
    assert leaf.moves == moves


@pytest.mark.parametrize('refusal', ['no_24v', 'uninitialized', 'latch', 'stop'])
def test_stale_reference_does_not_hide_real_refusal(
        installed_retained, retained_rig, monkeypatch, refusal):
    app, provider, observations, references, root = installed_retained
    leaf, waits, raw = setup_native(installed_retained, retained_rig, monkeypatch)
    desync(references)
    if refusal in {'no_24v', 'uninitialized'}:
        from bioxp.usb_driver import BioXpTester
        # Actual OEM board primitive, with only its transport-state seams replaced.
        monkeypatch.setattr(leaf, 'BOARD_DECK', 5, raising=False)
        monkeypatch.setattr(leaf, 'oem_no24v_state', lambda: refusal == 'no_24v', raising=False)
        monkeypatch.setattr(leaf, '_oem_board_state', lambda: {4: False, 5: False}, raising=False)
        monkeypatch.setattr(leaf, 'motor_oem_move_absolute',
                            BioXpTester.motor_oem_move_absolute.__get__(leaf))
    elif refusal == 'latch':
        monkeypatch.setattr(observations, 'query_latch', lambda: {'ok': True, 'value': 0})
    plane = app.state.operator_command_plane
    if refusal == 'stop':
        original_read = observations._read_axis_position
        def read_then_stop(axis):
            result = original_read(axis)
            plane.store.arm_interrupt_fence('oem.abort_all')
            return result
        monkeypatch.setattr(observations, '_read_axis_position', read_then_stop)
    plane.start()
    client = TestClient(app)
    body = {'schema_version': 'bioxp.operator_action_request.v2',
            'idempotency_key': 'offline-retained-real-refusal',
            'expected_ownership_generation': int(provider.generation_provider()),
            'expected_board_epoch_by_board': {},
            'inputs': {'target': 'WASTE_BIN', 'camera_offset': False}}
    response = client.post('/operator/v2/actions/oem.deck.move_to_location', json=body)
    assert response.status_code == 200, response.text
    receipt = terminal(client, response.json()['command_id'])
    assert receipt['status'] != 'completed', receipt
    assert not leaf.moves


@pytest.mark.parametrize('axis', ['x', 'y', 'z', 'xy'])
def test_axis_paths_dispatch_with_desynced_preparation_records(connected, monkeypatch, axis):
    rig = connected
    provider = rig.provider
    before = desync(provider.reference_store)
    profile = rig.native._motion_oem_axis_profile
    monkeypatch.setattr(rig.native, '_motion_oem_axis_profile',
                        lambda axis, startup=False: profile(axis, startup=startup))
    state = provider._load_state()
    for name in ('x_lifecycle', 'z_lifecycle'):
        state[name].update(state='failed_latched', reference_state='desynced',
                           active_receipt=None, pending_ticket=None)
    provider._save_state(state)
    if axis == 'x':
        result = provider.execute_x_intent('move_absolute', {
            'position_steps': 40000, 'command_id': 'offline-x-reference'})
    elif axis == 'y':
        result = provider.primitives.y_provider.move_absolute(40000)
    elif axis == 'z':
        result = provider.execute_z_intent('move_absolute', inputs={'position_steps': 70000},
            expected_generation=3, idempotency_key='offline-z-reference')
    else:
        provider.y_provider = provider.primitives.y_provider
        result = provider.execute_xy_intent(40000, 40000, {'command_id': 'offline-xy-reference'})
    assert result['ok'] is True, result
    assert rig.native.moves
    assert reference_truth(provider.reference_store) == before


@pytest.mark.parametrize('axis', ['x', 'y', 'z', 'xy'])
def test_axis_stop_still_excludes_dispatch_with_stale_reference(connected, axis):
    rig = connected
    provider = rig.provider
    desync(provider.reference_store)
    provider._x_interrupt_active = True
    provider._z_interrupt_active = True
    if axis == 'y':
        y = provider.primitives.y_provider
        y.state_store.begin_axis_interrupt('y')
        result = y.move_absolute(40000)
    elif axis == 'x':
        result = provider.execute_x_intent('move_absolute', {'position_steps': 40000})
    elif axis == 'z':
        result = provider.execute_z_intent('move_absolute', inputs={'position_steps': 70000},
            expected_generation=3, idempotency_key='offline-stopped-z')
    else:
        result = provider.execute_xy_intent(40000, 40000)
    assert result['ok'] is False
    assert not rig.native.moves


@pytest.mark.parametrize('caller_epochs', [{}, {'4': 1, '5': 1}])
def test_strict_xy_method_binds_current_owner_not_caller_epochs(retained_rig, caller_epochs):
    from fastapi import HTTPException
    provider, _, _, _, store, _ = retained_rig
    stamps = provider.deck_owner_authority_stamps()
    state = {'ownership_generation': 3, 'serial206_initialization_provider': {
        'x_authority': {'current_board_lifecycle_generation': stamps['board_epoch_5']},
        'board4_authority': {'active_board_epoch': stamps['board_epoch_4']}}}
    request = {'schema_version': 'bioxp.operator_method_request.v1',
        'name': 'oem.xy.move_absolute', 'idempotency_key': 'offline-strict-xy',
        'expected_ownership_generation': 3, 'expected_board_epoch_by_board': caller_epochs,
        'steps': [{'action_id': 'oem.xy.move_absolute', 'inputs': {'x': 40000, 'y': 40000}}],
        'metadata': {'method_action_id': 'oem.xy.move_absolute'}}
    result = store.admit_method(request, state=state, strict_authority=True)
    child = store.claim_next()
    assert child['method_id'] == result['method_id']
    assert child['expected_board_epoch_by_board'] == {
        '4': stamps['board_epoch_4'], '5': stamps['board_epoch_5']}
    replay = store.admit_method(request, state=state, strict_authority=True)
    assert replay['method_id'] == result['method_id'] and replay['idempotent_replay']
    with pytest.raises(HTTPException):
        store.admit_method({**request, 'steps': [{'action_id': 'oem.xy.move_absolute',
            'inputs': {'x': 41000, 'y': 40000}}]}, state=state, strict_authority=True)
