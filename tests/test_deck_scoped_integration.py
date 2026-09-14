"""Actual installed control-plane paths with copied retained SQLite authority."""
import json
import os
import time
from pathlib import Path
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig, qualify_test_references


def qualify_full_predecessor(retained_rig):
    from bioxp.oem_deck_movement import OEM_MOVABLE_OBJECT_DEFAULT_LOCATIONS
    provider, primitive, runtime, references, store, root = retained_rig
    qualify_test_references(references)
    state = provider._load_state()
    state['machine_status'].update(current_location='LOC_PARK', current_well=0,
        tip_loaded=False, tip_dirty=False, tip_location=-1, clean_path=False,
        plate_on_gantry=None, movable_plate_locations=dict(OEM_MOVABLE_OBJECT_DEFAULT_LOCATIONS))
    state['machine_status'].pop('latch_observation_id', None)
    state['machine_status'].pop('latch_closed', None)
    provider._save_state(state)
    provider.bind_tip_tray_state_reader(store.tip_tray_state)
    store.publish_tip_tray_transition(tray_id=0, transition='construct', operation_id='fixture-only-construction',
        command_id='fixture-only-construction', provenance={'source': 'explicit test predecessor'},
        **provider.deck_owner_authority_stamps())
    snapshot = provider.deck_authority_snapshot(expected_generation=3, target='LOC_PARK')
    assert snapshot['machine_state_revision'] == 1
    assert snapshot['current_location_id'] == 'LOC_PARK' and snapshot['latch_status'] is True
    assert sum(c[0] == 'latch' for c in primitive.calls) == 1
    assert provider._load_state()['machine_status'].get('construction_id') is None


def test_latch_only_full_predecessor_bootstrap(retained_rig):
    qualify_full_predecessor(retained_rig)


@pytest.fixture
def installed_retained(retained_rig, monkeypatch):
    from bioxp import operator_controls as controls, api, runtime_audit_store
    provider, primitive, runtime, references, prior_store, root = retained_rig
    prior_store.stop()
    monkeypatch.setattr(runtime_audit_store, 'CANONICAL_RUNTIME_ROOT', root)
    import src.bioxp.runtime_audit_store as other_runtime
    monkeypatch.setattr(other_runtime, 'CANONICAL_RUNTIME_ROOT', root)
    monkeypatch.setenv('BIOXP_OEM_RUNTIME_STATE_ROOT', str(root))
    monkeypatch.setattr(controls, 'current_release_identity', lambda: {'verified': True,
        'release_id': 'isolated-deck-fixture', 'source': {'manifest_sha256': '1'*64, 'aggregate_sha256': '2'*64}})
    monkeypatch.setattr(controls, 'current_authority_identity', lambda: {
        'evidence_lock_identity_verified': True, 'evidence_lock_sha256': '3'*64})
    monkeypatch.setattr(controls, 'current_registry_sha256', lambda: '4'*64)
    monkeypatch.setattr(type(controls.hardware_state), 'ownership_epoch', property(lambda self: 3))
    monkeypatch.setattr(controls.hardware_state, 'project', lambda *a, **k: {
        'domains': {}, 'freshness': {'state': 'stale', 'age_s': 90, 'fresh_for_s': 30}})
    monkeypatch.setattr(controls.hardware_state, 'ownership_projection', lambda: {'ownership_epoch': 3,
        'ownership': {'transport': 'owned', 'usb': 'service', 'router': 'running', 'CAN_READY': True}})
    def projection():
        stamps = provider.deck_owner_authority_stamps()
        return {'x_authority': {'active_board_epoch': stamps['board_epoch_5'],
                'current_board_lifecycle_generation': stamps['board_epoch_5']},
                'board4_authority': {'active_board_epoch': stamps['board_epoch_4']}}
    from bioxp.oem_compat.position_table import load_bound_oem_position_table
    app = FastAPI()
    app.add_api_route('/hardware/snapshot/collect', api.hardware_snapshot_collect, methods=['POST'])
    controls.install_operator_control_plane(app,
        maintenance_state_provider=lambda: {'motion_blocked': False, 'recovery_required': False, 'block_reason': None},
        reference_state_provider=lambda: references.snapshot(('x','y','z','g')),
        lifecycle_state_provider=lambda: {'operation_state': 'stopped'},
        serial206_initialization_state_provider=projection,
        oem_deck_provider=lambda: provider, oem_deck_position_table_provider=load_bound_oem_position_table)
    monkeypatch.setattr(api, 'app', app)
    monkeypatch.setattr(api, '_get_tester', lambda: object())
    monkeypatch.setattr(api, '_hardware_collectors', lambda tester: {})
    monkeypatch.setattr(api.hardware_state, 'collect', lambda *a, **k: {'ok': True, 'snapshot': {}})
    yield app, provider, primitive, references, root
    app.state.operator_command_plane.stop()


def catalog_payload(app):
    client = TestClient(app)
    client.get('/operator/v2/control-catalog')
    pending = app.state.operator_poll_cache._pending
    if pending is not None:
        pending.result(timeout=3)
    response = client.get('/operator/v2/control-catalog')
    assert response.status_code == 200, response.text
    return response.json()


def catalog_action(app):
    return next(row for row in catalog_payload(app)['actions'] if row['action_id'] == 'oem.deck.move_to_location')


def test_actual_collection_cached_catalog_and_durable_dispatch(installed_retained):
    from bioxp import api
    app, provider, primitive, references, root = installed_retained
    assert catalog_action(app)['enabled'] is False
    qualify_test_references(references)
    collected = api._collect_and_publish_hardware_snapshot(['axes','latch'], reason='isolated-explicit-refresh')
    assert collected['deck_authority']['enabled'] is True, collected
    before = list(primitive.calls)
    scoped_catalog = catalog_payload(app)
    catalog = next(r for r in scoped_catalog['actions'] if r['action_id'] == 'oem.deck.move_to_location')
    assert len(catalog['destination_options']) == 26
    assert primitive.calls == before
    assert catalog['enabled'] is True
    park = next(r for r in catalog['destination_options'] if r['target'] == 'LOC_PARK')
    assert park['enabled'] is False and 'deck_bootstrap_semantic_location_unavailable' in park['disabled_reason']
    assert all(r['enabled'] for r in catalog['destination_options'] if r['target'] != 'LOC_PARK')
    client = TestClient(app)
    body = {'schema_version': 'bioxp.operator_action_request.v2', 'idempotency_key': 'installed-first',
        'expected_ownership_generation': 3, 'expected_board_epoch_by_board': catalog['expected_board_epoch_by_board'],
        'inputs': {'target': 'LOC_OC', 'camera_offset': False}}
    rejected = client.post('/operator/v2/actions/oem.deck.move_to_location', json={**body, 'inputs': {'target': 'LOC_PARK', 'camera_offset': False}})
    assert rejected.status_code == 409, rejected.text
    admitted = client.post('/operator/v2/actions/oem.deck.move_to_location', json=body)
    assert admitted.status_code == 200, admitted.text
    command_id = admitted.json()['command_id']
    app.state.operator_command_plane.start()
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        compact = client.get('/operator/v2/actions/receipts/' + command_id).json()
        if compact['status'] not in {'queued','dispatched','issued_pending'}:
            break
        time.sleep(.01)
    assert compact['status'] == 'completed', compact
    detail = client.get('/operator/v2/actions/receipts/' + command_id + '?detail=true').json()
    assert detail['deck_movement']['semantic_state_committed'] is True
    assert app.state.operator_command_plane.store.deck_semantic_state()['current_location'] == 'LOC_OC'
    if os.environ.get('DECK_TEST_OUTPUT'):
        Path(os.environ['DECK_TEST_OUTPUT'] + '.catalog.json').write_text(json.dumps(
            {'catalog': scoped_catalog, 'catalog_action': catalog, 'collected': collected, 'compact': compact, 'detail': detail}, indent=2))


def fresh_process_receipts(root, command_id):
    """Install real receipt routes in a new interpreter; no provider or worker."""
    from bioxp import operator_controls as controls, runtime_audit_store
    import src.bioxp.runtime_audit_store as other_runtime
    from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot
    with pytest.MonkeyPatch.context() as mp:
        bind_serial206_oem_snapshot(mp)
        mp.setattr(runtime_audit_store, 'CANONICAL_RUNTIME_ROOT', Path(root))
        mp.setattr(other_runtime, 'CANONICAL_RUNTIME_ROOT', Path(root))
        mp.setenv('BIOXP_OEM_RUNTIME_STATE_ROOT', str(root))
        app = FastAPI()
        controls.install_operator_control_plane(app)
        client = TestClient(app)
        compact = client.get('/operator/v2/actions/receipts/' + command_id)
        detail = client.get('/operator/v2/actions/receipts/' + command_id + '?detail=true')
        assert compact.status_code == detail.status_code == 200, (compact.text, detail.text)
        app.state.operator_command_plane.stop()
        return {'compact': compact.json(), 'detail': detail.json()}


def test_actual_park_noop_fresh_process_export(installed_retained, retained_rig):
    import subprocess, sys
    from bioxp import api
    app, provider, primitive, references, root = installed_retained
    qualify_test_references(references)
    collected = api._collect_and_publish_hardware_snapshot(['axes','latch'], reason='isolated-scoped-refresh')
    assert collected['deck_authority']['enabled'] is True
    scoped_catalog = catalog_payload(app)
    action = next(r for r in scoped_catalog['actions'] if r['action_id'] == 'oem.deck.move_to_location')
    assert next(r for r in action['destination_options'] if r['target'] == 'LOC_OC')['enabled']
    assert not next(r for r in action['destination_options'] if r['target'] == 'LOC_PARK')['enabled']
    # Park qualification has an explicit complete test predecessor, not recovery
    # of the retained null case and not a new production construction event.
    _, _, runtime, _, _, _ = retained_rig
    store = app.state.operator_command_plane.store
    primitive.calls.clear()
    qualify_full_predecessor((provider, primitive, runtime, references, store, root))
    client = TestClient(app)
    admitted = client.post('/operator/v2/actions/oem.deck.move_to_location', json={
        'schema_version': 'bioxp.operator_action_request.v2', 'idempotency_key': 'actual-park-noop',
        'expected_ownership_generation': 3,
        'expected_board_epoch_by_board': action['expected_board_epoch_by_board'],
        'inputs': {'target': 'LOC_PARK', 'camera_offset': False}})
    assert admitted.status_code == 200, admitted.text
    command_id = admitted.json()['command_id']
    app.state.operator_command_plane.start()
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        compact = client.get('/operator/v2/actions/receipts/' + command_id).json()
        if compact['status'] not in {'queued', 'dispatched', 'issued_pending'}:
            break
        time.sleep(.01)
    assert compact['status'] == 'completed', compact
    detail = client.get('/operator/v2/actions/receipts/' + command_id + '?detail=true').json()
    assert detail['deck_movement']['semantic_state_committed'] is True
    assert not any(c[0] == 'move' for c in primitive.calls)
    app.state.operator_command_plane.stop()
    script = ('import json,sys; from tests.test_deck_scoped_integration import fresh_process_receipts; '
              'print(json.dumps(fresh_process_receipts(sys.argv[1], sys.argv[2])))')
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', script, str(root), command_id], text=True))
    assert reopened['compact'] == compact
    assert reopened['detail'] == detail
    assert reopened['compact']['physical_effect_verified'] is False
    assert reopened['compact']['completion_class'] == 'source_noop'
    for key in ('delivery_attempted','controller_command_acknowledged','controller_completion_verified',
                'hardware_postcondition_verified','physical_observation_verified'):
        assert reopened['detail']['deck_movement'][key] is False, (key, reopened['detail'])
    if os.environ.get('DECK_TEST_OUTPUT'):
        Path(os.environ['DECK_TEST_OUTPUT'] + '.bms.json').write_text(json.dumps(
            {'catalog': scoped_catalog, **reopened}, indent=2))
