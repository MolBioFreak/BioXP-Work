"""Actual installed control-plane paths with copied retained SQLite authority."""
import json
import os
import time
from pathlib import Path
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from tests.deck_scoped_rig import retained_rig, qualify_test_references


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
    snapshot = provider.deck_authority_snapshot(expected_generation=int(provider.generation_provider()), target='LOC_PARK')
    assert snapshot['machine_state_revision'] == 1
    assert snapshot['current_location_id'] == 'LOC_PARK' and snapshot['latch_status'] is True
    assert sum(c[0] == 'latch' for c in primitive.calls) == 1
    assert provider._load_state()['machine_status'].get('construction_id') is None




@pytest.fixture
def installed_retained(retained_rig, monkeypatch):
    from bioxp import operator_controls as controls, api, runtime_audit_store
    provider, primitive, runtime, references, prior_store, root = retained_rig
    prior_store.stop()
    # The copied controller lifecycle belongs to its recorded generation.
    # An unrelated synthetic generation would correctly invalidate it in the
    # real API projection, before the cache contract under test is reached.
    generation = provider._load_state()['x_lifecycle']['generation']
    assert type(generation) is int
    monkeypatch.setattr(provider, 'generation_provider', lambda: generation)
    from bioxp.serial206_y_provider import Serial206YProvider
    y_provider = Serial206YProvider(primitive, state_store=runtime,
        generation_provider=lambda: generation, reference_store=references)
    # Y's configuration display is outside this fixture's hardware scope;
    # board/axis authority and the API projection remain their real owners.
    monkeypatch.setattr(y_provider, 'profile', lambda: {})
    provider.y_provider = y_provider
    monkeypatch.setattr(runtime_audit_store, 'CANONICAL_RUNTIME_ROOT', root)
    import src.bioxp.runtime_audit_store as other_runtime
    monkeypatch.setattr(other_runtime, 'CANONICAL_RUNTIME_ROOT', root)
    monkeypatch.setenv('BIOXP_OEM_RUNTIME_STATE_ROOT', str(root))
    monkeypatch.setattr(controls, 'current_release_identity', lambda: {'verified': True,
        'release_id': 'isolated-deck-fixture', 'source': {'manifest_sha256': '1'*64, 'aggregate_sha256': '2'*64}})
    monkeypatch.setattr(controls, 'current_authority_identity', lambda: {
        'evidence_lock_identity_verified': True, 'evidence_lock_sha256': '3'*64})
    monkeypatch.setattr(controls, 'current_registry_sha256', lambda: '4'*64)
    monkeypatch.setattr(type(controls.hardware_state), 'ownership_epoch', property(lambda self: generation))
    monkeypatch.setattr(controls.hardware_state, 'project', lambda *a, **k: {
        'domains': {}, 'freshness': {'state': 'stale', 'age_s': 90, 'fresh_for_s': 30}})
    monkeypatch.setattr(controls.hardware_state, 'ownership_projection', lambda: {'ownership_epoch': generation,
        'ownership': {'transport': 'owned', 'usb': 'service', 'router': 'running', 'CAN_READY': True}})
    # Exercise the real API owner projection. It does not publish the invented
    # board4_authority/active_board_epoch fields used by the former fixture.
    monkeypatch.setattr(api, '_serial206_oem_initialization_provider', provider)
    projection = api.serial206_oem_initialization_provider_status
    from bioxp.oem_compat.position_table import load_bound_oem_position_table
    app = FastAPI()
    app.add_api_route('/hardware/snapshot/collect', api.hardware_snapshot_collect, methods=['POST'])
    monkeypatch.setattr(api, 'app', app)
    # Mirror the real lifespan producer order; do not install a missing camera
    # dependency and then replace the provider after the plane captures it.
    app.state.oem_preparation_camera = api._protocol_preparation_camera()
    controls.install_operator_control_plane(app,
        maintenance_state_provider=lambda: {'motion_blocked': False, 'recovery_required': False, 'block_reason': None},
        reference_state_provider=lambda: references.snapshot(('x','y','z','g')),
        lifecycle_state_provider=lambda: {'operation_state': 'stopped'},
        serial206_initialization_state_provider=projection,
        oem_deck_provider=lambda: provider, oem_deck_position_table_provider=load_bound_oem_position_table)
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


