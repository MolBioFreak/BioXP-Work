"""Normal readiness route -> real query/audit/SQLite with CAN leaf doubles."""
import json
import subprocess
import sys
import pytest
from fastapi.testclient import TestClient
from tests.test_deck_tip_query_publication import query_rig, named_move
from tests.test_deck_scoped_integration import installed_retained, catalog_action
from tests.test_deck_scoped_authority import retained_rig, qualify_test_references
from tests.test_deck_tip_query_publication_contradiction import assert_park_unready, park_authority, assert_worker_refused


def collect(rig, *, automatic=False):
    from bioxp import api
    app = rig[0]
    if not any(getattr(route, 'path', None) == '/hardware/snapshot/collect' for route in app.routes):
        app.add_api_route('/hardware/snapshot/collect', api.hardware_snapshot_collect, methods=['POST'])
    response = TestClient(app).post('/hardware/snapshot/collect', json={'automatic': automatic})
    assert response.status_code == 200, response.text
    return response.json()


def test_cold_collection_commits_query_without_inventing_location(query_rig):
    app, provider, primitive, refs, root, receipts, calls, wire, transport = query_rig
    qualify_test_references(refs)
    before = app.state.operator_command_plane.store.deck_semantic_state()
    result = collect(query_rig)
    assert calls == [0, 1, 2, 3]
    observation = result['park_tip_observation']
    assert observation['deck_state_publication']['status'] == 'published'
    after = app.state.operator_command_plane.store.deck_semantic_state()
    assert after['tip_loaded'] is False
    assert after['current_location'] == before['current_location']
    assert result['pipette_collection']['tip_exists'] is False
    park = next(r for r in catalog_action(app)['destination_options'] if r['target'] == 'LOC_PARK')
    assert park['enabled']  # Intent is selectable; a query cannot invent a predecessor.
    assert_park_unready(query_rig, 'deck_semantic_state_not_authoritative:location_revision')
    collect(query_rig)
    assert calls == [0, 1, 2, 3]  # No eager repeated query while owners remain valid.
    app.state.operator_command_plane.start()
    assert_worker_refused(query_rig, 'cold-park-active-worker',
                          'deck_semantic_state_not_authoritative:location_revision')
    assert calls == [0, 1, 2, 3]
    assert app.state.operator_command_plane.store.deck_semantic_state() == after


def test_after_real_worker_move_collection_restores_park_and_persists(query_rig):
    app, provider, primitive, refs, root, receipts, calls, wire, transport = query_rig
    named_move(query_rig)
    result = collect(query_rig)
    assert calls == [0, 1, 2, 3]
    park = next(r for r in catalog_action(app)['destination_options'] if r['target'] == 'LOC_PARK')
    assert park['enabled'], park
    assert park_authority(query_rig)['current_location_id'] == 'LOC_OC'
    observation = result['park_tip_observation']
    script = ('import json,sys; from tests.test_deck_tip_query_publication import reopen; '
              'print(json.dumps(reopen(sys.argv[1],sys.argv[2],int(sys.argv[3]))))')
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', script, str(root),
        observation['receipt_id'], str(provider.generation_provider())], text=True))
    assert reopened['semantic']['tip_loaded'] is False
    assert reopened['park_blocker']['current_location_id'] == 'LOC_OC'
    assert reopened['receipt']['result']['hardware_query_verified'] is True


@pytest.mark.parametrize('fault', ['missing', 'uncorrelated', 'loaded', 'reader_drift'])
def test_collection_never_manufactures_absence(query_rig, fault):
    app, provider, primitive, refs, root, receipts, calls, wire, transport = query_rig
    qualify_test_references(refs)
    if fault == 'missing': wire['missing'] = True
    if fault == 'uncorrelated': wire['correlated'] = False
    if fault == 'loaded': wire['data'] = [[32, 96, 49] for _ in range(4)]
    if fault == 'reader_drift': wire['reader_drift'] = True
    result = collect(query_rig)
    assert calls == [0, 1, 2, 3]
    after = app.state.operator_command_plane.store.deck_semantic_state()
    assert after['tip_loaded'] is not False
    if fault == 'loaded':
        assert after['tip_loaded'] is True
        assert after['tip_dirty'] is None and after['tip_location'] is None
    else:
        assert result['park_tip_observation'].get('available') is False or result['park_tip_observation'].get('deck_state_publication', {}).get('status') == 'blocked'


def test_parent_refresh_keeps_independent_child_receipt(query_rig, monkeypatch):
    from fastapi import FastAPI
    from bioxp import api, operator_controls
    from bioxp.oem_compat.position_table import load_bound_oem_position_table
    app, provider, primitive, refs, root, receipts, calls, wire, transport = query_rig
    app.state.operator_command_plane.stop()
    app = FastAPI()
    app.add_api_route('/hardware/snapshot/collect', api.hardware_snapshot_collect, methods=['POST'])
    operator_controls.install_operator_control_plane(app,
        maintenance_state_provider=lambda: {'motion_blocked': False, 'recovery_required': False, 'block_reason': None},
        reference_state_provider=lambda: refs.snapshot(('x', 'y', 'z', 'g')),
        lifecycle_state_provider=lambda: {'operation_state': 'stopped'},
        serial206_initialization_state_provider=api.serial206_oem_initialization_provider_status,
        oem_deck_provider=lambda: provider, oem_deck_position_table_provider=load_bound_oem_position_table)
    monkeypatch.setattr(api, 'app', app)
    qualify_test_references(refs)
    client = TestClient(app)
    body = {'expected_generation': int(provider.generation_provider()),
            'idempotency_key': 'readiness-parent-001', 'inputs': {}}
    try:
        response = client.post('/operator/actions/oem.deck.collect_authority', json=body)
        assert response.status_code == 200, response.text
        result = response.json()
        assert calls == [0, 1, 2, 3]
        rows = receipts.connection.execute('SELECT * FROM pipette_operations ORDER BY rowid DESC LIMIT 1').fetchone()
        child = dict(rows)
        assert child['command_id'] != result['command_id']
        source = json.loads(child['source_identity_json'])
        assert source['parent_operator_command_id'] == result['command_id']
        assert child['status'] == 'observed'  # Canonical hardware-query terminal, not motion completion.
        assert result['status'] == 'completed'
        stored_parent = client.get('/operator/actions/receipts/' + result['command_id'])
        assert stored_parent.status_code == 200, stored_parent.text
        assert stored_parent.json()['status'] == 'completed'
        replay = client.post('/operator/actions/oem.deck.collect_authority', json=body)
        assert replay.status_code == 200, replay.text
        assert calls == [0, 1, 2, 3]
    finally:
        app.state.operator_command_plane.stop()


@pytest.mark.parametrize('change', ['reader', 'interrupt'])
def test_reader_replacement_requires_new_query(query_rig, change):
    from types import SimpleNamespace
    app, provider, primitive, refs, root, receipts, calls, wire, transport = query_rig
    qualify_test_references(refs)
    collect(query_rig)
    if change == 'reader':
        for channel in transport._transports:
            channel._get_driver().bus.router = SimpleNamespace(reader_generation=1)
    else:
        transport._interrupt_epoch += 1
    collect(query_rig)
    assert calls == [0, 1, 2, 3] * 2


def test_automatic_foreground_yield_does_not_query(query_rig, monkeypatch):
    from bioxp import api
    app = query_rig[0]
    monkeypatch.setattr(app.state, 'operator_normal_action_active', lambda: True)
    result = collect(query_rig, automatic=True)
    assert result['reason'] == 'operator_action_pending'
    assert query_rig[6] == []


def test_foreground_arrival_during_bounded_tip_query_defers_deck(query_rig, monkeypatch):
    from bioxp import api
    app, provider, primitive, refs, root, receipts, calls, wire, transport = query_rig
    qualify_test_references(refs)
    busy = {'value': False}
    monkeypatch.setattr(app.state, 'operator_normal_action_active', lambda: busy['value'])
    monkeypatch.setattr(api, '_hardware_collectors', lambda *a, **kw: {})
    def arrived(channel):
        busy['value'] = True
    wire['during'] = arrived
    result = collect(query_rig, automatic=True)
    assert calls == [0, 1, 2, 3]  # One indivisible source query, no channel-level replay.
    assert result['park_tip_observation']['deck_state_publication']['status'] == 'published'
    assert result['deck_authority']['reason'] == 'operator_action_pending'


def test_late_foreground_yield_does_not_query(query_rig, monkeypatch):
    from bioxp import api
    app = query_rig[0]
    busy = {'value': False}
    monkeypatch.setattr(app.state, 'operator_normal_action_active', lambda: busy['value'])
    monkeypatch.setattr(api, '_hardware_collectors', lambda *a, **kw: {})
    def collected(*a, **kw):
        busy['value'] = True
        return {'ok': True, 'snapshot': {}}
    monkeypatch.setattr(api.hardware_state, 'collect', collected)
    result = collect(query_rig, automatic=True)
    assert result['deck_authority']['reason'] == 'operator_action_pending'
    assert query_rig[6] == []
