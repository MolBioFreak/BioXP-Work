"""Offline WP8 Park producer/SQLite/RunJob regression; physical leaves doubled."""
import copy
import json

import pytest
from tests.protocol_v1_integration_fixture import integrated_rig
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig
from tests.test_camera_oem_led_binding import led_rig
from tests.test_protocol_live_prepare_connected import (
    connected_camera, connected_motion_wait, authored_job,
)
from tests.test_protocol_workflow_connected import await_job
from tests.test_wake_lineage_connected import fresh_semantics_refs
from tests.test_protocol_oem_native import park_rig, result


IDENTITY = dict(command_id='park-parent', child_order=3, plan_digest='actual-plan')


def test_preparation_publication_then_real_runjob_park(integrated_rig, monkeypatch):
    rig = integrated_rig
    query = rig.client.post('/liquid/tip-status', headers={'Idempotency-Key': 'park-query'})
    assert query.status_code == 200, query.text
    observed = []
    real = rig.provider.wp8_park_gantry
    native_calls = []
    script_move = rig.provider.primitives.oem_initialize_motion_scriptmove_to_waste
    def observe_move(**kwargs):
        native_calls.append(kwargs)
        return script_move(**kwargs)
    monkeypatch.setattr(rig.provider.primitives, 'oem_initialize_motion_scriptmove_to_waste', observe_move, raising=False)

    def observe(operation, arguments, **identity):
        before_moves = len(native_calls)
        native = real(operation, arguments, **identity)
        state = dict(rig.store.connection.execute(
            'SELECT * FROM operator_plane_deck_semantic_state').fetchone())
        observed.append((native, identity, state, len(native_calls) - before_moves))
        return native

    monkeypatch.setattr(rig.provider, 'wp8_park_gantry', observe)
    job = rig.start(authored_job(rig, 'park-publication'))
    gate = await_job(rig.client, job['job_id'], lambda row:
        row['command']['terminal'] or row['execution']['runtime_state']['workflow']['gate'] == 'review', timeout=60)
    assert not gate['command']['terminal'], gate
    response = rig.client.post('/protocol/jobs/' + job['job_id'] + '/review', json={
        'command_id': job['job_id'], 'expected_ownership_generation': job['command']['ownership_generation'],
        'idempotency_key': 'park-ignore', 'reviewer': 'offline-test',
        'stage_id': 'lifecycle:prepare', 'action_id': 'lifecycle:prepare', 'decision': 'ignore'})
    assert response.status_code == 200, response.text
    done = rig.terminal(job)
    assert done['command']['status'] == 'completed', done
    updates = [row for row in observed if row[0].get('source_location_update')]
    assert updates, observed
    native, identity, state, moved = updates[0]
    assert native['ok'] and moved > 0
    assert native['source_location_update'] == {'current_location': 28, 'current_well': 0}
    assert (state['current_location'], state['current_well']) == ('LOC_PARK', 0)
    proof = json.loads(state['transition_provenance_json'])
    assert proof['source_operation'] == 'updateLocation'
    assert proof['upstream_source_command_id'] == rig.provider._wp8_identity(
        identity['command_id'], identity['child_order'], identity['plan_digest'])
    plan = rig.store.connection.execute('SELECT * FROM operator_plane_wp8_children WHERE command_id=? AND child_order=?',
        (identity['command_id'], identity['child_order'])).fetchone()
    assert plan['operation'] == 'parkGantry'
    # RunJob is the next actual Park caller after preparation, not a direct
    # stand-in invocation. Native Park returns before any motion leaf.
    index = observed.index(updates[0])
    next_native, next_identity, _, moved = observed[index + 1]
    assert next_native['source_noop'] is True and moved == 0
    assert next_native['controller_command_acknowledged'] is False
    assert next_native['controller_completion_verified'] is False
    assert next_native['source_children'] == []
    child = rig.store.connection.execute('SELECT receipt_json FROM operator_commands WHERE command_id=?',
        (next_identity['command_id'],)).fetchone()
    assert json.loads(child[0])['effective_inputs']['operation'] == 'thermal_door'
    reopened = fresh_semantics_refs(rig.root)
    assert reopened['projection']['current_location'] == 'LOC_PARK'
    assert reopened['projection']['current_well'] == 0
    assert rig.reopen(done)['workflow']['command']['status'] == 'completed'


@pytest.mark.parametrize('rehome', [False, True])
def test_adapter_preserves_native_evidence_and_identity(park_rig, rehome):
    provider, _, _ = park_rig
    publications = []
    provider._wp8_execution_fence_checker = lambda *a, **k: None
    provider.deck_owner_authority_stamps = lambda: dict(ownership_generation=1, board_epoch_4=2, board_epoch_5=3)
    provider._deck_semantic_state_publisher = lambda **kw: publications.append(kw) or kw
    native = provider.wp8_park_gantry('parkGantry', {'rehome': rehome}, **IDENTITY)
    assert native['source_location_update'] == {'current_location': 28, 'current_well': 0}
    assert native['source_children'] and native['controller_completion_verified'] is True
    assert len(publications) == 1
    assert publications[0]['source_command_id'] == 'park-parent:3:actual-plan'
    assert publications[0]['source_operation'] == 'updateLocation'
    assert publications[0]['updates'] == {'current_location': 'LOC_PARK', 'current_well': 0}


@pytest.mark.parametrize('extra', [dict(ok=False), dict(source_pause_scripts=True),
    dict(semantic_location_commit_allowed=False), dict(source_noop=True), dict(source_location_update=None)])
def test_nonpublishing_return_is_not_promoted(park_rig, extra):
    provider, _, _ = park_rig
    native = dict(ok=True, source_location_update={'current_location': 28, 'current_well': 0},
        controller_command_acknowledged=False, controller_completion_verified=False, source_children=[])
    native.update(extra)
    saved = copy.deepcopy(native)
    provider.parkGantry = lambda **kw: native
    provider.wp8_update_location = lambda *a, **k: pytest.fail('must not publish')
    assert provider.wp8_park_gantry('parkGantry', {}, **IDENTITY) is native
    assert native == saved


@pytest.mark.parametrize('missing', ['controller_command_acknowledged', 'controller_completion_verified'])
def test_native_incomplete_return_does_not_publish(park_rig, missing):
    provider, _, _ = park_rig
    leaf = result()
    leaf[missing] = False
    provider.primitives.oem_initialize_motion_scriptmove_to_waste = lambda **kw: leaf
    provider.wp8_update_location = lambda *a, **k: pytest.fail('must not publish')
    native = provider.wp8_park_gantry('parkGantry', {}, **IDENTITY)
    assert native['ok'] is False
    assert native[missing] is False
    assert native['source_location_update'] == {'current_location': 28, 'current_well': 0}


@pytest.mark.parametrize('failure', ['failed-native', 'lost-steps', 'already-parked', 'publication'])
def test_native_negatives(park_rig, failure):
    provider, state, trace = park_rig
    provider._wp8_execution_fence_checker = lambda *a, **k: None
    publications = []
    def publish(*args, **kwargs):
        publications.append(kwargs)
        raise RuntimeError('publication failed')
    provider.wp8_update_location = publish
    if failure == 'failed-native':
        provider.primitives.oem_initialize_motion_scriptmove_to_waste = lambda **kw: {'ok': False}
    elif failure == 'lost-steps':
        provider.primitives.home_xy = lambda: result(source_return={'x': 101, 'y': 0})
    elif failure == 'already-parked':
        state['current_location_id'] = 'LOC_PARK'
    if failure in ('failed-native', 'publication'):
        with pytest.raises(RuntimeError, match='park_source_child_failed|publication failed'):
            provider.wp8_park_gantry('parkGantry', {}, **IDENTITY)
    else:
        native = provider.wp8_park_gantry('parkGantry', {'rehome': True}, **IDENTITY)
        assert native.get('source_pause_scripts') or native.get('source_noop')
        assert 'source_location_update' not in native
    assert len(publications) == (1 if failure == 'publication' else 0)
    if failure == 'already-parked':
        assert trace == []
