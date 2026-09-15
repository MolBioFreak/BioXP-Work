"""Offline API/active-worker/native/SQLite/readiness qualification.

Explicit copied lifecycle and controller leaf doubles, NOT physical acceptance.
No executor, Park provider, terminal finalizer or receipt store is replaced.
"""
import json
import os
import subprocess
import sys
import threading
from contextlib import contextmanager
from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig, qualify_test_references
from tests.test_deck_scoped_integration import (
    installed_retained, qualify_full_predecessor, catalog_payload,
)
from tests.test_deck_automatic_refresh_owner import request, finish
from tests.test_deck_near_terminal import NearUSB
from tests.test_deck_tip_query_publication import query_rig, query


def export(name, row):
    if os.environ.get('DECK_TEST_OUTPUT'):
        Path(os.environ['DECK_TEST_OUTPUT'] + '.matrix-' + name + '.json').write_text(
            json.dumps(row, indent=2))


@pytest.mark.parametrize('target', ['LOC_OC', 'LOC_TC_BARCODE', 'LOC_RC_BARCODE', 'LOC_PARK', 'source_noop'])
def test_native_terminal_automatic_refresh_matrix(query_rig, retained_rig, monkeypatch, target):
    from bioxp import api, oem_machine_bundle
    from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
    from bioxp.serial206_y_provider import Serial206YProvider
    app, provider, observations, references, root = query_rig[:5]
    app.add_api_route('/hardware/snapshot/collect', api.hardware_snapshot_collect, methods=['POST'])
    api.serial206_oem_initialization_provider_status()
    qualify_test_references(references)
    runtime = retained_rig[2]
    store = app.state.operator_command_plane.store
    observations.calls.clear()
    qualify_full_predecessor((provider, observations, runtime, references, store, root))
    snapshot = oem_machine_bundle.get_active_oem_machine_snapshot()
    monkeypatch.setattr(oem_machine_bundle, '_active_snapshot',
        oem_machine_bundle.load_oem_machine_snapshot(snapshot.bundle_root / 'OEM_EVIDENCE_LOCK.json',
            operator_label_serial=206, require_operator_label=True))
    generation = int(provider.generation_provider())

    class MatrixUSB(NearUSB):
        expected_sta = True

        def begin_bus_event_window(self):
            return {**super().begin_bus_event_window(), 'owner_generation': generation}

        def motor_wait_target_reached(self, *args, **kwargs):
            row = super().motor_wait_target_reached(*args, **kwargs)
            row['event']['owner_generation'] = generation
            return row

        def motor_wait_target_reached_many(self, axes, **kwargs):
            # Source manual ordinary uses STA; native script Park does not.
            assert kwargs['sta_sequential'] is self.expected_sta
            return {'ok': True, 'per_axis': {axis: self.motor_wait_target_reached(board)
                for axis, board in [('x', 5), ('y', 4)]}}

    leaf = MatrixUSB((0, 0))
    adapter = Serial206ProductionPrimitiveAdapter(leaf, None, authority_provider=lambda: {},
        generation_provider=lambda: generation, reference_store=references)
    adapter.y_provider = Serial206YProvider(leaf, state_store=runtime,
        generation_provider=lambda: generation, reference_store=references)
    for name in ('oem_move_to', 'oem_move_z', 'oem_initialize_motion_scriptmove_to_waste'):
        monkeypatch.setattr(observations, name, getattr(adapter, name), raising=False)
    queries = []
    def position(axis):
        queries.append(axis)
        return leaf.positions[{'x': (5, 0), 'y': (4, 0), 'z': (4, 1)}[axis]]
    monkeypatch.setattr(observations, '_read_axis_position', position)
    monkeypatch.setattr(api, '_hardware_collectors', lambda tester, **kwargs: {})
    # Existing fixture doubles only generic hardware collection; native deck
    # collection and its scope/owner/cache publication remain actual owners.
    client = TestClient(app)
    tip_query = query(query_rig, 'matrix-tip-' + target)
    assert tip_query['hardware_query_verified'] is True
    if target == 'LOC_PARK':
        predecessor = request(provider, 'matrix-park-predecessor')
        predecessor['inputs']['target'] = 'LOC_OC'
        accepted = client.post('/operator/v2/actions/oem.deck.move_to_location', json=predecessor)
        assert accepted.status_code == 200, accepted.text
        app.state.operator_command_plane.start()
        previous = finish(client, accepted.json()['command_id'])
        assert previous['status'] == 'completed', previous
        assert store.wait_for_command_workers([accepted.json()['command_id']], timeout=3)
        leaf.moves.clear()
        leaf.expected_sta = False
    warm_before = catalog_payload(app)
    body = request(provider, 'matrix-' + target)
    body['inputs']['target'] = 'LOC_PARK' if target == 'source_noop' else target
    admitted = client.post('/operator/v2/actions/oem.deck.move_to_location', json=body)
    assert admitted.status_code == 200, admitted.text
    command_id = admitted.json()['command_id']
    app.state.operator_command_plane.start()
    detail = finish(client, command_id)
    export(target + '-terminal', {'detail': detail, 'moves': leaf.moves})
    assert detail['status'] == 'completed', detail
    assert detail['deck_movement']['semantic_state_committed'] is True
    assert store.wait_for_command_workers([command_id], timeout=3)
    before_refresh = provider.deck_observation_freshness(expected_generation=generation)
    assert before_refresh['available'] is False, before_refresh
    assert bool(leaf.moves) is (target != 'source_noop')
    if target != 'source_noop':
        from bioxp.oem_compat.position_table import load_bound_oem_position_table
        from bioxp.oem_deck_catalog import DeckCatalog
        from tests.test_deck_complete_oem import expected
        table = load_bound_oem_position_table()
        destination = DeckCatalog.from_position_table(table).resolve(target)
        if target == 'LOC_PARK':
            park = table.resolve(location_id='LOC_PARK')
            wire = {**dict(park.base_coordinates), 'z': park.z_low}
        else:
            _, wire = expected(table, destination)
        assert leaf.positions == {(5, 0): wire['x'], (4, 0): wire['y'], (4, 1): wire['z']}
        assert store.deck_semantic_state()['current_location'] == destination.location_name
    if target == 'source_noop':
        assert detail['completion_class'] == 'source_noop'
        assert detail['deck_movement']['delivery_attempted'] is False
    else:
        assert detail['deck_movement']['controller_completion_verified'] is True
    queries.clear()
    refreshed = client.post('/hardware/snapshot/collect', json={
        'automatic': True, 'domains': ['axes', 'latch']})
    assert refreshed.status_code == 200, refreshed.text
    after_refresh = provider.deck_observation_freshness(expected_generation=generation)
    assert after_refresh['available'] is True, (refreshed.json(), after_refresh)
    scopes = {name: provider.deck_authority_cached_snapshot(expected_generation=generation, target=name)
              for name in ('LOC_OC', 'LOC_PARK')}
    assert scopes['LOC_OC']['dependency_scope'] == 'offset.v1'
    assert scopes['LOC_PARK']['dependency_scope'] == 'full'
    assert scopes['LOC_OC']['captured_at'] == scopes['LOC_PARK']['captured_at']
    assert sorted(queries) == ['x', 'y', 'z'], queries
    for scope in scopes.values():
        for axis in 'xyz':
            assert scope['current_' + axis] == leaf.positions[{'x': (5, 0), 'y': (4, 0), 'z': (4, 1)}[axis]]
    before_passive = (list(queries), list(leaf.moves))
    catalog = catalog_payload(app)
    assert before_passive == (queries, leaf.moves), 'passive catalog issued hardware work'
    assert provider.deck_observation_freshness(expected_generation=generation)['available']
    compact = client.get('/operator/v2/actions/receipts/' + command_id).json()
    app.state.operator_command_plane.stop()
    script = ('import json,sys; from tests.test_deck_scoped_integration import fresh_process_receipts; '
              'print(json.dumps(fresh_process_receipts(sys.argv[1], sys.argv[2])))')
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', script, str(root), command_id], text=True))
    assert reopened['compact'] == compact and reopened['detail'] == detail
    assert compact['physical_effect_verified'] is False
    export(target, {'warm_before': warm_before, 'catalog': catalog, 'detail': detail,
        'before_refresh': before_refresh, 'refresh': refreshed.json(), 'after_refresh': after_refresh,
        'scopes': scopes, 'moves': leaf.moves, 'fresh_process': reopened})


def test_pending_admission_arrives_during_real_lease_wait(installed_retained, monkeypatch):
    from bioxp import api
    app, provider, primitive, references, root = installed_retained
    api.serial206_oem_initialization_provider_status()
    qualify_test_references(references)
    provider.deck_authority_snapshot(expected_generation=int(provider.generation_provider()), target='LOC_OC')
    original = provider.movement_lease
    waiting, admission_entered, admission_release = (threading.Event() for _ in range(3))
    refreshes, admissions, errors = [], [], []
    @contextmanager
    def lease():
        waiting.set()
        with original():
            yield
    monkeypatch.setattr(provider, 'movement_lease', lease)
    reader = app.state.operator_admission_state_reader
    collect = reader._collect
    def held_admission():
        admission_entered.set()
        assert admission_release.wait(5)
        return collect()
    monkeypatch.setattr(reader, '_collect', held_admission)
    monkeypatch.setattr(api, '_hardware_collectors', lambda tester, **kwargs: {})
    def refresh():
        try:
            refreshes.append(api._collect_and_publish_hardware_snapshot(['axes', 'latch'], reason='isolated-lease', automatic=True))
        except BaseException as exc:
            errors.append(repr(exc))
    body = request(provider, 'lease-pending-admission')
    submit = threading.Thread(target=lambda: admissions.append(TestClient(app).post(
        '/operator/v2/actions/oem.deck.move_to_location', json=body)))
    worker = threading.Thread(target=refresh)
    epoch = provider._deck_authority_cache_epoch
    calls = list(primitive.calls)
    try:
        with original():
            worker.start()
            assert waiting.wait(3)
            submit.start()
            assert admission_entered.wait(3)
            assert app.state.operator_normal_action_active()
        worker.join(3)
        assert not worker.is_alive() and not errors, errors
        assert refreshes[0]['deck_authority'] == {'available': False, 'reason': 'operator_action_pending'}
        assert provider._deck_authority_cache_epoch is epoch
        assert primitive.calls == calls
    finally:
        admission_release.set()
        if submit.ident is not None:
            submit.join(5)
        worker.join(5)
    assert admissions[0].status_code == 200, admissions[0].text
    assert not any(row[0] == 'move' for row in primitive.calls)
    export('lease', {'refreshes': refreshes, 'admission': admissions[0].json(), 'queries_unchanged': primitive.calls == calls})
