"""Real provider + disposable canonical SQLite + actual installed API contracts.
No device/network access: only the primitive I/O and upstream runtime owner are fake.
Explicit predecessor/tray fixtures are not claims about fresh machine readiness.
"""
import time
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from bioxp import operator_controls, api
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider
from test_r4_named_destination_vectors import rig, CASES
from test_r4_deck_producer_refresh import complete_predecessor
from test_oem_deck_install_binding import FakeRuntimeStore, FakeDeckPrimitives, FakeReferenceStore


@pytest.fixture
def installed(tmp_path, monkeypatch, rig, request):
    from bioxp.oem_runtime_store import OEMRuntimeStore
    OEMRuntimeStore(tmp_path / 'canonical-oem-runtime').close()
    monkeypatch.setenv('BIOXP_RUNTIME_STATE_ROOT', str(tmp_path))
    monkeypatch.setenv('BIOXP_OEM_RUNTIME_ROOT', str(tmp_path / 'canonical-oem-runtime'))
    monkeypatch.setattr(operator_controls, 'current_release_identity', lambda: {
        'verified': True, 'release_id': 'offline-deck-fixture',
        'source': {'manifest_sha256': '1' * 64, 'aggregate_sha256': '2' * 64}})
    monkeypatch.setattr(operator_controls, 'current_authority_identity', lambda: {
        'evidence_lock_identity_verified': True, 'evidence_lock_sha256': '3' * 64})
    monkeypatch.setattr(operator_controls, 'current_registry_sha256', lambda: '4' * 64)
    state = Serial206OemInitializationProvider._new_state()
    if getattr(request, 'param', 'legacy') == 'legacy':
        # Retained incomplete predecessor, not the source's fresh constructor.
        state['machine_status'].pop('construction_id', None)
        state['machine_status'].pop('constructed_tip_trays', None)
        state['machine_status'].update(current_location=None, current_well=None, tip_loaded=None, tip_dirty=None)
    state['x_lifecycle']['board_lifecycle_generation'] = 11
    runtime = FakeRuntimeStore(state)
    primitives = FakeDeckPrimitives()
    original_latch_query = primitives.deck_io_query_type
    def observed_latch(io_type=3):
        observation = original_latch_query(io_type)
        observation['ack']['provenance'] = {'reader_sequence': primitives.latch_reads}
        return observation
    primitives.deck_io_query_type = observed_latch
    refs = FakeReferenceStore().snapshot(('x', 'y', 'z', 'g'))
    references = type('References', (), {'snapshot': lambda self, axes: refs})()
    generation = {'value': 7}
    monkeypatch.setattr(type(operator_controls.hardware_state), 'ownership_epoch',
                        property(lambda self: generation['value']))
    provider = Serial206OemInitializationProvider(primitives, state_store=runtime,
        reference_store=references, generation_provider=lambda: generation['value'])
    slot = {'provider': provider}
    monkeypatch.setattr(operator_controls.hardware_state, 'project', lambda *names, **kwargs: {'domains': {}, 'freshness': {'state': 'fresh', 'age_s': 0, 'fresh_for_s': 30}})
    monkeypatch.setattr(operator_controls.hardware_state, 'ownership_projection', lambda: {'ownership_epoch': generation['value'], 'ownership': {'transport': 'owned', 'usb': 'service', 'router': 'running', 'CAN_READY': True}})
    app = FastAPI()
    app.add_api_route('/hardware/snapshot/collect', api.hardware_snapshot_collect, methods=['POST'])
    operator_controls.install_operator_control_plane(app,
        maintenance_state_provider=lambda: {'motion_blocked': False, 'recovery_required': False, 'block_reason': None},
        reference_state_provider=lambda: refs,
        lifecycle_state_provider=lambda: {'operation_state': 'stopped'},
        serial206_initialization_state_provider=lambda: {'x_authority': {'active_board_epoch': runtime.state['x_lifecycle']['board_lifecycle_generation']}, 'board4_authority': {'active_board_epoch': 10}},
        oem_deck_provider=lambda: slot['provider'], oem_deck_position_table_provider=lambda: rig[0])
    monkeypatch.setattr(api, 'app', app)
    monkeypatch.setattr(api, '_get_tester', lambda: object())
    monkeypatch.setattr(api, '_hardware_collectors', lambda tester: {})
    # Existing collection execution is faked; deck collection remains real.
    monkeypatch.setattr(api.hardware_state, 'collect', lambda requested, collectors: {'ok': True, 'snapshot': {}})
    yield app, provider, runtime, refs, generation, slot
    app.state.operator_command_plane.stop()


def action(app):
    client = TestClient(app)
    client.get('/operator/v2/control-catalog')
    # Metadata is stale-while-refresh, not synchronous collection. Drain the
    # already-requested passive refresh deterministically; never query hardware.
    pending = app.state.operator_poll_cache._pending
    if pending is not None:
        pending.result(timeout=2)
    response = client.get('/operator/v2/control-catalog')
    assert response.status_code == 200, response.text
    return next(row for row in response.json()['actions'] if row['action_id'] == 'oem.deck.move_to_location')


def ready(installed, *, location='LOC_MS'):
    app, provider, runtime, *_ = installed
    complete_predecessor(runtime, channel=-1)
    runtime.state['machine_status']['current_location'] = location
    provider.publish_tip_tray_transition(tray_id=0, transition='construct', operation_id='fixture-constructor',
        command_id='fixture-construct', provenance={'source': 'ClassTipTray..ctor', 'kind': 'host_semantic_default'})
    result = api._collect_and_publish_hardware_snapshot(['axes', 'latch'], reason='test-explicit-collection')
    assert result['deck_authority']['enabled'] is True, result
    assert app.state.operator_command_plane.store.deck_semantic_state()['tip_location'] == -1
    return result


def test_cold_warm_exact_15_second_expiry_and_explicit_recollection(installed):
    app, provider, *_ = installed
    assert action(app)['enabled'] is False
    ready(installed)
    before_reads = provider.primitives.latch_reads
    snapshot = provider.deck_authority_cached_snapshot(expected_generation=7)
    assert action(app)['enabled'] is True
    assert provider.primitives.latch_reads == before_reads
    assert provider.deck_authority_cached_snapshot(expected_generation=7)['captured_at'] == snapshot['captured_at']
    _, epoch, stored = provider._deck_authority_cache
    provider._deck_authority_cache = (time.monotonic() - 15.0, epoch, stored)
    assert action(app)['disabled_reason'] == 'canonical_deck_authority_unavailable:deck_authority_cache_stale'
    assert provider.primitives.latch_reads == before_reads
    ready(installed)
    assert action(app)['enabled'] is True


@pytest.mark.parametrize('change', ['reference', 'reference_missing', 'epoch', 'ownership', 'semantic', 'provider', 'failed_collection'])
def test_external_owner_changes_do_not_inherit_ready_cache(installed, monkeypatch, change):
    app, provider, runtime, refs, generation, slot = installed
    ready(installed)
    reads = provider.primitives.latch_reads
    if change == 'reference': refs['rows']['x']['state_version'] += 1
    elif change == 'reference_missing': del refs['rows']['x']
    elif change == 'epoch': runtime.state['x_lifecycle']['board_lifecycle_generation'] += 1
    elif change == 'ownership': generation['value'] += 1
    elif change == 'semantic':
        app.state.operator_command_plane.store.publish_deck_owner_state(source_operation='pipette_owner', source_command_id='external-tip-owner',
            updates={'tip_loaded': True, 'tip_dirty': True, 'tip_location': -1}, ownership_generation=7, board_epoch_4=10, board_epoch_5=11)
    elif change == 'provider': slot['provider'] = None
    else:
        monkeypatch.setattr(api.hardware_state, 'collect', lambda *args: {'ok': False})
        api._collect_and_publish_hardware_snapshot(['axes', 'latch'], reason='failed-explicit-collection')
    assert action(app)['enabled'] is False
    assert provider.primitives.latch_reads == reads


def test_same_provider_late_predecessor_bootstrap_and_late_binding(installed):
    app, provider, runtime, _, _, slot = installed
    assert app.state.oem_deck_bootstrap_diagnostic['reason'] == 'deck_bootstrap_semantic_location_unavailable'
    slot['provider'] = None
    assert action(app)['enabled'] is False
    slot['provider'] = provider
    assert action(app)['enabled'] is False
    assert app.state.oem_deck_provider is provider
    assert app.state.operator_command_plane.store.deck_semantic_state()['semantic_state_revision'] == 0
    ready(installed)
    assert action(app)['enabled'] is True


@pytest.mark.parametrize('row,offset', CASES, ids=[f'{r[0]}-camera={o}' for r,o in CASES])
def test_all_panel_vectors_real_provider_catalog_and_fresh_durable_admission(installed, row, offset):
    app, provider, *_ = installed
    ready(installed, location='LOC_PARK' if row[0] == 'LOC_PARK' else 'LOC_MS')
    catalog = action(app)
    assert len(catalog['destination_options']) == 26
    assert next(r for r in catalog['destination_options'] if r['target'] == row[0])['enabled'] is True
    reads = provider.primitives.latch_reads
    response = TestClient(app).post('/operator/v2/actions/oem.deck.move_to_location', json={
        'schema_version': 'bioxp.operator_action_request.v2', 'idempotency_key': 'fixture-panel-admission',
        'expected_ownership_generation': 7, 'expected_board_epoch_by_board': {'4': 10, '5': 11},
        'inputs': {'target': row[0], 'camera_offset': offset}})
    assert response.status_code == 200, response.text
    assert provider.primitives.latch_reads > reads  # admission never authorizes from cache
    receipt = TestClient(app).get('/operator/commands/' + response.json()['command_id'])
    assert receipt.status_code == 200
    assert receipt.json()['status'] == 'queued'
    command_id = response.json()['command_id']
    app.state.operator_command_plane.start()
    store = app.state.operator_command_plane.store
    terminal = None
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        terminal = store.get_command(command_id)
        if terminal['status'] not in {'queued', 'dispatched', 'issued_pending'}: break
        time.sleep(.01)
    import json
    assert terminal['status'] == 'completed', json.dumps(terminal, indent=2)
    assert terminal['deck_movement']['semantic_state_committed'] is True
    assert terminal['deck_movement']['physical_observation_verified'] is False
    # Park is the source's already-at-Park no-op, never a failed-eject exception.
    if row[0] == 'LOC_PARK':
        assert provider.primitives.calls == []
        assert terminal['source_noop'] is True
        assert terminal['source_noop_reason'] == 'already_at_park'
        assert terminal['controller_acknowledged'] is False
        assert terminal['remote_acknowledged'] is False
        assert terminal['physical_effect_verified'] is False
        assert terminal['completion_class'] == 'source_noop'


def test_incomplete_source_collection_does_not_fabricate_no_tip_location_or_tray(installed):
    app, provider, runtime, *_ = installed
    runtime.state['machine_status']['tip_loaded'] = False
    result = api._collect_and_publish_hardware_snapshot(['axes', 'latch'], reason='missing-source-events')
    assert result['deck_authority']['enabled'] is False
    store = app.state.operator_command_plane.store
    assert store.deck_semantic_state()['current_location'] is None
    assert store.tip_tray_state(0)['tip_available'] is None
    assert action(app)['enabled'] is False


def submit(app, key='fixture-cancel-recovery', *, target='LOC_OC'):
    response = TestClient(app).post('/operator/v2/actions/oem.deck.move_to_location', json={
        'schema_version': 'bioxp.operator_action_request.v2', 'idempotency_key': key,
        'expected_ownership_generation': 7, 'expected_board_epoch_by_board': {'4': 10, '5': 11},
        'inputs': {'target': target, 'camera_offset': False}})
    assert response.status_code == 200, response.text
    return response.json()['command_id']


def test_cancel_undispatched_real_provider_receipt_never_claims_delivery(installed):
    app, provider, *_ = installed
    ready(installed)
    command_id = submit(app)
    store = app.state.operator_command_plane.store
    receipt = store.get_command(command_id)
    recovery = store.recovery()
    result = store.cancel_command(command_id, {
        'idempotency_key': 'fixture-cancel', 'expected_version': receipt['state_version'],
        'expected_recovery_epoch': recovery['recovery_epoch'],
        'expected_global_safety_epoch': recovery['global_safety_epoch'],
        'expected_axis_safety_epoch': recovery['z_safety_epoch']})
    assert result['status'] == 'cleared'
    assert result['terminal_evidence']['reason'] == 'standalone_cancel'
    assert provider.primitives.calls == []
    assert store.recovery()['hold'] is False
    assert app.state.oem_deck_authority_collector()['enabled'] is True
    assert action(app)['enabled'] is True


def test_real_provider_failure_recovery_requires_truth_and_explicit_refresh(installed):
    from test_oem_deck_install_binding import _reconciliation_body
    app, provider, *_ = installed
    ready(installed)
    provider.primitives.oem_move_to = lambda *args, **kwargs: {'ok': False, 'controller_command_acknowledged': True}
    command_id = submit(app)
    app.state.operator_command_plane.start()
    store = app.state.operator_command_plane.store
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        receipt = store.get_command(command_id)
        if receipt['status'] == 'ambiguous': break
        time.sleep(.01)
    import json
    assert receipt['status'] == 'ambiguous', json.dumps(receipt, indent=2)
    assert action(app)['enabled'] is False
    assert app.state.oem_deck_authority_collector()['enabled'] is False
    # Explicit fake source observation; not a nearest-coordinate production inference.
    import hashlib
    coordinates = {'x': 100, 'y': 200, 'z': 65000}
    observation_id = hashlib.sha256(json.dumps(coordinates, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    provider.primitives.read_deck_semantic_observation = lambda: {
        'location_id': 'LOC_MS', 'well_id': 0, **coordinates, 'controller_position_observation_id': observation_id}
    response = TestClient(app).post('/operator/recovery/deck/' + command_id + '/reconcile', json=_reconciliation_body())
    assert response.status_code == 200, response.text
    assert store.recovery()['hold'] is False
    assert action(app)['enabled'] is False
    assert app.state.oem_deck_authority_collector()['enabled'] is True
    assert action(app)['enabled'] is True


def test_failed_park_ejection_refusal_is_not_reclassified_as_a_noop(installed):
    app, provider, *_ = installed
    ready(installed)
    ejections = []
    def fail_eject(**kwargs):
        ejections.append(kwargs)
        return {'ok': False, 'failure': 'fixture-eject-failed'}
    provider.primitives.eject_all_tips_for_oem_park = fail_eject
    provider.primitives.oem_initialize_motion_move_absolute = lambda *args, **kwargs: {'ok': True}
    command_id = submit(app, target='LOC_PARK')
    app.state.operator_command_plane.start()
    store = app.state.operator_command_plane.store
    receipt = None
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        receipt = store.get_command(command_id)
        if receipt['status'] not in {'queued', 'dispatched', 'issued_pending'}: break
        time.sleep(.01)
    assert ejections
    assert receipt['status'] == 'ambiguous'
    assert receipt['source_noop'] is False
    assert store.deck_semantic_state()['current_location'] == 'LOC_MS'
    assert store.recovery()['hold'] is True
    assert action(app)['enabled'] is False


@pytest.mark.parametrize('host,sensor', [(False, 1), (True, 0)])
def test_independent_latch_refusal_is_preserved_in_ready_catalog(installed, host, sensor):
    app, provider, *_ = installed
    ready(installed)
    provider.primitives.read_oem_latch_status = lambda: {'ok': True, 'value': host, 'observation_id': 'fixture-host-new'}
    provider.primitives.deck_io_query_type = lambda io_type=3: {'ok': True, 'value': sensor, 'ack': {'status': 100, 'value': sensor}}
    assert app.state.oem_deck_authority_collector()['disabled_reason'] == 'latch_not_closed'
    assert action(app)['disabled_reason'] == 'latch_not_closed'


@pytest.mark.parametrize('complete', [False, True])
def test_v2_ui_explicit_collection_to_deck_move_and_expiry(installed, monkeypatch, complete, request):
    app, provider, runtime, *_ = installed
    collected = []
    def collect(requested, collectors):
        collected.append(requested)
        return {'ok': True, 'snapshot': {}}
    monkeypatch.setattr(api.hardware_state, 'collect', collect)
    if complete:
        complete_predecessor(runtime, channel=-1)
        provider.publish_tip_tray_transition(tray_id=0, transition='construct', operation_id='ui-fixture',
            command_id='ui-fixture', provenance={'source': 'ClassTipTray..ctor', 'kind': 'host_semantic_default'})
    client = TestClient(app)
    client.__enter__()
    request.addfinalizer(lambda: client.__exit__(None, None, None))
    for _ in range(2):
        catalog = client.get('/operator/v2/control-catalog').json()
        refresh = next(row for row in catalog['actions'] if row['action_id'] == 'oem.deck.collect_authority')
        assert refresh['enabled'] is True, refresh
        assert not next(row for row in catalog['actions'] if row['action_id'] == 'oem.deck.move_to_location')['enabled']
    assert collected == [] and provider.primitives.latch_reads == 0
    for attempt in range(2):
        response = client.post('/operator/v2/actions/oem.deck.collect_authority', json={
            'schema_version': 'bioxp.operator_action_request.v2', 'idempotency_key': f'ui-collect-{attempt}',
            'expected_ownership_generation': 7, 'expected_board_epoch_by_board': {}, 'inputs': {}})
        assert response.status_code == 200, response.text
        assert response.json()['status'] == 'queued', response.json()
        terminal = response.json()
        deadline = time.monotonic() + 2
        while not terminal['terminal'] and time.monotonic() < deadline:
            time.sleep(.01)
            terminal = client.get(response.json()['status_path']).json()
        assert terminal['status'] == 'completed', terminal
        assert collected[-1] == ['axes', 'latch']
        assert provider.primitives.calls == []
        current = action(app)
        assert current['enabled'] is complete, current
        if not complete:
            assert app.state.operator_command_plane.store.deck_semantic_state()['current_location'] is None
            assert app.state.operator_command_plane.store.tip_tray_state(0)['tip_available'] is None
            blocked = client.post('/operator/v2/actions/oem.deck.move_to_location', json={
                'schema_version': 'bioxp.operator_action_request.v2', 'idempotency_key': 'ui-blocked-move',
                'expected_ownership_generation': 7, 'expected_board_epoch_by_board': {'4': 10, '5': 11},
                'inputs': {'target': 'LOC_OC', 'camera_offset': False}})
            assert blocked.status_code == 409
            break
        if attempt == 0:
            _, epoch, cached = provider._deck_authority_cache
            provider._deck_authority_cache = (time.monotonic() - 15.0, epoch, cached)
            reads = provider.primitives.latch_reads
            assert not action(app)['enabled']
            assert provider.primitives.latch_reads == reads
    if complete:
        command_id = submit(app, key='ui-refreshed-deck-move')
        app.state.operator_command_plane.start()
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            receipt = app.state.operator_command_plane.store.get_command(command_id)
            if receipt['status'] not in {'queued', 'dispatched', 'issued_pending'}:
                break
            time.sleep(.01)
        assert receipt['status'] == 'completed', receipt
        assert receipt['deck_movement']['semantic_state_committed'] is True
        assert not receipt['physical_effect_verified']


def test_v2_collection_alias_keeps_transport_gate(installed, monkeypatch):
    app, provider, *_ = installed
    monkeypatch.setattr(operator_controls.hardware_state, 'ownership_projection',
        lambda: {'ownership_epoch': 7, 'ownership': {'transport': 'unowned', 'usb': 'released', 'router': 'stopped'}})
    client = TestClient(app)
    row = next(r for r in client.get('/operator/v2/control-catalog').json()['actions']
               if r['action_id'] == 'oem.deck.collect_authority')
    assert not row['enabled']
    result = client.post('/operator/v2/actions/oem.deck.collect_authority', json={
        'schema_version': 'bioxp.operator_action_request.v2', 'idempotency_key': 'ui-no-transport',
        'expected_ownership_generation': 7, 'expected_board_epoch_by_board': {}, 'inputs': {}})
    assert result.status_code == 409, result.text
    assert provider.primitives.calls == [] and provider.primitives.latch_reads == 0
