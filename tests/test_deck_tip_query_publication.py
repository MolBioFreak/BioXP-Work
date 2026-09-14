"""Offline API/audit/SQLite/deck handoff; only CAN exchange is replaced.

Captured parent ?31 reply bytes: complete-system/park-tip/valid-query-result.json,
command pipette_39af8ac917b5459baaec040b371c9727. Fresh exchange identities below
belong to this isolated replay, not to the historical physical observation.
"""
import copy
import json
import os
import time
from pathlib import Path
import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig, qualify_test_references
from tests.test_deck_scoped_integration import installed_retained, catalog_action


@pytest.fixture
def query_rig(installed_retained, monkeypatch):
    from bioxp import api, operator_controls
    from bioxp.can_driver import BioXpCanDriver
    from bioxp.pipette import receipts
    from bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport
    app, provider, primitive, references, root = installed_retained
    # Infrastructure identity fixture shared with the installed control plane.
    for name in ('current_release_identity', 'current_authority_identity', 'current_registry_sha256'):
        monkeypatch.setattr(receipts, name, getattr(operator_controls, name))
    receipt_store = receipts.PipetteReceiptStore(root)
    monkeypatch.setattr(api, '_pipette_receipts', receipt_store)
    calls, wire = [], {'data': [[32, 96, 48] for _ in range(4)], 'correlated': True}
    transports = []
    for channel in range(4):
        driver = BioXpCanDriver.__new__(BioXpCanDriver)
        from types import SimpleNamespace
        driver.bus = SimpleNamespace(router=SimpleNamespace(reader_generation=1))
        def exchange(command, *, address, ack_mode, command_name, channel=channel):
            assert (command, address, ack_mode, command_name) == ('?31','report','query','query_tip_status')
            calls.append(channel)
            if wire.get('failure'):
                raise RuntimeError('isolated CAN failure')
            if callable(wire.get('during')):
                wire['during'](channel)
            now = time.monotonic()
            return {'ok': True, 'query_response_correlated': wire['correlated'],
                'command_name': command_name, 'payload': [63,51,49], 'board_id': 262 + channel * 8,
                'ack': {'received': not wire.get('missing'), 'arbitration_id': 1286 + channel * 8,
                    'data': wire['data'][channel]},
                'provenance': {'channel': channel,
                    'owner_generation': 2 if wire.get('reader_drift') and channel == 3 else 1,
                    'transaction_id': 'isolated-%s-%s' % (len(calls), channel),
                    'tx_timestamp': now - wire.get('age', 0),
                    'receive_timestamp': now - wire.get('age', 0), 'outcome': 'query_response'}}
        monkeypatch.setattr(driver, '_send_pipette_command', exchange)
        transports.append(CanPipetteTransport(driver_factory=lambda driver=driver: driver, pipette_id=channel))
    transport = FourPipetteTransport(transports)
    monkeypatch.setattr(api, '_get_pipette_transport', lambda: transport)
    # Register query before the real control-plane catalog/dispatch is built.
    # Reuse retained hardware infrastructure; no audit/executor stand-ins.
    app.state.operator_command_plane.stop()
    from fastapi import FastAPI
    from bioxp.oem_compat.position_table import load_bound_oem_position_table
    app = FastAPI()
    app.add_api_route('/liquid/tip-status', api.liquid_tip_status, methods=['POST'])
    operator_controls.install_operator_control_plane(app,
        maintenance_state_provider=lambda: {'motion_blocked': False, 'recovery_required': False, 'block_reason': None},
        reference_state_provider=lambda: references.snapshot(('x','y','z','g')),
        lifecycle_state_provider=lambda: {'operation_state': 'stopped'},
        serial206_initialization_state_provider=api.serial206_oem_initialization_provider_status,
        oem_deck_provider=lambda: provider, oem_deck_position_table_provider=load_bound_oem_position_table)
    monkeypatch.setattr(api, 'app', app)
    app.middleware('http')(api.bind_direct_pipette_idempotency)
    yield app, provider, primitive, references, root, receipt_store, calls, wire, transport
    app.state.operator_command_plane.stop()


def query(rig, key='tip-query'):
    response = TestClient(rig[0]).post('/liquid/tip-status', headers={'Idempotency-Key': key})
    assert response.status_code == 200, response.text
    return response.json()


def test_real_query_publishes_nonzero_owner_and_full_no_tip_park(query_rig):
    from bioxp import api
    app, provider, primitive, references, root, receipts, calls, wire, transport = query_rig
    assert catalog_action(app)['enabled'] is False
    qualify_test_references(references)
    api._collect_and_publish_hardware_snapshot(['axes','latch'], reason='isolated-before-query')
    store = app.state.operator_command_plane.store
    action = catalog_action(app)
    client = TestClient(app)
    admitted = client.post('/operator/v2/actions/oem.deck.move_to_location', json={
        'schema_version': 'bioxp.operator_action_request.v2', 'idempotency_key': 'prior-named-move',
        'expected_ownership_generation': int(provider.generation_provider()),
        'expected_board_epoch_by_board': action['expected_board_epoch_by_board'],
        'inputs': {'target': 'LOC_OC', 'camera_offset': False}})
    assert admitted.status_code == 200, admitted.text
    app.state.operator_command_plane.start()
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        move = client.get('/operator/v2/actions/receipts/' + admitted.json()['command_id']).json()
        if move['status'] not in ('queued','dispatched','issued_pending'):
            break
        time.sleep(.01)
    assert move['status'] == 'completed', move
    before = store.deck_semantic_state()
    assert before['current_location'] == 'LOC_OC' and before['semantic_state_revision'] > 0
    assert before['tip_loaded'] is None
    result = query(query_rig)
    assert calls == [0,1,2,3]
    assert result['hardware_query_verified'] and result['semantic_query_response_verified']
    assert result.get('deck_state_publication', {}).get('status') == 'published', result
    after = store.deck_semantic_state()
    assert (after['tip_loaded'], after['tip_dirty'], after['tip_location']) == (False, False, -1)
    assert after['semantic_state_revision'] == before['semantic_state_revision'] + 1
    for name in ('current_location','current_well','ambiguity_state','clean_path','movable_plate_locations'):
        assert after[name] == before[name]
    assert after['transition_provenance']['upstream_source_command_id'] == result['command_id']
    assert not provider._deck_authority_scoped_cache
    for field in ('delivery_verified','controller_acknowledged','completion_verified','physical_effect_verified'):
        assert result[field] is False
    # Actual partial canonical row remains partial; only unused CleanPath is N/A.
    park = provider.deck_authority_snapshot(expected_generation=int(provider.generation_provider()), target='LOC_PARK')
    assert park['dependency_scope'] == 'full'
    assert park['clean_path'] is None and 'clean_path' not in park['required_facts']
    provider._deck_execution_semantics(park, no_tip_park=True)
    api._collect_and_publish_hardware_snapshot(['axes','latch'], reason='isolated-after-query')
    action = catalog_action(app)
    assert next(row for row in action['destination_options'] if row['target'] == 'LOC_PARK')['enabled'], action
    with pytest.raises(RuntimeError, match='clean_path'):
        provider.deck_authority_snapshot(expected_generation=int(provider.generation_provider()))
    replay = query(query_rig)
    assert replay['replayed'] and calls == [0,1,2,3]
    assert store.deck_semantic_state() == after
    import subprocess, sys
    script = ('import json,sys; from tests.test_deck_tip_query_publication import reopen; '
              'print(json.dumps(reopen(sys.argv[1],sys.argv[2],int(sys.argv[3]))))')
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', script, str(root),
        result['receipt_id'], str(provider.generation_provider())], text=True))
    assert reopened['semantic'] == after
    from bioxp.critical_logging import critical_receipt
    # Existing approved storage strips wire bytes, not Boolean query evidence,
    # correlation, timestamps or source identity. Do not weaken that policy.
    assert reopened['receipt']['result']['channels'] == critical_receipt(result['channels'])
    assert reopened['park_blocker']['clean_path'] is None
    assert reopened['park_blocker']['current_location_id'] == 'LOC_OC'
    assert reopened['park_blocker']['tip_loaded'] is False
    if os.environ.get('DECK_TEST_OUTPUT'):
        Path(os.environ['DECK_TEST_OUTPUT'] + '.tip.json').write_text(json.dumps(
            {'before': before, 'query': result, 'after': after, 'park_authority': park}, indent=2))


@pytest.mark.parametrize('fault', ['malformed','missing','uncorrelated','loaded_unknown','mixed_unknown',
    'failure','generation','board','publication','stale_response','reader_drift','interrupt'])
def test_real_query_negative_controls(query_rig, monkeypatch, fault):
    app, provider, primitive, references, root, receipts, calls, wire, transport = query_rig
    store = app.state.operator_command_plane.store
    before = store.deck_semantic_state()
    if fault == 'malformed': wire['data'][2] = [32,96,50]
    if fault == 'missing': wire['missing'] = True
    if fault == 'uncorrelated': wire['correlated'] = False
    if fault == 'loaded_unknown': wire['data'] = [[32,96,49] for _ in range(4)]
    if fault == 'mixed_unknown': wire['data'][2] = [32,96,49]
    if fault == 'failure': wire['failure'] = True
    if fault == 'stale_response': wire['age'] = 10
    if fault == 'reader_drift': wire['reader_drift'] = True
    if fault == 'interrupt':
        wire['during'] = lambda channel: monkeypatch.setattr(transport, '_interrupt_epoch', 1)
    if fault == 'generation':
        generation = provider.generation_provider()
        wire['during'] = lambda channel: monkeypatch.setattr(provider, 'generation_provider', lambda: generation + 1)
    if fault == 'board':
        def drift(channel):
            if channel == 3:
                state = provider._load_state()
                state['x_lifecycle']['board_lifecycle_generation'] += 1
                provider._save_state(state)
        wire['during'] = drift
    if fault == 'publication':
        monkeypatch.setattr(provider, '_deck_semantic_state_publisher', lambda **kw: (_ for _ in ()).throw(RuntimeError('isolated publisher failure')))
    response = TestClient(app).post('/liquid/tip-status', headers={'Idempotency-Key': 'negative-'+fault})
    if fault in ('missing','malformed','failure'):
        assert response.status_code != 200
    else:
        assert response.status_code == 200, response.text
        result = response.json()
        expected_status = 'published' if fault in ('loaded_unknown','mixed_unknown') else 'blocked'
        assert result.get('deck_state_publication', {}).get('status') == expected_status, result.get('deck_state_publication')
        assert result['receipt_id'] and result['command_id']
    after = store.deck_semantic_state()
    if fault in ('loaded_unknown','mixed_unknown'):
        assert after['tip_loaded'] is True
        assert after['tip_dirty'] is None and after['tip_location'] is None
        for key in ('ambiguity_state','current_location','current_well'):
            assert after[key] == before[key]
    else:
        assert after == before
    assert len(calls) <= 4


def named_move(rig, *, fail=False):
    from bioxp import api
    app, provider, primitive, references = rig[:4]
    catalog_action(app)
    qualify_test_references(references)
    api._collect_and_publish_hardware_snapshot(['axes','latch'], reason='isolated-named-predecessor')
    action = catalog_action(app)
    client = TestClient(app)
    if fail:
        def failed_move(*args, **kwargs):
            raise RuntimeError('isolated controller completion unavailable')
        primitive.oem_move_to = failed_move
    response = client.post('/operator/v2/actions/oem.deck.move_to_location', json={
        'schema_version': 'bioxp.operator_action_request.v2', 'idempotency_key': 'predecessor-move',
        'expected_ownership_generation': int(provider.generation_provider()),
        'expected_board_epoch_by_board': action['expected_board_epoch_by_board'],
        'inputs': {'target': 'LOC_OC', 'camera_offset': False}})
    assert response.status_code == 200, response.text
    app.state.operator_command_plane.start()
    path = '/operator/v2/actions/receipts/' + response.json()['command_id']
    result = {}
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        result = client.get(path).json()
        if result['status'] not in ('queued','dispatched','issued_pending'):
            break
        time.sleep(.01)
    assert result['status'] == ('ambiguous' if fail else 'completed'), result
    return path, client.get(path + '?detail=true').json()


def reopen(root, receipt_id, generation):
    from bioxp.pipette.receipts import PipetteReceiptStore
    from bioxp.oem_runtime_store import OEMRuntimeStore
    from bioxp.operator_command_plane import OperatorCommandStore
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider
    from bioxp.services.reference_service import ReferenceStateStore
    from tests.test_deck_scoped_authority import Primitive
    from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot
    with pytest.MonkeyPatch.context() as mp:
        bind_serial206_oem_snapshot(mp)
        runtime = OEMRuntimeStore(root)
        store = OperatorCommandStore(root)
        references = ReferenceStateStore(Path(root) / 'bioxp_runtime.db')
        provider = Serial206OemInitializationProvider(Primitive(home=True), state_store=runtime,
            reference_store=references, generation_provider=lambda: generation)
        provider.bind_deck_semantic_state_reader(store.deck_semantic_state)
        rows = PipetteReceiptStore(root).read(limit=100)
        receipt = next(row for row in rows if row['receipt_id'] == receipt_id)
        try:
            park = provider.deck_authority_snapshot(expected_generation=generation, target='LOC_PARK')
        except RuntimeError as exc:
            park = str(exc)
        result = {'receipt': receipt, 'semantic': store.deck_semantic_state(), 'park_blocker': park}
        store.stop()
        runtime.close()
        return result


@pytest.mark.parametrize('fault', ['stale','source_identity','duplicate_channel','non_boolean','replayed',
    'duplicate_transaction','receipt_identity','source_channel','missing_channels'])
def test_observation_owner_rejects_invalid_publication(query_rig, fault):
    app, provider, _, _, _, _, _, _, _ = query_rig
    result = query(query_rig)
    observed = copy.deepcopy(result)
    if fault == 'stale': observed['channels'][0]['result']['observed_at'] -= 10
    if fault == 'source_identity': observed['source_identity']['authority_verified'] = False
    if fault == 'duplicate_channel': observed['channels'][1]['channel'] = 0
    if fault == 'non_boolean': observed['channels'][0]['tip_loaded'] = 0
    if fault == 'replayed': observed['replayed'] = True
    if fault == 'receipt_identity': observed['receipt_id'] = ''
    if fault == 'missing_channels': observed['channels'].pop()
    if fault == 'source_channel': observed['channels'][1]['result']['provenance']['channel'] = 0
    if fault == 'duplicate_transaction':
        observed['channels'][1]['result']['provenance']['transaction_id'] = observed['channels'][0]['result']['provenance']['transaction_id']
    store = app.state.operator_command_plane.store
    before = store.deck_semantic_state()
    with pytest.raises(RuntimeError):
        provider.publish_pipette_query_observation(observed,
            expected_authority=provider.deck_owner_authority_stamps(), query_started_at=time.monotonic()-5)
    assert store.deck_semantic_state() == before


def test_loaded_after_no_tip_does_not_inherit_no_tip_ancillary_facts(query_rig):
    app, provider, _, _, _, _, calls, wire, _ = query_rig
    named_move(query_rig)
    first = query(query_rig)
    assert first.get('deck_state_publication', {}).get('status') == 'published'
    wire['data'] = [[32,96,49] for _ in range(4)]
    loaded = query(query_rig, key='now-loaded')
    assert calls == [0,1,2,3,0,1,2,3]
    semantic = app.state.operator_command_plane.store.deck_semantic_state()
    # Absence-derived condition/location are not facts about newly loaded tips.
    assert not (semantic['tip_loaded'] is True and semantic['tip_dirty'] is False
                and semantic['tip_location'] == -1), (loaded, semantic)
    assert semantic['tip_loaded'] is True
    assert semantic['tip_dirty'] is None and semantic['tip_location'] is None
    with pytest.raises(RuntimeError, match='deck_semantic_state_not_authoritative:tip_dirty'):
        provider.deck_authority_snapshot(expected_generation=int(provider.generation_provider()), target='LOC_PARK')


@pytest.mark.parametrize('loaded', [False, True])
def test_query_keeps_real_failed_move_ambiguity_and_history(query_rig, loaded):
    app, provider, _, _, _, _, _, wire, _ = query_rig
    path, failed_receipt = named_move(query_rig, fail=True)
    store = app.state.operator_command_plane.store
    before = store.deck_semantic_state()
    assert failed_receipt['deck_movement']['ambiguity_state'] == 'recovery_required'
    recovery_blocker = store.deck_recovery_blocker()
    assert recovery_blocker is not None
    if loaded:
        wire['data'] = [[32,96,49] for _ in range(4)]
    result = query(query_rig)
    assert result.get('deck_state_publication', {}).get('status') == 'published', result
    after = store.deck_semantic_state()
    assert after['tip_loaded'] is loaded
    assert after['ambiguity_state'] == before['ambiguity_state']
    for key in ('current_location','current_well','current_tray','clean_path','movable_plate_locations'):
        assert after[key] == before[key]
    assert TestClient(app).get(path + '?detail=true').json() == failed_receipt
    assert store.deck_recovery_blocker() == recovery_blocker
    with pytest.raises(RuntimeError):
        provider.deck_authority_snapshot(expected_generation=int(provider.generation_provider()), target='LOC_PARK')


def test_query_handoff_precedes_competing_canonical_pipette_owner(query_rig):
    import concurrent.futures
    import threading
    app, provider, _, _, _, _, calls, wire, _ = query_rig
    entered, release, competing = threading.Event(), threading.Event(), threading.Event()
    def pause(channel):
        if channel == 0:
            entered.set()
            assert release.wait(3)
    wire['during'] = pause
    def later_owner():
        competing.set()
        # A distinct authoritative pipette mutation, not a simulated motor ACK.
        return provider.publish_pipette_owner_state(tip_loaded=True, tip_dirty=True,
            tip_location=2, source_command_id='isolated-later-pipette-owner')
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        pending_query = pool.submit(query, query_rig)
        assert entered.wait(3)
        pending_owner = pool.submit(later_owner)
        assert competing.wait(3)
        assert not pending_owner.done()
        release.set()
        result = pending_query.result(timeout=5)
        later = pending_owner.result(timeout=5)
    assert calls == [0,1,2,3]
    assert result.get('deck_state_publication', {}).get('status') == 'published'
    assert later['semantic_state_revision'] > result['deck_state_publication']['semantic_state_revision']
    final = app.state.operator_command_plane.store.deck_semantic_state()
    assert final['tip_loaded'] is True and final['tip_dirty'] is True and final['tip_location'] == 2
    assert final['transition_provenance']['upstream_source_command_id'] == 'isolated-later-pipette-owner'


def test_known_loaded_owner_facts_are_preserved(query_rig):
    app, provider, _, _, _, _, _, wire, _ = query_rig
    provider.publish_pipette_owner_state(tip_loaded=True, tip_dirty=True,
        tip_location=2, source_command_id='isolated-known-loaded-owner')
    wire['data'] = [[32,96,49] for _ in range(4)]
    result = query(query_rig)
    assert result.get('deck_state_publication', {}).get('status') == 'published', result.get('deck_state_publication')
    semantic = app.state.operator_command_plane.store.deck_semantic_state()
    assert semantic['tip_loaded'] is True and semantic['tip_dirty'] is True and semantic['tip_location'] == 2


@pytest.mark.parametrize('updates', [
    {'tip_loaded': 0}, {'tip_loaded': 'false'}, {'tip_dirty': 0},
    {'tip_dirty': 'unknown'}, {'tip_location': False}, {'tip_location': 4},
])
def test_canonical_owner_keeps_exact_known_field_types(query_rig, updates):
    app, provider = query_rig[:2]
    store = app.state.operator_command_plane.store
    before = store.deck_semantic_state()
    with pytest.raises(ValueError):
        store.publish_deck_owner_state(source_operation='pipette_owner',
            source_command_id='invalid-known-field', updates=updates,
            **provider.deck_owner_authority_stamps())
    assert store.deck_semantic_state() == before


def test_partial_contradictory_query_is_not_promoted(query_rig):
    app, provider, _, _, _, _, calls, wire, _ = query_rig
    named_move(query_rig)
    query(query_rig)
    store = app.state.operator_command_plane.store
    before = store.deck_semantic_state()
    wire['data'][0] = [32,96,49]
    wire['data'][1] = [32,96,50]  # positive first channel, malformed next channel
    response = TestClient(app).post('/liquid/tip-status', headers={'Idempotency-Key': 'partial-contradiction'})
    assert response.status_code != 200
    assert calls == [0,1,2,3,0,1,2,3]  # non-'1' returns2 and continues
    after = store.deck_semantic_state()
    assert after['tip_loaded'] is True and after['tip_dirty'] is None and after['tip_location'] is None
    assert after['current_location'] == before['current_location']
    with pytest.raises(RuntimeError, match='deck_semantic_state_not_authoritative:tip_dirty'):
        provider.deck_authority_snapshot(expected_generation=int(provider.generation_provider()), target='LOC_PARK')
