"""Governed HOME: real routes, native home owners, references and SQLite.

Only hardware/preparation leaves are controlled; no recovery snapshot or
reconciliation/store authority is manufactured by the success fixture.
"""
import copy
import json
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_scoped_recovery import stopped_failure as _source_stopped_failure, body
from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter


@pytest.fixture(autouse=True)
def native_routes(monkeypatch, request):
    if request.node.name == 'test_actual_retained_failed_command':
        monkeypatch.setenv('DECK_RETAINED_BASELINE', '/home/dalab/.hermes/profiles/fresh/robot-audit/deck-harmonization-implementation/continued-recovery/state')
    from bioxp import api, operator_controls as controls
    install = controls.install_operator_control_plane
    def with_routes(app, **kwargs):
        app.add_api_route('/motion/oem/manual/home', api.motion_oem_manual_home, methods=['POST'])
        app.add_api_route('/motion/oem/x/manual_home', api.motion_oem_x_manual_home, methods=['POST'])
        app.add_api_route('/motion/oem/y/home', api.motion_oem_y_home, methods=['POST'])
        return install(app, **kwargs)
    monkeypatch.setattr(controls, 'install_operator_control_plane', with_routes)


@pytest.fixture
def stopped_failure(installed_retained, monkeypatch, tmp_path, request):
    if request.node.name != 'test_actual_retained_failed_command':
        return _source_stopped_failure.__wrapped__(installed_retained, monkeypatch, tmp_path)
    from tests.test_deck_postmove_reference import USBLeaf
    app, provider, primitive, refs, root = installed_retained
    command_id = '90b1befa-47e0-47aa-8c29-848cbebe950e'
    detail = app.state.operator_command_plane.store.get_command(command_id)
    assert detail['status'] == 'ambiguous'
    leaf = USBLeaf()
    adapter = Serial206ProductionPrimitiveAdapter(leaf, None, authority_provider=lambda:{},
        generation_provider=provider.generation_provider, reference_store=refs)
    monkeypatch.setattr(primitive, '_read_axis_position', adapter._read_axis_position)
    return app, provider, primitive, refs, root, leaf, {'command_id':command_id, 'detail':detail}


@pytest.mark.parametrize('fault', ['failed', 'cached_noop', 'invalid_ack', 'generation_drift', 'board_drift'])
def test_y_home_common_reference_not_published_on_bad_native_proof(homed_replacement, monkeypatch, fault):
    app, provider, primitive, refs, root, leaf, data = homed_replacement
    y = provider.y_provider
    original = y.tester.motor_oem_go_home
    before = refs.snapshot(('y',))['rows']['y']
    def home(*args, **kwargs):
        result = original(*args, **kwargs)
        if fault == 'failed': result['ok'] = False
        elif fault == 'cached_noop':
            result = {'ok':True, 'source_noop':True,
                'home_decision':{'source_short_circuit':'MotorHome_and_CurrentPosition_zero'}}
        elif fault == 'invalid_ack': result['home_hit']['ack']['status'] = 1
        elif fault == 'generation_drift':
            generation = y.generation_provider()
            monkeypatch.setattr(y, 'generation_provider', lambda:generation+1)
        elif fault == 'board_drift':
            advance = y.state_store.record_board4_transition(active=True, ack={'status':100}, transition_id='test-during-y-home', ownership_generation=y.generation_provider(), continuity_proven=False)
            assert advance is not None
        return result
    monkeypatch.setattr(y.tester, 'motor_oem_go_home', home)
    y.home('manual_panel', command_id='negative-'+fault)
    after = refs.snapshot(('y',))['rows']['y']
    assert after == before
    assert provider.state_store.read_serial206_receipt('y', 'negative-'+fault) is None


@pytest.mark.parametrize('fault', [None, 'position_ack', 'position_cache', 'nonzero', 'speed_ack', 'moving', 'read_error'])
def test_y_home_observes_zero_after_source_sethome(homed_replacement, monkeypatch, fault):
    app, provider, primitive, refs, root, leaf, data = homed_replacement
    y = provider.y_provider
    raw = native_home()
    # Exact native goHome contract: position_after precedes SAP1;
    # the driver deliberately does not produce a post-setHome sample.
    raw.update(position_after={'ok':True, 'position':-7, 'ack':{'status':100}},
        position_after_sethome=None, source_return_code=7)
    before = refs.snapshot(('y',))['rows']['y']
    calls = []
    def home(*args, **kwargs):
        calls.append('home')
        return copy.deepcopy(raw)
    def position(*args, **kwargs):
        calls.append('position')
        if fault == 'read_error': raise OSError('isolated unavailable readback')
        return {'ok':fault != 'position_cache', 'position':2 if fault == 'nonzero' else 0,
            'ack':{'status':1 if fault == 'position_ack' else 100}}
    def speed(*args, **kwargs):
        calls.append('speed')
        return {'ok':True, 'speed':3 if fault == 'moving' else 0,
            'ack':{'status':1 if fault == 'speed_ack' else 100}}
    monkeypatch.setattr(y.tester, 'motor_oem_go_home', home)
    monkeypatch.setattr(y.tester, 'motor_get_position', position)
    monkeypatch.setattr(y.tester, 'motor_get_speed', speed)
    result = y.home('manual_panel', command_id='post-source-readback-'+str(fault))
    assert result['ok'] is True  # native manual source reporting is unchanged
    assert result['result']['home'] == raw  # no invented source sample
    assert result['controller_home_proof_verified'] is (fault is None)
    assert result['reference_published'] is (fault is None)
    after = refs.snapshot(('y',))['rows']['y']
    if fault is None:
        assert after['state_version'] > before['state_version']
        assert calls == ['home','position','speed']
        assert y.state_store.read_serial206_receipt('y', 'post-source-readback-None')['recovery_home']
    else:
        assert after == before


def test_actual_retained_failed_command(homed_replacement):
    test_home_replacement_route(homed_replacement)


def home_body(provider):
    return {**body(provider), 'current_location': None, 'current_well': None,
        'decision_id': 'fresh-home-decision',
        'approved_home_state': {'state': 'serial206_xyz_referenced_home',
            'approval_id': 'explicit-home-approval', 'approved_by': 'isolated-operator'}}


def native_home():
    ack = {'status': 100}
    return {'ok': True, 'source_return_code': 0, 'source_noop': False,
        'controller_command_acknowledged': True, 'controller_terminal_state_verified': True,
        'controller_home_proof_verified': True,
        'move_home': {'ok': True, 'ack': ack},
        'home_hit': {'home': True, 'reply_valid': True, 'ack': ack},
        'stop': {'ok': True, 'first_delivery': ack, 'second_delivery': ack},
        'wait': {'stopped': True, 'speed_reply_valid': True,
            'controller_terminal_state_verified': True, 'last_ack': ack, 'last_speed': 0},
        'set_home': {'controller_command_acknowledged': True, 'ack': ack},
        # Match the actual driver's return: this offset is before SAP1;
        # post-zero evidence must come from the owner's separate readback.
        'position_after': {'ok': True, 'position': -7, 'ack': ack},
        'position_after_sethome': None}


@pytest.fixture
def homed_replacement(stopped_failure, monkeypatch, request):
    app, provider, primitive, refs, root, leaf, data = stopped_failure
    # New process owners over the same copied durable database. Only the
    # hardware leaf persists across managed replacement; no authority is copied.
    from fastapi import FastAPI
    from bioxp import api, operator_controls as controls
    from bioxp.oem_runtime_store import OEMRuntimeStore
    from bioxp.services.reference_service import ReferenceStateStore
    from bioxp.serial206_y_provider import Serial206YProvider
    from bioxp.oem_compat.position_table import load_bound_oem_position_table
    generation = int(provider.generation_provider()) + 1
    epoch5 = provider.deck_owner_authority_stamps()['board_epoch_5'] + 1
    app.state.operator_command_plane.stop()
    runtime = OEMRuntimeStore(root)
    refs = ReferenceStateStore(root / 'bioxp_runtime.db')
    provider = type(provider)(primitive, state_store=runtime, reference_store=refs,
        generation_provider=lambda: generation)
    provider.y_provider = Serial206YProvider(primitive, state_store=runtime,
        reference_store=refs, generation_provider=lambda:generation)
    monkeypatch.setattr(provider.y_provider, 'profile', lambda **kw: {})
    monkeypatch.setattr(api, '_serial206_oem_initialization_provider', provider)
    monkeypatch.setattr(api, '_serial206_y_provider', provider.y_provider)
    monkeypatch.setattr(type(controls.hardware_state), 'ownership_epoch', property(lambda self:generation))
    monkeypatch.setattr(controls.hardware_state, 'ownership_projection', lambda: {
        'ownership_epoch':generation, 'ownership':{'transport':'owned','usb':'service','router':'running','CAN_READY':True}})
    app = FastAPI()
    controls.install_operator_control_plane(app,
        maintenance_state_provider=lambda:{'motion_blocked':False,'recovery_required':False,'block_reason':None},
        reference_state_provider=lambda:refs.snapshot(('x','y','z','g')),
        lifecycle_state_provider=lambda:{'operation_state':'stopped'},
        serial206_initialization_state_provider=api.serial206_oem_initialization_provider_status,
        oem_deck_provider=lambda:provider, oem_deck_position_table_provider=load_bound_oem_position_table)
    monkeypatch.setattr(api, 'app', app)
    prep = SimpleNamespace(current_board_lifecycle_generation=lambda: epoch5,
        prepare_for_initialize_motors=lambda **kw: {'ok': True, 'physical_motion': False,
            'board_lifecycle_generation': epoch5, 'board_lifecycle_reused': True})
    provider.preparation_provider = prep
    monkeypatch.setattr(primitive, 'prepare_x', prep.prepare_for_initialize_motors, raising=False)
    assert provider.execute_x_intent('prepare', {'command_id': 'new-x-prepare'})['ok'] is True
    monkeypatch.setattr(primitive, 'z_clear_profile_overrides', lambda: None, raising=False)
    zprep = provider.execute_z_intent('prepare', expected_generation=generation,
        idempotency_key='new-z-prepare')
    assert zprep['ok'] is True, zprep
    y = provider.y_provider
    monkeypatch.setattr(y, 'profile', lambda **kw: {})
    assert y.prepare(command_id='new-y-prepare')['ok'] is True
    leaf.positions.update({(5,0): 0, (4,0): 0, (4,1): 0})
    leaf.motor_query_home_switch = lambda *a, **kw: {'ok': True, 'reply_valid': True,
        'home': True, 'ack': {'status':100}}
    adapter = Serial206ProductionPrimitiveAdapter(leaf, None,
        authority_provider=lambda:{}, generation_provider=lambda:generation, reference_store=refs)
    monkeypatch.setattr(primitive, 'tester', leaf)
    monkeypatch.setattr(primitive, 'x_manual_panel_home',
        lambda **kw: adapter._x_home_result(native_home(), intent='manual_panel_home', source_method='offline-controller'), raising=False)
    monkeypatch.setattr(primitive, 'z_manual_home', adapter.z_manual_home, raising=False)
    monkeypatch.setattr(y, 'tester', leaf)
    leaf.motor_oem_go_home = lambda *a, **kw: native_home()
    z = provider.execute_z_intent('manual_home', expected_generation=generation, idempotency_key='fresh-z-home')
    assert z['ok'] is True, z
    if getattr(request, 'param', None) == 'paired_only':
        # Board-invalidated references: no standalone X/Y Home may seed the
        # authority that the paired producer is responsible for establishing.
        from bioxp.services.reference_service import MarkAxisDesyncedCommand
        refs.mark_desynced_many([MarkAxisDesyncedCommand(a,
            reason='offline board-invalidation starting state') for a in ('x', 'y')])
        x = yr = None
    else:
        x = provider.execute_x_intent('manual_panel_home', {'command_id':'fresh-x-home'})
        assert x['ok'] is True, x
        provider._load_state()  # existing native X reference publication
        yr = y.home('manual_panel', command_id='fresh-y-home')
        assert yr['reference_published'] is True, yr
    data['home_results'] = {'x':x, 'y':yr, 'z':z}
    yield app, provider, primitive, refs, root, leaf, data
    app.state.operator_command_plane.stop()
    runtime.close()


@pytest.mark.parametrize('fault', ['missing', 'failed', 'old_home', 'future_home', 'unreferenced',
    'nonhome', 'moving', 'invalid_ack', 'invalid_home_reply', 'owner', 'y_owner',
    'final_reference', 'final_owner', 'final_epoch', 'final_interrupt', 'stale_sample'])
def test_home_recovery_rejects_faults(homed_replacement, monkeypatch, fault):
    from bioxp.services.reference_service import MarkAxisDesyncedCommand
    app, provider, primitive, refs, root, leaf, data = homed_replacement
    store = app.state.operator_command_plane.store
    before = raw_command(store, data['command_id'])
    semantic = store.deck_semantic_state()
    if fault in {'missing', 'failed', 'old_home', 'future_home'}:
        state = provider._load_state()
        receipt = next(r for r in reversed(state['x_lifecycle']['receipts']) if 'recovery_home' in r)
        if fault == 'missing': del receipt['recovery_home']
        elif fault == 'failed': receipt['status'] = 'failed'
        elif fault == 'old_home': receipt['recovery_home']['started_at'] = 1
        else: receipt['recovery_home']['finished_at'] = 1e20
        provider._save_state(state)
    elif fault == 'unreferenced': refs.mark_desynced(MarkAxisDesyncedCommand('z', reason='isolated fault'))
    elif fault == 'nonhome': leaf.positions[5,0] = 1
    elif fault == 'moving': leaf.motor_get_speed = lambda *a, **k: {'ok':True, 'speed':1, 'ack':{'status':100}}
    elif fault == 'invalid_ack': leaf.motor_get_speed = lambda *a, **k: {'ok':True, 'speed':0, 'ack':{'status':2}}
    elif fault == 'invalid_home_reply': leaf.motor_query_home_switch = lambda *a, **k: {'ok':True, 'home':True, 'reply_valid':False, 'ack':{'status':100}}
    elif fault == 'owner': provider._home_recovery_owner_id = 'replacement-without-home'
    elif fault == 'y_owner': provider.y_provider._home_recovery_owner_id = 'replacement-without-home'
    elif fault.startswith('final_'):
        read = leaf.motor_get_speed
        count = [0]
        def drift(*args, **kwargs):
            count[0] += 1
            # Last hardware speed read of the transaction's second sample.
            if count[0] == 6:
                if fault == 'final_reference': refs.mark_desynced(MarkAxisDesyncedCommand('x', reason='final sample drift'))
                elif fault == 'final_owner': provider._home_recovery_owner_id = 'other'
                elif fault == 'final_interrupt': provider._z_interrupt_epoch += 1
                else: monkeypatch.setattr(provider, 'generation_provider', lambda: 999)
            return read(*args, **kwargs)
        leaf.motor_get_speed = drift
    else:
        reader = provider.deck_home_reconciliation_snapshot
        def stale(**kw):
            value = reader(**kw)
            value['captured_at'] -= 60
            return value
        monkeypatch.setattr(provider, 'deck_home_reconciliation_snapshot', stale)
    response = TestClient(app).post('/operator/recovery/deck/'+data['command_id']+'/reconcile', json=home_body(provider))
    assert response.status_code == 409, response.text
    assert raw_command(store, data['command_id']) == before
    assert store.deck_semantic_state() == semantic
    assert store.deck_recovery_blocker() == 'deck_recovery_hold'
    assert leaf.moves == []


def test_home_recovery_decision_replay(homed_replacement):
    app, provider, primitive, refs, root, leaf, data = homed_replacement
    client = TestClient(app)
    path = '/operator/recovery/deck/'+data['command_id']+'/reconcile'
    request = home_body(provider)
    first = client.post(path, json=request)
    assert first.status_code == 200, first.text
    assert client.post(path, json=request).json() == first.json()
    changed = copy.deepcopy(request)
    changed['approved_home_state']['approval_id'] = 'different'
    assert client.post(path, json=changed).status_code == 409
    assert leaf.moves == []


def test_canonical_home_dispatch_under_hold(homed_replacement, monkeypatch):
    import time
    app, provider, primitive, refs, root, leaf, data = homed_replacement
    plane = app.state.operator_command_plane
    store = plane.store
    outcomes = []
    with TestClient(app) as client:
        for action in ('oem.z.manual_home', 'oem.x.manual_panel_home', 'oem.y.manual_panel_home'):
            response = client.post('/operator/v2/actions/'+action, json={
                'schema_version':'bioxp.operator_action_request.v2', 'idempotency_key':'held-'+action,
                'expected_ownership_generation':provider.generation_provider(),
                'expected_board_epoch_by_board':{}, 'inputs':{}})
            assert response.status_code == 200, response.text
            command_id = response.json()['command_id']
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                finished = client.get('/operator/v2/actions/receipts/'+command_id+'?detail=true').json()
                if finished['status'] not in {'queued','running','dispatched','issued_pending'}:
                    break
                time.sleep(.01)
            assert finished['status'] == 'completed', json.dumps(finished)
            outcomes.append(finished)
            provider._load_state()
            assert store.deck_recovery_blocker() == 'deck_recovery_hold'
            # V2 manual actions use native direct claims, not the method queue.
            assert store.claim_next() is None
        response = client.post('/operator/recovery/deck/'+data['command_id']+'/reconcile', json=home_body(provider))
        assert response.status_code == 200, response.text
        path = '/operator/v2/actions/receipts/' + data['command_id']
        compact = client.get(path).json()
        detail = client.get(path+'?detail=true').json()
        catalogs, dashboards = [], []
        for _ in range(2):
            catalog = client.get('/operator/v2/control-catalog')
            dashboard = client.get('/operator/v2/dashboard')
            assert catalog.status_code == dashboard.status_code == 200, (catalog.text, dashboard.text)
            catalogs.append(catalog.json()); dashboards.append(dashboard.json())
        code = 'import json; from tests.test_deck_scoped_integration import fresh_process_receipts; print(json.dumps(fresh_process_receipts('+repr(str(root))+','+repr(data['command_id'])+')))'
        fresh = json.loads(subprocess.check_output([sys.executable, '-c', code], text=True))
        if os.environ.get('DECK_TEST_OUTPUT'):
            Path(os.environ['DECK_TEST_OUTPUT']+'.warm-native.json').write_text(json.dumps({
                'compact':compact, 'detail':detail, 'fresh_process':fresh,
                'raw_get_command':store.get_command(data['command_id']),
                'catalog':catalogs[-1], 'dashboard':dashboards[-1],
                'catalog_cycles':catalogs, 'dashboard_cycles':dashboards,
                'home_receipts':outcomes, 'reconciliation':response.json()}, indent=2))
    if os.environ.get('DECK_TEST_OUTPUT'):
        Path(os.environ['DECK_TEST_OUTPUT']+'.dispatch.json').write_text(json.dumps(outcomes, indent=2))


def test_populated_z_home_authority_is_not_diagnostic(homed_replacement):
    app, provider, primitive, refs, root, leaf, data = homed_replacement
    receipt = next(r for r in reversed(provider._load_state()['z_lifecycle']['receipts']) if 'recovery_home' in r)
    populated = {'command_id': 'populated-proof', 'result': {'diagnostics': [{'raw':list(range(100))} for _ in range(1000)]}, **receipt}
    populated['result'] = {'diagnostics': [{'raw':list(range(100))} for _ in range(1000)]}
    bounded = provider._append_z_receipt({'receipts':[]}, populated)
    assert bounded['recovery_home'] == receipt['recovery_home']
    assert all(type(bounded['recovery_home'][k]) is int for k in ('ownership_generation','reference_version'))


def raw_command(store, command_id):
    tables = ('operator_plane_commands', 'operator_plane_deck_commands', 'serial206_movement_commands')
    return {t: dict(store.connection.execute('SELECT * FROM '+t+' WHERE command_id=?', (command_id,)).fetchone()) for t in tables}


def test_home_replacement_route(homed_replacement):
    app, provider, primitive, refs, root, leaf, data = homed_replacement
    store = app.state.operator_command_plane.store
    before = raw_command(store, data['command_id'])
    calls = copy.deepcopy(primitive.calls)
    response = TestClient(app).post('/operator/recovery/deck/'+data['command_id']+'/reconcile', json=home_body(provider))
    assert response.status_code == 200, response.text
    assert raw_command(store, data['command_id']) == before
    client = TestClient(app)
    path = '/operator/v2/actions/receipts/' + data['command_id']
    compact = client.get(path).json()
    detail = client.get(path+'?detail=true').json()
    expected_resolution = {'command_id': data['command_id'], 'decision_id':home_body(provider)['decision_id'],
        'semantic_state_revision':response.json()['semantic_state_revision'],
        'transition_sequence':response.json()['transition_sequence']}
    assert 'deck_movement' not in compact
    assert detail['deck_movement']['recovery_resolution'] == expected_resolution
    assert compact['status'] == detail['status'] == 'ambiguous'
    assert detail['deck_movement']['ambiguity_state'] == 'recovery_required'
    code = 'import json; from tests.test_deck_scoped_integration import fresh_process_receipts; print(json.dumps(fresh_process_receipts('+repr(str(root))+','+repr(data['command_id'])+')))'
    fresh_receipts = json.loads(subprocess.check_output([sys.executable, '-c', code], text=True))
    assert 'deck_movement' not in fresh_receipts['compact']
    assert fresh_receipts['detail']['deck_movement']['recovery_resolution'] == expected_resolution
    if os.environ.get('DECK_TEST_OUTPUT'):
        Path(os.environ['DECK_TEST_OUTPUT']+'.recovered-receipts.json').write_text(json.dumps({
            'compact':compact, 'detail':detail, 'fresh_process':fresh_receipts}, indent=2))
    semantic = store.deck_semantic_state()
    assert semantic['current_location'] is None and semantic['current_well'] is None
    assert semantic['tip_dirty'] is None and semantic['plate_on_gantry'] is None
    assert semantic['ambiguity_state'] == 'none'
    assert store.deck_recovery_blocker() is None
    assert leaf.moves == []
    assert not any(row[0]=='move' for row in primitive.calls[len(calls):])
    code = 'import json; from bioxp.operator_command_plane import OperatorCommandStore; s=OperatorCommandStore('+repr(str(root))+'); print(json.dumps(s.deck_semantic_state())); s.stop()'
    fresh = json.loads(subprocess.check_output([sys.executable, '-c', code], text=True))
    assert fresh == semantic
    # Ordinary scoped eligibility still uses its native producer; unknown full
    # predecessor fields were not filled to permit home reconciliation.
    snap = provider.deck_authority_snapshot(expected_generation=provider.generation_provider(), target='LOC_OC')
    assert snap['tip_dirty'] is None and snap['current_location_id'] is None
    if os.environ.get('DECK_TEST_OUTPUT'):
        Path(os.environ['DECK_TEST_OUTPUT']+'.home.json').write_text(json.dumps({
            'historical':before, 'home_results':data['home_results'], 'decision':response.json(),
            'semantic':semantic, 'next_authority':snap}, indent=2))
