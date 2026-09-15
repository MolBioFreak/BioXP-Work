"""Native paired Home completion → existing governed recovery, offline leaves only."""
import copy
import json
import subprocess
import sys

import pytest
from fastapi.testclient import TestClient
from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from tests.test_deck_home_recovery import (
    retained_rig, installed_retained, native_routes, stopped_failure,
    homed_replacement, native_home, home_body, raw_command,
)


@pytest.fixture(autouse=True)
def canonical_xy_route(native_routes, monkeypatch):
    from bioxp import api, operator_controls as controls
    install = controls.install_operator_control_plane
    def with_xy(app, **kwargs):
        app.add_api_route('/motion/oem/home_xy', api.motion_oem_home_xy, methods=['POST'])
        return install(app, **kwargs)
    monkeypatch.setattr(controls, 'install_operator_control_plane', with_xy)


@pytest.fixture
def paired_home(homed_replacement, monkeypatch):
    app, provider, primitive, refs, root, leaf, data = homed_replacement
    adapter = Serial206ProductionPrimitiveAdapter(leaf, None, authority_provider=lambda: {},
        generation_provider=provider.generation_provider, reference_store=refs)
    monkeypatch.setattr(primitive, 'home_xy', adapter.home_xy, raising=False)
    calls = []
    native = {}
    def home(axis, **kwargs):
        calls.append((axis, kwargs))
        result = native_home()
        result.update(axis=axis, source_return_code=-30 if axis == 'x' else 7)
        # A realistic populated earlier child must not consume Y's outcome.
        result['home_decision'] = {'observations': [{'ack': {'status':100}, 'value': i}
                                                   for i in range(180)]}
        native[axis] = copy.deepcopy(result)
        return result
    leaf.motor_oem_go_home = home
    data.update(paired_calls=calls, paired_native=native)
    return app, provider, primitive, refs, root, leaf, data


def run_home(rig, key='paired-home'):
    provider = rig[1]
    return provider.execute_homexy_intent({'command_id': key, 'idempotency_key': key,
                                          'expected_generation': provider.generation_provider()})


def reconcile(rig):
    app, provider, _, _, _, _, data = rig
    return TestClient(app).post('/operator/recovery/deck/'+data['command_id']+'/reconcile',
                               json=home_body(provider))


def test_paired_native_home_reaches_existing_recovery_without_another_home(paired_home):
    app, provider, primitive, refs, root, leaf, data = paired_home
    store = app.state.operator_command_plane.store
    before = raw_command(store, data['command_id'])
    outcome = run_home(paired_home)
    assert outcome['ok'] is True, outcome
    # OEM return semantics, including nonzero offsets, remain unchanged.
    assert outcome['result']['source_return'] == {'x': -30, 'y': 7}
    assert outcome['result']['home'] == data['paired_native']
    assert all(row['position_after_sethome'] is None for row in data['paired_native'].values())
    assert outcome['state'] == 'prepared_unreferenced'  # no invented generic reference promotion
    calls = copy.deepcopy(data['paired_calls'])
    recovered = reconcile(paired_home)
    assert recovered.status_code == 200, recovered.text
    assert data['paired_calls'] == calls
    assert sorted(a for a, _ in calls) == ['x', 'y']
    assert leaf.moves == []
    assert raw_command(store, data['command_id']) == before
    assert store.deck_recovery_blocker() is None
    semantic = store.deck_semantic_state()
    assert semantic['current_location'] is None and semantic['current_well'] is None
    for axis in ('x', 'y'):
        receipt = provider.state_store.read_serial206_receipt(axis, 'paired-home')
        assert receipt['recovery_home']['command_id'] == 'paired-home'
        assert receipt['recovery_home']['reference_version'] == refs.snapshot((axis,))['rows'][axis]['state_version']
    code = ('import json;from bioxp.oem_runtime_store import OEMRuntimeStore;'
            's=OEMRuntimeStore('+repr(str(root))+');'
            'print(json.dumps({a:s.read_serial206_receipt(a,"paired-home") for a in ("x","y")}));s.close()')
    persisted = json.loads(subprocess.check_output([sys.executable, '-c', code], text=True))
    assert persisted['x']['recovery_home'] != persisted['y']['recovery_home']
    assert persisted['x']['result']['home']['y']['source_return_code'] == 7
    calls = copy.deepcopy(data['paired_calls'])
    replayed = run_home(paired_home)
    assert replayed['replayed'] is True and replayed['ok'] is True
    assert calls == data['paired_calls']


@pytest.mark.parametrize('fault', ['x_unverified', 'y_unverified', 'y_missing', 'y_cached_noop', 'x_exception'])
def test_bad_paired_home_does_not_create_recovery_authority(paired_home, fault):
    app, provider, primitive, refs, root, leaf, data = paired_home
    original = leaf.motor_oem_go_home
    def bad(axis, **kw):
        if fault == 'x_exception' and axis == 'x':
            raise RuntimeError('isolated native home failure')
        value = original(axis, **kw)
        if fault == axis+'_unverified': value['controller_home_proof_verified'] = False
        if axis == 'y' and fault == 'y_missing': return {'ok':True, 'source_return_code':0}
        if axis == 'y' and fault == 'y_cached_noop':
            return {'ok':True, 'source_noop':True, 'source_return_code':0,
                    'home_decision':{'source_short_circuit':'MotorHome_and_CurrentPosition_zero'}}
        return value
    leaf.motor_oem_go_home = bad
    run_home(paired_home)
    before = raw_command(app.state.operator_command_plane.store, data['command_id'])
    response = reconcile(paired_home)
    assert response.status_code == 409, response.text
    assert raw_command(app.state.operator_command_plane.store, data['command_id']) == before
    assert app.state.operator_command_plane.store.deck_recovery_blocker() == 'deck_recovery_hold'
    assert leaf.moves == []


def test_paired_home_route_has_no_lifecycle_false_block(paired_home):
    assert run_home(paired_home)['ok'] is True
    response = reconcile(paired_home)
    assert response.status_code == 200, response.text


def test_clipped_child_is_not_reconstructed_from_parent_true_flags(paired_home, monkeypatch):
    app, provider, primitive, refs, root, leaf, data = paired_home
    original = primitive.home_xy
    def clipped():
        result = original()
        # Reproduce the retained record's real truncation shape: the parent
        # says verified, but Y's required child proof has been discarded.
        result['home']['y'] = {'omitted': 'item_limit'}
        return result
    monkeypatch.setattr(primitive, 'home_xy', clipped)
    result = run_home(paired_home)
    assert result['result']['controller_home_proof_verified'] is True
    assert 'recovery_home' not in result['authority_receipt']
    assert reconcile(paired_home).status_code == 409


def test_native_home_has_no_new_physical_queries_or_reference_precondition(paired_home, monkeypatch):
    app, provider, primitive, refs, root, leaf, data = paired_home
    def extra(*a, **kw):
        raise AssertionError('Home handoff must not add a physical readback')
    monkeypatch.setattr(leaf, 'motor_get_position', extra)
    monkeypatch.setattr(leaf, 'motor_get_speed', extra)
    monkeypatch.setattr(leaf, 'motor_query_home_switch', extra)
    provider.reference_store = None
    result = run_home(paired_home)
    assert result['ok'] is True, result
    assert result['result']['source_return'] == {'x': -30, 'y': 7}
    assert 'recovery_home' not in result['authority_receipt']
    assert sorted(a for a, _ in data['paired_calls']) == ['x', 'y']


def test_canonical_homexy_dispatch_recovery_and_warm_consumers(paired_home):
    import time
    app, provider, primitive, refs, root, leaf, data = paired_home
    store = app.state.operator_command_plane.store
    before = raw_command(store, data['command_id'])
    with TestClient(app) as client:
        response = client.post('/operator/v2/actions/oem.xy.home', json={
            'schema_version':'bioxp.operator_action_request.v2',
            'idempotency_key':'canonical-paired-home',
            'expected_ownership_generation':provider.generation_provider(),
            'expected_board_epoch_by_board':{}, 'inputs':{}})
        assert response.status_code == 200, response.text
        command = response.json()['command_id']
        deadline = time.monotonic() + 10
        detail = {'status': 'not_observed'}
        while time.monotonic() < deadline:
            detail = client.get('/operator/v2/actions/receipts/'+command+'?detail=true').json()
            if detail['status'] not in {'queued','running','dispatched','issued_pending'}:
                break
            time.sleep(.01)
        assert detail['status'] == 'completed', json.dumps(detail)
        calls = copy.deepcopy(data['paired_calls'])
        recovered = client.post('/operator/recovery/deck/'+data['command_id']+'/reconcile',
                                json=home_body(provider))
        assert recovered.status_code == 200, recovered.text
        for _ in range(2):
            catalog = client.get('/operator/v2/control-catalog')
            dashboard = client.get('/operator/v2/dashboard')
            assert catalog.status_code == dashboard.status_code == 200
            assert 'deck_recovery_hold' not in catalog.text
        assert raw_command(store, data['command_id']) == before
        assert data['paired_calls'] == calls and leaf.moves == []


def test_paired_child_recording_failure_and_replay(paired_home, monkeypatch):
    app, provider, primitive, refs, root, leaf, data = paired_home
    append = provider.state_store.append_serial206_receipts_atomic
    def fail(_rows):
        raise OSError('isolated paired receipt recording failure')
    monkeypatch.setattr(provider.state_store, 'append_serial206_receipts_atomic', fail)
    result = run_home(paired_home)
    assert result['ok'] is False
    assert 'receipt_publication_exception' in result['failure']
    assert provider.state_store.read_serial206_receipt('y', 'paired-home') is None
    calls = copy.deepcopy(data['paired_calls'])
    assert reconcile(paired_home).status_code == 409
    monkeypatch.setattr(provider.state_store, 'append_serial206_receipts_atomic', append)
    replay = run_home(paired_home)
    assert replay['ok'] is True and replay['replayed'] is True
    assert data['paired_calls'] == calls
    assert reconcile(paired_home).status_code == 200


@pytest.mark.parametrize('fault', ['x_owner', 'y_owner', 'x_reference', 'y_reference',
    'z_reference', 'x_interrupt', 'y_interrupt', 'generation', 'moving', 'not_home',
    'home_reply', 'position_ack', 'final_reference'])
def test_paired_recovery_preserves_current_authority_checks(paired_home, monkeypatch, fault):
    from bioxp.services.reference_service import MarkAxisDesyncedCommand
    app, provider, primitive, refs, root, leaf, data = paired_home
    assert run_home(paired_home)['ok'] is True
    calls = copy.deepcopy(data['paired_calls'])
    before = raw_command(app.state.operator_command_plane.store, data['command_id'])
    if fault == 'x_owner': provider._home_recovery_owner_id = 'replacement-without-home'
    elif fault == 'y_owner': provider.y_provider._home_recovery_owner_id = 'replacement-without-home'
    elif fault.endswith('_reference') and fault != 'final_reference':
        refs.mark_desynced(MarkAxisDesyncedCommand(fault[0], reason='isolated current reference fault'))
    elif fault == 'x_interrupt': provider._x_interrupt_epoch += 1
    elif fault == 'y_interrupt':
        read = provider.state_store.axis_interrupt_snapshot
        monkeypatch.setattr(provider.state_store, 'axis_interrupt_snapshot',
                            lambda axis: {**read(axis), 'active': True})
    elif fault == 'generation': monkeypatch.setattr(provider, 'generation_provider', lambda:999)
    elif fault == 'moving': leaf.motor_get_speed = lambda *a, **kw: {'ok':True, 'speed':1, 'ack':{'status':100}}
    elif fault == 'not_home': leaf.positions[5,0] = 1
    elif fault == 'home_reply':
        leaf.motor_query_home_switch = lambda *a, **kw: {'ok':True, 'home':True, 'reply_valid':False, 'ack':{'status':100}}
    elif fault == 'position_ack':
        leaf.motor_get_position = lambda *a, **kw: {'ok':True, 'position':0, 'ack':{'status':2}}
    else:
        read = leaf.motor_get_speed
        counter = [0]
        def drift(*a, **kw):
            counter[0] += 1
            if counter[0] == 6:
                refs.mark_desynced(MarkAxisDesyncedCommand('x', reason='isolated final sample drift'))
            return read(*a, **kw)
        leaf.motor_get_speed = drift
    response = reconcile(paired_home)
    assert response.status_code == 409, response.text
    assert raw_command(app.state.operator_command_plane.store, data['command_id']) == before
    assert app.state.operator_command_plane.store.deck_recovery_blocker() == 'deck_recovery_hold'
    assert data['paired_calls'] == calls and leaf.moves == []
