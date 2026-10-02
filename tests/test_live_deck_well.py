"""Public well adapter through installed API, canonical FIFO and native provider.

Offline transport only; no robot requests. Exports are actual producer bodies.
"""
import json
import os
from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained, catalog_payload
from tests.test_deck_complete_admission import ready
from tests.test_deck_automatic_refresh_owner import request, finish

ACTION = 'oem.deck.move_to_well'
URL = '/operator/v2/actions/' + ACTION


def export(name, payload):
    root = os.environ.get('LIVE_DECK_EXPORT')
    if root:
        path = Path(root)
        path.mkdir(parents=True, exist_ok=True)
        (path / (name + '.json')).write_text(json.dumps(payload, indent=2))


def body(provider, key, location=3, well='A1', flag=1):
    value = request(provider, key)
    value['inputs'] = {'location_id': location, 'well': well, 'position_flag': flag}
    return value


@pytest.fixture
def connected(installed_retained, retained_rig, monkeypatch):
    from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot
    bind_serial206_oem_snapshot(monkeypatch)
    from types import SimpleNamespace
    monkeypatch.setattr('bioxp.oem_serial206_initialization.load_oem_parity_config',
        lambda _: SimpleNamespace(blockers=[], values={'GripperVersion': 1}))
    app, provider, primitive, references, root = installed_retained
    leaf, raw = ready(installed_retained, monkeypatch, retained_rig)
    from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
    from bioxp.serial206_y_provider import Serial206YProvider
    adapter = Serial206ProductionPrimitiveAdapter(leaf, None,
        authority_provider=lambda: {}, generation_provider=provider.generation_provider,
        reference_store=references)
    adapter.y_provider = Serial206YProvider(leaf, state_store=retained_rig[2],
        generation_provider=provider.generation_provider, reference_store=references)
    monkeypatch.setattr(leaf, 'motor_query_home_switch', primitive.motor_query_home_switch, raising=False)
    monkeypatch.setattr(leaf, 'motor_wait_target_reached_many', lambda axes, **kwargs: {
        'ok': True, 'per_axis': {a: leaf.motor_wait_target_reached(b)
            for a, b in [('x', 5), ('y', 4)]}})
    scripts = []
    def script(**kwargs):
        result = adapter.oem_scriptmove_to(**kwargs)
        scripts.append((kwargs, result))
        return result
    monkeypatch.setattr(primitive, 'oem_scriptmove_to', script, raising=False)
    client = TestClient(app)
    store = app.state.operator_command_plane.store
    return SimpleNamespace(app=app, provider=provider, primitive=primitive, leaf=leaf,
        raw=raw, scripts=scripts, store=store, client=client, root=root, adapter=adapter)


def test_public_well_fifo(connected):
    c = connected
    app, provider, client, store, leaf, raw = c.app, c.provider, c.client, c.store, c.leaf, c.raw
    catalog = catalog_payload(app)
    assert next(a for a in catalog['actions'] if a['action_id'] == ACTION)['enabled']
    export('catalog', catalog)
    requests = [body(provider, 'well-first'), request(provider, 'named-middle'),
                body(provider, 'well-last', well='H12')]
    ids = []
    for index, value in enumerate(requests):
        action = 'oem.deck.move_to_location' if index == 1 else ACTION
        admitted = client.post('/operator/v2/actions/' + action, json=value)
        assert admitted.status_code == 200, admitted.text
        result = admitted.json()
        assert result['action_id'] == action
        ids.append(result['command_id'])
        export('request-' + str(index), {'action_id': action, 'body': value})
        export('admission-' + str(index), result)
    assert [r['command_id'] for r in store.queue()['items']] == ids
    app.state.operator_command_plane.start()
    for index, cid in enumerate(ids):
        result = finish(client, cid)
        export('receipt-' + str(index), result)
        assert result['status'] == 'completed', result
    assert store.queue()['items'] == []
    semantic = store.deck_semantic_state()
    assert semantic['current_well'] == 95
    dashboard = catalog_payload(app)['dashboard']
    assert dashboard['deck']['head_alignment'] == {key: semantic[key] for key in (
        'tip_location', 'semantic_state_revision', 'producer_operation',
        'producer_command_id', 'ownership_generation')}
    export('dashboard', dashboard)
    assert ids[-1] in semantic['transition_provenance']['upstream_source_command_id']
    for index in (0, 2):
        evidence = store.wp8_operation_evidence(ids[index])
        assert [r['operation'] for r in evidence['children']] == ['scriptmoveTo', 'updateLocation']
        export('finite-' + str(index), evidence)
        replay = client.post(URL, json=requests[index]).json()
        assert replay['command_id'] == ids[index] and replay['action_id'] == ACTION
        lookup = client.get('/operator/idempotency/command/' + requests[index]['idempotency_key']).json()
        assert lookup['command_id'] == ids[index] and lookup['response']['action_id'] == ACTION
        export('lookup-' + str(index), lookup)
    assert leaf.moves and raw


@pytest.mark.parametrize('inputs', [
    {}, {'location_id': 3, 'well': 'A1'},
    {'location_id': True, 'well': 'A1', 'position_flag': 1},
    {'location_id': 3, 'well': True, 'position_flag': 1},
    {'location_id': 3, 'well': 'A1', 'position_flag': True},
    {'location_id': 32, 'well': 'A1', 'position_flag': 1},
    {'location_id': 3, 'well': 'I99', 'position_flag': 1},
    {'location_id': 3, 'well': 'A1', 'position_flag': 3},
    {'location_id': 3, 'well': 'A1', 'position_flag': 1, 'camera_offset': True},
])
def test_closed_manual_inputs(inputs):
    from bioxp.operator_command_plane import _validate_inputs
    from fastapi import HTTPException
    with pytest.raises(HTTPException) as failure:
        _validate_inputs(ACTION, inputs)
    assert failure.value.status_code == 422


def submit_well(c, key, **kwargs):
    request_body = body(c.provider, key, **kwargs)
    response = c.client.post(URL, json=request_body)
    assert response.status_code == 200, response.text
    return response.json()['command_id']


@pytest.mark.parametrize('location,wells', [
    (0, ['A1', 'D6', 'H12']), (1, ['A1', 'D6', 'H12']),
    (2, ['A1', 'D6', 'H12']), (3, ['A1', 'D6', 'H12']),
    (7, ['A1', 'D6', 'H12']), (8, ['A1', 'D6', 'H12']),
    (9, ['A1', 'D6', 'H12']), (10, ['A1', 'D6', 'H12']),
    (11, ['A1', 'D1', 'H1']), (12, ['A1', 'D1', 'H1']),
    (13, ['A1', 'D1', 'H1']), (14, ['A1', 'D1', 'H1']),
    (16, ['A1', 'D1', 'H1']),
])
def test_sequential_resource_wells(connected, location, wells):
    from bioxp.manual_pipetting import manual_position_plan
    from bioxp.oem_compat.position_table import well_id_from_label
    c = connected
    c.app.state.operator_command_plane.start()
    for well in wells:
        cid = submit_well(c, f'family-{location}-{well}', location=location, well=well)
        result = finish(c.client, cid)
        export(f'family-{location}-{well}-receipt', result)
        assert result['status'] == 'completed', result
        plan = json.loads(c.store.wp8_operation_evidence(cid)['operation']['plan_json'])
        expected = manual_position_plan(dict(operation='move', location_id=location, well=well, position_flag=1))
        assert plan == expected
        assert c.store.deck_semantic_state()['current_well'] == well_id_from_label(well)
        assert len(c.scripts) == wells.index(well) + 1
    export(f'family-{location}', {'requests': wells, 'scripts': c.scripts})


@pytest.mark.parametrize('tip', [-1, 0, 1, 2, 3])
@pytest.mark.parametrize('location', [3, 7, 11, 16])
def test_source_alignment(connected, tip, location):
    from bioxp.oem_compat.position_table import load_bound_oem_position_table
    from bioxp.oem_compat.pathing import LOCATION_ID_TO_NAME
    c = connected
    stamps = c.provider.deck_owner_authority_stamps()
    c.store.publish_deck_owner_state(source_operation='pipette_owner', source_command_id='offline-tip',
        updates={'tip_location': tip, 'tip_loaded': False}, **stamps)
    cid = submit_well(c, f'align-{tip}-{location}', location=location, well='H1')
    c.app.state.operator_command_plane.start()
    result = finish(c.client, cid)
    assert result['status'] == 'completed', result
    args, script = c.scripts[-1]
    assert args['tip_location'] == tip
    target = load_bound_oem_position_table().resolve(location_id=LOCATION_ID_TO_NAME[location])
    k = tip if tip != -1 and location not in {7, 8, 9, 10, 6, 16} else 0
    assert c.leaf.positions[4, 0] == target.base_coordinates['y'] + 2132 * target.inc_factor * (7 - 2*k)
    assert c.leaf.positions[5, 0] == target.base_coordinates['x']
    assert c.store.deck_display_state()['head_alignment']['tip_location'] == tip


@pytest.mark.parametrize('fault', ['failed_child', 'exception', 'stop'])
def test_native_failure_and_stop(connected, monkeypatch, fault):
    c = connected
    before = c.store.deck_semantic_state()
    if fault == 'failed_child':
        c.leaf.fault = fault
    else:
        original = c.leaf.motor_oem_move_absolute
        def move(*args, **kwargs):
            if fault == 'exception':
                raise RuntimeError('offline controller failure')
            result = original(*args, **kwargs)
            c.store.arm_interrupt_fence('oem.x.stop')
            return result
        monkeypatch.setattr(c.leaf, 'motor_oem_move_absolute', move)
    cid = submit_well(c, 'failure-' + fault)
    c.app.state.operator_command_plane.start()
    result = finish(c.client, cid)
    export('failure-' + fault, result)
    assert result['status'] != 'completed', result
    assert result['action_id'] == ACTION
    assert c.store.deck_semantic_state()['semantic_state_revision'] == before['semantic_state_revision']
    children = c.store.wp8_operation_evidence(cid)['children']
    assert children[1]['terminal_state'] == 'planned'
    c.store.clear_interrupt_fence('oem.x.stop')


def test_already_at_xy_target(connected):
    c = connected
    c.app.state.operator_command_plane.start()
    for index in range(2):
        cid = submit_well(c, 'repeated-' + str(index))
        result = finish(c.client, cid)
        assert result['status'] == 'completed', result
        if index == 0:
            c.leaf.moves.clear()
    # The existing caller may still execute its source Z body. Neither the
    # adapter nor this public route manufactures an XY controller ACK.
    assert not [m for m in c.leaf.moves if m[:2] in {(4, 0), (5, 0)}]
    export('noop', {'receipt': result, 'script': c.scripts[-1], 'moves': c.leaf.moves})
    assert c.scripts[-1][1]['plan']['branch'] == 'same_xy_move_z'
    assert [s['op'] for s in c.scripts[-1][1]['plan']['steps']] == ['moveZ']


@pytest.mark.parametrize('tip', [None, -1, 0, 3])
def test_head_publication_is_not_tip_presence(connected, tip):
    c = connected
    c.store.publish_deck_owner_state(source_operation='pipette_owner', source_command_id='head-' + str(tip),
        updates={'tip_location': tip, 'tip_loaded': False}, **c.provider.deck_owner_authority_stamps())
    semantic = c.store.deck_semantic_state()
    dashboard = catalog_payload(c.app)['dashboard']
    alignment = dashboard['deck']['head_alignment']
    assert alignment == {key: semantic[key] for key in alignment}
    assert alignment['tip_location'] == tip
    assert next(a for a in catalog_payload(c.app)['actions'] if a['action_id'] == ACTION)['enabled']
    export('head-' + str(tip), dashboard)


@pytest.mark.parametrize('mode', ['loaded_tip', 'carried_cover'])
def test_source_custody_branches(connected, monkeypatch, mode):
    c = connected
    stamps = c.provider.deck_owner_authority_stamps()
    if mode == 'loaded_tip':
        c.store.publish_deck_owner_state(source_operation='pipette_owner', source_command_id='offline-loaded',
            updates={'tip_location': 2, 'tip_loaded': True, 'tip_dirty': False}, **stamps)
    else:
        c.primitive.home = False
        c.leaf.positions[4, 2] = 1000
        monkeypatch.setattr(c.primitive, 'motor_get_position', c.leaf.motor_get_position)
        c.store.publish_deck_owner_state(source_operation='GantryLoad', source_command_id='offline-cover',
            updates={'plate_on_gantry': 4, 'tip_loaded': False, 'pseudo_z_home': 65000}, **stamps)
    cid = submit_well(c, 'custody-' + mode, location=3, well='H1')
    c.app.state.operator_command_plane.start()
    result = finish(c.client, cid)
    export('custody-' + mode, {'receipt': result, 'scripts': c.scripts})
    assert result['status'] == 'completed', result
    args, script = c.scripts[-1]
    assert script['plan']['branch'] == ('tip_loaded_midpoint_non_waste' if mode == 'loaded_tip'
        else 'no_tip_not_dirty_default_moveTo')
    if mode == 'carried_cover':
        assert args['gripper_confirmed'] is False
        assert script['execution']['execution_results'][0]['results'][0]['result']['branch'] == 'parallel_y_first'
    assert args['tip_loaded'] is (mode == 'loaded_tip')
    assert args['plate_on_gantry'] == (4 if mode == 'carried_cover' else None)
    assert c.store.deck_semantic_state()['plate_on_gantry'] == args['plate_on_gantry']
    assert all(child['operation'] in {'scriptmoveTo', 'updateLocation'}
        for child in c.store.wp8_operation_evidence(cid)['children'])


def test_absent_machine_target_retains_native_refusal(connected):
    c = connected
    cid = submit_well(c, 'absent-hotel', location=15)
    c.app.state.operator_command_plane.start()
    result = finish(c.client, cid)
    assert result['status'] != 'completed'
    assert 'machine_target_absent_from_serial206_position_table:15' in repr(result)
    assert not c.leaf.moves
    export('absent-hotel', result)


def test_more_than_small_batch_mixed_fifo(connected, monkeypatch):
    import threading
    c = connected
    entered, release = threading.Event(), threading.Event()
    finite = c.app.state.oem_wp8_operation_executor
    named = c.app.state.oem_deck_command_executor
    entries = []
    def record(executor):
        def execute(**kwargs):
            entries.append(kwargs['command_id'])
            if len(entries) == 1:
                entered.set()
                assert release.wait(12)
            return executor(**kwargs)
        return execute
    monkeypatch.setattr(c.app.state, 'oem_wp8_operation_executor', record(finite))
    monkeypatch.setattr(c.app.state, 'oem_deck_command_executor', record(named))
    ids = [submit_well(c, 'burst-first')]
    c.app.state.operator_command_plane.start()
    try:
        assert entered.wait(3)
        for index in range(8):
            if index % 2:
                value = request(c.provider, 'burst-' + str(index))
                value['inputs']['target'] = 'LOC_OC'
                response = c.client.post('/operator/v2/actions/oem.deck.move_to_location', json=value)
                assert response.status_code == 200, response.text
                ids.append(response.json()['command_id'])
            else:
                ids.append(submit_well(c, 'burst-' + str(index), well='H1'))
        assert [row['command_id'] for row in c.store.queue()['items']] == ids
        assert entries == ids[:1]
        assert len(c.store.live_command_worker_ids()) == 1
    finally:
        release.set()
    receipts = [finish(c.client, cid) for cid in ids]
    assert all(row['status'] == 'completed' for row in receipts), receipts
    assert entries == ids
    export('burst', receipts)


@pytest.mark.parametrize('flag', [0, 1, 2])
def test_explicit_native_position_flags(connected, flag):
    c = connected
    cid = submit_well(c, 'position-' + str(flag), well=13, flag=flag)
    c.app.state.operator_command_plane.start()
    result = finish(c.client, cid)
    assert result['status'] == 'completed', result
    plan = c.scripts[-1][1]['plan']
    assert plan['positionflag'] == flag
    assert c.scripts[-1][0]['column'] == 1 and c.scripts[-1][0]['row'] == 1
    assert c.store.deck_semantic_state()['current_well'] == 13


def test_fresh_process_public_identity_and_head(connected):
    import subprocess
    import sys
    c = connected
    cid = submit_well(c, 'fresh-public-identity')
    c.app.state.operator_command_plane.start()
    result = finish(c.client, cid)
    assert result['status'] == 'completed', result
    c.app.state.operator_command_plane.stop()
    expected = c.store.deck_display_state()
    code = ("import json,sys,os; import tests.z_stop_offline_guard; "
        "os.environ['BIOXP_OEM_RUNTIME_STATE_ROOT']=sys.argv[1]; "
        "from bioxp.operator_command_plane import OperatorCommandStore; "
        "s=OperatorCommandStore(sys.argv[1]); "
        "print(json.dumps({'command':s.get_command(sys.argv[2]), 'head':s.deck_display_state()})); s.stop()")
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', code, str(c.root), cid],
        text=True, timeout=20))
    assert reopened['command']['action_id'] == ACTION
    assert reopened['command']['status'] == 'completed'
    assert reopened['head'] == expected
    export('fresh-process', reopened)


def test_request_conflict_and_generation_refusal(connected):
    c = connected
    original = body(c.provider, 'request-binding')
    accepted = c.client.post(URL, json=original)
    assert accepted.status_code == 200
    for field, value in [('inputs', {**original['inputs'], 'well': 'B1'}),
                         ('expected_ownership_generation', original['expected_ownership_generation'] + 1)]:
        response = c.client.post(URL, json={**original, field: value})
        assert response.status_code == 409 and response.json()['detail']['error'] == 'idempotency_conflict'
    wrong = c.client.post(URL, json={**original, 'idempotency_key': 'wrong-generation',
        'expected_ownership_generation': original['expected_ownership_generation'] + 1})
    assert wrong.status_code == 409
    assert not c.leaf.moves
