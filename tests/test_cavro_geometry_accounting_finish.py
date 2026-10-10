"""Synthetic explicit geometry through production owners; no physical claims."""
import copy
import json
import os
from pathlib import Path
import pytest
from bioxp import api
from bioxp.pipette.cavro_application import ApplicationRequest, compile_application, capability_catalog
from bioxp.pipette.cavro_liquid import Recipe, compile_liquid_recipe
from bioxp.pipette.cavro_geometry import resolve_surface_move, resolve_rack_transform
from bioxp.pipette.effect_accounting import CompletedEffectAccount
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from tests.test_cavro_application import rig as wire_rig, request, stroke, run
from tests.test_cavro_liquid import recipe
from tests.test_cavro_surface_coordination import export
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_method_runtime_connected import mount, submit, action
from tests.test_protocol_workflow_connected import await_job
from tests.test_oem_pipette_calibration import MotionNative


def vessel(tapered=False, fill='20'):
    return {'vessel_id': 'source', 'provenance': 'synthetic test only', 'bottom': 'flat',
            'shape': 'tapered' if tapered else 'flat-sided', 'fill_ul': fill,
            'fill_provenance': 'authored test estimate', 'dead_volume_ul': '1',
            'inaccessible_volume_ul': '2',
            **({'cross_section_model': 'linear_area', 'area_points': [
                {'height_mm': '0', 'area_mm2': '2'}, {'height_mm': '10', 'area_mm2': '12'}]}
               if tapered else {'cross_section_model': 'constant', 'area_mm2': '10'})}


def surface(tapered=False, fill='20'):
    return {'operation': 'surface_move', 'intent': 'immersion',
            'reference': {'kind': 'vessel', 'vessel': vessel(tapered, fill),
                'selected_level': 'surface', 'bottom_head_steps': 40000,
                'calibration': {'calibration_id': 'fixture', 'steps_per_mm': '100',
                    'native_speed_per_mm_s': '10', 'positive_steps_direction': 'down'},
                'tip_correction_mm': '0', 'fallback_position_steps': 39000},
            'displacement': {'unit': 'steps', 'offset_steps': 50, 'speed_native': 300}}


def vector(x=0, y=0, z=0):
    return dict(x=str(x), y=str(y), z=str(z))


def rack():
    return {'calibration_id': 'fixture-rack', 'provenance': 'synthetic measured fixture',
        'origin_mm': vector(10, 20, 30), 'row_pitch_mm': vector(0, -9, 0),
        'column_pitch_mm': vector(9, 0, 0), 'tool_offset_mm': vector(1, 2, 3),
        'reference_tip_offset_mm': vector(0, 0, 5), 'reference_channel': 2,
        'channel_wells': [{'channel': 2, 'row': 0, 'column': 1, 'well_id': 'D2'},
                          {'channel': 0, 'row': -2, 'column': 1, 'well_id': 'B2'}],
        'x_steps_per_mm': vector(100, 0, 0), 'y_steps_per_mm': vector(0, -100, 0),
        'z_steps_per_mm': vector(0, 0, 100), 'native_origin_steps': vector(0, 5000, 0)}


def head():
    return {'context_id': 'mixed', 'trajectory_policy': 'authored_common_z', 'mounted_tips': [
        {'channel': ch, 'profile': {'profile_id': f'T{vol}', 'nominal_volume_ul': vol,
         'manufacturer': 'test only', 'sku': 'test', 'filtered': False, 'length_mm': length},
         'footprint': {'offset_mm': vector(0, y, length), 'radius_mm': '1', 'provenance': 'synthetic'},
         'vessel': vessel()} for ch, vol, length, y in [(2, 10, 5, 0), (0, 1000, 7, 18), (3, 50, 6, -9)]]}


def accounting(known=True):
    return {'context_id': 'fixture-empty' if known else 'fixture-unknown',
        'initial_channels': [{'channel': ch, 'pneumatic': [] if known else None,
            'liquid_intent': [] if known else None, 'provenance': 'explicit test state'} for ch in [0, 1, 2]],
        'vessels': [vessel(), dict(vessel(fill='0'), vessel_id='destination')]}


def authored(tapered=False, known=True):
    r = recipe()
    r.update(channels=[2], target_liquid_ul='20', commanded_aspiration_ul='22',
        leading_air={'volume_ul': '2', 'speed_ul_s': '30'},
        trailing_air={'volume_ul': '1', 'speed_ul_s': '30'},
        before_liquid=[surface(tapered, '16' if tapered else '20')],
        after_liquid=[], before_dispense=[], after_dispense=[],
        final_empty_speed_ul_s=None, shared_head=head(), accounting_context=accounting(known),
        aspiration_intent={'role': 'sample', 'channel_vessels': {'2': 'source'}},
        dispense_segments=[{'volume_ul': '5', 'speed_ul_s': '30', 'target_liquid_ul': '4',
            'accounting_intent': {'role': 'sample', 'channel_vessels': {'2': 'destination'}}}])
    return r


@pytest.mark.parametrize('dispense_type,kind', [(1, 'front_air'), (2, 'rear_air')])
def test_explicit_air_dispense_retains_distinct_intent(dispense_type, kind):
    acc = CompletedEffectAccount({'channel': 0, 'pneumatic': [{'kind': kind, 'volume_ul': '3'}],
                                 'liquid_intent': [{'kind': kind, 'volume_ul': '3'}], 'provenance': 'fixture'})
    result = acc.apply('dispense_air', {'volume_ul': '1', 'dispense_type': dispense_type})
    inventory = result['occupancy']['authored_liquid_intent']
    assert inventory['orifice_first_compartments'] == [{'kind': kind, 'volume_ul': '2'}]
    assert result['intended_liquid_delta_ul'] == 0


def test_exact_decimal_occupancy_and_accessibility_without_float_roundtrip():
    from decimal import Decimal
    initial = {'channel': 0, 'pneumatic': [], 'liquid_intent': [], 'provenance': 'fixture'}
    acc = CompletedEffectAccount(initial, [vessel(fill='1')])
    acc.apply('aspirate', {'volume_ul': '1', 'target_liquid_ul': '1'})
    out = acc.apply('aspirate', {'volume_ul': 1e-100, 'target_liquid_ul': 1e-100})
    expected = '1.' + '0' * 99 + '1'
    assert out['occupancy']['pneumatic']['occupancy_ul'] == expected
    assert out['occupancy']['authored_liquid_intent']['occupancy_ul'] == expected
    unknown = acc.apply('dispense', {'volume_ul': '1', 'accounting_vessel_id': 'source'})
    assert unknown['occupancy']['vessels'][0]['planned_fill_ul'] is None
    assert unknown['occupancy']['vessels'][0]['unknown_completed_liquid_effect']


@pytest.mark.parametrize('level,expected', [('bottom', 40050), ('accessible_floor', 40030)])
def test_selected_bottom_and_accessibility_contact(level, expected):
    op = surface()
    op['reference']['selected_level'] = level
    op['intent'] = 'contact'
    assert resolve_surface_move(op, {})['target_steps'] == expected


def test_export_actual_producers():
    from bioxp.manual_pipetting import ManualPipettingRequest
    r = authored(True)
    plan = compile_liquid_recipe(r)
    assert not plan['issues'], plan
    ApplicationRequest.model_validate(plan['application'])
    Recipe.model_validate(r)
    for name, body in [('application-schema', ApplicationRequest.model_json_schema()),
            ('manual-pipetting-schema', ManualPipettingRequest.model_json_schema()),
            ('recipe-schema', Recipe.model_json_schema()), ('application-catalog', capability_catalog()),
            ('geometry-accounting-recipe', r), ('geometry-accounting-plan', plan),
            ('geometry-accounting-application', plan['application'])]:
        export(name, body)


@pytest.mark.parametrize('tapered,fill,expected', [(False, '20', 39850), (False, '50', 39550),
                                                  (True, '16', 39650), (True, '6', 39850)])
def test_declared_geometry_consumed(tapered, fill, expected):
    out = resolve_surface_move(surface(tapered, fill), {})
    assert out['target_steps'] == expected
    assert out['detection']['geometry_used']
    assert not out['physical_surface_verified']


@pytest.mark.parametrize('change', ['unknown_fill', 'unknown_model', 'nonintegral', 'beyond_domain'])
def test_unknown_geometry_is_observation_not_gate(change):
    op = surface(True, '16')
    if change == 'unknown_fill': op['reference']['vessel']['fill_ul'] = None
    if change == 'unknown_model': op['reference']['vessel']['cross_section_model'] = None
    if change == 'nonintegral': op['reference']['vessel']['fill_ul'] = '1'
    if change == 'beyond_domain': op['reference']['vessel']['fill_ul'] = '1000'
    plan = compile_application(request(op))
    assert not plan['issues'], plan
    out = resolve_surface_move(op, {})
    assert out['target_steps'] == 39050
    assert not out['detection']['geometry_used'] and out['detection']['observation']


def test_signed_mapping_and_all_mounted_footprints():
    out = resolve_rack_transform(rack(), head(), [2])
    assert out['target_steps'] == {'x': 1800, 'y': 3200, 'z': 2200}
    rows = out['mounted_tip_footprints']
    assert [r['channel'] for r in rows] == [2, 0, 3]
    assert rows[0]['well_offset_mm'] == vector()
    assert rows[1]['well_offset_mm'] == vector(0, 0, 2)
    assert rows[1]['xy_bounds_mm'] == {'x': ['18', '20'], 'y': ['37', '39']}
    assert not rows[1]['pump_active'] and not rows[2]['pump_active']
    assert rows[2]['well_offset_mm'] is None


@pytest.mark.parametrize('known', [True, False])
def test_separate_inventory_air_boundaries_and_unknown(known):
    initial = accounting(known)['initial_channels'][0]
    acc = CompletedEffectAccount(initial, accounting()['vessels'])
    acc.apply('aspirate_air', {'volume_ul': '2', 'front_air': True})
    acc.apply('aspirate', {'volume_ul': '22', 'target_liquid_ul': '20', 'accounting_vessel_id': 'source'})
    acc.apply('aspirate_air', {'volume_ul': '1', 'front_air': False})
    out = acc.apply('dispense', {'volume_ul': '5', 'target_liquid_ul': '4', 'accounting_vessel_id': 'destination'})
    occ = out['occupancy']
    assert occ['pneumatic']['known_delta_ul'] == '20'
    assert occ['pneumatic']['cumulative_travel_ul'] == '30'
    assert occ['pneumatic']['peak_occupancy_ul'] == ('25' if known else None)
    assert occ['authored_liquid_intent']['peak_occupancy_ul'] == ('23' if known else None)
    assert occ['transitions']['pneumatic']['crossed'][0]['kind'] == 'rear_air'
    assert occ['transitions']['pneumatic']['crossed'][1]['volume_ul'] == '4'
    assert occ['transitions']['authored_liquid_intent']['crossed'][0]['removal_not_authored']
    assert occ['transitions']['authored_liquid_intent']['crossed'][1]['volume_ul'] == '4'
    assert occ['vessels'][0]['planned_fill_ul'] == '0'
    assert occ['vessels'][1]['planned_fill_ul'] == '4'
    assert out['measured_liquid_ul'] is None
    emptied = acc.apply('dispense_all', {})
    assert emptied['occupancy']['pneumatic']['occupancy_ul'] is None
    assert emptied['unknown_pneumatic_effect']


@pytest.mark.parametrize('tapered,known', [(False, True), (True, True), (True, False), (False, None)])
def test_request_recipe_real_finite_transport_stores_http(wire_rig, installed_retained, monkeypatch, tmp_path, tapered, known):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    provider = installed_retained[1]
    native = MotionNative()
    provider.primitives = Serial206ProductionPrimitiveAdapter(native, wire_rig.group,
        authority_provider=lambda: None, generation_provider=provider.generation_provider)
    receipts = PipetteReceiptStore(installed_retained[4])
    monkeypatch.setattr(api, '_pipette_receipts', receipts)
    monkeypatch.setattr(api, '_require_motion_route_ready', lambda: None)
    r = authored(tapered, known)
    if known is None:
        r['accounting_context']['vessels'][0]['fill_ul'] = None
        r['before_liquid'][0]['reference']['vessel']['fill_ul'] = None
    job = submit(app, client, [action('pipette_manual_physical', {'operation': 'cavro_liquid_recipe', 'recipe': r})],
                 key=f'finish-{tapered}-{known}')
    done = await_job(client, job, lambda r: r['command']['terminal'])
    source = done['execution']['runtime_state']['action_results'][0].get('pipette_result', {})
    export(f'finish-{tapered}-{known}', {'request': r, 'result': done, 'wire': wire_rig.wire, 'moves': native.moves})
    assert source.get('ok'), source.get('error', done)
    assert ('z', 39050 if known is None else 39650 if tapered else 39850, True) in native.moves
    event = [e for e in source['events'] if e['operation'] == 'dispense'][-1]
    channel = event['result']['channels'][0]
    # Follow actual transport publication rather than an invented success row.
    export('finish-channel', channel)
    occ = channel['result']['accounting']['occupancy']
    assert occ['pneumatic']['peak_occupancy_ul'] == ('25' if known else None)
    assert occ['pneumatic']['known_delta_ul'] == '20'
    assert occ['pneumatic']['cumulative_travel_ul'] == '30'
    assert occ['vessels'][0]['planned_fill_ul'] == (None if known is None else '0')
    assert occ['vessels'][1]['planned_fill_ul'] == '4'
    store = app.state.operator_command_plane.store
    ids = done['execution']['runtime_state']['workflow']['child_command_ids']
    finite = [cid for cid in ids if not cid.startswith('pipette_')]
    durable = store.wp8_operation_evidence(finite[0])
    assert durable['children'][0]['operation'] == 'sourceCavroApplication'
    assert 'peak_occupancy_ul' in json.dumps(durable)
    assert 'peak_occupancy_ul' in json.dumps(receipts.read(limit=100))
    assert client.get('/protocol/jobs/' + job).json()['execution']['runtime_state'] == done['execution']['runtime_state']
    reopened = PipetteReceiptStore(installed_retained[4])
    assert reopened.read(limit=100) == receipts.read(limit=100)
    export(f'finish-cold-{tapered}-{known}', {'pipette_receipts': reopened.read(limit=100), 'finite': durable})
    import subprocess, sys
    db = store.connection.execute('PRAGMA database_list').fetchone()[2]
    receipt_db = receipts.connection.execute('PRAGMA database_list').fetchone()[2]
    code = "import sqlite3,json,sys; c=sqlite3.connect('file:'+sys.argv[1]+'?mode=ro',uri=True); p=sqlite3.connect('file:'+sys.argv[2]+'?mode=ro',uri=True); print(json.dumps({'finite':[json.loads(r[0]) for r in c.execute('SELECT terminal_evidence_json FROM operator_plane_wp8_children WHERE command_id=? ORDER BY child_order',(sys.argv[3],))], 'pipette':[json.loads(r[0]) for r in p.execute('SELECT receipt_json FROM pipette_operations ORDER BY rowid')]}))"
    cold = json.loads(subprocess.run([sys.executable, '-c', code, db, receipt_db, finite[0]],
        check=True, capture_output=True, text=True).stdout)
    expected = {'finite': [json.loads(r[0]) for r in store.connection.execute(
        'SELECT terminal_evidence_json FROM operator_plane_wp8_children WHERE command_id=? ORDER BY child_order', (finite[0],))],
        'pipette': [json.loads(r[0]) for r in receipts.connection.execute('SELECT receipt_json FROM pipette_operations ORDER BY rowid')]}
    assert cold == expected
    export(f'finish-fresh-process-{tapered}-{known}', cold)


@pytest.mark.parametrize('pickup', [False, True])
def test_measured_rack_xy_z_real_finite_pickup(wire_rig, installed_retained, monkeypatch, tmp_path, pickup):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    provider = installed_retained[1]
    native = MotionNative()
    provider.primitives = Serial206ProductionPrimitiveAdapter(native, wire_rig.group,
        authority_provider=lambda: None, generation_provider=provider.generation_provider)
    receipts = PipetteReceiptStore(installed_retained[4])
    monkeypatch.setattr(api, '_pipette_receipts', receipts)
    monkeypatch.setattr(api, '_require_motion_route_ready', lambda: None)
    for driver in wire_rig.drivers:
        original = driver._send_pipette_command
        def report(command, *args, _original=original, **kwargs):
            if command == '?31':
                return {'ok': True, 'ack': {'received': True, 'data': [32, 96, 49]},
                        'query_response_correlated': True, 'fixture': 'inert tip report'}
            return _original(command, *args, **kwargs)
        monkeypatch.setattr(driver, '_send_pipette_command', report)
    positioning = {'operation': 'move', 'location_id': 2, 'well': 'A1', 'position_flag': 1}
    z = surface()
    z['reference'] = {'kind': 'rack', 'transform': rack(), 'fallback_position_steps': 39000}
    z['intent'] = 'contact'
    if pickup:
        ops = [{'operation': 'profile_pickup', 'profile': head()['mounted_tips'][0]['profile'],
            'calibration': {'calibration_id': 'fixture', 'rack_id': 'rack', 'positioning': positioning,
                'rack_transform': rack(), 'pickup': z, 'retract': None,
                'channel_wells': {'2': 'D2'}, 'provenance': 'synthetic fixture'}}]
    else:
        ops = [{'operation': 'position', 'positioning': positioning, 'rack_transform': rack()}, z]
    payload = request(*ops)
    payload['shared_head'] = head()
    plan = compile_application(payload)
    assert not plan['issues'], plan
    job = submit(app, client, [action('pipette_manual_physical', {'operation': 'cavro_application', 'application': payload})],
                 key='rack-' + str(pickup))
    done = await_job(client, job, lambda r: r['command']['terminal'])
    source = done['execution']['runtime_state']['action_results'][0].get('pipette_result', {})
    export('rack-' + str(pickup), {'request': payload, 'plan': plan, 'result': done, 'moves': native.moves})
    assert source.get('ok'), source.get('error', done)
    assert ('z', 2250, True) in native.moves
    assert any(m[0] == 'x' and m[1] == 1800 for m in native.moves), native.moves
    assert any(m[0] == 'y' and m[1] == 3200 for m in native.moves), native.moves
    geometry = next(e['result'] for e in source['events'] if e['operation'] == 'rack_geometry')
    assert len(geometry['mounted_tip_footprints']) == 3
    assert geometry['mounted_tip_footprints'][1]['well_offset_mm']['z'] == '2'
    positioned = next(e['result'] for e in source['events'] if e['operation'] == ('profile_pickup_geometry' if pickup else 'surface_reference'))
    assert positioned['mounted_tip_footprints'][1]['tip_reference_steps']['z'] == '3250'


def test_label_only_changes_no_commands():
    r = authored()
    before = compile_liquid_recipe(r)
    r['aspiration_intent']['label'] = 'different documentary liquid'
    r['dispense_segments'][0]['accounting_intent']['role'] = 'excess'
    after = compile_liquid_recipe(r)
    wires = lambda p: [op.get('wire_commands') for op in p['operations']]
    assert wires(before) == wires(after)


def test_multi_roles_labels_and_exact_completed_travel(wire_rig):
    r = authored()
    r.update(mode='multi', target_liquid_ul='30', commanded_aspiration_ul='35',
             before_liquid=[], dispense_segments=[])
    r['accounting_context']['vessels'][0]['fill_ul'] = '60'
    segment = {'volume_ul': '4', 'speed_ul_s': '30', 'target_liquid_ul': '3',
               'accounting_intent': {'role': 'sample', 'channel_vessels': {'2': 'destination'}}}
    r['multi'] = {'sample_count': 2, 'sample_volume_ul': '3', 'conditioning_volume_ul': '2',
        'conditioning_back_to_source_count': 2, 'conditioning_speed_ul_s': '30',
        'conditioning_target_liquid_ul': '2',
        'conditioning_intent': {'role': 'conditioning', 'channel_vessels': {'2': 'source'}},
        'conditioning_before': [], 'conditioning_after': [], 'excess_volume_ul': '1',
        'excess': {'before': [], 'after': [], 'segments': [{'volume_ul': '2', 'speed_ul_s': '30',
            'target_liquid_ul': '1', 'accounting_intent': {'role': 'excess', 'channel_vessels': {'2': 'destination'}}}]},
        'reaspiration': {'volume_ul': '1', 'speed_ul_s': '30'}, 'dispense_to_reaspiration_delay_ms': 0,
        'aliquots': [{'before': [], 'after': [], 'segments': [copy.deepcopy(segment)]} for _ in range(2)]}
    plan = compile_liquid_recipe(r)
    assert not plan['issues'], plan
    changed = copy.deepcopy(r)
    changed['multi'].update(sample_volume_ul='777', excess_volume_ul='888')
    relabeled = compile_liquid_recipe(changed)
    assert [o.get('wire_commands') for o in plan['operations']] == [o.get('wire_commands') for o in relabeled['operations']]
    assert relabeled['liquid_settings']['accounting_labels']['sample_label_differs_from_explicit_intent']
    out = run(wire_rig, plan['application'])
    assert out['ok'], out
    summary = out['accounting_summary']
    occ = summary['channels']['2']['occupancy']
    assert occ['roles']['conditioning']['count'] == 2
    assert occ['roles']['excess']['pneumatic_travel_ul'] == '2'
    assert occ['roles']['reaspiration']['pneumatic_travel_ul'] == '2'
    assert occ['pneumatic']['peak_occupancy_ul'] == '38'
    assert occ['pneumatic']['cumulative_travel_ul'] == '54'
    assert occ['pneumatic']['occupancy_ul'] == '26'
    assert summary['vessels'][0]['planned_fill_ul'] == '34'
    assert summary['vessels'][1]['planned_fill_ul'] == '7'
    export('finish-multi', {'request': r, 'plan': plan, 'result': out, 'label_mutation_plan': relabeled, 'wire': wire_rig.wire})


@pytest.mark.parametrize('fault', ['missing', 'device', 'interrupt'])
def test_partial_accounting_actual_http_and_both_stores(wire_rig, installed_retained, monkeypatch, tmp_path, fault):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    provider = installed_retained[1]
    provider.primitives = Serial206ProductionPrimitiveAdapter(MotionNative(), wire_rig.group,
        authority_provider=lambda: None, generation_provider=provider.generation_provider)
    receipts = PipetteReceiptStore(installed_retained[4])
    monkeypatch.setattr(api, '_pipette_receipts', receipts)
    monkeypatch.setattr(api, '_require_motion_route_ready', lambda: None)
    for driver in wire_rig.drivers:
        original = driver._send_pipette_command
        def send(command, *args, _original=original, **kwargs):
            if command.startswith('P22'):
                wire_rig.fault[fault] = True
            return _original(command, *args, **kwargs)
        monkeypatch.setattr(driver, '_send_pipette_command', send)
    r = authored()
    r['channels'] = [0, 1]
    r['aspiration_intent']['channel_vessels'] = {'0': 'source', '1': 'source'}
    job = submit(app, client, [action('pipette_manual_physical', {'operation': 'cavro_liquid_recipe', 'recipe': r})], key='partial-http-' + fault)
    done = await_job(client, job, lambda row: row['command']['terminal'])
    source = done['execution']['runtime_state']['action_results'][0]['pipette_result']
    assert not source['ok']
    summary = source['accounting_summary']
    assert summary['incomplete_application'] and summary['uncompleted_strokes_not_estimated']
    assert summary['channels']['1']['occupancy']['pneumatic']['known_delta_ul'] == '2'
    assert not any(w.startswith('D') for _, w in wire_rig.wire)
    store = app.state.operator_command_plane.store
    ids = done['execution']['runtime_state']['workflow']['child_command_ids']
    finite = next(cid for cid in ids if not cid.startswith('pipette_'))
    durable = store.wp8_operation_evidence(finite)
    assert 'known_delta_ul' in json.dumps(durable)
    assert client.get('/protocol/jobs/' + job).json()['execution']['runtime_state'] == done['execution']['runtime_state']
    reopened = PipetteReceiptStore(installed_retained[4])
    assert reopened.read(limit=100) == receipts.read(limit=100)
    export('finish-partial-http-' + fault, {'request': r, 'result': done, 'finite': durable, 'pipette_receipts': reopened.read(limit=100)})


@pytest.mark.parametrize('fault', ['missing', 'device', 'interrupt'])
def test_completed_prefix_not_failed_stroke(wire_rig, monkeypatch, fault):
    for driver in wire_rig.drivers:
        original = driver._send_pipette_command
        def send(command, *args, _original=original, **kwargs):
            if command.startswith("P22"):
                wire_rig.fault[fault] = True
            return _original(command, *args, **kwargs)
        monkeypatch.setattr(driver, "_send_pipette_command", send)
    payload = request(stroke('leading_air', 2), {**stroke('aspirate', 22), 'target_liquid_ul': '20'}, stroke('dispense', 5))
    payload['accounting_context'] = accounting()
    out = run(wire_rig, payload)
    assert not out['ok']
    assert not any(w.startswith('D') for _, w in wire_rig.wire)
    accounts = [t._effect_account for t in wire_rig.group._transports[:2]]
    assert accounts[1].pneumatic.delta == 2
    assert accounts[1].completed_strokes == 1
    export('finish-partial-' + fault, {'request': payload, 'result': out, 'receipts': wire_rig.store.read(limit=100)})
