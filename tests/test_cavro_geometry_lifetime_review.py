"""Eight bounded geometry/accounting counterexamples; physical leaves are inert."""
import copy
import json
import subprocess
import sys
import pytest
from bioxp import api
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from tests.test_cavro_application import request, stroke
from tests.test_cavro_geometry_accounting_finish import accounting, authored, rack, head, surface
from tests.test_cavro_surface_coordination import export
from tests.test_cavro_transport_repairs import receiver
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_method_runtime_connected import mount, submit, action
from tests.test_protocol_workflow_connected import await_job
from tests.test_oem_pipette_calibration import MotionNative


@pytest.fixture
def public(receiver, installed_retained, monkeypatch, tmp_path):
    from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot
    snapshot = bind_serial206_oem_snapshot(monkeypatch)
    from bioxp import oem_machine_bundle
    monkeypatch.setattr(oem_machine_bundle, '_active_snapshot', oem_machine_bundle.load_oem_machine_snapshot(
        snapshot.bundle_root / 'OEM_EVIDENCE_LOCK.json', operator_label_serial=206, require_operator_label=True))
    for leaf in receiver.group._transports:
        leaf._initialized = True
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    native = MotionNative()
    native.motor_wait_stopped = lambda *args, **kwargs: {'stopped': True, 'fixture': 'inert hardware completion'}
    provider = installed_retained[1]
    provider.primitives = Serial206ProductionPrimitiveAdapter(native, receiver.group,
        authority_provider=lambda: None, generation_provider=provider.generation_provider)
    receipts = PipetteReceiptStore(installed_retained[4])
    monkeypatch.setattr(api, '_pipette_receipts', receipts)
    monkeypatch.setattr(api, '_require_motion_route_ready', lambda: None)
    return app, client, native, receipts, receiver


def execute(public, payloads, key):
    app, client, native, receipts, wire = public
    actions = [dict(action('pipette_manual_physical', {'operation': 'cavro_application',
        'application': payload}), action_id='application-' + str(i)) for i, payload in enumerate(payloads)]
    job = submit(app, client, actions, key=key)
    done = await_job(client, job, lambda r: r['command']['terminal'])
    store = app.state.operator_command_plane.store
    # Read exact rows from both real SQLite owners in a fresh process, including
    # the workflow's persisted HTTP representation, not just channel counters.
    db = store.connection.execute('PRAGMA database_list').fetchone()[2]
    receipt_db = receipts.connection.execute('PRAGMA database_list').fetchone()[2]
    ids = done['execution']['runtime_state']['workflow']['child_command_ids']
    queries = [
        ('SELECT receipt_json FROM operator_commands WHERE command_id=?', [job]),
        ('SELECT terminal_evidence_json FROM operator_plane_wp8_children WHERE command_id IN (' + ','.join('?' for _ in ids) + ') ORDER BY command_id,child_order', ids),
        ('SELECT receipt_json FROM pipette_operations ORDER BY command_id', [])]
    code = """import json,sqlite3,sys
connections=[sqlite3.connect('file:'+p+'?mode=ro',uri=True) for p in sys.argv[1:3]]
queries=json.loads(sys.argv[3])
print(json.dumps([[json.loads(r[0]) for r in connections[0 if i<2 else 1].execute(q,args)] for i,(q,args) in enumerate(queries)]))
"""
    cold = json.loads(subprocess.check_output([sys.executable, '-c', code, db, receipt_db, json.dumps(queries)], text=True))
    expected = [[json.loads(r[0]) for r in (store.connection if i < 2 else receipts.connection).execute(q, args)]
        for i, (q, args) in enumerate(queries)]
    assert cold == expected
    assert cold[0][0]['execution']['runtime_state'] == done['execution']['runtime_state']
    assert client.get('/protocol/jobs/' + job).json()['execution']['runtime_state'] == done['execution']['runtime_state']
    results = [r['pipette_result'] for r in done['execution']['runtime_state']['action_results'] if 'pipette_result' in r]
    export('review-' + key, {'requests': payloads, 'job': done, 'cold': cold,
        'wire': wire.wire, 'moves': native.moves})
    return results, done, cold


def device(command, program=False):
    return {'operation': 'device_program' if program else 'device_command', 'channels': [0], 'timeout_ms': 30,
        **({'program': {'mode': 'execute', 'commands': [command]}} if program else {'command': command})}


@pytest.mark.parametrize('second', ['command', 'program', 'query', 'absent', 'empty_unknown'])
def test_accounting_occurrence_lifetime(public, second):
    seed = request(stroke('leading_air', 7, channels=[0]))
    seed['accounting_context'] = accounting()
    if second == 'command':
        op = device({'command': 'plunger_aspirate', 'value': 2, 'unit': 'ul'})
    elif second == 'program':
        op = device({'command': 'plunger_aspirate', 'value': 2, 'unit': 'ul'}, True)
    elif second == 'query':
        op = device({'command': 'report', 'selector': 57})
    elif second == 'empty_unknown':
        op = {'operation': 'empty', 'channels': [0], 'timeout_ms': 30, 'speed_ul_s': 50}
    else:
        op = stroke('aspirate', 2, channels=[0])
    payload = request(op)
    if second != 'absent':
        payload['accounting_context'] = accounting(second != 'empty_unknown')
        payload['accounting_context']['context_id'] = 'second-' + second
        for initial in payload['accounting_context']['initial_channels']:
            initial['provenance'] = 'second occurrence'
    if second == 'absent':
        seed_results, _, _ = execute(public, [seed], 'lifetime-seed-manual')
        group = public[4].group
        import asyncio
        from bioxp.services.pipette_service import run_pipette_operation
        async def inline(label, body, **kwargs):
            return body()
        manual = asyncio.run(run_pipette_operation('aspirate_air',
            lambda t: t.aspirate_air(1, channels=[0]), get_transport=lambda: group,
            run_blocking=inline, receipt_store=public[3],
            requested_inputs={'volume_ul': 1, 'channels': [0], 'front_air': True},
            runtime_binding={'idempotency_key': 'review-unscoped-manual',
                'entrypoint_id': 'protocol.pipette_manual_physical', 'caller_class': 'protocol_manual'}))
        # Finite repeat probe on a never-scoped leaf: legacy counters must keep
        # accumulating, but optional compartment lists must not grow forever.
        import time
        sizes, timings = [], []
        for i in range(32):
            start = time.monotonic()
            group.aspirate_air(1, channels=[3])
            timings.append(time.monotonic() - start)
            sizes.append(len(group._transports[3]._effect_account.pneumatic.segments))
        export('review-unscoped-repeat', {'manual': manual, 'segment_counts': sizes,
            'elapsed_s': timings, 'wire': public[4].wire})
        fresh = manual['channels'][0]['result']['accounting']
        assert not fresh['initial_fill_known']
        assert not fresh.get('occupancy', {}).get('vessels')
        assert not fresh.get('occupancy', {}).get('initial_state')
        assert sizes[-1] == sizes[0] == 0
        assert group._transports[3]._front_air_level_ul == 32
        results, _, cold = execute(public, [payload], 'lifetime-' + second)
        results = seed_results + results
    else:
        results, _, cold = execute(public, [seed, payload], 'lifetime-' + second)
    assert len(results) == 2 and all(r['ok'] for r in results)
    assert results[0]['accounting_summary']['channels']['0']['occupancy']['pneumatic']['known_delta_ul'] == '7'
    result = results[1]
    if second == 'query':
        assert result['accounting_summary']['channels'] == {}
        assert 'occupancy' not in json.dumps(result['events'])
        return
    event = result['events'][-1]['result']['channels'][0]['result']['accounting']
    if second == 'absent':
        assert 'accounting_summary' not in result
        assert not event['initial_fill_known']
        assert 'occupancy' not in event  # Optional inventories are not lifetime defaults.
        assert event['known_pneumatic_delta_ul'] == 3
    else:
        occ = event['occupancy']
        assert occ['pneumatic']['known_delta_ul'] == ('2' if second == 'command' else '0')
        assert occ['initial_state']['provenance'] == 'second occurrence'
        assert occ['pneumatic']['occupancy_ul'] == ('2' if second == 'command' else None)
        assert result['accounting_summary']['channels']['0']['occupancy'] == occ


@pytest.mark.parametrize('partial', [False, True])
def test_public_summary_and_geometry_projection(public, partial):
    from bioxp.pipette.cavro_liquid import compile_liquid_recipe
    raw = authored()
    raw['channels'] = [0, 1]
    raw['aspiration_intent']['channel_vessels'] = {'0': 'source', '1': 'source'}
    raw['dispense_segments'][0]['accounting_intent']['channel_vessels'] = {'0': 'destination', '1': 'destination'}
    if partial:
        # Fail channel 1 only at the liquid stroke, after a complete air prefix.
        endpoint = public[4].router.ep_out
        original = endpoint.write
        def write(frame, timeout):
            from tests.test_cavro_transport_repairs import novo_decode
            if b'P22' in novo_decode(frame)[5:]:
                public[4].fault['device'] = 1
            return original(frame, timeout)
        endpoint.write = write
    payload = compile_liquid_recipe(raw)['application']
    results, done, cold = execute(public, [payload], 'projection-' + str(partial))
    result = results[0]
    assert result['ok'] is not partial
    summary = result['accounting_summary']
    assert summary['incomplete_application'] is partial
    assert summary['channels']['1']['occupancy']['pneumatic']['known_delta_ul'] == ('2' if partial else '20')
    assert summary['actual_delivery_ul'] is None
    def summaries(value):
        if isinstance(value, str):
            try: yield from summaries(json.loads(value))
            except (ValueError, TypeError): pass
        elif isinstance(value, dict):
            if 'accounting_summary' in value: yield value['accounting_summary']
            for v in value.values(): yield from summaries(v)
        elif isinstance(value, list):
            for v in value: yield from summaries(v)
    assert summary in list(summaries(cold[0]))
    refs = [e['result'] for e in result['events'] if e['operation'] == 'surface_reference']
    assert len(refs[0]['mounted_tip_footprints']) == 3
    assert refs[0]['mounted_tip_footprints'][1]['tip_reference_steps']['z'] == '40850'


def test_geometry_lifetime_after_legacy_and_authored_movements(public, monkeypatch):
    position = {'operation': 'position', 'positioning': {'operation': 'move', 'location_id': 2, 'well': 'A1', 'position_flag': 1}}
    measured = dict(position, rack_transform=rack())
    tracked = dict(stroke('tracked_aspirate', 2, channels=[2]), trajectory=surface(), z_motor_current=20, z_timeout_ms=1000)
    plld = {'operation': 'plld', 'channels': [2], 'timeout_ms': 30, 'start_steps': 40000,
        'search_target_steps': 60000, 'search_speed_native': 300, 'z_motor_current': 20}
    payload = request(measured, surface(), position, surface(), measured,
        {'operation': 'z_move', 'target_steps': 38000, 'speed_native': 300}, tracked, plld, surface())
    payload['shared_head'] = head()
    # Optional exact footprint precision must not deselect an exact rack target.
    precise = copy.deepcopy(payload)
    precise['operations'] = [measured, surface()]
    precise['shared_head']['mounted_tips'][1]['footprint']['offset_mm']['z'] = '7.' + '0' * 2048 + '1'
    from bioxp.pipette.cavro_application import compile_application
    compiled = compile_application(precise)
    export('review-optional-footprint-plan', compiled)
    assert not compiled['issues'], compiled['issues']
    precise_results, _, _ = execute(public, [precise], 'geometry-optional-precision')
    results, _, _ = execute(public, [payload], 'geometry-lifetime')
    out = results[0]
    assert out['ok'], out.get('error')
    assert compiled['operations'][0]['resolved_rack']['geometry_used']
    assert precise_results[0]['ok']
    assert precise_results[0]['events'][0]['result']['geometry_used']
    refs = [e['result'] for e in out['events'] if e['operation'] == 'surface_reference']
    assert refs[0]['mounted_tip_footprints'][1]['coordinate_known']
    assert refs[1]['head_target_steps'].get('x') is None
    assert refs[1]['head_target_steps'].get('y') is None
    assert not refs[1]['mounted_tip_footprints'][1]['coordinate_known']
    z = next(e['result'] for e in out['events'] if e['operation'] == 'z_move' and e['inputs']['target_steps'] == 38000)
    assert z['mounted_tip_footprints'][1]['tip_reference_steps']['z'] == '39000'
    tracking = next(e['result'] for e in out['events'] if e['operation'] == 'tracking_reference')
    assert [r['pump_active'] for r in tracking['mounted_tip_footprints']] == [True, False, False]
    assert tracking['mounted_tip_footprints'][1]['tip_reference_steps']['z'] == '40850'
    observed = next(e['result'] for e in out['events'] if e['operation'] == 'plld_z_position')
    assert observed['head_target_steps']['z'] == observed['position_steps'] == 88000
    assert observed['mounted_tip_footprints'][1]['tip_reference_steps']['z'] == '89000'
    # Failure of optional presentation cannot change a chosen move or replace
    # the primary physical-leaf exception. No new successful-return fixture.
    import bioxp.pipette.cavro_application as application
    def unavailable(*args, **kwargs):
        raise ArithmeticError('injected optional footprint observation')
    monkeypatch.setattr(application, 'mounted_trajectory', unavailable)
    small = request(surface())
    small['shared_head'] = head()
    healthy, _, _ = execute(public, [small], 'geometry-observation-unavailable')
    assert healthy[0]['ok']
    assert healthy[0]['events'][0]['result']['observation_error'] == 'injected optional footprint observation'
    public[2].fail_at = ('z', 39850)
    failed, _, _ = execute(public, [small], 'geometry-primary-failure')
    assert not failed[0]['ok']
    assert failed[0]['error'] == 'injected_native_transfer_failure'
    assert failed[0]['exception_type'] == 'RuntimeError'
