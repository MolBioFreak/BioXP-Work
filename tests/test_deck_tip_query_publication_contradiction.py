"""Warm genuine no-tip authority, failed observed prefix, durable consumers."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import pytest
from fastapi.testclient import TestClient
from tests.test_deck_tip_query_publication import query_rig, query, named_move
from tests.test_deck_scoped_integration import installed_retained, catalog_action
from tests.test_deck_scoped_authority import retained_rig


@pytest.mark.parametrize('partial', [False, True])
def test_bound_lifecycle_query_uses_real_tester_queue_and_audit(query_rig, partial):
    import asyncio
    from bioxp import api
    from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
    from fastapi import HTTPException
    rig = query_rig
    warm_no_tip(rig)
    if partial:
        rig[7]['data'][0] = [32,96,49]
        rig[7]['data'][1] = [32,96,50]
    # Stage-bound lifecycle entry, not an unsupported generic query admission.
    # The attempt shape is the initialize_motion owner's persisted-stage contract.
    # Motor initialization before this boundary is not part of this query test.
    adapter = Serial206ProductionPrimitiveAdapter.__new__(Serial206ProductionPrimitiveAdapter)
    adapter.pipette_audit_runner = api._run_serial206_pipette_audit
    adapter._lifecycle_pipette_attempt = {
        'command_id': 'initializeMotion:queryTipStatus.initial:isolated',
        'idempotency_key': 'initializeMotion:queryTipStatus.initial:isolated',
    }
    async def dispatch():
        return await api._run_blocking('isolated admitted lifecycle query', adapter.query_tip_status, timeout_s=10)
    if partial:
        with pytest.raises(HTTPException) as failed:
            asyncio.run(dispatch())
        assert failed.value.status_code == 502
        result = failed.value.detail
    else:
        result = asyncio.run(dispatch())['controller_evidence']
    assert result['semantic_query_response_verified'] is (not partial)
    assert result['source_return_completed'] is True
    assert result['deck_state_publication']['status'] == 'published'
    assert rig[0].state.operator_command_plane.store.deck_semantic_state()['tip_loaded'] is partial
    assert rig[6] == [0,1,2,3] * 2
    opened = reopen(rig, result['receipt_id'])
    assert opened['receipt']['result']['ok'] is (not partial)
    row = rig[5].connection.execute('SELECT * FROM pipette_operations WHERE command_id=?',
        (result['command_id'],)).fetchone()
    assert row['caller_class'] == 'lifecycle'
    assert row['lifecycle_stage_id'] == 'serial206.query_tip_status'
    assert row['lifecycle_attempt_id'] == adapter._lifecycle_pipette_attempt['command_id']
    prior_calls = list(rig[6])
    prior_semantic = rig[0].state.operator_command_plane.store.deck_semantic_state()
    if partial:
        with pytest.raises(HTTPException) as replay:
            asyncio.run(dispatch())
        assert replay.value.detail['replayed'] is True
    else:
        assert asyncio.run(dispatch())['controller_evidence']['replayed'] is True
    assert rig[6] == prior_calls
    assert rig[0].state.operator_command_plane.store.deck_semantic_state() == prior_semantic


@pytest.mark.parametrize('partial', [False, True])
def test_actual_initialize_motion_owner_binds_query_attempt(query_rig, monkeypatch, partial):
    import asyncio
    from bioxp import api
    from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
    rig = query_rig
    provider, primitive = rig[1:3]
    # Real lifecycle/stage owners, with only hardware-facing primitive doubles.
    # No motor-reference qualification is claimed from these source returns.
    for name in ('motor_oem_home_axis', 'motor_set_axis_param',
                 'motor_oem_board_move_steps', 'motor_oem_axis_search_home',
                 'motor_set_home', 'motor_oem_move_absolute', 'motor_oem_door_search_home',
                 'oem_set_calibrated_ui_positions_zero', '_oem_no_motion_tmcl_with_readback',
                 'initiate_pipette_group_for_oem_initialize_motion',
                 'checked_pipette_status_for_oem_initialize_motion'):
        monkeypatch.setattr(primitive, name, lambda *a, **k: {'ok': True}, raising=False)
    monkeypatch.setattr(primitive, 'oem_initialize_motors_branch_binding',
        lambda: {'ok': True, 'serial_number': 206, 'camera_calibrated': False}, raising=False)
    monkeypatch.setattr(primitive, 'motor_oem_confirm_thermal_door_closed',
        lambda: {'ok': True, 'oem_predicates': {'tcDoorClosed': True}}, raising=False)
    primitive.pipette_audit_runner = api._run_serial206_pipette_audit
    for name in ('_run_audited_pipette', 'query_tip_status', 'query_all_pipette_tip_states'):
        monkeypatch.setattr(primitive, name,
            getattr(Serial206ProductionPrimitiveAdapter, name).__get__(primitive), raising=False)
    if partial:
        rig[7]['data'][0] = [32,96,49]
        rig[7]['data'][1] = [32,96,50]
    async def dispatch():
        return await api._run_blocking('isolated initializeMotion owner',
            lambda: provider.initialize_motion(mode='live', timeout_s=10), timeout_s=30)
    result = asyncio.run(dispatch())
    assert rig[6] == [0,1,2,3], result
    saved = provider._load_state()['initialize_motion_ledger']['stage_receipts']
    stage = next(item for item in saved if item['stage'] == 'initializeMotion.queryTipStatus.initial')
    rows = rig[5].connection.execute("SELECT * FROM pipette_operations WHERE lifecycle_stage_id=? AND lifecycle_attempt_id=?",
        ('initializeMotion.queryTipStatus.initial', stage['command_id'])).fetchall()
    assert len(rows) == 1
    row = rows[0]
    assert row['lifecycle_attempt_id'] == stage['command_id']
    assert row['caller_class'] == 'lifecycle'
    receipt = json.loads(row['receipt_json'])
    assert receipt['result']['source_return_completed'] is True
    assert receipt['result']['source_exception'] is False
    assert receipt['truth']['semantic_query_response_verified'] is (not partial)
    assert row['status'] == ('failed' if partial else 'observed')
    semantic = rig[0].state.operator_command_plane.store.deck_semantic_state()
    assert semantic['tip_loaded'] is partial
    assert semantic['transition_provenance']['upstream_source_command_id'] == row['command_id']
    assert getattr(primitive, '_lifecycle_pipette_attempt', None) is None


def park_option(rig):
    from bioxp import api
    api._collect_and_publish_hardware_snapshot(['axes', 'latch'], reason='isolated-query-consumer')
    action = catalog_action(rig[0])
    return next(row for row in action['destination_options'] if row['target'] == 'LOC_PARK')


def park_authority(rig):
    """Normal collection followed by passive scoped authority, not intent flags."""
    assert park_option(rig)['enabled']  # Named intent is independent of readiness.
    before = list(rig[2].calls)
    try:
        return rig[1].deck_authority_cached_snapshot(
            expected_generation=rig[1].generation_provider(), target='LOC_PARK')
    finally:
        assert rig[2].calls == before  # Cached consumer performs no physical queries.


def assert_park_unready(rig, match):
    with pytest.raises(RuntimeError, match=match) as refused:
        park_authority(rig)
    # Require fresh negative evidence, not an absent or expired cache.
    started, epoch, outcome = rig[1]._deck_authority_scoped_cache['park.full']
    assert epoch is rig[1]._deck_authority_cache_epoch
    assert time.monotonic() - started < 15.0
    assert outcome is refused.value
    assert str(refused.value) == match
    if os.environ.get('DECK_TEST_OUTPUT'):
        with Path(os.environ['DECK_TEST_OUTPUT'] + '.physical-refusals.jsonl').open('a') as out:
            out.write(json.dumps({'test': os.environ.get('PYTEST_CURRENT_TEST'),
                'reason': str(refused.value), 'age_s': time.monotonic() - started}) + '\n')
    return str(refused.value)


def warm_no_tip(rig):
    named_move(rig)
    query(rig)
    authority = park_authority(rig)
    assert authority['dependency_scope'] == 'full'
    assert authority['current_location_id'] == 'LOC_OC'
    assert authority['tip_loaded'] is False
    assert authority['latch_status'] is True and authority['machine_latch_closed'] is True
    assert authority['collection_tip_state']['tip_exists'] is False
    return rig[0].state.operator_command_plane.store.deck_semantic_state()


def invoke(rig, key):
    return TestClient(rig[0]).post('/liquid/tip-status', headers={'Idempotency-Key': key})


def reopen(rig, receipt_id):
    script = ('import json,sys; from tests.test_deck_tip_query_publication import reopen; '
              'print(json.dumps(reopen(sys.argv[1],sys.argv[2],int(sys.argv[3]))))')
    return json.loads(subprocess.check_output([sys.executable, '-c', script, str(rig[4]),
        receipt_id, str(rig[1].generation_provider())], text=True))


@pytest.mark.parametrize('fault', ['malformed', 'missing', 'throw'])
def test_failed_positive_prefix_is_durable_not_complete_and_recovers_only_on_new_query(query_rig, fault):
    rig = query_rig
    before = warm_no_tip(rig)
    app, provider, _, _, _, receipts, calls, wire, _ = rig
    wire['data'][0] = [32, 96, 49]
    wire['data'][1] = [32, 96, 50]
    if fault == 'missing':
        wire['during'] = lambda channel: wire.update(missing=channel == 1)
    if fault == 'throw':
        def later(channel):
            if channel == 1:
                raise RuntimeError('isolated later CAN failure')
        wire['during'] = later
    response = invoke(rig, 'contradiction-'+fault)
    assert response.status_code == 502, response.text
    detail = response.json()['detail']
    assert detail['ok'] is False and detail['partial_query'] is True
    assert detail['hardware_query_verified'] is False and detail['semantic_query_response_verified'] is False
    assert detail['receipt_id'] and detail['command_id']
    assert detail['channels'][0]['tip_loaded'] is True
    assert len(detail['channels']) == (1 if fault == 'throw' else 4)
    assert detail['channels'][0]['result']['query_response_correlated'] is True
    assert detail['source_return_completed'] is (fault != 'throw')
    assert detail['source_exception'] is (fault == 'throw')
    if fault == 'throw':
        assert detail['details']['failed_channel'] == 1
    else:
        assert detail['details']['invalid_channels'] == [1]
        assert detail['source_return'] == 1 and detail['source_tip_exists'] is True
    assert detail['deck_state_publication']['status'] == 'published'
    assert calls == ([0, 1, 2, 3, 0, 1] if fault == 'throw' else [0, 1, 2, 3] * 2)
    after = app.state.operator_command_plane.store.deck_semantic_state()
    assert (after['tip_loaded'], after['tip_dirty'], after['tip_location']) == (True, None, None)
    for key in ('current_location', 'current_well', 'ambiguity_state', 'clean_path', 'movable_plate_locations'):
        assert after[key] == before[key]
    assert after['transition_provenance']['upstream_source_command_id'] == detail['command_id']
    assert_park_unready(rig, 'deck_semantic_state_not_authoritative:tip_dirty')
    assert_worker_refused(rig, 'failed-prefix-'+fault, 'deck_semantic_state_not_authoritative:tip_dirty')
    opened = reopen(rig, detail['receipt_id'])
    assert opened['semantic'] == after
    assert isinstance(opened['park_blocker'], str) and 'tip_dirty' in opened['park_blocker']
    assert opened['receipt']['result']['ok'] is False
    for key in ('delivery_verified', 'controller_acknowledged', 'completion_verified',
                'semantic_query_response_verified', 'physical_effect_verified'):
        assert opened['receipt']['truth'][key] is False
    persisted = receipts.connection.execute('SELECT status FROM pipette_operations WHERE command_id=?',
        (detail['command_id'],)).fetchone()
    assert persisted['status'] == 'failed'
    prior_calls = list(calls)
    replay = invoke(rig, 'contradiction-'+fault)
    assert replay.status_code == 502, replay.text
    assert replay.json()['detail']['replayed'] is True
    assert calls == prior_calls and app.state.operator_command_plane.store.deck_semantic_state() == after
    wire.clear()
    wire.update(data=[[32,96,48] for _ in range(4)], correlated=True)
    fresh = query(rig, key='explicit-recovery-'+fault)
    assert fresh['command_id'] != detail['command_id']
    assert park_authority(rig)['collection_tip_state']['tip_exists'] is False
    assert reopen(rig, detail['receipt_id'])['receipt'] == opened['receipt']


@pytest.mark.parametrize('fault', ['false_prefix', 'missing_first', 'malformed_first', 'uncorrelated',
    'stale', 'reader', 'reader_replaced', 'generation', 'board', 'interrupt', 'later_owner'])
def test_warm_invalid_prefix_never_overwrites_current_owner(query_rig, monkeypatch, fault):
    rig = query_rig
    before = warm_no_tip(rig)
    app, provider, _, _, _, _, _, wire, transport = rig
    wire['data'][0] = [32,96,49]
    wire['data'][1] = [32,96,50]
    if fault == 'false_prefix': wire['data'][0] = [32,96,48]
    if fault == 'missing_first': wire['missing'] = True
    if fault == 'malformed_first': wire['data'][0] = [32,96,50]
    if fault == 'uncorrelated': wire['correlated'] = False
    if fault == 'stale': wire['age'] = 10
    if fault in ('reader', 'reader_replaced', 'generation', 'board', 'interrupt', 'later_owner'):
        def change(channel):
            if channel != 1: return
            if fault == 'reader': transport._transports[0]._driver.bus.router.reader_generation += 1
            if fault == 'reader_replaced':
                from types import SimpleNamespace
                transport._transports[0]._driver.bus.router = SimpleNamespace(reader_generation=1)
            if fault == 'generation':
                generation = provider.generation_provider()
                monkeypatch.setattr(provider, 'generation_provider', lambda: generation + 1)
            if fault == 'board':
                state = provider._load_state()
                state['x_lifecycle']['board_lifecycle_generation'] += 1
                provider._save_state(state)
            if fault == 'interrupt': transport._interrupt_epoch += 1
            if fault == 'later_owner':
                provider.publish_pipette_owner_state(tip_loaded=True, tip_dirty=True,
                    tip_location=2, source_command_id='later-owned-mutation')
        wire['during'] = change
    response = invoke(rig, 'invalid-'+fault)
    assert response.status_code == 502, response.text
    after = app.state.operator_command_plane.store.deck_semantic_state()
    if fault == 'later_owner':
        assert (after['tip_loaded'], after['tip_dirty'], after['tip_location']) == (True, True, 2)
        assert after['transition_provenance']['upstream_source_command_id'] == 'later-owned-mutation'
    else:
        assert after == before
    if fault in ('false_prefix', 'missing_first', 'malformed_first', 'uncorrelated', 'stale', 'reader', 'reader_replaced', 'interrupt'):
        # MachineStatus stays unchanged; distinct collection source cannot
        # manufacture verified absence from malformed/default channel returns.
        assert_park_unready(rig, 'pipette_collection_reader_or_stop_changed'
                            if fault in ('reader', 'reader_replaced', 'interrupt')
                            else 'pipette_collection_state_not_authoritative')


@pytest.mark.parametrize('failure', ['publisher', 'receipt'])
def test_recording_failure_is_not_motor_reference_recovery(query_rig, monkeypatch, failure):
    rig = query_rig
    before = warm_no_tip(rig)
    app, provider, _, references, _, receipts, _, wire, _ = rig
    reference_before = references.snapshot(('x', 'y', 'z', 'g'))['rows']
    wire['data'][0] = [32,96,49]
    wire['data'][1] = [32,96,50]
    if failure == 'publisher':
        monkeypatch.setattr(provider, '_deck_semantic_state_publisher',
            lambda **kw: (_ for _ in ()).throw(RuntimeError('isolated publisher failure')))
    else:
        from bioxp.pipette.receipts import PipetteReceiptError
        monkeypatch.setattr(receipts, 'record',
            lambda **kw: (_ for _ in ()).throw(PipetteReceiptError('isolated receipt failure')))
    response = invoke(rig, 'persist-failure-'+failure)
    assert response.status_code == (502 if failure == 'publisher' else 503)
    assert references.snapshot(('x', 'y', 'z', 'g'))['rows'] == reference_before
    assert app.state.operator_command_plane.store.deck_semantic_state() == before
    assert not provider._deck_authority_scoped_cache
    if failure == 'publisher':
        detail = response.json()['detail']
        assert detail['deck_state_publication']['status'] == 'blocked'
        opened = reopen(rig, detail['receipt_id'])
        assert opened['receipt']['result']['ok'] is False
    # No motor reset/latch: collection presence or unresolved receipt owns
    # this refusal independently of the unchanged MachineStatus mirror.
    assert_park_unready(rig, 'deck_semantic_state_not_authoritative:clean_path'
                        if failure == 'publisher' else 'pipette_collection_receipt_pending')


def submit_named(rig, target, key, *, expected_status='completed'):
    from bioxp import api
    app, provider = rig[:2]
    client = TestClient(app)
    api._collect_and_publish_hardware_snapshot(['axes', 'latch'], reason='isolated-next-command')
    action = catalog_action(app)
    assert action['enabled'] and action['expected_board_epoch_by_board'], action
    response = client.post('/operator/v2/actions/oem.deck.move_to_location', json={
        'schema_version': 'bioxp.operator_action_request.v2', 'idempotency_key': key,
        'expected_ownership_generation': provider.generation_provider(),
        'expected_board_epoch_by_board': action['expected_board_epoch_by_board'],
        'inputs': {'target': target, 'camera_offset': False}})
    assert response.status_code == 200, response.text
    path = '/operator/v2/actions/receipts/' + response.json()['command_id']
    end = time.monotonic()+5
    result = {}
    while time.monotonic() < end:
        result = client.get(path).json()
        if result['status'] not in ('queued', 'dispatched', 'issued_pending'): break
        time.sleep(.01)
    assert result['status'] == expected_status, json.dumps(client.get(path+'?detail=true').json())
    return client.get(path+'?detail=true').json()


def assert_worker_refused(rig, key, reason):
    """Real POST -> active worker -> native executor rejection -> durable GET."""
    before = len(rig[2].calls)
    receipt = submit_named(rig, 'LOC_PARK', key, expected_status='failed')
    assert receipt['terminal'] is True
    assert [row['to_status'] for row in receipt['transitions']] == ['queued', 'dispatched', 'failed']
    terminal = receipt['source_receipt']['terminal_evidence']
    assert terminal['detail'] == reason
    assert terminal['delivery_attempted'] is False
    assert receipt['physical_effect_verified'] is False
    assert receipt['transport_exchanges'] == []
    leaves = rig[2].calls[before:]
    assert all(row[0] in ('home', 'position', 'xyz', 'latch') for row in leaves), leaves
    script = ('import json,sys; from tests.test_deck_scoped_integration import fresh_process_receipts; '
              'print(json.dumps(fresh_process_receipts(sys.argv[1],sys.argv[2])))')
    durable = json.loads(subprocess.check_output([sys.executable, '-c', script, str(rig[4]),
        receipt['command_id']], text=True))
    assert durable['detail'] == receipt
    assert durable['compact']['status'] == 'failed'
    if os.environ.get('DECK_TEST_OUTPUT'):
        Path(os.environ['DECK_TEST_OUTPUT'] + '.' + key + '.worker.json').write_text(json.dumps({
            'receipt': receipt, 'fresh_process': durable, 'physical_leaf_calls': leaves}, indent=2))
    return receipt


def test_no_tip_query_then_real_queued_park_and_next_named_command(query_rig, monkeypatch):
    rig = query_rig
    from bioxp import oem_machine_bundle
    from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot
    snapshot = bind_serial206_oem_snapshot(monkeypatch)
    # Existing native recorder fixture: real immutable validator, synthetic label
    # only inside private /dev and denied USB discovery. No live flags.
    snapshot = oem_machine_bundle.load_oem_machine_snapshot(
        snapshot.bundle_root / 'OEM_EVIDENCE_LOCK.json', operator_label_serial=206,
        require_operator_label=True)
    monkeypatch.setattr(oem_machine_bundle, '_active_snapshot', snapshot)
    from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
    rig[2].oem_initialize_motion_scriptmove_to_waste = (
        Serial206ProductionPrimitiveAdapter.oem_initialize_motion_scriptmove_to_waste.__get__(rig[2]))
    before_park = warm_no_tip(rig)
    query_command = before_park['transition_provenance']['upstream_source_command_id']
    query_receipt = json.loads(rig[5].connection.execute(
        'SELECT receipt_json FROM pipette_operations WHERE command_id=?', (query_command,)).fetchone()[0])
    receipt = submit_named(rig, 'LOC_PARK', 'queried-park')
    assert receipt['deck_movement']['semantic_state_committed'] is True
    semantic = rig[0].state.operator_command_plane.store.deck_semantic_state()
    assert semantic['current_location'] == 'LOC_PARK'
    assert any(row[0] == 'move' for row in rig[2].calls)
    reopened = reopen(rig, query_receipt['receipt_id'])
    assert reopened['semantic'] == semantic
    assert reopened['receipt']['truth']['semantic_query_response_verified'] is True
    script = ('import json,sys; from tests.test_deck_scoped_integration import fresh_process_receipts; '
              'print(json.dumps(fresh_process_receipts(sys.argv[1],sys.argv[2])))')
    durable = json.loads(subprocess.check_output([sys.executable, '-c', script, str(rig[4]),
        receipt['command_id']], text=True))
    assert durable['detail'] == receipt
    assert durable['compact']['status'] == 'completed'
    submit_named(rig, 'LOC_OC', 'after-queried-park')
    assert rig[0].state.operator_command_plane.store.deck_semantic_state()['current_location'] == 'LOC_OC'


def test_sqlite_publication_failure_rolls_back_without_motor_reset(query_rig):
    """Actual SQLite refusal: intent accepted, active worker denies delivery."""
    import sqlite3
    rig = query_rig
    before = warm_no_tip(rig)
    app, provider, primitive, references, root, receipts, calls, wire, _ = rig
    reference_before = references.snapshot(('x', 'y', 'z', 'g'))['rows']
    denied = []
    def deny_tip_update(action, table, column, database, trigger):
        if action == sqlite3.SQLITE_UPDATE and table == 'operator_plane_deck_semantic_state':
            denied.append((table, column))
            return sqlite3.SQLITE_DENY
        return sqlite3.SQLITE_OK
    connection = app.state.operator_command_plane.store.connection
    connection.set_authorizer(deny_tip_update)
    wire['data'][0] = [32,96,49]
    try:
        response = invoke(rig, 'actual-sqlite-publication-failure')
    finally:
        connection.set_authorizer(None)
    assert denied, 'actual SQLite owner UPDATE must reach the injected storage refusal'
    assert response.status_code == 200, response.text
    result = response.json()
    assert result['semantic_query_response_verified'] is True
    assert result['deck_state_publication']['status'] == 'blocked'
    assert result['deck_state_publication']['reason'] == 'not authorized'
    assert references.snapshot(('x', 'y', 'z', 'g'))['rows'] == reference_before
    assert app.state.operator_command_plane.store.deck_semantic_state() == before
    assert reopen(rig, result['receipt_id'])['semantic'] == before
    assert not provider._deck_authority_scoped_cache
    assert calls == [0,1,2,3] * 2
    refusal = assert_park_unready(rig, 'deck_semantic_state_not_authoritative:clean_path')
    prior_motion = len(primitive.calls)
    admitted = assert_worker_refused(rig, 'storage-limit-active-worker', refusal)
    leaf_calls = primitive.calls[prior_motion:]
    assert not any(row[0] == 'move' for row in leaf_calls)
    assert app.state.operator_command_plane.store.deck_semantic_state() == before
    assert references.snapshot(('x', 'y', 'z', 'g'))['rows'] == reference_before

    if os.environ.get('DECK_TEST_OUTPUT'):
        Path(os.environ['DECK_TEST_OUTPUT'] + '.storage-limit.json').write_text(json.dumps({
            'query': result, 'retained_owner': before, 'physical_refusal': refusal, 'worker_receipt': admitted, 'leaf_calls': leaf_calls,
            'references_unchanged': True, 'park_executed': False}, indent=2))
