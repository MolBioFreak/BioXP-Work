"""A1/C1-C4: real named executor/provider/adapter/SQLite, offline wire only."""
import json
from collections import Counter
import pytest
from tests.test_wake_setup_debloat import rig, cycle
from bioxp.operator_command_plane import OperatorCommandStore
from bioxp.oem_deck_movement import make_deck_command_executor
from bioxp.oem_compat.position_table import load_bound_oem_position_table


@pytest.fixture
def named(rig, monkeypatch, request):
    driver, provider, frames, fault, root = rig
    driver._motor_last_tx_ts = {}
    cycle(driver)
    from bioxp.serial206_y_provider import Serial206YProvider
    provider.primitives.y_provider = Serial206YProvider(driver, state_store=provider.state_store,
        generation_provider=lambda: 7, reference_store=provider.reference_store)
    store = OperatorCommandStore(root)
    provider.bind_deck_semantic_state_reader(store.deck_semantic_state)
    provider.bind_deck_semantic_state_publisher(store.publish_deck_owner_state)
    provider.bind_tip_tray_state_reader(store.tip_tray_state)
    provider.bind_tip_tray_state_publisher(store.publish_tip_tray_transition)
    store.bind_deck_owner_authority_reader(provider.deck_owner_authority_stamps,
                                         scope=provider.deck_owner_authority_scope)
    # Native I/O runs unchanged; sensor electrical values are transport replies.
    wire = driver.send_tmcl
    positions = {}
    fault['latch'] = 1
    def exchange(board, command, typ, motor, value, **kw):
        result = wire(board, command, typ, motor, value, **kw)
        if command == 4 and result['status'] == 100:
            positions[board, motor] = value
        if command == 6 and typ == 1:
            result['value'] = positions.get((board, motor), 0)
        if command == 15 and typ == 3:
            result['value'] = fault['latch']
        return result
    monkeypatch.setattr(driver, 'send_tmcl', exchange)
    monkeypatch.setattr(driver, 'send_tmcl_retry', exchange)
    # Independent source host latch owner.
    driver._oem_latch_status = True
    driver._oem_latch_status_generation = 0
    provider.bind_pipette_collection_state_reader(lambda: {'tip_exists': False})
    stamps = provider.deck_owner_authority_stamps()
    for op, updates in (() if getattr(request, 'param', None) == 'cold' else (
        ('updateLocation', {'current_location': 'LOC_MS', 'current_well': 0}),
        ('pipette_owner', {'tip_loaded': False, 'tip_dirty': False, 'tip_location': -1}),
        ('sourceForceToHighHome', {'pseudo_z_home': 500}),
    )):
        store.publish_deck_owner_state(source_operation=op, source_command_id='fixture-'+op,
                                       updates=updates, **stamps)
    counts = Counter()
    for name in ('deck_authority_snapshot', '_fresh_deck_latch_observation', '_load_state'):
        original = getattr(provider, name)
        def counted(*a, _name=name, _original=original, **kw):
            counts[_name] += 1
            return _original(*a, **kw)
        monkeypatch.setattr(provider, name, counted)
    execute = make_deck_command_executor(provider_getter=lambda: provider,
        position_table_provider=load_bound_oem_position_table, command_store=store)
    def run(target='LOC_OC', key='move', camera_offset=False):
        epochs = {'4': stamps['board_epoch_4'], '5': stamps['board_epoch_5']}
        request = dict(schema_version='bioxp.operator_action_request.v2', action_id='oem.deck.move_to_location',
            expected_ownership_generation=7, expected_board_epoch_by_board=epochs,
            idempotency_key=key, inputs={'target': target, 'camera_offset': camera_offset})
        admitted = store.admit_command(request, state={'ownership_generation': 7,
            'serial206_initialization_provider': {'x_authority': {'current_board_lifecycle_generation': epochs['5']},
            'board4_authority': {'active_board_epoch': epochs['4']}}}, assessment={'enabled': True})
        claimed = store.claim_next()
        assert claimed['command_id'] == admitted['command_id']
        counts.clear(); frames.clear()
        result = execute(command_id=admitted['command_id'], target=target, camera_offset=camera_offset,
                         expected_ownership_generation=7, expected_board_epoch_by_board=epochs)
        store.finish(admitted['command_id'], status='completed' if result['ok'] else 'failed',
            payload=result, claimed=claimed, controller_acknowledged=result['controller_command_acknowledged'],
            full_response=result)
        import os
        from pathlib import Path
        if os.environ.get('CORE_TRACE_DIR'):
            destination = Path(os.environ['CORE_TRACE_DIR'])
            destination.mkdir(parents=True, exist_ok=True)
            (destination / (admitted['command_id'] + '.json')).write_text(json.dumps({
                'target': target, 'key': key, 'counts': dict(counts), 'wire': frames,
                'result': result, 'semantic': store.deck_semantic_state()}, indent=2))
        return result, admitted['command_id']
    yield run, provider, store, counts, frames, fault
    store.stop()


@pytest.mark.parametrize('named', ['warm', 'cold'], indirect=True)
def test_warm_ordinary(named):
    run, provider, store, counts, frames, fault = named
    result, cid = run()
    assert result['ok'], json.dumps(result, indent=2)
    assert counts['deck_authority_snapshot'] == 1
    assert counts['_fresh_deck_latch_observation'] == 2
    assert store.deck_semantic_state()['current_location'] == 'LOC_OC'


def test_park_noop_has_only_caller_latch(named):
    run, provider, store, counts, frames, fault = named
    store.publish_deck_owner_state(source_operation='updateLocation', source_command_id='fixture-park',
        updates={'current_location': 'LOC_PARK', 'current_well': 0}, **provider.deck_owner_authority_stamps())
    result, cid = run('LOC_PARK')
    assert result['ok'], json.dumps(result, indent=2)
    assert counts['deck_authority_snapshot'] == 0
    assert counts['_fresh_deck_latch_observation'] == 1
    assert result['delivery_attempted'] is False
    assert result['controller_completion_verified'] is False
    assert result['provider_results'][0]['source_noop'] is True
    assert not any(command in {2, 4} for _, command, _, _, _ in frames)


@pytest.mark.parametrize('named', ['warm', 'cold'], indirect=True)
def test_park_travel(named):
    run, provider, store, counts, frames, fault = named
    result, cid = run('LOC_PARK')
    assert result['ok'], json.dumps(result, indent=2)
    assert counts['deck_authority_snapshot'] == 1
    assert counts['_fresh_deck_latch_observation'] == 2
    assert result['delivery_attempted'] is True


@pytest.mark.parametrize('target', ['LOC_OC', 'LOC_PARK'])
@pytest.mark.parametrize('which', ['host', 'sensor', 'opens', 'closes'])
def test_final_manual_predicates(named, monkeypatch, target, which):
    run, provider, store, counts, frames, fault = named
    driver = provider.primitives.tester
    original = provider.force_to_high_home
    if which == 'closes':
        driver._oem_latch_status = False
    def force(**kw):
        result = original(**kw)
        if which in {'host', 'opens'}:
            driver._oem_latch_status = False
        elif which == 'sensor':
            fault['latch'] = 0
        else:
            driver._oem_latch_status = True
        return result
    monkeypatch.setattr(provider, 'force_to_high_home', force)
    if target == 'LOC_PARK':
        store.publish_deck_owner_state(source_operation='updateLocation', source_command_id='fixture-park',
            updates={'current_location': 'LOC_PARK', 'current_well': 0}, **provider.deck_owner_authority_stamps())
    result, cid = run(target)
    assert result['ok'] is (which == 'closes')
    stages = store.connection.execute('SELECT operation,terminal_state,terminal_evidence_json FROM operator_plane_deck_stages WHERE command_id=? ORDER BY stage_order', (cid,)).fetchall()
    assert stages[0]['terminal_state'] == 'completed'
    if which != 'closes':
        assert result['error'] == 'latch_not_closed'
        assert not any(command in {2, 4} for _, command, _, _, _ in frames)
    else:
        assert all(row['terminal_state'] == 'completed' for row in stages)
    assert len(stages) == 4  # target step never pruned by the early latch


def test_consecutive_and_fresh_process(named):
    import subprocess, sys
    run, provider, store, counts, frames, fault = named
    for index, target in enumerate(('LOC_OC', 'LOC_MS', 'LOC_PARK', 'LOC_PARK', 'LOC_OC')):
        result, cid = run(target, 'consecutive-'+str(index))
        assert result['ok']
        assert counts['deck_authority_snapshot'] == (0 if index == 3 else 1)
        assert store.deck_semantic_state()['current_location'] == target
    code = "import sys,json; from bioxp.operator_command_plane import OperatorCommandStore; s=OperatorCommandStore(sys.argv[1]); print(json.dumps(s.deck_semantic_state())); s.stop()"
    process = subprocess.run([sys.executable, '-c', code, str(store.root)], capture_output=True, text=True)
    assert process.returncode == 0, process.stderr
    assert json.loads(process.stdout)['current_location'] == 'LOC_OC'


@pytest.mark.parametrize('pending', [False, True])
def test_closeout_builds_only_pending_evidence(named, monkeypatch, pending):
    import inspect
    run, provider, store, counts, frames, fault = named
    built = []
    original = store._deck_stage_evidence
    def evidence(*a, **kw):
        if inspect.currentframe().f_back.f_code.co_name == 'commit_deck_success':
            built.append(a[0].operation)
        return original(*a, **kw)
    monkeypatch.setattr(store, '_deck_stage_evidence', evidence)
    terminalize = store.terminalize_deck_stage
    def stage(cid, step, **kw):
        if pending and step.operation == 'moveTo':
            return
        return terminalize(cid, step, **kw)
    monkeypatch.setattr(store, 'terminalize_deck_stage', stage)
    result, cid = run()
    assert result['ok']
    assert built == (['moveTo'] if pending else [])
    assert all(row[0] == 'completed' for row in store.connection.execute(
        'SELECT terminal_state FROM operator_plane_deck_stages WHERE command_id=?', (cid,)))


@pytest.mark.parametrize('target,camera,loaded', [
    ('LOC_OC', True, False), ('LOC_TC_BARCODE', False, False), ('LOC_OC', False, True),
])
def test_source_move_branches(named, target, camera, loaded):
    run, provider, store, counts, frames, fault = named
    if loaded:
        state = provider._load_state()
        state['machine_status']['tip_loaded'] = True
        provider._save_state(state)
    result, cid = run(target, camera_offset=camera)
    assert result['ok'], json.dumps(result, indent=2)
    assert counts['deck_authority_snapshot'] == 1
    if 'BARCODE' in target:
        stages = store.connection.execute('SELECT operation FROM operator_plane_deck_stages WHERE command_id=? ORDER BY stage_order', (cid,)).fetchall()
        assert stages[-1][0] == 'moveZCamera'


def test_delivery_reuses_owner_read_and_keeps_sqlite_checks(named, monkeypatch):
    import inspect
    run, provider, store, counts, frames, fault = named
    reads = Counter()
    original = store._deck_owner_authority_reader
    def read():
        reads[inspect.currentframe().f_back.f_code.co_name] += 1
        return original()
    monkeypatch.setattr(store, '_deck_owner_authority_reader', read)
    result, cid = run()
    assert result['ok']
    assert reads['record_delivery_attempt'] == 2
    assert reads['_validate_deck_owner_authority'] == 1  # pseudo-home publication only


@pytest.mark.parametrize('change', ['stop', 'owner'])
def test_real_dispatch_fences_before_movement(named, monkeypatch, change):
    run, provider, store, counts, frames, fault = named
    original = provider.force_to_high_home
    def force(**kw):
        result = original(**kw)
        if change == 'stop':
            store.arm_interrupt_fence('oem.x.stop')
        else:
            provider.generation_provider = lambda: 8
        return result
    monkeypatch.setattr(provider, 'force_to_high_home', force)
    with pytest.raises(RuntimeError):
        run()
    assert not any(command in {2, 4} for _, command, _, _, _ in frames)
