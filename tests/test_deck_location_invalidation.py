"""Standalone dispatch → native provider → SQLite → genuine named Park."""
import json
import subprocess
import sys

import pytest

from tests.test_debloat_core import named
from tests.test_wake_setup_debloat import rig
from bioxp.deck_location_invalidation import STANDALONE_ACTIONS, standalone_xyz_route


@pytest.fixture
def native_named(named, monkeypatch):
    driver = named[1].primitives.tester
    wire = driver.send_tmcl
    positions = {}
    def exchange(board, command, typ, motor, value, **kw):
        result = wire(board, command, typ, motor, value, **kw)
        if result['status'] == 100 and (command == 4 or (command == 5 and typ == 1)):
            positions[board, motor] = value
        if command == 6 and typ == 1 and (board, motor) in positions:
            result['value'] = positions[board, motor]
        return result
    monkeypatch.setattr(driver, 'send_tmcl', exchange)
    monkeypatch.setattr(driver, 'send_tmcl_retry', exchange)
    return named


def dispatch(store, provider, action, inputs=None):
    stamps = provider.deck_owner_authority_stamps()
    epochs = {'4': stamps['board_epoch_4'], '5': stamps['board_epoch_5']}
    admitted = store.admit_command(dict(schema_version='bioxp.operator_action_request.v2',
        action_id=action, expected_ownership_generation=7,
        expected_board_epoch_by_board=epochs, idempotency_key='independent-'+action,
        inputs=inputs or {}), state={'ownership_generation': 7,
        'serial206_initialization_provider': {'x_authority': {'current_board_lifecycle_generation': epochs['5']},
        'board4_authority': {'active_board_epoch': epochs['4']}}}, assessment={'enabled': True})
    assert store.deck_semantic_state()['current_location'] == 'LOC_PARK'
    claimed = store.claim_next()
    assert claimed['command_id'] == admitted['command_id']
    return claimed


@pytest.mark.parametrize('action,intent,inputs', [
    ('oem.x.manual_panel_home', 'manual_panel_home', {}),
    ('oem.x.diagnostic_home_axis', 'home_axis', {}),
    ('oem.x.set_home', 'set_home', {}),
    ('oem.x.move_absolute', 'move_absolute', {'position_steps': 12000}),
    ('oem.x.move_steps', 'move_steps', {'steps': 1000}),
])
def test_park_independent_x_then_real_park(native_named, action, intent, inputs):
    run, provider, store, counts, frames, fault = native_named
    result, _ = run('LOC_PARK', 'first-park')
    assert result['ok'] and result['delivery_attempted']
    before = store.deck_semantic_state()
    refs = provider.reference_store.snapshot(('y', 'z', 'g'))
    claimed = dispatch(store, provider, action, inputs)
    assert store.connection.execute('SELECT current_location FROM operator_plane_deck_semantic_state').fetchone()[0] == 'UNKNOWN'
    snapshot = provider.state_store.read_oem_serial206_initialization_state()
    assert snapshot['machine_status']['current_location'] == 32
    after = store.deck_semantic_state()
    for key in ('current_well', 'current_tray', 'tip_loaded', 'tip_dirty', 'tip_location',
                'plate_on_gantry', 'movable_plate_locations', 'pseudo_z_home'):
        assert before[key] == after[key]
    assert refs == provider.reference_store.snapshot(('y', 'z', 'g'))
    result = provider.execute_x_intent(intent, {**inputs, 'command_id': claimed['command_id'],
        'idempotency_key': claimed['command_id'], 'expected_generation': 7})
    assert result['ok'], json.dumps(result, indent=2)
    store.finish(claimed['command_id'], status='completed', payload=result, claimed=claimed)
    result, _ = run('LOC_PARK', 'second-park')
    assert result['ok'] and result['delivery_attempted'], json.dumps(result, indent=2)
    assert store.deck_semantic_state()['current_location'] == 'LOC_PARK'
    result, _ = run('LOC_PARK', 'third-park')
    assert result['ok'] and result['delivery_attempted'] is False
    assert result['controller_completion_verified'] is False


def test_interrupted_dispatch_survives_actual_fresh_process(native_named):
    run, provider, store, counts, frames, fault = native_named
    assert run('LOC_PARK', 'park')[0]['ok']
    dispatch(store, provider, 'oem.x.manual_panel_home')
    code = """import sys,json
from bioxp.operator_command_plane import OperatorCommandStore
from bioxp.oem_runtime_store import OEMRuntimeStore
s=OperatorCommandStore(sys.argv[1])
r=OEMRuntimeStore(sys.argv[1])
print(json.dumps({'semantic':s.deck_semantic_state()['current_location'],
                 'fallback':r.read_oem_serial206_initialization_state()['machine_status']['current_location']}))
s.stop()
"""
    completed = subprocess.run([sys.executable, '-c', code, str(store.root)], capture_output=True, text=True)
    assert completed.returncode == 0, completed.stderr
    assert json.loads(completed.stdout) == {'semantic': 'UNKNOWN', 'fallback': 32}


def test_failure_stays_unknown_and_park_travels(native_named):
    run, provider, store, counts, frames, fault = native_named
    assert run('LOC_PARK', 'park')[0]['ok']
    claimed = dispatch(store, provider, 'oem.x.move_absolute', {'position_steps': 12000})
    def wire_failure(board, command, typ, motor, value):
        if command == 4:
            raise RuntimeError('injected offline transport interruption')
    fault['on_frame'] = wire_failure
    result = provider.execute_x_intent('move_absolute', {'position_steps': 12000,
        'command_id': claimed['command_id'], 'idempotency_key': claimed['command_id'], 'expected_generation': 7})
    assert result['ok'] is False
    store.finish(claimed['command_id'], status='failed', payload=result, claimed=claimed)
    assert store.deck_semantic_state()['current_location'] == 'UNKNOWN'
    fault['reject'] = None
    fault['on_frame'] = None
    frames.clear()
    from bioxp.oem_deck_movement import DeckExecutionFailure
    # The real native driver retains the failed transport's pending motor
    # outcome. Park must enter its source travel, not falsely return already-Park.
    with pytest.raises(DeckExecutionFailure):
        run('LOC_PARK', 'return')
    assert counts['deck_authority_snapshot'] == 1
    assert store.deck_semantic_state()['current_location'] == 'UNKNOWN'


@pytest.mark.parametrize('path', ['/motion/oem/manual/relative', '/motion/oem/manual/absolute',
    '/motion/oem/manual/home', '/motion/oem/manual/sethome', '/motion/axis/relative',
    '/motion/axis/absolute', '/motion/axis/zero', '/motion/axis/home'])
@pytest.mark.parametrize('axis', ['x', 'y', 'z', 'g', 'gripper', 'door'])
def test_manual_route_scope(path, axis):
    assert standalone_xyz_route(path, {'axis': axis}) is (
        path.startswith('/motion/oem/manual/') and axis in {'x', 'z'})


@pytest.mark.parametrize('axis', ['x', 'y', 'z', 'g', 'door'])
@pytest.mark.parametrize('operation', ['move-negative', 'move-positive', 'home',
    'park-6000', 'status', 'stop', 'not-a-diagnostic'])
def test_diagnostic_route_scope(axis, operation):
    assert standalone_xyz_route('/motion/diagnostics/execute',
        {'axis': axis, 'operation': operation}) is (
            axis == 'x' and operation in {'move-negative', 'move-positive', 'home', 'park-6000'})


def test_generated_live_route_actions_share_semantic_alias_invalidation():
    from bioxp import api
    from bioxp.operator_controls import _build_catalog
    actions, targets = _build_catalog(api.app)
    checked = []
    for action in actions:
        if action['action_id'] not in STANDALONE_ACTIONS:
            continue
        target = targets.get(action['action_id'])
        if target is None:
            continue
        inputs = {'axis': 'z', **target.get('fixed_inputs', {})}
        assert standalone_xyz_route(target['path'], inputs), action['action_id']
        generated = [row for row in actions if row['category'] == 'route'
                     and row['informational_path'] == target['path']]
        for row in generated:
            assert standalone_xyz_route(row['informational_path'], inputs), row['action_id']
        checked.append(action['action_id'])
    assert {'oem.x.manual_panel_home', 'oem.y.manual_panel_home',
            'oem.z.manual_home', 'oem.xy.home'} <= set(checked)


@pytest.mark.parametrize('action', ['oem.x.stop', 'oem.y.stop', 'oem.z.stop', 'oem.abort_all',
    'oem.x.prepare', 'oem.z.prepare', 'oem.deck.move_to_location', 'oem.deck._mov_execution',
    'oem.deck._finite_operation', 'oem.z.scriptmove_to', 'oem.xyz.move_to'])
def test_internal_passive_preview_excluded(action):
    assert action not in STANDALONE_ACTIONS


def test_internal_native_move_does_not_clear_caller_location(native_named):
    run, provider, store, counts, frames, fault = native_named
    assert run('LOC_PARK', 'park')[0]['ok']
    result = provider.execute_x_intent('move_absolute', {'position_steps': 12000,
        'command_id': 'internal-script-x', 'idempotency_key': 'internal-script-x', 'expected_generation': 7})
    assert store.deck_semantic_state()['current_location'] == 'LOC_PARK'


@pytest.mark.parametrize('action,intent,inputs', [
    ('oem.z.move_steps', 'move_steps', {'steps': 1000}),
    ('oem.z.move_absolute', 'move_absolute', {'position_steps': 12000}),
    ('oem.z.set_home', 'set_home', {}),
    ('oem.z.diagnostic_home_axis', 'diagnostic_home_axis', {}),
    ('oem.y.move_steps', 'move_steps', {'steps': 1000}),
    ('oem.y.move_absolute', 'move_absolute', {'target_steps': 12000}),
    ('oem.y.manual_panel_home', 'home', {}),
])
def test_park_independent_yz_then_real_park(native_named, action, intent, inputs):
    run, provider, store, counts, frames, fault = native_named
    assert run('LOC_PARK', 'park')[0]['ok']
    claimed = dispatch(store, provider, action, inputs)
    if action.startswith('oem.z.'):
        result = provider.execute_z_intent(intent, inputs={**inputs, 'command_id': claimed['command_id']},
            expected_generation=7, idempotency_key=claimed['command_id'])
    else:
        y = provider.primitives.y_provider
        if intent == 'home':
            result = y.home('manual_panel', command_id=claimed['command_id'])
        elif intent == 'move_steps':
            result = y.move_steps(inputs['steps'], command_id=claimed['command_id'])
        else:
            result = y.move_absolute(inputs['target_steps'], command_id=claimed['command_id'])
    if action == 'oem.z.set_home':
        # Existing adapter reports SAP1 acknowledgement only in its child.
        # Preserve that failure result; the coordinate reset still invalidates.
        assert result['ok'] is False
        assert result['result']['position']['position'] == 0
    else:
        assert result['ok'], json.dumps(result, indent=2)
    store.finish(claimed['command_id'], status='completed' if result['ok'] else 'failed', payload=result, claimed=claimed)
    assert store.deck_semantic_state()['current_location'] == 'UNKNOWN'
    result, _ = run('LOC_PARK', 'return')
    assert result['ok'] and result['delivery_attempted'], json.dumps(result, indent=2)
    assert run('LOC_PARK', 'repeat')[0]['delivery_attempted'] is False


def test_dispatch_piggybacks_one_transaction_no_snapshot_or_owner_publication(native_named, monkeypatch):
    run, provider, store, counts, frames, fault = native_named
    assert run('LOC_PARK', 'park')[0]['ok']
    snapshot_count = store.connection.execute('SELECT COUNT(*) FROM serial206_authority_snapshots').fetchone()[0]
    monkeypatch.setattr(provider, '_deck_semantic_state_publisher', lambda **kw: pytest.fail('new blocking publication'))
    sql = []
    store.connection.set_trace_callback(sql.append)
    dispatch(store, provider, 'oem.x.move_absolute', {'position_steps': 12000})
    store.connection.set_trace_callback(None)
    # Admission and dispatch each already own exactly one durable write fence.
    assert sum(s == 'BEGIN IMMEDIATE' for s in sql) == 2
    assert sum(s == 'COMMIT' for s in sql) == 2
    assert store.connection.execute('SELECT COUNT(*) FROM serial206_authority_snapshots').fetchone()[0] == snapshot_count


def test_recording_failure_is_not_motion_gate(native_named):
    run, provider, store, counts, frames, fault = native_named
    assert run('LOC_PARK', 'park')[0]['ok']
    store.connection.execute("CREATE TEMP TRIGGER fail_location BEFORE UPDATE ON operator_plane_deck_semantic_state BEGIN SELECT RAISE(ABORT,'injected bookkeeping failure'); END")
    # Don't change physical schema/migrations; this connection-local test trigger
    # raises only inside the invalidation savepoint.
    claimed = dispatch(store, provider, 'oem.x.move_absolute', {'position_steps': 12000})
    assert store.deck_semantic_state()['current_location'] == 'UNKNOWN'
    assert provider.state_store.read_oem_serial206_initialization_state()['machine_status']['current_location'] == 32
    frames.clear()
    result = provider.execute_x_intent('move_absolute', {'position_steps': 12000,
        'command_id': claimed['command_id'], 'idempotency_key': claimed['command_id'], 'expected_generation': 7})
    assert result['ok'] and any(command == 4 for _, command, _, _, _ in frames)
    store.connection.execute('DROP TRIGGER fail_location')


@pytest.mark.parametrize('mode', ['xsteps', 'xy', 'homexy'])
def test_legacy_dispatch_transaction_invalidates_before_native_delivery(native_named, mode):
    from bioxp.operator_receipt_store import OperatorReceiptStore
    run, provider, store, counts, frames, fault = native_named
    assert run('LOC_PARK', 'park')[0]['ok']
    legacy = OperatorReceiptStore(root=store.root)
    row = dict(schema_version='bioxp.operator_action_receipt.v1', command_id='legacy-x',
        action_id='oem.x.move_steps', kind='motion', safety_class='motion',
        status='admission_pending', idempotency_key='legacy-x', ownership_generation=7,
        inputs={'steps': 1000}, requested_inputs={'steps': 1000}, started_at='1',
        physical_effect_verified=False, remote_acknowledged=False, controller_acknowledged=False)
    claimed, created = legacy.claim(row)
    assert created
    row['status'] = 'queued'
    legacy.put(row, _expected_status=claimed['status'])
    assert store.deck_semantic_state()['current_location'] == 'LOC_PARK'
    row['status'] = 'dispatched'
    legacy.put(row, _expected_status='queued', _invalidate_deck_location=True)
    assert store.connection.execute('SELECT current_location FROM operator_plane_deck_semantic_state').fetchone()[0] == 'UNKNOWN'
    assert provider.state_store.read_oem_serial206_initialization_state()['machine_status']['current_location'] == 32
    values = {'command_id': 'legacy-x', 'idempotency_key': 'legacy-x', 'expected_generation': 7}
    provider.y_provider = provider.primitives.y_provider
    if mode == 'xy':
        result = provider.execute_xy_intent(12000, 12000, values)
    elif mode == 'homexy':
        result = provider.execute_homexy_intent(values)
    else:
        result = provider.execute_x_intent('move_steps', {'steps': 1000, **values})
    assert result['ok'], json.dumps(result, indent=2)
    row['status'] = 'completed'
    legacy.put(row, _expected_status='dispatched')
    result, _ = run('LOC_PARK', 'return')
    assert result['ok'] and result['delivery_attempted']
    legacy._audit_database.close()
