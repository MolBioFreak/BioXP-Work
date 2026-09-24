"""Offline source-path debloat through real provider/adapters and SQLite.

The connected fixture replaces only physical native seams. Parallel XY below
also executes BioXpTester.motor_oem_move_absolute, not a successful move stub.
"""
from collections import Counter

import pytest

from bioxp.oem_deck_movement import compile_finite_plate_operation
from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from tests.test_cover_carry_release_connected import connected
from tests.test_deck_scoped_authority import retained_rig, rig
from tests.test_deck_complete_effective import BoardLeaf


@pytest.mark.parametrize('door,plate,destination,error', [
    (False, 4, 18, None), (True, 4, 18, 'thermal_door_must_be_closed'),
    (False, 3, 5, 'thermal_door_must_be_open'), (True, 3, 5, None),
    (None, 4, 18, 'source_authority_missing:thermal_door_open'),
])
def test_outer_move_only_reads_consumed_door_state(rig, monkeypatch, door, plate, destination, error):
    provider, primitive, _, state = rig
    state['machine_status']['thermal_door_open'] = door
    def unexpected():
        pytest.fail('outer move_plate must not rebuild routing/table/gripper state')
    monkeypatch.setattr(provider, 'mov_execution_machine_state', unexpected)
    inputs = {'plate': plate, 'destination': destination}
    machine = provider.wp8_operation_machine_state('move_plate', inputs)
    assert machine == {'thermal_door_open': door}
    assert primitive.calls == []
    if error:
        with pytest.raises(RuntimeError, match=error):
            compile_finite_plate_operation('move_plate', source_leaf_available=True, **machine, **inputs)
    else:
        plan = compile_finite_plate_operation('move_plate', source_leaf_available=True, **machine, **inputs)
        assert [c['operation'] for c in plan['children']] == ['catchPlate', 'releasePlate']


@pytest.mark.parametrize('open_fact', ['latch_status', 'machine_latch_closed'])
def test_real_manual_latch_denials_still_block_before_motion(rig, monkeypatch, open_fact):
    from bioxp.oem_deck_movement import DeckAuthoritySnapshot, NamedLocationIntent, compile_named_location
    from bioxp.oem_deck_catalog import DeckCatalog
    from bioxp.oem_compat.position_table import load_bound_oem_position_table
    provider, primitive, _, _ = rig
    if open_fact == 'latch_status':
        monkeypatch.setattr(primitive, 'read_oem_latch_status',
            lambda: {'ok': True, 'value': False, 'observation_id': 'offline-open-host'})
    else:
        monkeypatch.setattr(primitive, 'query_latch', lambda: {'ok': True, 'value': 0})
    authority = DeckAuthoritySnapshot(**provider.deck_authority_snapshot(expected_generation=3, target='LOC_OC'))
    assert getattr(authority, open_fact) is False
    table = load_bound_oem_position_table()
    plan = compile_named_location(NamedLocationIntent(target='LOC_OC'),
        DeckCatalog.from_position_table(table), table, authority)
    assert plan.blocked_reason == 'latch_not_closed'
    assert not any(step.operation == 'moveTo' for step in plan.steps)
    assert not any(row[0] == 'move' for row in primitive.calls)


def join_tasks(provider):
    for row in provider._wp8_tasks.values():
        row['thread'].join(timeout=2)
        assert not row['thread'].is_alive()
        assert row['state'] == 'completed'


@pytest.mark.parametrize('owner', ['null', 'noncallable', 'callable'])
def test_consecutive_native_transfers_keep_live_custody_and_conditional_settle(connected, monkeypatch, owner):
    r = connected
    provider = r.provider
    calls, sleeps, states = [], [], []
    original_sleep = provider.primitives.sleep
    monkeypatch.setattr(provider.primitives, 'sleep',
        lambda milliseconds: sleeps.append(milliseconds / 1000) or original_sleep(milliseconds))
    provider.primitives.wp8_led2_set = (
        (lambda on: calls.append(on) or {'ok': True}) if owner == 'callable'
        else (None if owner == 'null' else object()))
    original = provider.wp8_operation_machine_state
    def observe(operation, inputs):
        result = original(operation, inputs)
        states.append((operation, result))
        return result
    monkeypatch.setattr(provider, 'wp8_operation_machine_state', observe)
    for ordinal, destination in enumerate((20, 19)):
        result = provider._wp8_compile_and_execute(operation='move_plate',
            inputs={'plate': 5, 'destination': destination, 'run_in_parallel': True},
            command_id=f'offline-debloat-{ordinal}', owner_identity=r.owner)
        assert result['ok'] is True
        join_tasks(provider)
        for transfer in result['source_children']:
            children = transfer['result']['source_children']
            led_index = next(i for i, row in enumerate(children) if row['operation'] == 'led2On')
            led, settle = children[led_index:led_index + 2]
            assert led['result']['led_owner_present'] is (owner == 'callable')
            assert settle['operation'] == 'Sleep'
            assert settle['result'].get('source_branch_skipped', False) is (owner != 'callable')
            assert any(row['operation'] == 'SnapshotImage' for row in children)
        semantic = r.store.deck_semantic_state()
        assert semantic['plate_on_gantry'] is None
        assert semantic['movable_plate_locations']['REAGENT_COVER'] == (
            'LOC_RC_COVER_STORAGE' if destination == 20 else 'LOC_RC_COVER')
    assert calls == ([True, False] * 4 if owner == 'callable' else [])
    assert [row['plate_location'] for op, row in states if op == 'catch_plate'] == [19, 20]
    assert [row['current_tray'] for op, row in states if op == 'release_plate'] == [5, 5]
    assert all(set(row) == {'thermal_door_open'} for op, row in states if op == 'move_plate')
    assert sleeps.count(0.1) == 6  # snapshot and post-lowering source waits remain
    assert sleeps.count(0.2) == 4
    assert sleeps.count(1.0) == (6 if owner == 'callable' else 2)  # two source G/Z home interleaves


def test_three_native_moves_after_real_publication_need_no_latch_record(connected):
    from bioxp.oem_deck_movement import (
        ClassMoveToIntent, compile_mov_execution, bind_mov_execution_script_plan,
        execute_mov_execution,
    )
    r = connected
    provider, store = r.provider, r.store
    stamps = provider.deck_owner_authority_stamps()
    state = {'ownership_generation': stamps['ownership_generation'],
        'serial206_initialization_provider': {
            'x_authority': {'current_board_lifecycle_generation': stamps['board_epoch_5']},
            'board4_authority': {'active_board_epoch': stamps['board_epoch_4']}}}
    initial_inventory = store.deck_semantic_state()['movable_plate_locations']
    for line, plate in enumerate((4, 5, 4), 1):
        intent = ClassMoveToIntent(line, plate_name=plate, well='A1')
        admitted = store.admit_internal_mov_execution(intent, state=state,
            idempotency_key=f'offline-native-sequential-{line}')
        claimed = store.claim_next()
        assert claimed['command_id'] == admitted['command_id']
        machine = provider.mov_execution_machine_state()
        plan = compile_mov_execution(intent, machine)
        preview = provider.preview_scriptmove_to(plan.steps[0].arguments)
        plan = bind_mov_execution_script_plan(plan, preview['plan'])
        result = execute_mov_execution(admitted['command_id'], plan,
            provider=provider, command_store=store)
        assert result['ok'] is True, result
        store.finish(admitted['command_id'], status='completed', payload=result, claimed=claimed)
        semantic = store.deck_semantic_state()
        # Real publish_mov_execution_transition replaces provenance without
        # latch overlays. No value is fabricated/restamped to run action 2/3.
        assert semantic['latch_status'] is None
        assert semantic['machine_latch_closed'] is None
        assert semantic['latch_observation_id'] is None
        assert semantic['movable_plate_locations'] == initial_inventory
        assert semantic['current_location'] == ('LOC_OC_COVER' if plate == 4 else 'LOC_RC_COVER')
        assert semantic['producer_operation'] == 'updatePlateLocation'
        with pytest.raises(RuntimeError, match='deck_semantic_state_not_authoritative:latch_status'):
            provider._canonical_deck_semantic_state()  # full/manual consumer unchanged
    assert any(event[:2] == ('move', 'x') for event in r.native.events)


@pytest.mark.parametrize('failure', ['return_false', 'raise'])
def test_led_failure_keeps_existing_catch_failure_and_unlock(connected, monkeypatch, failure):
    r = connected
    sleeps = []
    original_sleep = r.provider.primitives.sleep
    monkeypatch.setattr(r.provider.primitives, 'sleep',
        lambda milliseconds: sleeps.append(milliseconds / 1000) or original_sleep(milliseconds))
    def failed(on):
        if failure == 'raise':
            raise RuntimeError('offline_led_failure')
        return {'ok': False}
    r.provider.primitives.wp8_led2_set = failed
    with pytest.raises(RuntimeError, match='wp8_nested_child_failed:catchPlate'):
        r.provider._wp8_compile_and_execute(operation='move_plate',
            inputs={'plate': 5, 'destination': 20}, command_id='offline-led-failure', owner_identity=r.owner)
    assert 1.0 not in sleeps
    assert r.store.deck_semantic_state()['movable_plate_locations']['REAGENT_COVER'] == 'LOC_RC_COVER'
    assert r.store.deck_semantic_state()['plate_on_gantry'] is None
    assert r.provider._wp8_gripper_lock.acquire(blocking=False)
    r.provider._wp8_gripper_lock.release()


class CountedBoard(BoardLeaf):
    def __init__(self, before, fault=None):
        super().__init__(before, fault)
        self.reads = Counter()
        self.windows = []
    def motor_get_position(self, board, motor=0):
        self.reads[board, motor] += 1
        return super().motor_get_position(board, motor)
    def collect_bus_events(self, **kwargs):
        self.windows.append(kwargs)
        if self.fault == 'controller_error':
            return [{'board': 4, 'motor': 0, 'status': 130, 'event_sequence': 2,
                     'receive_owner': 'offline-usb', 'owner_generation': 3}]
        return []


def xy_adapter(leaf):
    return Serial206ProductionPrimitiveAdapter(leaf, None,
        authority_provider=lambda: {}, generation_provider=lambda: 3)


@pytest.mark.parametrize('before,target', [((1000, 1000), (4000, 5000)),
    ((1000, 0), (4000, -1687)), ((0, 0), (-1000, -1687)), ((4000, 5000), (4000, 5000))])
def test_xy_native_read_counts_movement_and_exact_noop(monkeypatch, before, target):
    monkeypatch.setattr('bioxp.oem_serial206_initialization.time.sleep', lambda _: None)
    leaf = CountedBoard(before)
    result = xy_adapter(leaf).move_xy(*target, wait_timeout_s=5,
        source_context='ClassControlInterface.btnLOC1_Click')
    assert result['ok'] is True
    if before == target:
        assert result['source_noop'] is True
        assert not leaf.moves
        assert leaf.reads == {(5, 0): 1, (4, 0): 1}
        assert leaf.windows == []
    else:
        # CI source delta read + fresh board pre-TX read + one final read.
        assert leaf.reads == {(5, 0): 3, (4, 0): 3}
        assert result['controller_terminal_state_verified'] is True
        assert leaf.windows == [{'duration_s': 0.0, 'timeout_ms': 12, 'max_events': 128}]
        assert result['after'] == {'x': max(0, target[0]), 'y': max(0, target[1])}
        assert result['commands']['y']['before'] == result['commands']['y']['move']['before']


def test_zero_duration_collector_retains_buffered_terminal_and_error(monkeypatch):
    """Real bus decoder/snapshot path; only the receive queue is synthetic."""
    from types import SimpleNamespace
    from bioxp.usb_driver import BioXpTester

    frames = []
    for sequence, status in enumerate((128, 130), start=1):
        payload = [0, 0, 0, 0, 8, 4, status, 0, 0, 0, 0, 0, 0]
        raw = [126, *payload, sum(payload) & 255, 126]
        frames.append({'raw': raw, 'receive_sequence': sequence,
                       'receive_owner': 'offline-receiver', 'owner_generation': 3})
    tester = object.__new__(BioXpTester)
    tester.novo_router = SimpleNamespace(
        queue_snapshot=lambda name: frames if name == 'valid_async' else [])
    monkeypatch.setattr('bioxp.usb_driver.time.sleep',
                        lambda seconds: pytest.fail('post-terminal collector must not sleep'))
    events = tester._collect_bus_events_locked(duration_s=0.0, timeout_ms=12, max_events=128)
    assert [(row['board'], row['motor'], row['status']) for row in events] == [
        (4, 0, 128), (4, 0, 130)]
    assert [row['event_sequence'] for row in events] == [1, 2]
    assert len(frames) == 2  # snapshot, not consuming/clearing the receive queue


@pytest.mark.parametrize('fault', ['missing_event', 'missing_ack', 'wrong_position', 'moving', 'stale', 'controller_error'])
def test_xy_failure_evidence_is_not_a_new_source_success_gate(monkeypatch, fault):
    monkeypatch.setattr('bioxp.oem_serial206_initialization.time.sleep', lambda _: None)
    leaf = CountedBoard((1000, 1000), fault)
    result = xy_adapter(leaf).move_xy(4000, 5000, wait_timeout_s=5,
        source_context='ClassControlInterface.btnLOC1_Click')
    assert result['ok'] is True  # unchanged source-call completion, not proof
    assert not (result['controller_terminal_state_verified'] and result['target_position_verified']
                and result['controller_command_acknowledged'])
    if fault == 'controller_error':
        assert result['axis_evidence']['y']['controller_error_events'][0]['status'] == 130


@pytest.mark.parametrize('failure', ['missing_position', 'no24v', 'not_initialized'])
def test_y_issue_native_failure_still_has_no_motor_delivery(monkeypatch, failure):
    leaf = CountedBoard((1000, 1000))
    if failure == 'missing_position':
        monkeypatch.setattr(leaf, 'motor_get_position', lambda *a, **k: {'ok': False})
    elif failure == 'no24v':
        monkeypatch.setattr(leaf, 'oem_no24v_state', lambda: True)
    else:
        monkeypatch.setattr(leaf, '_oem_board_state', lambda: {})
    adapter = xy_adapter(leaf)
    if failure == 'no24v':
        with pytest.raises(RuntimeError, match='Lost 24V'):
            adapter._move_xy_y_issue_absolute(5000, event_window=leaf.begin_bus_event_window())
    else:
        result = adapter._move_xy_y_issue_absolute(5000, event_window=leaf.begin_bus_event_window())
        assert result['ok'] is False
        if failure == 'missing_position':
            assert result['command_issued'] is False
    assert not leaf.moves


@pytest.mark.parametrize('row,message', [({'ok': False}, 'axis position readback failed'),
    ({'ok': True, 'position': None}, 'axis position readback is not an integer')])
def test_parallel_final_read_retains_existing_read_error(monkeypatch, row, message):
    monkeypatch.setattr('bioxp.oem_serial206_initialization.time.sleep', lambda _: None)
    leaf = CountedBoard((1000, 1000))
    original = leaf.motor_get_position
    def read(board, motor=0):
        if leaf.moves and len(leaf.moves) == 2:
            return row
        return original(board, motor)
    monkeypatch.setattr(leaf, 'motor_get_position', read)
    with pytest.raises(RuntimeError, match=message):
        xy_adapter(leaf).move_xy(4000, 5000, wait_timeout_s=5,
            source_context='ClassControlInterface.btnLOC1_Click')
