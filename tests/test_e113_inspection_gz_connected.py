"""Controlled physical exchange/events, not recovered incident wire.

Real finite parent/children, provider, production adapter, native driver,
NovoRouter and SQLite finalization. No success/wait/planner/store doubles.
"""
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

import bioxp.usb_driver as usb
import bioxp.oem_serial206_initialization as mod
from bioxp.oem_deck_movement import make_wp8_operation_executor
from tests.test_debloat_core_inspection import retained_rig
from tests.test_wake_setup_debloat import rig
from tests.test_cover_carry_release_connected import connected
from tests.protocol_v1_integration_fixture import NativePhysicalRecorder
from tests.test_manual_park_context_connected import Clock, no_hardware
from tests.test_motor_receive_identity import receive


@pytest.fixture
def native_inspection(connected, monkeypatch):
    r = connected
    clock = Clock()
    monkeypatch.setattr(usb, 'time', clock)
    monkeypatch.setattr(mod, 'time', clock)
    native = NativePhysicalRecorder(monkeypatch)
    native.positions = dict(r.native.positions)
    native.door_open_position = r.provider._wp8_door_config()['open']
    scenario = {'variant': 'before_ack', 'hit': False}
    raw = []
    move_to_raw = []
    real = native.exchange

    def exchange(board, command, typ, motor, value, **kw):
        address = board, motor
        if (scenario['variant'] == 'same_position' and command == 6 and typ == 1
                and address == (4, 1) and native.positions[5, 0] == 1506
                and native.positions[4, 0] == 71):
            native.positions[address] = 114092
            scenario['hit'] = True
        if command == 138:
            native.trace.append((board, command, typ, motor, value))
            return {'status': 100, 'value': 0}
        if command == 4 and typ == 0:
            native.trace.append((board, command, typ, motor, value))
            selected = address == (4, 1) and (value == 114092 or (
                scenario['variant'] == 'partial_failure' and value == 107496))
            variant = scenario['variant'] if selected else 'before_ack'
            if selected:
                scenario['hit'] = True
            native.positions[address] = 67110 if selected and variant in (
                'unequal', 'late', 'wrong_motor', 'old_generation', 'fault', 'stop', 'partial_failure') else value
            if variant == 'before_ack':
                receive(native.tester, board=board, motor=motor)
            elif variant == 'after_ack':
                clock.pending.append((clock.now + .02, lambda: receive(native.tester, board=board, motor=motor)))
            elif variant == 'late':
                clock.pending.append((clock.now + 21, lambda: receive(native.tester, board=board, motor=motor)))
            elif variant == 'wrong_motor':
                receive(native.tester, board=board, motor=2)
            elif variant == 'old_generation':
                router = native.tester.novo_router
                frame = receive(native.tester, board=board, motor=motor)
                router.shutdown()
                router._dispatch(frame)
            elif variant == 'fault':
                router = native.tester.novo_router
                # status130 axis byte2 of CAN payload, not target byte6.
                body = bytes([0, 0, 0, 0, 8, board, 130, motor, 0, 0, 0, 2, 0])
                encoded = bytes(usb.novo_encode(body))
                router._dispatch(router._decode_record(usb.novo_decode(encoded), encoded, router._clock()))
            elif variant == 'stop':
                native.tester.motor_oem_force_abort_motion(reason='controlled_inspection_stop')
            return {'status': 100, 'value': value}
        # Source motor parameter writes/readbacks at the physical seam.
        if command in (5, 6) and typ not in (1, 3, 9, 10, 12, 13):
            native.trace.append((board, command, typ, motor, value))
            if command == 5:
                native.axis_parameters[board, typ, motor] = value
            return {'status': 100, 'value': native.axis_parameters.get((board, typ, motor), 0)}
        return real(board, command, typ, motor, value, **kw)

    monkeypatch.setattr(native.tester, '_send_motor', exchange)
    monkeypatch.setattr(native.tester, 'send_tmcl_retry', exchange)
    refs = r.provider.reference_store
    adapter = mod.Serial206ProductionPrimitiveAdapter(native.tester, None,
        authority_provider=lambda: {}, generation_provider=lambda: 3, reference_store=refs)
    from bioxp.serial206_y_provider import Serial206YProvider
    adapter.y_provider = Serial206YProvider(native.tester, state_store=r.provider.state_store,
        generation_provider=lambda: 3, reference_store=refs)
    r.provider.primitives = adapter
    original_move_to = adapter.oem_move_to
    def observe_move_to(*a, **kw):
        result = original_move_to(*a, **kw)
        move_to_raw.append(result)
        return result
    monkeypatch.setattr(adapter, 'oem_move_to', observe_move_to)
    original = native.tester.motor_oem_move_absolute
    def observe(*a, **kw):
        try:
            result = original(*a, **kw)
        except usb.OemMotionCompletionError as exc:
            raw.append(exc.motion_evidence)
            raise
        raw.append(result)
        return result
    monkeypatch.setattr(native.tester, 'motor_oem_move_absolute', observe)
    r.actual, r.clock, r.scenario, r.raw, r.move_to_raw = native, clock, scenario, raw, move_to_raw
    yield r
    for task in r.provider._wp8_tasks.values():
        task['thread'].join(timeout=2)
    native.close()


def execute_parent(r):
    store, provider = r.store, r.provider
    stamps = provider.deck_owner_authority_stamps()
    state = {'ownership_generation': 3, 'serial206_initialization_provider': {
        'x_authority': {'current_board_lifecycle_generation': stamps['board_epoch_5']},
        'board4_authority': {'active_board_epoch': stamps['board_epoch_4']}}}
    parent = 'controlled-inspection-parent'
    store.bind_workflow_dispatcher(lambda command: None)
    store.admit_workflow(command_id=parent, idempotency_key=parent,
        plan_fingerprint='controlled-inspection', requested_inputs={'bundle': {'execution': {'runtime_state': {}}}},
        ownership_generation=3, resources=('axis:x', 'axis:y', 'axis:z', 'gripper'), board_epochs={})
    assert store.claim_next()['command_id'] == parent
    with store.workflow_context(parent, source_occurrence_id='inspection:0'):
        child = store.admit_internal_wp8_operation('cover_inspection', inputs={
            'deck_inspection': True, 'screen_resolution_high': False, 'inspection_log_only': False},
            state=state, idempotency_key='controlled-inspection-finite', prepared_plan=r.plan)
    cid = child['command_id']
    claimed = store.claim_next()
    assert claimed['command_id'] == cid
    r.cid = cid
    try:
        result = make_wp8_operation_executor(provider_getter=lambda: provider, command_store=store)(
            command_id=cid, plan=r.plan)
    except Exception as exc:
        result = {'ok': False, 'exception_type': type(exc).__name__, 'exception': str(exc)}
    for task in provider._wp8_tasks.values():
        task['thread'].join(timeout=2)
    store.finish(cid, status='completed' if result['ok'] else 'ambiguous', payload={'response': result}, claimed=claimed)
    store.finish_workflow(parent, status='completed' if result['ok'] else 'ambiguous', payload={}, lifecycle_settled=True)
    evidence = store.wp8_operation_evidence(cid)
    code = 'import sqlite3,json,sys; c=sqlite3.connect(sys.argv[1]); r=c.execute("SELECT status,receipt_json FROM operator_commands WHERE command_id=?",(sys.argv[2],)).fetchone(); print(json.dumps(list(r)))'
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', code,
        str(Path(r.root)/'bioxp_runtime.db'), cid], text=True))
    assert reopened[0] == ('completed' if result['ok'] else 'ambiguous')
    if os.environ.get('E113_INSPECTION_EXPORT'):
        out = Path(os.environ['E113_INSPECTION_EXPORT'])
        out.mkdir(parents=True, exist_ok=True)
        (out/(r.scenario['variant']+'.json')).write_text(json.dumps({
            'controlled': True, 'candidate_provider_overlay': bool(os.environ.get('E113_INSPECTION_PROVIDER_HUNK')),
            'result': result, 'evidence': evidence, 'raw_native': r.raw, 'raw_move_to': r.move_to_raw,
            'wire_commands': r.actual.trace, 'custody': store.deck_semantic_state(), 'reopened': reopened}, indent=2))
    return result, evidence


@pytest.mark.parametrize('variant', ['before_ack', 'after_ack', 'ack_without_target', 'unequal',
    'late', 'wrong_motor', 'old_generation', 'fault', 'stop', 'same_position', 'partial_failure'])
def test_connected_inspection_gz_variants(native_inspection, variant):
    r = native_inspection
    r.scenario['variant'] = variant
    result, evidence = execute_parent(r)
    assert r.scenario['hit'], result
    assert result['ok'] is (variant in ('before_ack', 'after_ack', 'ack_without_target', 'same_position')), result
    rows = evidence['children']
    relocation = next(x for x in rows if x['child_order'] == 8)
    assert relocation['terminal_state'] == ('completed' if result['ok'] else 'ambiguous')
    state = r.store.deck_semantic_state()
    if variant == 'partial_failure':
        assert state['movable_plate_locations']['OUTPUT_COVER'] == 'LOC_OC_COVER_STORAGE'
        assert state['movable_plate_locations']['REAGENT_COVER'] == 'LOC_RC_COVER'
        assert state['plate_on_gantry'] is None
        assert not any(x == (4, 4, 0, 1, 114092) for x in r.actual.trace)
        assert not any(x['child_order'] > 8 and x['terminal_state'] == 'completed' for x in rows)
        return
    # The injected final Park failure is AFTER both real cover releases.
    assert state['movable_plate_locations'] == {'OUTPUT_COVER': 'LOC_OC_COVER_STORAGE',
        'REAGENT_COVER': 'LOC_RC_COVER_STORAGE'}
    assert state['plate_on_gantry'] is None
    if variant == 'same_position':
        z = next(x for x in r.raw if x.get('requested_position') == 114092)
        assert z['source_noop'] is True
        assert z['source_return_code'] == 114092
        assert z['ack'] is None
        assert z['command_sent'] is False
        assert not any(x == (4, 4, 0, 1, 114092) for x in r.actual.trace)
        return
    if variant == 'stop':
        assert result['exception'] == 'Lost 24V power move abs2. moveToAbs()'
        assert r.actual.tester.oem_no24v_state() is True
        assert r.actual.tester._oem_user_stopped is True
        return
    z = next(x for x in r.raw if x.get('requested_position') == 114092)
    assert z['ack']['status'] == 100
    assert z['wire_position'] == 114092
    if variant in ('before_ack', 'after_ack'):
        assert z['wait']['ok'] is True
    elif variant != 'stop':
        assert z['wait']['ok'] is False
        if variant != 'fault':
            assert z['wait']['elapsed_ms'] == 20000
        assert z['timeout_position']['position'] == (114092 if variant == 'ack_without_target' else 67110)
        if variant == 'ack_without_target':
            assert z['completion_class'] == 'oem_timeout_target_equal'
            assert z['source_return_code'] == 114092
        else:
            assert 'Reach GZ position time out' in result['exception']
    if variant == 'fault':
        events = r.actual.tester.collect_bus_events(duration_s=0)
        stall = next(x for x in events if x.get('status') == 130)
        assert stall['motor'] == 1
        assert stall['board'] == 4
    if variant == 'late':
        before = r.store.wp8_operation_evidence(r.cid)
        r.clock.sleep(2)
        arrival = r.actual.tester.motor_oem_wait_target_reached(4, motor=1, timeout_s=0)
        assert arrival['ok'] is True
        assert arrival['event']['command_correlation'] == 'unavailable_on_wire'
        assert r.store.wp8_operation_evidence(r.cid) == before
    assert len([x for x in r.actual.trace if x == (4, 4, 0, 1, 114092)]) == 1
