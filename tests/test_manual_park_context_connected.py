"""Offline Park -> planner -> executor -> adapter -> native driver -> router.

Only board exchange and time are synthetic. Retained state and the real pipette
query publisher supply provider inputs; no motion/receipt completion is mocked.
"""
import time
from types import SimpleNamespace

import pytest

from bioxp import oem_machine_bundle
import bioxp.oem_serial206_initialization as mod
import bioxp.usb_driver as usb
from tests.protocol_v1_integration_fixture import NativePhysicalRecorder
from tests.test_deck_scoped_authority import retained_rig, qualify_test_references
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig, query
from tests.test_motor_receive_identity import receive


class Clock:
    def __init__(self):
        self.now = 1000.0
        self.pending = []

    def __getattr__(self, name):
        return getattr(time, name)

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        ready = [item for item in self.pending if item[0] <= self.now]
        self.pending = [item for item in self.pending if item[0] > self.now]
        for _, callback in ready:
            callback()


@pytest.fixture(autouse=True)
def no_hardware(monkeypatch):
    import socket
    original = socket.socket
    def socket_without_network(family=socket.AF_INET, *args, **kwargs):
        if family != socket.AF_UNIX:
            raise AssertionError('offline Park test cannot open network transport')
        return original(family, *args, **kwargs)
    monkeypatch.setattr(socket, 'socket', socket_without_network)
    def closed(*args, **kwargs):
        raise AssertionError('offline Park test cannot open physical transport')
    import usb.core
    monkeypatch.setattr(usb.core, 'find', closed)
    try:
        import serial
    except ImportError:
        pass
    else:
        monkeypatch.setattr(serial, 'Serial', closed)


@pytest.fixture
def park(query_rig, monkeypatch):
    query(query_rig)
    _, provider, observations, refs, *_ = query_rig
    qualify_test_references(refs)
    snapshot = oem_machine_bundle.get_active_oem_machine_snapshot()
    snapshot = oem_machine_bundle.load_oem_machine_snapshot(
        snapshot.bundle_root / 'OEM_EVIDENCE_LOCK.json', operator_label_serial=206,
        require_operator_label=True)
    monkeypatch.setattr(oem_machine_bundle, '_active_snapshot', snapshot)
    clock = Clock()
    monkeypatch.setattr(mod, 'time', clock)
    monkeypatch.setattr(usb, 'time', clock)
    native = NativePhysicalRecorder(monkeypatch)
    native.positions = {(5, 0): 90213, (4, 0): 93211, (4, 1): 500, (4, 2): 0}
    scenario = {'x': 0.1, 'y': 0.1, 'z': 0.1, 'z_position': None}
    real_exchange = native.exchange

    def exchange(board, command, typ, bank, value, **kwargs):
        address = (board, bank)
        axis = {(5, 0): 'x', (4, 0): 'y', (4, 1): 'z'}.get(address)
        if axis and command in (4, 138):
            assert typ == 0
            native.trace.append((board, command, typ, bank, value))
            if command == 4:
                native.positions[address] = (
                    scenario['z_position'] if axis == 'z' and scenario['z_position'] is not None else value)
                delay = scenario[axis]
                if delay is not None:
                    clock.pending.append((clock.now + delay,
                        lambda b=board, m=bank: receive(native.tester, board=b, motor=m)))
            return {'status': 100, 'value': value}
        return real_exchange(board, command, typ, bank, value, **kwargs)

    monkeypatch.setattr(native.tester, '_send_motor', exchange)
    monkeypatch.setattr(native.tester, 'send_tmcl_retry', exchange)
    waits = []
    real_wait = native.tester._wait_router_motor_events

    def observe_wait(targets, timeout_s, event_window):
        start = clock.now
        result = real_wait(targets, timeout_s, event_window)
        waits.append((tuple(targets), timeout_s, clock.now - start, result))
        return result

    monkeypatch.setattr(native.tester, '_wait_router_motor_events', observe_wait)
    adapter = mod.Serial206ProductionPrimitiveAdapter(native.tester, None,
        authority_provider=lambda: {}, generation_provider=lambda: 3, reference_store=refs)
    raw = []
    real_move = adapter.oem_move_to

    def observe_move(*args, **kwargs):
        result = real_move(*args, **kwargs)
        raw.append(result)
        return result

    monkeypatch.setattr(adapter, 'oem_move_to', observe_move)
    monkeypatch.setattr(observations, 'oem_initialize_motion_scriptmove_to_waste',
        adapter.oem_initialize_motion_scriptmove_to_waste, raising=False)
    authority = dict(tip_loaded=False, tip_dirty=False, tip_location=-1, clean_path=False,
        pseudo_z_home=500, ownership_generation=3, board_epoch_4=7, board_epoch_5=9,
        current_location_id='LOC_OC', current_well_id=0, machine_state_revision=1,
        semantic_state_provenance_digest='a'*64, plate_on_gantry=None, gripper_confirmed=True,
        collection_tip_state=provider._park_collection_state())
    yield SimpleNamespace(provider=provider, adapter=adapter, native=native,
        scenario=scenario, waits=waits, raw=raw, authority=authority, clock=clock)
    native.close()


def invoke(park, **kwargs):
    return park.provider.parkGantry(authority_snapshot=park.authority, **kwargs)


@pytest.mark.parametrize('x,y,expected', [
    (0.1, 0.1, (True, True)),
    (0.1, None, (True, False)),
    (None, 6.0, (False, True)),
    (None, None, (False, False)),
])
def test_manual_park_sealed_sta_separate_budgets_and_numeric_timeouts(park, x, y, expected):
    park.scenario.update(x=x, y=y)
    from bioxp.oem_deck_movement import DeckExecutionFailure
    if all(expected):
        result = invoke(park)
        assert result["ok"] is True
    else:
        # Preserve existing terminal receipt aggregation, separately from the
        # source numeric XY return: final Z must execute before this report.
        with pytest.raises(DeckExecutionFailure, match="park_source_child_failed"):
            invoke(park)
    xy = park.raw[-1]['operations'][0]
    assert xy['source_context'] == 'ClassControlInterface.btnLOC1_Click'
    assert xy['source_context_sealed'] is True
    assert xy['wait_schedule'] == 'STA_WaitAny_X_then_Y'
    assert [w[:2] for w in park.waits] == [(((5, 0),), 5.0), (((4, 0),), 5.0), (((4, 1),), 20.0)]
    assert tuple(w[3]['ok'] for w in park.waits[:2]) == expected
    assert xy['ok'] is True  # numeric timeout is logged, not thrown
    assert xy['source_calls_completed'] is True
    assert (4, 4, 0, 0, 71) in park.native.trace
    assert (4, 4, 0, 1, 114092) in park.native.trace
    assert park.native.positions[4, 1] == 114092
    assert (5, 5, 5, 0, 350) in park.native.trace
    assert (4, 5, 5, 0, 400) in park.native.trace
    if x is None:
        assert park.waits[0][2] == pytest.approx(5.0)
    if y is None:
        assert park.waits[1][2] == pytest.approx(5.0)


@pytest.mark.parametrize('context,schedule', [(None, 'unsealed_legacy_WaitAll'),
    ('ControlLib.inspectCover', 'MTA_WaitAll')])
def test_explicit_worker_context_keeps_atomic_waitall(park, context, schedule):
    park.scenario.update(x=0.1, y=None)
    from bioxp.oem_deck_movement import DeckExecutionFailure
    with pytest.raises(DeckExecutionFailure, match="park_source_child_failed"):
        invoke(park, source_context=context)
    xy = park.raw[-1]['operations'][0]
    assert xy['source_context'] == context
    assert xy['wait_schedule'] == schedule
    assert park.waits[0][:2] == (((5, 0), (4, 0)), 5.0)
    assert park.waits[0][3]['reached'] == {}
    # A failed WaitAll consumes neither axis. No invented completion evidence.
    assert park.native.tester.motor_oem_wait_target_reached(5, 0, timeout_s=0)['ok']


@pytest.mark.parametrize('position,throws', [(114092, False), (67110, True)])
def test_park_z_timeout_keeps_head_fallback_and_unequal_throw(park, position, throws):
    park.scenario.update(z=None, z_position=position)
    if throws:
        with pytest.raises(usb.OemMotionCompletionError, match='Reach GZ position time out') as caught:
            invoke(park)
        assert '114092' in str(caught.value)
        evidence = caught.value.motion_evidence
        assert evidence['timeout_position']['position'] == 67110
        prior_xy = evidence['prior_operations'][0]
        assert prior_xy['source_context'] == 'ClassControlInterface.btnLOC1_Click'
        assert prior_xy['source_calls_completed'] is True
    else:
        result = invoke(park)
        assert result['ok'] is True
    assert park.waits[-1][0] == ((4, 1),)
    assert park.waits[-1][1:3] == pytest.approx((20.0, 20.0))
    assert park.waits[-1][3]['ok'] is False
    assert (4, 4, 0, 1, 114092) in park.native.trace
    assert park.native.positions[4, 1] == position


def test_step_executor_direct_xy_preserves_explicit_context(park):
    from bioxp.oem_homing_routes import _execute_oem_steps_live
    result = _execute_oem_steps_live([{'op': 'moveXY', 'x': 1506, 'y': 71}], park.adapter,
        wait_timeout_s=60, speed=None, acc=None,
        source_context='ControlLib.inspectCover')
    xy = result['execution_results'][0]['results'][0]['result']
    assert xy['source_context'] == 'ControlLib.inspectCover'
    assert xy['wait_schedule'] == 'MTA_WaitAll'


def test_real_manual_named_dispatch_selects_sta(park, query_rig, monkeypatch):
    from bioxp.oem_deck_movement import make_deck_command_executor
    from bioxp.oem_compat.position_table import load_bound_oem_position_table
    app, provider, observations, *_ = query_rig
    store = app.state.operator_command_plane.store
    monkeypatch.setattr(observations, 'oem_move_to', park.adapter.oem_move_to)
    execute = make_deck_command_executor(provider_getter=lambda: provider,
        position_table_provider=load_bound_oem_position_table, command_store=store)
    generation = provider.generation_provider()
    def named(target):
        stamps = provider.deck_owner_authority_stamps()
        epochs = {'4': stamps['board_epoch_4'], '5': stamps['board_epoch_5']}
        request = dict(schema_version='bioxp.operator_action_request.v2',
            action_id='oem.deck.move_to_location', expected_ownership_generation=generation,
            expected_board_epoch_by_board=epochs, idempotency_key='manual-' + target,
            inputs={'target': target, 'camera_offset': False})
        admitted = store.admit_command(request, state={'ownership_generation': generation,
            'serial206_initialization_provider': {
                'x_authority': {'current_board_lifecycle_generation': epochs['5']},
                'board4_authority': {'active_board_epoch': epochs['4']}}}, assessment={'enabled': True})
        claimed = store.claim_next()
        assert claimed['command_id'] == admitted['command_id']
        result = execute(command_id=admitted['command_id'], target=target, camera_offset=False,
            expected_ownership_generation=generation, expected_board_epoch_by_board=epochs)
        assert result['ok'] is True
        store.finish(admitted['command_id'], status='completed', payload=result, claimed=claimed,
            controller_acknowledged=result['controller_command_acknowledged'], full_response=result)
        return result
    named('LOC_OC')
    query(query_rig, key='after-manual-oc')
    result = named('LOC_PARK')
    assert result['source_branch'] == 'park'
    assert park.raw[-1]['operations'][0]['source_context'] == 'ClassControlInterface.btnLOC1_Click'
    assert park.raw[-1]['operations'][0]['wait_schedule'] == 'STA_WaitAny_X_then_Y'
    assert store.deck_semantic_state()['current_location'] == 'LOC_PARK'
    assert park.native.positions[4, 1] == 114092


def test_wp8_worker_entry_does_not_inherit_manual_default(park, monkeypatch):
    # Observe the actual wrapper's call, without replacing Park's body.
    captured = []
    real = park.provider.parkGantry
    def observe(**kwargs):
        captured.append(kwargs)
        return real(authority_snapshot=park.authority, **kwargs)
    monkeypatch.setattr(park.provider, 'parkGantry', observe)
    # This standalone provider probe has no WP8 child claim to publish under.
    monkeypatch.setattr(park.provider, 'wp8_update_location', lambda *a, **kw: None)
    park.provider.wp8_park_gantry('parkGantry', {'rehome': False})
    assert captured[0]['source_context'] is None
    assert park.raw[-1]['operations'][0]['wait_schedule'] == 'unsealed_legacy_WaitAll'
