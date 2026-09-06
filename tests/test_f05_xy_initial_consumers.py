"""Reachable F05 X/Y consumers: real adapter, primitives and router, fake wire only."""
import pytest

from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from bioxp.serial206_y_provider import Serial206YProvider
from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot
from test_motor_receive_identity import setup_driver, receive


class MemoryReference:
    def __init__(self):
        self.motions = []

    def snapshot(self, axes):
        return {'ok': True, 'durable_clean': True, 'authority_untrusted': False, 'rows': {}}

    def record_motion(self, axis, kind):
        self.motions.append((axis, kind))
        return {'ok': True}


class MemoryObservations:
    def __init__(self):
        self.observations = []

    def board4_authority_projection(self):
        return {'board': {'state': 'active', 'active_board_epoch': 1}, 'axes': {}}

    def record_axis_observation(self, axis, **row):
        self.observations.append((axis, row))
        return {'ok': True, 'discrepancy_steps': row['observed_position_steps'] - row['requested_position_steps']}


def xy_adapter(monkeypatch, mode='initial'):
    bind_serial206_oem_snapshot(monkeypatch)
    driver = setup_driver(initial_signaled=True)
    driver._oem_board_initialized = {4: True, 5: True}
    registers = {(board, 0, 1): 1000 for board in (4, 5)}
    calls, received = [], []

    def wire(board, cmd, param, motor, value, **kwargs):
        calls.append((board, cmd, param, motor, value))
        key = (board, motor, param)
        if cmd == 138:
            return None if mode == 'initial' else {'status': 100, 'value': 0}
        if cmd == 5:
            registers[key] = value
        if cmd == 4:
            registers[(board, motor, 1)] = value if mode == 'timeout_equal' else value + 6
            if mode == 'event':
                received.append(receive(driver, board, motor))
            return {'status': 100, 'value': 0}
        return {'status': 100, 'value': registers.get(key, 0)}

    driver._send_motor = wire
    adapter = Serial206ProductionPrimitiveAdapter.__new__(Serial206ProductionPrimitiveAdapter)
    adapter.tester = driver
    adapter.reference_store = MemoryReference()
    adapter.bind_y_provider(Serial206YProvider(driver, state_store=MemoryObservations(), generation_provider=lambda: 1))
    return adapter, driver, calls, received


def assert_x_completion(result, driver, received, *, target_observed):
    assert result['ok'] is True
    assert result['source_return_code'] == 0
    assert result['completion_class'] == ('event_128' if target_observed else 'oem_initial_latch')
    assert result['wait']['ok'] is True
    if target_observed:
        event = result['wait']['event']
        assert event['status'] == 128
        assert event['latch_disposition'] == 'consumed'
        assert event['event_sequence'] == received[0].receive_sequence
        assert event['receive_owner'] == received[0].receive_owner
        assert event['owner_generation'] == received[0].owner_generation
    else:
        assert result['wait']['event'] is None
        assert driver.collect_bus_events(duration_s=0) == []
    assert result['target_event_128_observed'] is target_observed
    assert result['controller_terminal_state_verified'] is target_observed
    assert result['physical_effect_verified'] is False
    assert not driver.motor_oem_wait_target_reached(5, 0, timeout_s=0)['ok']


@pytest.mark.parametrize('mode', ['initial', 'event'])
def test_public_x_absolute_keeps_source_success_distinct_from_received_proof(monkeypatch, mode):
    adapter, driver, calls, received = xy_adapter(monkeypatch, mode)
    result = adapter.x_move_absolute(position_steps=1100, wait_timeout_s=.002)
    assert result['move']['oem_wait_for_stop'] is True
    assert_x_completion(result, driver, received, target_observed=mode == 'event')
    assert adapter.reference_store.motions == [('x', 'absolute')]
    assert [(b, c) for b, c, *_ in calls if c in (138, 4)] == [(5, 138), (5, 4)]


@pytest.mark.parametrize('mode', ['initial', 'event'])
def test_x_issue_ticket_does_not_promote_initial_wait_to_terminal_proof(monkeypatch, mode):
    adapter, driver, calls, received = xy_adapter(monkeypatch, mode)
    ticket = adapter._x_issue_absolute(1100, source_mode='ClassControlInterface.moveX')
    assert ticket['ok'] is True
    assert ticket['move']['oem_wait_for_stop'] is True
    assert ticket['completion_class'] == ('event_128' if mode == 'event' else 'oem_initial_latch')
    assert ticket['controller_terminal_state_verified'] is (mode == 'event')
    assert ticket['physical_effect_verified'] is False
    assert not driver.motor_oem_wait_target_reached(5, 0, timeout_s=0)['ok']
    assert adapter.reference_store.motions == []


@pytest.mark.parametrize('mode', ['initial', 'event'])
def test_x_deferred_finalize_uses_actual_wait_class(monkeypatch, mode):
    adapter, driver, calls, received = xy_adapter(monkeypatch, mode)
    ticket = adapter.x_move_absolute(position_steps=1100, wait_for_stop=False)
    assert ticket['move']['oem_wait_for_stop'] is False
    assert ticket['completion_class'] is None
    result = adapter._x_finalize(ticket, timeout_s=.002, motion_kind='absolute')
    assert_x_completion(result, driver, received, target_observed=mode == 'event')
    assert adapter.reference_store.motions == [('x', 'absolute')]


def test_near_axis_xy_nested_x_preserves_initial_truth(monkeypatch):
    adapter, driver, calls, received = xy_adapter(monkeypatch)
    result = adapter.move_xy(1100, 1000, wait_timeout_s=.002)
    assert result['branch'] == 'near_axis_sequential'
    assert result['launch_order'] == ['x']
    assert result['ok'] is True
    nested = result['commands']['x']
    assert nested['ok'] is True
    assert nested['completion_class'] == 'oem_initial_latch'
    assert nested['target_event_128_observed'] is False
    assert nested['controller_terminal_state_verified'] is False
    assert nested['physical_effect_verified'] is False
    assert driver.collect_bus_events(duration_s=0) == []
    assert not driver.motor_oem_wait_target_reached(5, 0, timeout_s=0)['ok']
    assert result['physical_effect_verified'] is False
    assert adapter.reference_store.motions == [('x', 'absolute')]


@pytest.mark.parametrize('mode', ['initial', 'event', 'timeout_equal'])
def test_public_y_absolute_accepts_real_source_completion(monkeypatch, mode):
    adapter, driver, calls, received = xy_adapter(monkeypatch, mode)
    result = adapter.y_provider.move_absolute(1100, command_id='f05-y')
    primitive = result['result']
    assert primitive['ok'] is True
    assert primitive['oem_wait_for_stop'] is True
    assert result['ok'] is True
    expected_class = {'initial': 'oem_initial_latch', 'event': 'event_128',
                      'timeout_equal': 'oem_timeout_target_equal'}[mode]
    assert result['completion_class'] == primitive['completion_class'] == expected_class
    assert result['controller_completion_verified'] is (mode == 'event')
    assert result['physical_effect_verified'] is False
    assert result['discrepancy']['discrepancy_steps'] == (0 if mode == 'timeout_equal' else 6)
    assert result['position_after']['position'] == (1100 if mode == 'timeout_equal' else 1106)
    assert result['authority_observation']['ok'] is True
    assert adapter.y_provider.state_store.observations == [('y', {
        'requested_position_steps': 1100, 'observed_position_steps': result['position_after']['position'],
        'receipt_id': 'f05-y'})]
    wait = primitive['wait']
    assert wait['ok'] is (mode != 'timeout_equal')
    if mode == 'event':
        event = wait['event']
        assert event['latch_disposition'] == 'consumed'
        assert event['event_sequence'] == received[0].receive_sequence
        assert event['receive_owner'] == received[0].receive_owner
        assert event['owner_generation'] == received[0].owner_generation
    else:
        assert wait['event'] is None
        assert driver.collect_bus_events(duration_s=0) == []
    assert not driver.motor_oem_wait_target_reached(4, 0, timeout_s=0)['ok']
    assert [(b, c) for b, c, *_ in calls if c in (138, 4)] == [(4, 138), (4, 4)]


@pytest.mark.parametrize('target', [(1000, 1100), (1010, 1100), (1100, 1010)])
def test_near_axis_xy_accepts_bound_y_initial_source_call(monkeypatch, target):
    adapter, driver, calls, received = xy_adapter(monkeypatch)
    result = adapter.move_xy(*target, wait_timeout_s=.002)
    assert result['branch'] == 'near_axis_sequential'
    assert result['ok'] is True
    assert result['source_calls_completed'] is True
    assert result['failure'] is None
    assert result['physical_effect_verified'] is False
    axes = ['y'] if target[0] == 1000 else ['x', 'y']
    assert result['launch_order'] == axes
    for axis in axes:
        # The bounded nested JSON budget can omit the second command. Its
        # source success is covered by the real caller's all(commands) result;
        # Y-only above preserves the provider fields without that truncation.
        if axis == axes[0]:
            nested = result['commands'][axis]
            assert nested['ok'] is True
            assert nested['completion_class'] == 'oem_initial_latch'
            assert nested['physical_effect_verified'] is False
            if axis == 'x':
                assert nested['controller_terminal_state_verified'] is False
                assert nested['target_event_128_observed'] is False
            else:
                assert nested['controller_completion_verified'] is False
        assert not driver.motor_oem_wait_target_reached(5 if axis == 'x' else 4, 0, timeout_s=0)['ok']
    assert len(adapter.y_provider.state_store.observations) == 1
    assert adapter.y_provider.state_store.observations[0][1]['observed_position_steps'] == target[1] + 6
    assert driver.collect_bus_events(duration_s=0) == []
    assert [b for b, c, *_ in calls if c == 4] == ([4] if axes == ['y'] else [5, 4])


def test_y_internal_acceleration_overload_preserves_initial_success_and_restore(monkeypatch):
    adapter, driver, calls, received = xy_adapter(monkeypatch)
    result = adapter.y_provider.move_absolute_internal(
        'acceleration_overload', 1100, acceleration_override=700)
    assert result['ok'] is True
    assert result['completion_class'] == 'oem_initial_latch'
    assert result['controller_completion_verified'] is False
    assert result['acceleration_overload']['restoration_complete'] is True
    assert [(c, v) for b, c, p, m, v in calls if b == 4 and (c == 4 or c == 5 and p == 5)] == [
        (5, 700), (4, 1100), (5, 400)]
    assert not driver.motor_oem_wait_target_reached(4, 0, timeout_s=0)['ok']


@pytest.mark.parametrize('case', [
    'test_absolute_move_clamps_controller_target_and_waits_for_oem_completion_by_default',
    'test_absolute_move_preserves_timeout_equal_completion_and_terminal_discrepancy',
])
def test_existing_y_absolute_assertions_with_memory_store(monkeypatch, tmp_path, case):
    # Reuse the unchanged bounded provider regression assertions, but not its
    # unrelated SQLite authority verification (which times out in isolation).
    import test_serial206_y_provider as existing
    provider = existing.Serial206YProvider(
        existing.FakeYTester(), state_store=MemoryObservations(), generation_provider=lambda: 4)
    monkeypatch.setattr(existing, 'make_provider', lambda _path: provider)
    getattr(existing, case)(tmp_path)
