"""F05: public provider -> real primitive/router; only wire hardware is fake."""
import pytest

from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from test_motor_receive_identity import setup_driver, receive


def adapter_for(mode):
    driver = setup_driver(initial_signaled=mode == 'initial')
    driver._oem_board_initialized = {4: True}
    registers = {(4, 1, 1): 0, (4, 1, 3): 0}

    def wire(board, cmd, param, motor, value, **kwargs):
        key = (board, motor, param)
        if cmd == 5:
            registers[key] = value
        if cmd == 138 and mode == 'initial':
            return None
        if cmd == 4:
            registers[(board, motor, 1)] = value + 6
            if mode == 'initial':
                return {'status': 100, 'value': 0}
            receive(driver, board=4, motor=1 if mode != 'wrong_axis' else 0)
            if mode == 'consumed':
                assert driver.motor_oem_wait_target_reached(4, 1, timeout_s=0)['ok']
            elif mode == 'previous_generation':
                driver.novo_router.shutdown()
        return {'status': 100, 'value': registers.get(key, 0)}

    driver._send_motor = wire
    adapter = Serial206ProductionPrimitiveAdapter.__new__(Serial206ProductionPrimitiveAdapter)
    adapter.tester = driver
    return adapter, driver


@pytest.mark.parametrize('mode', ['valid', 'consumed', 'previous_generation', 'wrong_axis'])
def test_public_z_absolute_uses_owned_consumption_not_retained_diagnostics(mode, monkeypatch):
    from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot
    bind_serial206_oem_snapshot(monkeypatch)
    adapter, driver = adapter_for(mode)
    result = adapter.z_move_absolute(requested_position_steps=10000, pseudo_home_steps=0, wait_timeout_s=.002)
    assert result['ok'] is (mode == 'valid')
    assert result['physical_effect_verified'] is False
    if mode == 'valid':
        assert result['target_events'][0]['latch_disposition'] == 'consumed'
        assert not driver.motor_oem_wait_target_reached(4, 1, timeout_s=0)['ok']
    assert driver.collect_bus_events(duration_s=0), 'diagnostic receive records must remain available'


@pytest.mark.parametrize('caller', ['provider', 'absolute_primitive', 'relative_strict'])
def test_initial_source_latch_is_not_reported_as_received_completion(caller, monkeypatch):
    from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot
    bind_serial206_oem_snapshot(monkeypatch)
    adapter, driver = adapter_for('initial')
    if caller == 'provider':
        result = adapter.z_move_absolute(requested_position_steps=10000, pseudo_home_steps=0, wait_timeout_s=.002)
        assert result['target_events'] == []
    elif caller == 'absolute_primitive':
        result = driver.motor_oem_move_absolute(4, 10000, motor=1, wait_for_stop=True)
    else:
        result = driver.motor_z_move_relative_strict(10000, timeout_s=.002)
        assert result['controller_terminal_state_verified'] is False
    assert result['ok'] is True
    assert result['completion_class'] == 'oem_initial_latch'
    assert driver.collect_bus_events(duration_s=0) == []
    assert not driver.motor_oem_wait_target_reached(4, 1, timeout_s=0)['ok']


@pytest.mark.parametrize('axes', [('x',), ('x', 'y')])
@pytest.mark.parametrize('initial', [True, False])
@pytest.mark.parametrize('generation_change', [False, True])
def test_receipt_binds_consumed_wait_not_later_collector_target(axes, initial, generation_change):
    adapter, driver = adapter_for('initial' if initial else 'valid')
    adapter.reference_store = None
    addresses = {'x': (5, 0), 'y': (4, 0)}
    targets = [addresses[axis] for axis in axes]
    window = driver.begin_bus_event_window()
    if not initial:
        for board, motor in targets:
            receive(driver, board, motor)
    # Both producer shapes occur in reachable completion paths.
    if len(axes) == 1:
        waits = {'x': driver.motor_oem_wait_target_reached(5, timeout_s=0, event_window=window)}
    else:
        waits = driver.motor_wait_target_reached_many(targets, timeout_s=0, event_window=window)['per_axis']
    assert all(wait['ok'] for wait in waits.values())
    consumed = {axis: wait['event'] for axis, wait in waits.items()}
    driver.novo_router.queue_clear('valid_async')
    collect = driver.collect_bus_events
    late = []

    def collector(**kwargs):
        if generation_change:
            driver.novo_router.shutdown()
        for board, motor in targets:
            late.append(receive(driver, board, motor))
        return collect(**kwargs)

    driver.collect_bus_events = collector
    commands = {axis: {'ok': True, 'command_issued': True, 'controller_command_acknowledged': True,
                       'event_window': window} for axis in axes}
    receipt = adapter._finalize_move_xy_receipt(
        {'requested': {axis: 0 for axis in axes}}, commands=commands, waits=waits,
        after={axis: 0 for axis in axes}, restore={}, required_axes=axes,
    )
    driver.collect_bus_events = collect
    assert receipt['ok'] is True
    assert receipt['controller_terminal_state_verified'] is (not initial)
    assert all(row['position_verified'] and row['terminal_speed_verified']
               for row in receipt['axis_evidence'].values())
    assert all(row['target_event_128_observed'] is (not initial)
               for row in receipt['axis_evidence'].values())
    assert all(row['latch_disposition'] == 'set' for row in receipt['events'])
    for axis, frame in zip(axes, late):
        # Exact received identity of the historical consumed event stays in
        # the receipt even when diagnostics contain only a different Set.
        reported = receipt['waits'][axis]['event']
        if consumed[axis] is None:
            assert reported is None
        else:
            for key in ('source', 'board', 'motor', 'latch_disposition',
                        'event_sequence', 'receive_owner', 'owner_generation', 'received_at'):
                assert reported.get(key) == consumed[axis].get(key)
        if not initial:
            assert consumed[axis]['event_sequence'] != frame.receive_sequence
            assert consumed[axis]['owner_generation'] == window['owner_generation']
        next_wait = driver.motor_oem_wait_target_reached(*addresses[axis], timeout_s=0)
        assert next_wait['event']['event_sequence'] == frame.receive_sequence
        assert not driver.motor_oem_wait_target_reached(*addresses[axis], timeout_s=0)['ok']


@pytest.mark.parametrize('axes', [('x',), ('x', 'y')])
def test_receipt_keeps_owned_consumed_proof_without_retained_target_copy(axes):
    adapter, driver = adapter_for('valid')
    adapter.reference_store = None
    addresses = {'x': (5, 0), 'y': (4, 0)}
    window = driver.begin_bus_event_window()
    for axis in axes:
        receive(driver, *addresses[axis])
    waits = driver.motor_wait_target_reached_many(
        [addresses[axis] for axis in axes], timeout_s=0, event_window=window)['per_axis']
    driver.novo_router.queue_clear('valid_async')
    receipt = adapter._finalize_move_xy_receipt(
        {'requested': {axis: 0 for axis in axes}},
        commands={axis: {'ok': True, 'command_issued': True, 'event_window': window} for axis in axes},
        waits=waits, after={axis: 0 for axis in axes}, restore={}, required_axes=axes,
    )
    assert receipt['events'] == []
    assert receipt['controller_terminal_state_verified'] is True
    for axis in axes:
        assert not driver.motor_oem_wait_target_reached(*addresses[axis], timeout_s=0)['ok']


@pytest.mark.parametrize('mismatch', ['axis', 'receive_owner', 'generation'])
def test_receipt_rejects_consumed_wait_from_other_axis_or_window(mismatch):
    adapter, driver = adapter_for('valid')
    adapter.reference_store = None
    window = driver.begin_bus_event_window()
    other = setup_driver() if mismatch == 'receive_owner' else driver
    if mismatch == 'generation':
        other.novo_router.shutdown()
    board = 4 if mismatch == 'axis' else 5
    receive(other, board, 0)
    foreign_wait = other.motor_oem_wait_target_reached(board, timeout_s=0)
    assert foreign_wait['ok']
    receive(driver, 5, 0)  # A separate eligible retained target cannot repair association.
    receipt = adapter._finalize_move_xy_receipt(
        {'requested': {'x': 0}},
        commands={'x': {'ok': True, 'command_issued': True, 'event_window': window}},
        waits={'x': foreign_wait}, after={'x': 0}, restore={}, required_axes=('x',),
    )
    assert receipt['ok'] is True
    assert receipt['controller_terminal_state_verified'] is False
    assert receipt['axis_evidence']['x']['target_event_128_observed'] is False


@pytest.mark.parametrize('has_signal', [False, True])
def test_xy_receipt_is_diagnostic_and_does_not_wait_or_consume_again(has_signal):
    adapter, driver = adapter_for('valid')
    adapter.reference_store = None
    window = driver.begin_bus_event_window()
    if has_signal:
        receive(driver, board=5, motor=0)
        receive(driver, board=4, motor=0)
    waits = driver.motor_wait_target_reached_many([(5, 0), (4, 0)], timeout_s=0, event_window=window)
    commands = {axis: {'ok': True, 'command_issued': True, 'controller_command_acknowledged': True,
                       'event_window': window} for axis in ('x', 'y')}
    receipt = adapter._finalize_move_xy_receipt(
        {'requested': {'x': 0, 'y': 0}}, commands=commands, waits=waits['per_axis'],
        after={'x': 0, 'y': 0}, restore={}, required_axes=('x', 'y'),
    )
    assert receipt['ok'] is True, 'OEM moveXY ignores numeric wait timeout'
    assert receipt['controller_terminal_state_verified'] is has_signal
    assert all(row['wait_accepted'] is has_signal for row in receipt['axis_evidence'].values())
    assert not driver.motor_oem_wait_target_reached(5, 0, timeout_s=0)['ok']
    if has_signal:
        assert all(row['latch_disposition'] == 'consumed' for row in receipt['events'])
