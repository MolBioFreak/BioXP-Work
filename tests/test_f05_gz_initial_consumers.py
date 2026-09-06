"""G/Z source consumption is not received-event proof (real primitive/router)."""
import pytest

from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot
from test_motor_receive_identity import setup_driver, receive
from test_f05_xy_initial_consumers import MemoryReference


def gz_adapter(monkeypatch, modes):
    bind_serial206_oem_snapshot(monkeypatch)
    driver = setup_driver(initial_signaled=True)
    driver._oem_board_initialized = {4: True}
    # Fixture-only prepared lifecycle; preserve untouched construction handles.
    driver._oem_active_board_lifecycle_generation = 1
    driver._oem_no_motion_profiles_ready = {'g', 'z'}
    driver._oem_no_motion_profile_generations = {'g': 1, 'z': 1}
    registers = {(4, motor, 1): 1000 for motor in (1, 2)}
    for axis in ('g', 'z'):
        profile = driver._motion_oem_axis_profile(axis)
        for param, field in ((4, 'speed'), (5, 'acc'), (6, 'run_current'), (205, 'stall_guard')):
            registers[(profile['board'], profile['motor'], param)] = int(profile[field])
    calls, received = [], {}

    def wire(board, cmd, param, motor, value, **kwargs):
        calls.append((board, cmd, param, motor, value))
        key = (board, motor, param)
        if cmd == 138:
            return None if modes[motor] == 'initial' else {'status': 100, 'value': 0}
        if cmd == 5:
            registers[key] = value
        if cmd == 4:
            registers[(board, motor, 1)] = value
            if modes[motor] == 'event':
                received[motor] = receive(driver, board, motor)
        return {'status': 100, 'value': registers.get(key, 0)}

    driver._send_motor = wire
    adapter = Serial206ProductionPrimitiveAdapter.__new__(Serial206ProductionPrimitiveAdapter)
    adapter.tester = driver
    adapter._z_profile_overrides = {}
    adapter.reference_store = MemoryReference()
    return adapter, driver, calls, received


@pytest.mark.parametrize('g_mode,z_mode', [
    ('initial', 'initial'), ('initial', 'event'),
    ('event', 'initial'), ('event', 'event'), ('timeout', 'timeout'),
])
def test_gz_pair_source_success_and_owned_receive_proof(monkeypatch, g_mode, z_mode):
    modes = {2: g_mode, 1: z_mode}
    adapter, driver, calls, received = gz_adapter(monkeypatch, modes)
    # Observe, never replace, the real consuming wait. The public receipt's
    # bounded JSON projection may omit deeply nested per-channel provenance.
    waits = []
    real_wait = driver.motor_oem_wait_targets_reached

    def observe_wait(*args, **kwargs):
        receipt = real_wait(*args, **kwargs)
        waits.append(receipt)
        return receipt

    monkeypatch.setattr(driver, 'motor_oem_wait_targets_reached', observe_wait)
    result = adapter.z_move_gz(gripper_position_steps=1100, z_position_steps=1200,
                               wait_timeout_s=.002)
    assert len(waits) == 1
    consumed_wait = waits[0]
    success = g_mode != 'timeout'
    assert result['ok'] is success
    assert result['controller_command_acknowledged'] is True
    assert result['failure'] == (None if success else 'move_gz_controller_evidence_unverified')
    assert result['targets'] == {'g': 1100, 'z': 1200}
    assert result['physical_effect_verified'] is False
    assert [(b, c, m) for b, c, p, m, v in calls if c in (138, 4)] == [
        (4, 138, 2), (4, 4, 2), (4, 138, 1), (4, 4, 1)]
    assert result['wait']['ok'] is success
    if success:
        for motor, mode in modes.items():
            event = consumed_wait['reached'][f'4:{motor}']
            assert event['board'] == 4 and event['motor'] == motor
            assert event['latch_disposition'] == 'consumed'
            if mode == 'event':
                assert event['source'] == 'novo_router_async'
                assert event['status'] == 128
                assert event['event_sequence'] == received[motor].receive_sequence
                assert event['receive_owner'] == received[motor].receive_owner
                assert event['owner_generation'] == received[motor].owner_generation
            else:
                assert event['source'] == 'ClassMotor.initialState'
                assert all(k not in event for k in (
                    'status', 'event_sequence', 'receive_owner', 'owner_generation'))
    else:
        assert result['wait']['reached'] == {}
        assert result['wait']['failure'] == 'oem_moveXY_target_event_timeout'
    assert not driver.motor_oem_wait_targets_reached(((4, 2), (4, 1)), timeout_s=0)['ok']
    for motor in modes:
        assert not driver.motor_oem_wait_target_reached(4, motor, timeout_s=0)['ok']
    assert result['controller_terminal_state_verified'] is (g_mode == z_mode == 'event')
