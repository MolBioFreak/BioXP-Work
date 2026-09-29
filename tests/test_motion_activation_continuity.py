"""Connected no-deactivation preparation: real driver/provider/source/SQLite."""
import pytest

from bioxp.motion_safety import Serial206MotionAuthority, prepare_motion_without_motion
from bioxp.oem_runtime_store import OEMRuntimeStore
from tests.test_wake_setup_debloat import rig, cycle


@pytest.fixture
def connected(rig, monkeypatch):
    driver, provider, frames, fault, root = rig
    driver._motor_last_tx_ts = {}
    wire = driver.send_tmcl
    def query(board, command, typ, motor, value, **kwargs):
        row = wire(board, command, typ, motor, value, **kwargs)
        if command == 15 and typ in (1, 3):
            row['value'] = 1
        return row
    monkeypatch.setattr(driver, 'send_tmcl', query)
    monkeypatch.setattr(driver, 'send_tmcl_retry', query)
    try:
        yield rig
    finally:
        # Actual cold activation starts the native services. Stop and join their
        # timers before monkeypatch restores transport; never hide thread errors.
        timers = []
        for owner in [driver, *fault.get('additional_drivers', [])]:
            for service in owner._oem_fan_services().values():
                service['running'] = False
                timer = service.get('timer')
                if timer is not None:
                    timer.cancel()
                    timers.append(timer)
        for timer in timers:
            timer.join()


def prepare(driver, provider):
    result = provider.prepare_global_motion_without_motion(
        driver, authority=Serial206MotionAuthority.from_active_snapshot())
    assert result['ok'] is True, result
    assert result['physical_motion'] is False
    assert result['homing_performed'] is False
    assert result['motor_torque_verified'] is False
    assert result['motor_output_state'] == 'unknown'
    return result


def assert_setup(frames):
    assert not any(row[1] in (1, 2, 3, 4, 30) for row in frames)
    assert not any(row[1] == 5 and row[2] in (0, 1, 7) for row in frames)
    assert not any(row[1] == 64 and row[4] == 0 for row in frames)
    assert [row for row in frames if row[0] == 4 and row[1] == 5 and row[3] == 1] == [
        (4, 5, 4, 1, 1791), (4, 5, 5, 1, 576),
        (4, 5, 6, 1, 31), (4, 5, 205, 1, 3)]
    assert (5, 14, 2, 0, 1) in frames  # retained global latch actuation


def test_cold_then_two_warm_preparations(connected):
    driver, provider, frames, fault, root = connected
    generation = None
    board_epoch = None
    for attempt in range(3):
        start = len(frames)
        result = prepare(driver, provider)
        issued = frames[start:]
        assert_setup(issued)
        assert [row for row in issued if row[1] == 64] == (
            [(board, 64, 0, 0, 1) for board in driver.BOARDS] if attempt == 0 else [])
        current_epoch = provider.state_store.board4_authority_projection()['board']
        if attempt:
            assert result['board_lifecycle_generation'] == generation
            assert current_epoch == board_epoch  # no invented warm callback
        generation = result['board_lifecycle_generation']
        board_epoch = current_epoch
        state = OEMRuntimeStore(root).read_oem_serial206_initialization_state()
        assert state['x_lifecycle']['state'] == 'prepared_unreferenced'
        assert state['z_lifecycle']['state'] == 'prepared_unreferenced'
        assert driver._oem_no_motion_profiles_ready.issuperset({'x','y','z','g','door'})
        assert not any(s['stage_id'] == 'deactivateBoard' for s in result['stage_ledger'])


def test_only_genuinely_inactive_board_is_activated(connected):
    driver, provider, frames, fault, root = connected
    fault['reject'] = (5, 64, 0, 0)
    first = provider.prepare_global_motion_without_motion(
        driver, authority=Serial206MotionAuthority.from_active_snapshot())
    assert first['ok'] is False
    assert driver._oem_board_state()[5] is False
    fault['reject'] = None
    start = len(frames)
    result = prepare(driver, provider)
    assert isinstance(result['board_lifecycle_generation'], int)
    assert [row for row in frames[start:] if row[1] == 64] == [(5, 64, 0, 0, 1)]
    assert_setup(frames[start:])


def test_chiller_status_two_remains_accepted(connected, monkeypatch):
    driver, provider, frames, fault, root = connected
    wire = driver.send_tmcl
    def reply(board, command, typ, motor, value, **kwargs):
        row = wire(board, command, typ, motor, value, **kwargs)
        if board == 7 and command == 64:
            row['status'] = 2
        return row
    monkeypatch.setattr(driver, 'send_tmcl', reply)
    monkeypatch.setattr(driver, 'send_tmcl_retry', reply)
    prepare(driver, provider)
    assert driver._oem_board_state()[7] is True


def test_missing_generation_on_initialized_boards_needs_no_cycle(connected):
    driver, provider, frames, fault, root = connected
    # Actual accepted activation callbacks establish provider/SQLite state, but
    # no complete-cycle token has ever existed.
    driver.oem_activate_uninitialized_boards()
    assert driver.oem_current_board_lifecycle_generation() is None
    start = len(frames)
    result = prepare(driver, provider)
    assert isinstance(result['board_lifecycle_generation'], int)
    assert not any(row[1] == 64 for row in frames[start:])
    assert_setup(frames[start:])


def test_transport_replacement_keeps_physical_registers(connected):
    driver, provider, frames, fault, root = connected
    first = prepare(driver, provider)
    # Real reset invoked by reconnect; fake controller registers persist.
    driver._reset_transport_recovery_state()
    assert driver.oem_current_board_lifecycle_generation() is None
    start = len(frames)
    result = prepare(driver, provider)
    assert result['board_lifecycle_generation'] > first['board_lifecycle_generation']
    assert_setup(frames[start:])
    assert [row for row in frames[start:] if row[1] == 64] == [
        (board, 64, 0, 0, 1) for board in driver.BOARDS]


def test_new_driver_and_provider_with_persistent_controller_registers(connected, monkeypatch):
    from bioxp.usb_driver import BioXpTester, novo_decode
    from bioxp.novo_router import NovoRouter
    from bioxp.oem_serial206_initialization import (
        Serial206OemInitializationProvider, Serial206ProductionPrimitiveAdapter)
    from bioxp.services.reference_service import ReferenceStateStore
    driver, provider, frames, fault, root = connected
    prepare(driver, provider)
    # Persist an arbitrary coordinate in the wire model, outside preparation.
    driver.send_tmcl(4, 5, 1, 1, 98765)
    fresh = BioXpTester.__new__(BioXpTester)
    fault['additional_drivers'] = [fresh]
    fresh.novo_router = NovoRouter(ep_in=object(), ep_out=object(), decode=novo_decode)
    fresh._motor_last_tx_ts = {}
    fresh._motor_noresp_streak = {}
    fresh._chiller_last_tx_ts = 0.0
    fresh._chiller_noresp_streak = 0
    monkeypatch.setattr(fresh, 'send_tmcl', driver.send_tmcl)
    monkeypatch.setattr(fresh, 'send_tmcl_retry', driver.send_tmcl_retry)
    refs = ReferenceStateStore(root / 'bioxp_runtime.db')
    adapter = Serial206ProductionPrimitiveAdapter(fresh, None,
        authority_provider=Serial206MotionAuthority.from_active_snapshot,
        generation_provider=lambda: 8, reference_store=refs)
    renewed = Serial206OemInitializationProvider(adapter,
        state_store=OEMRuntimeStore(root), reference_store=refs,
        generation_provider=lambda: 8)
    fresh._board_activation_observer = renewed.notify_board_activation
    assert fresh.oem_current_board_lifecycle_generation() is None
    assert not any(fresh._oem_board_state().values())
    start = len(frames)
    prepare(fresh, renewed)
    assert_setup(frames[start:])
    assert [row for row in frames[start:] if row[1] == 64] == [
        (board, 64, 0, 0, 1) for board in fresh.BOARDS]
    assert fresh.send_tmcl(4, 6, 1, 1, 0)['value'] == 98765


@pytest.mark.parametrize('failure', ['partial_ack', 'transport_exception', 'transport_drift', 'parameter_ack'])
def test_failure_does_not_publish_success(connected, failure):
    driver, provider, frames, fault, root = connected
    if failure == 'partial_ack':
        fault['reject'] = (5, 64, 0, 0)
    elif failure == 'parameter_ack':
        fault['reject'] = (4, 5, 6, 1)
    else:
        def fail(board, command, typ, motor, value):
            if command == 64 and board == 5:
                fault['on_frame'] = None
                if failure == 'transport_exception':
                    raise OSError('offline transport failed')
                driver._reset_transport_recovery_state()
        fault['on_frame'] = fail
    result = provider.prepare_global_motion_without_motion(
        driver, authority=Serial206MotionAuthority.from_active_snapshot())
    assert result['ok'] is False
    assert not any(row[1] == 64 and row[4] == 0 for row in frames)
    assert OEMRuntimeStore(root).read_oem_serial206_initialization_state()['x_lifecycle']['state'] != 'prepared_unreferenced'


@pytest.mark.parametrize('axis', ['x', 'z'])
def test_component_refresh_retains_lifecycle_and_other_profiles(connected, axis):
    driver, provider, frames, fault, root = connected
    first = prepare(driver, provider)
    start = len(frames)
    result = prepare_motion_without_motion(driver,
        authority=Serial206MotionAuthority.from_active_snapshot(),
        components=(axis,), reuse_current_board_lifecycle=True)
    assert result['ok'] is True, result
    assert result['board_lifecycle_generation'] == first['board_lifecycle_generation']
    assert not any(row[1] in (14, 64) for row in frames[start:])
    writes = [row for row in frames[start:] if row[1] == 5]
    assert all((row[0], row[3]) == ((5, 0) if axis == 'x' else (4, 1)) for row in writes)
    assert driver._oem_no_motion_profiles_ready.issuperset({'x','y','z','g','door'})


@pytest.mark.parametrize('fault_kind', ['door_reply', 'no24v'])
def test_existing_physical_prechecks_stop_before_activation(connected, monkeypatch, fault_kind):
    driver, provider, frames, fault, root = connected
    wire = driver.send_tmcl
    def bad_sensor(board, command, typ, motor, value, **kwargs):
        row = wire(board, command, typ, motor, value, **kwargs)
        if command == 15:
            if fault_kind == 'door_reply' and typ == 1:
                row['status'] = 1
            elif fault_kind == 'no24v' and typ == 0:
                row['value'] = 1
        return row
    monkeypatch.setattr(driver, 'send_tmcl', bad_sensor)
    monkeypatch.setattr(driver, 'send_tmcl_retry', bad_sensor)
    result = provider.prepare_global_motion_without_motion(
        driver, authority=Serial206MotionAuthority.from_active_snapshot())
    assert result['ok'] is False
    assert not any(row[1] in (5, 64) for row in frames)


def test_explicit_complete_cycle_contract_unchanged(connected):
    driver, provider, frames, fault, root = connected
    result = cycle(driver)
    assert result['source_order'] == ['cmd64=0', 'cmd64=1']
    assert [row for row in frames if row[1] == 64] == [
        (board, 64, 0, 0, value) for value in (0, 1) for board in driver.BOARDS]
    rejected = driver.oem_begin_board_lifecycle_generation(deactivation={}, activation={})
    assert rejected['ok'] is False
    assert rejected['failure'] == 'incomplete_oem_deactivate_activate_cycle'
