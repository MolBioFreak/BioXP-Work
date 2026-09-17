"""Recorder qualification, not successful connected-wake acceptance.

No setup/home/lifecycle methods are replaced. Connected acceptance remains the
unchanged test_protocol_v1_integrated_lifecycle deferred case.
"""
import pytest

from tests.protocol_v1_integration_fixture import NativePhysicalRecorder


@pytest.fixture
def recorder(monkeypatch):
    from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot
    bind_serial206_oem_snapshot(monkeypatch)
    native = NativePhysicalRecorder(monkeypatch)
    native.positions = {(4, 0): 0, (4, 1): 0, (4, 2): 0, (5, 0): 0, (6, 0): 10000}
    native.door_open_position = 10000
    yield native
    native.close()


def test_recorder_real_cycle_mints_generation_only_after_both_halves(recorder):
    driver = recorder.tester
    off = driver.deactivate_boards()
    assert not any(driver._oem_board_state().values())
    on = driver.activate_boards()
    result = driver.oem_begin_board_lifecycle_generation(deactivation=off, activation=on)
    assert result['ok'] is True
    assert driver.oem_current_board_lifecycle_generation() == result['board_lifecycle_generation']
    assert recorder.trace == [(b, 64, 0, 0, v) for v in (0, 1) for b in driver.BOARDS]
    assert getattr(driver, '_oem_no_motion_profiles_ready', set()) == set()


def test_recorder_rejected_board_reply_cannot_mint_generation(recorder):
    driver = recorder.tester
    off = driver.deactivate_boards()
    recorder.replies[4, 64, 0, 0] = {'status': 1, 'value': 1}
    on = driver.activate_boards()
    result = driver.oem_begin_board_lifecycle_generation(deactivation=off, activation=on)
    assert result['ok'] is False
    assert driver.oem_current_board_lifecycle_generation() is None


@pytest.mark.parametrize('axis,board,motor,speed', [
    ('z', 4, 1, 1791), ('g', 4, 2, 200), ('x', 5, 0, 250), ('y', 4, 0, 250)])
def test_recorder_executes_native_axis_search_and_home(recorder, axis, board, motor, speed):
    # Authentic preparation fixture's controller switch progression, not a
    # successful home result. Native queryHome/goHome/setHome decide outcome.
    switches = iter((0, 1))
    recorder.replies[board, 6, 9, motor] = lambda: {'status': 100, 'value': next(switches)}
    result = recorder.tester.motor_oem_axis_search_home(axis, speed=speed)
    assert result['ok'] is True
    assert result['go_home']['controller_home_proof_verified'] is True
    assert (board, 2, 0, motor, speed) in recorder.trace
    assert recorder.trace[-1] == (board, 5, 1, motor, 0)
    assert recorder.tester.motor_get_position(board, motor=motor)['position'] == 0


def test_recorder_native_relative_move_consumes_real_ingress(recorder):
    result = recorder.tester.motor_move_relative(4, 10000, motor=2)
    assert result['ok'] is True
    waited = recorder.tester.motor_oem_wait_target_reached(4, 2, timeout_s=.1)
    assert waited['ok'] is True
    assert waited['event']['status'] == 128
    assert recorder.positions[4, 2] == 10000
    # The native auto-reset signal cannot be consumed twice.
    assert recorder.tester.motor_oem_wait_target_reached(4, 2, timeout_s=.001)['ok'] is False


def test_recorder_preserves_native_no24v_before_home(recorder):
    recorder.tester.oem_latch_24v_dropped(reason='offline fault')
    with pytest.raises(RuntimeError, match='Lost 24V power axisSearchHome'):
        recorder.tester.motor_oem_axis_search_home('z', speed=1791)
    assert recorder.trace == []


@pytest.mark.parametrize('frame', [(4, 64, 0, 0, 2), (5, 15, 4, 0, 0),
                                   (4, 2, 0, 1, 500), (4, 4, 1, 2, 9999),
                                   (5, 2, 0, 0, 251), (4, 2, 0, 0, 251),
                                   (6, 2, 0, 0, 51), (5, 5, 4, 0, 1701),
                                   (7, 9, 0, 0, -24), (7, 9, 1, 1, -25)])
def test_recorder_unknown_commands_stay_closed(recorder, frame):
    with pytest.raises(AssertionError, match='unrecorded native physical command'):
        recorder.exchange(*frame)


def test_recorder_native_door_search_observes_shared_position(recorder):
    result = recorder.tester.motor_oem_door_search_home(startup=True)
    assert result['controller_home_proof_verified'] is True
    assert result['controller_terminal_state_verified'] is True
    assert (6, 2, 0, 0, 50) in recorder.trace
    assert recorder.tester.motor_get_position(6, motor=0)['position'] == 0


@pytest.mark.parametrize('parameter', [0, 1])
def test_recorder_native_chiller_offset_verified_readback(recorder, parameter):
    result = recorder.tester.chiller_gp_write(parameter, 0, -25, verify=True)
    assert result['verified'] is True
    assert result['readback']['value'] == -25
    assert recorder.trace == [(7, 9, parameter, 0, -25), (7, 10, parameter, 0, 0)]


def test_recorder_publication_reader_is_native_not_stale_observation(recorder):
    from types import SimpleNamespace
    from tests.protocol_v1_integration_fixture import PrimitiveComposition
    from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
    adapter = Serial206ProductionPrimitiveAdapter(recorder.tester, None,
        authority_provider=lambda: {}, generation_provider=lambda: 1)
    def stale(*args, **kwargs):
        raise AssertionError('stale observation reader used')
    composition = PrimitiveComposition(SimpleNamespace(motor_get_position=stale), adapter)
    recorder.positions[5, 0] = 10000
    assert composition.motor_get_position(5, motor=0)['position'] == 10000
    recorder.exchange(5, 5, 1, 0, 0)
    assert composition.motor_get_position(5, motor=0)['position'] == 0
