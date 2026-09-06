"""ClassMotor.cs:39 initial signal is source state, never a received frame."""
import pytest

from bioxp.novo_router import NovoRouter
from bioxp.usb_driver import novo_decode
from test_motor_receive_identity import setup_driver, receive, wait


def fresh_driver():
    return setup_driver(initial_signaled=True)


@pytest.mark.parametrize('ack', [None, {'status': 100}, {'status': 2}])
def test_initial_latch_null_query_preserves_nonnull_resets_and_wait_consumes_once(ack):
    driver = fresh_driver()
    driver._send_motor = lambda *args, **kwargs: ack
    driver.motor_query_motor_stop(5)
    result = wait(driver)
    assert result['ok'] is (ack is None)
    if ack is None:
        assert result['event'] is None
        assert result['completion_class'] == 'oem_initial_latch'
        assert result['events'] == []
        signal = result['reached']['5:0']
        assert signal['source'] == 'ClassMotor.initialState'
        assert not {'status', 'received_at', 'event_sequence', 'receive_owner', 'raw'} & signal.keys()
    assert not wait(driver)['ok']


def test_first_wait_without_query_consumes_initial_source_signal_only_once():
    driver = fresh_driver()
    assert wait(driver)['ok']
    assert not wait(driver)['ok']
    assert driver.collect_bus_events(duration_s=0) == []
    receive(driver)
    assert wait(driver)['event']['status'] == 128


def test_initial_waitall_is_atomic_and_sta_consumes_individually():
    driver = fresh_driver()
    driver._send_motor = lambda *args, **kwargs: {'status': 100}
    driver.motor_query_motor_stop(4)
    assert not driver.motor_oem_wait_targets_reached([(5, 0), (4, 0)], timeout_s=0)['ok']
    assert wait(driver)['ok']
    other = fresh_driver()
    other._send_motor = driver._send_motor
    other.motor_query_motor_stop(4)
    result = other.motor_wait_target_reached_many([(5, 0), (4, 0)], timeout_s=0, sta_sequential=True)
    assert not result['ok']
    assert result['per_axis']['x']['ok']
    assert not wait(other)['ok']


@pytest.mark.parametrize('targets', [((5, 0),), ((5, 0), (4, 0))])
@pytest.mark.parametrize('pre_window', [True, False])
def test_received_set_coalesces_with_untouched_initial_signal(targets, pre_window):
    driver = fresh_driver()
    window = driver.begin_bus_event_window()
    frames = [receive(driver, board, motor) for board, motor in targets]
    if pre_window:
        window = driver.begin_bus_event_window()
    if len(targets) == 1:
        result = driver.motor_oem_wait_target_reached(5, timeout_s=0, event_window=window)
        assert result['ok']
        assert (result['event'] is None) is pre_window
        assert result['completion_class'] == ('oem_initial_latch' if pre_window else 'event_128')
    else:
        result = driver.motor_oem_wait_targets_reached(targets, timeout_s=0, event_window=window)
        assert result['ok']
    for board, motor in targets:
        signal = result['reached'][f'{board}:{motor}']
        assert signal['latch_disposition'] == 'consumed'
        if pre_window:
            assert signal['source'] == 'ClassMotor.initialState'
            assert not {'status', 'received_at', 'event_sequence', 'receive_owner', 'owner_generation', 'raw'} & signal.keys()
        else:
            frame = next(frame for frame in frames if frame.data[0] == board)
            assert signal['event_sequence'] == frame.receive_sequence
            assert signal['receive_owner'] == frame.receive_owner
            assert signal['status'] == 128
        assert not driver.motor_oem_wait_target_reached(board, motor, timeout_s=0)['ok']
    assert len(result['events']) == (0 if pre_window else len(targets))
    diagnostics = driver.collect_bus_events(duration_s=0)
    assert len(diagnostics) == len(targets)
    assert all(row['latch_disposition'] == 'consumed' for row in diagnostics)


def test_reader_owner_replacement_does_not_reinitialize_constructed_motor_latches():
    driver = fresh_driver()
    assert wait(driver)['ok']
    driver.novo_router.shutdown()
    driver.novo_router = NovoRouter(ep_in=object(), ep_out=object(), decode=novo_decode)
    assert not wait(driver)['ok']
    # Another constructed motor retains its unconsumed initial state.
    assert wait(driver, 4, 1)['ok']
    assert not wait(driver, 4, 1)['ok']
