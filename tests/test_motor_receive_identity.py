"""Offline producer -> retained router queue -> real motor waiter regressions."""
import pytest

from bioxp.novo_router import NovoRouter
from bioxp.usb_driver import BioXpTester, novo_encode, novo_decode


def setup_driver(*, initial_signaled=False):
    driver = object.__new__(BioXpTester)
    driver.novo_router = NovoRouter(ep_in=object(), ep_out=object(), decode=novo_decode)
    driver.oem_no24v_state = lambda: False
    if not initial_signaled:
        # These receive-identity tests begin after first-use source signals
        # have been consumed, not at initially-signaled ClassMotor construction.
        for board, motor in ((4, 0), (4, 1), (4, 2), (5, 0), (6, 0)):
            assert driver.motor_oem_wait_target_reached(board, motor, timeout_s=0)['ok']
    return driver


def receive(driver, board=5, motor=0):
    router = driver.novo_router
    body = bytes([0, 0, 0, 0, 8, board, 128, 138, 0, 0, 0, motor, 0])
    raw = bytes(novo_encode(body))
    frame = router._decode_record(novo_decode(raw), raw, router._clock())
    router._dispatch(frame)
    return router.queue_snapshot('valid_async')[-1]


def test_rereading_ingress_frame_cannot_satisfy_new_wait():
    driver = setup_driver()
    receive(driver)
    first = driver.collect_bus_events(duration_s=0)[0]
    window = driver.begin_bus_event_window()
    second = driver.collect_bus_events(duration_s=0)[0]
    result = driver.motor_oem_wait_target_reached(5, timeout_s=.002, event_window=window)
    assert not result['ok'], 'retained pre-reset frame was minted as a new completion'
    assert first['event_sequence'] == second['event_sequence']


def test_consumed_signal_remains_in_wait_receipt():
    driver = setup_driver()
    receive(driver)
    result = wait(driver)
    assert result['event'] in result['events']


def test_empty_waitall_is_already_satisfied():
    driver = setup_driver()
    assert driver.motor_oem_wait_targets_reached([], timeout_s=.002)['ok']


def wait(driver, board=5, motor=0):
    return driver.motor_oem_wait_target_reached(board, motor, timeout_s=.002)


def test_consumption_is_axis_local_and_coalesces_duplicate_sets():
    driver = setup_driver()
    receive(driver)
    receive(driver)
    receive(driver, board=4, motor=1)
    assert wait(driver)['ok']
    assert not wait(driver)['ok'], 'AutoResetEvent must consume, not count duplicate Set calls'
    assert wait(driver, 4, 1)['ok']
    assert not wait(driver, 4, 0)['ok']


@pytest.mark.parametrize('ack', [None, {'status': 100}, {'status': 2}])
def test_query_stop_resets_only_addressed_axis_after_nonnull_return(ack):
    driver = setup_driver()
    receive(driver)
    receive(driver, board=4, motor=0)
    driver._send_motor = lambda *args, **kwargs: ack
    result = driver.motor_query_motor_stop(5, 0)
    assert result['source_return_code'] == (1 if ack == {'status': 2} else 0)
    assert wait(driver)['ok'] is (ack is None)
    assert wait(driver, 4)['ok']


def test_previous_transport_generation_is_not_a_current_completion():
    driver = setup_driver()
    old = receive(driver)
    driver.novo_router.shutdown()
    assert not wait(driver)['ok']
    driver.novo_router._dispatch(old)
    assert not wait(driver)['ok']
    receive(driver)
    result = wait(driver)
    assert result['ok']
    assert result['event']['owner_generation'] != old.owner_generation


def test_waitall_timeout_does_not_consume_partial_signal():
    driver = setup_driver()
    receive(driver)
    result = driver.motor_oem_wait_targets_reached([(5, 0), (4, 0)], timeout_s=.002)
    assert not result['ok']
    assert wait(driver)['ok']
    assert not wait(driver)['ok']


def test_waitall_consumes_both_and_per_axis_reset_preserves_first_axis():
    driver = setup_driver()
    driver._send_motor = lambda *args, **kwargs: {'status': 100}
    driver.motor_query_motor_stop(5)
    receive(driver)
    driver.motor_query_motor_stop(4)
    receive(driver, board=4)
    result = driver.motor_oem_wait_targets_reached([(5, 0), (4, 0)], timeout_s=.002)
    assert result['ok']
    assert not wait(driver)['ok']
    assert not wait(driver, 4)['ok']


def test_genuinely_late_wire_event_remains_unattributed_but_sets_oem_latch():
    driver = setup_driver()
    driver._send_motor = lambda *args, **kwargs: {'status': 100}
    driver.motor_query_motor_stop(5)
    assert not wait(driver)['ok']
    # Arrival after timeout: no command identifier exists in these wire bytes.
    receive(driver)
    result = wait(driver)
    assert result['ok']
    assert result['event'].get('command_correlation') == 'unavailable_on_wire'
    assert 'transaction_id' not in result['event']


def test_null_query_preserves_pre_window_signal_in_xy_wait():
    driver = setup_driver()
    receive(driver)
    window = driver.begin_bus_event_window()
    driver._send_motor = lambda *args, **kwargs: None
    driver.motor_query_motor_stop(5)
    driver.motor_query_motor_stop(4)
    receive(driver, board=4)
    assert driver.motor_oem_wait_targets_reached([(5, 0), (4, 0)], timeout_s=.002, event_window=window)['ok']


def test_sta_wait_consumes_first_axis_even_when_second_times_out():
    driver = setup_driver()
    receive(driver)
    result = driver.motor_wait_target_reached_many([(5, 0), (4, 0)], timeout_s=.002, sta_sequential=True)
    assert not result['ok']
    assert result['per_axis']['x']['ok']
    assert not wait(driver)['ok']


def test_real_receive_loop_assigns_identity_before_any_inspection():
    driver = setup_driver()
    router = driver.novo_router
    raw = bytes(novo_encode(bytes([0, 0, 0, 0, 8, 5, 128, 138, 0, 0, 0, 0, 0])))
    class Endpoint:
        def read(self, size, timeout):
            router._stop.set()
            return raw
    router.ep_in = Endpoint()
    router._receive_loop()
    frame = router.queue_snapshot('valid_async')[0]
    assert frame.receive_sequence == 1
    assert frame.owner_generation == router.reader_generation
    window = driver.begin_bus_event_window()
    assert not driver.motor_oem_wait_target_reached(5, timeout_s=.002, event_window=window)['ok']
    assert router.queue_snapshot('valid_async')[0] is frame


def test_owner_replacement_cannot_reuse_window_or_record():
    driver = setup_driver()
    old = receive(driver)
    window = driver.begin_bus_event_window(reset_wait_latch=False)
    driver.novo_router = NovoRouter(ep_in=object(), ep_out=object(), decode=novo_decode)
    driver.novo_router._dispatch(old)
    assert not wait(driver)['ok']
    receive(driver)
    assert not driver.motor_oem_wait_target_reached(5, timeout_s=.002, event_window=window)['ok']
    assert wait(driver)['ok']
