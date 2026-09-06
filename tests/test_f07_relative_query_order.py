"""Deck moveSteps query order: ClassDeckBoard.cs:229-284, CI:4165-4204.

queryActualPosition (ClassMotor.cs:565-591) updates cache only on status100,
returns cache on null, and returns 1 (without cache update) on non100.
Board timeout discards its query result and returns -1; CI ignores the board
return and makes a distinct query. Fake wire only; queries/waits run for real.
"""
import pytest

from test_motor_receive_identity import setup_driver, receive
from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot


def relative_driver(monkeypatch, mode, replies, *, late=False, drop=False):
    bind_serial206_oem_snapshot(monkeypatch)
    driver = setup_driver(initial_signaled=mode == 'initial')
    driver._oem_board_initialized = {5: True}
    driver._oem_position_cache = {(5, 0): 500}
    trace = []
    pending = iter(replies)
    power = [False]
    driver.oem_no24v_state = lambda: power[0]
    query = driver.motor_get_position
    wait = driver.motor_oem_wait_target_reached

    def observed_query(*args, **kwargs):
        trace.append(('query_enter', driver._oem_position_cache[(5, 0)]))
        result = query(*args, **kwargs)
        trace.append(('query_return', result['position'], result['position_source'],
                      driver._oem_position_cache[(5, 0)]))
        return result

    def observed_wait(*args, **kwargs):
        trace.append(('wait_enter',))
        result = wait(*args, **kwargs)
        trace.append(('wait_return', result['ok']))
        return result

    def wire(board, cmd, param, motor, value, **kwargs):
        trace.append(('wire', board, cmd, param, motor, value))
        assert (board, motor) == (5, 0)
        if cmd == 138:
            return None if mode == 'initial' else {'status': 100, 'value': 0}
        if cmd == 4:
            if mode == 'completion':
                receive(driver, 5, 0)
            return None  # Ignored leaf null must not gate the wait or queries.
        assert (cmd, param, value) == (6, 1, 0)
        if late:
            receive(driver, 5, 0)
        if drop:
            power[0] = True
        return next(pending)

    driver._send_motor = wire
    monkeypatch.setattr(driver, 'motor_get_position', observed_query)
    monkeypatch.setattr(driver, 'motor_oem_wait_target_reached', observed_wait)
    return driver, trace, wait


@pytest.mark.parametrize('mode', ['zero', 'timeout', 'completion', 'initial'])
@pytest.mark.parametrize('replies,values,sources,caches', [
    ([{'status': 100, 'value': 611}, {'status': 100, 'value': 722}],
     [611, 722], ['controller_reply', 'controller_reply'], [611, 722]),
    ([{'status': 100, 'value': 611}, None],
     [611, 611], ['controller_reply', 'oem_cached'], [611, 611]),
    ([None, {'status': 100, 'value': 722}],
     [500, 722], ['oem_cached', 'controller_reply'], [500, 722]),
    ([{'status': 2, 'value': 999}, None],
     [1, 500], ['oem_error_sentinel', 'oem_cached'], [500, 500]),
    ([None, {'status': 2, 'value': 999}],
     [500, 1], ['oem_cached', 'oem_error_sentinel'], [500, 500]),
])
def test_deck_public_relative_query_order_and_return_provenance(
        monkeypatch, mode, replies, values, sources, caches):
    driver, trace, wait = relative_driver(monkeypatch, mode, replies)
    result = driver.motor_x_move_relative_strict(0 if mode == 'zero' else 10, timeout_s=0)
    expected = []
    if mode != 'zero':
        expected = [('wire', 5, 138, 0, 0, 0), ('wire', 5, 4, 1, 0, 10),
                    ('wait_enter',), ('wait_return', mode != 'timeout')]
    for index in range(2):
        expected += [('query_enter', 500 if index == 0 else caches[0]),
                     ('wire', 5, 6, 1, 0, 0),
                     ('query_return', values[index], sources[index], caches[index])]
    assert trace == expected
    print({'mode': mode, 'trace': trace,
           'board_return': result['board_wrapper_return'],
           'public_return': result['public_wrapper_return'],
           'completion_class': result['completion_class']})
    assert result['board_wrapper_return'] == (-1 if mode == 'timeout' else values[0])
    assert result['public_wrapper_return'] == result['source_return_code'] == values[1]
    assert result['board_position']['position_source'] == sources[0]
    assert result['terminal_position']['position_source'] == sources[1]
    assert driver._oem_position_cache[(5, 0)] == caches[1]
    assert result['source_call_completed'] is True
    assert result['ok'] is True  # Integer return, not delivery/readback proof.
    assert result['controller_command_acknowledged'] is False
    assert result['completion_class'] == {
        'zero': 'oem_zero_step_noop', 'timeout': 'timeout',
        'completion': 'event_128', 'initial': 'oem_initial_latch',
    }[mode]
    assert result['controller_terminal_state_verified'] is (mode == 'completion')
    if mode == 'initial':
        assert result['wait']['event'] is None
    if mode == 'completion':
        assert result['wait']['event']['latch_disposition'] == 'consumed'
    assert not wait(5, 0, timeout_s=0)['ok']


def test_deck_board_null_has_no_query_or_move(monkeypatch):
    driver, trace, _ = relative_driver(monkeypatch, 'zero', [])
    driver._oem_board_present = lambda board: False
    result = driver.motor_x_move_relative_strict(10, timeout_s=0)
    assert trace == []
    assert result['public_wrapper_return'] == 0
    assert result['board_wrapper_return'] is None
    assert driver._oem_position_cache[(5, 0)] == 500


def test_timeout_queries_do_not_consume_later_completion(monkeypatch):
    driver, trace, wait = relative_driver(monkeypatch, 'timeout', [None, None], late=True)
    result = driver.motor_x_move_relative_strict(10, timeout_s=0)
    assert sum(row[0] == 'query_return' for row in trace) == 2
    assert result['board_wrapper_return'] == -1
    assert result['completion_class'] == 'timeout'
    assert result['controller_terminal_state_verified'] is False
    assert wait(5, 0, timeout_s=0)['ok']
    assert not wait(5, 0, timeout_s=0)['ok']


@pytest.mark.parametrize('mode,stage', [('zero', 4), ('timeout', 3), ('completion', 4)])
def test_deck_board_query_power_loss_prevents_public_query(monkeypatch, mode, stage):
    driver, trace, _ = relative_driver(
        monkeypatch, mode, [{'status': 100, 'value': 611}, None], drop=True)
    with pytest.raises(RuntimeError, match=f'Lost 24V power moveSteps{stage}'):
        driver.motor_x_move_relative_strict(0 if mode == 'zero' else 10, timeout_s=0)
    assert [row for row in trace if row[0] == 'query_return'] == [
        ('query_return', 611, 'controller_reply', 611)]
    assert driver._oem_position_cache[(5, 0)] == 611
