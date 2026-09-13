"""Native X/Y/Z parity with simulated wire replies; no hardware access."""
import struct
import pytest
from bioxp.serial206_y_provider import Serial206YProvider
from bioxp.usb_driver import novo_decode, OemMotionCompletionError
from tests.test_z_absolute_wire_contract import make_adapter
from tests.test_motor_receive_identity import receive


def native_axes(monkeypatch, *, event, discrepancy):
    adapter, _ = make_adapter(monkeypatch, 50000)
    driver = adapter.tester
    driver._oem_board_initialized = {4: True, 5: True}
    driver._oem_position_cache = {(5, 0): 50000, (4, 0): 50000, (4, 1): 50000}
    positions = dict(driver._oem_position_cache)
    writes = []
    def wire(board, cmd, param, motor, value, **kwargs):
        raw = bytes(driver._build_frame(board, cmd, param, motor, value))
        decoded = novo_decode(raw)
        assert decoded[3:8] == bytes([board, 7, cmd, param, motor])
        assert struct.unpack('>i', decoded[8:12])[0] == value
        channel = (board, motor)
        if cmd == 4:
            writes.append((board, cmd, param, motor, value))
            positions[channel] = (positions[channel] + value if param == 1 else value) + discrepancy
            if event:
                receive(driver, board=board, motor=motor)
        if cmd == 6 and param == 1:
            return {'status': 100, 'value': positions[channel]}
        return {'status': 100, 'value': 0}
    driver.send_tmcl = driver.send_tmcl_retry = wire
    adapter.reference_store = None
    adapter.y_provider = Serial206YProvider(driver, state_store=None, generation_provider=lambda: 7)
    return adapter, positions, writes


@pytest.mark.parametrize('axis,channel', [('x',(5,0)), ('y',(4,0)), ('z',(4,1))])
@pytest.mark.parametrize('event', [True, False])
@pytest.mark.parametrize('discrepancy', [-4,4,31])
def test_public_relative_call_preserves_oem_return_not_board_timeout(monkeypatch,axis,channel,event,discrepancy):
    adapter, positions, writes = native_axes(monkeypatch,event=event,discrepancy=discrepancy)
    if axis == 'x':
        result = adapter.x_move_steps(steps=1000, wait_timeout_s=.002)
    elif axis == 'z':
        result = adapter.z_move_steps(steps=1000, wait_timeout_s=.002)
    else:
        result = adapter.y_provider.move_steps(1000, wait_timeout_s=.002)
    assert result['ok'] is True, result
    native = result['result'] if axis == 'y' else result
    assert native['source_call_completed'] is True
    assert native['public_wrapper_return'] == positions[channel] == 51000 + discrepancy
    assert native['board_wrapper_return'] == (positions[channel] if event else -1)
    assert native['completion_class'] == ('event_128' if event else 'timeout')
    assert native['controller_terminal_state_verified'] is event
    assert writes == [(channel[0],4,1,channel[1],1000)]
    assert result['physical_effect_verified'] is False
    if axis == 'y':
        assert result['controller_completion_verified'] is event
        assert result['coordinate_truth_available'] is True


@pytest.mark.parametrize('axis', ['x','y','z'])
def test_no24v_still_throws_without_a_move(monkeypatch,axis):
    adapter, positions, writes = native_axes(monkeypatch,event=False,discrepancy=4)
    adapter.tester.oem_no24v_state = lambda: True
    call = {'x':lambda:adapter.x_move_steps(steps=1000),
            'y':lambda:adapter.y_provider.move_steps(1000),
            'z':lambda:adapter.z_move_steps(steps=1000)}[axis]
    # X/Z preflight may report refusal; Y reaches the same source exception.
    try:
        result = call()
    except RuntimeError as exc:
        assert '24V' in str(exc)
    else:
        assert result['ok'] is False
    assert writes == []


@pytest.mark.parametrize('axis', ['x','y','z'])
def test_absolute_event_accepts_discrepancy_without_reusing_relative_timeout_rule(monkeypatch,axis):
    adapter, positions, writes=native_axes(monkeypatch,event=True,discrepancy=4)
    if axis=='x':
        result=adapter.x_move_absolute(position_steps=51000)
    elif axis=='y':
        result=adapter.y_provider.move_absolute(51000)
    else:
        result=adapter.z_move_absolute(requested_position_steps=51000,pseudo_home_steps=500)
    assert result['ok'] is True
    assert len(writes)==1
    assert next(v for k,v in positions.items() if v!=50000)==51004


@pytest.mark.parametrize('axis', ['x','y','z'])
def test_absolute_missing_event_discrepancy_remains_source_exception(monkeypatch,axis):
    adapter, positions, writes=native_axes(monkeypatch,event=False,discrepancy=4)
    original=adapter.tester.motor_oem_wait_target_reached
    adapter.tester.motor_oem_wait_target_reached=lambda board,motor=0,**kw: original(board,motor,**dict(kw,timeout_s=.002))
    with pytest.raises(OemMotionCompletionError):
        if axis=='x': adapter.x_move_absolute(position_steps=51000)
        elif axis=='y': adapter.y_provider.move_absolute(51000)
        else: adapter.z_move_absolute(requested_position_steps=51000,pseudo_home_steps=500)
    assert len(writes)==1
