"""Offline vectors derived from SSD ClassCanLib IL and NovoCANUSB projection.

ClassCanLib IL: 10241-10288 (Stop), 10578-10620 (checkMotorStopped),
14332-14355 (cached speed), 16245-16269 (0/99 DLC8).
ClassNovoCANUSB.cs: 787-839, 979-989, 1121-1132 (match then forward).
No tester construction, transport reader, hardware or live runtime calls.
"""
import threading
from types import SimpleNamespace

import pytest
from bioxp.novo_router import NovoRouter
from bioxp.novo_usb_can import novo_decode
from bioxp.usb_driver import BioXpTester


def oem_record(body):
    # NovoEncoding.cs additive checksum and7D/7E XOR20; independent wire oracle.
    payload = [*body, sum(body) & 255]
    wire = [126]
    for byte in payload:
        wire.extend([125, byte ^ 32] if byte in (125, 126) else [byte])
    return bytes([*wire, 126])


def frame(module=99, board=4, status=128, command=138, axis=1, dlc=8):
    # Target axis is payload[6], NOT the command/axis field used by stall.
    data = bytes([board, status, command, 0, 0, 0, axis, 0])[:dlc]
    body = module.to_bytes(4, 'big') + bytes([dlc]) + data
    raw = oem_record(body)
    return NovoRouter._decode_record(novo_decode(raw), raw, 1.0)


@pytest.mark.parametrize('module', [0, 99])
@pytest.mark.parametrize('status', [100, 128, 130, 2])
def test_oem_pending_match_has_no_status_exclusion(module, status):
    f = frame(module=module, status=status)
    assert NovoRouter.tmcl_matcher(board_id=4, command=138)(f).matched


@pytest.mark.parametrize('module,board,command,dlc', [(98,4,138,8),(99,5,138,8),(99,4,3,8),(99,4,138,7)])
def test_unaccepted_envelope_or_identity_does_not_match(module, board, command, dlc):
    assert not NovoRouter.tmcl_matcher(board_id=4, command=138)(frame(module,board,command=command,dlc=dlc)).matched


def test_command64_matches_board_only_but_not_other_board():
    match = NovoRouter.tmcl_matcher(board_id=4, command=64)
    assert match(frame(command=3)).matched
    assert not match(frame(board=5, command=64)).matched


@pytest.mark.parametrize('module', [0,99])
def test_pending_query138_consumes_target_without_async_set(module):
    router = NovoRouter(ep_in=object(), ep_out=object(), decode=novo_decode)
    pending = SimpleNamespace(matcher=router.tmcl_matcher(board_id=4, command=138), frames=[], skipped=[], skipped_total=0, event=threading.Event())
    router._pending = pending
    router._dispatch(frame(module=module))
    assert pending.event.is_set()
    assert len(pending.frames) == 1
    assert (4,1) not in router._motor_signals
    assert router.queue_snapshot('valid_async') == []


@pytest.mark.parametrize('module,board,status,axis,expected', [(0,4,128,1,True),(99,4,128,1,True),(99,5,128,0,True),(99,3,128,1,False),(99,11,128,1,False),(98,4,128,1,False),(99,4,130,1,False),(99,4,128,3,False)])
def test_unmatched_oem_forward_filter_and_target_axis(module, board, status, axis, expected):
    router = NovoRouter(ep_in=object(), ep_out=object(), decode=novo_decode)
    router._dispatch(frame(module,board,status,axis=axis))
    assert ((board,axis) in router._motor_signals) is expected


@pytest.mark.parametrize('speed_ack,switch_ack,set_ack,verified', [(None,None,None,False),({'status':100},None,{'status':100},False),({'status':100},{'status':100},None,False),({'status':100},{'status':100},{'status':100},True)])
def test_home_source_return_does_not_promote_null_fallbacks(speed_ack, switch_ack, set_ack, verified):
    driver = object.__new__(BioXpTester)
    driver.oem_no24v_state = lambda: False
    driver._oem_board_state = lambda: {4:True}
    driver._motion_oem_axis_profile = lambda axis: {'board':4,'motor':1}
    # ClassMotor.queryLeftSwitchStatus: reply[6] == 1 -> scalar0 -> queryHome true.
    driver.motor_get_axis_param = lambda board, param, **k: {'ack': speed_ack if param == 3 else switch_ack, 'value':0 if param == 3 else 1}
    driver.motor_get_position = lambda *a, **k: {'position':0,'ack':{'status':100}}
    driver.begin_bus_event_window = lambda: {}
    driver.motor_move_left = lambda *a, **k: {'ack':{'status':100}}
    driver._send_motor = lambda *a, **k: set_ack
    driver.motor_oem_stop_exact = lambda *a, **k: {'ok':True,'first_delivery':{'status':100},'second_delivery':{'status':100}}
    result = driver.motor_oem_go_home('z',speed=1791,rehome=False)
    assert result['ok'] is True and result['source_return_code'] == 0
    assert result['controller_home_proof_verified'] is verified
    assert result['controller_terminal_state_verified'] is (speed_ack is not None)


def test_home_abort_follows_board_stop_no24v_guard_without_motor_delivery():
    driver = object.__new__(BioXpTester)
    latched = [False]
    delivered = []
    driver.oem_no24v_state = lambda: latched[0]
    driver._oem_board_state = lambda: {4:True}
    driver._motion_oem_axis_profile = lambda axis: {'board':4,'motor':1}
    driver.motor_get_axis_param = lambda board, param, **k: {'ack':{'status':100},'value':0 if param == 3 else 1}
    driver.motor_get_position = lambda *a, **k: {'position':0,'ack':{'status':100}}
    driver.begin_bus_event_window = lambda: {}
    def move_left(*args, **kwargs):
        latched[0] = True
        return {'ack':{'status':100}}
    driver.motor_move_left = move_left
    driver.motor_oem_stop_exact = lambda *a, **k: delivered.append('StopMotor') or {}
    driver._send_motor = lambda *a, **k: delivered.append('setHome') or {'status':100}
    # Head.goHome lines108-114 calls board stopMotor, not its unguarded leaf.
    with pytest.raises(RuntimeError, match='Lost 24V power stopMotor 1'):
        driver.motor_oem_go_home('z',speed=1791,rehome=False)
    assert delivered == []


@pytest.mark.parametrize('valid', [False,True])
def test_aggregate_zero_proof_requires_fresh_reply_not_cached_scalar(valid):
    from bioxp.motion_safety import physical_aggregate_stop
    authority = SimpleNamespace(components=[('z',4,1,True)], machine_serial=206)
    driver = object.__new__(BioXpTester)
    driver.motor_get_axis_param = lambda *a, **k: {'ack':{'status':100} if valid else None,'value':0}
    driver.motor_oem_board_stop = lambda *a, **k: {'source_call_completed':True,'source_return_code':0,'double_stop_acknowledged':True,'controller_command_acknowledged':True}
    driver.motor_oem_force_abort_motion = lambda **k: {'ok':True,'latched':True}
    authority.controller_evidence = lambda: {}
    result = physical_aggregate_stop(driver,authority)
    assert result['controller_terminal_state_verified'] is valid
    assert result['components'][0]['zero_speed_verified'] is valid


def test_escaped_diagnostic_preserves_raw_and_decodes_normalized_fields():
    driver = object.__new__(BioXpTester)
    # Escaped payload[3] shifts all raw offsets after it. Stall axis is data[2].
    body = bytes([0,0,0,99,8,4,130,1,126,125,0,2,0])
    raw = oem_record(body)
    row = driver._decode_bus_event_frame(raw)
    assert row['board'] == 4 and row['status'] == 130 and row['motor'] == 1
    assert row['value'] == int.from_bytes(bytes([126,125,0,2]), 'big', signed=True)
    assert row['raw'] == list(raw)


@pytest.mark.parametrize('ack,value,verified', [(None,None,False),({'status':100},0,True),({'status':2},0,False)])
def test_cached_speed_scalar_is_not_fresh_terminal_proof(ack, value, verified):
    driver = object.__new__(BioXpTester)
    driver.motor_get_axis_param = lambda *a, **k: {'ack':ack,'value':value}
    row = driver.motor_get_speed(4,1)
    assert row['speed'] == (100000 if ack == {'status':2} else 0)
    if ack == {'status':2}:
        return
    wait = driver.motor_wait_stopped(4,1,min_polls=1)
    assert wait['stopped'] is True  # OEM scalar behavior is retained.
    assert wait.get('controller_terminal_state_verified') is verified
    assert wait.get('speed_reply_valid') is verified


@pytest.mark.parametrize('no24v,initialized', [(True,True),(True,False),(False,False)])
def test_home_check_stopped_short_circuits_without_speed_query(no24v, initialized):
    driver = object.__new__(BioXpTester)
    driver.oem_no24v_state = lambda: no24v
    driver._oem_board_state = lambda: {4:initialized}
    driver.motor_get_speed = lambda *a, **k: pytest.fail('OEM predicate must not query speed')
    row = driver.motor_wait_stopped(4,1,min_polls=1,oem_board_predicate=True)
    assert row['stopped'] is True
    assert row['controller_terminal_state_verified'] is False
    assert row['polls'] == 0


@pytest.mark.parametrize('no24v,initialized', [(True,False),(False,False),(False,True)])
def test_board_stop_no24v_before_initialization_and_void_return(no24v, initialized):
    driver = object.__new__(BioXpTester)
    driver.oem_no24v_state = lambda: no24v
    driver._oem_board_state = lambda: {4:initialized}
    calls = []
    driver.motor_oem_stop_exact = lambda *a, **k: calls.append('stop') or {'ok':False,'source_return_code':1}
    if no24v:
        with pytest.raises(RuntimeError, match='Lost 24V'): driver.motor_oem_board_stop(4,1,axis_name='z')
    else:
        row = driver.motor_oem_board_stop(4,1,axis_name='z')
        assert row['source_call_completed'] is True
        if initialized:
            assert row['source_board_return'] is None
            assert row['source_return_code'] == 1  # retained leaf result, not board return
        else:
            assert row['source_noop'] is True
    assert calls == (['stop'] if initialized and not no24v else [])
