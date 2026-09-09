"""Independent ordered source vectors; never construct a USB-connected tester.

ClassNovo.TransmitMessage IL_000e/0014/0041/008a (global lock,1ms,
sendCommand,10ms normal return). Head.goHome RVA5ee0; Deck.goHome
RVA4a40: reads only in the rehome and final queryHome branches.
"""
import threading
from types import SimpleNamespace

import pytest
import bioxp.usb_driver as driver_module
from bioxp.usb_driver import BioXpTester


@pytest.mark.parametrize('outcome', ['ack', 'null', 'usb_error'])
def test_source_whole_transmit_schedule_and_return(monkeypatch, outcome):
    d = object.__new__(BioXpTester)
    events = []
    lock = threading.RLock()
    d._transport_guard = lambda: lock
    d._record_usb_sniff_ledger = lambda *a, **k: None
    d._motor_last_tx_ts = {}
    d._motor_noresp_streak = {}
    monkeypatch.setattr(driver_module.time, 'sleep', lambda s: events.append(('sleep', round(s, 6), lock._is_owned())))
    def transact(raw, **kw):
        # raw decoded by independent framing reader would be board-specific;
        # this boundary records complete sendCommand calls, not retry writes.
        events.append(('sendCommand', kw['provenance']['expected_board'], kw['timeout_s'], kw['ordinary_motor_retry'], lock._is_owned()))
        if outcome == 'usb_error':
            raise driver_module.usb.core.USBError('synthetic I/O failure')
        return {'ok': outcome == 'ack', 'frames': [{'data': [4,100,6,0,0,0,9,0]}] if outcome == 'ack' else []}
    d.novo_router = SimpleNamespace(transact=transact, tmcl_matcher=lambda **k: object())
    for board in (4,5):
        result = d._send_motor(board,6,1,0,0,read_timeout_ms=60000,max_reads=1,ordinary_motor_retry=True)
        events.append(('return', None if result is None else result['value']))
    expected = []
    for board in (4,5):
        expected.extend([('sleep',.001,True), ('sendCommand',board,60.0,True,True)])
        if outcome != 'usb_error':
            expected.append(('sleep',.010,True))
        expected.append(('return',9 if outcome == 'ack' else None))
    assert events == expected


def home_driver(events, monkeypatch, axis, rehome, speed_values=(7,0), switch_values=(False,True)):
    d = object.__new__(BioXpTester)
    board, motor = {'x':(5,0),'y':(4,0),'z':(4,1),'g':(4,2),'door':(6,0)}[axis]
    d.oem_no24v_state = lambda: False
    d._oem_board_state = lambda: {board: True}
    d._motion_oem_axis_profile = lambda *a, **k: {'board':board,'motor':motor}
    d.begin_bus_event_window = lambda: {}
    d._oem_motor_home_cache = {}
    d._oem_position_cache = {(board,motor):123}
    def record(name, value):
        events.append(name)
        return value
    speeds = iter(speed_values)
    switches = iter(switch_values)
    monkeypatch.setattr(driver_module.time, 'sleep', lambda s: events.append(('sleep',s)))
    d.motor_get_speed = lambda *a, **k: record('GAP3', {'speed':next(speeds),'speed_reply_valid':True,'ack':{'status':100}})
    d.motor_query_home_switch = lambda *a, **k: record('GAP9', {'home':next(switches), 'reply_valid':True,'ack':{'status':100}})
    d.motor_get_position = lambda *a, **k: record('GAP1', {'position':-123,'ack':{'status':100}})
    d.motor_get_switch_activity = lambda *a, **k: record('extra_switches', {})
    d.motor_oem_move_absolute = lambda *a, **k: record('board_abs10000', {'ok':True,'source_return_code':10000})
    d.motor_move_left = lambda *a, **k: record('MoveLeft', {'ack':{'status':100}})
    d.motor_oem_board_stop = lambda *a, **k: record('board_StopMotor', {'ok':True,'first_delivery':{'status':100},'second_delivery':{'status':100}})
    d.motor_set_home = lambda *a, **k: record('setHome', {'ack':{'status':100}})
    return d


@pytest.mark.parametrize('axis', ['x','y','z'])
@pytest.mark.parametrize('rehome', [False,True])
def test_home_order_no_extra_reads_or_poll_sleep(monkeypatch, axis, rehome):
    events = []
    # Additional values let the baseline expose its full wrong sequence rather
    # than fail solely from exhausted fixture responses.
    d = home_driver(events,monkeypatch,axis,rehome,speed_values=(7,7,0),switch_values=(False,False,True,True))
    result = d.motor_oem_go_home(axis,speed=250,rehome=rehome)
    events.append(('return',result['source_return_code']))
    expected = (['board_abs10000'] + (['GAP1'] if axis == 'x' else [])) if rehome else []
    expected += ['MoveLeft','GAP3','GAP3','GAP3','GAP9','GAP9','GAP9','board_StopMotor','GAP1','setHome',('return',123)]
    assert events == expected
    assert result['home_after'] is result['home_hit']
    assert result['position_after_sethome'] is None
    assert result['controller_home_proof_verified'] is True
    assert result['physical_effect_verified'] is False


@pytest.mark.parametrize('axis', ['x','y','z'])
def test_home_exception_order_no_followup_proof(monkeypatch, axis):
    events = []
    d = home_driver(events,monkeypatch,axis,True,speed_values=(0,0),switch_values=(False,True))
    def fail(*a, **k):
        events.append('board_abs10000')
        raise RuntimeError('source absolute timeout')
    d.motor_oem_move_absolute = fail
    with pytest.raises(RuntimeError,match='source absolute timeout'):
        d.motor_oem_go_home(axis,speed=250,rehome=True)
    events.append('exception')
    assert events == ['board_abs10000','exception']


@pytest.mark.parametrize('axis', ['x','y','z','g','door'])
def test_axis_search_preparation_only_source_switch(monkeypatch, axis):
    events = []
    d = home_driver(events,monkeypatch,axis,False,switch_values=(True,))
    d.motor_oem_go_home = lambda *a, **k: events.append(('goHome',k['rehome'])) or {'ok':True,'source_return_code':0}
    result = d.motor_oem_axis_search_home(axis,speed=250)
    events.append(('return',result['source_return_code']))
    assert events == ['setHome','GAP9','board_abs10000',('sleep',.5),('goHome',False),('return',0)]
    assert result['switches_before_axis_search'] is None


def decode_tx(raw):
    body = []
    escaped = False
    for byte in raw[1:-1]:
        if escaped:
            body.append(byte ^ 32)
            escaped = False
        elif byte == 125:
            escaped = True
        else:
            body.append(byte)
    assert sum(body[:-1]) & 255 == body[-1]
    assert body[4] == 7
    return (int.from_bytes(bytes(body[:4]), 'big'), body[5], body[6], body[7], int.from_bytes(bytes(body[8:12]), 'big', signed=True))


@pytest.mark.parametrize('axis,board,motor', [('x',5,0),('y',4,0),('z',4,1)])
@pytest.mark.parametrize('nulls', [False,True])
def test_home_full_real_leaf_tx_sleep_return_chain(monkeypatch, axis, board, motor, nulls):
    d = object.__new__(BioXpTester)
    events = []
    lock = threading.RLock()
    d._transport_guard = lambda: lock
    d._record_usb_sniff_ledger = lambda *a, **k: None
    d._motor_last_tx_ts = {}
    d._motor_noresp_streak = {}
    d._oem_position_cache = {(board,motor):123}
    d._oem_motor_home_cache = {}
    d.oem_no24v_state = lambda: False
    d._oem_board_state = lambda: {board:True}
    d._motion_oem_axis_profile = lambda *a, **k: {'board':board,'motor':motor}
    d.begin_bus_event_window = lambda: {}
    monkeypatch.setattr(driver_module.time,'sleep',lambda s: events.append(('sleep',round(s,6))))
    def transact(raw, **kw):
        tx = decode_tx(raw)
        events.append(('TX',tx,kw['timeout_s']))
        assert lock._is_owned()
        assert kw['ordinary_motor_retry'] is True
        command, param = tx[1:3]
        if nulls and ((command == 6 and param in (3,9)) or command == 5):
            return {'ok':False,'frames':[]}
        value = -123 if (command,param) == (6,1) else 1 if (command,param) == (6,9) else 0
        data = [board,100,command,*value.to_bytes(4,'big',signed=True),0]
        return {'ok':True,'frames':[{'data':data}]}
    d.novo_router = SimpleNamespace(transact=transact,tmcl_matcher=lambda **k: object())
    row = d.motor_oem_go_home(axis,speed=250,rehome=False)
    events.append(('return',row['source_return_code']))
    calls = [(2,0,250,1.0),(6,3,0,60.0)]
    if nulls:
        calls.append((6,3,0,60.0))  # queryMotorSpeed's second WHOLE call
    calls += [(6,9,0,60.0),(3,0,0,60.0),(3,0,0,60.0),(6,1,0,60.0),(5,1,0,60.0)]
    expected = []
    for command,param,value,budget in calls:
        expected += [('sleep',.001),('TX',(board,command,param,motor,value),budget),('sleep',.010)]
    expected.append(('return',123))
    assert events == expected
    assert row['controller_home_proof_verified'] is (not nulls)
    assert row['controller_terminal_state_verified'] is (not nulls)
    assert row['physical_effect_verified'] is False


@pytest.mark.parametrize('ack,scalar', [(None,0),({'status':100},0),({'status':2},1)])
def test_generic_relative_nonzero_has_no_board_absolute_sleep(monkeypatch, ack, scalar):
    # Thermal.doorSearchHome -> moveSteps(+2000) -> query138 then relative4.
    # ClassMotor.MovetoRelPosition has no Sleep; TransmitMessage owns pacing.
    d = object.__new__(BioXpTester)
    events = []
    monkeypatch.setattr(driver_module.time,'sleep',lambda s: events.append(('sleep',s)))
    d.motor_query_motor_stop = lambda *a, **k: events.append(('query138',6,0)) or {'ack':{'status':100}}
    def send(*args, **kwargs):
        events.append(('TX',args,kwargs['read_timeout_ms'],kwargs['ordinary_motor_retry']))
        return ack
    d._send_motor = send
    result = d.motor_move_relative(6,2000,motor=0)
    events.append(('return',result['source_return_code']))
    assert events == [('query138',6,0),('TX',(6,4,1,0,2000),60000,True),('return',scalar)]
    assert result['controller_acknowledged'] is (ack == {'status':100})
