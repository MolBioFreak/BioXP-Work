"""Independent locked-IL vectors: Head 5ee0/6210, Thermal ab38, CI19a44.
No real transport, runtime or device is constructed. Source Set is not wire proof.
"""
import asyncio
from types import SimpleNamespace
import pytest
from bioxp.usb_driver import BioXpTester
from bioxp.novo_router import NovoRouter
from bioxp.novo_usb_can import novo_decode
import bioxp.usb_driver as module


def driver():
    d = object.__new__(BioXpTester)
    d.novo_router = NovoRouter(ep_in=object(), ep_out=object(), decode=novo_decode)
    d._oem_motor_initial_signals = set()
    d._oem_position_cache = {(6, 0): 100}
    d._oem_board_state = lambda: {4: True, 5: True, 6: True}
    d._oem_board_present = lambda b: b in (4, 5, 6)
    d._motion_oem_axis_profile = lambda key, **kw: {'board': 4, 'motor': 2, 'axis_max_steps': 80000}
    d.collect_bus_events = lambda **kw: []
    d.begin_bus_event_window = lambda **kw: d.novo_router.receive_cursor()
    return d


def wire_event(router, board, motor):
    body = list((99).to_bytes(4, 'big')) + [8, board, 128, 138, 0, 0, 0, motor, 0]
    raw = [126]
    for byte in body + [sum(body) & 255]:
        raw.extend([125, byte ^ 32] if byte in (125, 126) else [byte])
    raw = bytes(raw + [126])
    router._dispatch(router._decode_record(novo_decode(raw), raw, 1.0))


@pytest.mark.parametrize('pending', [False, True])
@pytest.mark.parametrize('during', [False, True])
def test_thermal_preclear_abort_set_releases_wait_then_queries_position_branch4(monkeypatch, pending, during):
    d = driver(); calls = []
    monkeypatch.setattr(module.time, 'sleep', lambda s: calls.append(('sleep', s)))
    def abort():
        calls.append(('abort',))
        if pending:
            wire_event(d.novo_router, 6, 0)
        d.motor_oem_force_abort_motion()
    def send(board, command, typ, motor, value, **kw):
        calls.append(('tx', board, command, typ, motor, value))
        if command == 4 and not during:
            abort()
        return {'status': 100}
    d._send_motor = send
    def collect(**kw):
        calls.append(('wait_poll',)); abort(); return []
    d.collect_bus_events = collect
    d.motor_get_position = lambda b, **kw: calls.append(('position', b, kw['motor'])) or {'position': 2100}
    with pytest.raises(RuntimeError, match='moveSteps4'):
        d._motor_oem_door_preclear(6, 0, {'min_steps': 0, 'max_steps': 100000})
    assert calls == [('tx', 6, 138, 0, 0, 0), ('tx', 6, 4, 1, 0, 2000)] + ([('wait_poll',)] if during else []) + [('abort',), ('sleep', .001), ('position', 6, 0)]
    assert d.novo_router.take_motor_events([(6,0)]) == []
    assert d.oem_no24v_state()


@pytest.mark.parametrize('board,motor', [(4,0),(4,1),(4,2),(5,0),(6,0)])
def test_abort_latch_consumes_once_coalesces_and_is_not_wire_proof(monkeypatch, board, motor):
    d = driver(); monkeypatch.setattr(module.time, 'sleep', lambda s: None)
    wire_event(d.novo_router, board, motor)
    d.motor_oem_force_abort_motion()
    result = d.motor_oem_wait_target_reached(board, motor, timeout_s=0)
    assert result['ok'] and result['completion_class'] == 'oem_abort_latch'
    assert result['target_reached'] is False and result['source_wait_signaled'] is True
    assert result['event'] is None and result['physical_effect_verified'] is False
    assert d.novo_router.take_motor_events([(board,motor)]) == []


@pytest.mark.parametrize('nonnull', [False, True])
def test_query138_only_nonnull_resets_abort_set(monkeypatch, nonnull):
    d = driver(); monkeypatch.setattr(module.time, 'sleep', lambda s: None)
    d.motor_oem_force_abort_motion()
    d._send_motor = lambda *a, **kw: {'status': 2} if nonnull else None
    d.motor_query_motor_stop(6,0)
    result = d.novo_router.take_motor_events([(6,0)])
    assert bool(result) is (not nonnull)


def test_waitall_preserves_partial_software_set_and_pending_wire():
    d = driver(); r = d.novo_router
    r.set_motor_abort_event(4,2)
    assert r.take_motor_events([(4,2),(5,0)]) == []
    wire_event(r,5,0)
    result = r.take_motor_events([(4,2),(5,0)])
    assert len(result) == 2 and result[0]['source'] == 'board.forceAbortMotion'
    assert r.take_motor_events([(4,2)]) == []


def test_abort_preserves_addressed_stop_guard(monkeypatch):
    d = driver(); monkeypatch.setattr(module.time, 'sleep', lambda s: None)
    d._send_motor = lambda *a, **kw: pytest.fail('No24V Stop must not transmit')
    d.motor_oem_force_abort_motion()
    with pytest.raises(RuntimeError, match='Lost 24V'):
        d.motor_oem_board_stop(4, motor=2, axis_name='g')


@pytest.mark.parametrize('version,speed', [(0,600),(1,1500)])
@pytest.mark.parametrize('nested_consumes', [False,True])
def test_g_recovery_order_flag_clear_nested_g_y_then_same_latch_wait(monkeypatch, version, speed, nested_consumes):
    d = driver(); calls = []
    d._motion_oem_gripper_version = lambda: version
    def sleep(s):
        calls.append(('sleep',s))
        if s == 10:
            d._oem_24v_dropped = True
    monkeypatch.setattr(module.time, 'sleep', sleep)
    def position(b, **kw):
        calls.append(('position',b,kw['motor']))
        return {'position': 10 if len(calls)==1 else 33000}
    d.motor_get_position = position
    def send(b,c,t,m,v,**kw):
        calls.append(('tx',b,c,t,m,v))
        if c == 4:
            wire_event(d.novo_router,b,m)
        return {'status':100}
    d._send_motor = send
    def nested(axis, **kw):
        calls.append(('home',axis,kw['speed'],d.oem_no24v_state()))
        if axis == 'g' and nested_consumes:
            assert d.novo_router.take_motor_events([(4,2)])
        return {'ok':True}
    d.motor_oem_axis_search_home = nested
    actual_wait = d.motor_oem_wait_target_reached
    def wait(b, motor, timeout_s, event_window):
        calls.append(('wait',b,motor,timeout_s))
        return actual_wait(b,motor,timeout_s=0,event_window=event_window)
    d.motor_oem_wait_target_reached = wait
    result = d.motor_oem_move_absolute(4,33000,motor=2,gripper_recover=True)
    expected = [('position',4,2),('tx',4,138,0,2,0),('sleep',.001),('tx',4,4,0,2,33000),('sleep',10.0),('home','g',speed,False),('sleep',5.0),('home','y',1000,False),('wait',4,2,20.0)]
    if nested_consumes: expected += [('position',4,2)]
    assert calls == expected
    assert result['completion_class'] == ('oem_timeout_target_equal' if nested_consumes else 'event_128')


@pytest.mark.parametrize('recoveries', [0,1,2])
def test_g_active_loop_source_repeated_recovery_no_added_telemetry(monkeypatch,recoveries):
    d = driver(); calls=[]; clock=[0.0]; polls=[0]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(module.time,'time',lambda:clock[0])
    d.motor_move_left = lambda b, **kw: calls.append(('left',b,kw['motor'],kw['speed'])) or {'ack':{'status':100}}
    def speed(b,**kw):
        calls.append(('speed',b,kw['motor']))
        polls[0]+=1
        if polls[0]<=recoveries: clock[0]=31.0
        return {'speed': 10 if polls[0]<=recoveries else 0,'ack':{'status':100},'speed_reply_valid':True}
    d.motor_get_speed=speed
    d._motor_oem_gripper_recovery_position=lambda:33000
    def recovery(b,p,**kw):
        calls.append(('recover',b,p,kw)); return {'ok':True}
    d.motor_oem_move_absolute=recovery
    d.motor_query_home_switch=lambda b,**kw:calls.append(('home',b,kw['motor'])) or {'home':True}
    d.motor_oem_board_stop=lambda b,**kw:calls.append(('stop',b,kw['motor'])) or {}
    d.motor_get_position=lambda b,**kw:calls.append(('position',b,kw['motor'])) or {'position':-7}
    d.motor_set_home=lambda b,**kw:calls.append(('sethome',b,kw['motor'])) or {}
    result=d.motor_oem_go_home('g',speed=200,rehome=False,timeout_s=.1)
    expected=[('left',4,2,200)]
    for _ in range(recoveries):expected += [('speed',4,2),('recover',4,33000,{'motor':2,'wait_for_stop':True,'gripper_recover':True})]
    expected += [('speed',4,2)]
    if not recoveries:expected += [('home',4,2),('stop',4,2),('position',4,2),('sethome',4,2)]
    assert calls == expected
    assert result['source_return_code'] == (0 if recoveries else 7)
    assert result['physical_effect_verified'] is False


@pytest.mark.parametrize('version,wide,offset,expected',[(0,999,5,59505),(1,33020,5,33020),(1,0,5,33005),(2,33020,5,None)])
def test_source_status4_binding(monkeypatch,version,wide,offset,expected):
    import bioxp.oem_initialization as init
    d=driver();d._motion_oem_gripper_version=lambda:version
    monkeypatch.setattr(init,'build_machine_calibration_manifest',lambda:{'ok':True,'gripper':{'GripperOpenWide':{'value':wide},'originOffsetG':{'value':offset}}})
    if expected is None:
        with pytest.raises(RuntimeError,match='binding unavailable'):d._motor_oem_gripper_recovery_position()
    else:assert d._motor_oem_gripper_recovery_position()==expected


def test_public_provider_abort_has_no_motor_tx_or_controller_proof(monkeypatch):
    from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
    d=driver();calls=[];monkeypatch.setattr(module.time,'sleep',lambda s:calls.append(('sleep',s)))
    d._send_motor=lambda *a,**kw:pytest.fail('CI forceAbort has no transmission')
    p=object.__new__(Serial206ProductionPrimitiveAdapter);p.tester=d;p.reference_store=None
    result=p.x_abort(reason='operator')
    assert calls==[('sleep',.001)]
    assert result['ok'] and result['logical_abort']['affected_waiters']==['y','z','g','x','door']
    assert result['controller_command_acknowledged'] is False
    assert result['controller_terminal_state_verified'] is False
    assert result['physical_scope']=='none_software_flags_and_waiters'


@pytest.mark.parametrize('manual',[False,True])
@pytest.mark.parametrize('version',[0,1,2])
@pytest.mark.parametrize('raises',[False,True])
def test_g_current_lifetime_normal_only_no_standby(monkeypatch,manual,version,raises):
    d=driver();calls=[]
    preset={'board':4,'motor':2,'gripper_version':version}
    d._motion_oem_axis_profile=lambda *a,**k:preset
    d.motor_set_axis_param=lambda b,p,v,**kw:calls.append(('param',b,p,v,kw['motor'])) or {'ok':True}
    def home(axis,**kw):
        calls.append(('home',axis,kw['speed']))
        if raises:raise RuntimeError('source fault')
        return {'ok':True}
    d.motor_oem_go_home=home;d.motor_oem_axis_search_home=home
    def run():
        return d.motor_oem_home_axis('g') if manual else d.motor_oem_home_axis_board_test('g')
    if raises:
        with pytest.raises(RuntimeError,match='source fault'):run()
    else:run()
    speed=(600 if version==0 else 200) if manual else (150 if version==0 else 200)
    expected=[('param',4,6,31,2)]
    if not manual:expected += [('param',4,205,5,2)]
    expected += [('home','g',speed)]
    if version==1 and not raises:expected += [('param',4,6,10,2)]
    assert calls==expected


@pytest.mark.parametrize('manual',[False,True])
def test_g_missing_board_no_current_writes(manual):
    d=driver();d._oem_board_present=lambda b:False
    d.motor_set_axis_param=lambda *a,**kw:pytest.fail('board null')
    result=d.motor_oem_home_axis('g') if manual else d.motor_oem_home_axis_board_test('g')
    assert result['source_noop']=='board_null'
    if manual:
        assert result['home']['source_noop'] is True
        from bioxp.oem_gripper import gripper_home
        receipt=gripper_home(d,operator_ack='GRIPPER_HOME',reason='offline absent-board check')
        assert receipt['motion_commanded'] is False
        assert receipt['physical_effect_verified'] is False


def test_axis_search_stores_speed_before_query_home():
    d=driver();calls=[]
    d.motor_set_home=lambda *a,**kw:calls.append('sethome') or {}
    def home(*a,**kw):
        calls.append(('home',d._oem_search_home_speeds[(4,2)]));return {'home':False}
    d.motor_query_home_switch=home
    d.motor_oem_go_home=lambda *a,**kw:calls.append(('go',d._oem_motor_home_cache[(4,2)])) or {'ok':True}
    d.motor_oem_axis_search_home('g',speed=1500)
    assert calls==['sethome',('home',1500),('go',False)]


@pytest.mark.parametrize('valid',[False,True])
def test_diagnostic_stop_cached_zero_not_fresh_proof(monkeypatch,valid):
    import bioxp.api as api
    d=driver();d.motor_stop=lambda *a,**kw:{'ok':True}
    monkeypatch.setattr(api,'restore_gripper_idle_current',lambda *a,**kw:{'ok':True})
    monkeypatch.setattr(api,'_collect_axis_diagnostic_status',lambda *a,**kw:{'rows':{'g':{'speed':{'speed':0,'speed_reply_valid':valid,'ack':{'status':100} if valid else None},'current':{'run_current_param6':10,'standby_current_param7':10}}}})
    async def execute(label,fn,**kw):return fn(d)
    monkeypatch.setattr(api,'_run_safety_interrupt_blocking',execute)
    req=api.AxisDiagnosticStopRequest(axis='g',operator_ack='STOP_AXIS')
    if valid:assert asyncio.run(api.motion_diagnostics_stop(req))['verified_stopped'] is True
    else:
        with pytest.raises(api.HTTPException) as e:asyncio.run(api.motion_diagnostics_stop(req))
        assert e.value.detail['verified_stopped'] is False


@pytest.mark.parametrize('case',['No24V','uninitialized','equal','high_limit'])
def test_g_recovery_ordinary_entry_guards_precede_clear(monkeypatch,case):
    d=driver(); calls=[]
    monkeypatch.setattr(module.time,'sleep',lambda s:pytest.fail('early return must not clear/recover'))
    d._oem_24v_dropped=case=='No24V'
    if case=='uninitialized':d._oem_board_state=lambda:{4:False}
    d.motor_get_position=lambda *a,**k:calls.append('position') or {'position':33000 if case=='equal' else 79995}
    if case=='No24V':
        with pytest.raises(RuntimeError,match='move abs1'):d.motor_oem_move_absolute(4,33000,motor=2,gripper_recover=True)
        assert calls==[]
    else:
        result=d.motor_oem_move_absolute(4,90000 if case=='high_limit' else 33000,motor=2,gripper_recover=True)
        assert result['source_return_code']=={'uninitialized':1,'equal':33000,'high_limit':79995}[case]


def test_g_version_preserves_unknown_not_version_zero():
    d=driver();d._machine_config_bundle=lambda:{'ok':True,'config':{'config':{'GripperVersion':2}}}
    assert d._motion_oem_gripper_version()==2


@pytest.mark.parametrize('caught',[False,True])
def test_home_gz_second_z_wrapper_current_and_exception_lifetime(monkeypatch,caught):
    d=driver();calls=[]
    d._motion_oem_axis_profile=lambda axis,**kw:{'board':4,'motor':1 if axis=='z' else 2,'axis_max_steps':160000}
    d.motor_set_axis_param=lambda b,p,v,**kw:calls.append(('current',kw['motor'],v)) or {'ok':True}
    d.motor_oem_move_absolute=lambda b,p,**kw:calls.append(('abs',kw['motor'],p,kw['wait_for_stop'])) or {'ok':True}
    d.motor_oem_go_home=lambda *a,**kw:calls.append(('home',kw['speed'])) or {'ok':True,'source_return_code':0 if caught else 1}
    d.deck_io_set_type=lambda *a:calls.append(('solenoid',a)) or {'ok':True}
    d._mark_oem_latch_unlocked=lambda:calls.append(('latch',False))
    monkeypatch.setattr(module.time,'sleep',lambda s:calls.append(('sleep',s)))
    result=d.motor_oem_home_gz(delay_s=2,gripper_version=1,development_machine=False,caught_plate_x_home=lambda:calls.append(('xhome',)) or {})
    expected=[('current',2,31),('current',1,31),('abs',1,65000,False),('sleep',2.0),('home',200)]
    expected += [('xhome',),('solenoid',(2,0)),('latch',False)] if caught else [('current',1,31),('abs',1,65000,True),('current',2,10)]
    assert calls==expected
    assert result['ok'] is (not caught)


def test_diagnostic_g_receipt_fences_addressed_stop_not_aggregate_abort(monkeypatch,tmp_path):
    from test_operator_controls import make_app, action_for
    from bioxp.operator_command_plane import OperatorCommandStore
    from fastapi.testclient import TestClient
    calls=[]
    original=OperatorCommandStore.mark_interrupt_delivery_active
    def mark(self,ident,action):
        calls.append(action);return original(self,ident,action)
    monkeypatch.setattr(OperatorCommandStore,'mark_interrupt_delivery_active',mark)
    receipts=[]
    begin=OperatorCommandStore.begin_interrupt
    def begun(self,*args,**kwargs):
        receipt=begin(self,*args,**kwargs);receipts.append(receipt);return receipt
    monkeypatch.setattr(OperatorCommandStore,'begin_interrupt',begun)
    app,dispatches=make_app(tmp_path,monkeypatch)
    client=TestClient(app)
    catalog=client.get('/operator/control-catalog').json()
    action=action_for(catalog,'POST','/motion/diagnostics/stop')
    response=client.post('/operator/actions/'+action['action_id'],json={'expected_generation':catalog['ownership_generation'],'idempotency_key':'addressed-g-stop','inputs':{'axis':'g'}})
    assert response.status_code==200, response.text
    # Outer delivery fence then command-plane reconciliation fence; one dispatch.
    assert calls==['oem.g.stop','oem.g.stop']
    assert dispatches==[('diagnostic_stop',None)]
    assert len(receipts)==1 and receipts[0]['scope']=='g'
    assert receipts[0]['oem_abort_latched'] is False
