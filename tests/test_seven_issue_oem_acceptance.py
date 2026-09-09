"""S1–S7 acceptance probes against OEM software defaults and call semantics.
No devices/network; real disposable runtime SQLite for construction/reopen.
"""
from types import SimpleNamespace
import threading
import pytest
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.oem_serial206_initialization import (
    Serial206OemInitializationProvider as Provider,
    SERIAL206_INITIALIZE_MOTION_STAGE_SPECS,
    SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS,
)
from bioxp.usb_driver import BioXpTester
from test_oem_deck_install_binding import _operator_store
from test_r4_deck_collection_integration import installed, action, submit, rig
from bioxp import api
import time


@pytest.mark.parametrize('installed', ['fresh'], indirect=True)
def test_s1_s3_fresh_no_tip_collection_to_named_move(installed):
    app,p,runtime,*_ = installed
    spec=next(s for s in SERIAL206_INITIALIZE_MOTION_STAGE_SPECS if s.key=='initializeMotion.tip_loaded_false.no_tip')
    p._apply_initialize_motion_transition(runtime.state,spec,{'ok':True})
    # No location, well, tray or latch values are seeded. The only fake facts
    # supplied by installed are actual hardware/reference observations.
    result=api._collect_and_publish_hardware_snapshot(['axes','latch'],reason='fresh-constructor')
    assert result['deck_authority']['enabled'] is True,result
    store=app.state.operator_command_plane.store
    assert store.deck_semantic_state()['current_location']=='LOC_MS'
    assert store.deck_semantic_state()['tip_loaded'] is False
    assert store.tip_tray_state(4)['occupancy']==[False]*96
    command_id=submit(app,key='fresh-ctor-named-move')
    app.state.operator_command_plane.start()
    deadline=time.monotonic()+3
    while time.monotonic()<deadline:
        receipt=store.get_command(command_id)
        if receipt['status'] in {'completed','failed','rejected'}:break
        time.sleep(.01)
    assert receipt['status']=='completed',receipt



def provider(tmp_path):
    runtime = OEMRuntimeStore(tmp_path / 'runtime')
    p = Provider(SimpleNamespace(), state_store=runtime, generation_provider=lambda: 7)
    return p, runtime


def stage(p, key):
    spec = next(s for s in SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS if s.key == key)
    return p._execute_stage(spec, timeout_s=30)


def test_s1_real_sqlite_constructor_and_reopen(tmp_path):
    p, runtime = provider(tmp_path)
    state = p._load_state()
    m = state['machine_status']
    assert (m['current_location'],m['current_well'],m['tip_loaded'],m['tip_dirty'],m['tip_location']) == (0,0,False,False,-1)
    saved = runtime.read_oem_serial206_initialization_state()
    assert saved is not None
    assert saved['machine_status'] == m
    m.update(current_location=6,current_well=12,tip_dirty=True)
    p._save_state(state)
    other = Provider(SimpleNamespace(),state_store=OEMRuntimeStore(tmp_path/'runtime'),generation_provider=lambda:7)
    assert other._load_state()['machine_status'] == m


def test_s2_constructor_reaches_existing_tray_reader_without_hardware(tmp_path):
    p, runtime = provider(tmp_path)
    store = _operator_store(tmp_path/'operator')
    try:
        p.bind_deck_semantic_state_reader(store.deck_semantic_state)
        p.bind_deck_semantic_state_publisher(store.publish_deck_owner_state)
        p.bind_tip_tray_state_reader(store.tip_tray_state)
        p.bind_tip_tray_state_publisher(store.publish_tip_tray_transition)
        p._load_state()
        p.refresh_deck_semantic_bootstrap(expected_generation=7)
        trays=[store.tip_tray_state(i) for i in range(5)]
        assert all(t['tip_available'] is True for t in trays), p.deck_semantic_bootstrap_diagnostic()
        assert trays[4]['occupancy'] == [False]*96
    finally:
        store.stop()


def test_s3_no_tip_preserves_constructor_and_retained_values(tmp_path):
    p,_=provider(tmp_path)
    spec=next(s for s in SERIAL206_INITIALIZE_MOTION_STAGE_SPECS if s.key=='initializeMotion.tip_loaded_false.no_tip')
    state=p._load_state()
    p._apply_initialize_motion_transition(state,spec,{'ok':True})
    m=state['machine_status']
    assert (m['current_location'],m['current_well'],m['tip_dirty']) == (0,0,False)
    m.update(current_location=6,current_well=7,tip_dirty=True,tip_loaded=True)
    p._apply_initialize_motion_transition(state,spec,{'ok':True})
    assert (m['current_location'],m['current_well'],m['tip_dirty'],m['tip_loaded']) == (6,7,True,False)


def test_s4_initializer_uses_direct_board_primitive():
    calls=[]
    primitive=SimpleNamespace(motor_oem_board_move_steps=lambda *a,**k:calls.append((a,k)) or {'source_call_completed':True,'board_wrapper_return':-1})
    p=Provider(primitive)
    result=stage(p,'gripper-clear-10000')
    assert calls==[((4,10000),{'motor':2,'axis':'g','timeout_s':20.0})]
    assert result['board_wrapper_return']==-1


@pytest.mark.parametrize('serial,closed,camera,expected',[(9,False,True,[]),(206,True,True,['confirm']),(206,False,False,['confirm']),(206,False,True,['confirm','open'])])
def test_s5_single_short_circuit_condition(serial,closed,camera,expected):
    calls=[]
    p=Provider(SimpleNamespace(
        oem_initialize_motors_branch_binding=lambda:{'serial_number':serial,'camera_calibrated':camera},
        motor_oem_confirm_thermal_door_closed=lambda:calls.append('confirm') or {'oem_predicates':{'tcDoorClosed':closed}},
        motor_oem_open_thermal_door=lambda **kw:calls.append('open')))
    if expected==['confirm','open']:
        with pytest.raises(RuntimeError,match='Cannot close thermal cycler door'):stage(p,'door-closed-predicate')
    else:assert stage(p,'door-closed-predicate')['ok']
    assert calls==expected


def fallback():
    d=object.__new__(BioXpTester)
    d.novo_router=None
    d._oem_motor_initial_signals=set()
    d.collect_bus_events=lambda **kw:[]
    return d


def event(board):return {'board':board,'motor':0,'status':128,'event_sequence':1}


@pytest.mark.parametrize('software', ['initial','abort'])
@pytest.mark.parametrize('wire_board', [4,5])
def test_s6_mixed_waitall(software,wire_board):
    d=fallback()
    key=(5 if wire_board==4 else 4,0)
    setattr(d,'_oem_motor_initial_signals' if software=='initial' else '_oem_abort_signals',{key})
    d.collect_bus_events=lambda **kw:[event(wire_board)]
    assert d.motor_oem_wait_targets_reached([(4,0),(5,0)],timeout_s=0)['ok']


def test_s6_partial_timeout_retains_wire_for_next_wait():
    d=fallback()
    d.collect_bus_events=lambda **kw:[event(4)]
    assert not d.motor_oem_wait_targets_reached([(4,0),(5,0)],timeout_s=0)['ok']
    d.collect_bus_events=lambda **kw:[event(5)]
    result=d.motor_oem_wait_targets_reached([(4,0),(5,0)],timeout_s=0)
    assert result['ok'],result


def test_s6_partial_signal_consumed_by_one_concurrent_waiter_only():
    d=fallback()
    d.collect_bus_events=lambda **kw:[event(4)]
    assert not d.motor_oem_wait_targets_reached([(4,0),(5,0)],timeout_s=0)['ok']
    d.collect_bus_events=lambda **kw:[]
    barrier=threading.Barrier(3); results=[]
    def run():
        barrier.wait(); results.append(d.motor_oem_wait_target_reached(4,0,timeout_s=0)['ok'])
    ts=[threading.Thread(target=run) for _ in range(2)]
    for t in ts:t.start()
    barrier.wait()
    for t in ts:t.join(2);assert not t.is_alive()
    assert sorted(results)==[False,True]


def test_s7_startup_wrapper_no_preparation_or_restore():
    d=fallback();calls=[]
    d._motion_oem_axis_profile=lambda *a,**k:{'board':4,'motor':2,'home_speed':200}
    d.motor_oem_axis_search_home=lambda *a,**k:calls.append(('home',a,k)) or {'ok':True}
    d.motor_prepare_axis=lambda *a,**k:pytest.fail('non-OEM startup preparation')
    d.motor_restore_gripper_idle_current=lambda **k:pytest.fail('non-OEM startup restore')
    d.motor_get_axis_param=lambda *a,**k:pytest.fail('non-OEM proof readback')
    d.motor_oem_home_axis('g',startup=True)
    assert [c[0] for c in calls]==['home']


@pytest.mark.parametrize('outcome', ['uninitialized','complete','timeout','power','null'])
def test_s4_actual_board_primitive_call_and_return(outcome):
    d=fallback(); calls=[]
    d._oem_board_present=lambda b:outcome!='null'
    d._oem_board_state=lambda:{4:outcome!='uninitialized'}
    d._oem_24v_dropped=outcome=='power'
    d._oem_position_cache={(4,2):100}
    d._motion_oem_axis_profile=lambda *a,**k:{'axis_min_steps':0,'axis_max_steps':80000}
    d.motor_query_motor_stop=lambda *a,**k:calls.append('stop-query') or {'wait_latch_reset':True}
    d.begin_bus_event_window=lambda **k:{}
    d._send_motor=lambda *a,**k:calls.append(('send',a)) or {'status':100}
    d.motor_oem_wait_target_reached=lambda *a,**k:calls.append(('wait',k['timeout_s'])) or {'ok':outcome=='complete'}
    d.motor_get_position=lambda *a,**k:calls.append('position') or {'position':10100}
    if outcome in {'power','null'}:
        with pytest.raises(RuntimeError):d.motor_oem_board_move_steps(4,10000,2,axis='g')
        assert calls==[]
    else:
        result=d.motor_oem_board_move_steps(4,10000,2,axis='g')
        assert result['board_wrapper_return']==(1 if outcome=='uninitialized' else 10100 if outcome=='complete' else -1)
        assert calls==([] if outcome=='uninitialized' else ['stop-query',('send',(4,4,1,2,10000)),('wait',20.0),'position'])


def test_s5_door_home_does_not_evaluate_outer_condition():
    calls=[]
    p=Provider(SimpleNamespace(motor_oem_door_search_home=lambda **k:calls.append('home') or {'status_after':{'oem_predicates':{'tcDoorClosed':False}}}))
    stage(p,'door-home')
    assert calls==['home']


def test_s5_open_exception_propagates_once():
    calls=[]
    def opening(**kw):calls.append('open');raise ValueError('open failed')
    p=Provider(SimpleNamespace(oem_initialize_motors_branch_binding=lambda:{'serial_number':206,'camera_calibrated':True},
        motor_oem_confirm_thermal_door_closed=lambda:{'oem_predicates':{'tcDoorClosed':False}},
        motor_oem_open_thermal_door=opening))
    with pytest.raises(ValueError,match='open failed'):stage(p,'door-closed-predicate')
    assert calls==['open']


@pytest.mark.parametrize('kind',['reset','generation','wrong-axis','coalesce'])
def test_s6_signal_retention_boundaries(kind):
    d=fallback()
    d.collect_bus_events=lambda **kw:[event(4),event(4)]
    assert not d.motor_oem_wait_targets_reached([(4,0),(5,0)],timeout_s=0)['ok']
    d.collect_bus_events=lambda **kw:[]
    if kind=='reset':
        d._send_motor=lambda *a,**k:None
        d.motor_query_motor_stop(4,0) # OEM null response does not reset.
        assert d.motor_oem_wait_target_reached(4,0,timeout_s=0)['ok']
    elif kind=='generation':
        d._oem_abort_generation=1
        assert not d.motor_oem_wait_target_reached(4,0,timeout_s=0)['ok']
    elif kind=='wrong-axis':
        assert not d.motor_oem_wait_target_reached(6,0,timeout_s=0)['ok']
        assert d.motor_oem_wait_target_reached(4,0,timeout_s=0)['ok']
    else:
        assert d.motor_oem_wait_target_reached(4,0,timeout_s=0)['ok']
        assert not d.motor_oem_wait_target_reached(4,0,timeout_s=0)['ok']
