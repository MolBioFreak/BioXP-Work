"""Seven-issue lifecycle and rejection vectors; disposable SQLite only."""
import copy
import pytest
from types import SimpleNamespace
from test_seven_issue_oem_acceptance import provider, fallback, event, stage, installed, rig
from test_oem_deck_install_binding import _operator_store
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider as Provider, SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp import api


def bind(p,store):
    p.bind_deck_semantic_state_reader(store.deck_semantic_state)
    p.bind_deck_semantic_state_publisher(store.publish_deck_owner_state)
    p.bind_tip_tray_state_reader(store.tip_tray_state)
    p.bind_tip_tray_state_publisher(store.publish_tip_tray_transition)
    p.deck_owner_authority_stamps=lambda:{'ownership_generation':7,'board_epoch_4':10,'board_epoch_5':11}
    store.bind_deck_owner_authority_reader(p.deck_owner_authority_stamps)


def test_s2_interrupted_publication_resume_depletion_and_restart(tmp_path):
    p,r=provider(tmp_path);p._load_state()
    store=_operator_store(tmp_path/'operator');bind(p,store)
    original=store.publish_tip_tray_transition
    def fault(**kw):
        if kw['tray_id']==2:raise OSError('disposable publication fault')
        return original(**kw)
    p._tip_tray_state_publisher=fault
    with pytest.raises(OSError):p._publish_constructed_tip_trays()
    assert store.tip_tray_state(0)['revision']==1
    p._tip_tray_state_publisher=original
    p.publish_tip_tray_transition(tray_id=0,transition='remove_all',operation_id='deplete',command_id='deplete',provenance={'source':'ClassTipTray.removeAll'})
    depleted=copy.deepcopy(store.tip_tray_state(0))
    p._publish_constructed_tip_trays()
    assert store.tip_tray_state(0)==depleted
    assert all(store.tip_tray_state(i)['revision']==1 for i in range(1,5))
    store.stop()
    p2=Provider(SimpleNamespace(),state_store=OEMRuntimeStore(tmp_path/'runtime'),generation_provider=lambda:7)
    reopened=_operator_store(tmp_path/'operator');bind(p2,reopened)
    try:
        p2._publish_constructed_tip_trays()
        assert reopened.tip_tray_state(0)==depleted
        assert reopened.tip_tray_state(4)['occupancy']==[False]*96
    finally:reopened.stop()


@pytest.mark.parametrize('fail_stage',[None,'gripper-clear-10000','gripper-home','door-home'])
def test_s4_full_enclosing_stage_current_lifetime(fail_stage):
    calls=[];current=[None]
    class Primitives:
        def motor_set_axis_param(self,b,param,value,**kw):
            if (b,param,kw.get('motor'))==(4,6,2):current[0]=value;calls.append(('current',value))
            return {'ok':True}
        def motor_oem_board_move_steps(self,*a,**kw):
            assert current[0]==31;calls.append(('clear',10000))
            if fail_stage=='gripper-clear-10000':raise ValueError('source fault')
            return {'source_call_completed':True,'board_wrapper_return':-1}
        def motor_oem_axis_search_home(self,*a,**kw):
            assert current[0]==31;calls.append(('home','g'))
            if fail_stage=='gripper-home':raise ValueError('source fault')
            return {'ok':True}
        def motor_oem_door_search_home(self,**kw):
            assert current[0]==31;calls.append(('home','door'))
            if fail_stage=='door-home':raise ValueError('source fault')
            return {'ok':True}
        def motor_oem_home_axis(self,axis,**kw):
            if axis!='z':assert current[0]==31
            calls.append(('home',axis));return {'ok':True}
        def oem_initialize_motors_branch_binding(self):return {'serial_number':206,'camera_calibrated':True}
        def motor_oem_confirm_thermal_door_closed(self):return {'oem_predicates':{'tcDoorClosed':True}}
        def __getattr__(self,name):return lambda *a,**k:{'ok':True}
    p=Provider(Primitives(),sleep=lambda _:None)
    if fail_stage:
        with pytest.raises(ValueError,match='source fault'):
            for spec in SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS:p._execute_stage(spec,timeout_s=30)
        assert current[0]==31 and ('current',10) not in calls
    else:
        for spec in SERIAL206_INITIALIZE_MOTORS_STAGE_SPECS:p._execute_stage(spec,timeout_s=30)
        assert calls[0]==('home','z') and calls[-1]==('current',10)
        assert calls.count(('current',31))==1 and calls.count(('clear',10000))==1 and calls.count(('home','g'))==1


@pytest.mark.parametrize('host,sensor',[(False,1),(True,0)])
@pytest.mark.parametrize('installed',['fresh'],indirect=True)
def test_s1_fresh_constructor_preserves_independent_latch_refusal(installed,host,sensor):
    app,p,*_=installed
    p.primitives.read_oem_latch_status=lambda:{'ok':True,'value':host,'observation_id':'fresh-host'}
    p.primitives.deck_io_query_type=lambda io_type=3:{'ok':True,'value':sensor,'ack':{'status':100,'value':sensor}}
    result=api._collect_and_publish_hardware_snapshot(['axes','latch'],reason='fresh-refusal')
    assert result['deck_authority']['enabled'] is False
    assert p.primitives.calls==[]


@pytest.mark.parametrize('installed',['fresh'],indirect=True)
def test_s1_fresh_publication_does_not_mask_concurrent_invalidation(installed):
    app,p,*_=installed
    original=p._deck_semantic_bootstrap_publisher
    def invalidate(snapshot):
        result=original(snapshot)
        p.invalidate_deck_authority_cache(reason='concurrent-reference-change')
        return result
    p._deck_semantic_bootstrap_publisher=invalidate
    result=api._collect_and_publish_hardware_snapshot(['axes','latch'],reason='fresh-race')
    assert result['deck_authority']['enabled'] is False
    assert p._deck_authority_cache is None


def test_s6_explicit_reset_discards_retained_wire():
    d=fallback();d.collect_bus_events=lambda **kw:[event(4)]
    assert not d.motor_oem_wait_targets_reached([(4,0),(5,0)],timeout_s=0)['ok']
    d.collect_bus_events=lambda **kw:[]
    d._send_motor=lambda *a,**k:{'status':100}
    d._tmcl_success=lambda ack:True
    d.motor_query_motor_stop(4,0)
    assert not d.motor_oem_wait_target_reached(4,0,timeout_s=0)['ok']


def test_s6_generation_change_during_receive_rejects_wire():
    d=fallback()
    def receive(**kw):d._oem_abort_generation=1;return [event(4),event(5)]
    d.collect_bus_events=receive
    assert not d.motor_oem_wait_targets_reached([(4,0),(5,0)],timeout_s=0)['ok']


@pytest.mark.parametrize('status,motor',[(100,0),(128,9)])
def test_s6_rejected_frames_cannot_complete(status,motor):
    d=fallback();d.collect_bus_events=lambda **kw:[{'board':4,'motor':motor,'status':status,'event_sequence':1}]
    assert not d.motor_oem_wait_target_reached(4,0,timeout_s=0)['ok']


def test_s7_exception_never_restores_current():
    d=fallback();d._motion_oem_axis_profile=lambda *a,**k:{'board':4,'motor':2,'home_speed':200}
    def fault(*a,**k):raise ValueError('home source fault')
    d.motor_oem_axis_search_home=fault
    d.motor_set_axis_param=lambda *a,**k:pytest.fail('startup restore on exception')
    d.motor_restore_gripper_idle_current=lambda **k:pytest.fail('startup restore on exception')
    with pytest.raises(ValueError,match='home source fault'):d.motor_oem_home_axis('g',startup=True)
