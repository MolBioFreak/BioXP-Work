"""Offline qualification: real driver primitives, source routes and receipts.
Run in an unshare -Urn namespace, with both BIOXP runtime roots task-owned.
No BioXpTester construction, USB discovery, sockets or production API startup.
"""
import ast
import contextlib
import os
from pathlib import Path
import socket
import tempfile

import pytest

ROOT = Path(__file__).resolve().parents[2]
assert os.environ.get('BIOXP_OEM_RUNTIME_STATE_ROOT'), 'task-owned runtime required'
assert {name for _, name in socket.if_nameindex()} == {'lo'}, 'isolated network namespace required'
import usb.core

def forbidden(*a, **kw):
    raise AssertionError('offline qualification forbids live hardware/network')
usb.core.find = forbidden
socket.create_connection = forbidden

from bioxp.usb_driver import BioXpTester, OemMotionCompletionError
import bioxp.usb_driver as driver_module
from bioxp import oem_gripper as gripper

class Wire(BioXpTester):
    def __init__(self):
        self.calls = []
        self.position = 50
        self.arrival = None
        self.opened, self.closed = False, False
        self.ack_status = 100
        self.sensor_ack = 100
        self.version = 1
        self.initialized = True
        self.present = True
        self._oem_initial_motor_signal_state = set()
        self.novo_router = None
        self.recoveries = []
    def _oem_board_present(self, board): return self.present
    def _oem_board_state(self): return {4:self.initialized, 6:self.initialized}
    def oem_no24v_state(self): return False
    def _motion_oem_gripper_version(self): return self.version
    def _motion_oem_axis_profile(self, axis, **kw):
        return {'board':6 if axis=='door' else 4, 'motor':0 if axis=='door' else 2,
                'open_position':16000, 'close_position':0, 'stall_guard':6,
                'run_current':31, 'gripper_version':self.version, 'home_speed':50}
    def oem_current_board_lifecycle_generation(self): return 9
    def _motor_oem_door_source_settings(self): return {'serial':206,'camera_calibrated':True,'min_steps':0,'max_steps':160000}
    def _transport_guard(self): return contextlib.nullcontext()
    def _oem_initial_motor_signals(self): return set()
    def begin_bus_event_window(self, **kw): return {'sequence':0}
    def _take_fallback_motor_signals(self, targets, event_window):
        return {f'{b}:{m}':{'board':b,'motor':m,'status':128,'source':'offline_wire'} for b,m in targets}
    def motor_oem_axis_search_home(self, axis, **kw):
        self.recoveries.append((axis,kw))
        return {'ok':True,'physical_effect_verified':False}
    def _send_motor(self, board, cmd, typ, motor, value, **kw):
        self.calls.append((board,cmd,typ,motor,value))
        if cmd==5 and typ==1: self.position=value
        if cmd==2: self.closed=True
        if cmd==4:
            self.position = value if self.arrival is None else self.arrival
            return {'status':self.ack_status,'value':value}
        if cmd==6:
            values={1:self.position,3:0,9:int(self.closed),10:int(self.opened),12:1,13:1,6:31,7:10}
            return {'status':self.sensor_ack if typ in {9,10} else 100,'value':values.get(typ,0)}
        if cmd==13: return {'status':self.sensor_ack,'value':int(self.closed)}
        return {'status':100,'value':value}

@pytest.fixture
def wire(monkeypatch):
    monkeypatch.setattr(driver_module.time,'sleep',lambda _:None)
    monkeypatch.setattr(gripper,'_machine_gripper_config',lambda:{'ok':True})
    monkeypatch.setattr(gripper,'_gripper_calibrated_position',lambda _:100)
    return Wire()

def invoke_g(wire, op='close'):
    return getattr(gripper,'gripper_'+op)(wire,operator_ack='GRIPPER_'+op.upper(),reason='offline')

@pytest.mark.parametrize('start,arrival,expected',[(100,100,True),(0,1,False),(0,100,True)])
def test_gripper_actual_move_to_abs_arrival(wire,start,arrival,expected):
    wire.position, wire.arrival = start,arrival
    result=invoke_g(wire)
    assert result['ok'] is expected,result
    assert result['physical_effect_verified'] is False
    assert result['restore']=={'performed':False,'reason':'manual_handler_retains_parameters'}
    assert not any(row[1]==5 for row in wire.calls), 'Close must retain existing parameters'
    assert result['motion_commanded'] is (start!=100)
    if start==100:
        assert result['source_noop'] is True
        assert not any(row[1]==4 for row in wire.calls)

@pytest.mark.parametrize('version,speed,stall',[(0,100,True),(1,1500,False)])
def test_manual_open_source_current_speed_and_recovery(wire,version,speed,stall):
    wire.version=version
    result=invoke_g(wire,'open')
    assert result['ok'] is True,result
    writes=[(r[2],r[4]) for r in wire.calls if r[1]==5]
    assert writes==[(6,31),(6,31),(4,speed)]+([(205,5)] if stall else [])
    assert [r[0] for r in wire.recoveries]==['g','y']
    assert all(r[2] not in {7,12,13} for r in wire.calls if r[1]==5)

@pytest.mark.parametrize('status',[1,7])
def test_gripper_failed_ack_not_success(wire,status):
    wire.ack_status=status
    assert invoke_g(wire)['ok'] is False

@pytest.mark.parametrize('op,opened,closed,ok',[
    ('open',True,False,True),('open',True,True,False),('open',False,False,False),
    ('close',False,True,True),('close',True,True,False),('close',False,False,False)])
def test_door_real_predicate_producer_and_move_consumer(wire,op,opened,closed,ok):
    wire.opened,wire.closed=opened,closed
    result=getattr(wire,'motor_oem_'+op+'_thermal_door')()
    assert result['ok'] is ok,result
    assert [(r[2],r[4]) for r in wire.calls if r[1]==5]==[(205,8),(6,31)]
    assert any(r[1]==4 for r in wire.calls), 'Intermediate Close must execute source move'
    assert result['wait']['completion_class']=='event_128'
    assert result['wait']['physical_effect_verified'] is False

@pytest.mark.parametrize('op',['open','close'])
def test_door_sensor_failed_ack_not_confirmation(wire,op):
    wire.opened,wire.closed=op=='open',op=='close'
    wire.sensor_ack=7
    result=getattr(wire,'motor_oem_'+op+'_thermal_door')()
    assert result['ok'] is False,result
    assert result['after']['predicates_verified'] is False

@pytest.mark.parametrize('op',['open','close'])
@pytest.mark.parametrize('missing',['home','right'])
def test_missing_door_sensor_reply_never_means_opposite_inactive(wire,op,missing):
    wire.opened,wire.closed=op=='open',op=='close'
    send=wire._send_motor
    def absent(board,cmd,typ,motor,value,**kw):
        if cmd==6 and ((missing=='home' and typ==9) or (missing=='right' and typ==10)):
            return None
        return send(board,cmd,typ,motor,value,**kw)
    wire._send_motor=absent
    result=getattr(wire,'motor_oem_'+op+'_thermal_door')()
    assert result['ok'] is False,result
    assert result['after']['predicates_verified'] is False

@pytest.mark.parametrize('closed',[True,False])
def test_generic_door_home_outer_preclear_order(wire,closed):
    wire.closed=closed
    wire.motor_oem_door_search_home=lambda **kw:{'ok':True,'source_call_completed':True,'physical_effect_verified':False}
    result=wire.motor_oem_home_axis_board_test('door')
    assert result['ok'] is True,result
    ops=[(r[1],r[2],r[4]) for r in wire.calls if r[1] in {4,5}]
    assert ops==([(5,205,8),(4,0,1000)] if closed else [])+[(5,205,6)]


def test_partial_move_exception_preserves_issued_evidence(wire):
    evidence={'command_sent':True,'ack':{'status':100},'wait':{'ok':False},'recovery':{'ok':True}}
    def failed(*a,**kw): raise OemMotionCompletionError('after issue',evidence=evidence)
    wire.motor_oem_move_absolute=failed
    result=invoke_g(wire,'open')
    assert result['ok'] is False and result['motion_commanded'] is True
    assert result['move_to_calibrated']==evidence
    assert len(result['prepare'])==3


@pytest.mark.parametrize('arrival,expected_status',[(100,200),(1,409)])
def test_actual_api_operator_dispatch_sqlite_receipt(wire,monkeypatch,tmp_path,arrival,expected_status):
    from fastapi import FastAPI, HTTPException
    from fastapi.testclient import TestClient
    import bioxp.operator_controls as controls
    from bioxp.oem_runtime_store import OEMRuntimeStore
    import bioxp.runtime_audit_store as audit
    from bioxp.operator_receipt_store import OperatorReceiptStore
    monkeypatch.setenv('BIOXP_OEM_RUNTIME_STATE_ROOT',str(tmp_path))
    monkeypatch.setenv('BIOXP_OEM_RUNTIME_ROOT',str(tmp_path))
    monkeypatch.setattr(audit,'CANONICAL_RUNTIME_ROOT',tmp_path)
    OEMRuntimeStore(tmp_path).close()
    monkeypatch.setattr(controls,'current_release_identity',lambda:{'verified':True,'release_id':'offline','source':{'manifest_sha256':'1'*64,'aggregate_sha256':'2'*64}})
    monkeypatch.setattr(controls,'current_authority_identity',lambda:{'evidence_lock_identity_verified':True,'evidence_lock_sha256':'3'*64})
    monkeypatch.setattr(controls,'current_registry_sha256',lambda:'4'*64)
    class Hardware:
        ownership_epoch=7
        armed=True
        def ownership_projection(self):
            return {'ownership_epoch':7,'ownership':{'transport':'owned','usb':'service','router':'running','CAN_READY':True}}
        def project(self,*names,**kw):
            fresh={'state':'fresh','age_s':0,'fresh_for_s':30}
            observations={'power':{'safety_valid':True},'latch':{'door_sensor':1,'latch_sensor':1},'interlock':{'motion_arm':{'armed':self.armed}}}
            return {'snapshot_id':'offline','freshness':fresh,'domains':{name:{'status':'observed','observation':observations.get(name,{}),'freshness':fresh} for name in names}}
    hardware=Hardware()
    monkeypatch.setattr(controls,'hardware_state',hardware)
    app=FastAPI()
    # Mount exact production route functions without importing production API
    # singleton/startup. Their real gripper producer and operator consumer run.
    async def blocking(label,fn,**kw): return fn()
    ns={'app':app,'HTTPException':HTTPException,'_get_tester':lambda:wire,
        '_require_motion_route_ready':lambda:None,'_run_blocking':blocking,
        'gripper_close':gripper.gripper_close,'gripper_open':gripper.gripper_open}
    tree=ast.parse((ROOT/'src/bioxp/api.py').read_text())
    names={'_gripper_success_or_409','motion_gripper_close','motion_gripper_open',
           '_thermal_door_success_or_409','motion_thermal_door_close','motion_thermal_door_open'}
    nodes=[n for n in tree.body if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef)) and n.name in names]
    assert len(nodes)==6
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(ROOT/'src/bioxp/api.py'),'exec'),ns)
    references={'g':{'state':'referenced'},'door':{'state':'unknown'}}
    controls.install_operator_control_plane(app,
        maintenance_state_provider=lambda:{'motion_blocked':False,'recovery_required':False},
        lifecycle_state_provider=lambda:{'operation_state':'stopped'},
        reference_state_provider=lambda:{'rows':references})
    wire.arrival=arrival
    with TestClient(app) as client:
        catalog=client.get('/operator/control-catalog')
        assert catalog.status_code==200,catalog.text
        action=next(a for a in catalog.json()['actions'] if a['informational_path']=='/motion/gripper/close')
        # Export actual assessed producer rows, not hand-authored enable flags.
        output=os.environ.get('BIOXP_MANUAL_CATALOG_FIXTURE')
        if output and arrival==100:
            import json
            snapshots={}
            for phase,armed,referenced in [('unarmed',False,False),('armed',True,False),('referenced',True,True)]:
                hardware.armed=armed
                references['door']['state']='referenced' if referenced else 'unknown'
                rows=[]
                for item in catalog.json()['actions']:
                    if item['informational_path'] not in {'/motion/gripper/open','/motion/gripper/close','/motion/thermal_door/open','/motion/thermal_door/close'}: continue
                    admission=client.post('/operator/actions/'+item['action_id']+'/admission',json={'expected_generation':7,'inputs':{}})
                    assert admission.status_code==200,admission.text
                    rows.append({**item,**admission.json()})
                snapshots[phase]=rows
            Path(output).write_text(json.dumps(snapshots,indent=2)+'\n')
            hardware.armed=True

        response=client.post('/operator/actions/'+action['action_id'],json={'expected_generation':7,'idempotency_key':'offline-close-result','inputs':{}})
        assert response.status_code==200,response.text
        receipt=response.json()
        assert receipt['response']['http_status']==expected_status,receipt
        command_id=receipt['command_id']
        read=client.get('/operator/actions/receipts/'+command_id+'?detail=true')
        assert read.status_code==200,read.text
        saved=read.json()
        assert saved['response']['http_status']==expected_status,saved
        assert saved['physical_effect_verified'] is False,saved
        restarted=OperatorReceiptStore(tmp_path).by_command(command_id,include_evidence=True)
        assert restarted['response']==saved['response']
        import subprocess, sys, json
        script = "from bioxp.operator_receipt_store import OperatorReceiptStore; import json,sys; print(json.dumps(OperatorReceiptStore(sys.argv[1]).by_command(sys.argv[2],include_evidence=True)))"
        process=subprocess.run([sys.executable,'-c',script,str(tmp_path),command_id],check=True,capture_output=True,text=True)
        assert json.loads(process.stdout)['response']==saved['response']
        output=os.environ.get('BIOXP_MANUAL_RECEIPT_FIXTURE_DIR')
        if output:
            Path(output).mkdir(parents=True,exist_ok=True)
            (Path(output)/('gripper-'+str(expected_status)+'.json')).write_text(json.dumps(saved,indent=2)+'\n')


@pytest.mark.parametrize('case', ['proved','source_noop','bad_sensor','generation_changed'])
def test_actual_door_home_reference_publication_and_admission(wire,monkeypatch,tmp_path,case):
    import asyncio
    from typing import Any, Literal
    from pydantic import BaseModel, ConfigDict
    from fastapi import FastAPI, HTTPException
    from bioxp.services.reference_service import ReferenceStateStore, MarkAxisReferencedCommand
    from bioxp.oem_runtime_store import OEMRuntimeStore
    import bioxp.runtime_audit_store as audit
    from bioxp.operator_controls import _assess_action, _build_catalog
    monkeypatch.setenv('BIOXP_OEM_RUNTIME_ROOT',str(tmp_path))
    monkeypatch.setenv('BIOXP_OEM_RUNTIME_STATE_ROOT',str(tmp_path))
    monkeypatch.setattr(audit,'CANONICAL_RUNTIME_ROOT',tmp_path)
    OEMRuntimeStore(tmp_path).close()
    refs=ReferenceStateStore(tmp_path/'bioxp_runtime.db')
    class Hardware: ownership_epoch=7
    hardware=Hardware()
    wire.motor_oem_verify_motion_interlock=lambda:{'ok':True}
    if case=='source_noop': wire.initialized=False
    if case=='bad_sensor': wire.sensor_ack=7
    if case=='generation_changed':
        original=wire.motor_oem_home_axis_board_test
        def cross(*a,**kw):
            result=original(*a,**kw)
            hardware.ownership_epoch+=1
            return result
        wire.motor_oem_home_axis_board_test=cross
    app=FastAPI()
    async def blocking(label,fn,**kw): return fn()
    ns={'app':app,'Any':Any,'Literal':Literal,'BaseModel':BaseModel,'ConfigDict':ConfigDict,
        'HTTPException':HTTPException,'_get_tester':lambda:wire,'_require_motion_route_ready':lambda:None,
        '_require_oem_no_motion_profile_or_409':lambda *a:None,'_run_blocking':blocking,
        '_reference_state_store':refs,'MarkAxisReferencedCommand':MarkAxisReferencedCommand,'hardware_state':hardware}
    tree=ast.parse((ROOT/'src/bioxp/api.py').read_text())
    names={'OemManualHomeRequest','motion_oem_manual_home','motion_thermal_door_open','_thermal_door_success_or_409'}
    nodes=[n for n in tree.body if isinstance(n,(ast.ClassDef,ast.FunctionDef,ast.AsyncFunctionDef)) and n.name in names]
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(ROOT/'src/bioxp/api.py'),'exec'),ns)
    if case=='bad_sensor':
        with pytest.raises(RuntimeError, match='Failed to find door home'):
            asyncio.run(ns['motion_oem_manual_home'](ns['OemManualHomeRequest'](axis='door')))
        result=None
    else:
        result=asyncio.run(ns['motion_oem_manual_home'](ns['OemManualHomeRequest'](axis='door')))
        assert result['ok'] is True,result
        assert result['physical_effect_verified'] is False
    # Re-open the real SQLite reference owner, then use actual admission.
    saved=ReferenceStateStore(tmp_path/'bioxp_runtime.db').snapshot(['door'])
    fresh={'state':'fresh','age_s':0,'fresh_for_s':30}
    observations={'power':{'safety_valid':True},'latch':{'door_sensor':1,'latch_sensor':1},'interlock':{'motion_arm':{'armed':True}}}
    state={'ownership':{'transport':'owned','usb':'service','router':'running','CAN_READY':True},
        'maintenance':{'motion_blocked':False,'recovery_required':False},'lifecycle':{'operation_state':'stopped'},
        'snapshot_id':'offline','references':saved,'domains':{n:{'status':'observed','freshness':fresh,'observation':observations.get(n,{})} for n in ['axes','power','latch','interlock']}}
    actions,_=_build_catalog(app)
    action=next(a for a in actions if a['informational_path']=='/motion/thermal_door/open')
    admission=_assess_action(action,state,{})
    assert admission['enabled'] is (case=='proved'),(saved,admission)
    if case=='proved':
        assert result['home']['controller_home_proof_verified'] is True
        assert result['reference_position']['position_reply_valid'] is True
        assert result['reference_state']['durable_clean'] is True
    else:
        assert admission['disabled_reason']=='DOOR axis is not homed.'
