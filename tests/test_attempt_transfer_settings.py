import pytest
from bioxp.protocols.models import ProtocolAction, ProtocolActionKind as K
from bioxp.protocols.recovery import adjusted, supported
from bioxp.pipette.cavro_liquid import compile_liquid_recipe


def pos(op, loc):
    return {'operation':'position','positioning':{'operation':op,'location_id':loc}}


def action(multi=False, detect=False):
    seg = [{'volume_ul':1,'speed_ul_s':1}]
    m = dict(sample_count=2,sample_volume_ul=1,conditioning_volume_ul=1,conditioning_back_to_source_count=1,
             conditioning_speed_ul_s=1,conditioning_before=[pos('lower',16)],conditioning_after=[],excess_volume_ul=0,
             excess=None,reaspiration=None,dispense_to_reaspiration_delay_ms=None,
             aliquots=[dict(before=[pos('lower',12)],segments=seg,after=[]) for _ in range(2)]) if multi else None
    recipe=dict(mode='multi' if m else 'single',multi=m,channels=[1,3],timeout_ms=1000,target_liquid_ul=1,
        commanded_aspiration_ul=1,aspiration_speed_ul_s=1,aspiration_delay_ms=10,leading_air=None,trailing_air=None,
        before_leading_air=[],before_liquid=[] if detect else [pos('lower',16)],after_liquid=[],
        before_dispense=[] if m else [pos('lower',12)],after_dispense=[],dispense_segments=[] if m else seg,
        final_empty_before=[],final_empty_speed_ul_s=None)
    if detect: recipe['liquid_detection']={'location_id':16}
    return ProtocolAction('a','s',K.PIPETTE_MANUAL_PHYSICAL,{'operation':'cavro_liquid_recipe','recipe':recipe})


@pytest.mark.parametrize('multi',[False,True])
def test_positioned_recipe(multi):
    a=action(multi)
    assert supported(a)
    updated=adjusted(a,dict(source_offset_steps=-17,destination_offset_steps=23,dispense_delay_ms=31))
    compiled=compile_liquid_recipe(updated.to_payload()['params']['recipe'])
    ops=compiled['application']['operations']
    assert [op['positioning']['offset_steps'] for op in ops if op['operation']=='position']==([-17,-17,23,23] if multi else [-17,23])
    assert [op['duration_ms'] for op in ops if op['operation']=='delay']==([10,31,31] if multi else [10,31])
    assert 'offset_steps' not in a.params['recipe']['before_liquid'][0]['positioning']


def test_search_pressure_adjustment():
    a=action(detect=True)
    b=adjusted(a,{'plld_threshold_adc':87})
    op=compile_liquid_recipe(b.to_payload()['params']['recipe'])['operations'][0]
    assert op['operation']=='liquid_search' and op['pressure_settings']=={'plld_threshold_adc':87}
    for bad in [{'plld_threshold_adc':True},{'plld_threshold_adc':1001},{'on_contact_delay_ms':0}]:
        with pytest.raises(ValueError): adjusted(a,bad)


@pytest.mark.parametrize('form,count', [('single',1),('multi',2),('conditioned',1)])
def test_bms_emitted_forms(form,count,monkeypatch):
    # Frozen from BMS 6aa1a6a compile_method(test_bioxp_cavro_transfer.request(...)).
    import json,gzip
    from pathlib import Path
    from types import SimpleNamespace as NS
    import bioxp.oem_serial206_initialization as native
    from bioxp.pipette import depth_offsets
    from bioxp import plate_profiles
    monkeypatch.setattr(native,'load_bound_oem_position_table',lambda: NS(resolve=lambda **k:NS(z_low=4000,z_high=3000)))
    monkeypatch.setattr(depth_offsets,'resolve',lambda **kw:{'offset_steps':7})
    monkeypatch.setattr(plate_profiles,'plate_at',lambda s:None)
    provider=object.__new__(native.Serial206OemInitializationProvider)
    provider.primitives=NS(pipette_transport=NS(_tip_loaded=True,_tip_type=50))
    provider.moveZ=lambda z:{'ok':True,'target':z}
    with gzip.open(Path(__file__).parents[1]/'testdata/contracts/attempt_transfer_forms.json.gz','rt') as f:
        rows=json.load(f)[form]
    assert [a['operation'] for a in rows]==['cavro_liquid_recipe','lift']*2
    for params in rows[::2]:
        original=ProtocolAction('a','s',K.PIPETTE_MANUAL_PHYSICAL,params)
        a=adjusted(original,{'source_offset_steps':-11,'destination_offset_steps':47,'dispense_delay_ms':88})
        ops=compile_liquid_recipe(a.to_payload()['params']['recipe'])['operations']
        lower=[op['native_plan']['children'][0] for op in ops if op.get('positioning',{}).get('operation')=='lower']
        targets=[provider.wp8_pipette_source_leaf(child['operation'],child['arguments'],command_id='inert',child_order=i,plan_digest='d')['target'] for i,child in enumerate(lower)]
        assert targets==[3996]*(1 if form=='single' else 2)+[4054]*count
        assert sum(op.get('duration_ms')==88 for op in ops)==count
        assert original.to_payload()['params']==params


@pytest.mark.parametrize('failure', ['none','capture','apply','search','restore','search+restore','stop'])
def test_pressure_scope(failure,monkeypatch):
    from types import SimpleNamespace as NS
    from bioxp.pipette import cavro_application as mod
    from bioxp.pipette.cavro_receiving import decode_report
    from bioxp.oem_compat import position_table
    prior={1:21,3:92}; registers=dict(prior); sends=[]
    def query(cmd,channels,timeout_ms):
        assert cmd['ascii']=='?32' and channels==[1,3]
        return {'ok':True,'channels':[{'channel':ch,'result':{'semantic_query_response_verified':failure!='capture',
            'report':decode_report(cmd,bytes([32,96])+str(registers[ch]).encode())}} for ch in channels]}
    group=NS(_tip_type=50,_interrupt_epoch=0,_forceabort=lambda:False,run_cavro_command=query)
    def wire(t,channels,ascii,timeout,**kwargs):
        sends.append((channels,ascii))
        if ascii.startswith('p0,'):
            for ch in channels:
                if failure in ('restore','search+restore') and ascii=='p0,92R': raise RuntimeError('restore fault')
                registers[ch]=int(ascii[3:-1])
                if failure=='apply' and registers[ch]==87: raise RuntimeError('partial send')
        return {'ok':True,'channels':[{'channel':ch,'completion':{'ok':True}} for ch in channels]}
    monkeypatch.setattr(mod,'_send_wire',wire)
    monkeypatch.setattr(position_table,'load_bound_oem_position_table',lambda:NS(resolve=lambda **k:NS(z_low=6000,z_high=2000)))
    provider=NS(primitives=NS(pipette_transport=group,z_set_max_speed=lambda n:{'ok':True}),
                _wp8_execution_fence_checker=lambda *a,**k:None,_manual_pipette_receipt_runner=lambda n,f,*a:f(group))
    def move(z):
        assert registers=={1:87,3:87}
        if failure=='stop':group._interrupt_epoch+=1
        if failure in ('search','search+restore','stop'):raise RuntimeError('search fault')
        return {'ok':True}
    provider.moveZ=move
    req=compile_liquid_recipe(adjusted(action(detect=True),{'plld_threshold_adc':87}).to_payload()['params']['recipe'])['application']
    # Isolate the scientific search result from the scope's success case. Test the
    # same capture, setter and unwind owners on the full inline failure paths below.
    if failure in ('none','restore'):
        def record(name,fn,**kw):return fn(group)
        evidence=[]
        search=lambda apply:(apply(),move(0))
        if failure=='restore':
            with pytest.raises(mod.PipetteCommandError,match='restoration unresolved'):mod._pressure_scope(req['operations'][0],record,evidence,search)
        else:mod._pressure_scope(req['operations'][0],record,evidence,search)
    else:
        result=mod.run_application_inline(provider,req,command_id='inert',owner_identity={'source_identity':'attempt'})
        assert not result['ok']
        evidence=result.get('pressure_restoration',[])
        if failure=='capture':assert not sends
        if failure in ('search','search+restore','stop'):assert result['error']=='search fault'
        if failure=='stop':assert result['interrupted'] and all(r['status']=='unresolved' for r in evidence)
    if failure in ('restore','search+restore'):
        assert registers=={1:21,3:87} and any(r['status']=='unresolved' for r in evidence)
    elif failure!='stop':assert registers==prior


@pytest.mark.parametrize('field,selector,report,old,new', [
    ('plld_threshold_adc',0,32,15,55), ('plld_persistence_ms',1,33,5,8),
    ('plld_start_position_increments',2,34,0,1), ('plld_speed_increments_s',3,35,150,160),
    ('plld_travel_increments',4,36,44000,500), ('plld_direction',5,37,0,1),
    ('pressure_interval_ms',6,38,20,10), ('pressure_samples_per_can',7,27,1,3),
    ('hybrid_interval_ms',8,39,50,60)])
def test_supported_parameter_roundtrip(field,selector,report,old,new,monkeypatch):
    from types import SimpleNamespace as NS
    from bioxp.pipette import cavro_application as mod
    from bioxp.pipette.cavro_receiving import decode_report
    def query(cmd,channels,timeout_ms):
        assert cmd['ascii']==f'?{report}'
        return {'ok':True,'channels':[{'channel':ch,'result':{'semantic_query_response_verified':True,
            'report':decode_report(cmd,b' '+b'`'+str(old).encode())}} for ch in channels]}
    writes=[]
    def write(group,chs,wire,timeout):
        writes.append((chs,wire))
        return {'ok':True,'channels':[{'channel':ch,'completion':{'ok':True}} for ch in chs]}
    monkeypatch.setattr(mod,'_send_wire',write)
    a=adjusted(action(detect=True),{field:new})
    op=compile_liquid_recipe(a.to_payload()['params']['recipe'])['operations'][0]
    evidence=[]
    mod._pressure_scope(op,lambda n,f,**kw:f(NS(run_cavro_command=query)),evidence,lambda apply:apply())
    assert writes==[([1,3],f'p{selector},{new}R'),([3],f'p{selector},{old}R'),([1],f'p{selector},{old}R')]
