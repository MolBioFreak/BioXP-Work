"""Lifecycle pipette portions, not full lifecycle or hardware qualification."""
import pytest
from bioxp.services.pipette_service import build_oem_pipette_handlers, build_oem_pipette_lifecycle_helpers
from tests.test_protocol_oem_pipette import action
from tests.test_protocol_oem_pipette_composites import composite_bindings
from tests.test_protocol_oem_air_native import collection


def helpers(args):
    return build_oem_pipette_lifecycle_helpers(
        before_native_entry=args['before_native_entry'],pipette_call=args['pipette_call'],
        source_bindings=args['source_bindings'],settings=args['settings'],
        move_to_waste=args['move_to_waste'],sweep_handler=lambda a,s,*,clearall=False:{'ok':True,'source_occurrence_id':a.source_occurrence_id,'clearall':clearall})


def test_epilogue_sweep_explicit_clearall_and_original_identity():
    args, state, *_ = composite_bindings()
    result = helpers(args)['epilogue_sweep'](state, source_occurrence_id='lifecycle:sweep:7')
    assert result['clearall'] is True
    assert result['source_occurrence_id'] == 'lifecycle:sweep:7'
    assert result['source_return'] is None


def test_three_prologue_baselines_require_three_explicit_source_substep_keys():
    args,state,n,entries,fx,_=composite_bindings()
    baseline=helpers(args)['pressure_baseline']
    for index in range(3):
        baseline(state,source_occurrence_id=f'lifecycle:script_prologue:pressure:{index}')
    assert n.calls==[('pressure',)]*3
    assert len(entries)==len(set(entries))==3
    assert len([row for row in state.source_model.calls if row[0]=='baseline'])==3
    with pytest.raises(TypeError): baseline(state)


@pytest.mark.parametrize('remain',[False,True])
def test_run_job_query_nonscript_eject_requery_and_hold(remain):
    args,state,n,entries,fx,_=composite_bindings()
    n.tips=True
    original=n.eject_all_tips
    def eject(**kw):
        result=original(**kw); n.tips=remain; return result
    n.eject_all_tips=eject
    result=helpers(args)['run_job_tip_prefix'](state,source_occurrence_id='lifecycle:run_job:tips')
    assert [row[0] for row in n.calls]==['tips','eject','tips']
    assert n.calls[1][1]=={'check_missing_tip':False,'wait':True,'channels':[2]}
    assert ('waste',) in fx
    assert ('sleep',.5) in fx and ('sleep',.1) in fx
    if remain:
        assert result['source_error_hold'] and result['source_pause_scripts']
        assert state.source_model.logical_tip_present is True
    else:
        assert state.source_model.logical_tip_present is False


def test_real_I_native_false_wait_is_retained_through_da_service(monkeypatch):
    args,state,_,entries,fx,_=composite_bindings()
    group,drivers,trace=collection()
    drivers[1].wait_ok=False
    monkeypatch.setattr(group,'set_top_speed',lambda value: trace.append(('source_speed',value)) or {'ok':True})
    args['settings']['LogPressure']=True
    args['pipette_call']=lambda name,operation,a,s,key:operation(group)
    result=build_oem_pipette_handlers(**args)['da'](action('da',['5.9']),state)
    native=result['native_results'][-1]['result']
    assert trace[0]==('source_speed',30.0)
    assert native['source_return']==0 and native['source_wait_return'] is False
    assert native['completion_verified'] is False
    assert result['ok'] and not result['physical_effect_verified']
    assert len([row for row in trace if row[0]=='dispense'])==4
    assert all(row[2]==5.0 for row in trace if row[0]=='dispense')
    assert len(entries)==len(set(entries))
