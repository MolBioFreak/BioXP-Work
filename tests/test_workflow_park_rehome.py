"""Workflow Park through real compiler/provider/adapter; only wire is offline."""
from contextlib import nullcontext
from types import SimpleNamespace
import pytest
from tests.test_debloat_core import named
from tests.test_wake_setup_debloat import rig
from bioxp.oem_deck_movement import execute_finite_plate_operation, compile_finite_plate_operation


def workflow(named, monkeypatch, params):
    from bioxp import api
    _, provider, store, _, frames, fault = named
    from bioxp.oem_deck_movement import make_wp8_operation_executor
    plans=[]
    driver=provider.primitives.tester
    wire=driver.send_tmcl
    searching=set()
    def exchange(b,cmd,typ,motor,value,**kw):
        row=wire(b,cmd,typ,motor,value,**kw)
        if cmd==2 and value==200 and motor==0: searching.add(b)
        if cmd==6 and typ==1 and motor==0 and b in searching:
            row["value"]=fault.get("home_offsets",{}).get(b,0)
        if cmd==5 and typ==1 and motor==0: searching.discard(b)
        return row
    monkeypatch.setattr(driver,"send_tmcl",exchange)
    monkeypatch.setattr(driver,"send_tmcl_retry",exchange)
    store.bind_workflow_dispatcher(lambda command:None)
    store.admit_workflow(command_id="offline",idempotency_key="offline",plan_fingerprint="park",
        requested_inputs={"bundle":{"execution":{"runtime_state":{}}}},ownership_generation=7,
        resources=("axis:x","axis:y","axis:z","gripper"),board_epochs={})
    assert store.claim_next()["command_id"]=="offline"
    assert store._renew_owner(lease_seconds=600.0)
    executor=make_wp8_operation_executor(provider_getter=lambda:provider,command_store=store)
    stamps=provider.deck_owner_authority_stamps()
    admission={"ownership_generation":7,"serial206_initialization_provider":{
        "x_authority":{"current_board_lifecycle_generation":stamps["board_epoch_5"]},
        "board4_authority":{"active_board_epoch":stamps["board_epoch_4"]}}}
    def execute(plan, action, state):
        plans.append(plan)
        admitted=store.admit_internal_wp8_operation(plan["operation"],inputs={},state=admission,
            idempotency_key="park-child",prepared_plan=plan)
        claimed=store.claim_next()
        assert claimed["command_id"]==admitted["command_id"]
        return executor(command_id=admitted["command_id"],plan=plan)
    monkeypatch.setattr(api, "_require_motion_route_ready", lambda: None)
    monkeypatch.setattr(api, "_protocol_command_store", lambda:store)
    monkeypatch.setattr(api.app.state, "oem_workflow_plan_executor", execute, raising=False)
    frames.clear()
    result=api._protocol_live_park_handler(SimpleNamespace(params=params, action_id="park"), SimpleNamespace(job_id="offline"))
    return result, plans


def test_workflow_omitted_uses_oem_rehome(named, monkeypatch):
    result, plans=workflow(named, monkeypatch, {})
    assert plans[0]["children"][0]["arguments"]["rehome"] is True
    assert result["ok"] is True


def test_workflow_contract_reports_oem_default():
    from bioxp.protocols.method_contract import method_contract
    assert method_contract()["bindings"]["park"]["omitted_rehome"] is True


@pytest.mark.parametrize("params", [{}, {"rehome": True}])
def test_workflow_real_homexy_stop_capture_zero_and_final_park(named, monkeypatch, params):
    result, plans=workflow(named, monkeypatch, params)
    _, provider, store, _, frames, _=named
    assert result["ok"] is True
    for board in (5,4):
        homes=[i for i,r in enumerate(frames) if r==(board,2,0,0,200)]
        zeros=[i for i,r in enumerate(frames) if r==(board,5,1,0,0)]
        assert len(homes)==len(zeros)==1
        assert homes[0]<zeros[0]
        assert any(r[0]==board and r[1]==3 and r[3]==0 for r in frames[homes[0]:zeros[0]])
        assert (board,6,1,0,0) in frames[homes[0]:zeros[0]]
    assert store.deck_semantic_state()["current_location"]=="LOC_PARK"


def test_explicit_false_skips_home_and_shared_default_stays_false(named, monkeypatch):
    result, plans=workflow(named, monkeypatch, {"rehome": False})
    assert result["ok"] is True
    assert plans[0]["children"][0]["arguments"]=={"rehome":False}
    assert compile_finite_plate_operation("park_gantry",source_leaf_available=True)["children"][0]["arguments"]=={"rehome":False}
    assert not any(r[1]==2 or (r[1]==5 and r[2]==1) for r in named[4])
    assert named[2].deck_semantic_state()["current_location"]=="LOC_PARK"


@pytest.mark.parametrize("params", [{}, {"rehome":True}, {"rehome":False}])
def test_already_park_preserves_early_return(named, monkeypatch, params):
    _,p,s,_,frames,_=named
    s.publish_deck_owner_state(source_operation="updateLocation",source_command_id="already-park",
        updates={"current_location":"LOC_PARK","current_well":0},**p.deck_owner_authority_stamps())
    p.bind_pipette_collection_state_reader(lambda:pytest.fail("no tip query at Park"))
    result,_=workflow(named,monkeypatch,params)
    assert result["ok"] is True
    assert frames==[]


@pytest.mark.parametrize("board,offset", [(5,100),(5,101),(5,-101),(4,101),(4,-101)])
def test_actual_home_offsets_pause_strictly_above_100_without_retry(named, monkeypatch, board, offset):
    from bioxp.oem_deck_movement import DeckExecutionFailure
    _,p,s,_,frames,_=named
    named[5]["home_offsets"]={board:offset}
    before=s.deck_semantic_state()["current_location"]
    if abs(offset)>100:
        with pytest.raises(DeckExecutionFailure) as error:
            workflow(named,monkeypatch,{})
        paused=error.value.provider_results[0]
        assert paused["source_pause_scripts"] is True
        assert paused["semantic_location_commit_allowed"] is False
        assert "exceeded 100" in paused["source_error_event"]
        ops=[r["operation"] for r in paused["source_children"]]
        assert ops==["scriptmoveTo(current,well,28,0,0)","HomeXY()"]
        # Provider has already homed/reset both axes before the finite child fails.
        assert s.deck_semantic_state()["current_location"]==before
    else:
        result,_=workflow(named,monkeypatch,{})
        assert result["ok"] is True
        assert s.deck_semantic_state()["current_location"]=="LOC_PARK"
    for b in (4,5):
        assert frames.count((b,2,0,0,200))==1
        assert frames.count((b,5,1,0,0))==1


@pytest.mark.parametrize("stuck", [False,True])
def test_workflow_keeps_tip_cleanup_and_manual_removal_pause(named,monkeypatch,stuck):
    from bioxp.oem_deck_movement import DeckExecutionFailure
    _,p,s,_,frames,_=named
    tips={"tip_exists":True}
    p.bind_pipette_collection_state_reader(lambda:dict(tips))
    calls=[]
    def eject():
        calls.append("eject")
        return {"ok":True}
    def query(**kw):
        calls.append("query")
        tips["tip_exists"]=stuck
        return {"ok":True}
    monkeypatch.setattr(p.primitives,"eject_all_tips_for_oem_park",eject)
    monkeypatch.setattr(p.primitives,"query_all_pipette_tip_states",query)
    if stuck:
        with pytest.raises(DeckExecutionFailure) as error:
            workflow(named,monkeypatch,{})
        result=error.value.provider_results[0]
        assert result["governance_outcome"]=="manual_tip_removal_required"
        assert result["source_pause_scripts"] is True
        assert not any(r[1]==2 and r[4]==200 for r in frames)
        assert s.deck_semantic_state()["current_location"]!="LOC_PARK"
    else:
        result,_=workflow(named,monkeypatch,{})
        assert result["ok"] is True
        assert s.deck_semantic_state()["current_location"]=="LOC_PARK"
        assert (4,4,0,1,80000) in frames
        assert (5,4,0,0,79000) in frames
    assert calls==["eject","query"]
