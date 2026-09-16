"""Finite source composites with explicit native/model doubles and real claims.

No hardware or CV success is asserted. The SQL case tests real claim identity,
not merely a synthetic duplicate-key dictionary.
"""
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import fields, replace
from types import SimpleNamespace
import threading
import pytest

from bioxp.services.pipette_service import OemPipetteSourceBindings, build_oem_pipette_handlers
from tests.test_protocol_oem_pipette import Native, Model, action, bindings


class FluidModel(Model):
    def __init__(self, volume=60):
        super().__init__()
        self.volume = volume
        self.tip_zone_index = None
        self.old_tip_well = None
        self.allow_to_stop = True
    def current_well_volume(self, loc, well, *, current_tray): return self.volume
    def update_fluid_name(self, loc, well, *, current_tray): self.calls.append(("name", loc, well, current_tray))
    def update_fluid_level(self, loc, well, tip, delta, *, current_tray):
        self.calls.append(("fluid", loc, well, tip, delta)); self.volume += delta
    def select_tip(self, kind, pipette): return (0, 7, "A1")
    def tip_removed(self, *args, **kwargs): self.calls.append(("removed", args, kwargs))
    def tip_restored(self, *args): self.calls.append(("restored", args))
    def next_hotel_tip(self): return "H1"
    def tip_hotel_empty(self): return False
    def hotel_loaded(self): self.calls.append(("hotel_loaded",))


class FluidNative(Native):
    def __init__(self):
        super().__init__()
        self.tip_count = 4
    def aspirate(self, command):
        self.calls.append(("asp", command.volume_ul)); return {"ok": True}
    def dispense(self, command):
        self.calls.append(("dsp", command.volume_ul)); return {"ok": True}
    def dispense_all(self):
        self.calls.append(("dspall",)); return {"ok": True}
    def aspirate_for_oem_script(self, volume, *, pressure_stream):
        self.calls.append(("asp_stream", volume, pressure_stream)); return {"ok": True}
    def dispense_for_oem_script(self, volume, *, pressure_stream):
        self.calls.append(("dsp_stream", volume, pressure_stream)); return {"ok": True}
    def dispense_air_for_oem_script(self, volume, front_air, pressure_stream=False):
        self.calls.append(("da", volume, front_air, pressure_stream))
        return {"ok": True, "source_return": 0, "completion_verified": False}
    def query_tip_status_all(self):
        self.calls.append(("tips",)); return {"ok": True, "source_return": self.tip_count, "source_tip_exists": self.tips}
    def query_tip_status_for_oem_script(self, pipette):
        self.calls.append(("single_tip", pipette)); return {"ok": True, "source_return": 1}
    def loadTip(self, tip_type, tip_location=-1):
        self.calls.append(("load", tip_type, tip_location)); return {"ok": True}
    def KeepTip(self, pipette):
        self.calls.append(("keep", pipette)); return {"ok": True}


def composite_bindings(volume=60):
    transport = FluidNative()
    args, state, _, entries, effects = bindings(transport)
    state.source_model = FluidModel(volume)
    facts = dict(current_location=0, current_well=0, current_tray=0,
                 tip_type=200, tip_location=-1, fluid_level=20.0, speed=30.0,
                 current_location_name="LOC_MS", z_high=80000)
    def record(name):
        def callback(*values):
            effects.append((name, *values[:-2]))
            return {"ok": True, "source_return": 0, "x": 100, "y": 200, "z": 300}
        return callback
    callbacks = {f.name: record(f.name) for f in fields(OemPipetteSourceBindings)}
    callbacks.update(facts=lambda a,s: dict(facts),
                     is_pierced=lambda *a: False,
                     z_low=lambda *a: 70000,
                     tip_exists=lambda *a: transport.tips,
                     stopped=lambda *a: False,
                     led2_on=None, check_tips=None, take_aspirate_image=None,
                     source_error_event=lambda msg,a,s: effects.append(("error", msg)),
                     source_capabilities=frozenset({"aspirate_pressure_stream", "dispense_pressure_stream", "query_tip_status_single"}),
                     sleep=lambda seconds: effects.append(("sleep", seconds)))
    def publication(changes, a, s):
        facts.update(changes); return record("tip_state")(changes,a,s)
    def location(loc, well, a, s):
        facts.update(current_location=loc, current_well=well,
                     current_location_name={6:"WASTE_BIN",7:"TECANRACK1",15:"LOC_TIP_HOTEL"}.get(loc,"LOC_MS"))
        return record("publish_location")(loc,well,a,s)
    callbacks.update(tip_state=publication, publish_location=location)
    args["source_bindings"] = OemPipetteSourceBindings(**callbacks)
    args["settings"] = dict(LogPressure=False, StartMode=1, CameraInstalled=False,
                            CameraCalibrated=False, OverPressChecked=False, CheckSnapTips=False)
    return args, state, transport, entries, effects, facts


def mix_options(**extra):
    return {"m_aspirateOptions": {"m_volume": 20, "m_air": 0, "m_delay": 0},
            "m_dispenseAllOptions": {"m_delay": 0}, "m_repeat": 2, **extra}


def test_all_eight_composites_are_finite_non_alias_handlers():
    args, *_ = composite_bindings()
    result = build_oem_pipette_handlers(**args)
    assert {"da","masp","dsa","mmix","rmb","ampmix","ldtip","ejt"} <= set(result)
    assert "arbitrary_transport_method" not in result


@pytest.mark.parametrize("stream", [False,True])
def test_da_exact_I_adapter_volume_truncation_and_pressure_flag(stream):
    args,state,n,entries,fx,_ = composite_bindings()
    args["settings"]["LogPressure"] = stream
    result = build_oem_pipette_handlers(**args)["da"](action("da",["12.9"]),state)
    assert n.calls[:2] == [("speed",30.0),("da",12.0,True,stream)]
    assert ("pressure",) in n.calls if not stream else ("pressure",) not in n.calls
    assert result["physical_effect_verified"] is False
    assert result["native_results"][2]["result"]["completion_verified"] is False
    assert len(entries) == len(set(entries))


@pytest.mark.parametrize("opcode,options,expected", [
    ("masp", {"m_volume":140,"m_overaspirate":2,"m_air":3.9,"m_delay":0}, [71,71]),
    ("dsa", {"m_purge":False,"m_delay":0}, [70,70]),
])
def test_split_source_transfers_preserve_arguments_and_model_order(opcode,options,expected):
    args,state,n,entries,fx,facts = composite_bindings(200)
    facts["fluid_level"] = 140
    result=build_oem_pipette_handlers(**args)[opcode](action(opcode,options),state)
    kind="asp" if opcode=="masp" else "dsp"
    assert [row[1] for row in n.calls if row[0]==kind] == expected
    deltas=[row[-1] for row in state.source_model.calls if row[0]=="fluid"]
    assert deltas == [-71,-71] if opcode=="masp" else deltas == [70,70]
    assert result["source_return"] == [37]
    assert len(entries) == len(set(entries))
    if opcode=="masp": assert ("air",3,[2],False) in n.calls


@pytest.mark.parametrize("mix_type,transfers,orbit", [("N",4,0),("H",4,0),("C",24,12)])
def test_mmix_repeated_source_steps_have_unique_identities(mix_type,transfers,orbit):
    args,state,n,entries,fx,_ = composite_bindings()
    result=build_oem_pipette_handlers(**args)["mmix"](action("mmix",mix_options(m_mixType=mix_type)),state)
    assert len([r for r in n.calls if r[0] in ("asp","dsp")]) == transfers
    assert len([r for r in fx if r[0]=="move_xy"]) == orbit
    assert len(entries)==len(set(entries))==len(result["native_results"])
    assert all(row["step_id"].startswith("oem:7:") for row in result["native_results"])


def test_rmb_orbit_portions_and_ampmix_source_constants(monkeypatch):
    args,state,n,entries,fx,_ = composite_bindings(40)
    opts={"m_aspirateOptions":{"m_volume":12,"m_air":4,"m_delay":0},
          "m_dispenseOptions":{"m_volume":5,"m_delay":0},"m_repeat":1,"m_orbit":True}
    build_oem_pipette_handlers(**args)["rmb"](action("rmb",opts),state)
    assert [r[1] for r in n.calls if r[0]=="dsp"] == [5,5,2]
    assert len([r for r in fx if r[0]=="move_xy"]) == 18
    monkeypatch.setattr("bioxp.services.pipette_service.random.Random",lambda:SimpleNamespace(random=lambda:0.25))
    args,state,n,entries,fx,_=composite_bindings(60)
    build_oem_pipette_handlers(**args)["ampmix"](action("ampmix",{"m_repeat":2}),state)
    assert [r[1] for r in n.calls if r[0]=="asp"] == [10,30,30,30,30]
    assert [r[1] for r in n.calls if r[0]=="dsp"] == [30,30,30,30,10]
    assert len(entries)==len(set(entries))
    assert ("sleep",2.0) in fx


def test_selected_missing_stream_or_cv_fails_before_native_prefix():
    args,state,n,entries,fx,_=composite_bindings()
    args["settings"]["LogPressure"]=True
    args["source_bindings"]=replace(args["source_bindings"],source_capabilities=frozenset())
    handler=build_oem_pipette_handlers(**args)["masp"]
    with pytest.raises(ValueError,match="pressure stream"): handler(action("masp",{}),state)
    assert n.calls==[] and entries==[]
    args["settings"].update(CameraInstalled=True,CameraCalibrated=True)
    handler=build_oem_pipette_handlers(**args)["ldtip"]
    with pytest.raises(ValueError,match="CV"): handler(action("ldtip",["T50"]),state)
    assert n.calls==[] and entries==[]


@pytest.mark.parametrize("pipette",[-1,2])
def test_ldtip_real_source_pressload_and_selected_model_publication(pipette):
    args,state,n,entries,fx,facts=composite_bindings()
    facts["tip_type"]=201
    result=build_oem_pipette_handlers(**args)["ldtip"](action("ldtip",["T50","T",str(pipette)]),state)
    assert result["ok"] and ("load",50,pipette) in n.calls
    assert ("keep",pipette) in n.calls if pipette!=-1 else not any(r[0]=="keep" for r in n.calls)
    transitions=[r for r in fx if r[0]=="tip_transition"]
    assert transitions[-1][2]==([48] if pipette==2 else [0,24,48,72])
    assert state.source_model.old_tip_well=="A1"
    assert state.source_model.logical_tip_present is True
    assert state.source_model.allow_to_stop is False
    assert len(entries)==len(set(entries))


def test_ejt_joins_both_source_children_before_result():
    args,state,n,entries,fx,facts=composite_bindings()
    facts.update(current_location=6,current_location_name="WASTE_BIN")
    entered,release=threading.Event(),threading.Event()
    original=n.eject_all_tips
    def eject(**kwargs):
        entered.set(); assert release.wait(3); return original(**kwargs)
    n.eject_all_tips=eject
    with ThreadPoolExecutor(max_workers=3) as owner:
        # This is an explicit TEST owner, not a production D scheduler.
        args["source_bindings"]=replace(args["source_bindings"],start_child=lambda name,call,a,s:owner.submit(call))
        # Fixture guard's last-entry assertion is intentionally thread-local here.
        args["pipette_call"]=lambda name,call,a,s,key:call(n)
        future=owner.submit(build_oem_pipette_handlers(**args)["ejt"],action("ejt",[]),state)
        assert entered.wait(2)
        assert not future.done()
        release.set(); result=future.result(timeout=3)
    assert result["ok"] and state.source_model.logical_tip_present is False
    assert ("move_z",65000) in fx and ("move_x",79000) in fx
    assert len(entries)==len(set(entries))


def test_ldtip_hotel_retains_zone_and_uses_single_hotel_well():
    args,state,n,entries,fx,facts=composite_bindings()
    facts["tip_type"]=201
    state.source_model.tip_zone_index=7
    build_oem_pipette_handlers(**args)["ldtip"](action("ldtip",["T50","T","-1","H"]),state)
    assert ("single_tip",0) in n.calls and ("keep",0) in n.calls
    assert ("load",50,0) in n.calls
    assert [r for r in fx if r[0]=="tip_transition"][-1][1:4]==(4,[84],"remove")
    assert state.source_model.tip_zone_index==7
    assert state.source_model.old_tip_well is None


def test_ldtip_same_type_source_early_return_does_not_invent_cleanup():
    args,state,n,entries,fx,facts=composite_bindings()
    facts["tip_type"]=50
    build_oem_pipette_handlers(**args)["ldtip"](action("ldtip",["T50"]),state)
    assert [r[0] for r in n.calls]==["pressure","tips"]
    assert ("stall_guard",10) in fx
    assert not any(r[0] in {"set_z_current_max","tip_transition"} for r in fx)


def test_ldtip_source_exhaustion_keeps_completed_prefix():
    args,state,n,entries,fx,facts=composite_bindings()
    facts["tip_type"]=201; n.tip_count=0
    selected=iter([(0,7,"A1"),None])
    state.source_model.select_tip=lambda *a:next(selected)
    with pytest.raises(RuntimeError,match="Tips are not available") as failed:
        build_oem_pipette_handlers(**args)["ldtip"](action("ldtip",["T50"]),state)
    assert failed.value.oem_partial_results
    assert any(r[0]=="tip_transition" for r in fx)
    assert ("error","Tips are not available") in fx


@pytest.mark.parametrize("single",[-1,2])
def test_ejt_restores_source_group_or_single_labels(single):
    args,state,n,entries,fx,facts=composite_bindings()
    facts.update(current_location=7,current_location_name="TECANRACK1",tip_location=single)
    state.source_model.old_tip_well="A1"; state.source_model.tip_zone_index=9
    def owned(name,call,a,s):
        future=Future()
        try: future.set_result(call())
        except Exception as exc: future.set_exception(exc)
        return future
    args["source_bindings"]=replace(args["source_bindings"],start_child=owned)
    build_oem_pipette_handlers(**args)["ejt"](action("ejt",["rm"]),state)
    transition=[r for r in fx if r[0]=="tip_transition"][0]
    assert transition[1:]==(0,[0,24,48,72] if single==-1 else [48],"restore","rm",9)
    assert ("restored",(0,"A1" if single==-1 else 48,"rm",9)) in state.source_model.calls


def test_repeated_composite_real_sql_claims_do_not_reconcile_away_native_calls(tmp_path):
    from bioxp.oem_runtime_store import OEMRuntimeStore
    from bioxp.operator_command_plane import OperatorCommandStore
    from bioxp.runtime_audit_store import RuntimeAuditDatabase
    OEMRuntimeStore(tmp_path).close()
    store=OperatorCommandStore(tmp_path); store.bind_workflow_dispatcher(lambda c:None)
    store.admit_workflow(command_id="parent",idempotency_key="parent-key",plan_fingerprint="plan",
        requested_inputs={"bundle":{"execution":{"runtime_state":{}}}},ownership_generation=1,
        resources=("pipette",),board_epochs={})
    store.claim_next(); db=RuntimeAuditDatabase(tmp_path)
    args,state,n,entries,fx,_=composite_bindings()
    from bioxp.protocols.runtime_state import ProtocolSourceModel, SourceTray, SourceWell
    state.source_model = ProtocolSourceModel(trays={"POOL_PLATE": SourceTray(
        "pool", 0, [SourceWell("sample", 60, 300) for _ in range(96)])})
    actual=[]
    def pipette(name,operation,a,s,key):
        with store.workflow_context("parent",source_occurrence_id=a.source_occurrence_id):
            child,created=db.claim(dict(command_id=key,idempotency_key=key,action_id="pipette."+name,
                operation=name,entrypoint_id="test",caller_class="protocol",control_class="physical_liquid_command",
                ownership_generation=1,requested_inputs={"source":a.source_occurrence_id,"step":key}),pipette=True)
            assert created, "a later intended transfer was incorrectly reconciled as a prior child"
            native=operation(n); actual.append((key,name,child["parent_command_id"]))
            db.finalize_claim(command_id=child["command_id"],pipette_operation_id=child["pipette_operation_id"],
                expected_status=child["status"],status="completed",outcome="completed",failure_code=None,result=native)
            return native
    args["pipette_call"]=pipette
    try:
        result=build_oem_pipette_handlers(**args)["mmix"](action("mmix",mix_options(m_mixType="C")),state)
        assert len([r for r in n.calls if r[0] in ("asp","dsp")])==24
        assert len(actual)==len(set(r[0] for r in actual))
        assert all(r[2]=="parent" for r in actual)
        assert db.connection.execute("SELECT COUNT(*) FROM pipette_operations").fetchone()[0]==len(actual)
        done=store.finish_workflow("parent",status="completed",payload={"source":result["source_occurrence_id"]},lifecycle_settled=True)
        assert done["command"]["status"]=="completed"
    finally:
        store.stop()
