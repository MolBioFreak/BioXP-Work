"""Finite provider/compiler qualification; native IO explicitly doubled."""
from types import SimpleNamespace
from contextlib import nullcontext
import pytest

from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider as Provider
from bioxp.oem_deck_movement import (
    compile_finite_plate_operation, execute_finite_plate_operation,
    OEM_PIPETTE_LEAVES, WP8_COMPILED_CHILD_OPERATIONS,
)
from bioxp.pipette.transport import FourPipetteTransport, CanPipetteTransport
from bioxp.oem_compat.position_table import PositionTarget, PositionTable
from bioxp.oem_compat.pathing import LOCATION_ID_TO_NAME
from bioxp.protocols.runtime_state import ProtocolRuntimeState


def native(**extra):
    return dict(ok=True, controller_command_acknowledged=True,
                controller_completion_verified=True, **extra)


@pytest.fixture
def rig(monkeypatch):
    trace = []
    def record(name):
        def call(*args, **kwargs):
            trace.append((name, args, kwargs))
            return native()
        return call
    transport = FourPipetteTransport([CanPipetteTransport(driver_factory=lambda: pytest.fail("driver acquisition")) for _ in range(4)])
    tester = SimpleNamespace(
        motor_oem_move_z_home=lambda **kw: trace.append(("home", (), kw)) or native(home={"source_return_code": -7}),
        motor_oem_board_move_steps=record("relative"), motor_oem_home_axis_board_test=record("home_gripper"),
        deck_io_set_type=record("solenoid"), _motion_oem_axis_profile=lambda axis: {"stall_guard": 3},
    )
    primitives = SimpleNamespace(tester=tester, pipette_transport=transport,
        oem_move_xy=record("xy"), oem_move_to=record("move_to"), z_set_current_max=record("current"),
        z_pipette_position=record("pipette_z"), _z_set_profile_parameter=record("stall"),
        _read_axis_position=lambda axis: {"x": 11, "y": 22, "z": 33}[axis],
        x_move_absolute=record("x"))
    p = Provider(primitives, sleep=lambda sec: trace.append(("sleep", sec)))
    machine = dict(current_location=3, current_well=14, tip_location=-1, tip_loaded=False,
        tip_dirty=False, pseudo_z_home=65000, plate_on_gantry=None, well_pierced={})
    p.mov_execution_machine_state = lambda: dict(machine)
    # Scoped source moves read the canonical owner, not the full-state getter.
    p.bind_deck_semantic_state_reader(lambda: dict(machine,
        semantic_state_revision=0, ambiguity_state="none", transition_provenance={}))
    p._canonical_deck_semantic_state = lambda: {"current_tray": 2}
    p._deck_gripper_confirmed = lambda: False
    p._park_collection_state = lambda: {"tip_exists": True}
    p.moveZ = lambda value: trace.append(("z", value)) or native()
    p.scriptmoveTo = record("scriptmove")
    p.deck_owner_authority_stamps = lambda: dict(ownership_generation=1, board_epoch_4=1, board_epoch_5=1)
    table = PositionTable([PositionTarget(name, base_coordinates={"x": 100, "y": 200},
        z_low=90000, z_high=30000, inc_factor=1) for name in LOCATION_ID_TO_NAME.values()])
    monkeypatch.setattr("bioxp.oem_serial206_initialization.load_bound_oem_position_table", lambda: table)
    plans = []
    def execute(plan, action, state):
        plans.append(plan)
        def invoke(child):
            result = p.execute_wp8_child(child, command_id=f"child-{len(plans)}", child_order=child["order"], plan_digest=plan["plan_digest"])
            if result.get("ok") is not True:
                raise RuntimeError("native_failed")
            return result
        return execute_finite_plate_operation(plan, invoke)
    callbacks = p.build_oem_pipette_source_callbacks(execute_plan=execute,
        start_child=lambda *a: pytest.fail("unrequested owned task"), stopped=lambda: False,
        rgb_writer=record("rgb"), settings={"LogPressure": False})
    return p, callbacks, trace, plans, transport, machine


def test_builder_is_pure_and_roster_is_concrete(rig):
    p, c, trace, plans, _, _ = rig
    assert trace == plans == []
    assert set(c) == {"source_bindings", "settings", "script_move", "publish_location", "publish_tip_transition",
        "lift_for_air", "move_to_waste", "move_z", "move_x", "tip_load_move", "move_z_home",
        "set_z_current_max", "remove_tip", "script_move_to_waste", "source_error_event"}
    assert {leaf for leaf, _ in OEM_PIPETTE_LEAVES.values()} <= set(p.wp8_child_binding_inventory())
    assert set(p.wp8_child_binding_inventory()) == WP8_COMPILED_CHILD_OPERATIONS
    assert c["source_bindings"].check_tips is None  # CV is not a camera alias
    for name in OEM_PIPETTE_LEAVES:
        assert p.wp8_operation_machine_state(name, {}) == {}
    assert trace == plans == []


def test_facts_read_real_cached_transport_without_hardware(rig):
    _, c, trace, _, t, _ = rig
    t._transports[0]._liquid_level_ul = 19.25
    t._transports[0]._top_speed = 42.0
    t._tip_type = 50
    f = c["source_bindings"].facts(None, None)
    assert (f["fluid_level"], f["speed"], f["tip_type"], f["current_tray"]) == (19.25, 42.0, 50, 2)
    assert c["source_bindings"].tip_exists(None, None) is True
    assert trace == []


@pytest.mark.parametrize("location,height", [(16,64503),(3,54425),(11,38299),(12,38299),(13,38299),(14,38299),(6,28220)])
def test_lift_air_literal_offsets(rig, location, height):
    _, c, trace, plans, _, machine = rig
    machine["current_location"] = location
    assert c["lift_for_air"](None,None)["ok"]
    assert trace == [("z", 90000-height)]
    assert plans[0]["operation"] == "pipette_lift"


def test_lift_default_lower_and_position_returns(rig):
    _, c, trace, _, _, _ = rig
    n = c["source_bindings"]
    assert n.lift_to(3,None,None,None)["source_return"] == 0
    assert n.lower_to(3,None,None)["source_return"] == 90000
    assert trace == [("z",30000),("z",90000)]
    assert {k:n.position(None,None)[k] for k in ("x","y","z")} == dict(x=11,y=22,z=33)


def test_source_home_default_and_false_preserve_native_return(rig):
    _, c, trace, _, _, _ = rig
    n=c["source_bindings"]
    for value in (None,False,True):
        assert n.home(value,None,None)["source_return"] == -7
    assert [r[2]["rehome"] for r in trace] == [True,False,True]


def test_shift_camera_order_and_partial_failure(rig):
    p,c,trace,plans,_,_=rig
    c["source_bindings"].shift_camera(None,None)
    assert [r[0] for r in trace] == ["home","relative","relative"]
    assert trace[0][2]["rehome"] is False
    assert [r[1][:2] for r in trace[1:]] == [(5,2832),(4,-1888)]
    trace.clear()
    p.primitives.tester.motor_oem_move_z_home = lambda **kw: {"ok":False}
    with pytest.raises(RuntimeError,match="native_failed"):
        c["source_bindings"].shift_camera(None,None)
    assert trace == []


def test_script_and_nonscript_waste_and_sweep_are_distinct(rig):
    _,c,trace,plans,_,_=rig
    c["script_move"](7,1,None,None)
    c["tip_load_move"](7,2,1,None,None)
    c["move_to_waste"](None,None)
    c["source_bindings"].hotel_move(None,None)
    assert trace[0][2] == dict(destination=7,column=0,row=1,position_flag=0,run_in_parallel=True)
    assert trace[1][2] == dict(destination=7,column=2,row=1,position_flag=2,run_in_parallel=False)
    assert [r[0] for r in trace[2:]] == ["move_to","move_to"]
    assert all(not r[2]["run_in_parallel"] for r in trace[2:])
    assert [plan["operation"] for plan in plans] == ["pipette_script_move","pipette_script_move","pipette_waste","pipette_hotel"]


def test_current_stall_and_rgb_source_contract(rig):
    _,c,trace,_,_,_=rig
    n=c["source_bindings"]
    c["set_z_current_max"](None,None)
    n.set_z_current_max(31,None,None)
    n.stall_guard(10,None,None)
    n.stall_guard(None,None,None)
    n.set_color(255,255,255,None,None)
    n.set_color(255,255,255,None,None)
    assert [r[1] for r in trace[:2]] == [(None,),(31,)]
    assert [r[2]["value"] for r in trace[2:4]] == [10,3]
    assert all(r[2]["param"] == 205 for r in trace[2:4])
    assert [r[0] for r in trace].count("rgb") == 1


def test_lifecycle_confirm_home_and_unlatch_failure(rig):
    p,_,trace,_,_,_=rig
    def run(name):
        plan=compile_finite_plate_operation(name,source_leaf_available=True)
        return execute_finite_plate_operation(plan,lambda child:p.execute_wp8_child(child,
            command_id="life",child_order=child["order"],plan_digest=plan["plan_digest"]))
    assert run("confirm_gripper")["source_return"] is False
    run("home_gripper")
    assert trace[0][1] == ("g",)
    p.primitives.tester.deck_io_set_type=lambda *a: {"ok":False}
    p._deck_semantic_state_publisher=lambda **kw: pytest.fail("failed unlatch published")
    plan=compile_finite_plate_operation("unlatch",source_leaf_available=True)
    child=plan["children"][0]
    assert p.execute_wp8_child(child,command_id="life",child_order=0,plan_digest=plan["plan_digest"])["ok"] is False


def test_real_canonical_tip_flags_publication_and_fresh_owner(rig,tmp_path):
    from bioxp.oem_runtime_store import OEMRuntimeStore
    from bioxp.operator_command_plane import OperatorCommandStore
    p,c,_,_,_,_=rig
    initial=OEMRuntimeStore(tmp_path); initial.close()
    store=OperatorCommandStore(tmp_path)
    stamps=p.deck_owner_authority_stamps()
    store.bind_deck_owner_authority_reader(lambda: stamps,scope=nullcontext)
    p._deck_semantic_state_publisher=store.publish_deck_owner_state
    try:
        c["source_bindings"].tip_state({"tip_loaded":True,"tip_dirty":True,"tip_location":2},None,None)
        assert store.deck_semantic_state()["tip_location"] == 2
    finally: store.connection.close()
    reopened=OperatorCommandStore(tmp_path)
    try:
        row=reopened.deck_semantic_state()
        assert row["tip_loaded"] and row["tip_dirty"] and row["tip_location"] == 2
    finally: reopened.connection.close()


def test_all_fourteen_d_bodies_bind_and_air_executes_real_provider(rig):
    from bioxp.services.pipette_service import build_oem_pipette_handlers
    p,c,trace,plans,_,_=rig
    calls=[]
    from tests.pressure_source_v1_support import pressure_source_transport
    pressure_transport = pressure_source_transport()
    def pipette(name, operation, action, state, identity):
        calls.append((name,identity))
        return operation(SimpleNamespace(set_top_speed=lambda *a,**kw:native(), aspirate_air=lambda *a,**kw:native(), read_pressure_for_oem_source=pressure_transport.read_pressure_for_oem_source, _tip_location_channels=lambda:[0,1,2,3]))
    handlers=build_oem_pipette_handlers(before_native_entry=lambda *a:None,pipette_call=pipette,**c)
    assert set(handlers) == {"la","ms","retip","aa","da","iniPipette","ldtip","ejt","sweep","masp","dsa","mmix","rmb","ampmix"}
    action=SimpleNamespace(params={"arguments":["5"]},source_key="original",source_occurrence_id="source:4")
    state=ProtocolRuntimeState(protocol_id="source",dry_run=False)
    result=handlers["aa"](action,state)
    assert result["ok"] and result["source_return"] == ["original"]
    assert plans[0]["operation"] == "pipette_lift"
    assert calls and calls[0][1].startswith("source:4:")
    assert c["source_bindings"].stopped(action,state) is False


def test_owned_child_callback_signature_uses_existing_owner(rig):
    p,_,_,_,_,_=rig
    owned=[]
    sentinel=object()
    def start(name,call):
        owned.append((name,call))
        return sentinel
    c=p.build_oem_pipette_source_callbacks(execute_plan=lambda *a:pytest.fail("unused"),start_child=start,stopped=lambda:True)
    callback=lambda:None
    assert c["source_bindings"].start_child("ejt.motion",callback,None,None) is sentinel
    assert owned == [("ejt.motion",callback)]
    assert c["source_bindings"].stopped(None,None) is True


def test_repeated_ms_real_wp8_claims_and_canonical_location(rig,tmp_path):
    from bioxp.oem_runtime_store import OEMRuntimeStore
    from bioxp.operator_command_plane import OperatorCommandStore
    from bioxp.oem_deck_movement import make_wp8_operation_executor
    from bioxp.services.pipette_service import build_oem_pipette_handlers
    from bioxp.protocols.runtime_state import SourceTray, SourceWell
    p,_,trace,_,_,_=rig
    initial=OEMRuntimeStore(tmp_path); initial.close()
    store=OperatorCommandStore(tmp_path)
    stamps=p.deck_owner_authority_stamps()
    store.bind_deck_owner_authority_reader(lambda:stamps,scope=nullcontext)
    store.bind_workflow_dispatcher(lambda command:None)
    p._deck_semantic_state_publisher=store.publish_deck_owner_state
    # Explicit native-double starting observation through the real publisher;
    # delivery lineage requires the canonical board/generation stamp as live.
    store.publish_deck_owner_state(source_operation="pipette_owner",source_command_id="starting-observation",
        updates={"tip_loaded":False,"tip_dirty":False,"tip_location":-1},**stamps)
    store.admit_workflow(command_id="parent",idempotency_key="parent",plan_fingerprint="captured",
        requested_inputs={"bundle":{"execution":{"runtime_state":{}}}},ownership_generation=1,
        resources=("axis:x","axis:y","axis:z","gripper"),board_epochs={})
    assert store.claim_next()["command_id"] == "parent"
    execute_wp8=make_wp8_operation_executor(provider_getter=lambda:p,command_store=store)
    admission_state={"ownership_generation":1,"serial206_initialization_provider":{
        "x_authority":{"current_board_lifecycle_generation":1},"board4_authority":{"active_board_epoch":1}}}
    children=[]
    def execute(plan,action,state):
        step=len(children)
        with store.workflow_context("parent",source_occurrence_id=f"{action.source_occurrence_id}:native:{step}"):
            admitted=store.admit_internal_wp8_operation(plan["operation"],inputs={},state=admission_state,
                idempotency_key=f"nested-{step}",prepared_plan=plan)
        cid=admitted["command_id"]; children.append(cid)
        claimed=store.claim_next()
        assert claimed["command_id"] == cid
        response=execute_wp8(command_id=cid,plan=plan)
        store.finish(cid,status="completed",payload={"response":response},claimed=claimed)
        return response
    callbacks=p.build_oem_pipette_source_callbacks(execute_plan=execute,start_child=lambda *a:pytest.fail("task"),stopped=lambda:False,settings={"LogPressure":False})
    handlers=build_oem_pipette_handlers(before_native_entry=lambda *a:store.assert_workflow_current("parent"),**callbacks)
    state=ProtocolRuntimeState(protocol_id="canonical",dry_run=False)
    state.source_model.strips=[SourceTray(str(i),11+i,[SourceWell("sample",50,100),SourceWell(None,0,100)]) for i in range(4)]
    action=SimpleNamespace(params={"arguments":["sample","5"]},source_key="key",source_occurrence_id="repeated")
    try:
        for _ in range(2):
            assert handlers["ms"](action,state)["ok"]
        assert len(children) == len(set(children)) == 4
        assert len([r for r in trace if r[0] == "scriptmove"]) == 2
        assert store.deck_semantic_state()["current_location"] == "LOC_STRIP1"
        assert store.deck_semantic_state()["current_well"] == 0
        for cid in children:
            assert store.get_command(cid)["status"] == "completed"
            assert store.connection.execute("SELECT parent_command_id FROM operator_commands WHERE command_id=?",(cid,)).fetchone()[0] == "parent"
            assert store.wp8_operation_evidence(cid)["children"][0]["terminal_state"] == "completed"
        assert store.finish_workflow("parent",status="completed",payload={},lifecycle_settled=True)["command"]["status"] == "completed"
    finally: store.connection.close()
    reopened=OperatorCommandStore(tmp_path)
    try:
        assert reopened.deck_semantic_state()["current_location"] == "LOC_STRIP1"
        assert all(reopened.get_command(cid)["status"] == "completed" for cid in children)
    finally: reopened.connection.close()


def test_selected_source_publication_shapes(rig):
    p,c,trace,_,_,_=rig
    publications=[]
    p._deck_semantic_state_publisher=lambda **kw: publications.append(kw) or kw
    p.publish_tip_tray_transition=lambda **kw: publications.append(kw) or kw
    c["publish_location"](3,14,None,None)
    c["source_bindings"].pierce(2,14,True,None,None)
    c["remove_tip"](0,"A1",None,None)
    c["publish_tip_transition"](0,[0,24,48,72],None,None)
    assert publications[0]["updates"] == {"current_location":"LOC_RC","current_well":14}
    assert publications[1]["updates"] == {"well_pierced":[2,14,1]}
    assert publications[2]["well_ids"] == [0,24,48,72]
    assert publications[2]["transition"] == "remove"
    assert publications[3]["transition"] == "restore"
    assert len({r.get("source_command_id",r.get("operation_id")) for r in publications}) == 4
