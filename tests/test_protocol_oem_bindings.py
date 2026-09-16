"""Connected API composition with explicit native recorders; denied runner only."""
from contextlib import contextmanager
from types import SimpleNamespace
import pytest
from bioxp import api
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider as Provider
from bioxp.oem_deck_movement import execute_finite_plate_operation
from bioxp.protocols.models import ProtocolDocument, ProtocolAction, ProtocolActionKind
from bioxp.protocols.runtime_state import ProtocolRuntimeState, ProtocolWorkflowState
from bioxp.protocols.validators import validate_oem_selected_dependencies, validate_protocol_support


def document(opcode, arguments=(), settings=None, form="raw"):
    return ProtocolDocument.from_payload({"protocol_id": "bindings", "metadata": {
        "input_mode": "oem_prepared", "source_settings": {"LogPressure": False, **(settings or {})}},
        "stages": [{"stage_id": "one", "actions": [{"action_id": "a", "stage_id": "one",
        "kind": "oem_operation", "oem_opcode": opcode, "source_occurrence_id": "source:0",
        "params": {"arguments": arguments, "argument_type": form}}]}]})


@pytest.fixture
def rig(monkeypatch):
    trace = []
    class Store:
        occurrence = None
        @contextmanager
        def workflow_context(self, job, *, source_occurrence_id):
            old, self.occurrence = self.occurrence, source_occurrence_id
            try:
                yield
            finally:
                self.occurrence = old
        def assert_workflow_current(self, job):
            trace.append(("fence", job))
        @contextmanager
        def normal_mutation_scope(self, *, resources):
            trace.append(("claim", self.occurrence, resources))
            yield "publication-child"
    store = Store()
    from bioxp.pipette.transport import FourPipetteTransport
    provider = Provider(primitives=SimpleNamespace(pipette_transport=object.__new__(FourPipetteTransport)))
    provider.wp8_operation_machine_state = lambda operation, inputs: {}
    provider.sleep = lambda seconds: trace.append(("sleep", seconds))
    def execute(plan, action, state):
        trace.append(("plan", store.occurrence, plan["operation"]))
        return execute_finite_plate_operation(plan, lambda child: trace.append(("native", child)) or {"ok": True})
    monkeypatch.setattr(api, "_protocol_command_store", lambda: store)
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", provider)
    monkeypatch.setattr(api.app.state, "oem_workflow_plan_executor", execute, raising=False)
    state = ProtocolRuntimeState(protocol_id="bindings", job_id="job", dry_run=False)
    state.workflow = ProtocolWorkflowState(command_id="job")
    return store, provider, state, trace


def bind(doc):
    return api._protocol_bindings({"protocol": {"document": doc.to_payload()}})


def test_all_nonthermal_composites_bound_without_preflight_native_reads(rig):
    from bioxp.protocols.executor import ProtocolExecutor
    store, provider, state, trace = rig
    doc = document("ldtip", ("50", "false", "0"), settings={"StartMode": 0,
        "CameraInstalled": False, "CameraCalibrated": False, "OverPressChecked": None,
        "CheckSnapTips": False, "JobName": None})
    ordinary, handlers, lifecycle = bind(doc)
    assert {"la", "ms", "retip", "aa", "da", "iniPipette", "ldtip", "ejt", "sweep",
            "masp", "dsa", "mmix", "rmb", "ampmix"} <= set(handlers)
    validate_protocol_support(doc, handlers=ordinary, oem_handlers=handlers)
    assert trace == []
    assert {"run_job", "script_prologue", "epilogue_sweep", "epilogue_park",
            "deferred_pause_enter", "ordinary_pause_prepare"} <= set(lifecycle)
    assert "sp" not in handlers and "splid" not in handlers
    assert "wake" not in lifecycle and "cleanup" in lifecycle
    with pytest.raises(ValueError, match="lifecycle"):
        validate_protocol_support(doc, handlers=ordinary, oem_handlers=handlers,
            lifecycle_handlers=lifecycle, required_lifecycle=ProtocolExecutor.required_lifecycle(doc))
    assert trace == []


def test_run_job_real_pipette_prefix_then_finite_gripper_and_door(rig, monkeypatch):
    store, provider, state, trace = rig
    provider._park_collection_state = lambda: {"tip_exists": False}
    provider.wp8_operation_machine_state = lambda operation, inputs: {"door_is_open": True} if operation == "thermal_door" else {}
    def pipette(name, call, action, runtime, identity):
        trace.append(("pipette", identity, name))
        return call(SimpleNamespace(query_tip_status_all=lambda: {"ok": True, "source_return": 0}))
    monkeypatch.setattr(api, "_protocol_source_pipette_call", pipette)
    def execute(plan, action, runtime):
        trace.append(("plan", store.occurrence, plan["operation"]))
        return execute_finite_plate_operation(plan,
            lambda child: trace.append(("native", child)) or {"ok": True, "source_return": False})
    monkeypatch.setattr(api.app.state, "oem_workflow_plan_executor", execute)
    _, _, lifecycle = bind(document("park"))
    result = lifecycle["run_job"](state)
    assert result["ok"]
    assert [row[2] for row in trace if row[0] == "plan"] == [
        "pipette_tip_state", "confirm_gripper", "home_gripper", "thermal_door"]
    assert next(row for row in trace if row[0] == "pipette")[1].startswith("lifecycle:run_job_tip_prefix:0:")
    assert ("sleep", .5) in trace


@pytest.mark.parametrize("failure", [False, True])
def test_real_air_overload_composed_after_finite_lift_and_original_fence(rig, monkeypatch, failure):
    from tests.test_protocol_oem_air_native import collection
    from bioxp.oem_deck_movement import DeckExecutionFailure
    store, provider, state, trace = rig
    group, drivers, native_trace = collection()
    # Explicit calibration leaf double; actual provider facts/cache remain used.
    from bioxp.oem_compat.position_table import PositionTable, PositionTarget
    from bioxp.oem_compat.pathing import LOCATION_ID_TO_NAME
    table = PositionTable([PositionTarget(LOCATION_ID_TO_NAME[3], base_coordinates={"x": 0, "y": 0},
        z_low=90000, z_high=30000, inc_factor=1)])
    monkeypatch.setattr("bioxp.oem_serial206_initialization.load_bound_oem_position_table", lambda: table)
    provider.primitives.pipette_transport = group
    provider.mov_execution_machine_state = lambda: {"current_location": 3, "current_well": 0, "tip_location": -1}
    provider._canonical_deck_semantic_state = lambda: {"current_tray": 0}
    monkeypatch.setattr(group, "set_top_speed", lambda value: trace.append(("speed", value)) or {"ok": True})
    monkeypatch.setattr(api, "_protocol_source_pipette_call", lambda name, call, action, runtime, identity: call(group))
    if failure:
        monkeypatch.setattr(api.app.state, "oem_workflow_plan_executor", lambda *args: {
            "ok": False, "delivery_attempted": True, "provider_results": [{"native_partial": True}]})
    doc = document("da", ("5",), settings={"LogPressure": True})
    _, handlers, _ = bind(doc)
    if failure:
        with pytest.raises(DeckExecutionFailure) as caught:
            handlers["da"](doc.stages[0].actions[0], state)
        assert caught.value.provider_results[0]["provider_results"] == [{"native_partial": True}]
        assert not native_trace and not [r for r in trace if r[0] == "speed"]
    else:
        result = handlers["da"](doc.stages[0].actions[0], state)
        assert result["ok"]
        assert len([r for r in native_trace if r[0] == "dispense"]) == 4
        assert next(r for r in trace if r[0] == "plan")[2] == "pipette_lift"
        assert ("speed", 30.0) in trace


def test_api_eject_uses_same_executor_owned_tasks_with_source_identity(rig, monkeypatch):
    from bioxp.protocols.executor import ProtocolExecutor
    from tests.test_protocol_oem_air_native import collection
    from bioxp.oem_compat.position_table import PositionTable, PositionTarget
    from bioxp.oem_compat.pathing import LOCATION_ID_TO_NAME
    monkeypatch.setattr("bioxp.runtime_audit_store.workflow_claim_context", lambda: {"source_occurrence_id": "source:0"})
    store, provider, state, trace = rig
    group, _, _ = collection()
    provider.primitives.pipette_transport = group
    provider.mov_execution_machine_state = lambda: {"current_location": 6, "current_well": 0, "tip_location": -1}
    provider._canonical_deck_semantic_state = lambda: {"current_tray": 0}
    provider._park_collection_state = lambda: {"tip_exists": False}
    provider.wp8_operation_machine_state = lambda operation, inputs: {"door_is_open": True} if operation == "thermal_door" else {}
    table = PositionTable([PositionTarget(LOCATION_ID_TO_NAME[6], base_coordinates={"x": 0, "y": 0}, z_low=90000, z_high=30000, inc_factor=1)])
    monkeypatch.setattr("bioxp.oem_serial206_initialization.load_bound_oem_position_table", lambda: table)
    transport = SimpleNamespace(query_tip_status_all=lambda: {"ok": True, "source_return": 0},
        read_pressure=lambda: {"ok": True, "channels": []}, eject_all_tips=lambda **kw: {"ok": True})
    monkeypatch.setattr(api, "_protocol_source_pipette_call", lambda name, call, action, runtime, identity: call(transport))
    def execute(plan, action, runtime):
        trace.append(("plan", store.occurrence, plan["operation"]))
        return execute_finite_plate_operation(plan, lambda child: {"ok": True, "source_return": True})
    monkeypatch.setattr(api.app.state, "oem_workflow_plan_executor", execute)
    doc = document("ejt", settings={"CheckSnapTips": False, "JobName": None})
    from bioxp.protocols.runtime_state import SourceTray, SourceWell
    state.source_model.tip_trays = [SourceTray(str(i), 7 + i, [SourceWell("Reuse", 0, 0, False) for _ in range(96)]) for i in range(4)]
    bindings = api._protocol_bindings({"protocol": {"document": doc.to_payload()}}, source_executor=lambda: executor)
    ordinary, handlers, lifecycle = bindings
    # Explicit missing thermal/lifecycle leaf doubles only for host orchestration.
    doubled = []
    for name in ProtocolExecutor.required_lifecycle(doc):
        if name not in lifecycle:
            lifecycle[name] = lambda runtime, name=name: doubled.append(name) or {"ok": True, "offline_leaf_double": name}
    executor = ProtocolExecutor(dry_run=False, job_id=state.job_id, handlers=ordinary, oem_handlers=handlers,
        lifecycle_handlers=lifecycle, before_native_entry=lambda identity, runtime: trace.append(("entry", identity)),
        source_script_begin=bindings.source_script_begin, source_script_returned=bindings.source_script_returned)
    result = executor.execute(doc, state=state)
    import json
    assert executor.outcome == "completed", json.dumps(result.to_payload(), default=str)
    children = [row["action_id"] for row in result.action_results if row.get("kind") == "owned_child"]
    assert children == ["source:0:ejt.ejection", "source:0:ejt.motion"]
    plans = [row[1] for row in trace if row[0] == "plan" and row[1].startswith("source:0:")]
    assert len(plans) == len(set(plans)) and plans
    assert doubled  # This is not live thermal support.


def test_original_stop_blocks_nested_source_entry(rig, monkeypatch):
    store, provider, state, trace = rig
    def refused(job):
        raise ValueError("workflow_interrupted")
    store.assert_workflow_current = refused
    monkeypatch.setattr(api, "_protocol_source_pipette_call", lambda *a: pytest.fail("pipette entered after Stop"))
    doc = document("iniPipette")
    _, handlers, _ = bind(doc)
    with pytest.raises(ValueError, match="workflow_interrupted"):
        handlers["iniPipette"](doc.stages[0].actions[0], state)
    assert trace == []


def test_api_real_snapshot_compiler_and_unique_nested_keys(rig):
    store, provider, state, trace = rig
    doc = document("snapshot", settings={"CheckSnapTips": True})
    _, handlers, _ = bind(doc)
    action = doc.stages[0].actions[0]
    handlers["snapshot"](action, state)
    handlers["snapshot"](action, state)
    assert [row[1] for row in trace if row[0] == "plan"] == ["source:0:native:0", "source:0:native:1"]
    assert [row[1]["operation"] for row in trace if row[0] == "native"] == ["scriptmoveTo", "updateLocation", "Sleep", "SnapshotImage"] * 2


def test_real_lifecycle_three_pressure_reads_then_image_null_branch(rig, monkeypatch):
    store, provider, state, trace = rig
    def pipette(name, call, action, runtime, identity):
        transport = SimpleNamespace(read_pressure=lambda: trace.append(("read_pressure", identity)) or {
            "ok": True, "channels": [{"channel": 2, "result": {"pressure": 31.0}}]})
        return call(transport)
    monkeypatch.setattr(api, "_protocol_source_pipette_call", pipette)
    _, _, lifecycle = bind(document("park", settings={"JobName": None}))
    result = lifecycle["script_prologue"](state)
    assert result["ok"]
    identities = [row[1] for row in trace if row[0] == "read_pressure"]
    assert len(identities) == len(set(identities)) == 3
    assert state.source_model.pressure_baseline == [0.0, 0.0, 31.0, 0.0]
    assert not [row for row in trace if row[0] == "native"]


@pytest.mark.parametrize("opcode,args,form", [
    ("masp", {"m_volume": "5"}, "ClassAspirate"),
    ("dsa", {"m_ntd": 1}, "ClassDispenseAll"),
    ("ampmix", {"m_repeat": 1.5}, "ClassAmpMix"),
    ("mmix", {"m_aspirateOptions": {"bogus": 1}}, "ClassMix"),
    ("rmb", {"m_dispenseOptions": {"m_delay": True}}, "ClassDebubble"),
    ("mov", {"m_destination": 3}, "ClassMoveTo"),
])
def test_typed_fields_reject_before_native(opcode, args, form, rig):
    with pytest.raises(ValueError):
        bind(document(opcode, args, form=form))
    assert rig[3] == []


def test_nullable_nondefault_options_unchanged():
    doc = document("masp", {"m_volume": 17.25, "m_speed": None, "m_overaspirate": -2}, form="ClassAspirate")
    before = doc.to_payload()
    validate_oem_selected_dependencies(doc)
    assert doc.to_payload() == before


@pytest.mark.parametrize("settings", [{"DeckInspection": True, "StartMode": 1}, {"DeckInspection": True, "StartMode": 2}])
def test_selected_barcode_dependency_refuses_before_native(settings, rig):
    with pytest.raises(ValueError, match="ReadBarcode"):
        bind(document("dopen", settings=settings))
    assert rig[3] == []


def test_native_pressure_stream_not_guessed(rig, monkeypatch):
    from bioxp.pipette.transport import FourPipetteTransport
    monkeypatch.setattr(FourPipetteTransport, "aspirate_for_oem_script", None)
    with pytest.raises(ValueError, match="aspirate_pressure_stream"):
        bind(document("masp", {}, settings={"LogPressure": True}, form="ClassAspirate"))
    assert rig[3] == []


def test_handler_selected_preflight_is_run_without_execution():
    doc = document("da", ("3",))
    called = []
    def handler(action, state):
        pytest.fail("preflight dispatched native work")
    def preflight(action):
        called.append(action.source_occurrence_id)
        raise ValueError("selected native missing")
    handler.preflight = preflight
    with pytest.raises(ValueError, match="selected native missing"):
        validate_protocol_support(doc, oem_handlers={"da": handler}, handlers={})
    assert called == ["source:0"]


def test_prepared_mov_uses_typed_intent_and_records_child(rig, monkeypatch):
    store, provider, state, trace = rig
    def admit(intent, *, idempotency_key):
        trace.append(("mov", intent, idempotency_key, store.occurrence))
        return {"command_id": "mov-child"}
    monkeypatch.setattr(api.app.state, "oem_mov_execution_admitter", admit, raising=False)
    store.get_command = lambda cid: {"status": "completed", "terminal_evidence": {"response": {"source_return": [7]}}}
    doc = document("mov", {"m_destination": {"enum_type": "locationID", "value": 6},
                           "m_well": "A1", "m_oldWell": False}, form="ClassMoveTo")
    _, handlers, _ = bind(doc)
    result = handlers["mov"](doc.stages[0].actions[0], state)
    assert result["ok"] and result["source_return"] == [7]
    recorded = next(row for row in trace if row[0] == "mov")
    assert recorded[1].location_id == 6 and recorded[1].well == "A1"
    assert recorded[2:] == ("protocol:job:source:0:mov", "source:0")
    assert state.workflow.child_command_ids == ["mov-child"]


def test_critical_image_settings_rejected_before_pressure_or_motion(rig):
    with pytest.raises(ValueError, match="CameraXOffset"):
        bind(document("park", settings={"JobName": "captured-job"}))
    assert rig[3] == []


@pytest.mark.parametrize("opcode,args", [("led", ("1", "2", "nan")), ("so", ("2.2",)), ("catch", ("absent",)), ("release", ("absent",))])
def test_late_raw_native_arguments_are_preflighted(rig, opcode, args):
    with pytest.raises(ValueError):
        bind(document(opcode, args))
    assert rig[3] == []


def test_retip_actual_canonical_publication_and_fresh_readback(tmp_path, monkeypatch):
    from contextlib import nullcontext
    from bioxp.oem_runtime_store import OEMRuntimeStore
    from bioxp.operator_command_plane import OperatorCommandStore
    from bioxp.protocols.runtime_state import SourceTray, SourceWell
    initial = OEMRuntimeStore(tmp_path)
    initial.close()
    owner = OperatorCommandStore(tmp_path)
    stamps = {"ownership_generation": 1, "board_epoch_4": 1, "board_epoch_5": 1}
    owner.bind_deck_owner_authority_reader(lambda: stamps, scope=nullcontext)
    owner.publish_tip_tray_transition(tray_id=0, transition="construct", operation_id="constructed",
        command_id="constructed", provenance={"test_native_double": True}, **stamps)
    owner.publish_tip_tray_transition(tray_id=0, transition="remove_well", operation_id="removed",
        command_id="removed", well_ids=[0], provenance={"test_native_double": True}, **stamps)
    owner.bind_workflow_dispatcher(lambda command: None)
    owner.admit_workflow(command_id="job", idempotency_key="job", plan_fingerprint="captured",
        requested_inputs={"bundle": {"execution": {"runtime_state": {}}}},
        ownership_generation=1, resources=("pipette",), board_epochs={})
    owner.claim_next()
    provider = object.__new__(Provider)
    provider._tip_tray_state_publisher = owner.publish_tip_tray_transition
    provider.deck_owner_authority_stamps = lambda: stamps
    provider.invalidate_deck_authority_cache = lambda **kw: None
    monkeypatch.setattr(api, "_protocol_command_store", lambda: owner)
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", provider)
    monkeypatch.setattr(api.app.state, "oem_workflow_plan_executor", None, raising=False)
    state = ProtocolRuntimeState(protocol_id="bindings", job_id="job", dry_run=False)
    state.workflow = ProtocolWorkflowState(command_id="job")
    state.source_model.tip_trays = [SourceTray("0", 7, [SourceWell("Reuse", 0, 0, True)])]
    doc = document("retip")
    _, handlers, _ = bind(doc)
    try:
        result = handlers["retip"](doc.stages[0].actions[0], state)
        assert result["ok"] and not state.source_model.tip_trays[0].wells[0].empty
        child = state.workflow.child_command_ids[0]
        claim = owner.connection.execute("SELECT parent_command_id,status FROM operator_commands WHERE command_id=?", (child,)).fetchone()
        assert tuple(claim) == ("job", "completed")
        fresh = OperatorCommandStore(tmp_path)
        try:
            persisted = fresh.tip_tray_state(0)
            assert persisted["command_id"] == child
            assert persisted["operation_id"] == "job:source:0:0:retip"
            assert persisted["occupancy"][0] is True
        finally:
            fresh.stop()
    finally:
        owner.stop()


from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig


def test_retip_r3_provider_compiler_canonical_worker_and_sqlite(installed_retained, retained_rig, monkeypatch):
    import threading
    from tests.test_deck_complete_admission import ready
    from bioxp.protocols.runtime_state import SourceTray, SourceWell
    from bioxp.operator_command_plane import OperatorCommandStore
    app, provider, primitives, references, root = installed_retained
    ready(installed_retained, monkeypatch, retained_rig)
    store = app.state.operator_command_plane.store
    stamps = provider.deck_owner_authority_stamps()
    # ready() refreshes hardware but leaves the reset semantic row unstamped.
    # Supply the fixture's initial source observation through its real owner.
    store.publish_deck_owner_state(source_operation="pipette_owner",
        source_command_id="retip-starting-observation",
        updates={"tip_loaded": False, "tip_dirty": False, "tip_location": -1}, **stamps)
    semantic = store.deck_semantic_state()
    assert {key: semantic[key] for key in stamps} == stamps
    delivery_calls = []
    record_delivery = store.record_delivery_attempt
    def track_delivery(*args, **kwargs):
        delivery_calls.append(kwargs)
        return record_delivery(*args, **kwargs)
    monkeypatch.setattr(store, "record_delivery_attempt", track_delivery)
    store.publish_tip_tray_transition(tray_id=0, transition="construct", operation_id="retip-seed",
        command_id="retip-seed", provenance={"offline_fixture": True}, **stamps)
    store.publish_tip_tray_transition(tray_id=0, transition="remove_well", operation_id="retip-remove",
        command_id="retip-remove", well_ids=[0], provenance={"offline_fixture": True}, **stamps)
    monkeypatch.setattr(provider.primitives, "pipette_transport", None, raising=False)
    monkeypatch.setattr(api, "_protocol_command_store", lambda: store)
    entered, release = threading.Event(), threading.Event()
    def dispatch(command):
        entered.set()
        assert release.wait(8)
    store.bind_workflow_dispatcher(dispatch)
    store.admit_workflow(command_id="retip-parent", idempotency_key="retip-parent",
        plan_fingerprint="captured", requested_inputs={"bundle": {}},
        ownership_generation=stamps["ownership_generation"], resources=["pipette"], board_epochs={})
    state = ProtocolRuntimeState(protocol_id="bindings", job_id="retip-parent", dry_run=False)
    state.workflow = ProtocolWorkflowState(command_id="retip-parent")
    state.source_model.tip_trays = [SourceTray("0", 7, [SourceWell("Reuse", 0, 0, True)])]
    doc = document("retip")
    _, handlers, _ = bind(doc)
    app.state.operator_command_plane.start()
    try:
        assert entered.wait(5)
        try:
            result = handlers["retip"](doc.stages[0].actions[0], state)
        except Exception as exc:
            import json
            pytest.fail(json.dumps(getattr(exc, "provider_results", {"error": str(exc)}), default=str))
        assert result["ok"] and not state.source_model.tip_trays[0].wells[0].empty
        child = state.workflow.child_command_ids[0]
        assert store.get_command(child)["status"] == "completed"
        evidence = store.wp8_operation_evidence(child)
        assert len(evidence["children"]) == 1
        assert evidence["children"][0]["operation"] == "sourceTipTransition"
        assert evidence["children"][0]["terminal_state"] == "completed"
        plan = store.get_command(child)["effective_inputs"]["prepared_plan"]
        assert plan["children"][0]["arguments"]["transition"] == "restore"
        assert not delivery_calls and not store.has_delivery_attempt(child)
        assert store.get_command(child)["terminal_evidence"]["response"]["delivery_attempted"] is False
        fresh = OperatorCommandStore(root)
        try:
            tip = fresh.tip_tray_state(0)
            assert tip["occupancy"][0] is True and tip["command_id"] == child
            assert fresh.get_command(child)["status"] == "completed"
            assert fresh.get_command(child)["terminal_evidence"]["response"]["delivery_attempted"] is False
            assert not fresh.has_delivery_attempt(child)
            assert fresh.wp8_operation_evidence(child)["children"] == evidence["children"]
            assert fresh.connection.execute("SELECT parent_command_id FROM operator_commands WHERE command_id=?", (child,)).fetchone()[0] == "retip-parent"
        finally:
            fresh.stop()
    finally:
        release.set()
        app.state.operator_command_plane.stop()


def test_actual_pipette_adapter_lifecycle_context_has_distinct_canonical_children(query_rig, monkeypatch):
    from bioxp.services.pipette_service import _OemLifecycleContext
    app, provider, primitive, references, root, receipts, calls, wire, transport = query_rig
    store = app.state.operator_command_plane.store
    store.bind_workflow_dispatcher(lambda command: None)
    store.admit_workflow(command_id="pipette-parent", idempotency_key="pipette-parent",
        plan_fingerprint="captured", requested_inputs={"bundle": {}},
        ownership_generation=provider.deck_owner_authority_stamps()["ownership_generation"],
        resources=["pipette"], board_epochs={})
    store.claim_next()
    monkeypatch.setattr(api, "_protocol_command_store", lambda: store)
    state = ProtocolRuntimeState(protocol_id="bindings", job_id="pipette-parent", dry_run=False)
    state.workflow = ProtocolWorkflowState(command_id="pipette-parent")
    action = _OemLifecycleContext("lifecycle:run_job_tip_prefix:0", params={"arguments": ()})
    results = [api._protocol_source_pipette_call("query_tip_status_all", lambda t: t.query_tip_status_all(),
        action, state, action.source_occurrence_id + f":{n}:query") for n in range(2)]
    assert calls == [0, 1, 2, 3] * 2
    assert all(result.get("ok") is True for result in results), results
    children = [result["command_id"] for result in results]
    assert len(set(children)) == 2 and "pipette-parent" not in children
    rows = store.connection.execute("SELECT command_id,parent_command_id,status FROM operator_commands WHERE parent_command_id=? ORDER BY sequence", ("pipette-parent",)).fetchall()
    assert [tuple(row) for row in rows] == [(child, "pipette-parent", "observed") for child in children]


@pytest.mark.parametrize("delivered", [False, True])
def test_structured_native_failure_survives_canonical_child_and_reopen(
    installed_retained, retained_rig, monkeypatch, delivered,
):
    import threading
    from bioxp.oem_deck_movement import DeckExecutionFailure
    from bioxp.operator_command_plane import OperatorCommandStore
    from tests.test_deck_complete_admission import ready
    app, provider, primitives, references, root = installed_retained
    ready(installed_retained, monkeypatch, retained_rig)
    plane = app.state.operator_command_plane
    entered, release = threading.Event(), threading.Event()
    def dispatch(command):
        entered.set()
        assert release.wait(8)
    plane.store.bind_workflow_dispatcher(dispatch)
    generation = provider.deck_owner_authority_stamps()["ownership_generation"]
    plane.store.admit_workflow(command_id="failure-parent", idempotency_key="failure-parent",
        plan_fingerprint="captured", requested_inputs={"bundle": {}}, ownership_generation=generation,
        resources=["axis:x", "axis:y", "axis:z"], board_epochs={})
    partial = [{"ok": False, "source_pause_scripts": True,
                "source_error_event": "Please manually remove tips on Pipettes",
                "delivery_attempted": delivered, "source_return": False}]
    def fail(**kwargs):
        raise DeckExecutionFailure("native source Park boundary", delivery_attempted=delivered,
            controller_command_acknowledged=delivered, provider_results=partial)
    monkeypatch.setattr(app.state, "oem_wp8_operation_executor", fail)
    monkeypatch.setattr(provider.primitives, "pipette_transport", None, raising=False)
    # This fault test doubles the native pre-Park observation, not custody.
    monkeypatch.setattr(provider, "wp8_operation_machine_state", lambda operation, inputs: {})
    monkeypatch.setattr(api, "_protocol_command_store", lambda: plane.store)
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", provider)
    monkeypatch.setattr(api.app.state, "oem_workflow_plan_executor", app.state.oem_workflow_plan_executor, raising=False)
    state = ProtocolRuntimeState(protocol_id="bindings", job_id="failure-parent", dry_run=False)
    state.workflow = ProtocolWorkflowState(command_id="failure-parent")
    doc = document("park")
    _, handlers, _ = bind(doc)
    plane.start()
    try:
        assert entered.wait(5)
        result = handlers["park"](doc.stages[0].actions[0], state)
        assert result["ok"] is False and result["source_pause_scripts"] is True
        assert result["source_error_event"] == partial[0]["source_error_event"]
        assert result["status"] == ("ambiguous" if delivered else "failed")
        child = result["command_id"]
        assert state.workflow.child_command_ids == [child]
        fresh = OperatorCommandStore(root)
        try:
            evidence = fresh.get_command(child)["terminal_evidence"]
            assert evidence["response"]["provider_results"] == partial
            assert evidence["response"]["controller_command_acknowledged"] is delivered
            assert evidence["delivery_attempted"] is delivered
        finally:
            fresh.stop()
    finally:
        release.set()
        plane.stop()
