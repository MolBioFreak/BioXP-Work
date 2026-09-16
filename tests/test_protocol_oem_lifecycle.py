"""Offline executor qualification. Every native/thermal leaf is an explicit double."""
from concurrent.futures import Future
from dataclasses import replace
from threading import Event, Thread
from time import monotonic
from types import SimpleNamespace
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider

import pytest

from bioxp.protocols.compiler import compile_prepared_oem_protocol
from bioxp.protocols.executor import LIFECYCLE_HOOKS, OwnedOperation, ProtocolExecutor
from bioxp.protocols.models import ProtocolAction, ProtocolActionKind, ProtocolDocument, ProtocolStage
from bioxp.protocols.runtime_state import (
    ProtocolRuntimeState, ProtocolSourceModel, ProtocolWorkflowState, SourceTray, SourceWell,
)


ARGS = {"splid": ["30", "0", "1"], "sp": ["30", "0", "1"], "step": ["1"],
        "wait": ["1"], "led": ["1", "2", "3"], "la": ["REAGENT_PLATE", "A1", "1"]}


def document(*opcodes, **metadata):
    return compile_prepared_oem_protocol({
        "protocol_id": "offline", "metadata": metadata,
        "operations": [{"oem_opcode": op, "arguments": ARGS.get(op, [])} for op in opcodes],
    })


def engine(doc, *, handlers=None, overrides=None, publish=None, entry=None):
    trace = []
    provider = Serial206OemInitializationProvider(primitives=SimpleNamespace())

    def hook(name):
        def run(state):
            trace.append(name)
            if name == "script_finally":
                return executor.finalize_source_host(state)
            if name == "script_prologue":
                state.source_model.logical_tip_present = False
                state.source_model.carried_plate_present = False
                state.source_model.allow_to_stop = True
            return {"ok": True, "offline_native_double": name}
        return run

    hooks = {name: hook(name) for name in LIFECYCLE_HOOKS}
    hooks.update(overrides or {})
    executor = ProtocolExecutor(
        dry_run=False, job_id="canonical-parent", oem_handlers=handlers or {},
        lifecycle_handlers=hooks, before_native_entry=entry or (lambda identity, state: None),
        on_state_change=publish,
        source_script_begin=lambda state: provider.wp8_source_script_begin(command_id=state.workflow.command_id),
        source_script_returned=lambda state: provider.wp8_source_script_returned(command_id=state.workflow.command_id),
    )
    return executor, trace


def wait_until(predicate):
    deadline = monotonic() + 4
    while not predicate():
        assert monotonic() < deadline, "bounded offline condition not reached"
        Event().wait(0.005)


def start(executor, doc):
    result = []
    errors = []

    def run():
        try:
            result.append(executor.execute(doc))
        except BaseException as error:
            errors.append(error)
    thread = Thread(target=run, daemon=True)
    thread.start()
    return thread, result, errors


def finish(run):
    thread, result, errors = run
    thread.join(4)
    assert not thread.is_alive(), "executor did not settle"
    assert not errors
    assert len(result) == 1
    return result[0]


def test_preflight_entire_roster_before_prologue():
    doc = document("step", "led")
    executor, trace = engine(doc)
    report = executor.preflight(doc)
    assert report["missing"] == ["opcode:led"]
    with pytest.raises(ValueError, match="led"):
        executor.execute(doc)
    assert not trace
    assert "epilogue_lid" in executor.required_lifecycle(doc)
    assert "prepare" not in executor.required_lifecycle(doc)
    assert "prepare" in executor.required_lifecycle(document("step", oem_prepare=True))


def test_tc_only_overlap_then_lid_join_and_final_settlement():
    doc = document("splid", "step")
    tc_entered, release = Event(), Event()

    def tc(action, state):
        tc_entered.set()
        assert release.wait(4)
        return {"ok": True, "command_id": "tc-child", "native_result": "returned"}

    executor, trace = engine(doc, handlers={"splid": tc})
    run = start(executor, doc)
    assert tc_entered.wait(2)
    wait_until(lambda: "epilogue_sweep" in trace)
    assert "epilogue_lid" not in trace
    assert run[0].is_alive()
    release.set()
    state = finish(run)
    assert state.completed
    assert trace.index("epilogue_sweep") < trace.index("epilogue_lid") < trace.index("epilogue_park")
    assert state.workflow.child_command_ids == ["tc-child"]
    assert executor.outcome == "completed"


def test_synchronous_tip_and_tc_native_doubles_overlap():
    doc = document("iniPipette", "splid")
    tip_entered, tc_entered, release = Event(), Event(), Event()

    def tip(action, state):
        tip_entered.set()
        assert release.wait(4)
        return {"ok": True}

    def tc(action, state):
        tc_entered.set()
        assert release.wait(4)
        return {"ok": True}

    executor, _ = engine(doc, handlers={"iniPipette": tip, "splid": tc})
    run = start(executor, doc)
    assert tip_entered.wait(2) and tc_entered.wait(2)
    release.set()
    assert finish(run).completed


def test_general_thermal_operation_does_not_allow_sweep_overlap():
    doc = document("sp", "step")
    pending = Future()
    executor, trace = engine(doc, handlers={"sp": lambda action, state: pending})
    run = start(executor, doc)
    wait_until(lambda: any(r.get("pending") for r in executor._state.action_results))
    assert "epilogue_sweep" not in trace
    pending.set_result({"ok": True})
    assert finish(run).completed


def test_late_failure_preserves_successful_sibling_and_blocks_lid():
    doc = document("splid", "step")
    pending = Future()
    executor, trace = engine(doc, handlers={"splid": lambda action, state: pending})
    run = start(executor, doc)
    wait_until(lambda: "epilogue_sweep" in trace)
    pending.set_result({"ok": False, "native_error": "correlated"})
    state = finish(run)
    assert executor.outcome == "failed" and not state.completed
    assert "epilogue_lid" not in trace
    assert "source_error" in trace
    assert next(r for r in state.action_results if r.get("source_marker"))["ok"] is True


@pytest.mark.parametrize("returned", [None, {}, {"status": "completed"}])
def test_missing_explicit_outcome_is_uncertain_not_success(returned):
    doc = document("led")
    executor, trace = engine(doc, handlers={"led": lambda action, state: returned})
    state = executor.execute(doc)
    assert executor.outcome == "ambiguous"
    assert not state.completed and state.workflow.phase == "reconciling"
    assert "cleanup" not in trace and "source_error" not in trace


@pytest.mark.parametrize("stop", ["abort", "false", "stop"])
@pytest.mark.parametrize("gate", ["ordinary_pause", "deferred_pause", "delaypoint", "review", "error_hold"])
def test_gate_termination_cross_product(gate, stop):
    doc = document("delaypoint" if gate == "delaypoint" else "step", delayed_start=gate == "delaypoint")
    if gate == "review":
        stage = doc.stages[0]
        doc = replace(doc, stages=(replace(stage, actions=(replace(stage.actions[0], review_required=True),)),))
    executor, trace = engine(doc)
    # Gate setup through public control/source result, never by releasing a
    # fabricated native task. A prologue double signals the source pause.
    original = executor._lifecycle_handlers["script_prologue"]

    def prologue(state):
        result = original(state)
        if gate in {"ordinary_pause", "deferred_pause"}:
            executor.request_control("pause", control_id="pause", mode=gate.split("_")[0])
        return result
    executor._lifecycle_handlers["script_prologue"] = prologue
    if gate == "error_hold":
        doc = document("dopen")
        executor._oem_handlers["dopen"] = lambda action, state: {"ok": False, "source_error_hold": True}
    run = start(executor, doc)
    wait_until(lambda: executor._state and executor._state.workflow.gate == gate)
    if stop == "abort":
        executor.request_control("abort", control_id="abort")
    elif stop == "false":
        executor.source_error(false_abort=True)
    else:
        executor.interrupt(control_id="addressed-stop")
    state = finish(run)
    assert not state.completed
    assert state.workflow.gate is None
    if stop == "stop":
        assert executor.outcome == "interrupted"
        assert not any(x in trace for x in ("cleanup", "abort_true_finish", "source_error", "abort_false"))
    elif stop == "false":
        assert "abort_false" in trace and "cleanup" not in trace
    elif gate == "error_hold":
        assert "source_error" in trace and "abort_true_finish" not in trace
    else:
        assert trace.count("abort_true_prefix") == 1
        assert trace.count("abort_true_finish") == 1
        assert trace.count("cleanup") == 1
    assert "ordinary_pause_restore" not in trace and "wake" not in trace


def test_deferred_wake_then_distinct_continue_and_idempotent_control():
    doc = document("step")
    executor, trace = engine(doc)
    original = executor._lifecycle_handlers["script_prologue"]

    def prologue(state):
        result = original(state)
        executor.request_control("pause", control_id="pause", mode="deferred")
        return result
    executor._lifecycle_handlers["script_prologue"] = prologue
    run = start(executor, doc)
    wait_until(lambda: executor._state and executor._state.workflow.gate == "deferred_pause")
    with pytest.raises(ValueError, match="full wake"):
        executor.request_control("continue", control_id="too-early", gate="deferred_pause", gate_id="pause")
    executor.request_control("wake", control_id="wake", gate_id="pause")
    wait_until(lambda: executor._wake_complete)
    assert run[0].is_alive()
    assert executor._state.workflow.gate == "deferred_pause"
    with pytest.raises(ValueError, match="another gate"):
        executor.acknowledge_review(control_id="bad-review", gate_id="pause")
    executor.request_control("continue", control_id="continue", gate="deferred_pause", gate_id="pause")
    assert finish(run).completed
    assert trace.count("wake") == 1
    assert executor.request_control("wake", control_id="wake", gate_id="pause")["reached"] is True


def test_cooperative_abort_keeps_existing_source_progression_to_safe_boundary():
    doc = document("led", "la", "step")
    executor, trace = engine(doc)

    def hold_tip(action, state):
        state.source_model.logical_tip_present = True
        executor.request_control("abort", control_id="abort")
        return {"ok": True}

    def release_tip(action, state):
        trace.append("source_release_tip")
        state.source_model.logical_tip_present = False
        return {"ok": True}
    executor._oem_handlers.update(led=hold_tip, la=release_tip)
    state = executor.execute(doc)
    assert executor.outcome == "interrupted"
    assert "source_release_tip" in trace and "safe_stop_exit" in trace
    assert not any(r.get("source_marker") for r in state.action_results)
    assert trace.index("source_release_tip") < trace.index("safe_stop_exit") < trace.index("abort_true_finish")


def test_review_never_approves_unsafe_abort_progression():
    doc = document("led", "step")
    stage = doc.stages[0]
    doc = replace(doc, stages=(replace(stage, actions=(replace(stage.actions[0], review_required=True), stage.actions[1])),))

    def tip(action, state):
        state.source_model.logical_tip_present = True
        return {"ok": True}
    executor, trace = engine(doc, handlers={"led": tip})
    run = start(executor, doc)
    wait_until(lambda: executor._state and executor._state.workflow.gate == "review")
    executor.request_control("abort", control_id="abort")
    state = finish(run)
    assert executor.outcome == "ambiguous"
    assert state.workflow.held_reason == "review_blocks_termination"
    assert "cleanup" not in trace
    assert not any(r.get("source_marker") for r in state.action_results)


def test_stop_retains_entered_child_and_never_runs_future_occurrence():
    doc = document("splid", "step", "led")
    child = Future()
    entered = Event()

    def tc(action, state):
        entered.set()
        return child
    executor, trace = engine(doc, handlers={"splid": tc, "led": lambda a, s: {"ok": True}})
    original = executor._lifecycle_handlers["epilogue_sweep"]
    # Request Stop while the entered TC child still belongs to this parent.
    executor._oem_handlers["led"] = lambda a, s: (executor.interrupt(control_id="stop") or {"ok": True})
    run = start(executor, doc)
    assert entered.wait(2)
    wait_until(lambda: executor._interrupted)
    assert run[0].is_alive() and not child.cancelled()
    child.set_result({"ok": True, "command_id": "original-child"})
    state = finish(run)
    assert executor.outcome == "interrupted"
    assert "cleanup" not in trace and "epilogue_lid" not in trace
    assert state.workflow.child_command_ids == ["original-child"]


def test_unaffected_stop_leaves_delay_gate_untouched():
    doc = document("delaypoint", delayed_start=True)
    executor, _ = engine(doc)
    run = start(executor, doc)
    wait_until(lambda: executor._state and executor._state.workflow.gate == "delaypoint")
    gate_id = executor._state.workflow.gate_id
    executor.interrupt(control_id="unrelated-X", affected=False)
    assert executor._state.workflow.gate_id == gate_id
    executor.request_control("continue", control_id="start-now", gate="delaypoint", gate_id=gate_id)
    assert finish(run).completed


def test_recording_failure_retains_results_without_physical_cleanup():
    doc = document("step")

    def publish(state):
        if any(r.get("source_marker") for r in state.action_results):
            raise OSError("offline injected recording failure")
    executor, trace = engine(doc, publish=publish)
    state = executor.execute(doc)
    assert executor.outcome == "ambiguous"
    assert next(r for r in state.action_results if r.get("source_marker"))["ok"] is True
    assert "cleanup" not in trace and "source_error" not in trace


def test_no_rehydrated_live_cursor_resume():
    doc = document("step")
    executor, _ = engine(doc)
    state = executor.execute(doc)
    restored = ProtocolRuntimeState.from_payload(state.to_payload())
    other, trace = engine(doc)
    with pytest.raises(ValueError, match="reconciliation"):
        other.execute(doc, state=restored)
    assert not trace


def test_wait_is_host_owned_and_abort_releases_without_timer_expiry():
    doc = document("wait")
    action = doc.stages[0].actions[0]
    doc = replace(doc, stages=(replace(doc.stages[0], actions=(replace(action, params={"arguments": ["600"]}),)),))
    executor, trace = engine(doc)
    run = start(executor, doc)
    wait_until(lambda: executor._state and any(r.get("pending") and r.get("kind") == "oem_operation" for r in executor._state.action_results))
    executor.request_control("abort", control_id="abort")
    state = finish(run)
    assert executor.outcome == "interrupted"
    row = next(r for r in state.action_results if r.get("kind") == "oem_operation")
    assert row["host_exit"] == "termination"


def test_source_model_exact_mutations_selection_and_serialization():
    wells = [SourceWell("fluid", 5, 10) for _ in range(96)]
    wells[24].volume = 1
    model = ProtocolSourceModel(trays={"REAGENT_PLATE": SourceTray("reagent", 1, wells)})
    assert model.la(["7", "la", "reagent_plate", "a1", "2"]) == ["7"]
    assert [wells[i].volume for i in (0, 24, 48, 72)] == [3, 1, 3, 3]
    assert model.la([None, "la", "REAGENT_PLATE", "A1", "-20", "F"]) == [None]
    assert wells[0].volume == 3  # ignored source capacity return, no clamping
    model.strips = [SourceTray(str(i), i, [SourceWell("target", 1, 10), SourceWell("other", 2, 10)]) for i in range(4)]
    assert model.select_strip("target", 1.001) == (0, 0)
    assert model.select_strip("missing", 1) == (0, -1)
    model.tip_trays = [SourceTray("tray", 5, [SourceWell("Reuse", 0, 1, True), SourceWell("Used", 0, 1, True)]),
                       SourceTray("hotel", 15, [SourceWell("Reuse", 0, 1, True)])]
    assert model.retip_wells() == [("tray", [0])]
    model.retip_committed("tray", [0])
    model.pressure_baseline.append({"channel": 1, "value": 4})
    restored = ProtocolSourceModel.from_payload(model.to_payload())
    assert restored.to_payload() == model.to_payload()
    assert restored.retip_wells() == [("tray", [])]
    restored.pressure_baseline[0]["value"] = 9
    assert model.pressure_baseline[0]["value"] == 4


@pytest.mark.parametrize("patch", [{"phase": "terminating"}, {"gate": "paused"}, {"extra": 1}, {"child_command_ids": [1]}])
def test_workflow_wire_rejects_unfrozen_vocabulary(patch):
    with pytest.raises(ValueError):
        ProtocolWorkflowState.from_payload({"command_id": "parent", **patch})


def test_delayed_general_child_allows_source_tc_overlap():
    doc = document("delaypoint", "splid", delayed_start=True)
    entered = Event()
    executor, trace = engine(doc, handlers={"splid": lambda a, s: (entered.set() or {"ok": True})})
    run = start(executor, doc)
    assert entered.wait(2)
    wait_until(lambda: executor._state.workflow.gate == "delaypoint")
    assert "epilogue_sweep" not in trace
    executor.request_control("continue", control_id="start", gate="delaypoint", gate_id=executor._state.workflow.gate_id)
    assert finish(run).completed


@pytest.mark.parametrize("false_abort", [True, False])
def test_source_control_effect_not_stranded_behind_active_native_child(false_abort):
    doc = document("splid", "step")
    child = Future()
    entered = Event()
    executor, trace = engine(doc, handlers={"splid": lambda a, s: (entered.set() or child)})
    hook_name = "abort_false" if false_abort else "source_error_request"

    def control(state):
        trace.append(hook_name)
        child.set_result({"ok": False, "source_return": "aborted"})
        return {"ok": True}
    executor._lifecycle_handlers[hook_name] = control
    run = start(executor, doc)
    assert entered.wait(2)
    executor.source_error(false_abort=false_abort)
    state = finish(run)
    assert not state.completed and trace.count(hook_name) == 1
    assert "cleanup" not in trace


def test_nested_child_owned_after_outer_return_and_late_failure():
    doc = document("led")
    nested = Future()
    executor, trace = engine(doc, handlers={"led": lambda a, s: {
        "ok": True, "native_return": "original_success",
        "owned_children": [OwnedOperation(nested, ("TC",), "nested-child")],
    }})
    run = start(executor, doc)
    wait_until(lambda: "epilogue_sweep" in trace)
    nested.set_result({"ok": False, "error": "late_child"})
    state = finish(run)
    assert executor.outcome == "failed"
    assert next(r for r in state.action_results if r.get("native_return"))["ok"] is True
    assert "nested-child" in state.workflow.child_command_ids


def test_affected_stop_fences_future_occurrence_without_native_retry():
    doc = document("led", "step", "led")
    entered = []
    executor, trace = engine(doc)

    def first(action, state):
        entered.append(action.source_occurrence_id)
        executor.interrupt(control_id="stop")
        return {"ok": True, "command_id": "original-led"}
    executor._oem_handlers["led"] = first
    state = executor.execute(doc)
    assert executor.outcome == "interrupted"
    assert len(entered) == 1
    assert not any(r.get("source_marker") for r in state.action_results)
    assert "cleanup" not in trace


def test_authority_refusal_after_wait_never_captures_fresh_permission():
    doc = document("splid", "step")
    child = Future()
    allow = [True]
    checked = []

    def entry(identity, state):
        checked.append(identity)
        if not allow[0]:
            raise RuntimeError("original parent epoch invalid")
    executor, trace = engine(doc, handlers={"splid": lambda a, s: child}, entry=entry)
    run = start(executor, doc)
    wait_until(lambda: "epilogue_sweep" in trace)
    allow[0] = False
    child.set_result({"ok": True})
    state = finish(run)
    assert executor.outcome == "ambiguous"
    assert "epilogue_lid" not in trace and "cleanup" not in trace
    assert state.workflow.held_reason == "native_authority_unavailable"


def test_abort_after_wake_does_not_wake_again_or_continue():
    doc = document("step")
    executor, trace = engine(doc)
    original = executor._lifecycle_handlers["script_prologue"]

    def prologue(state):
        result = original(state)
        executor.request_control("pause", control_id="pause", mode="deferred")
        return result
    executor._lifecycle_handlers["script_prologue"] = prologue
    run = start(executor, doc)
    wait_until(lambda: executor._state and executor._state.workflow.gate == "deferred_pause")
    executor.request_control("wake", control_id="wake", gate_id="pause")
    wait_until(lambda: executor._wake_complete)
    executor.request_control("abort", control_id="abort")
    state = finish(run)
    assert executor.outcome == "interrupted" and trace.count("wake") == 1
    assert not any(r.get("source_marker") for r in state.action_results)


def test_stop_during_cleanup_preserves_entered_piece_without_finally_entry():
    doc = document("step")
    child = Future()
    cleanup_entered = Event()
    executor, trace = engine(doc)
    original = executor._lifecycle_handlers["script_prologue"]

    def prologue(state):
        result = original(state)
        executor.request_control("abort", control_id="abort")
        return result

    def cleanup(state):
        cleanup_entered.set()
        return child
    executor._lifecycle_handlers.update(script_prologue=prologue, cleanup=cleanup)
    run = start(executor, doc)
    assert cleanup_entered.wait(2)
    executor.interrupt(control_id="stop")
    assert run[0].is_alive()
    child.set_result({"ok": True, "cleanup_piece": "returned"})
    state = finish(run)
    assert executor.outcome == "interrupted"
    assert trace.count("script_finally") == 1
    assert next(r for r in state.action_results if r.get("cleanup_piece"))["ok"] is True


@pytest.mark.parametrize("raw,ok", [("not-an-integer", True), ("0", False), ("-1", False)])
def test_wait_source_parse_and_timer_error(raw, ok):
    doc = document("wait")
    action = replace(doc.stages[0].actions[0], params={"arguments": [raw]})
    doc = replace(doc, stages=(replace(doc.stages[0], actions=(action,)),))
    executor, _ = engine(doc)
    state = executor.execute(doc)
    assert state.completed is ok


def test_dry_run_nested_prepared_payload_is_json_serializable():
    import json
    doc = document("step", nested={"captured": [1, {"two": 2}]})
    state = ProtocolExecutor().execute(doc)
    json.dumps(state.to_payload())
    restored = ProtocolRuntimeState.from_payload(state.to_payload())
    assert restored.completed


def test_legacy_live_outcome_has_no_none_success_and_xml_is_never_live():
    action = ProtocolAction("a", "s", ProtocolActionKind.LED)
    doc = ProtocolDocument("legacy", stages=(ProtocolStage("s", actions=(action,)),))
    executor = ProtocolExecutor(dry_run=False, handlers={ProtocolActionKind.LED: lambda a, s: None})
    assert not executor.execute(doc).completed
    with pytest.raises(ValueError, match="generator"):
        executor.execute(replace(doc, metadata={"input_mode": "oem_xml"}))


def test_real_finite_d_e_bindings_consume_same_runtime_model():
    from bioxp.services.pipette_service import build_oem_pipette_handlers
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider
    selected = [("la", ["REAGENT_PLATE", "A1", "2"]), ("ms", ["target", "1"]),
                ("retip", []), ("led", ["999", "-1", "4"]), ("snapshot", [])]
    model = ProtocolSourceModel(
        trays={"REAGENT_PLATE": SourceTray("reagent", 1, [SourceWell("fluid", 5, 10) for _ in range(96)])},
        strips=[SourceTray(str(i), i, [SourceWell("target", 1, 10), SourceWell(None, 0, 10)]) for i in range(4)],
        tip_trays=[SourceTray("tips", 12, [SourceWell("Reuse", 0, 0)])],
    )
    doc = compile_prepared_oem_protocol({
        "protocol_id": "connected-offline", "metadata": {"source_model": model.to_payload()},
        "operations": [{"source_key": "repeated", "oem_opcode": op, "arguments": args} for op, args in selected],
    })
    calls = []
    def recorded(name, *values):
        calls.append((name, values))
        return {"ok": True}
    handlers = build_oem_pipette_handlers(
        before_native_entry=lambda identity, state: calls.append(("entry", identity)),
        script_move=lambda loc, row, a, s: recorded("move", loc, row),
        publish_location=lambda loc, well, a, s: recorded("location", loc, well),
        publish_tip_transition=lambda tray, wells, a, s: recorded("retip", tray, wells),
    )
    provider = Serial206OemInitializationProvider.__new__(Serial206OemInitializationProvider)
    native = provider.build_oem_native_handlers(
        settings={"CheckSnapTips": False, "DeckInspection": False, "StartMode": 0},
        execute_plan=lambda *args: pytest.fail("No native plan is selected in this test"),
        rgb_writer=lambda r, g, b: recorded("rgb", r, g, b),
    )
    executor, _ = engine(doc, handlers={**handlers, **native})
    state = executor.execute(doc)
    assert state.completed
    assert state.source_model.trays["REAGENT_PLATE"].wells[0].volume == 3
    assert not state.source_model.tip_trays[0].wells[0].empty
    assert ("move", (0, 0)) in calls and ("location", (0, 0)) in calls
    assert ("rgb", (255, 0, 4)) in calls
    assert next(r for r in state.action_results if r.get("source_model_updated"))["source_return"] == ["repeated"]


def test_pressure_history_and_sweep_are_source_model_not_occupancy():
    model = ProtocolSourceModel(tip_trays=[SourceTray("tips", 12, [SourceWell(None, 0, 0) for _ in range(96)])])
    assert model.sweep_locations(0) is None
    wells = model.tip_trays[0].wells
    for i in (0, 24, 48, 72, 12, 36, 60, 84, 1):
        wells[i].content = "rm"
    assert model.sweep_locations(0) == ["A1", "B1"]
    assert model.sweep_locations(0, True) == ["A1", "B1", "A2"]
    assert model.get_tip_tray_location(0) == 12
    model.add_pressure_base([1, 2, 3, 4])
    model.add_pressure_base([5, 6, 7, 8])
    assert model.pressure_baseline == [5, 6, 7, 8]
    assert model.pressure_history[0] == [0, 0, 0, 1, 5]
    assert ProtocolSourceModel.from_payload(model.to_payload()).to_payload() == model.to_payload()


def test_native_partial_exception_evidence_is_retained():
    doc = document("led")
    def failed(a, s):
        exc = RuntimeError("native")
        exc.oem_partial_results = [{"step_id": "original", "result": {"ok": True}}]
        exc.detail = {"command_id": "actual-child", "receipt": "original-receipt"}
        raise exc
    executor, _ = engine(doc, handlers={"led": failed})
    state = executor.execute(doc)
    result = next(r for r in state.action_results if r.get("error") == "source_child_failed")
    assert result["oem_partial_results"][0]["step_id"] == "original"
    assert result["detail"]["receipt"] == "original-receipt"


def test_run_job_source_early_return_never_enters_prologue():
    doc = document("step")
    executor, trace = engine(doc, overrides={"run_job": lambda s: {"ok": True, "source_pause_scripts": True}})
    run = start(executor, doc)
    wait_until(lambda: executor._state and executor._state.workflow.gate == "error_hold")
    assert "script_prologue" not in trace
    executor.request_control("abort", control_id="abort")
    assert not finish(run).completed
    assert "script_prologue" not in trace and "abort_true_finish" not in trace


def test_plain_native_and_dry_run_keep_non_oem_contract():
    action = ProtocolAction("a", "s", ProtocolActionKind.NOTE)
    doc = ProtocolDocument("native", stages=(ProtocolStage("s", actions=(action,)),))
    assert ProtocolExecutor().execute(doc).completed
    executor = ProtocolExecutor(dry_run=False, job_id="parent", before_native_entry=lambda i, s: None)
    state = executor.execute(doc)
    assert state.completed and executor.required_lifecycle(doc) == ()
    assert all(r.get("kind") != "lifecycle" for r in state.action_results)
