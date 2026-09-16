"""Executor owner qualification; authentic provider lifetime, physical leaves doubled."""
from concurrent.futures import Future
from dataclasses import replace
from threading import Event, get_ident

import pytest

from bioxp.protocols.executor import ProtocolExecutor, OwnedOperation, ALL_DOMAINS
from bioxp.protocols.runtime_state import ProtocolRuntimeState
from tests.test_protocol_oem_lifecycle import document, engine, start, finish, wait_until
from tests.test_protocol_source_lifetime import provider, connected


def test_oem_preflight_requires_actual_lifetime_pair_but_dry_run_does_not():
    doc = document("step")
    old, _ = engine(doc)
    executor = ProtocolExecutor(dry_run=False, job_id="parent", lifecycle_handlers=old._lifecycle_handlers,
                                before_native_entry=lambda i, s: None)
    assert executor.preflight(doc)["missing"] == ["source_script_begin", "source_script_returned"]
    with pytest.raises(ValueError, match="source_script_begin"):
        executor.execute(doc)
    assert ProtocolExecutor().execute(doc).completed
    assert {"safe_stop_request", "source_error_request"} <= set(old.required_lifecycle(doc))


@pytest.mark.parametrize("control,hook", [("abort", "abort_true_prefix"), ("safe_stop", "safe_stop_request")])
def test_delivered_stop_flag_not_accepted_intent_during_shutdown(control, hook):
    shutdown, release, native = Event(), Event(), Event()
    doc = document("splid", "step")
    def thermal(action, state):
        native.set()
        wait_until(executor.source_stopped)
        return {"ok": True}
    def prefix(state):
        assert not executor.source_stopped()
        shutdown.set()
        assert release.wait(3)
        assert not executor.source_stopped()
        return {"ok": True}
    executor, trace = engine(doc, handlers={"splid": thermal}, overrides={hook: prefix})
    run = start(executor, doc)
    try:
        assert native.wait(2)
        executor.request_control(control, control_id="stop-intent")
        assert shutdown.wait(2)
        assert not executor.source_stopped()
        assert not executor._source_cancelled
    finally:
        release.set()
    finish(run)
    assert executor.source_stopped() and executor.outcome == "interrupted"
    assert trace.count("script_finally") == 1


@pytest.mark.parametrize("control,hook", [("abort", "abort_true_prefix"), ("safe_stop", "safe_stop_request")])
def test_real_pipette_ldtip_stop_consumer_observes_shutdown_before_stop(control, hook):
    from bioxp.services.pipette_service import build_oem_pipette_handlers
    from tests.test_protocol_oem_pipette_composites import composite_bindings
    from tests.test_protocol_oem_bindings import document as native_document
    args, _, transport, entries, effects, facts = composite_bindings()
    entering, shutdown, probed, release = Event(), Event(), Event(), Event()
    observed = []
    def stopped(action, state):
        assert state is executor._state
        entering.set()
        assert shutdown.wait(3)
        observed.append(executor.source_stopped())
        probed.set()
        wait_until(executor.source_stopped)
        observed.append(executor.source_stopped())
        return executor.source_stopped()
    args["source_bindings"] = replace(args["source_bindings"], stopped=stopped)
    def guard(identity, state):
        assert state is executor._state
        executor._entry(identity)
        entries.append(identity)
    def pipette(name, operation, action, state, identity):
        assert state is executor._state and identity == entries[-1]
        return operation(transport)
    args.update(before_native_entry=guard, pipette_call=pipette)
    actual_handler = build_oem_pipette_handlers(**args)["ldtip"]
    def prefix(state):
        shutdown.set()
        assert release.wait(3)
        return {"ok": True}
    doc = native_document("ldtip", ["T50", "False", "0"])
    executor, _ = engine(doc, handlers={"ldtip": actual_handler}, overrides={hook: prefix})
    run = start(executor, doc)
    try:
        assert entering.wait(2)
        executor.request_control(control, control_id="source-stop")
        assert probed.wait(2)
        assert observed == [False] and not executor._source_cancelled
    finally:
        release.set()
    state = finish(run)
    assert observed == [False, True]
    assert transport.calls == [("pressure",)]
    assert any(row.get("native_results") for row in state.action_results)


def test_request_inline_with_every_body_worker_occupied():
    doc = document("led")
    blocked = [Event() for _ in ALL_DOMAINS]
    release, ready = Event(), Event()
    owner_ids = []
    def child(index):
        blocked[index].set()
        assert release.wait(4)
        return {"ok": True, "child": index}
    def body(action, state):
        for i in range(len(blocked)):
            executor.start_child("saturated:" + str(i), lambda i=i: child(i), domains=("TC",))
        assert all(event.wait(2) for event in blocked)
        ready.set()
        assert release.wait(4)
        return {"ok": True}
    def prefix(state):
        owner_ids.append(get_ident())
        assert not executor._source_return_notified
        release.set()
        return {"ok": True}
    executor, trace = engine(doc, handlers={"led": body}, overrides={"abort_false": prefix})
    run = start(executor, doc)
    try:
        assert ready.wait(3)
        executor.source_error(false_abort=True)
        state = finish(run)
    finally:
        release.set()
    assert owner_ids == [run[0].ident]
    assert len([r for r in state.action_results if "child" in r]) == len(blocked)
    assert "cleanup" not in trace and executor.outcome == "failed"


def test_board_error_precedes_generic_error_and_releases_retained_child():
    doc = document("led")
    pending = Future()
    def body(action, state):
        return {"ok": False, "source_board_error_event": True, "source_error_event": True,
                "owned_children": [OwnedOperation(pending, ("TC",), "actual-child")]}
    def false_abort(state):
        assert "actual-child" in state.workflow.child_command_ids
        assert not pending.done()
        pending.set_result({"ok": False, "native_error": "board"})
        return {"ok": True}
    executor, trace = engine(doc, handlers={"led": body}, overrides={"abort_false": false_abort})
    state = executor.execute(doc)
    assert executor._termination == "abort_false" and executor.outcome == "failed"
    assert not set(trace).intersection({"source_error_request", "source_error", "cleanup"})
    assert any(r.get("native_error") == "board" for r in state.action_results)


def test_error_request_then_real_wrapper_return_then_host_finally_then_tail_join():
    p, wrapper, child = provider(), Future(), Future()
    entered, requested, tail = Event(), Event(), Event()
    doc = document("led")
    def body(action, state):
        entered.set()
        return wrapper
    def request(state):
        assert not p._wp8_stop_event.is_set()
        requested.set()
        return executor.cancel_source(state)
    def cleanup_error(state):
        assert child.done() and executor._source_host_finalized
        assert p.wp8_wait_stop("waitStop", {})["signaled"]
        tail.set()
        return {"ok": True}
    executor, trace = connected(doc, p, handlers={"led": body}, overrides={
        "source_error_request": request, "source_error": cleanup_error})
    run = start(executor, doc)
    try:
        assert entered.wait(2)
        executor.source_error()
        assert requested.wait(2)
        assert not p._wp8_stop_event.is_set()
        wrapper.set_result({"ok": True, "owned_children": [OwnedOperation(child, ("TC",), "late-native")]})
        wait_until(lambda: executor._source_host_finalized)
        assert not tail.is_set() and run[0].is_alive()
    finally:
        if not wrapper.done(): wrapper.set_result({"ok": True})
        child.set_result({"ok": True})
    finish(run)
    assert tail.is_set() and trace.count("script_finally") == 1
    assert "cleanup" not in trace


def test_late_false_abort_is_serviced_during_actual_return_drain():
    p, wrapper = provider(), Future()
    entered, first = Event(), Event()
    doc = document("led")
    def body(action, state):
        entered.set()
        return wrapper
    def request(state):
        first.set()
        return executor.cancel_source(state)
    def false_abort(state):
        assert not p._wp8_stop_event.is_set()
        wrapper.set_result({"ok": True, "native_evidence": "returned-after-request"})
        return executor.cancel_source(state)
    executor, trace = connected(doc, p, handlers={"led": body}, overrides={
        "source_error_request": request, "abort_false": false_abort})
    run = start(executor, doc)
    try:
        assert entered.wait(2)
        executor.source_error()
        assert first.wait(2)
        executor.source_error(false_abort=True)
        state = finish(run)
    finally:
        if not wrapper.done(): wrapper.set_result({"ok": True})
    assert any(r.get("native_evidence") for r in state.action_results)
    assert "source_error" not in trace and "cleanup" not in trace


def test_late_false_abort_can_release_child_during_error_tail_conflict_join():
    p, child = provider(), Future()
    doc = document("led")
    def body(action, state):
        return {"ok": False, "source_error_event": True,
                "owned_children": [OwnedOperation(child, ("TC",), "held-native")]}
    def false_abort(state):
        assert p._wp8_source_script_returned and not child.done()
        child.set_result({"ok": False, "native_error": "released-by-abort"})
        return executor.cancel_source(state)
    executor, trace = connected(doc, p, handlers={"led": body}, overrides={"abort_false": false_abort})
    run = start(executor, doc)
    try:
        wait_until(lambda: executor._source_host_finalized)
        assert run[0].is_alive()
        executor.source_error(false_abort=True)
        state = finish(run)
    finally:
        if not child.done(): child.set_result({"ok": True})
    assert "source_error" not in trace and "cleanup" not in trace
    assert any(row.get("native_error") == "released-by-abort" for row in state.action_results)


def test_deferred_request_releases_delaypoint_without_continue_and_requires_wake():
    doc = document("delaypoint", "step", delayed_start=True)
    executor, trace = engine(doc)
    run = start(executor, doc)
    wait_until(lambda: executor._state and executor._state.workflow.gate == "delaypoint")
    executor.request_control("pause", control_id="defer", mode="deferred")
    wait_until(lambda: executor._state.workflow.gate == "deferred_pause")
    row = next(r for r in executor._state.action_results if r.get("host_exit"))
    assert row["host_exit"] == "deferred_request"
    assert not any(request["action"] == "continue" for request in executor._controls.values())
    executor.request_control("wake", control_id="wake", gate_id="defer")
    wait_until(lambda: executor._wake_complete)
    executor.request_control("continue", control_id="continue", gate="deferred_pause", gate_id="defer")
    assert finish(run).completed


def test_cancel_source_identity_and_no_early_return_before_home_failure():
    p, home, release = provider(), Event(), Event()
    doc = document("delaypoint", delayed_start=True)
    def exit_(state):
        forged = ProtocolRuntimeState.from_payload(state.to_payload())
        with pytest.raises(ValueError, match="attempt"):
            executor.cancel_source(forged)
        with pytest.raises(ValueError, match="precedes"):
            executor.finalize_source_host(state)
        result = executor.cancel_source(state)
        assert result == {"ok": True, "source_stop_scripts": True}
        assert executor.cancel_source(state) == result
        assert not p._wp8_stop_event.is_set()
        home.set()
        assert release.wait(3)
        return {"ok": False, "home_evidence": "failed-after-cancellation"}
    executor, trace = connected(doc, p, overrides={"safe_stop_exit": exit_})
    run = start(executor, doc)
    try:
        wait_until(lambda: executor._state and executor._state.workflow.gate == "delaypoint")
        with pytest.raises(ValueError, match="termination"):
            executor.cancel_source(executor._state)
        executor.request_control("safe_stop", control_id="safe")
        assert home.wait(2)
        assert not p._wp8_stop_event.is_set()
    finally:
        release.set()
    state = finish(run)
    assert not state.completed and "cleanup" not in trace
    assert trace.count("script_finally") == 1
    assert p._wp8_source_script_returned
    assert any(r.get("home_evidence") for r in state.action_results)
    with pytest.raises(ValueError, match="attempt"):
        executor.cancel_source(state)


@pytest.mark.parametrize("value", [{"ok": False, "failure_evidence": "host-finally"}, None])
def test_failed_host_finalization_is_once_unknown_and_keeps_source_signal(value):
    p, calls = provider(), []
    def final(state):
        calls.append(state)
        assert p._wp8_source_script_returned
        return value
    doc = document("step")
    executor, trace = connected(doc, p, overrides={"script_finally": final})
    state = executor.execute(doc)
    executor._finalize_script_host()
    assert len(calls) == 1 and executor.outcome == "ambiguous" and not state.completed
    assert p._wp8_stop_event.is_set() and "cleanup" not in trace


@pytest.mark.parametrize("unlocked", [True, False, "true", None])
def test_unlock_flag_updates_only_host_status_without_erasing_failed_evidence(unlocked):
    from bioxp.protocols.runtime_state import ProtocolWorkflowState
    doc = document("step")
    executor, _ = engine(doc)
    state = ProtocolRuntimeState.from_document(doc, dry_run=False, job_id=executor.job_id)
    state.workflow = ProtocolWorkflowState(command_id=executor.job_id, gate="ordinary_pause", gate_id="gate")
    state.source_model.logical_tip_present = state.source_model.carried_plate_present = True
    state.paused = True
    executor._state = state
    executor._pause = ("ordinary", "pause")
    result = {"kind": "lifecycle", "hook": "source_error"}
    value = {"ok": False, "source_unlock_completed": unlocked, "source_children": [
        {"operation": "unlatch", "ok": unlocked is True}, {"operation": "led", "ok": False}]}
    executor._consume(value, result)
    assert result == {"kind": "lifecycle", "hook": "source_error", **value}
    assert executor._failed and state.source_model.logical_tip_present and state.source_model.carried_plate_present
    assert executor._source_stop_requested is (unlocked is True)
    assert state.paused is (unlocked is not True)
    assert (state.workflow.gate is None) is (unlocked is True)


def test_invalid_async_request_retains_entered_work_and_does_not_replay():
    p, pending = provider(), Future()
    calls = []
    def prefix(state):
        calls.append(state)
        return pending
    doc = document("delaypoint", delayed_start=True)
    executor, trace = connected(doc, p, overrides={"abort_true_prefix": prefix})
    run = start(executor, doc)
    try:
        wait_until(lambda: executor._state and executor._state.workflow.gate == "delaypoint")
        executor.request_control("abort", control_id="abort")
        wait_until(lambda: executor._unknown)
        assert run[0].is_alive() and not pending.cancelled()
        assert not p._wp8_stop_event.is_set()
    finally:
        pending.set_result({"ok": True, "request_native_evidence": "retained"})
    state = finish(run)
    assert len(calls) == 1 and executor.outcome == "ambiguous"
    assert any(r.get("request_native_evidence") for r in state.action_results)
    assert "cleanup" not in trace and trace.count("script_finally") == 1
