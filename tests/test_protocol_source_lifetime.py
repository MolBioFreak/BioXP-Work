"""Host-only lifetime qualification; all physical lifecycle work is doubled."""
from concurrent.futures import Future
from threading import Event
from types import SimpleNamespace

import pytest

from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider as Provider
from bioxp.protocols.executor import ProtocolExecutor, OwnedOperation
from tests.test_protocol_oem_lifecycle import document, engine, start, finish, wait_until


def provider():
    return Provider(primitives=SimpleNamespace())


def connected(doc, p, *, overrides=None, handlers=None, publish=None, returned=None):
    old, trace = engine(doc, overrides=overrides, handlers=handlers, publish=publish)
    executor = ProtocolExecutor(
        dry_run=False, job_id="canonical-parent", lifecycle_handlers=old._lifecycle_handlers,
        oem_handlers=handlers or {}, before_native_entry=lambda i, s: None,
        on_state_change=publish,
        source_script_begin=lambda s: p.wp8_source_script_begin(command_id=s.workflow.command_id),
        source_script_returned=returned or (lambda s: p.wp8_source_script_returned(command_id=s.workflow.command_id)),
    )
    return executor, trace


def test_pair_required_and_not_an_oem_hook():
    with pytest.raises(ValueError, match="pair"):
        ProtocolExecutor(source_script_begin=lambda s: {"ok": True})
    with pytest.raises(ValueError, match="Unknown lifecycle"):
        ProtocolExecutor(lifecycle_handlers={"source_script_returned": lambda s: {"ok": True}})


def test_provider_attempt_ownership_consumption_and_no_resignal():
    p = provider()
    assert p._wp8_stop_event.is_set()
    with pytest.raises(RuntimeError, match="mismatch"):
        p.wp8_source_script_returned(command_id="old")
    p.wp8_source_script_begin(command_id="one")
    assert not p._wp8_stop_event.is_set()
    p.wp8_source_script_begin(command_id="one")
    with pytest.raises(RuntimeError, match="overlap"):
        p.wp8_source_script_begin(command_id="two")
    p.wp8_source_script_returned(command_id="one")
    assert p.wp8_wait_stop("waitStop", {})["signaled"]
    p.wp8_source_script_returned(command_id="one")
    assert not p._wp8_stop_event.is_set()
    with pytest.raises(RuntimeError, match="already_returned"):
        p.wp8_source_script_begin(command_id="one")
    p.wp8_source_script_begin(command_id="two")
    with pytest.raises(RuntimeError, match="mismatch"):
        p.wp8_source_script_returned(command_id="one")
    assert not p._wp8_stop_event.is_set()
    p.wp8_source_script_returned(command_id="two")
    assert p._wp8_stop_event.is_set()


@pytest.mark.parametrize("boundary", ["script_prologue", "body", "epilogue_park"])
@pytest.mark.parametrize("diversion", ["interrupt", "recording_failure"])
def test_delayed_flattened_wrapper_blocks_signal_not_cancelled(boundary, diversion):
    p = provider()
    entered, pending = Event(), Future()
    doc = document("led") if boundary == "body" else document("step")
    def delayed(*args):
        assert not p._wp8_stop_event.is_set()
        entered.set()
        return pending
    fail_recording = Event()
    def publish(state):
        if fail_recording.is_set():
            raise OSError("fixture recording failure")
    executor, trace = connected(doc, p, handlers={"led": delayed} if boundary == "body" else None,
        overrides={boundary: delayed} if boundary != "body" else None, publish=publish)
    run = start(executor, doc)
    try:
        assert entered.wait(2)
        if diversion == "interrupt":
            executor.interrupt(control_id="addressed-stop")
        else:
            fail_recording.set()
            executor._publish()
        assert not p._wp8_stop_event.wait(.15)
        assert run[0].is_alive() and not pending.cancelled()
    finally:
        pending.set_result({"ok": True, "command_id": "original-wrapper-child"})
    state = finish(run)
    assert p._wp8_stop_event.is_set()
    assert "original-wrapper-child" in state.workflow.child_command_ids
    assert executor.outcome == ("interrupted" if diversion == "interrupt" else "ambiguous")
    assert "cleanup" not in trace and "script_finally" not in trace


def test_returned_wrapper_signals_while_native_child_custody_remains():
    p, entered, child = provider(), Event(), Future()
    def prologue(state):
        entered.set()
        return {"ok": True, "owned_children": [OwnedOperation(child, ("General",), "native-child")]}
    doc = document("step")
    executor, trace = connected(doc, p, overrides={"script_prologue": prologue})
    run = start(executor, doc)
    try:
        assert entered.wait(2)
        wait_until(lambda: "native-child" in executor._state.workflow.child_command_ids)
        executor.interrupt(control_id="external-stop")
        assert p._wp8_stop_event.wait(2)
        assert run[0].is_alive() and not child.done()
        assert not executor._state.completed
        assert p.wp8_wait_stop("waitStop", {})["signaled"]
        executor._return_source()
        assert not p._wp8_stop_event.is_set()
    finally:
        child.set_result({"ok": False, "uncertain": True, "native_evidence": "retained"})
    state = finish(run)
    assert executor.outcome == "ambiguous"
    assert any(r.get("native_evidence") == "retained" for r in state.action_results)
    assert "cleanup" not in trace and not p._wp8_stop_event.is_set()


@pytest.mark.parametrize("failure", ["failed", "hold", "control"])
def test_no_script_begin_or_return_before_successful_run_job(failure):
    p = provider()
    def run_job(state):
        if failure == "control":
            executor.request_control("safe_stop", control_id="before-script")
        return {"ok": failure != "failed", "source_pause_scripts": failure == "hold"}
    doc = document("step")
    executor, trace = connected(doc, p, overrides={"run_job": run_job})
    run = start(executor, doc)
    if failure == "hold":
        wait_until(lambda: executor._state.workflow.gate == "error_hold")
        executor.interrupt(control_id="stop-before-script")
    finish(run)
    assert p._wp8_source_script_owner is None
    assert not p._wp8_source_script_returned
    assert "script_prologue" not in trace and "cleanup" not in trace


def test_failed_return_notification_is_once_ambiguous_and_no_cleanup():
    p, calls = provider(), []
    def returned(state):
        calls.append(state.workflow.command_id)
        raise OSError("host notification failed")
    doc = document("step")
    executor, trace = connected(doc, p, returned=returned)
    state = executor.execute(doc)
    assert calls == ["canonical-parent"]
    assert not state.completed and executor.outcome == "ambiguous"
    assert not p._wp8_stop_event.is_set()
    assert "cleanup" not in trace and "script_finally" not in trace


def test_normal_begin_after_run_job_and_return_before_finally():
    p, observations = provider(), []
    def run_job(state):
        observations.append(("run_job", p._wp8_stop_event.is_set(), p._wp8_source_script_owner))
        return {"ok": True}
    def prologue(state):
        observations.append(("prologue", p._wp8_stop_event.is_set(), p._wp8_source_script_owner))
        return {"ok": True}
    def final(state):
        observations.append(("finally", p._wp8_stop_event.is_set(), p._wp8_source_script_owner))
        return {"ok": True, "offline_double": True}
    doc = document("step")
    executor, trace = connected(doc, p, overrides={"run_job": run_job, "script_prologue": prologue, "script_finally": final})
    assert executor.execute(doc).completed
    assert observations == [("run_job", True, None), ("prologue", False, "canonical-parent"), ("finally", True, "canonical-parent")]
    assert "cleanup" not in trace


@pytest.mark.parametrize("control", ["safe_stop", "abort"])
@pytest.mark.parametrize("notification_fails", [False, True])
def test_cleanup_double_requires_successful_source_return(control, notification_fails):
    p, calls = provider(), []
    def returned(state):
        calls.append("returned")
        if notification_fails:
            return {"ok": False}
        return p.wp8_source_script_returned(command_id=state.workflow.command_id)
    def cleanup(state):
        assert p._wp8_source_script_returned
        assert p.wp8_wait_stop("waitStop", {})["signaled"]
        calls.append("cleanup")
        return {"ok": True, "offline_cleanup_double": True}
    doc = document("delaypoint", delayed_start=True)
    executor, trace = connected(doc, p, overrides={"cleanup": cleanup}, returned=returned)
    run = start(executor, doc)
    wait_until(lambda: executor._state is not None and executor._state.workflow.gate == "delaypoint")
    executor.request_control(control, control_id="source-control")
    finish(run)
    assert calls == (["returned"] if notification_fails else ["returned", "cleanup"])
    assert executor.outcome == ("ambiguous" if notification_fails else "interrupted")
    assert not p._wp8_stop_event.is_set()


def test_behavioral_initial_event_is_cleared_at_actual_prologue_entry():
    # Baseline-compatible constructor: absence of the new interface must not
    # short-circuit this negative control with an unexpected-keyword error.
    import inspect
    p, observations = provider(), []
    doc = document("step")
    old, _ = engine(doc, overrides={"script_prologue": lambda s: observations.append(p._wp8_stop_event.is_set()) or {"ok": True}})
    kwargs = {}
    if "source_script_begin" in inspect.signature(ProtocolExecutor).parameters:
        kwargs = {"source_script_begin": lambda s: p.wp8_source_script_begin(command_id=s.workflow.command_id),
                  "source_script_returned": lambda s: p.wp8_source_script_returned(command_id=s.workflow.command_id)}
    executor = ProtocolExecutor(dry_run=False, job_id="parent", lifecycle_handlers=old._lifecycle_handlers,
        before_native_entry=lambda i, s: None, **kwargs)
    assert executor.execute(doc).completed
    assert observations == [False], "construction-time signal is not source-return proof"


def test_late_abort_does_not_reenter_source_exit_or_release_native_custody():
    p, child, cleanup_entered = provider(), Future(), Event()
    def park(state):
        return {"ok": True, "owned_children": [OwnedOperation(child, ("General",), "park-child")]}
    def cleanup(state):
        assert child.done()
        assert p.wp8_wait_stop("waitStop", {})["signaled"]
        cleanup_entered.set()
        return {"ok": True, "offline_cleanup_double": True}
    doc = document("step")
    executor, trace = connected(doc, p, overrides={"epilogue_park": park, "cleanup": cleanup})
    run = start(executor, doc)
    try:
        wait_until(lambda: p._wp8_source_script_returned)
        executor.request_control("abort", control_id="late-abort")
        wait_until(lambda: "abort_true_finish" in trace)
        assert "safe_stop_exit" not in trace
        assert run[0].is_alive() and not cleanup_entered.is_set()
        assert p._wp8_stop_event.is_set()
    finally:
        child.set_result({"ok": True, "native_evidence": "park-child-return"})
    state = finish(run)
    assert cleanup_entered.is_set() and executor.outcome == "interrupted"
    assert not p._wp8_stop_event.is_set()
    assert any(r.get("native_evidence") == "park-child-return" for r in state.action_results)
