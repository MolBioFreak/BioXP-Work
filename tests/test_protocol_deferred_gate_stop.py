"""Deferred source stop-break; real executor/lifetime, explicit lifecycle doubles."""
import pytest

from tests.test_protocol_oem_lifecycle import document, engine, start, finish, wait_until


@pytest.mark.parametrize("control", ["abort", "safe_stop", "continue"])
def test_reached_deferred_gate_retains_stop_break_or_continues(control):
    doc = document("delaypoint", "led", delayed_start=True)
    bodies = []

    def body(action, state):
        bodies.append(action.action_id)
        return {"ok": True}

    executor, trace = engine(doc, handlers={"led": body})
    run = start(executor, doc)
    try:
        wait_until(lambda: executor._state and executor._state.workflow.gate == "delaypoint")
        executor.request_control("pause", control_id="defer", mode="deferred")
        wait_until(lambda: executor._state.workflow.gate == "deferred_pause")
        assert executor._state.workflow.reached_control_id == "defer"
        assert not bodies
        assert trace.count("deferred_pause_enter") == 1
        if control == "continue":
            # Wake is deliberately a lifecycle double here, not wake acceptance.
            executor.request_control("wake", control_id="wake", gate_id="defer")
            wait_until(lambda: executor._wake_complete)
            assert not bodies
            executor.request_control("continue", control_id="exit", gate="deferred_pause", gate_id="defer")
        else:
            executor.request_control(control, control_id="exit")
        state = finish(run)
    finally:
        if run[0].is_alive():
            executor.interrupt(control_id="test-teardown")
            finish(run)

    assert trace.count("script_finally") == 1
    assert executor._source_returned and executor._source_host_finalized
    assert "safe_stop_exit" not in trace
    assert state.workflow.gate is None
    assert state.workflow.reached_control_id == "exit"
    if control == "continue":
        assert state.completed and executor.outcome == "completed"
        assert len(bodies) == 1 and trace.count("wake") == 1
        assert "epilogue_lid" in trace and "cleanup" not in trace
        assert "safe_stop_request" not in trace and "abort_true_prefix" not in trace
    else:
        assert not state.completed and executor.outcome == "interrupted"
        assert not bodies and "wake" not in trace and "epilogue_lid" not in trace
        assert trace.count("cleanup") == 1
        assert trace.index("script_finally") < trace.index("cleanup")
        request = "abort_true_prefix" if control == "abort" else "safe_stop_request"
        assert trace.count(request) == 1
        assert trace.index(request) < trace.index("script_finally")
        assert trace.count("abort_true_finish") == (1 if control == "abort" else 0)
