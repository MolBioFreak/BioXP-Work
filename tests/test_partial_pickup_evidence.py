"""Critical pickup outcomes survive the actual SQLite terminal boundary.

Injected source-step failures are not replays of the historical P3 fault. They
exercise its demonstrated evidence-loss boundary without controller I/O.
"""
from contextlib import nullcontext
import json
import sqlite3

import pytest

from bioxp.manual_pipetting import manual_physical_plan
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.operator_command_plane import OperatorCommandStore
from bioxp.operator_controls import _bounded_json, _workflow_terminal_result
from bioxp.pipette.oem_calibration import CalibrationExecutionError, _Sequence
from bioxp.protocols.operational_results import finite_child_outcomes
from tests.test_protocol_oem_provider_bindings import rig  # noqa: F401


def pickup_result(failed_step):
    seq = _Sequence("ControlLib.btnLoadTip_Click:1324-1368")
    seq.result.update(tray=1, well="A10", kept_channels=[2], released_channels=[0, 1, 3],
                      release_lift_steps=20157)
    # Realistic shape but deliberately oversized synthetic wire attachment.
    seq.step("moveTo", lambda: {"ok": True, "provenance": {"frames": ["wire-frame" * 100] * 200}})
    seq.step("releaseUnneededTips", lambda: {"ok": True, "channels": [0, 1, 3], "sends": [
        {"channel": ch, "result": {"ok": True, "completion_verified": True,
         "controller_acknowledged": True, "delivery_verified": True, "ascii_command": "E1R"}}
        for ch in (0, 1, 3)]})

    def fail():
        exc = RuntimeError("injected exact terminal cause")
        exc.evidence = {"failure_evidence": [{"operation": "sourceHomeZ" if failed_step == "MoveZHome" else "sourceTipState",
            "exception_type": "RuntimeError", "exception_message": str(exc),
            "result": {"ok": False, "home": {"ok": False, "source_return_code": 7},
                       "controller_terminal_state_verified": False}}]}
        raise exc

    try:
        seq.step("MoveZHome", fail if failed_step == "MoveZHome" else lambda: {"ok": True, "source_return": 7})
        seq.step("TipDirty=false", fail if failed_step == "TipDirty=false" else lambda: {"ok": True})
    except CalibrationExecutionError as exc:
        return {**exc.evidence, "error": str(exc), "delivery_attempted": True}
    return {**seq.result, "rack_tips_removed": [57], "delivery_attempted": True}


@pytest.mark.parametrize("failed_step", ["MoveZHome", "TipDirty=false", None])
def test_pickup_terminal_failure_and_prefix_survive_cold_sqlite(rig, tmp_path, failed_step):
    provider, *_ = rig
    initial = OEMRuntimeStore(tmp_path)
    initial.close()
    store = OperatorCommandStore(tmp_path)
    stamps = provider.deck_owner_authority_stamps()
    store.bind_deck_owner_authority_reader(lambda: stamps, scope=nullcontext)
    store.bind_workflow_dispatcher(lambda command: None)
    store.publish_deck_owner_state(source_operation="pipette_owner", source_command_id="fixture-tip-state",
        updates={"tip_loaded": False, "tip_dirty": False, "tip_location": -1}, **stamps)
    store.admit_workflow(command_id="parent", idempotency_key="parent", plan_fingerprint="fixture",
        requested_inputs={"bundle": {}}, ownership_generation=1,
        resources=("axis:x", "axis:y", "axis:z", "pipette"), board_epochs={})
    assert store.claim_next()["command_id"] == "parent"
    assert store._renew_owner(lease_seconds=600)
    plan = manual_physical_plan({"operation": "load_tip", "tray": 1, "well": "A10", "lift_z": True,
                                "keep_channels": (2,), "release_lift_mm": 10.0})
    admission = {"ownership_generation": 1, "serial206_initialization_provider": {
        "x_authority": {"current_board_lifecycle_generation": 1}, "board4_authority": {"active_board_epoch": 1}}}
    with store.workflow_context("parent", source_occurrence_id="pickup"):
        cid = store.admit_internal_wp8_operation(plan["operation"], inputs={}, state=admission,
            idempotency_key="pickup", prepared_plan=plan)["command_id"]
    claimed = store.claim_next()
    assert claimed["command_id"] == cid
    original = pickup_result(failed_step)
    assert len(json.dumps(original)) > 131072
    try:
        store.persist_wp8_plan(cid, plan, authority_stamps=stamps)
        store.terminalize_wp8_child(cid, 0, state="ambiguous" if failed_step else "completed", result=original,
                                    dispatch_attempt_id=claimed["dispatch_attempt_id"])
        child_outcomes = finite_child_outcomes(cid, store.wp8_operation_evidence(cid))
        store.finish(cid, status="ambiguous" if failed_step else "completed",
                     payload={"response": {"ok": not failed_step, "child_outcomes": child_outcomes}}, claimed=claimed)
        store.finish_workflow("parent", status="ambiguous" if failed_step else "completed", payload={},
                              lifecycle_settled=not failed_step)
        path = store.path
    finally:
        store.stop()
        store.connection.close()
    with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as reader:
        raw = reader.execute("SELECT terminal_evidence_json FROM operator_plane_wp8_children WHERE command_id=?", (cid,)).fetchone()[0]
        result = json.loads(raw)["result"]
        terminal = json.loads(reader.execute("SELECT terminal_json FROM operator_plane_commands WHERE command_id=?", (cid,)).fetchone()[0])
    assert result["kind"] == "manual_load_tip" and "bounded" not in result
    assert len(raw) < 10000
    assert result["ok"] is (failed_step is None)
    assert result["steps"][0] == {"operation": "moveTo", "result": {"ok": True}}
    assert [s["channel"] for s in result["steps"][1]["result"]["sends"]] == [0, 1, 3]
    projected = _workflow_terminal_result({"status": "ambiguous" if failed_step else "completed", "command_id": cid,
                                          "terminal_evidence": terminal})
    assert projected["child_outcomes"][0]["steps"] == result["steps"]
    if failed_step:
        assert result["steps"][-1]["operation"] == failed_step
        assert "injected exact terminal cause" in result["steps"][-1]["error"]
        assert result["steps"][-1]["evidence"]["failure_evidence"][0]["exception_message"] == "injected exact terminal cause"
    else:
        assert result["rack_tips_removed"] == [57]
        assert result["steps"][-2]["result"]["source_return"] == 7
    # Even a deliberately tiny diagnostic budget must not replace the typed outcome.
    assert _bounded_json(result, 10) == result
    assert original["steps"][0]["result"]["provenance"]["frames"]  # no mutation of native input
