"""Networkless fault injection through the real finite executor/plane/store."""
from contextlib import nullcontext
from types import SimpleNamespace
import json

import pytest

from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.operator_command_plane import OperatorCommandStore, OperatorCommandPlane
from bioxp.oem_deck_movement import DeckExecutionFailure, make_wp8_operation_executor


@pytest.fixture
def rig(tmp_path):
    runtime = OEMRuntimeStore(tmp_path)
    runtime.close()
    store = OperatorCommandStore(tmp_path)
    stamps = {"ownership_generation": 1, "board_epoch_4": 1, "board_epoch_5": 1}
    store.bind_deck_owner_authority_reader(lambda: stamps, scope=nullcontext)
    state = {"ownership_generation": 1, "serial206_initialization_provider": {
        "x_authority": {"current_board_lifecycle_generation": 1},
        "board4_authority": {"active_board_epoch": 1}}}
    provider = SimpleNamespace(deck_owner_authority_stamps=lambda: stamps,
        wp8_operation_machine_state=lambda *args: {})
    plane = object.__new__(OperatorCommandPlane)
    plane.store = store
    plane.machine_state_provider = lambda: state
    plane.app = SimpleNamespace(state=SimpleNamespace(oem_deck_provider=provider))
    plane.app.state.oem_wp8_operation_executor = make_wp8_operation_executor(
        provider_getter=lambda: provider, command_store=store)
    yield store, plane, provider, state
    store.stop()


def run_failure(rig, failure):
    store, plane, provider, state = rig
    def dispatch(*args, **kwargs):
        if isinstance(failure, Exception):
            raise failure
        return failure
    provider.execute_wp8_child = dispatch
    admitted = store.admit_internal_wp8_operation("manual_load_tip",
        inputs={"tray": 1, "well": "A10", "overpress": False, "lift_z": True}, state=state)
    cid = admitted["command_id"]
    claimed = store.claim_next()
    assert claimed["command_id"] == cid
    assert store._renew_owner(lease_seconds=600)
    plane._dispatch_one(claimed)
    return cid


def pickup(failure):
    return {"ok": False, "delivery_attempted": True,
        "source_anchor": "ControlLib.btnLoadTip_Click:1324-1368",
        "steps": [{"operation": "moveTo", "result": {"ok": True, "delivery_attempted": True}},
                  {"operation": "release", "result": {"ok": True, "delivery_verified": True}},
                  {"operation": "MoveZHome", "result": failure}]}


@pytest.mark.parametrize("failure,expected", [
    (pickup({"ok": False, "delivery_attempted": False}), "failed"),
    (pickup({"ok": False, "tx_ok": False, "delivery_verified": False}), "failed"),
    (pickup({"ok": False, "delivery_attempted": True}), "ambiguous"),
    (pickup({"ok": False, "error": "native return unknown"}), "ambiguous"),
    (DeckExecutionFailure("known controller failure", delivery_attempted=True, known_failure=True), "failed"),
    (DeckExecutionFailure("unknown completion", delivery_attempted=True), "ambiguous"),
])
def test_failure_classification_and_recovery_agree(rig, failure, expected):
    store, *_ = rig
    cid = run_failure(rig, failure)
    receipt = store.get_command(cid)
    assert receipt["status"] == expected
    child = store.wp8_operation_evidence(cid)["children"][0]
    assert child["terminal_state"] == expected
    recovery = store.recovery()
    assert recovery["hold"] is (expected == "ambiguous")
    assert (cid in recovery["outcome_unknown_command_ids"]) is (expected == "ambiguous")
    result = json.loads(store.wp8_operation_evidence(cid)["operation"]["terminal_result_json"])
    assert result["outcome_unknown"] is (expected == "ambiguous")
    if isinstance(failure, dict):
        saved = json.loads(child["terminal_evidence_json"])["result"]
        assert saved["steps"][:2] == failure["steps"][:2]


@pytest.mark.parametrize("failure,expected", [
    ({"ok": False, "delivery_attempted": False}, "failed"),
    ({"ok": False, "delivery_attempted": True, "kind": "cavro_application",
      "events": [{"status": "completed"}, {"status": "failed"}]}, "failed"),
    (DeckExecutionFailure("pre-send refusal", delivery_attempted=False), "failed"),
    (DeckExecutionFailure("known failure", delivery_attempted=True, known_failure=True), "failed"),
    (DeckExecutionFailure("unknown return", delivery_attempted=True), "ambiguous"),
])
def test_mov_failure_does_not_borrow_prior_stage_sends(failure, expected):
    from bioxp.oem_deck_movement import execute_mov_execution, MovExecutionPlan, MovExecutionStep
    terminals = []
    # Unit seam records native-stage classification; the tests above exercise
    # real SQLite ownership, terminalization, receipt and recovery receiving.
    ledger = SimpleNamespace(
        persist_mov_execution_plan=lambda *a: None,
        record_delivery_attempt=lambda *a, **kw: {"dispatch_attempt_id": "offline"},
        terminalize_mov_execution_stage=lambda *a, **kw: terminals.append(kw))
    prior = {"ok": True, "delivery_attempted": True,
             "controller_command_acknowledged": True, "controller_completion_verified": True}
    def fail():
        if isinstance(failure, Exception):
            raise failure
        return failure
    provider = SimpleNamespace(first=lambda: prior, second=fail)
    plan = MovExecutionPlan(script_line=1, source_branch="offline-fault", authority_digest="offline",
        destination=6, plate_name=0, well_id=0, well_source="offline", source_hazards=(),
        machine_state_updates={}, destination_translation={}, steps=(
            MovExecutionStep(0, "first", "offline", {}),
            MovExecutionStep(1, "second", "offline", {})))
    result = execute_mov_execution("offline", plan, provider=provider, command_store=ledger)
    assert result["ok"] is False
    assert result["delivery_attempted"] is True  # Actual preceding completed effect.
    assert result["ambiguity_state"] == expected
    assert terminals[-1]["state"] == expected
    assert result["provider_results"][0]["ok"] is True


def home(rig, action, status="completed"):
    store, _, _, state = rig
    admitted = store.admit_command({"action_id": action, "inputs": {},
        "idempotency_key": action + str(store.connection.total_changes),
        "schema_version": "bioxp.operator_action_request.v2",
        "expected_ownership_generation": 1}, state=state)
    claimed = store.claim_next()
    assert claimed["command_id"] == admitted["command_id"]
    store.finish(claimed["command_id"], status=status, payload={"ok": status == "completed"}, claimed=claimed)
    return claimed["command_id"]


def immutable_rows(store, cid):
    return [tuple(store.connection.execute("SELECT * FROM " + table + " WHERE command_id=?", (cid,)).fetchone())
            for table in ("operator_plane_commands", "operator_commands", "serial206_movement_commands")]


def test_home_resolution_is_per_command_additive_and_received(rig):
    store, *_ = rig
    # An earlier home is not evidence for a subsequent unknown effect.
    home(rig, "oem.z.manual_home")
    first = run_failure(rig, pickup({"ok": False, "delivery_attempted": True}))
    before = immutable_rows(store, first)
    assert not store.settle_deck_recovery_by_home()
    z = home(rig, "oem.z.manual_home")
    second = run_failure(rig, pickup({"ok": False, "delivery_attempted": True}))
    # Settlement is bookkeeping even while an unrelated live owner exists.
    state = rig[3]
    admitted = store.admit_command({"action_id": "oem.z.manual_home", "inputs": {},
        "idempotency_key": "live-home", "expected_ownership_generation": 1}, state=state)
    live = store.claim_next()
    assert live["command_id"] == admitted["command_id"]
    resources = list(store.connection.execute("SELECT * FROM serial206_command_resources"))
    assert resources
    assert store.settle_deck_recovery_by_home()
    assert list(store.connection.execute("SELECT * FROM serial206_command_resources")) == resources
    assert store.get_command(live["command_id"])["status"] == "dispatched"
    store.finish(live["command_id"], status="failed", payload={"ok": False}, claimed=live)
    assert immutable_rows(store, first) == before
    assert store.recovery()["outcome_unknown_command_ids"] == [second]
    assert store.recovery()["hold"] is True
    for reader in (store.get_command, store.get_command_summary, store.command_detail_v2):
        receipt = reader(first)
        assert receipt["status"] == "ambiguous"
        resolution = receipt["deck_movement"]["recovery_resolution"]
        assert resolution["command_id"] == first
        assert {r["axis"]: r["command_id"] for r in resolution["home_evidence"]} == {"z": z}
    assert not store.settle_deck_recovery_by_home()
    for action in ("oem.z.manual_home", "oem.x.manual_panel_home", "oem.y.manual_panel_home"):
        home(rig, action)
    assert store.settle_deck_recovery_by_home()
    assert store.recovery()["outcome_unknown_command_ids"] == []
    assert store.recovery()["hold"] is False
    assert immutable_rows(store, first) == before
    assert store.get_command(second)["status"] == "ambiguous"
    assert not store.settle_deck_recovery_by_home()

@pytest.mark.parametrize("envelope,expected", [
    ({"ok": False, "error": "command_failed", "message": "eject refused", "details": {
        "channel": 0, "result": {"tx_ok": False, "delivery_verified": False,
        "error": "completion already registered"}, "sends": []}}, "failed"),
    ({"ok": False, "error": "command_failed", "details": {
        "channel": 1, "result": {"tx_ok": False, "delivery_verified": False},
        "sends": [{"channel": 0, "result": {"ok": True, "delivery_verified": True}}]}}, "failed"),
    ({"ok": False, "error": "command_failed", "details": {
        "channel": 1, "result": {"tx_ok": False, "delivery_verified": False},
        "sends": [{"channel": 0, "result": {"ok": False, "delivery_attempted": True}}]}}, "ambiguous"),
    ({"ok": False, "completed_children": [{"operation": "first", "result": {"ok": True}}],
      "failure_evidence": [{"operation": "sourceHomeZ", "exception_type": "RuntimeError",
        "exception_message": "refused", "result": {"tx_ok": False, "delivery_verified": False}}]}, "failed"),
    ({"ok": False, "completed_children": [], "failure_evidence": [{"operation": "sourceHomeZ",
      "exception_type": "RuntimeError", "exception_message": "timeout",
      "result": {"delivery_attempted": True}}]}, "ambiguous"),
    ({"ok": False, "diagnostics": {"result": {"tx_ok": False, "delivery_verified": False}}}, "ambiguous"),
])
def test_production_exception_envelopes(rig, envelope, expected):
    result = pickup({})
    result["steps"][-1] = {"operation": "MoveZHome", "evidence": envelope, "error": "raised"}
    result["failed_step"] = result["steps"][-1]
    cid = run_failure(rig, result)
    store = rig[0]
    assert store.get_command(cid)["status"] == expected
    retained = json.loads(store.wp8_operation_evidence(cid)["children"][0]["terminal_evidence_json"])["result"]
    assert retained["failed_step"]["evidence"] == envelope
    assert retained["steps"][:2] == result["steps"][:2]


def test_unknown_scope_is_not_invented_and_mixed_axes(rig):
    store, *_ = rig
    unknown = run_failure(rig, DeckExecutionFailure("unrecorded step", delivery_attempted=True))
    xyz = run_failure(rig, {"ok": False, "delivery_attempted": True, "steps": [
        {"operation": "moveTo", "result": {"ok": False, "delivery_attempted": True}}]})
    z = run_failure(rig, pickup({"ok": False, "delivery_attempted": True}))
    assert store.get_command(unknown)["affected_axes"] is None
    assert store.get_command(xyz)["affected_axes"] == ["x", "y", "z"]
    home(rig, "oem.z.manual_home", "failed")
    assert not store.settle_deck_recovery_by_home()
    home(rig, "oem.z.manual_home")
    assert store.settle_deck_recovery_by_home()
    assert store._home_recovery_resolution(z)
    assert store._home_recovery_resolution(xyz) is None
    home(rig, "oem.xy.home")
    assert store.settle_deck_recovery_by_home()
    assert store._home_recovery_resolution(xyz)
    assert store._home_recovery_resolution(unknown) is None
    assert store.recovery()["outcome_unknown_command_ids"] == [unknown]


def test_no_repeat_writer_or_history_scan(rig, monkeypatch):
    store, *_ = rig
    run_failure(rig, pickup({"ok": False, "delivery_attempted": True}))
    assert not store.settle_deck_recovery_by_home()
    statements = []
    store.connection.set_trace_callback(statements.append)
    with monkeypatch.context() as m:
        m.setattr(store, "_transaction", lambda: pytest.fail("repeat writer opened"))
        for _ in range(3):
            assert not store.settle_deck_recovery_by_home()
    store.connection.set_trace_callback(None)
    assert len(statements) == 3
    assert all("MAX(transition_sequence)" in sql for sql in statements)


def test_home_order_uses_terminal_time_not_mutable_update(rig):
    store, *_ = rig
    earlier = home(rig, "oem.z.manual_home")
    cid = run_failure(rig, pickup({"ok": False, "delivery_attempted": True}))
    with store._transaction() as conn:
        conn.execute("UPDATE operator_commands SET updated_at=updated_at+1000 WHERE command_id=?", (earlier,))
    assert not store.settle_deck_recovery_by_home()
    assert store._home_recovery_resolution(cid) is None
    home(rig, "oem.z.manual_home")
    assert store.settle_deck_recovery_by_home()


@pytest.mark.parametrize("independent,unknown_child", [(False, False), (True, False), (False, True)])
def test_workflow_resolution_requires_only_resolved_children(rig, independent, unknown_child):
    store, *_ = rig
    store.bind_workflow_dispatcher(lambda _: None)
    store.admit_workflow(command_id="parent", idempotency_key="parent", plan_fingerprint="fixture",
        requested_inputs={"bundle": {}}, ownership_generation=1,
        resources=("axis:x", "axis:y", "axis:z"), board_epochs={})
    assert store.claim_next()["command_id"] == "parent"
    assert store._renew_owner(lease_seconds=600)
    with store.workflow_context("parent", source_occurrence_id="pickup"):
        child = run_failure(rig, pickup({"ok": False, "delivery_attempted": True}))
    if unknown_child:
        with store.workflow_context("parent", source_occurrence_id="unknown"):
            other = run_failure(rig, DeckExecutionFailure("unknown step", delivery_attempted=True))
    store.finish_workflow("parent", status="failed", payload={}, lifecycle_settled=not independent)
    before = [tuple(store.connection.execute("SELECT * FROM " + table + " WHERE command_id='parent'").fetchone())
              for table in ("operator_commands", "operator_plane_commands")]
    home(rig, "oem.z.manual_home")
    assert store.settle_deck_recovery_by_home()
    assert store._home_recovery_resolution(child)
    resolution = store.get_command("parent")["recovery_resolution"]
    assert bool(resolution) is (not independent and not unknown_child)
    assert ("parent" in store.recovery()["outcome_unknown_command_ids"]) is (independent or unknown_child)
    assert before == [tuple(store.connection.execute("SELECT * FROM " + table + " WHERE command_id='parent'").fetchone())
                      for table in ("operator_commands", "operator_plane_commands")]
    if resolution:
        assert resolution["child_evidence"][0]["command_id"] == child

