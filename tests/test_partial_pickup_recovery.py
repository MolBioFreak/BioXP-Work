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
    home(rig, "oem.x.manual_panel_home", "failed")
    assert not store.settle_deck_recovery_by_home()
    x = home(rig, "oem.x.manual_panel_home")
    assert not store.settle_deck_recovery_by_home()
    y = home(rig, "oem.y.manual_panel_home")
    second = run_failure(rig, pickup({"ok": False, "delivery_attempted": True}))
    assert store.settle_deck_recovery_by_home()
    assert immutable_rows(store, first) == before
    assert store.recovery()["outcome_unknown_command_ids"] == [second]
    assert store.recovery()["hold"] is True
    for reader in (store.get_command, store.get_command_summary, store.command_detail_v2):
        receipt = reader(first)
        assert receipt["status"] == "ambiguous"
        resolution = receipt["deck_movement"]["recovery_resolution"]
        assert resolution["command_id"] == first
        assert {r["axis"]: r["command_id"] for r in resolution["home_evidence"]} == {"x": x, "y": y, "z": z}
    assert not store.settle_deck_recovery_by_home()
    for action in ("oem.z.manual_home", "oem.x.manual_panel_home", "oem.y.manual_panel_home"):
        home(rig, action)
    assert store.settle_deck_recovery_by_home()
    assert store.recovery()["outcome_unknown_command_ids"] == []
    assert store.recovery()["hold"] is False
    assert immutable_rows(store, first) == before
    assert store.get_command(second)["status"] == "ambiguous"
    assert not store.settle_deck_recovery_by_home()
