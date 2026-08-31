from __future__ import annotations

from fastapi import HTTPException
from contextlib import contextmanager
import json
import pytest
import sqlite3
import statistics
import time

from bioxp import operator_command_plane
from bioxp.operator_command_plane import ACTION_REQUEST_SCHEMA, OperatorCommandStore
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.oem_deck_catalog import DeckCatalog, configured_location_names
from bioxp.oem_deck_movement import (
    DeckAuthoritySnapshot, DeckExecutionFailure, NamedLocationIntent,
    compile_named_location, make_deck_command_executor,
)
from bioxp.oem_compat.position_table import PositionTable


def _operator_store(root):
    runtime_owner = OEMRuntimeStore(root)
    runtime_owner.close()
    return OperatorCommandStore(root)


def _state() -> dict:
    return {
        "ownership_generation": 7,
        "serial206_initialization_provider": {
            "x_authority": {"active_board_epoch": 11},
            "board4_authority": {"active_board_epoch": 10},
        },
    }


def _request(key: str = "deck-command-1") -> dict:
    return {
        "schema_version": ACTION_REQUEST_SCHEMA, "idempotency_key": key,
        "expected_ownership_generation": 7, "expected_board_epoch_by_board": {"4": 10, "5": 11},
        "action_id": "oem.deck.move_to_location", "inputs": {"target": "LOC_OC", "camera_offset": False},
    }


def _ambiguous_deck_store(tmp_path):
    store = _operator_store(tmp_path)
    admitted = store.admit_command(_request("deck-recovery-ambiguous"), state=_state())
    table = _stage_table()
    authority = DeckAuthoritySnapshot(
        7, "provider-owner", 10, 11, table.digest, 3,
        {"x": 1, "y": 1, "z": 1, "g": 1}, {"global": 0, "x": 0, "y": 0, "z": 0},
        "latch-1", "position-1", 1.0, 0, 0, 65000, "LOC_MS", 0,
        False, False, -1, True, None, 500, "BIOXP", True, True,
    )
    plan = compile_named_location(
        NamedLocationIntent("LOC_OC"), DeckCatalog.from_position_table(table), table, authority
    )
    store.persist_deck_plan(admitted["command_id"], plan)
    assert store._acquire_owner() is True
    claimed = store.claim_next()
    assert claimed is not None
    store.mark_deck_recovery_required(
        admitted["command_id"], reason="controller outcome unknown",
        controller_command_acknowledged=True,
    )
    store.finish(
        admitted["command_id"], status="ambiguous",
        payload={"delivery_attempted": True, "outcome_unknown": True},
        controller_acknowledged=True, claimed=claimed,
    )
    return store, admitted["command_id"]


def test_one_durable_deck_recovery_predicate_blocks_all_cross_table_states(tmp_path) -> None:
    store = _operator_store(tmp_path)
    producer = store.admit_command(_request("predicate-producer"), state=_state())

    def set_state(*, hold: bool, ambiguity: str) -> None:
        with store._transaction() as conn:
            semantic = conn.execute(
                "SELECT semantic_state_revision FROM operator_plane_deck_semantic_state WHERE singleton=1"
            ).fetchone()
            before = int(semantic[0])
            provenance = {
                "source_operation": "direct_sql_recovery_predicate_seam",
                "command_id": producer["command_id"],
                "before_revision": before,
                "after_revision": before + 1,
                "ambiguity_state": ambiguity,
            }
            conn.execute(
                "UPDATE operator_plane_safety SET recovery_hold=? WHERE singleton=1",
                (int(hold),),
            )
            conn.execute(
                "UPDATE operator_plane_deck_semantic_state SET semantic_state_revision=?,producer_operation=?,producer_command_id=?,transition_provenance_json=?,ambiguity_state=? WHERE singleton=1",
                (
                    before + 1,
                    "direct_sql_recovery_predicate_seam",
                    producer["command_id"],
                    json.dumps(provenance, sort_keys=True, separators=(",", ":")),
                    ambiguity,
                ),
            )

    cases = [
        (False, "none", None),
        (True, "none", "deck_recovery_state_inconsistent"),
        (False, "recovery_required", "deck_recovery_state_inconsistent"),
        (True, "recovery_required", "deck_recovery_hold"),
        (True, "ambiguous", "deck_recovery_hold"),
    ]
    for index, (hold, ambiguity, expected) in enumerate(cases):
        set_state(hold=hold, ambiguity=ambiguity)
        assert store.deck_recovery_blocker() == expected
        if expected is None:
            admitted = store.admit_command(_request(f"predicate-clean-{index}"), state=_state())
            assert admitted["status"] == "queued"
        else:
            with pytest.raises(HTTPException) as blocked:
                store.admit_command(_request(f"predicate-blocked-{index}"), state=_state())
            assert blocked.value.detail["error"] == expected
    set_state(hold=False, ambiguity="none")
    assert store.deck_recovery_blocker() is None
    store.stop()


def test_deck_command_admission_is_async_fifo_idempotent_and_fenced(tmp_path) -> None:
    store = _operator_store(tmp_path)
    first = store.admit_command(_request(), state=_state())
    replay = store.admit_command(_request(), state=_state())
    second = store.admit_command(_request("deck-command-2"), state=_state())
    assert first["status"] == "queued" and replay["command_id"] == first["command_id"]
    assert second["stream_sequence"] == first["stream_sequence"] + 1
    with pytest.raises(HTTPException) as stale:
        store.admit_command({**_request("deck-command-3"), "expected_board_epoch_by_board": {"4": 9, "5": 11}}, state=_state())
    assert stale.value.status_code == 409
    store.stop()


def test_no_provider_io_admission_latency_and_capacity_acceptance_record(tmp_path, monkeypatch, capsys) -> None:
    sample_count = 32
    monkeypatch.setattr(operator_command_plane, "COMMAND_CAPACITY", sample_count)
    store = _operator_store(tmp_path / "count")
    latencies_ms = []
    request = None
    for index in range(sample_count):
        request = {
            "schema_version": ACTION_REQUEST_SCHEMA, "idempotency_key": f"latency-{index}",
            "expected_ownership_generation": 7, "expected_board_epoch_by_board": {},
            "action_id": "oem.x.move_steps", "inputs": {"steps": 1},
        }
        started = time.perf_counter_ns()
        store.admit_command(request, state=_state())
        latencies_ms.append((time.perf_counter_ns() - started) / 1_000_000)
    ordered = sorted(latencies_ms)
    evidence = {
        "sample_count": sample_count, "sequential_submissions": sample_count,
        "p50_ms": statistics.median(ordered),
        "p95_ms": ordered[max(0, int(sample_count * 0.95) - 1)],
        "max_ms": max(ordered), "baseline_max_ms": 100.0,
    }
    print(json.dumps(evidence, sort_keys=True))
    assert evidence["max_ms"] < evidence["baseline_max_ms"]
    assert request is not None
    with pytest.raises(HTTPException) as count_rejected:
        store.admit_command({**request, "idempotency_key": "count-rejected"}, state=_state())
    assert count_rejected.value.detail["error"] == "capacity_exceeded"
    store.stop()

    monkeypatch.setattr(operator_command_plane, "COMMAND_CAPACITY", 1024)
    monkeypatch.setattr(operator_command_plane, "COMMAND_BYTES_CAPACITY", 1)
    byte_store = _operator_store(tmp_path / "bytes")
    with pytest.raises(HTTPException) as bytes_rejected:
        byte_store.admit_command({**request, "idempotency_key": "bytes-rejected"}, state=_state())
    assert bytes_rejected.value.detail["error"] == "capacity_exceeded"
    byte_store.stop()
    assert capsys.readouterr().out


def test_active_deck_command_exclusively_owns_all_normal_movement_resources(tmp_path) -> None:
    store = _operator_store(tmp_path)
    deck = store.admit_command(_request("deck-global-resource-owner"), state=_state())
    axis = store.admit_command(
        {
            "schema_version": ACTION_REQUEST_SCHEMA,
            "idempotency_key": "axis-blocked-by-deck",
            "expected_ownership_generation": 7,
            "expected_board_epoch_by_board": {},
            "action_id": "oem.x.move_steps",
            "inputs": {"steps": 100},
        },
        state=_state(),
    )
    assert store._acquire_owner() is True
    claimed = store.claim_next()
    assert claimed is not None and claimed["command_id"] == deck["command_id"]
    resources = {
        row[0]
        for row in store.connection.execute(
            "SELECT resource_key FROM serial206_command_resources WHERE command_id=?",
            (deck["command_id"],),
        ).fetchall()
    }
    assert resources == {
        "axis:x", "axis:y", "axis:z", "axis:g",
        "motor:4:0", "motor:4:1", "motor:5:0",
    }
    assert store.claim_next() is None
    assert store.get_command(axis["command_id"])["status"] == "queued"
    store.stop()


def test_blocked_earliest_movement_cannot_be_overtaken_by_later_nonconflicting_axis(tmp_path) -> None:
    store = _operator_store(tmp_path)
    active_x = store.admit_command(
        {
            "schema_version": ACTION_REQUEST_SCHEMA, "idempotency_key": "fifo-active-x",
            "expected_ownership_generation": 7, "expected_board_epoch_by_board": {},
            "action_id": "oem.x.move_steps", "inputs": {"steps": 100},
        }, state=_state(),
    )
    assert store._acquire_owner() is True
    assert store.claim_next()["command_id"] == active_x["command_id"]
    blocked_deck = store.admit_command(_request("fifo-blocked-deck"), state=_state())
    later_z = store.admit_command(
        {
            "schema_version": ACTION_REQUEST_SCHEMA, "idempotency_key": "fifo-later-z",
            "expected_ownership_generation": 7, "expected_board_epoch_by_board": {},
            "action_id": "oem.z.move_steps", "inputs": {"steps": 100},
        }, state=_state(),
    )

    assert store.claim_next() is None
    assert store.get_command(blocked_deck["command_id"])["status"] == "queued"
    assert store.get_command(later_z["command_id"])["status"] == "queued"
    store.stop()


def test_deck_input_schema_rejects_raw_fields_and_contradictory_barcode(tmp_path) -> None:
    store = _operator_store(tmp_path)
    with pytest.raises(HTTPException) as raw:
        store.admit_command({**_request(), "inputs": {"target": "LOC_OC", "camera_offset": False, "x": 1}}, state=_state())
    assert raw.value.status_code == 422
    with pytest.raises(HTTPException) as contradictory:
        store.admit_command({**_request("deck-command-4"), "inputs": {"target": "LOC_TC_BARCODE", "camera_offset": True}}, state=_state())
    assert contradictory.value.status_code == 422
    store.stop()


def test_existing_store_persists_deck_plan_children_and_success_only_semantic_transition(tmp_path) -> None:
    store = _operator_store(tmp_path)
    admitted = store.admit_command(_request(), state=_state())
    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    authority = DeckAuthoritySnapshot(
        7, "provider-owner", 10, 11, table.digest, 3,
        {"x": 1, "y": 1, "z": 1, "g": 1}, {"global": 0, "x": 0, "y": 0, "z": 0},
        "latch-1", "position-1", 1.0, 0, 0, 65000, "LOC_MS", 0,
        False, False, -1, True, None, 500, "BIOXP", True, True,
    )
    catalog = DeckCatalog.from_position_table(table)
    plan = compile_named_location(NamedLocationIntent("LOC_OC"), catalog, table, authority)

    store.persist_deck_pseudo_home(
        admitted["command_id"], 500, source_operation="ForceToHighHome",
        ownership_generation=7, board_epoch_4=10, board_epoch_5=11,
    )
    before = store.deck_semantic_state()
    store.persist_deck_plan(admitted["command_id"], plan)
    planned = store.get_command(admitted["command_id"])
    assert planned["deck_movement"]["target"] == "LOC_OC"
    assert planned["deck_movement"]["position_table_revision"] == table.digest
    assert planned["deck_movement"]["plan_digest"] == plan.plan_digest
    assert [row["operation"] for row in planned["deck_movement"]["stages"]] == [step.operation for step in plan.steps]
    assert store.deck_semantic_state()["current_location"] == before["current_location"]

    store.commit_deck_success(
        admitted["command_id"],
        plan,
        [{"provider_command_id": "provider-1", "controller_completion_verified": True}],
    )
    after = store.deck_semantic_state()
    assert after["current_location"] == "LOC_OC"
    assert after["current_well"] == 0
    assert after["semantic_state_revision"] > before["semantic_state_revision"]
    detail = store.get_command(admitted["command_id"])["deck_movement"]
    assert detail["controller_completion_verified"] is True
    assert detail["semantic_state_committed"] is True
    assert detail["physical_observation_verified"] is False
    assert [stage["terminal_state"] for stage in detail["stages"]] == ["completed"] * len(plan.steps)
    for stage in detail["stages"]:
        assert stage["terminal_evidence"]["operation"] == stage["operation"]
        assert stage["terminal_evidence"]["source_anchor"] == stage["source_anchor"]
        assert len(stage["terminal_evidence"]["arguments_digest"]) == 64
    store.stop()


def test_barcode_success_commits_resolved_oem_location_not_panel_target(tmp_path) -> None:
    store = _operator_store(tmp_path)
    admitted = store.admit_command(
        {**_request("deck-barcode-success"), "inputs": {"target": "LOC_TC_BARCODE", "camera_offset": False}},
        state=_state(),
    )
    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    authority = DeckAuthoritySnapshot(
        7, "provider-owner", 10, 11, table.digest, 3,
        {"x": 1, "y": 1, "z": 1, "g": 1}, {"global": 0, "x": 0, "y": 0, "z": 0},
        "latch-1", "position-1", 1.0, 0, 0, 65000, "LOC_MS", 0,
        False, False, -1, True, None, 500, "BIOXP", True, True,
    )
    plan = compile_named_location(
        NamedLocationIntent("LOC_TC_BARCODE"), DeckCatalog.from_position_table(table), table, authority
    )
    store.persist_deck_plan(admitted["command_id"], plan)
    store.commit_deck_success(
        admitted["command_id"],
        plan,
        [
            {"provider_command_id": "provider-barcode-move", "controller_completion_verified": True},
            {"provider_command_id": "provider-barcode-camera", "controller_completion_verified": True},
        ],
    )

    semantic = store.deck_semantic_state()
    assert plan.resolved_location_id == 2
    assert semantic["current_location"] == "LOC_TC"
    assert semantic["transition_provenance"]["current_location"] == "LOC_TC"
    store.stop()


def test_explicit_no_io_stage_evidence_preserves_delivery_false() -> None:
    step = type("Step", (), {"operation": "check_latch_status", "source_anchor": "source", "arguments": None})()
    evidence = OperatorCommandStore._deck_stage_evidence(
        step, {"value": True, "delivery_attempted": False, "physical_motion_commanded": False}
    )
    assert evidence["delivery_attempted"] is False


def test_deck_schema_is_exact_and_rejects_direct_sql_rebinding(tmp_path) -> None:
    store = _operator_store(tmp_path)
    conn = store.connection
    expected_columns = {
        "operator_plane_deck_commands": (
            "command_id", "target", "target_label", "resolved_location_id",
            "destination_catalog_revision", "position_table_revision",
            "authority_snapshot_digest", "complete_authority_digest", "plan_digest",
            "source_branch", "source_anchors_json", "source_hazards_json",
            "delivery_attempted", "controller_command_acknowledged",
            "controller_completion_verified", "hardware_postcondition_verified",
            "semantic_state_committed", "physical_observation_verified",
            "transition_revision", "ambiguity_state", "provider_evidence_json",
            "planned_at", "committed_at",
        ),
        "operator_plane_deck_stages": (
            "command_id", "stage_order", "operation", "source_anchor",
            "resources_json", "arguments_json", "dependency_order_json",
            "terminal_evidence_json", "terminal_state",
        ),
        "operator_plane_deck_semantic_transitions": (
            "transition_revision", "command_id", "source_operation",
            "before_revision", "after_revision", "transition_json", "created_at",
        ),
    }
    for table, columns in expected_columns.items():
        assert tuple(row[1] for row in conn.execute(f"PRAGMA table_info({table})")) == columns

    trigger_names = {
        row[0]
        for row in conn.execute(
            "SELECT name FROM sqlite_master WHERE type='trigger' AND tbl_name LIKE 'operator_plane_deck_%'"
        )
    }
    assert {
        "operator_plane_deck_commands_identity_immutable_v1",
        "operator_plane_deck_commands_evidence_coherence_v1",
        "operator_plane_deck_commands_no_delete_v1",
        "operator_plane_deck_stages_identity_immutable_v1",
        "operator_plane_deck_stages_terminal_immutable_v1",
        "operator_plane_deck_stages_no_delete_v1",
        "operator_plane_deck_semantic_state_coherence_v1",
        "operator_plane_deck_semantic_transitions_coherence_v1",
        "operator_plane_deck_semantic_transitions_no_update_v1",
        "operator_plane_deck_semantic_transitions_no_delete_v1",
    } <= trigger_names
    assert conn.execute("PRAGMA foreign_key_check").fetchall() == []

    admitted = store.admit_command(_request("deck-schema-1"), state=_state())
    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    authority = DeckAuthoritySnapshot(
        7, "provider-owner", 10, 11, table.digest, 3,
        {"x": 1, "y": 1, "z": 1, "g": 1}, {"global": 0, "x": 0, "y": 0, "z": 0},
        "latch-1", "position-1", 1.0, 0, 0, 65000, "LOC_MS", 0,
        False, False, -1, True, None, 500, "BIOXP", True, True,
    )
    plan = compile_named_location(
        NamedLocationIntent("LOC_OC"), DeckCatalog.from_position_table(table), table, authority
    )
    store.persist_deck_plan(admitted["command_id"], plan)
    with pytest.raises(sqlite3.DatabaseError):
        conn.execute(
            "UPDATE operator_plane_deck_commands SET target='LOC_RC' WHERE command_id=?",
            (admitted["command_id"],),
        )
    with pytest.raises(sqlite3.DatabaseError):
        conn.execute(
            "UPDATE operator_plane_deck_commands SET delivery_attempted=1 WHERE command_id=?",
            (admitted["command_id"],),
        )
    with pytest.raises(sqlite3.DatabaseError):
        conn.execute(
            "DELETE FROM operator_plane_deck_stages WHERE command_id=? AND stage_order=0",
            (admitted["command_id"],),
        )
    with pytest.raises(sqlite3.DatabaseError):
        conn.execute("UPDATE operator_plane_deck_semantic_state SET pseudo_z_home=500 WHERE singleton=1")
    store.stop()


class _StageProvider:
    def __init__(self, table, *, mode):
        self.table = table; self.mode = mode; self.calls = 0

    @contextmanager
    def movement_lease(self):
        yield

    def force_to_high_home(self, **kwargs): return {"ok": True}

    def deck_authority_snapshot(self, *, expected_generation):
        return {
            "ownership_generation": expected_generation, "provider_owner_id": "stage-provider",
            "board_epoch_4": 10, "board_epoch_5": 11, "position_table_sha256": self.table.digest,
            "machine_state_revision": 1, "reference_versions": {"x": 1, "y": 1, "z": 1, "g": 1},
            "safety_epochs": {"global": 0, "x": 0, "y": 0, "z": 0},
            "latch_observation_id": "l", "controller_position_observation_id": "p", "captured_at": 1.0,
            "current_x": 0, "current_y": 0, "current_z": 65000, "current_location_id": "LOC_MS",
            "current_well_id": 0, "tip_loaded": False, "tip_dirty": False, "tip_location": -1,
            "clean_path": True, "plate_on_gantry": None, "pseudo_z_home": 500, "device_type": "BIOXP",
            "latch_status": True, "machine_latch_closed": True,
        }

    def moveTo(self, **kwargs):
        self.calls += 1
        if self.mode == "ambiguous":
            return {"ok": False, "controller_command_acknowledged": True}
        return {"ok": True, "controller_command_acknowledged": True, "controller_completion_verified": True, "hardware_postcondition_verified": True}


class _NoopParkProvider(_StageProvider):
    def deck_authority_snapshot(self, *, expected_generation):
        snapshot = super().deck_authority_snapshot(expected_generation=expected_generation)
        snapshot.update({"current_location_id": "LOC_PARK", "current_well_id": 4})
        return snapshot

    def parkGantry(self, **kwargs):
        self.calls += 1
        return {
            "ok": True, "source_noop": True, "delivery_attempted": False,
            "controller_command_acknowledged": False, "controller_completion_verified": False,
            "hardware_postcondition_verified": False,
        }


def _stage_table():
    return PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])


def test_already_parked_no_io_branch_still_commits_caller_update_location(tmp_path):
    store = _operator_store(tmp_path)
    admitted = store.admit_command(
        {**_request("already-parked"), "inputs": {"target": "LOC_PARK", "camera_offset": False}}, state=_state()
    )
    table = _stage_table(); provider = _NoopParkProvider(table, mode="noop")
    executor = make_deck_command_executor(provider_getter=lambda: provider, position_table_provider=lambda: table, command_store=store)
    response = executor(
        command_id=admitted["command_id"], target="LOC_PARK", camera_offset=False,
        expected_ownership_generation=7, expected_board_epoch_by_board={"4": 10, "5": 11},
    )
    semantic = store.deck_semantic_state()
    assert response["ok"] is True and response["delivery_attempted"] is False
    assert response["controller_command_acknowledged"] is False
    assert response["controller_completion_verified"] is False
    assert response["hardware_postcondition_verified"] is False
    assert semantic["current_location"] == "LOC_PARK" and semantic["current_well"] == 0
    store.stop()


@pytest.mark.parametrize(("mode", "expected"), [("missing", "failed"), ("ambiguous", "ambiguous")])
def test_real_store_executor_terminalizes_pre_io_and_post_io_stage_evidence(tmp_path, mode, expected):
    store = _operator_store(tmp_path)
    admitted = store.admit_command(_request(f"stage-{mode}"), state=_state())
    table = _stage_table(); provider = _StageProvider(table, mode=mode)
    if mode == "missing":
        provider.moveTo = None
    executor = make_deck_command_executor(provider_getter=lambda: provider, position_table_provider=lambda: table, command_store=store)
    assert store._acquire_owner() is True
    claimed = store.claim_next(); assert claimed is not None
    try:
        response = executor(command_id=admitted["command_id"], target="LOC_OC", camera_offset=False, expected_ownership_generation=7, expected_board_epoch_by_board={"4": 10, "5": 11})
    except DeckExecutionFailure as exc:
        assert exc.delivery_attempted is False
        store.finish(admitted["command_id"], status="failed", payload={"delivery_attempted": False}, claimed=claimed)
    else:
        assert response["delivery_attempted"] is True
        store.finish(admitted["command_id"], status="ambiguous", payload={"delivery_attempted": True, "outcome_unknown": True}, controller_acknowledged=True, claimed=claimed)
    detail = store.get_command(admitted["command_id"])["deck_movement"]
    assert all(stage["terminal_state"] != "planned" for stage in detail["stages"])
    assert detail["stages"][-1]["terminal_state"] == expected
    assert detail["stages"][-1]["terminal_evidence"]["delivery_attempted"] is (mode == "ambiguous")
    store.stop()


def test_restart_and_ambiguous_recovery_hold_blocks_new_deck_admission_and_dispatch(tmp_path):
    first = _operator_store(tmp_path)
    admitted = first.admit_command(_request("restart-active"), state=_state())
    assert first._acquire_owner() is True
    assert first.claim_next()["command_id"] == admitted["command_id"]
    first.connection.execute("UPDATE operator_plane_lane SET owner_lease_until=0 WHERE singleton=1")
    first.connection.close()

    recovered = _operator_store(tmp_path)
    assert recovered.recovery()["hold"] is True
    assert recovered.claim_next() is None
    with pytest.raises(HTTPException) as blocked:
        recovered.admit_command(_request("blocked-new-deck"), state=_state())
    assert blocked.value.status_code == 409
    assert blocked.value.detail["error"] == "deck_recovery_hold"
    recovered.stop()


def test_unknown_outcome_acknowledgement_preserves_deck_hold_and_blocks_admission(tmp_path):
    store, command_id = _ambiguous_deck_store(tmp_path)
    recovery = store.recovery()

    result = store.resolve_recovery(
        recovery["recovery_epoch"],
        {
            "idempotency_key": "ack-deck-outcome-unknown",
            "expected_version": recovery["version"],
            "expected_safety_epoch": recovery["global_safety_epoch"],
            "acknowledge_command_ids": [command_id],
            "operation": "cancel_pending",
        },
    )

    assert result["outcome_remains"] == "unknown"
    assert store.recovery()["hold"] is True
    assert store.deck_semantic_state()["ambiguity_state"] == "recovery_required"
    with pytest.raises(HTTPException) as blocked:
        store.admit_command(_request("still-blocked-after-ack"), state=_state())
    assert blocked.value.detail["error"] == "deck_recovery_hold"
    store.stop()


def test_governed_deck_reconciliation_is_atomic_and_clears_hold_only_with_exact_evidence(tmp_path):
    store, command_id = _ambiguous_deck_store(tmp_path)
    before = store.deck_semantic_state()
    recovery_before = store.recovery()
    table = _stage_table()
    common = {
        "command_id": command_id,
        "current_location": "LOC_MS",
        "current_well": 0,
        "current_authority": {
            "ownership_generation": 7,
            "provider_owner_id": "provider-owner",
            "board_epoch_4": 10,
            "board_epoch_5": 11,
            "position_table_sha256": table.digest,
            "machine_state_revision": before["semantic_state_revision"],
            "latch_status": True, "machine_latch_closed": True,
            "latch_observation_id": "controller-latch-reconciliation-1",
            "controller_position_observation_id": "controller-position-reconciliation-1",
            "current_x": 101, "current_y": 202, "current_z": 65000, "captured_at": 123.0,
            "observed_location_id": "LOC_MS", "observed_well_id": 0,
        },
        "current_position_table_revision": table.digest,
        "current_destination_catalog_revision": DeckCatalog.from_position_table(table).revision,
        "decision": {
            "decision_id": "deck-reconciliation-decision-1",
            "approved_by": "operator-test",
            "reason": "independent source-compatible position reconciliation",
        },
    }

    with pytest.raises(ValueError, match="controller position observation"):
        store.reconcile_deck_recovery(**{
            **common,
            "current_authority": {
                **common["current_authority"], "controller_position_observation_id": "",
            },
        })
    assert store.deck_semantic_state() == before
    assert store.recovery() == recovery_before

    result = store.reconcile_deck_recovery(
        **common,
        final_authority_reader=lambda: dict(common["current_authority"]),
    )

    after = store.deck_semantic_state()
    assert result["command_id"] == command_id
    assert result["reconciliation_decision"]["decision_id"] == "deck-reconciliation-decision-1"
    assert after["current_location"] == "LOC_MS"
    assert after["current_well"] == 0
    assert after["semantic_state_revision"] == before["semantic_state_revision"] + 1
    assert after["ambiguity_state"] == "none"
    assert after["transition_provenance"]["ambiguous_command_id"] == command_id
    assert after["transition_provenance"]["controller_position_observation"]["observation_id"] == "controller-position-reconciliation-1"
    assert store.recovery()["hold"] is False
    admitted = store.admit_command(_request("admitted-after-deck-reconciliation"), state=_state())
    assert admitted["status"] == "queued"
    store.stop()
