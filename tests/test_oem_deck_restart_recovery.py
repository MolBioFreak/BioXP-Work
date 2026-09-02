from __future__ import annotations

from contextlib import contextmanager
import json
import sqlite3
import time

import pytest

from bioxp.oem_deck_movement import (
    ClassMoveToIntent,
    DeckAuthoritySnapshot,
    DeckMovementExecutor,
    DeckMovementPlan,
    DeckPlanStep,
    compile_finite_plate_operation,
    compile_mov_execution,
)
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.operator_command_plane import OperatorCommandStore


def test_dispatcher_waits_for_a_live_prior_lease_then_acquires_ownership(tmp_path):
    OEMRuntimeStore(tmp_path).close()
    prior = OperatorCommandStore(tmp_path)
    replacement = OperatorCommandStore(tmp_path)
    assert prior._owner_acquired is True
    assert replacement._owner_acquired is False

    prior.connection.execute(
        "UPDATE operator_plane_lane SET owner_lease_until=? WHERE singleton=1",
        (time.time() + 0.05,),
    )
    started = time.monotonic()
    replacement.start(lambda _command: None)
    assert time.monotonic() - started < 2.0
    assert replacement._thread is not None
    assert replacement._thread.is_alive()
    assert replacement._owner_acquired is True
    owner = replacement.connection.execute(
        "SELECT owner_id FROM operator_plane_lane WHERE singleton=1"
    ).fetchone()[0]
    assert owner == replacement.owner_id
    replacement.stop()
    prior.connection.close()


def test_stop_joins_workers_spawned_during_shutdown(tmp_path):
    OEMRuntimeStore(tmp_path).close()
    store = OperatorCommandStore(tmp_path)

    class Worker:
        def __init__(self, on_join=None):
            self.alive = True
            self.joined = False
            self.on_join = on_join

        def is_alive(self):
            return self.alive

        def join(self, timeout=None):
            self.joined = True
            if self.on_join is not None:
                self.on_join()
            self.alive = False

    terminalizer = Worker()

    def spawn_terminalizer():
        with store._worker_lock:
            store._workers.add(terminalizer)

    dispatcher_worker = Worker(spawn_terminalizer)
    with store._worker_lock:
        store._workers.add(dispatcher_worker)

    store.stop()

    assert dispatcher_worker.joined is True
    assert terminalizer.joined is True


def _movable() -> dict[str, str]:
    return {
        "POOL_PLATE": "LOC_P_TC", "OUTPUT_PLATE": "LOC_P_OC", "REAGENT_PLATE": "LOC_RC",
        "STRIP_ONE": "LOC_STRIP1", "STRIP_TWO": "LOC_STRIP2", "STRIP_THREE": "LOC_STRIP3",
        "STRIP_FOUR": "LOC_STRIP4", "REAGENT_COVER": "LOC_RC_COVER_STORAGE",
        "OUTPUT_COVER": "LOC_OC_COVER_STORAGE", "BIO_SECURITY_COVER": "LOC_BSC",
        "TROUGH": "LOC_TROUGH",
    }


def _recovery_store(root) -> tuple[OperatorCommandStore, str]:
    OEMRuntimeStore(root).close()
    store = OperatorCommandStore(root)
    store.bootstrap_deck_semantic_state({
        "current_location": "LOC_MS", "current_well": 0, "current_tray": None,
        "tip_loaded": False, "tip_dirty": False, "tip_location": -1, "clean_path": True,
        "plate_on_gantry": None, "movable_plate_locations": _movable(), "pseudo_z_home": 65000,
        "ownership_generation": 1, "board_epoch_4": 2, "board_epoch_5": 3,
        "latch_status": True, "machine_latch_closed": True, "latch_observation_id": "latch-wp9",
        "source_operation": "test_source_snapshot", "source_command_id": "source-wp9",
    })
    state = {
        "ownership_generation": 1,
        "serial206_initialization_provider": {
            "x_authority": {"active_board_epoch": 3},
            "board4_authority": {"active_board_epoch": 2},
        },
    }
    admitted = store.admit_internal_mov_execution(
        ClassMoveToIntent(script_line=1, plate_name=0), state=state, idempotency_key="wp9-command",
    )
    claimed = store.claim_next()
    assert claimed is not None
    plan = compile_mov_execution(
        ClassMoveToIntent(script_line=1, plate_name=0),
        {
            "save_tip": False, "old_well": False, "old_location": 18, "old_well_text": "A1",
            "plate_locations": {0: 23}, "plate_pierced": {}, "well_pierced": {},
            "trough_version": 1, "authority_digest": "a" * 64,
            "board_epoch_4": 2, "board_epoch_5": 3,
        },
    )
    store.persist_deck_plan(admitted["command_id"], plan)
    return store, admitted["command_id"]


def _finite_recovery_store(root) -> tuple[OperatorCommandStore, str]:
    OEMRuntimeStore(root).close()
    store = OperatorCommandStore(root)
    store.bootstrap_deck_semantic_state({
        "current_location": "LOC_MS", "current_well": 0, "current_tray": None,
        "tip_loaded": False, "tip_dirty": False, "tip_location": -1, "clean_path": True,
        "plate_on_gantry": None, "movable_plate_locations": _movable(), "pseudo_z_home": 65000,
        "ownership_generation": 1, "board_epoch_4": 2, "board_epoch_5": 3,
        "latch_status": True, "machine_latch_closed": True, "latch_observation_id": "latch-wp9-finite",
        "source_operation": "test_source_snapshot", "source_command_id": "source-wp9-finite",
    })
    state = {
        "ownership_generation": 1,
        "serial206_initialization_provider": {
            "x_authority": {"active_board_epoch": 3},
            "board4_authority": {"active_board_epoch": 2},
        },
    }
    admitted = store.admit_internal_wp8_operation(
        "send_z_and_gripper_home", inputs={}, state=state,
        idempotency_key="wp9-finite-command",
    )
    assert store.claim_next() is not None
    return store, admitted["command_id"]


@pytest.mark.parametrize(
    ("persist_plan", "expected_reason"),
    [
        (False, "process_owner_loss_before_deck_plan"),
        (True, "process_owner_loss_before_first_tx"),
    ],
)
def test_claimed_deck_command_without_delivery_recovers_as_pre_io_failure(
    tmp_path, persist_plan, expected_reason,
):
    OEMRuntimeStore(tmp_path).close()
    store = OperatorCommandStore(tmp_path)
    store.bootstrap_deck_semantic_state({
        "current_location": "LOC_MS", "current_well": 0, "current_tray": None,
        "tip_loaded": False, "tip_dirty": False, "tip_location": -1, "clean_path": True,
        "plate_on_gantry": None, "movable_plate_locations": _movable(), "pseudo_z_home": 65000,
        "ownership_generation": 1, "board_epoch_4": 2, "board_epoch_5": 3,
        "latch_status": True, "machine_latch_closed": True, "latch_observation_id": "latch-pre-io",
        "source_operation": "test_source_snapshot", "source_command_id": "source-pre-io",
    })
    state = {
        "ownership_generation": 1,
        "serial206_initialization_provider": {
            "x_authority": {"active_board_epoch": 3},
            "board4_authority": {"active_board_epoch": 2},
        },
    }
    admitted = store.admit_internal_mov_execution(
        ClassMoveToIntent(script_line=2, plate_name=0),
        state=state,
        idempotency_key="wp9-pre-io-command",
    )
    assert store.claim_next() is not None
    if persist_plan:
        plan = compile_mov_execution(
            ClassMoveToIntent(script_line=2, plate_name=0),
            {
                "save_tip": False, "old_well": False, "old_location": 18,
                "old_well_text": "A1", "plate_locations": {0: 23},
                "plate_pierced": {}, "well_pierced": {}, "trough_version": 1,
                "authority_digest": "a" * 64, "board_epoch_4": 2, "board_epoch_5": 3,
            },
        )
        store.persist_deck_plan(admitted["command_id"], plan)
    store.connection.execute(
        "UPDATE operator_plane_lane SET owner_lease_until=0 WHERE singleton=1"
    )
    store.connection.close()

    recovered = OperatorCommandStore(tmp_path)
    try:
        row = recovered.get_command(admitted["command_id"])
        assert row is not None and row["status"] == "failed"
        assert row["terminal_evidence"]["reason"] == expected_reason
        assert row["terminal_evidence"]["delivery_attempted"] is False
        assert recovered.deck_recovery_blocker() is None
        persisted = recovered.connection.execute(
            "SELECT 1 FROM operator_plane_deck_commands WHERE command_id=?",
            (admitted["command_id"],),
        ).fetchone()
        assert (persisted is not None) is persist_plan
    finally:
        recovered.connection.close()


def test_late_mov_execution_transition_cannot_publish_after_command_loses_dispatch_lease(tmp_path):
    store, command_id = _recovery_store(tmp_path)
    before = tuple(store.connection.execute(
        "SELECT current_location,current_well,semantic_state_revision FROM "
        "operator_plane_deck_semantic_state WHERE singleton=1"
    ).fetchone())
    with store._transaction() as conn:
        with store._authority_write():
            conn.execute(
                "UPDATE operator_plane_commands SET status='failed',finished_at=?,updated_at=? "
                "WHERE command_id=?",
                (1.0, 1.0, command_id),
            )
            conn.execute(
                "UPDATE serial206_movement_commands SET state='failed',finished_at=? "
                "WHERE command_id=?",
                (1.0, command_id),
            )
            conn.execute(
                "UPDATE operator_plane_lane SET active_command_id=NULL,active_attempt_id=NULL "
                "WHERE singleton=1"
            )
    with pytest.raises(RuntimeError, match="movExecution dispatch authority is stale"):
        store.publish_mov_execution_transition(
            command_id, {"current_location": 23, "current_well": 1},
        )
    after = tuple(store.connection.execute(
        "SELECT current_location,current_well,semantic_state_revision FROM "
        "operator_plane_deck_semantic_state WHERE singleton=1"
    ).fetchone())
    assert after == before
    store.connection.close()


def test_wp7_stage_completion_and_semantic_publication_roll_back_together(tmp_path):
    store, command_id = _recovery_store(tmp_path)
    stage = store.connection.execute(
        "SELECT stage_order FROM operator_plane_deck_stages WHERE command_id=? ORDER BY stage_order LIMIT 1",
        (command_id,),
    ).fetchone()
    assert stage is not None
    step = DeckPlanStep(int(stage[0]), "updateLocation", "test:atomic-wp7")
    before = tuple(store.connection.execute(
        "SELECT current_location,current_well,semantic_state_revision FROM operator_plane_deck_semantic_state WHERE singleton=1"
    ).fetchone())

    with pytest.raises(ValueError, match="current location"):
        store.complete_mov_execution_stage(
            command_id, step, result={"ok": True, "semantic_update_ready": True},
            semantic_transition={"current_location": 999, "current_well": 1},
        )

    assert store.connection.execute(
        "SELECT terminal_state FROM operator_plane_deck_stages WHERE command_id=? AND stage_order=?",
        (command_id, int(stage[0])),
    ).fetchone()[0] == "planned"
    assert tuple(store.connection.execute(
        "SELECT current_location,current_well,semantic_state_revision FROM operator_plane_deck_semantic_state WHERE singleton=1"
    ).fetchone()) == before
    store.connection.close()


def test_wp8_child_completion_and_state_mutation_roll_back_together(tmp_path):
    store, command_id = _recovery_store(tmp_path)
    plan = {
        "schema_version": "bioxp.oem_wp8_operation.v1",
        "operation": "atomic-test",
        "authority_digest": "b" * 64,
        "children": [{
            "order": 0, "depends_on": [], "operation": "updateLocation",
            "arguments": {}, "ignored_return": False, "awaited": True,
            "exception_policy": "propagate", "state_mutation": {"current_location": 23},
        }],
    }
    from bioxp.oem_deck_movement import _digest
    plan["plan_digest"] = _digest(plan)
    stamps = {"ownership_generation": 1, "board_epoch_4": 2, "board_epoch_5": 3}
    store.persist_wp8_plan(command_id, plan, authority_stamps=stamps)

    with pytest.raises(RuntimeError, match="terminal-child-owned"):
        store.complete_wp8_child(
            command_id, 0, result={"ok": True, "semantic_update_ready": True},
            state_mutation={"current_location": 24}, authority_stamps=stamps,
        )

    assert store.connection.execute(
        "SELECT terminal_state FROM operator_plane_wp8_children WHERE command_id=? AND child_order=0",
        (command_id,),
    ).fetchone()[0] == "planned"
    assert store.connection.execute(
        "SELECT COUNT(*) FROM operator_plane_wp8_state_transitions WHERE command_id=?",
        (command_id,),
    ).fetchone()[0] == 0
    store.connection.close()


def test_post_io_ambiguity_preserves_exact_last_confirmed_semantic_revision_and_values(tmp_path):
    store, command_id = _recovery_store(tmp_path)
    before = tuple(store.connection.execute(
        "SELECT current_location,current_well,semantic_state_revision,producer_operation,"
        "producer_command_id,transition_provenance_json,ambiguity_state "
        "FROM operator_plane_deck_semantic_state WHERE singleton=1"
    ).fetchone())

    store.mark_deck_recovery_required(
        command_id, reason="controller_timeout", controller_command_acknowledged=True
    )

    after = tuple(store.connection.execute(
        "SELECT current_location,current_well,semantic_state_revision,producer_operation,"
        "producer_command_id,transition_provenance_json,ambiguity_state "
        "FROM operator_plane_deck_semantic_state WHERE singleton=1"
    ).fetchone())
    assert after == before
    assert store.deck_recovery_blocker() == "deck_recovery_hold"
    store.connection.close()


def test_recovery_blocker_is_derived_from_hold_and_unresolved_command_not_semantic_projection(tmp_path):
    store, command_id = _recovery_store(tmp_path)
    store.connection.execute(
        "UPDATE operator_plane_safety SET recovery_hold=1 WHERE singleton=1"
    )
    with store._authority_write():
        store.connection.execute(
            "UPDATE operator_plane_deck_commands SET ambiguity_state='recovery_required' WHERE command_id=?",
            (command_id,),
        )
    with store._authority_write():
        store.connection.execute(
            "UPDATE operator_plane_commands SET status='ambiguous' WHERE command_id=?",
            (command_id,),
        )
    assert store.deck_recovery_blocker() == "deck_recovery_hold"
    store.connection.close()


def test_stop_generation_is_checked_before_first_write_and_after_every_child():
    snapshot = DeckAuthoritySnapshot(
        ownership_generation=1, provider_owner_id="owner", board_epoch_4=2, board_epoch_5=3,
        position_table_sha256="a" * 64, machine_state_revision=1,
        reference_versions={"x": 1, "y": 1, "z": 1, "g": 1},
        safety_epochs={"global": 0, "x": 0, "y": 0, "z": 0},
        latch_observation_id="latch", controller_position_observation_id="position",
        captured_at=1.0, current_x=0, current_y=0, current_z=0,
        current_location_id="LOC_MS", current_well_id=0, tip_loaded=False,
        tip_dirty=False, tip_location=-1, clean_path=True, plate_on_gantry=None,
        pseudo_z_home=65000, device_type="BIOXP", latch_status=True,
        machine_latch_closed=True,
    )
    plan = DeckMovementPlan(
        target="LOC_BC", target_label="Barcode", resolved_location_id=1,
        source_branch="barcode", authority_digest=snapshot.digest,
        position_table_sha256="a" * 64, catalog_revision="b" * 64,
        steps=(DeckPlanStep(0, "moveTo", "source:one"), DeckPlanStep(1, "moveZCamera", "source:two")),
        semantic_transition={}, blocked_reason=None,
    )

    class Provider:
        def __init__(self):
            self.calls = []
        def moveTo(self):
            self.calls.append("moveTo")
            return {"ok": True}
        def moveZCamera(self):
            self.calls.append("moveZCamera")
            return {"ok": True}

    provider = Provider()
    checks = []
    executor = DeckMovementExecutor(provider, lambda: snapshot)
    executor.execute(
        plan, before_first_write=lambda _plan: checks.append("before"),
        after_each_child=lambda step: checks.append(step.order),
    )
    assert provider.calls == ["moveTo", "moveZCamera"]
    assert checks == ["before", 0, 1]


def test_delivery_attempts_are_append_only_and_bound_to_dispatch_identity(tmp_path):
    store, command_id = _recovery_store(tmp_path)
    stamps = {"ownership_generation": 1, "board_epoch_4": 2, "board_epoch_5": 3}
    lineage = store.connection.execute(
        "SELECT d.plan_digest,s.stage_order,s.operation FROM operator_plane_deck_commands d "
        "JOIN operator_plane_deck_stages s ON s.command_id=d.command_id "
        "WHERE d.command_id=? ORDER BY s.stage_order LIMIT 1",
        (command_id,),
    ).fetchone()
    assert lineage is not None
    work_identity = f"stage:{int(lineage['stage_order'])}:{str(lineage['operation'])}"

    first = store.record_delivery_attempt(
        command_id,
        work_kind="wp7_stage",
        work_identity=work_identity,
        plan_digest=str(lineage["plan_digest"]),
        authority_stamps=stamps,
    )
    second = store.record_delivery_attempt(
        command_id,
        work_kind="wp7_stage",
        work_identity=work_identity,
        plan_digest=str(lineage["plan_digest"]),
        authority_stamps=stamps,
    )

    assert second["attempt_sequence"] == first["attempt_sequence"] + 1
    rows = store.connection.execute(
        "SELECT * FROM operator_plane_delivery_attempts WHERE command_id=? ORDER BY attempt_sequence",
        (command_id,),
    ).fetchall()
    assert len(rows) == 2
    assert all(row["dispatch_attempt_id"] for row in rows)
    assert all(row["owner_id"] == store.owner_id for row in rows)
    assert all(row["ownership_generation"] == 1 for row in rows)
    with pytest.raises(Exception):
        store.connection.execute(
            "UPDATE operator_plane_delivery_attempts SET work_identity='counterfeit' WHERE attempt_sequence=?",
            (first["attempt_sequence"],),
        )
    with pytest.raises(Exception):
        store.connection.execute(
            "DELETE FROM operator_plane_delivery_attempts WHERE attempt_sequence=?",
            (first["attempt_sequence"],),
        )
    store.connection.close()


def test_startup_recovery_uses_delivery_attempt_and_running_wp8_task_truth(tmp_path):
    store, command_id = _recovery_store(tmp_path)
    stamps = {"ownership_generation": 1, "board_epoch_4": 2, "board_epoch_5": 3}
    plan = compile_finite_plate_operation(
        "send_z_and_gripper_home", source_leaf_available=True,
        run_in_parallel=True, gripper_position=4000, closed_position=3000,
    )
    store.persist_wp8_plan(command_id, plan, authority_stamps=stamps)
    store.create_wp8_background_task(
        command_id,
        1,
        task_id=f"{command_id}:1:{plan['plan_digest']}:z-home",
        task_kind="startMoveZPseudoHome",
        plan_digest=plan["plan_digest"],
        authority_stamps=stamps,
    )
    store.mark_wp8_background_task(
        command_id, 1, state="running", evidence={"thread_started": True}
    )
    store.connection.execute(
        "UPDATE operator_plane_lane SET owner_lease_until=0 WHERE singleton=1"
    )
    store.connection.close()

    recovered = OperatorCommandStore(tmp_path)
    try:
        command = recovered.get_command(command_id)
        assert command is not None and command["status"] == "ambiguous"
        assert command["terminal_evidence"]["delivery_attempted"] is True
        task = recovered.connection.execute(
            "SELECT state FROM operator_plane_wp8_background_tasks WHERE command_id=? AND child_order=1",
            (command_id,),
        ).fetchone()
        assert task is not None and task["state"] == "interrupted"
        assert recovered.deck_recovery_blocker() == "deck_recovery_hold"
    finally:
        recovered.connection.close()


def test_startup_recovery_uses_real_finite_operation_ledgers_after_provider_delivery(tmp_path):
    store, command_id = _finite_recovery_store(tmp_path)
    stamps = {"ownership_generation": 1, "board_epoch_4": 2, "board_epoch_5": 3}
    plan = compile_finite_plate_operation(
        "send_z_and_gripper_home", source_leaf_available=True,
        run_in_parallel=True, gripper_position=4000, closed_position=3000,
    )
    store.persist_wp8_plan(command_id, plan, authority_stamps=stamps)
    store.create_wp8_background_task(
        command_id, 1,
        task_id=f"{command_id}:1:{plan['plan_digest']}:finite-restart",
        task_kind="startMoveZPseudoHome", plan_digest=plan["plan_digest"],
        authority_stamps=stamps,
    )
    store.mark_wp8_background_task(
        command_id, 1, state="running", evidence={"thread_started": True},
    )
    store.connection.execute(
        "UPDATE operator_plane_lane SET owner_lease_until=0 WHERE singleton=1"
    )
    store.connection.close()

    recovered = OperatorCommandStore(tmp_path)
    try:
        command = recovered.get_command(command_id)
        assert command is not None and command["status"] == "ambiguous"
        assert command["terminal_evidence"]["delivery_attempted"] is True
        assert recovered.deck_recovery_blocker() == "deck_recovery_hold"
    finally:
        recovered.connection.close()


def test_startup_recovery_reconciles_each_durable_ledger_and_preserves_completed_tasks(tmp_path):
    store, command_id = _recovery_store(tmp_path)
    stamps = {"ownership_generation": 1, "board_epoch_4": 2, "board_epoch_5": 3}
    plan = compile_finite_plate_operation(
        "send_z_and_gripper_home", source_leaf_available=True,
        run_in_parallel=True, gripper_position=4000, closed_position=3000,
    )
    store.persist_wp8_plan(command_id, plan, authority_stamps=stamps)
    task_id = f"{command_id}:1:{plan['plan_digest']}:completed"
    store.create_wp8_background_task(command_id, 1, task_id=task_id,
        task_kind="startMoveZPseudoHome", plan_digest=plan["plan_digest"], authority_stamps=stamps)
    store.settle_wp8_background_task(task_id, state="completed",
        evidence={"result": {"ok": True}, "error": None})
    stage = store.connection.execute(
        "SELECT stage_order,operation,source_anchor,resources_json,arguments_json "
        "FROM operator_plane_deck_stages WHERE command_id=? ORDER BY stage_order LIMIT 1",
        (command_id,),
    ).fetchone()
    assert stage is not None
    store.terminalize_mov_execution_stage(
        command_id,
        DeckPlanStep(
            int(stage["stage_order"]), str(stage["operation"]), str(stage["source_anchor"]),
            tuple(json.loads(stage["resources_json"])), json.loads(stage["arguments_json"]),
        ),
        state="completed",
        result={"ok": True, "delivery_attempted": True,
                "controller_command_acknowledged": True, "controller_completion_verified": True},
    )
    wp7_plan_digest = store.connection.execute(
        "SELECT plan_digest FROM operator_plane_deck_commands WHERE command_id=?", (command_id,),
    ).fetchone()[0]
    store.record_delivery_attempt(
        command_id, work_kind="wp7_stage",
        work_identity=f"stage:{int(stage['stage_order'])}:{str(stage['operation'])}",
        plan_digest=wp7_plan_digest, authority_stamps=stamps,
    )
    store.connection.execute("UPDATE operator_plane_lane SET owner_lease_until=0 WHERE singleton=1")
    store.connection.close()

    recovered = OperatorCommandStore(tmp_path)
    try:
        command = recovered.get_command(command_id)
        assert command is not None and command["status"] == "ambiguous"
        reconciliation = command["terminal_evidence"]["ledger_reconciliation"]
        assert reconciliation["wp7_stages"][0]["terminal_state"] == "completed"
        assert reconciliation["background_tasks"][0]["state"] == "completed"
        assert "provider_completion_without_coherent_publication" in reconciliation["contradictions"]
        assert recovered.connection.execute(
            "SELECT state FROM operator_plane_wp8_background_tasks WHERE task_id=?", (task_id,),
        ).fetchone()[0] == "completed"
    finally:
        recovered.connection.close()


def test_startup_reconciles_completed_wp8_parent_with_pending_background_child(tmp_path):
    store, command_id = _recovery_store(tmp_path)
    plan = compile_finite_plate_operation(
        "send_z_and_gripper_home", source_leaf_available=True,
        run_in_parallel=True, gripper_position=4000, closed_position=3000,
    )
    stamps = {"ownership_generation": 1, "board_epoch_4": 2, "board_epoch_5": 3}
    store.persist_wp8_plan(command_id, plan, authority_stamps=stamps)
    task_id = f"{command_id}:3:{plan['plan_digest']}:pending"
    store.create_wp8_background_task(
        command_id, 3, task_id=task_id, task_kind="startGripperHomeAndUnlock",
        plan_digest=plan["plan_digest"], authority_stamps=stamps,
    )
    store.mark_wp8_background_task(
        command_id, 3, state="running", evidence={"background_pending": True},
    )
    store.finish(
        command_id, status="completed",
        payload={"ok": True, "background_pending": True},
    )
    store.connection.execute(
        "UPDATE operator_plane_lane SET owner_lease_until=0 WHERE singleton=1"
    )
    store.connection.close()

    recovered = OperatorCommandStore(tmp_path)
    try:
        receipt = recovered.get_command(command_id)
        assert receipt is not None and receipt["status"] == "completed"
        task = recovered.connection.execute(
            "SELECT state FROM operator_plane_wp8_background_tasks WHERE task_id=?", (task_id,),
        ).fetchone()
        assert task is not None and task["state"] == "interrupted"
        assert recovered.deck_recovery_blocker() == "deck_recovery_hold"
        deck = recovered.connection.execute(
            "SELECT ambiguity_state FROM operator_plane_deck_commands WHERE command_id=?", (command_id,),
        ).fetchone()
        assert deck is not None and deck["ambiguity_state"] == "recovery_required"
    finally:
        recovered.connection.close()


def test_startup_reconciles_ambiguous_wp8_parent_with_pending_background_child(tmp_path):
    store, command_id = _recovery_store(tmp_path)
    task_id = _pending_wp8_parent(store, command_id)
    store.finish(
        command_id, status="ambiguous",
        payload={"error": "dispatcher_exception", "outcome_unknown": True},
    )
    store.connection.execute(
        "UPDATE operator_plane_lane SET owner_lease_until=0 WHERE singleton=1"
    )
    store.connection.close()

    recovered = OperatorCommandStore(tmp_path)
    try:
        receipt = recovered.get_command(command_id)
        assert receipt is not None and receipt["status"] == "ambiguous"
        task = recovered.connection.execute(
            "SELECT state FROM operator_plane_wp8_background_tasks WHERE task_id=?", (task_id,),
        ).fetchone()
        assert task is not None and task["state"] == "interrupted"
        assert recovered.deck_recovery_blocker() == "deck_recovery_hold"
    finally:
        recovered.connection.close()


def _pending_wp8_parent(store: OperatorCommandStore, command_id: str):
    plan = compile_finite_plate_operation(
        "send_z_and_gripper_home", source_leaf_available=True,
        run_in_parallel=True, gripper_position=4000, closed_position=3000,
    )
    stamps = {"ownership_generation": 1, "board_epoch_4": 2, "board_epoch_5": 3}
    store.persist_wp8_plan(command_id, plan, authority_stamps=stamps)
    task_id = f"{command_id}:3:{plan['plan_digest']}:pending-parent"
    store.create_wp8_background_task(
        command_id, 3, task_id=task_id, task_kind="startGripperHomeAndUnlock",
        plan_digest=plan["plan_digest"], authority_stamps=stamps,
    )
    store.mark_wp8_background_task(
        command_id, 3, state="running", evidence={"background_pending": True},
    )
    return task_id


def test_wp8_parent_remains_issued_pending_until_background_success(tmp_path):
    store, command_id = _recovery_store(tmp_path)
    task_id = _pending_wp8_parent(store, command_id)

    pending = store.resolve_wp8_parent_after_background(
        command_id,
        payload={"response": {"ok": True, "background_pending": True}, "delivery_attempted": True},
    )
    assert pending["status"] == "issued_pending"

    store.settle_wp8_background_task(
        task_id, state="completed", evidence={"result": {"ok": True}, "error": None},
    )
    completed = store.get_command(command_id)
    assert completed is not None and completed["status"] == "completed"
    store.connection.close()


def test_wp8_late_background_failure_terminalizes_pending_parent_as_ambiguous(tmp_path):
    store, command_id = _recovery_store(tmp_path)
    task_id = _pending_wp8_parent(store, command_id)
    store.resolve_wp8_parent_after_background(
        command_id,
        payload={"response": {"ok": True, "background_pending": True}, "delivery_attempted": True},
    )

    store.settle_wp8_background_task(
        task_id,
        state="failed",
        evidence={"result": {"ok": False}, "error": {"type": "NestedOperationFailure"}},
    )

    failed = store.get_command(command_id)
    assert failed is not None and failed["status"] == "ambiguous"
    assert failed["terminal_evidence"]["background_terminal_state"] == "failed"
    assert store.deck_recovery_blocker() == "deck_recovery_hold"
    store.connection.close()


def test_stale_wp8_background_callback_cannot_settle_after_lane_takeover(tmp_path):
    store, command_id = _recovery_store(tmp_path)
    task_id = _pending_wp8_parent(store, command_id)
    store.resolve_wp8_parent_after_background(
        command_id,
        payload={"response": {"ok": True, "background_pending": True}, "delivery_attempted": True},
    )
    store.connection.execute(
        "UPDATE operator_plane_lane SET owner_id='replacement-owner',owner_lease_until=9999999999 "
        "WHERE singleton=1"
    )

    with pytest.raises(sqlite3.DatabaseError):
        with store._transaction() as conn:
            conn.execute(
                "UPDATE operator_plane_wp8_background_tasks "
                "SET state='completed',evidence_json='{}',updated_at=9999999999 WHERE task_id=?",
                (task_id,),
            )
    with pytest.raises(RuntimeError, match="background task terminal authority is stale"):
        store.settle_wp8_background_task(
            task_id, state="completed", evidence={"result": {"ok": True}, "error": None},
        )

    assert store.connection.execute(
        "SELECT state FROM operator_plane_wp8_background_tasks WHERE task_id=?", (task_id,),
    ).fetchone()[0] == "running"
    assert store.get_command(command_id)["status"] == "issued_pending"
    store.connection.execute(
        "UPDATE operator_plane_lane SET owner_lease_until=0 WHERE singleton=1"
    )
    store.connection.close()

    recovered = OperatorCommandStore(tmp_path)
    try:
        assert recovered.get_command(command_id)["status"] == "ambiguous"
        assert recovered.connection.execute(
            "SELECT state FROM operator_plane_wp8_background_tasks WHERE task_id=?", (task_id,),
        ).fetchone()[0] == "interrupted"
    finally:
        recovered.connection.close()


def test_wp8_background_task_creation_rejects_stale_dispatch_authority(tmp_path):
    store, command_id = _recovery_store(tmp_path)
    store.connection.execute(
        "UPDATE operator_plane_lane SET owner_lease_until=0 WHERE singleton=1"
    )
    with pytest.raises(RuntimeError, match="background task dispatch authority is stale"):
        store.create_wp8_background_task(
            command_id, 0, task_id=f"{command_id}:0:{'e' * 64}:stale",
            task_kind="startGripperHomeAndUnlock", plan_digest="e" * 64,
            authority_stamps={
                "ownership_generation": 1, "board_epoch_4": 2, "board_epoch_5": 3,
            },
        )
    assert store.connection.execute(
        "SELECT COUNT(*) FROM operator_plane_wp8_background_tasks WHERE command_id=?",
        (command_id,),
    ).fetchone()[0] == 0
    store.connection.close()
