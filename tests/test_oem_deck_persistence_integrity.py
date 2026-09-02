from __future__ import annotations

from dataclasses import replace
import json
import sqlite3
import threading

import pytest

from bioxp.operator_command_plane import OperatorCommandStore
from bioxp.oem_deck_movement import (
    ClassMoveToIntent,
    MovExecutionChild,
    OEM_MOVABLE_OBJECT_DEFAULT_LOCATIONS,
    compile_finite_plate_operation,
    compile_mov_execution,
)
import bioxp.oem_runtime_store as runtime_store_module
from bioxp.oem_deck_schema_v6 import verify_deck_schema_v6
from bioxp.oem_runtime_store import (
    OEMRuntimeStore,
    OEM_DECK_SCHEMA_VERSION,
    canonical_runtime_migration_registry,
    verify_canonical_runtime_database,
)


def _store(root) -> OperatorCommandStore:
    OEMRuntimeStore(root).close()
    return OperatorCommandStore(root)


def test_fresh_database_has_exact_wp9_schema_manifest_and_complete_migration_ledger(tmp_path):
    store = _store(tmp_path)
    try:
        verify_canonical_runtime_database(store.connection)
        assert store.connection.execute("PRAGMA foreign_key_check").fetchall() == []
        assert int(store.connection.execute("PRAGMA user_version").fetchone()[0]) == OEM_DECK_SCHEMA_VERSION
        rows = store.connection.execute(
            "SELECT version,result,name FROM runtime_schema_migrations ORDER BY version"
        ).fetchall()
        registry = canonical_runtime_migration_registry()
        assert [(int(row[0]), str(row[1]), str(row[2])) for row in rows] == [
            (item.version, "committed", item.name) for item in registry
        ]
        required = {
            "operator_plane_evidence",
            "operator_plane_interrupt_evidence",
            "operator_plane_interrupt_attempts",
            "operator_plane_recovery_acknowledgements",
            "operator_plane_deck_commands",
            "operator_plane_deck_recovery_decisions",
            "operator_plane_mov_execution_children",
            "operator_plane_wp8_operations",
        }
        tables = {
            str(row[0]) for row in store.connection.execute(
                "SELECT name FROM sqlite_master WHERE type='table'"
            ).fetchall()
        }
        assert required <= tables
        required_support_triggers = {
            "operator_plane_evidence_no_delete",
            "operator_plane_evidence_no_update",
            "operator_plane_evidence_coherence_insert_v1",
            "operator_plane_interrupt_evidence_no_delete",
            "operator_plane_interrupt_evidence_no_update",
            "operator_plane_interrupt_evidence_coherence_insert_v1",
            "operator_plane_interrupt_attempts_no_update_v2",
            "operator_plane_interrupt_attempts_no_delete_v2",
            "operator_plane_interrupt_attempts_lineage_insert_v1",
            "operator_plane_recovery_acknowledgements_no_update_v1",
            "operator_plane_recovery_acknowledgements_no_delete_v1",
            "operator_plane_recovery_acknowledgements_authorized_insert_v2",
        }
        triggers = {
            str(row[0]) for row in store.connection.execute(
                "SELECT name FROM sqlite_master WHERE type='trigger'"
            ).fetchall()
        }
        assert required_support_triggers <= triggers
    finally:
        store.connection.close()


def test_canonical_v5_deck_schema_migrates_additively_and_preserves_semantic_state(
    tmp_path, monkeypatch,
):
    original = runtime_store_module._migrate_oem_deck_schema_v6

    def stop_after_v5(*_args, **_kwargs):
        raise RuntimeError("stop-after-v5")

    monkeypatch.setattr(runtime_store_module, "_migrate_oem_deck_schema_v6", stop_after_v5)
    with pytest.raises(RuntimeError, match="stop-after-v5"):
        OEMRuntimeStore(tmp_path)
    monkeypatch.setattr(runtime_store_module, "_migrate_oem_deck_schema_v6", original)

    connection = sqlite3.connect(tmp_path / "bioxp_runtime.db", isolation_level=None)
    connection.row_factory = sqlite3.Row
    legacy = object.__new__(OperatorCommandStore)
    legacy.connection = connection
    legacy._lock = threading.RLock()
    legacy._authority_write_depth = 0
    legacy._configure()
    with pytest.raises(RuntimeError, match="operator_plane_interrupt_history"):
        legacy._schema()
    connection.execute("DROP TABLE operator_plane_interrupt_history")
    runtime_store_module._apply_operator_command_plane_schema_v1(connection)
    trigger_sql = str(connection.execute(
        "SELECT sql FROM sqlite_master WHERE type='trigger' "
        "AND name='operator_plane_deck_semantic_state_coherence_v1'"
    ).fetchone()[0])
    connection.execute("DROP TRIGGER operator_plane_deck_semantic_state_coherence_v1")
    connection.execute(
        "UPDATE operator_plane_deck_semantic_state SET "
        "current_location='LOC_MS',current_well=17,plate_on_gantry='COVER',"
        "semantic_state_revision=9 WHERE singleton=1"
    )
    connection.execute(trigger_sql)
    global_before = {
        str(row[0]): runtime_store_module.normalize_sql_definition(row[1])
        for row in connection.execute(
            "SELECT name,sql FROM sqlite_master WHERE type='trigger' AND name IN (%s)"
            % ",".join("?" for _ in runtime_store_module.DECK_SCHEMA_V5_LEGACY_GLOBAL_TRIGGERS),
            tuple(sorted(runtime_store_module.DECK_SCHEMA_V5_LEGACY_GLOBAL_TRIGGERS)),
        )
    }
    assert set(global_before) == set(runtime_store_module.DECK_SCHEMA_V5_LEGACY_GLOBAL_TRIGGERS)
    connection.close()

    migrated = OEMRuntimeStore(tmp_path)
    migrated.close()
    store = OperatorCommandStore(tmp_path)
    try:
        row = store.connection.execute(
            "SELECT current_location,current_well,plate_on_gantry,semantic_state_revision,"
            "save_tip,old_well,old_well_text,old_location,plate_pierced_json,well_pierced_json "
            "FROM operator_plane_deck_semantic_state WHERE singleton=1"
        ).fetchone()
        assert tuple(row) == (
            "LOC_MS", 17, "COVER", 9, 0, 0, "", None, "{}", "{}",
        )
        verify_canonical_runtime_database(store.connection)
        assert store.connection.execute("PRAGMA foreign_key_check").fetchall() == []
        global_after = {
            str(row[0]): runtime_store_module.normalize_sql_definition(row[1])
            for row in store.connection.execute(
                "SELECT name,sql FROM sqlite_master WHERE type='trigger' AND name IN (%s)"
                % ",".join("?" for _ in global_before),
                tuple(sorted(global_before)),
            )
        }
        assert global_after == global_before
    finally:
        store.connection.close()


def test_v5_counterfeit_trigger_aborts_before_destructive_ddl(tmp_path, monkeypatch) -> None:
    original = runtime_store_module._migrate_oem_deck_schema_v6
    monkeypatch.setattr(
        runtime_store_module, "_migrate_oem_deck_schema_v6",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(RuntimeError("stop-after-v5")),
    )
    with pytest.raises(RuntimeError, match="stop-after-v5"):
        OEMRuntimeStore(tmp_path)
    monkeypatch.setattr(runtime_store_module, "_migrate_oem_deck_schema_v6", original)
    connection = sqlite3.connect(tmp_path / "bioxp_runtime.db", isolation_level=None)
    connection.row_factory = sqlite3.Row
    connection.create_function("authority_write_allowed", 0, lambda: 1)
    legacy = object.__new__(OperatorCommandStore)
    legacy.connection = connection
    legacy._lock = threading.RLock()
    legacy._authority_write_depth = 0
    legacy._configure()
    with pytest.raises(RuntimeError, match="operator_plane_interrupt_history"):
        legacy._schema()
    connection.execute("DROP TABLE operator_plane_interrupt_history")
    runtime_store_module._apply_operator_command_plane_schema_v1(connection)
    connection.execute("DROP TRIGGER operator_plane_deck_commands_no_delete_v1")
    connection.execute(
        "CREATE TRIGGER operator_plane_deck_commands_no_delete_v1 "
        "BEFORE DELETE ON operator_plane_deck_commands BEGIN SELECT 1; END"
    )
    before_sql = str(connection.execute(
        "SELECT sql FROM sqlite_master WHERE type='trigger' "
        "AND name='operator_plane_deck_commands_no_delete_v1'"
    ).fetchone()[0])
    destructive: list[str] = []
    connection.set_trace_callback(
        lambda sql: destructive.append(sql)
        if sql.lstrip().upper().startswith(("DROP ", "ALTER ")) else None
    )

    with pytest.raises(RuntimeError, match="v5 normalized SQL"):
        runtime_store_module._migrate_oem_deck_schema_v6(
            connection, tmp_path, runtime_store_module.canonical_runtime_migration_registry()[5],
        )

    connection.set_trace_callback(None)
    assert destructive == []
    assert connection.execute("PRAGMA user_version").fetchone()[0] == 5
    assert connection.execute(
        "SELECT sql FROM sqlite_master WHERE type='trigger' "
        "AND name='operator_plane_deck_commands_no_delete_v1'"
    ).fetchone()[0] == before_sql
    connection.close()


def test_v6_migration_identity_binds_accepted_schema_manifests(monkeypatch):
    before = runtime_store_module.oem_deck_schema_v6_migration_identity().ddl_sha256
    monkeypatch.setattr(
        runtime_store_module,
        "DECK_SCHEMA_V6_TABLES",
        runtime_store_module.DECK_SCHEMA_V6_TABLES | {"sentinel_schema_object"},
    )
    after = runtime_store_module.oem_deck_schema_v6_migration_identity().ddl_sha256
    assert after != before


def test_v6_verifier_rejects_unexpected_same_domain_object(tmp_path):
    store = _store(tmp_path)
    try:
        store.connection.execute(
            "CREATE INDEX operator_plane_deck_counterfeit_idx ON operator_plane_deck_commands(target)"
        )
        with pytest.raises(RuntimeError, match="object manifest is not exact"):
            verify_deck_schema_v6(store.connection)
    finally:
        store.connection.close()


def test_v6_verifier_attests_exact_sql_and_complete_constraint_tuples(tmp_path):
    store = _store(tmp_path)
    try:
        store.connection.execute("DROP INDEX operator_plane_deck_commands_plan_idx")
        store.connection.execute(
            "CREATE INDEX operator_plane_deck_commands_plan_idx ON operator_plane_deck_commands(target)"
        )
        with pytest.raises(RuntimeError, match="normalized SQL|constraint tuples"):
            verify_deck_schema_v6(store.connection)
    finally:
        store.connection.close()


def test_mov_execution_parallel_children_persist_canonical_join_kind(tmp_path):
    store = _store(tmp_path)
    try:
        store.bootstrap_deck_semantic_state({
            "current_location": "LOC_MS",
            "current_well": 0,
            "current_tray": None,
            "tip_loaded": False,
            "tip_dirty": False,
            "tip_location": -1,
            "clean_path": True,
            "plate_on_gantry": None,
            "movable_plate_locations": dict(OEM_MOVABLE_OBJECT_DEFAULT_LOCATIONS),
            "pseudo_z_home": 65000,
            "ownership_generation": 1,
            "board_epoch_4": 2,
            "board_epoch_5": 3,
            "latch_status": True,
            "machine_latch_closed": True,
            "latch_observation_id": "latch-wp9-children",
            "source_operation": "test_source_snapshot",
            "source_command_id": "source-wp9-children",
        })
        authority = {
            "ownership_generation": 1,
            "serial206_initialization_provider": {
                "x_authority": {"active_board_epoch": 3},
                "board4_authority": {"active_board_epoch": 2},
            },
        }
        intent = ClassMoveToIntent(script_line=1, plate_name=0)
        admitted = store.admit_internal_mov_execution(
            intent, state=authority, idempotency_key="wp9-child-plan",
        )
        assert store.claim_next() is not None
        plan = compile_mov_execution(
            intent,
            {
                "save_tip": False,
                "old_well": False,
                "old_location": 18,
                "old_well_text": "A1",
                "plate_locations": {0: 23},
                "plate_pierced": {},
                "well_pierced": {},
                "trough_version": 1,
                "authority_digest": "a" * 64,
                "board_epoch_4": 2,
                "board_epoch_5": 3,
            },
            movement_children=("moveX", "moveY"),
        )
        store.persist_mov_execution_plan(admitted["command_id"], plan)
        rows = store.connection.execute(
            "SELECT operation,join_kind FROM operator_plane_mov_execution_children "
            "WHERE command_id=? ORDER BY child_order",
            (admitted["command_id"],),
        ).fetchall()
        assert [tuple(row) for row in rows] == [
            ("moveX", "Task.WaitAll"), ("moveY", "Task.WaitAll"),
        ]
    finally:
        store.connection.close()


def test_wp7_plan_and_source_children_roll_back_as_one_transaction(tmp_path):
    store = _store(tmp_path)
    try:
        authority = {
            "ownership_generation": 1,
            "serial206_initialization_provider": {
                "x_authority": {"active_board_epoch": 3},
                "board4_authority": {"active_board_epoch": 2},
            },
        }
        intent = ClassMoveToIntent(script_line=1, plate_name=0)
        admitted = store.admit_internal_mov_execution(
            intent, state=authority, idempotency_key="wp9-atomic-plan",
        )
        assert store.claim_next() is not None
        plan = compile_mov_execution(
            intent,
            {
                "save_tip": False, "old_well": False, "old_location": 18,
                "old_well_text": "A1", "plate_locations": {0: 23},
                "plate_pierced": {}, "well_pierced": {}, "trough_version": 1,
                "authority_digest": "a" * 64, "board_epoch_4": 2, "board_epoch_5": 3,
            },
        )
        invalid_step = replace(
            plan.steps[0],
            source_children=(MovExecutionChild("moveX", {}),),
            join="invalid_join",
        )
        invalid_plan = replace(plan, steps=(invalid_step, *plan.steps[1:]))

        with pytest.raises(sqlite3.IntegrityError):
            store.persist_mov_execution_plan(admitted["command_id"], invalid_plan)

        for table in (
            "operator_plane_deck_commands",
            "operator_plane_deck_stages",
            "operator_plane_mov_execution_children",
        ):
            assert store.connection.execute(
                f"SELECT COUNT(*) FROM {table} WHERE command_id=?",
                (admitted["command_id"],),
            ).fetchone()[0] == 0
    finally:
        store.connection.close()


def test_wp8_background_and_delivery_lineage_is_database_enforced(tmp_path):
    store = _store(tmp_path)
    try:
        with pytest.raises(sqlite3.DatabaseError):
            store.connection.execute(
                "INSERT INTO operator_plane_delivery_attempts("
                "command_id,work_kind,work_identity,dispatch_attempt_id,plan_digest,owner_id,"
                "ownership_generation,board_epoch_4,board_epoch_5,created_at) "
                "VALUES('missing','wp8_child','0','attempt',?,'owner',1,2,3,1.0)",
                ("a" * 64,),
            )
        with pytest.raises(sqlite3.DatabaseError):
            store.connection.execute(
                "INSERT INTO operator_plane_wp8_background_tasks("
                "command_id,child_order,task_id,task_kind,plan_digest,dispatch_attempt_id,"
                "authority_stamps_json,state,evidence_json,created_at,updated_at) "
                "VALUES('missing',0,'task','kind',?,'attempt','{}','created','{}',1.0,1.0)",
                ("a" * 64,),
            )

        store.bootstrap_deck_semantic_state({
            "current_location": "LOC_MS", "current_well": 0, "current_tray": None,
            "tip_loaded": False, "tip_dirty": False, "tip_location": -1, "clean_path": True,
            "plate_on_gantry": None,
            "movable_plate_locations": dict(OEM_MOVABLE_OBJECT_DEFAULT_LOCATIONS),
            "pseudo_z_home": 65000, "ownership_generation": 1,
            "board_epoch_4": 2, "board_epoch_5": 3,
            "latch_status": True, "machine_latch_closed": True,
            "latch_observation_id": "latch-wp9-lineage",
            "source_operation": "test_source_snapshot",
            "source_command_id": "source-wp9-lineage",
        })
        authority = {
            "ownership_generation": 1,
            "serial206_initialization_provider": {
                "x_authority": {"active_board_epoch": 3},
                "board4_authority": {"active_board_epoch": 2},
            },
        }
        admitted = store.admit_internal_wp8_operation(
            "send_z_and_gripper_home", inputs={}, state=authority,
            idempotency_key="wp9-background-immutability",
        )
        assert store.claim_next() is not None
        plan = compile_finite_plate_operation(
            "send_z_and_gripper_home", source_leaf_available=True,
            run_in_parallel=True, gripper_position=4000, closed_position=3000,
        )
        stamps = {"ownership_generation": 1, "board_epoch_4": 2, "board_epoch_5": 3}
        store.persist_wp8_plan(admitted["command_id"], plan, authority_stamps=stamps)
        dispatch_attempt_id = store.connection.execute(
            "SELECT dispatch_attempt_id FROM operator_plane_commands WHERE command_id=?",
            (admitted["command_id"],),
        ).fetchone()[0]
        for work_kind, work_identity, plan_digest in (
            ("wp8_background_task", "child:3:startGripperHomeAndUnlock", "f" * 64),
            ("wp8_background_task", "child:3:catchPlate", plan["plan_digest"]),
            ("wp8_child", "child:3:startGripperHomeAndUnlock", plan["plan_digest"]),
        ):
            with pytest.raises(sqlite3.DatabaseError):
                with store._transaction() as conn:
                    conn.execute(
                        "INSERT INTO operator_plane_delivery_attempts("
                        "command_id,work_kind,work_identity,dispatch_attempt_id,plan_digest,owner_id,"
                        "ownership_generation,board_epoch_4,board_epoch_5,created_at) VALUES(?,?,?,?,?,?,?,?,?,?)",
                        (
                            admitted["command_id"], work_kind, work_identity,
                            dispatch_attempt_id, plan_digest, store.owner_id, 1, 2, 3, 1.0,
                        ),
                    )
        task_id = f"{admitted['command_id']}:3:{plan['plan_digest']}:immutable"
        store.create_wp8_background_task(
            admitted["command_id"], 3, task_id=task_id,
            task_kind="startGripperHomeAndUnlock", plan_digest=plan["plan_digest"],
            authority_stamps=stamps,
        )
        task_columns = {
            row[1] for row in store.connection.execute(
                "PRAGMA table_info(operator_plane_wp8_background_tasks)"
            ).fetchall()
        }
        assert "delivery_attempt_sequence" in task_columns
        linked_delivery = store.connection.execute(
            "SELECT task.delivery_attempt_sequence,attempt.work_kind,attempt.work_identity "
            "FROM operator_plane_wp8_background_tasks task "
            "JOIN operator_plane_delivery_attempts attempt "
            "ON attempt.attempt_sequence=task.delivery_attempt_sequence WHERE task.task_id=?",
            (task_id,),
        ).fetchone()
        assert linked_delivery is not None and linked_delivery[0] > 0
        assert tuple(linked_delivery[1:]) == (
            "wp8_background_task", "child:3:startGripperHomeAndUnlock",
        )
        attempt_count = store.connection.execute(
            "SELECT COUNT(*) FROM operator_plane_delivery_attempts WHERE command_id=?",
            (admitted["command_id"],),
        ).fetchone()[0]
        with pytest.raises(sqlite3.DatabaseError):
            store.create_wp8_background_task(
                admitted["command_id"], 1, task_id=task_id,
                task_kind="startMoveZPseudoHome", plan_digest=plan["plan_digest"],
                authority_stamps=stamps,
            )
        assert store.connection.execute(
            "SELECT COUNT(*) FROM operator_plane_delivery_attempts WHERE command_id=?",
            (admitted["command_id"],),
        ).fetchone()[0] == attempt_count
        child_one_marker = store.record_delivery_attempt(
            admitted["command_id"], work_kind="wp8_background_task",
            work_identity="child:1:startMoveZPseudoHome",
            plan_digest=plan["plan_digest"], authority_stamps=stamps,
        )
        for task_kind, dispatch_id, forged_stamps in (
            ("startGripperHomeAndUnlock", dispatch_attempt_id, stamps),
            ("startMoveZPseudoHome", "wrong-dispatch", stamps),
            ("startMoveZPseudoHome", dispatch_attempt_id,
             {"ownership_generation": 9, "board_epoch_4": 2, "board_epoch_5": 3}),
        ):
            with pytest.raises(sqlite3.DatabaseError):
                with store._transaction() as conn:
                    conn.execute(
                        "INSERT INTO operator_plane_wp8_background_tasks("
                        "command_id,child_order,task_id,task_kind,plan_digest,dispatch_attempt_id,"
                        "delivery_attempt_sequence,authority_stamps_json,state,evidence_json,created_at,updated_at) "
                        "VALUES(?,?,?,?,?,?,?,?,'created','{}',1.0,1.0)",
                        (
                            admitted["command_id"], 1,
                            f"forged-{task_kind}-{dispatch_id}-{forged_stamps['ownership_generation']}",
                            task_kind, plan["plan_digest"], dispatch_id,
                            child_one_marker["attempt_sequence"], json.dumps(forged_stamps),
                        ),
                    )
        with pytest.raises(sqlite3.DatabaseError):
            store.connection.execute(
                "UPDATE operator_plane_wp8_background_tasks SET evidence_json='{\"forged\":true}' WHERE task_id=?",
                (task_id,),
            )
        store.settle_wp8_background_task(
            task_id, state="completed", evidence={"result": {"ok": True}},
        )
        with pytest.raises(sqlite3.DatabaseError):
            store.connection.execute(
                "UPDATE operator_plane_wp8_background_tasks SET state='failed' WHERE task_id=?",
                (task_id,),
            )
        with pytest.raises(sqlite3.DatabaseError):
            store.connection.execute(
                "UPDATE operator_plane_wp8_background_tasks SET task_id='rebound' WHERE task_id=?",
                (task_id,),
            )
    finally:
        store.connection.close()


def test_direct_sql_rejects_forged_command_plan_stage_and_recovery_rows(tmp_path):
    store = _store(tmp_path)
    try:
        with pytest.raises(sqlite3.DatabaseError):
            store.connection.execute(
                "INSERT INTO operator_plane_deck_commands("
                "command_id,target,target_label,resolved_location_id,destination_catalog_revision,"
                "position_table_revision,authority_snapshot_digest,complete_authority_digest,plan_digest,"
                "source_branch,source_anchors_json,source_hazards_json,planned_at) "
                "VALUES('forged','LOC_MS','Manual Scan',1,?,?,?,?,?,'ordinary','[]','[]',1.0)",
                ("0" * 64, "1" * 64, "2" * 64, "3" * 64, "4" * 64),
            )
        with pytest.raises(sqlite3.DatabaseError):
            store.connection.execute(
                "INSERT INTO operator_plane_deck_recovery_decisions("
                "decision_id,command_id,stream_sequence,dispatch_attempt_id,recovery_epoch,"
                "plan_digest,authority_snapshot_digest,position_table_revision,destination_catalog_revision,"
                "controller_observation_id,decision_json,receipt_json,created_at) "
                "VALUES('forged','forged',1,'attempt',1,?,?,?,?,?,'{}','{}',1.0)",
                ("0" * 64, "1" * 64, "2" * 64, "3" * 64, "observation"),
            )
        with pytest.raises(sqlite3.DatabaseError):
            store.connection.execute(
                "DELETE FROM operator_plane_deck_semantic_state WHERE singleton=1"
            )
    finally:
        store.connection.close()
