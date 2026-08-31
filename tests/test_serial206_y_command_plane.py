import asyncio
import hashlib
import json
import sqlite3
import threading

import pytest

pytest.skip(
    "retired duplicate operator mutation/scheduler authority",
    allow_module_level=True,
)


def _state(*, generation=4, board4_epoch=7, board5_epoch=11):
    return {
        "ownership_generation": generation,
        "serial206_initialization_provider": {
            "x_authority": {"active_board_epoch": board5_epoch},
            "y_authority": {
                "board_authority": {"active_board_epoch": board4_epoch},
                "live_status": {"position_steps": 1000},
            },
            "z_authority": {"state": "referenced_ready", "reference_state": "referenced"},
        },
    }


def test_additive_interrupt_history_schema_is_rebuilt_without_data_loss(tmp_path):
    initial = OperatorCommandStore(tmp_path)
    initial.stop()
    initial.connection.close()
    connection = sqlite3.connect(tmp_path / "bioxp_runtime.db")
    connection.execute("PRAGMA foreign_keys=OFF")
    receipt = {"interrupt_attempt_id": "attempt-1", "status": "stopped"}
    wrapper = {"receipt": receipt, "stream": "y"}
    receipt_json = json.dumps(receipt, sort_keys=True, separators=(",", ":"))
    wrapper_json = json.dumps(wrapper, sort_keys=True, separators=(",", ":"))
    record_sha256 = hashlib.sha256(wrapper_json.encode("utf-8")).hexdigest()
    connection.executescript(
        """
        BEGIN IMMEDIATE;
        ALTER TABLE operator_plane_interrupt_history
            RENAME TO operator_plane_interrupt_history_canonical;
        CREATE TABLE operator_plane_interrupt_history (
            record_sha256 TEXT PRIMARY KEY,
            stream TEXT NOT NULL CHECK(stream IN ('x','y','z')),
            interrupt_attempt_id TEXT NOT NULL,
            receipt_json TEXT NOT NULL CHECK(json_valid(receipt_json)),
            imported_at REAL NOT NULL,
            source_wrapper_json TEXT,
            UNIQUE(stream,interrupt_attempt_id)
        ) WITHOUT ROWID;
        DROP TABLE operator_plane_interrupt_history_canonical;
        COMMIT;
        """
    )
    connection.execute(
        "INSERT INTO operator_plane_interrupt_history("
        "record_sha256,stream,interrupt_attempt_id,receipt_json,imported_at,source_wrapper_json"
        ") VALUES(?,?,?,?,?,?)",
        (record_sha256, "y", "attempt-1", receipt_json, 1.0, wrapper_json),
    )
    connection.commit()
    connection.close()

    migrated = OperatorCommandStore(tmp_path)

    columns = tuple(
        row[1]
        for row in migrated.connection.execute(
            "PRAGMA table_info(operator_plane_interrupt_history)"
        ).fetchall()
    )
    row = migrated.connection.execute(
        "SELECT record_sha256,stream,interrupt_attempt_id,receipt_json,source_wrapper_json,imported_at "
        "FROM operator_plane_interrupt_history"
    ).fetchone()
    assert columns == (
        "record_sha256", "stream", "interrupt_attempt_id", "receipt_json",
        "source_wrapper_json", "imported_at",
    )
    assert tuple(row) == (record_sha256, "y", "attempt-1", receipt_json, wrapper_json, 1.0)
    migrated.stop()
    migrated.connection.close()


def test_command_store_accepts_typed_y_actions_and_bounds(tmp_path):
    assert _validate_inputs("oem.y.move_steps", {"steps": 20}) == {"steps": 20}
    assert _validate_inputs("oem.y.move_absolute", {"target_steps": 102957}) == {"target_steps": 102957}
    with pytest.raises(HTTPException):
        _validate_inputs("oem.y.move_absolute", {"target_steps": 2**31})
    with pytest.raises(HTTPException):
        _validate_inputs("oem.y.move_steps", {"steps": 1, "speed": 1800})


def test_y_absolute_terminality_requires_event_128_position_and_speed_zero():
    assert _y_absolute_terminal_disposition(
        completion_class="event_128",
        event_128=True,
        target_steps=400,
        observed_position_steps=400,
        terminal_speed_zero=True,
    ) == "completed"
    assert _y_absolute_terminal_disposition(
        completion_class="oem_timeout_target_equal",
        event_128=False,
        target_steps=400,
        observed_position_steps=400,
        terminal_speed_zero=True,
    ) == "completed"
    assert _y_absolute_terminal_disposition(
        completion_class="event_128",
        event_128=True,
        target_steps=400,
        observed_position_steps=400,
        terminal_speed_zero=False,
    ) == "failed"
    assert _y_absolute_terminal_disposition(
        completion_class="event_128",
        event_128=True,
        target_steps=400,
        observed_position_steps=399,
        terminal_speed_zero=True,
    ) == "failed"


def test_y_epoch_is_observational_and_does_not_fence_dispatch(tmp_path):
    state = _state()
    assert _active_board_epochs(state, "oem.y.move_absolute") == {"4": 7}
    store = OperatorCommandStore(tmp_path)
    request = {
        "schema_version": "bioxp.operator_action_request.v2",
        "action_id": "oem.y.move_absolute",
        "inputs": {"target_steps": 1200},
        "expected_ownership_generation": 4,
        "expected_board_epoch_by_board": {"4": 7},
        "idempotency_key": "strict-y-epoch-1",
    }
    admitted = store.admit_command(request, state=state)
    replay = store.admit_command(request, state=state)
    assert replay["command_id"] == admitted["command_id"]
    assert replay["idempotent_replay"] is True
    with pytest.raises(HTTPException, match="idempotency"):
        store.admit_command({**request, "expected_board_epoch_by_board": {"4": 8}}, state=_state(board4_epoch=8))
    with store._authority_write():
        store.connection.execute(
            "UPDATE serial206_board_authority SET state='active',active_board_epoch=8 WHERE board_id=4"
        )
    claimed = store.claim_next()
    assert claimed is not None
    assert claimed["command_id"] == admitted["command_id"]
    current = store.get_command(admitted["command_id"])
    assert current is not None
    assert current["status"] == "dispatched"


def test_y_epoch_map_is_optional_observational_metadata(tmp_path):
    store = OperatorCommandStore(tmp_path)
    base = {
        "schema_version": "bioxp.operator_action_request.v2",
        "action_id": "oem.y.move_absolute",
        "inputs": {"target_steps": 1200},
        "expected_ownership_generation": 4,
    }
    for index, epochs in enumerate(({}, {"4": 7, "5": 11})):
        admitted = store.admit_command(
            {
                **base,
                "idempotency_key": f"observational-y-prepare-{index}",
                "expected_board_epoch_by_board": epochs,
            },
            state=_state(),
        )
        assert admitted["status"] == "queued"


def test_y_safety_epoch_is_persisted_and_y_stop_does_not_clear_z(tmp_path):
    store = OperatorCommandStore(tmp_path)
    before = store.connection.execute("SELECT x_epoch,y_epoch,z_epoch FROM operator_plane_safety WHERE singleton=1").fetchone()
    assert tuple(before) == (0, 0, 0)
    receipt = store.begin_interrupt(
        "oem.y.stop",
        state={"ownership_generation": 4, "serial206_initialization_provider": {"y_authority": {"live_status": {"position_steps": 1000}}, "z_authority": {"state": "referenced_ready", "reference_state": "referenced"}}},
        request={"idempotency_key": "y-stop-key-001", "expected_generation": 4, "controller_stop_delivered": True},
    )
    assert receipt["scope"] == "y"
    after = store.connection.execute("SELECT x_epoch,y_epoch,z_epoch FROM operator_plane_safety WHERE singleton=1").fetchone()
    assert tuple(after) == (0, 1, 0)
    axes = store.connection.execute(
        "SELECT axis,lifecycle_state,reference_state,interrupt_epoch FROM serial206_axis_authority ORDER BY axis"
    ).fetchall()
    by_axis = {row[0]: tuple(row[1:]) for row in axes}
    assert by_axis["y"] == ("unprepared", "unreferenced", 1)
    assert by_axis["z"] == ("unprepared", "unreferenced", 0)
    assert by_axis["gripper"] == ("unprepared", "unreferenced", 0)
    store.stop()


def test_aggregate_abort_invalidates_all_board4_members(tmp_path):
    store = OperatorCommandStore(tmp_path)
    store.begin_interrupt(
        "oem.abort_all",
        state=_state(),
        request={"idempotency_key": "aggregate-abort-1", "expected_generation": 4},
    )
    rows = store.connection.execute(
        "SELECT axis,lifecycle_state,reference_state,interrupt_epoch FROM serial206_axis_authority ORDER BY axis"
    ).fetchall()
    assert {row[0]: tuple(row[1:]) for row in rows} == {
        "gripper": ("reconciliation_required", "reconciliation_required", 1),
        "y": ("reconciliation_required", "reconciliation_required", 1),
        "z": ("reconciliation_required", "reconciliation_required", 1),
    }


def test_xy_method_accepts_missing_board_epochs_as_observational_metadata(tmp_path):
    store = OperatorCommandStore(tmp_path)
    state = _state()
    del state["serial206_initialization_provider"]["x_authority"]["active_board_epoch"]
    del state["serial206_initialization_provider"]["y_authority"]["board_authority"]["active_board_epoch"]
    request = {
        "schema_version": "bioxp.operator_method_request.v1",
        "name": "oem.xy.home",
        "idempotency_key": "strict-xy-missing-epochs",
        "expected_ownership_generation": 4,
        "expected_board_epoch_by_board": {},
        "failure_policy": "fail_fast",
        "steps": [{"action_id": "oem.xy.home", "inputs": {}, "repeat": 1}],
        "metadata": {"method_action_id": "oem.xy.home"},
    }
    admitted = store.admit_method(request, state=state)
    assert admitted["status"] == "queued"


def test_xy_method_epochs_are_idempotent_observations_and_do_not_fence_dispatch(tmp_path):
    store = OperatorCommandStore(tmp_path)
    request = {
        "schema_version": "bioxp.operator_method_request.v1",
        "name": "oem.xy.move_absolute",
        "idempotency_key": "strict-xy-method-1",
        "expected_ownership_generation": 4,
        "expected_board_epoch_by_board": {"4": 7, "5": 11},
        "failure_policy": "fail_fast",
        "steps": [
            {"action_id": "oem.xy.move_absolute", "inputs": {"x": 200, "y": 300}, "repeat": 1},
            {"action_id": "oem.xy.home", "inputs": {}, "repeat": 1},
        ],
        "metadata": {"method_action_id": "oem.xy.move_absolute"},
    }
    admitted = store.admit_method(request, state=_state())
    first_child = store.connection.execute(
        "SELECT command_id,expected_board_epochs_json FROM serial206_movement_commands WHERE method_id=? ORDER BY method_order LIMIT 1",
        (admitted["method_id"],),
    ).fetchone()
    assert json.loads(first_child[1]) == {}
    assert [row[0] for row in store.connection.execute(
        "SELECT resource_key FROM serial206_command_resources WHERE command_id=? ORDER BY resource_key",
        (first_child[0],),
    )] == ["axis:x", "axis:y", "motor:4:0", "motor:5:0"]
    assert tuple(store.connection.execute(
        "SELECT state,active_board_epoch FROM operator_plane_board_authority WHERE board_id=5"
    ).fetchone()) == ("faulted", None)
    with pytest.raises(HTTPException, match="idempotency"):
        store.admit_method(
            {**request, "expected_board_epoch_by_board": {"4": 8, "5": 11}},
            state=_state(board4_epoch=8),
        )
    with store._authority_write():
        store.connection.execute(
            "UPDATE serial206_board_authority SET state='active',active_board_epoch=8 WHERE board_id=4"
        )
    claimed = store.claim_next()
    assert claimed is not None
    assert claimed["command_id"] == first_child[0]
    method = store.connection.execute(
        "SELECT status FROM operator_plane_methods WHERE method_id=?", (admitted["method_id"],)
    ).fetchone()
    children = store.connection.execute(
        "SELECT command_id,status FROM operator_plane_commands WHERE method_id=? ORDER BY method_sequence", (admitted["method_id"],)
    ).fetchall()
    assert method[0] == "running"
    assert [row[1] for row in children] == ["dispatched", "queued"]


def test_operator_y_json_fallback_is_retired(tmp_path):
    store = OperatorCommandStore(tmp_path)
    with pytest.raises(RuntimeError, match="SQLite persistence is required"):
        store.append_y_interrupt_fallback(
            {"interrupt_attempt_id": "y-fallback-attempt-1"},
            reason="forced-test",
        )




def test_y_command_is_queued_with_canonical_inputs(tmp_path):
    store = OperatorCommandStore(tmp_path)
    response = store.admit_command(
        {
            "schema_version": "bioxp.operator_action_request.v2",
            "action_id": "oem.y.move_steps",
            "inputs": {"steps": 100},
            "expected_ownership_generation": 4,
            "idempotency_key": "y-command-key-001",
            "expected_board_epoch_by_board": {"4": 7},
        },
        state=_state(),
    )
    assert response["status"] == "queued"
    assert response["action_id"] == "oem.y.move_steps"
    canonical = store.connection.execute(
        "SELECT action_id,state,axis_scope,board_scope_json FROM serial206_movement_commands WHERE command_id=?",
        (response["command_id"],),
    ).fetchone()
    assert tuple(canonical[:3]) == ("oem.y.move_steps", "queued", "y")
    assert store.connection.execute(
        "SELECT resource_key FROM serial206_command_resources WHERE command_id=? ORDER BY resource_key",
        (response["command_id"],),
    ).fetchall()
    store.stop()


def test_canonical_y_claim_and_finish_follow_closed_states(tmp_path):
    store = OperatorCommandStore(tmp_path)
    admitted = store.admit_command(
        {
            "schema_version": "bioxp.operator_action_request.v2",
            "action_id": "oem.y.move_steps",
            "inputs": {"steps": 100},
            "expected_ownership_generation": 4,
            "idempotency_key": "y-command-key-002",
            "expected_board_epoch_by_board": {"4": 7},
        },
        state=_state(),
    )
    claimed = store.claim_next()
    assert claimed is not None
    assert claimed["command_id"] == admitted["command_id"]
    row = store.connection.execute("SELECT state FROM serial206_movement_commands WHERE command_id=?", (claimed["command_id"],)).fetchone()
    assert row[0] == "dispatched"
    other = store.admit_command(
        {
            "schema_version": "bioxp.operator_action_request.v2",
            "action_id": "oem.z.move_steps",
            "inputs": {"steps": 100},
            "expected_ownership_generation": 4,
            "idempotency_key": "z-command-key-002",
        },
        state={"ownership_generation": 4, "serial206_initialization_provider": {"y_authority": {"live_status": {"position_steps": 1000}}, "z_authority": {"state": "referenced_ready", "reference_state": "referenced"}}},
    )
    concurrent_claim = store.claim_next()
    assert concurrent_claim is not None
    assert concurrent_claim["command_id"] == other["command_id"]
    store.finish(claimed["command_id"], status="completed", payload={"completion_class": "event_128"})
    row = store.connection.execute("SELECT state FROM serial206_movement_commands WHERE command_id=?", (claimed["command_id"],)).fetchone()
    assert row[0] == "completed"



def test_nonblocking_y_absolute_keeps_command_dispatched(tmp_path):
    store = OperatorCommandStore(tmp_path)
    admitted = store.admit_command(
        {
            "schema_version": "bioxp.operator_action_request.v2",
            "action_id": "oem.y.move_absolute",
            "inputs": {"target_steps": 100},
            "expected_ownership_generation": 4,
            "idempotency_key": "y-command-key-003",
            "expected_board_epoch_by_board": {"4": 7},
        },
        state=_state(),
    )
    claimed = store.claim_next()
    assert claimed is not None
    store.mark_dispatched(claimed["command_id"], payload={"pending": True})
    row = store.connection.execute("SELECT status FROM operator_plane_commands WHERE command_id=?", (admitted["command_id"],)).fetchone()
    assert row[0] == "issued_pending"
    store.stop()
