import asyncio
import json
import sqlite3
import threading

import pytest
from fastapi import FastAPI
from fastapi import HTTPException

from src.bioxp.operator_command_plane import (
    OperatorCommandStore,
    OperatorCommandPlane,
    _active_board_epochs,
    _validate_inputs,
    _y_absolute_terminal_disposition,
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


def test_command_store_accepts_typed_y_actions_and_bounds(tmp_path):
    assert _validate_inputs("oem.y.move_steps", {"steps": 20}) == {"steps": 20}
    assert _validate_inputs("oem.y.move_absolute", {"target_steps": 102956}) == {"target_steps": 102956}
    with pytest.raises(HTTPException):
        _validate_inputs("oem.y.move_absolute", {"target_steps": 102957})
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
    ) == "ambiguous"
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


def test_strict_y_epoch_is_extracted_required_replayed_and_fenced_before_dispatch(tmp_path):
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
    store.connection.execute(
        "UPDATE serial206_board_authority SET state='active',active_board_epoch=8 WHERE board_id=4"
    )
    assert store.claim_next() is None
    row = store.connection.execute(
        "SELECT status,terminal_json FROM operator_plane_commands WHERE command_id=?",
        (admitted["command_id"],),
    ).fetchone()
    assert row[0] == "failed"
    assert "board_epoch_changed_before_dispatch" in row[1]


def test_strict_y_epoch_map_must_be_exact_board4(tmp_path):
    store = OperatorCommandStore(tmp_path)
    base = {
        "schema_version": "bioxp.operator_action_request.v2",
        "action_id": "oem.y.prepare",
        "inputs": {},
        "expected_ownership_generation": 4,
        "idempotency_key": "strict-y-prepare-1",
    }
    for epochs in ({}, {"4": 7, "5": 11}):
        with pytest.raises(HTTPException, match="board_epoch"):
            store.admit_command({**base, "expected_board_epoch_by_board": epochs}, state=_state())


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
    assert by_axis["y"] == ("reconciliation_required", "reconciliation_required", 1)
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


def test_xy_method_requires_exact_board4_and_board5_epochs(tmp_path):
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
    with pytest.raises(HTTPException, match="board_epoch"):
        store.admit_method(request, state=state)


def test_strict_xy_method_epochs_are_idempotent_and_parent_fails_transactionally(tmp_path):
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
    assert json.loads(first_child[1]) == {"4": 7, "5": 11}
    assert [row[0] for row in store.connection.execute(
        "SELECT resource_key FROM serial206_command_resources WHERE command_id=? ORDER BY resource_key",
        (first_child[0],),
    )] == ["axis:x", "axis:y", "motor:4:0", "motor:5:0"]
    assert tuple(store.connection.execute(
        "SELECT state,active_board_epoch FROM operator_plane_board_authority WHERE board_id=5"
    ).fetchone()) == ("active", 11)
    with pytest.raises(HTTPException, match="idempotency"):
        store.admit_method(
            {**request, "expected_board_epoch_by_board": {"4": 8, "5": 11}},
            state=_state(board4_epoch=8),
        )
    store.connection.execute(
        "UPDATE serial206_board_authority SET state='active',active_board_epoch=8 WHERE board_id=4"
    )
    assert store.claim_next() is None
    method = store.connection.execute(
        "SELECT status FROM operator_plane_methods WHERE method_id=?", (admitted["method_id"],)
    ).fetchone()
    children = store.connection.execute(
        "SELECT command_id,status FROM operator_plane_commands WHERE method_id=? ORDER BY method_sequence", (admitted["method_id"],)
    ).fetchall()
    assert method[0] == "failed"
    assert [row[1] for row in children] == ["failed", "cancelled"]
    sibling_transitions = store.connection.execute(
        "SELECT state FROM operator_plane_transitions WHERE command_id=?", (children[1][0],)
    ).fetchall()
    assert [row[0] for row in sibling_transitions][-1] == "cancelled"
    assert store.connection.execute(
        "SELECT COUNT(*) FROM operator_plane_outbox WHERE command_id=?", (children[1][0],)
    ).fetchone()[0] == 1


def test_operator_y_fallback_is_versioned_searchable_and_archived_after_import(tmp_path):
    first = OperatorCommandStore(tmp_path)
    first.append_y_interrupt_fallback(
        {
            "interrupt_attempt_id": "y-fallback-attempt-1",
            "interrupt_id": "y-interrupt-1",
            "action_id": "oem.y.stop",
            "controller_stop_acknowledged": True,
        },
        reason="forced-test",
    )
    first.stop()
    first.connection.close()
    second = OperatorCommandStore(tmp_path)
    history = second.connection.execute(
        "SELECT stream,interrupt_attempt_id,receipt_json FROM operator_plane_interrupt_history"
    ).fetchone()
    assert tuple(history[:2]) == ("y", "y-fallback-attempt-1")
    assert json.loads(history[2])["controller_stop_acknowledged"] is True
    assert not (tmp_path / "operator_y_interrupt_fallback.v2.jsonl").exists()
    assert len(list(tmp_path.glob("operator_y_interrupt_fallback.v2.imported.*.jsonl"))) == 1


@pytest.mark.parametrize("failure_stage", ["begin", "finalize"])
def test_delivered_y_stop_falls_back_for_every_post_delivery_persistence_exception(failure_stage):
    app = FastAPI()

    @app.post("/stop")
    async def stop():
        return {
            "ok": True,
            "stop": {"first_delivery": {"status": 100}, "second_delivery": {"status": 100}},
            "terminal_speed": {"stopped": True, "last_speed": 0, "last_ack": {"status": 100}},
        }

    class Store:
        def __init__(self):
            self._priority_fence = threading.Event()
            self.saved = None

        def begin_interrupt(self, *args, **kwargs):
            if failure_stage == "begin":
                raise OSError("begin failed")
            return {"persistence_state": "committed", "interrupt_id": "interrupt-1", "active_command_id": "command-1", "active_command_ids": ["command-1"], "cutoff": 9}

        def finalize_interrupt(self, **kwargs):
            raise sqlite3.IntegrityError("finalize failed")

        def append_y_interrupt_fallback(self, receipt, *, reason):
            self.saved = {**receipt, "reason": reason}
            return self.saved

    plane = object.__new__(OperatorCommandPlane)
    plane.app = app
    plane.store = Store()
    plane.dispatch = {"oem.y.stop": {"method": "POST", "path": "/stop", "fixed_inputs": {}, "locations": {}}}
    plane.machine_state_provider = lambda: _state()
    result = asyncio.run(plane.invoke_y_interrupt({"reason": "test"}))
    assert result["persistence_state"] == "fsync_fallback"
    assert result["controller_stop_delivered"] is True
    assert result["first_stop_ack"]["status"] == 100
    assert result["second_stop_ack"]["status"] == 100
    assert result["terminal_speed_evidence"]["last_speed"] == 0
    if failure_stage == "finalize":
        assert result["active_command_id"] == "command-1"
        assert result["cutoff"] == 9


def test_command_plane_failure_stop_records_double_ack_zero_speed_and_reconciliation():
    app = FastAPI()

    @app.post("/stop")
    async def stop():
        return {
            "ok": True,
            "stop": {"first_delivery": {"status": 100}, "second_delivery": {"status": 100}},
            "double_stop_acknowledged": True,
            "terminal_speed": {"stopped": True, "last_speed": 0, "last_ack": {"status": 100}},
            "terminal_speed_zero": True,
        }

    class Store:
        def __init__(self):
            self.reconciled = None

        def mark_y_reconciliation_required(self, *, receipt_id):
            self.reconciled = receipt_id

    plane = object.__new__(OperatorCommandPlane)
    plane.app = app
    plane.store = Store()
    plane.dispatch = {"oem.y.stop": {"method": "POST", "path": "/stop", "fixed_inputs": {}, "locations": {}}}
    plane.machine_state_provider = lambda: _state()
    evidence = asyncio.run(plane._dispatch_y_failure_stop("command-1", reason="controller_error_13"))
    assert evidence["acknowledged"] is True
    assert evidence["response"]["stop"]["first_delivery"]["status"] == 100
    assert evidence["response"]["stop"]["second_delivery"]["status"] == 100
    assert evidence["response"]["terminal_speed"]["last_speed"] == 0
    assert plane.store.reconciled == "failure-stop:command-1"


def test_internal_m04_sources_dispatch_through_private_typed_command_plane_paths():
    app = FastAPI()
    observed = []

    @app.post("/overload")
    async def overload(payload: dict):
        observed.append(("acceleration_overload", payload))
        return {"ok": True, "intent": "acceleration_overload", **payload}

    @app.post("/board-test")
    async def board_test(payload: dict):
        observed.append(("board_test_my", payload))
        return {"ok": True, "intent": "board_test_my", **payload}

    plane = object.__new__(OperatorCommandPlane)
    plane.app = app
    plane.dispatch = {
        "oem.y.internal.acceleration_overload": {"method": "POST", "path": "/overload", "fixed_inputs": {}, "locations": {"target_steps": {"location": "body", "wire_name": "target_steps"}, "acceleration_override": {"location": "body", "wire_name": "acceleration_override"}}},
        "oem.y.internal.board_test_my": {"method": "POST", "path": "/board-test", "fixed_inputs": {}, "locations": {"target_steps": {"location": "body", "wire_name": "target_steps"}}},
    }
    plane.machine_state_provider = lambda: _state()
    overload_result = asyncio.run(plane.invoke_internal_y_absolute("acceleration_overload", target_steps=2000, acceleration_override=250))
    board_result = asyncio.run(plane.invoke_internal_y_absolute("board_test_my", target_steps=3000))
    assert overload_result["acceleration_override"] == 250
    assert board_result["intent"] == "board_test_my"
    assert observed == [
        ("acceleration_overload", {"target_steps": 2000, "acceleration_override": 250}),
        ("board_test_my", {"target_steps": 3000}),
    ]
    with pytest.raises(HTTPException, match="source_fixed"):
        asyncio.run(plane.invoke_internal_y_absolute("board_test_my", target_steps=3000, acceleration_override=250))


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
    assert store.claim_next() is None
    store.finish(claimed["command_id"], status="completed", payload={"completion_class": "event_128"})
    row = store.connection.execute("SELECT state FROM serial206_movement_commands WHERE command_id=?", (claimed["command_id"],)).fetchone()
    assert row[0] == "completed"
    other_claim = store.claim_next()
    assert other_claim is not None
    assert other_claim["command_id"] == other["command_id"]


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
