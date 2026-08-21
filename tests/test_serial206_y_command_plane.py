import pytest
from fastapi import HTTPException

from src.bioxp.operator_command_plane import OperatorCommandStore, _validate_inputs


def test_command_store_accepts_typed_y_actions_and_bounds(tmp_path):
    assert _validate_inputs("oem.y.move_steps", {"steps": 20}) == {"steps": 20}
    assert _validate_inputs("oem.y.move_absolute", {"target_steps": 102956}) == {"target_steps": 102956}
    with pytest.raises(HTTPException):
        _validate_inputs("oem.y.move_absolute", {"target_steps": 102957})
    with pytest.raises(HTTPException):
        _validate_inputs("oem.y.move_steps", {"steps": 1, "speed": 1800})


def test_y_safety_epoch_is_persisted_and_y_stop_does_not_clear_z(tmp_path):
    store = OperatorCommandStore(tmp_path)
    before = store.connection.execute("SELECT x_epoch,y_epoch,z_epoch FROM operator_plane_safety WHERE singleton=1").fetchone()
    assert tuple(before) == (0, 0, 0)
    receipt = store.begin_interrupt(
        "oem.y.stop",
        state={"ownership_generation": 4, "serial206_initialization_provider": {"z_authority": {"state": "referenced_ready", "reference_state": "referenced"}}},
        request={"idempotency_key": "y-stop-key-001", "expected_generation": 4},
    )
    assert receipt["scope"] == "y"
    after = store.connection.execute("SELECT x_epoch,y_epoch,z_epoch FROM operator_plane_safety WHERE singleton=1").fetchone()
    assert tuple(after) == (0, 1, 0)
    store.stop()


def test_y_command_is_queued_with_canonical_inputs(tmp_path):
    store = OperatorCommandStore(tmp_path)
    response = store.admit_command(
        {
            "action_id": "oem.y.move_steps",
            "inputs": {"steps": 100},
            "expected_ownership_generation": 4,
            "idempotency_key": "y-command-key-001",
        },
        state={"ownership_generation": 4, "serial206_initialization_provider": {"z_authority": {"state": "referenced_ready", "reference_state": "referenced"}}},
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
            "action_id": "oem.y.move_steps",
            "inputs": {"steps": 100},
            "expected_ownership_generation": 4,
            "idempotency_key": "y-command-key-002",
        },
        state={"ownership_generation": 4, "serial206_initialization_provider": {"z_authority": {"state": "referenced_ready", "reference_state": "referenced"}}},
    )
    claimed = store.claim_next()
    assert claimed["command_id"] == admitted["command_id"]
    row = store.connection.execute("SELECT state FROM serial206_movement_commands WHERE command_id=?", (claimed["command_id"],)).fetchone()
    assert row[0] == "dispatched"
    other = store.admit_command(
        {
            "action_id": "oem.x.move_steps",
            "inputs": {"steps": 100},
            "expected_ownership_generation": 4,
            "idempotency_key": "x-command-key-002",
        },
        state={"ownership_generation": 4, "serial206_initialization_provider": {"z_authority": {"state": "referenced_ready", "reference_state": "referenced"}}},
    )
    other_claim = store.claim_next()
    assert other_claim is not None
    assert other_claim["command_id"] == other["command_id"]
    store.finish(claimed["command_id"], status="completed", payload={"completion_class": "event_128"})
    row = store.connection.execute("SELECT state FROM serial206_movement_commands WHERE command_id=?", (claimed["command_id"],)).fetchone()
    assert row[0] == "completed"


def test_nonblocking_y_absolute_keeps_command_dispatched(tmp_path):
    store = OperatorCommandStore(tmp_path)
    admitted = store.admit_command(
        {
            "action_id": "oem.y.move_absolute",
            "inputs": {"target_steps": 100},
            "expected_ownership_generation": 4,
            "idempotency_key": "y-command-key-003",
        },
        state={"ownership_generation": 4},
    )
    claimed = store.claim_next()
    assert claimed is not None
    store.mark_dispatched(claimed["command_id"], payload={"pending": True})
    row = store.connection.execute("SELECT status FROM operator_plane_commands WHERE command_id=?", (admitted["command_id"],)).fetchone()
    assert row[0] == "dispatched"
    store.stop()
