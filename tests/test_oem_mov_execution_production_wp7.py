from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
from typing import Callable

from fastapi import FastAPI
import pytest

from bioxp.oem_deck_movement import ClassMoveToIntent, compile_mov_execution
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.operator_command_plane import OperatorCommandPlane, OperatorCommandStore


def _movable() -> dict[str, str]:
    return {
        "POOL_PLATE": "LOC_P_TC",
        "OUTPUT_PLATE": "LOC_P_OC",
        "REAGENT_PLATE": "LOC_RC",
        "STRIP_ONE": "LOC_STRIP1",
        "STRIP_TWO": "LOC_STRIP2",
        "STRIP_THREE": "LOC_STRIP3",
        "STRIP_FOUR": "LOC_STRIP4",
        "REAGENT_COVER": "LOC_RC_COVER_STORAGE",
        "OUTPUT_COVER": "LOC_OC_COVER_STORAGE",
        "BIO_SECURITY_COVER": "LOC_BSC",
        "TROUGH": "LOC_TROUGH",
    }


def _bootstrap(store: OperatorCommandStore) -> None:
    store.bootstrap_deck_semantic_state({
        "current_location": "LOC_MS", "current_well": 0, "current_tray": None,
        "tip_loaded": False, "tip_dirty": False, "tip_location": -1,
        "clean_path": True, "plate_on_gantry": None,
        "movable_plate_locations": _movable(), "pseudo_z_home": 65000,
        "ownership_generation": 7, "board_epoch_4": 10, "board_epoch_5": 11,
        "latch_status": True, "machine_latch_closed": True,
        "latch_observation_id": "latch-production-wp7",
        "source_operation": "test_source_snapshot",
        "source_command_id": "source-production-wp7",
    })


class Provider:
    def __init__(self) -> None:
        self.calls: list[str] = []
        self.before_provider_call: Callable[[str], None] = lambda operation: None

    @contextmanager
    def movement_lease(self):
        self.calls.append("lease")
        yield

    def mov_execution_machine_state(self):
        return {
            "save_tip": False, "old_well": False, "old_well_text": "A1",
            "old_location": 18, "plate_locations": {0: 23},
            "plate_pierced": {}, "well_pierced": {}, "strip_pierced": {},
            "tip_location": 5, "trough_version": 1,
            "authority_digest": "a" * 64, "board_epoch_4": 10,
            "board_epoch_5": 11, "position_table_by_location": {},
        }

    def get_next_well(self, plate_name: int, material: str, offset: float) -> int:
        raise AssertionError("material branch was not requested")

    def preview_scriptmove_to(self, arguments):
        return {"plan": {
            "ok": True, "schema_version": "bioxp.oem_scriptmove_path_plan.v1",
            "source_method_token": "0x06000120", "source_il_sha256": "3" * 64,
            "steps": [{"op": "moveTo", "x": 1, "y": 2, "z": 3}],
            "source_hazards": [],
        }}

    def scriptmoveTo(self, **_arguments):
        self.before_provider_call("scriptmoveTo")
        self.calls.append("scriptmoveTo")
        return {
            "ok": True, "delivery_attempted": True,
            "controller_command_acknowledged": True,
            "controller_completion_verified": True,
        }

    def updateLocation(self, **_arguments):
        self.before_provider_call("updateLocation")
        self.calls.append("updateLocation")
        return {"ok": True, "delivery_attempted": False, "semantic_update_ready": True}

    def updatePlateLocation(self, **_arguments):
        self.before_provider_call("updatePlateLocation")
        self.calls.append("updatePlateLocation")
        return {"ok": True, "delivery_attempted": False, "semantic_update_ready": True}

    def rPunchFoil(self, **_arguments):
        self.before_provider_call("rPunchFoil")
        self.calls.append("rPunchFoil")
        return {
            "ok": True, "delivery_attempted": True,
            "controller_command_acknowledged": True,
            "controller_completion_verified": True,
        }


class CompileFailureProvider(Provider):
    def mov_execution_machine_state(self):
        raise RuntimeError("injected compile prerequisite failure")


def test_private_mov_execution_reaches_canonical_worker_and_durable_children(tmp_path: Path) -> None:
    OEMRuntimeStore(tmp_path).close()
    store = OperatorCommandStore(tmp_path)
    _bootstrap(store)
    state = {
        "ownership_generation": 7,
        "serial206_initialization_provider": {
            "x_authority": {"active_board_epoch": 11},
            "board4_authority": {"active_board_epoch": 10},
        },
    }
    admitted = store.admit_internal_mov_execution(
        ClassMoveToIntent(script_line=17, plate_name=0, continuation="r"), state=state,
        idempotency_key="production-wp7",
    )
    assert store._acquire_owner() is True
    claimed = store.claim_next()
    assert claimed is not None and claimed["command_id"] == admitted["command_id"]

    app = FastAPI()
    provider = Provider()
    observed_markers: list[str] = []

    def require_pre_tx_marker(operation: str) -> None:
        row = store.connection.execute(
            "SELECT work_identity FROM operator_plane_delivery_attempts "
            "WHERE command_id=? ORDER BY attempt_sequence DESC LIMIT 1",
            (admitted["command_id"],),
        ).fetchone()
        assert row is not None
        assert str(row["work_identity"]).endswith(f":{operation}")
        observed_markers.append(operation)

    provider.before_provider_call = require_pre_tx_marker
    app.state.oem_deck_provider = provider
    plane = object.__new__(OperatorCommandPlane)
    plane.app = app
    plane.machine_state_provider = lambda: state
    plane.actions = []
    plane.dispatch = {}
    plane.by_id = {}
    plane.store = store
    plane._dispatch_one(claimed)

    receipt = store.get_command(admitted["command_id"])
    assert receipt is not None and receipt["status"] == "completed", receipt
    assert provider.calls == ["lease", "scriptmoveTo", "updateLocation", "updatePlateLocation", "rPunchFoil"]
    assert observed_markers == ["scriptmoveTo", "updateLocation", "updatePlateLocation", "rPunchFoil"]
    plan = store.connection.execute(
        "SELECT source_branch,plan_digest FROM operator_plane_deck_commands WHERE command_id=?",
        (admitted["command_id"],),
    ).fetchone()
    assert tuple(plan) and plan["source_branch"] == "normal"
    stages = store.connection.execute(
        "SELECT operation,terminal_state FROM operator_plane_deck_stages WHERE command_id=? ORDER BY stage_order",
        (admitted["command_id"],),
    ).fetchall()
    assert [(row["operation"], row["terminal_state"]) for row in stages] == [
        ("scriptmoveTo", "completed"),
        ("updateLocation", "completed"),
        ("updatePlateLocation", "completed"),
        ("rPunchFoil", "completed"),
    ]
    semantic = store.deck_semantic_state()
    assert semantic["current_location"] == "LOC_TC"
    assert semantic["movable_plate_locations"]["POOL_PLATE"] == "LOC_TC"
    assert semantic["well_pierced"] == {"0:0:1": True}
    store.connection.close()


def test_mov_execution_compile_failure_is_failed_without_recovery_hold(tmp_path: Path) -> None:
    OEMRuntimeStore(tmp_path).close()
    store = OperatorCommandStore(tmp_path)
    _bootstrap(store)
    state = {
        "ownership_generation": 7,
        "serial206_initialization_provider": {
            "x_authority": {"active_board_epoch": 11},
            "board4_authority": {"active_board_epoch": 10},
        },
    }
    admitted = store.admit_internal_mov_execution(
        ClassMoveToIntent(script_line=18, plate_name=0), state=state,
        idempotency_key="production-wp7-compile-failure",
    )
    claimed = store.claim_next()
    assert claimed is not None
    app = FastAPI()
    app.state.oem_deck_provider = CompileFailureProvider()
    plane = object.__new__(OperatorCommandPlane)
    plane.app = app
    plane.machine_state_provider = lambda: state
    plane.actions = []
    plane.dispatch = {}
    plane.by_id = {}
    plane.store = store
    plane._dispatch_one(claimed)
    receipt = store.get_command(admitted["command_id"])
    assert receipt is not None and receipt["status"] == "failed"
    assert store.deck_recovery_blocker() is None
    assert store.connection.execute(
        "SELECT COUNT(*) FROM operator_plane_deck_stages WHERE command_id=?",
        (admitted["command_id"],),
    ).fetchone()[0] == 0
    store.connection.close()


def test_mov_execution_stage_completion_cannot_cross_a_committed_stop(tmp_path: Path) -> None:
    OEMRuntimeStore(tmp_path).close()
    store = OperatorCommandStore(tmp_path)
    _bootstrap(store)
    state = {
        "ownership_generation": 7,
        "serial206_initialization_provider": {
            "x_authority": {"active_board_epoch": 11},
            "board4_authority": {"active_board_epoch": 10},
        },
    }
    admitted = store.admit_internal_mov_execution(
        ClassMoveToIntent(script_line=19, plate_name=0),
        state=state,
        idempotency_key="production-wp7-stale-completion",
    )
    claimed = store.claim_next()
    assert claimed is not None
    provider = Provider()
    plan = compile_mov_execution(
        ClassMoveToIntent(script_line=19, plate_name=0),
        provider.mov_execution_machine_state(),
    )
    store.persist_mov_execution_plan(admitted["command_id"], plan)
    dispatch_attempt_id = str(claimed["dispatch_attempt_id"])
    with store._transaction() as conn, store._authority_write():
        conn.execute(
            "UPDATE operator_plane_commands SET status='stopped' WHERE command_id=?",
            (admitted["command_id"],),
        )
        conn.execute(
            "UPDATE serial206_movement_commands SET state='interrupted' WHERE command_id=?",
            (admitted["command_id"],),
        )
    with pytest.raises(RuntimeError, match="completion authority is stale"):
        store.terminalize_mov_execution_stage(
            admitted["command_id"],
            plan.steps[0],
            state="completed",
            result={
                "ok": True,
                "delivery_attempted": True,
                "controller_command_acknowledged": True,
                "controller_completion_verified": True,
            },
            dispatch_attempt_id=dispatch_attempt_id,
        )
    assert store.connection.execute(
        "SELECT terminal_state FROM operator_plane_deck_stages "
        "WHERE command_id=? AND stage_order=0",
        (admitted["command_id"],),
    ).fetchone()[0] == "planned"
    store.connection.close()


def test_wp7_committed_delivery_marker_survives_stale_post_provider_completion(tmp_path: Path) -> None:
    OEMRuntimeStore(tmp_path).close()
    store = OperatorCommandStore(tmp_path)
    _bootstrap(store)
    state = {
        "ownership_generation": 7,
        "serial206_initialization_provider": {
            "x_authority": {"active_board_epoch": 11},
            "board4_authority": {"active_board_epoch": 10},
        },
    }
    admitted = store.admit_internal_mov_execution(
        ClassMoveToIntent(script_line=20, plate_name=0, continuation="r"), state=state,
        idempotency_key="production-wp7-marker-stale-completion",
    )
    claimed = store.claim_next()
    assert claimed is not None
    provider = Provider()
    original_assert = store.assert_deck_execution_current

    def stale_after_provider(command_id: str, *, boundary: str) -> None:
        if boundary.startswith("after_provider"):
            raise RuntimeError("deck_execution_dispatch_authority_stale")
        original_assert(command_id, boundary=boundary)

    store.assert_deck_execution_current = stale_after_provider
    app = FastAPI()
    app.state.oem_deck_provider = provider
    plane = object.__new__(OperatorCommandPlane)
    plane.app = app
    plane.machine_state_provider = lambda: state
    plane.actions = []
    plane.dispatch = {}
    plane.by_id = {}
    plane.store = store

    plane._dispatch_one(claimed)

    receipt = store.get_command(admitted["command_id"])
    assert store.connection.execute(
        "SELECT COUNT(*) FROM operator_plane_delivery_attempts WHERE command_id=?",
        (admitted["command_id"],),
    ).fetchone()[0] == 1
    assert receipt is not None and receipt["status"] == "ambiguous"
    assert receipt["terminal_evidence"]["delivery_attempted"] is True
    assert store.deck_recovery_blocker() == "deck_recovery_hold"
    store.connection.close()
