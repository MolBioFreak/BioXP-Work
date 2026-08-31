from __future__ import annotations

import hashlib
import json
import sqlite3
import threading
from contextlib import contextmanager

from bioxp.operator_command_plane import OperatorCommandStore
from bioxp.oem_deck_movement import (
    DeckAuthoritySnapshot,
    DeckMovementExecutor,
    DeckMovementPlan,
    DeckPlanStep,
)


def _recovery_store() -> OperatorCommandStore:
    store = object.__new__(OperatorCommandStore)
    store._lock = threading.RLock()
    store._authority_write_depth = 0
    store.connection = sqlite3.connect(":memory:", isolation_level=None)
    store.connection.row_factory = sqlite3.Row
    store.connection.create_function(
        "authority_write_allowed", 0, lambda: 1 if store._authority_write_depth else 0
    )
    store.connection.executescript(
        """
        CREATE TABLE operator_plane_commands(command_id TEXT PRIMARY KEY);
        CREATE TABLE operator_plane_idempotency(
          operation_kind TEXT NOT NULL, idempotency_key TEXT NOT NULL,
          fingerprint TEXT NOT NULL, command_id TEXT, method_id TEXT,
          response_json TEXT NOT NULL, created_at REAL NOT NULL,
          PRIMARY KEY(operation_kind,idempotency_key)
        );
        CREATE TABLE operator_plane_safety(
          singleton INTEGER PRIMARY KEY, recovery_epoch INTEGER NOT NULL,
          recovery_version INTEGER NOT NULL, recovery_hold INTEGER NOT NULL,
          updated_at REAL NOT NULL
        );
        INSERT INTO operator_plane_safety VALUES(1,4,7,0,1.0);
        CREATE TABLE operator_plane_deck_commands(
          command_id TEXT PRIMARY KEY, delivery_attempted INTEGER NOT NULL DEFAULT 0,
          controller_command_acknowledged INTEGER NOT NULL DEFAULT 0,
          controller_completion_verified INTEGER NOT NULL DEFAULT 0,
          hardware_postcondition_verified INTEGER NOT NULL DEFAULT 0,
          semantic_state_committed INTEGER NOT NULL DEFAULT 0,
          ambiguity_state TEXT NOT NULL DEFAULT 'none', provider_evidence_json TEXT
        );
        CREATE TABLE operator_plane_deck_semantic_state(
          singleton INTEGER PRIMARY KEY, current_location TEXT, current_well INTEGER,
          semantic_state_revision INTEGER NOT NULL, producer_operation TEXT,
          producer_command_id TEXT, transition_provenance_json TEXT NOT NULL,
          ambiguity_state TEXT NOT NULL, updated_at REAL NOT NULL
        );
        INSERT INTO operator_plane_commands VALUES('deck-1');
        INSERT INTO operator_plane_deck_commands(command_id) VALUES('deck-1');
        INSERT INTO operator_plane_deck_semantic_state VALUES(
          1,'LOC_MS',0,11,'verified_success','prior-command','{}','none',1.0
        );
        """
    )
    return store


def test_post_io_ambiguity_preserves_exact_last_confirmed_semantic_revision_and_values():
    store = _recovery_store()
    before = tuple(store.connection.execute(
        "SELECT current_location,current_well,semantic_state_revision,producer_operation,"
        "producer_command_id,transition_provenance_json,ambiguity_state "
        "FROM operator_plane_deck_semantic_state WHERE singleton=1"
    ).fetchone())

    store.mark_deck_recovery_required(
        "deck-1", reason="controller_timeout", controller_command_acknowledged=True
    )

    after = tuple(store.connection.execute(
        "SELECT current_location,current_well,semantic_state_revision,producer_operation,"
        "producer_command_id,transition_provenance_json,ambiguity_state "
        "FROM operator_plane_deck_semantic_state WHERE singleton=1"
    ).fetchone())
    assert after == before
    assert store.deck_recovery_blocker() == "deck_recovery_hold"


def test_recovery_blocker_is_derived_from_hold_and_unresolved_command_not_semantic_projection():
    store = _recovery_store()
    store.connection.execute(
        "UPDATE operator_plane_safety SET recovery_hold=1 WHERE singleton=1"
    )
    store.connection.execute(
        "UPDATE operator_plane_deck_commands SET ambiguity_state='recovery_required' WHERE command_id='deck-1'"
    )
    assert store.deck_recovery_blocker() == "deck_recovery_hold"


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
