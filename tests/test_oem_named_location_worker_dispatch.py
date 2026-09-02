from __future__ import annotations

from contextlib import contextmanager
from types import SimpleNamespace

from fastapi import HTTPException
import pytest

from bioxp.operator_command_plane import ACTION_REQUEST_SCHEMA, OperatorCommandPlane, OperatorCommandStore
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.oem_deck_catalog import configured_location_names
from bioxp.oem_deck_movement import (
    DeckExecutionFailure,
    OEM_MOVABLE_OBJECT_DEFAULT_LOCATIONS,
    make_deck_command_executor,
)
from bioxp.oem_compat.position_table import PositionTable


class FakeStore:
    def action_fenced(self, action_id): return False
    def finish(self, command_id, **kwargs): self.finished = (command_id, kwargs)
    def mark_deck_recovery_required(self, *args, **kwargs): self.recovery = (args, kwargs)


class State:
    ownership_generation = 1
    oem_deck_command_executor = None
    oem_deck_command_assessment = staticmethod(lambda _state: {"enabled": True})


class App:
    state = State()


def _store(root):
    OEMRuntimeStore(root).close()
    store = OperatorCommandStore(root)
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
        "pseudo_z_home": 500,
        "ownership_generation": 7,
        "board_epoch_4": 10,
        "board_epoch_5": 11,
        "latch_status": True,
        "machine_latch_closed": True,
        "latch_observation_id": "test-latch-bootstrap",
        "source_operation": "test_bootstrap",
        "source_command_id": "test-bootstrap",
    })
    return store


def test_canonical_worker_dispatches_deck_only_through_bound_executor() -> None:
    calls = []
    App.state.oem_deck_command_executor = lambda **kwargs: calls.append(kwargs) or {"ok": True, "controller_completion_verified": True, "semantic_state_committed": True}
    plane = OperatorCommandPlane.__new__(OperatorCommandPlane)
    plane.app = App(); plane.store = FakeStore(); plane.machine_state_provider = lambda: {"ownership_generation": 1}
    plane._dispatch_one({"command_id": "cmd-1", "action_id": "oem.deck.move_to_location", "requested_inputs": {"target": "LOC_OC", "camera_offset": False}, "effective_inputs": {"target": "LOC_OC", "camera_offset": False}, "ownership_generation": 1, "expected_board_epoch_by_board": {"4": 10, "5": 11}, "dispatch_attempt_id": "attempt-1"})
    assert calls == [{"command_id": "cmd-1", "target": "LOC_OC", "camera_offset": False, "expected_ownership_generation": 1, "expected_board_epoch_by_board": {"4": 10, "5": 11}}]
    assert plane.store.finished[1]["status"] == "completed"


def test_pre_io_deck_executor_exception_is_failed_not_ambiguous() -> None:
    def fail_before_io(**kwargs):
        raise DeckExecutionFailure("authority unavailable", delivery_attempted=False)

    App.state.oem_deck_command_executor = fail_before_io
    plane = OperatorCommandPlane.__new__(OperatorCommandPlane)
    plane.app = App(); plane.store = FakeStore(); plane.machine_state_provider = lambda: {"ownership_generation": 1}
    plane._dispatch_one({"command_id": "cmd-2", "action_id": "oem.deck.move_to_location", "requested_inputs": {"target": "LOC_OC", "camera_offset": False}, "effective_inputs": {"target": "LOC_OC", "camera_offset": False}, "ownership_generation": 1, "expected_board_epoch_by_board": {"4": 10, "5": 11}, "dispatch_attempt_id": "attempt-2"})
    assert plane.store.finished[1]["status"] == "failed"
    assert plane.store.finished[1]["payload"]["delivery_attempted"] is False
    assert "outcome_unknown" not in plane.store.finished[1]["payload"]


def test_post_io_deck_executor_exception_remains_ambiguous_and_is_not_redispatched() -> None:
    calls = []

    def fail_after_io(**kwargs):
        calls.append(kwargs)
        raise DeckExecutionFailure("controller completion unavailable", delivery_attempted=True)

    App.state.oem_deck_command_executor = fail_after_io
    plane = OperatorCommandPlane.__new__(OperatorCommandPlane)
    plane.app = App(); plane.store = FakeStore(); plane.machine_state_provider = lambda: {"ownership_generation": 1}
    plane._dispatch_one({"command_id": "cmd-3", "action_id": "oem.deck.move_to_location", "requested_inputs": {"target": "LOC_OC", "camera_offset": False}, "effective_inputs": {"target": "LOC_OC", "camera_offset": False}, "ownership_generation": 1, "expected_board_epoch_by_board": {"4": 10, "5": 11}, "dispatch_attempt_id": "attempt-3"})
    assert len(calls) == 1
    assert plane.store.finished[1]["status"] == "ambiguous"
    assert plane.store.finished[1]["payload"]["delivery_attempted"] is True
    assert plane.store.finished[1]["payload"]["outcome_unknown"] is True


def test_generic_pre_io_exception_is_failed_and_response_delivery_truth_controls_ambiguity() -> None:
    def generic_pre_io(**kwargs):
        raise RuntimeError("snapshot rejected")

    App.state.oem_deck_command_executor = generic_pre_io
    plane = OperatorCommandPlane.__new__(OperatorCommandPlane)
    plane.app = App(); plane.store = FakeStore(); plane.machine_state_provider = lambda: {"ownership_generation": 1}
    claimed = {"command_id": "cmd-4", "action_id": "oem.deck.move_to_location", "requested_inputs": {"target": "LOC_OC", "camera_offset": False}, "effective_inputs": {"target": "LOC_OC", "camera_offset": False}, "ownership_generation": 1, "expected_board_epoch_by_board": {"4": 10, "5": 11}, "dispatch_attempt_id": "attempt-4"}
    plane._dispatch_one(claimed)
    assert plane.store.finished[1]["status"] == "failed"

    App.state.oem_deck_command_executor = lambda **kwargs: {
        "ok": False, "delivery_attempted": True,
        "controller_command_acknowledged": True, "controller_completion_verified": False,
        "error": "provider_stage_failed:moveTo",
    }
    plane.store = FakeStore()
    plane._dispatch_one({**claimed, "command_id": "cmd-5"})
    assert plane.store.finished[1]["status"] == "ambiguous"
    assert plane.store.finished[1]["controller_acknowledged"] is True
    assert plane.store.finished[1]["payload"]["outcome_unknown"] is True
    assert plane.store.recovery[0] == ("cmd-5",)


class _RaisingPrimitiveProvider:
    def __init__(self, table):
        self.table = table
        self.invocations = 0

    @contextmanager
    def movement_lease(self):
        yield

    def force_to_high_home(self, **_kwargs):
        return {"ok": True}

    def deck_authority_snapshot(self, *, expected_generation):
        return {
            "ownership_generation": expected_generation,
            "provider_owner_id": "raising-provider",
            "board_epoch_4": 10,
            "board_epoch_5": 11,
            "position_table_sha256": self.table.digest,
            "machine_state_revision": 1,
            "reference_versions": {"x": 1, "y": 1, "z": 1, "g": 1},
            "safety_epochs": {"global": 0, "x": 0, "y": 0, "z": 0},
            "latch_observation_id": "latch-1",
            "controller_position_observation_id": "position-1",
            "captured_at": 1.0,
            "current_x": 0,
            "current_y": 0,
            "current_z": 65000,
            "current_location_id": "LOC_MS",
            "current_well_id": 0,
            "tip_loaded": False,
            "tip_dirty": False,
            "tip_location": -1,
            "clean_path": True,
            "plate_on_gantry": None,
            "pseudo_z_home": 500,
            "device_type": "BIOXP",
            "latch_status": True,
            "machine_latch_closed": True,
        }

    def moveTo(self, **_kwargs):
        self.invocations += 1
        raise RuntimeError("transport outcome unavailable")


class _CompletedPrimitiveProvider(_RaisingPrimitiveProvider):
    def moveTo(self, **_kwargs):
        self.invocations += 1
        return {
            "ok": True,
            "provider_command_id": "provider-completed-1",
            "controller_command_acknowledged": True,
            "controller_completion_verified": True,
            "hardware_postcondition_verified": True,
        }


class _MissingTerminalPrimitiveProvider(_CompletedPrimitiveProvider):
    def __init__(self, table, *, contradictory: bool):
        super().__init__(table)
        self.contradictory = contradictory

    def moveTo(self, **_kwargs):
        self.invocations += 1
        result = {
            "ok": True,
            "provider_command_id": "provider-missing-terminal-1",
            "controller_command_acknowledged": True,
            "hardware_postcondition_verified": True,
        }
        if self.contradictory:
            result["controller_completion_verified"] = True
            result["primitive_result"] = {"controller_terminal_state_verified": False}
        return result


class _InterruptFencePrimitiveProvider(_CompletedPrimitiveProvider):
    def __init__(self, table):
        super().__init__(table)
        self.calls: list[str] = []

    def force_to_high_home(self, **_kwargs):
        self.calls.append("force_to_high_home")
        return {"ok": True, "psudo_z_home_steps": 500}

    def moveTo(self, **_kwargs):
        self.calls.append("moveTo")
        return {
            "ok": True,
            "provider_command_id": "provider-fenced-move-1",
            "controller_command_acknowledged": True,
            "controller_completion_verified": True,
            "hardware_postcondition_verified": True,
        }

    def moveZCamera(self, **_kwargs):
        self.calls.append("moveZCamera")
        return {
            "ok": True,
            "provider_command_id": "provider-fenced-camera-1",
            "controller_command_acknowledged": True,
            "controller_completion_verified": True,
            "hardware_postcondition_verified": True,
        }


def test_named_location_rechecks_interrupt_fence_at_every_provider_boundary(
    tmp_path, monkeypatch
) -> None:
    store = _store(tmp_path)
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
        "pseudo_z_home": 500,
        "ownership_generation": 7,
        "board_epoch_4": 10,
        "board_epoch_5": 11,
        "latch_status": True,
        "machine_latch_closed": True,
        "latch_observation_id": "interrupt-fence-latch",
        "source_operation": "test_bootstrap",
        "source_command_id": "test-bootstrap",
    })
    state = {
        "ownership_generation": 7,
        "serial206_initialization_provider": {
            "x_authority": {"active_board_epoch": 11},
            "board4_authority": {"active_board_epoch": 10},
        },
    }
    request = {
        "schema_version": ACTION_REQUEST_SCHEMA,
        "idempotency_key": "named-location-interrupt-fence",
        "expected_ownership_generation": 7,
        "expected_board_epoch_by_board": {"4": 10, "5": 11},
        "action_id": "oem.deck.move_to_location",
        "inputs": {"target": "LOC_TC_BARCODE", "camera_offset": False},
    }
    admitted = store.admit_command(request, state=state)
    assert store._acquire_owner() is True
    assert store.claim_next() is not None
    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    provider = _InterruptFencePrimitiveProvider(table)
    boundaries: list[str] = []

    def assert_current(command_id: str, *, boundary: str) -> None:
        assert command_id == admitted["command_id"]
        boundaries.append(boundary)
        if boundary == "before_provider_stage_4_moveZCamera":
            raise RuntimeError("deck_execution_interrupt_fence_active")

    monkeypatch.setattr(store, "assert_deck_execution_current", assert_current)
    executor = make_deck_command_executor(
        provider_getter=lambda: provider,
        position_table_provider=lambda: table,
        command_store=store,
    )

    with pytest.raises(RuntimeError, match="deck_execution_interrupt_fence_active"):
        executor(
            command_id=admitted["command_id"],
            target="LOC_TC_BARCODE",
            camera_offset=False,
            expected_ownership_generation=7,
            expected_board_epoch_by_board={"4": 10, "5": 11},
        )

    assert provider.calls == ["force_to_high_home", "moveTo"]
    assert boundaries == [
        "before_force_to_high_home",
        "after_force_to_high_home",
        "before_terminalize_stage_0_ForceToHighHome",
        "before_terminalize_stage_1_check_latch_status",
        "before_terminalize_stage_2_check_machine_latch_closed",
        "before_provider_stage_3_moveTo",
        "after_provider_stage_3_moveTo",
        "before_terminalize_stage_3_moveTo",
        "before_provider_stage_4_moveZCamera",
    ]
    store.stop()


def test_named_location_persists_plan_and_force_delivery_before_first_provider_write(
    tmp_path, monkeypatch,
) -> None:
    store = _store(tmp_path)
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
        "pseudo_z_home": 500,
        "ownership_generation": 7,
        "board_epoch_4": 10,
        "board_epoch_5": 11,
        "latch_status": True,
        "machine_latch_closed": True,
        "latch_observation_id": "plan-before-force-latch",
        "source_operation": "test_bootstrap",
        "source_command_id": "test-bootstrap",
    })
    state = {
        "ownership_generation": 7,
        "serial206_initialization_provider": {
            "x_authority": {"active_board_epoch": 11},
            "board4_authority": {"active_board_epoch": 10},
        },
    }
    request = {
        "schema_version": ACTION_REQUEST_SCHEMA,
        "idempotency_key": "named-location-plan-before-force",
        "expected_ownership_generation": 7,
        "expected_board_epoch_by_board": {"4": 10, "5": 11},
        "action_id": "oem.deck.move_to_location",
        "inputs": {"target": "LOC_OC", "camera_offset": False},
    }
    admitted = store.admit_command(request, state=state)
    assert store._acquire_owner() is True
    assert store.claim_next() is not None
    original_assert_current = store.assert_deck_execution_current
    boundaries: list[str] = []

    def record_assert_current(command_id: str, *, boundary: str) -> None:
        boundaries.append(boundary)
        original_assert_current(command_id, boundary=boundary)

    monkeypatch.setattr(store, "assert_deck_execution_current", record_assert_current)
    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])

    class PlanAwareProvider(_CompletedPrimitiveProvider):
        def force_to_high_home(self, **_kwargs):
            plan_count = store.connection.execute(
                "SELECT COUNT(*) FROM operator_plane_deck_commands WHERE command_id=?",
                (admitted["command_id"],),
            ).fetchone()[0]
            marker_count = store.connection.execute(
                "SELECT COUNT(*) FROM operator_plane_delivery_attempts WHERE command_id=? AND work_identity='stage:0:ForceToHighHome'",
                (admitted["command_id"],),
            ).fetchone()[0]
            assert plan_count == 1
            assert marker_count == 1
            return {
                "ok": True,
                "controller_command_acknowledged": True,
                "controller_completion_verified": True,
                "hardware_postcondition_verified": True,
            }

    provider = PlanAwareProvider(table)
    executor = make_deck_command_executor(
        provider_getter=lambda: provider,
        position_table_provider=lambda: table,
        command_store=store,
    )

    result = executor(
        command_id=admitted["command_id"],
        target="LOC_OC",
        camera_offset=False,
        expected_ownership_generation=7,
        expected_board_epoch_by_board={"4": 10, "5": 11},
    )

    assert result["ok"] is True
    assert "before_semantic_commit" in boundaries
    store.stop()


@pytest.mark.parametrize("contradictory", [False, True])
def test_provider_ok_without_compatible_controller_terminal_proof_is_ambiguous_and_not_committed(
    tmp_path, contradictory
) -> None:
    store = _store(tmp_path)
    state = {
        "ownership_generation": 7,
        "serial206_initialization_provider": {
            "x_authority": {"active_board_epoch": 11},
            "board4_authority": {"active_board_epoch": 10},
        },
    }
    request = {
        "schema_version": ACTION_REQUEST_SCHEMA,
        "idempotency_key": f"terminal-proof-{int(contradictory)}",
        "expected_ownership_generation": 7,
        "expected_board_epoch_by_board": {"4": 10, "5": 11},
        "action_id": "oem.deck.move_to_location",
        "inputs": {"target": "LOC_OC", "camera_offset": False},
    }
    admitted = store.admit_command(request, state=state)
    before = store.deck_semantic_state()
    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    provider = _MissingTerminalPrimitiveProvider(table, contradictory=contradictory)
    executor = make_deck_command_executor(
        provider_getter=lambda: provider,
        position_table_provider=lambda: table,
        command_store=store,
    )
    assert store._acquire_owner() is True
    claimed = store.claim_next()
    app = SimpleNamespace(state=SimpleNamespace(
        ownership_generation=7,
        oem_deck_command_executor=executor,
        oem_deck_command_assessment=lambda _state: {"enabled": True},
    ))
    plane = OperatorCommandPlane.__new__(OperatorCommandPlane)
    plane.app = app
    plane.store = store
    plane.machine_state_provider = lambda: state

    plane._dispatch_one(claimed)

    receipt = store.get_command(admitted["command_id"])
    assert provider.invocations == 1
    assert receipt["status"] == "ambiguous"
    assert receipt["deck_movement"]["semantic_state_committed"] is False
    assert receipt["deck_movement"]["stages"][-1]["terminal_state"] == "ambiguous"
    after = store.deck_semantic_state()
    assert after["current_location"] == before["current_location"]
    assert after["current_well"] == before["current_well"]
    assert after["ambiguity_state"] == "none"
    assert store.recovery()["hold"] is True
    assert store.claim_next() is None
    store.stop()


class _LatchPredicateProvider(_CompletedPrimitiveProvider):
    def __init__(self, table, *, host_latch, sensor_latch):
        super().__init__(table)
        self.host_latch = host_latch
        self.sensor_latch = sensor_latch
        self.force_invocations = 0

    def force_to_high_home(self, **_kwargs):
        self.force_invocations += 1
        return {"ok": True, "psudo_z_home_steps": 500}

    def deck_authority_snapshot(self, *, expected_generation):
        snapshot = super().deck_authority_snapshot(expected_generation=expected_generation)
        snapshot.update({
            "latch_status": self.host_latch,
            "machine_latch_closed": self.sensor_latch,
            "latch_observation_id": (
                f"deck-latch:host={int(self.host_latch)}:sensor={int(self.sensor_latch)}"
            ),
        })
        return snapshot


@pytest.mark.parametrize(
    ("host_latch", "sensor_latch", "failed_operation"),
    [
        (False, True, "check_latch_status"),
        (True, False, "check_machine_latch_closed"),
    ],
)
def test_each_unsafe_latch_predicate_is_persisted_from_its_own_observation(
    tmp_path, host_latch, sensor_latch, failed_operation
) -> None:
    store = _store(tmp_path)
    state = {
        "ownership_generation": 7,
        "serial206_initialization_provider": {
            "x_authority": {"active_board_epoch": 11},
            "board4_authority": {"active_board_epoch": 10},
        },
    }
    request = {
        "schema_version": ACTION_REQUEST_SCHEMA,
        "idempotency_key": f"unsafe-latch-{int(host_latch)}-{int(sensor_latch)}",
        "expected_ownership_generation": 7,
        "expected_board_epoch_by_board": {"4": 10, "5": 11},
        "action_id": "oem.deck.move_to_location",
        "inputs": {"target": "LOC_OC", "camera_offset": False},
    }
    admitted = store.admit_command(request, state=state)
    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    provider = _LatchPredicateProvider(
        table, host_latch=host_latch, sensor_latch=sensor_latch
    )
    executor = make_deck_command_executor(
        provider_getter=lambda: provider,
        position_table_provider=lambda: table,
        command_store=store,
    )
    assert store._acquire_owner() is True
    assert store.claim_next() is not None

    result = executor(
        command_id=admitted["command_id"], target="LOC_OC", camera_offset=False,
        expected_ownership_generation=7,
        expected_board_epoch_by_board={"4": 10, "5": 11},
    )

    assert result["ok"] is False and result["error"] == "latch_not_closed"
    assert provider.force_invocations == 1
    assert provider.invocations == 0
    stages = {
        row["operation"]: row
        for row in store.get_command(admitted["command_id"])["deck_movement"]["stages"]
    }
    assert stages["ForceToHighHome"]["terminal_state"] == "completed"
    assert stages[failed_operation]["terminal_state"] == "failed"
    assert stages[failed_operation]["terminal_evidence"]["reason"] == "source_predicate_not_satisfied"
    failed_evidence = stages[failed_operation]["terminal_evidence"]["provider_evidence"]
    assert failed_evidence["value"] is False
    assert failed_evidence["observation_id"] == (
        f"deck-latch:host={int(host_latch)}:sensor={int(sensor_latch)}"
    )
    other = (
        "check_machine_latch_closed"
        if failed_operation == "check_latch_status"
        else "check_latch_status"
    )
    assert stages[other]["terminal_state"] == "completed"
    assert stages[other]["terminal_evidence"]["provider_evidence"]["value"] is True
    store.stop()


class _EpochFenceProvider(_CompletedPrimitiveProvider):
    def __init__(self, table, epoch_snapshots):
        super().__init__(table)
        self.epoch_snapshots = list(epoch_snapshots)
        self.snapshot_reads = 0
        self.force_invocations = 0

    def force_to_high_home(self, **_kwargs):
        self.force_invocations += 1
        return {"ok": True}

    def deck_authority_snapshot(self, *, expected_generation):
        snapshot = super().deck_authority_snapshot(expected_generation=expected_generation)
        index = min(self.snapshot_reads, len(self.epoch_snapshots) - 1)
        board4, board5 = self.epoch_snapshots[index]
        self.snapshot_reads += 1
        snapshot["board_epoch_4"] = board4
        snapshot["board_epoch_5"] = board5
        return snapshot


def test_real_worker_store_post_io_exception_sets_durable_recovery_hold(tmp_path) -> None:
    store = _store(tmp_path)
    state = {
        "ownership_generation": 7,
        "serial206_initialization_provider": {
            "x_authority": {"active_board_epoch": 11},
            "board4_authority": {"active_board_epoch": 10},
        },
    }
    request = {
        "schema_version": ACTION_REQUEST_SCHEMA,
        "idempotency_key": "worker-post-io-hold",
        "expected_ownership_generation": 7,
        "expected_board_epoch_by_board": {"4": 10, "5": 11},
        "action_id": "oem.deck.move_to_location",
        "inputs": {"target": "LOC_OC", "camera_offset": False},
    }
    admitted = store.admit_command(request, state=state)
    before = store.deck_semantic_state()
    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    provider = _RaisingPrimitiveProvider(table)
    executor = make_deck_command_executor(
        provider_getter=lambda: provider,
        position_table_provider=lambda: table,
        command_store=store,
    )
    assert store._acquire_owner() is True
    claimed = store.claim_next()
    assert claimed is not None
    app = SimpleNamespace(state=SimpleNamespace(
        ownership_generation=7,
        oem_deck_command_executor=executor,
        oem_deck_command_assessment=lambda _state: {"enabled": True},
    ))
    plane = OperatorCommandPlane.__new__(OperatorCommandPlane)
    plane.app = app
    plane.store = store
    plane.machine_state_provider = lambda: state

    plane._dispatch_one(claimed)

    receipt = store.get_command(admitted["command_id"])
    delivery = store.connection.execute(
        "SELECT work_kind,work_identity,dispatch_attempt_id,plan_digest,owner_id,"
        "ownership_generation,board_epoch_4,board_epoch_5 "
        "FROM operator_plane_delivery_attempts WHERE command_id=?",
        (admitted["command_id"],),
    ).fetchall()
    assert provider.invocations == 1, receipt
    assert len(delivery) == 2
    assert [row["work_identity"] for row in delivery] == [
        "stage:0:ForceToHighHome",
        "stage:3:moveTo",
    ]
    for row in delivery:
        assert row["work_kind"] == "named_stage"
        assert row["dispatch_attempt_id"] == claimed["dispatch_attempt_id"]
        assert row["plan_digest"] == receipt["deck_movement"]["plan_digest"]
        assert row["owner_id"] == store.owner_id
        assert tuple(row[key] for key in (
            "ownership_generation", "board_epoch_4", "board_epoch_5",
        )) == (7, 10, 11)
    assert receipt["status"] == "ambiguous"
    assert receipt["controller_acknowledged"] is False
    assert receipt["terminal_evidence"]["delivery_attempted"] is True
    assert store.recovery()["hold"] is True
    after = store.deck_semantic_state()
    assert after["current_location"] == before["current_location"]
    assert after["current_well"] == before["current_well"]
    assert after["current_tray"] == before["current_tray"]
    assert receipt["deck_movement"]["semantic_state_committed"] is False
    assert receipt["deck_movement"]["stages"][-1]["terminal_state"] == "ambiguous"
    assert store.claim_next() is None
    with pytest.raises(HTTPException) as blocked:
        store.admit_command({**request, "idempotency_key": "worker-post-io-blocked"}, state=state)
    assert blocked.value.detail["error"] == "deck_recovery_hold"
    store.stop()


def test_semantic_commit_failure_after_controller_completion_is_recovery_required(tmp_path) -> None:
    store = _store(tmp_path)
    state = {
        "ownership_generation": 7,
        "serial206_initialization_provider": {
            "x_authority": {"active_board_epoch": 11},
            "board4_authority": {"active_board_epoch": 10},
        },
    }
    request = {
        "schema_version": ACTION_REQUEST_SCHEMA,
        "idempotency_key": "semantic-commit-failure",
        "expected_ownership_generation": 7,
        "expected_board_epoch_by_board": {"4": 10, "5": 11},
        "action_id": "oem.deck.move_to_location",
        "inputs": {"target": "LOC_OC", "camera_offset": False},
    }
    admitted = store.admit_command(request, state=state)
    before = store.deck_semantic_state()
    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    provider = _CompletedPrimitiveProvider(table)

    def fail_semantic_commit(*_args, **_kwargs):
        raise RuntimeError("injected SQLite publication failure")

    store.commit_deck_success = fail_semantic_commit
    executor = make_deck_command_executor(
        provider_getter=lambda: provider,
        position_table_provider=lambda: table,
        command_store=store,
    )
    assert store._acquire_owner() is True
    claimed = store.claim_next()
    assert claimed is not None
    plane = OperatorCommandPlane.__new__(OperatorCommandPlane)
    plane.app = SimpleNamespace(state=SimpleNamespace(
        ownership_generation=7,
        oem_deck_command_executor=executor,
        oem_deck_command_assessment=lambda _state: {"enabled": True},
    ))
    plane.store = store
    plane.machine_state_provider = lambda: state

    plane._dispatch_one(claimed)

    receipt = store.get_command(admitted["command_id"])
    detail = receipt["deck_movement"]
    after = store.deck_semantic_state()
    assert provider.invocations == 1
    assert receipt["status"] == "ambiguous"
    assert receipt["controller_acknowledged"] is True
    assert detail["controller_completion_verified"] is True
    assert detail["semantic_state_committed"] is False
    assert detail["ambiguity_state"] == "recovery_required"
    assert all(stage["terminal_state"] != "planned" for stage in detail["stages"])
    assert detail["stages"][-1]["terminal_evidence"]["controller_completion_verified"] is True
    assert after["current_location"] == before["current_location"]
    assert after["current_well"] == before["current_well"]
    assert store.recovery()["hold"] is True
    assert store.claim_next() is None
    store.stop()


def test_stage_persistence_failure_after_provider_completion_requires_durable_recovery(tmp_path) -> None:
    store = _store(tmp_path)
    state = {
        "ownership_generation": 7,
        "serial206_initialization_provider": {
            "x_authority": {"active_board_epoch": 11},
            "board4_authority": {"active_board_epoch": 10},
        },
    }
    request = {
        "schema_version": ACTION_REQUEST_SCHEMA,
        "idempotency_key": "stage-persistence-after-io",
        "expected_ownership_generation": 7,
        "expected_board_epoch_by_board": {"4": 10, "5": 11},
        "action_id": "oem.deck.move_to_location",
        "inputs": {"target": "LOC_OC", "camera_offset": False},
    }
    admitted = store.admit_command(request, state=state)
    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    provider = _CompletedPrimitiveProvider(table)
    original_terminalize = store.terminalize_deck_stage
    raised = False

    def fail_after_stage_persistence(command_id, step, **kwargs):
        nonlocal raised
        original_terminalize(command_id, step, **kwargs)
        if step.operation == "moveTo" and kwargs.get("result") is not None and not raised:
            raised = True
            raise RuntimeError("injected stage evidence persistence failure")

    store.terminalize_deck_stage = fail_after_stage_persistence
    executor = make_deck_command_executor(
        provider_getter=lambda: provider,
        position_table_provider=lambda: table,
        command_store=store,
    )
    assert store._acquire_owner() is True
    claimed = store.claim_next()
    assert claimed is not None
    plane = OperatorCommandPlane.__new__(OperatorCommandPlane)
    plane.app = SimpleNamespace(state=SimpleNamespace(
        oem_deck_command_executor=executor,
        oem_deck_command_assessment=lambda _state: {"enabled": True},
    ))
    plane.store = store
    plane.machine_state_provider = lambda: state

    plane._dispatch_one(claimed)

    receipt = store.get_command(admitted["command_id"])
    assert provider.invocations == 1
    assert receipt["status"] == "ambiguous"
    assert receipt["controller_acknowledged"] is True
    assert receipt["deck_movement"]["controller_completion_verified"] is True
    assert receipt["deck_movement"]["semantic_state_committed"] is False
    assert receipt["deck_movement"]["ambiguity_state"] == "recovery_required"
    assert store.deck_semantic_state()["ambiguity_state"] == "none"
    assert store.recovery()["hold"] is True
    assert store.claim_next() is None
    store.stop()


def test_outer_terminalization_failure_after_semantic_commit_publishes_deck_hold(tmp_path) -> None:
    store = _store(tmp_path)
    state = {
        "ownership_generation": 7,
        "serial206_initialization_provider": {
            "x_authority": {"active_board_epoch": 11},
            "board4_authority": {"active_board_epoch": 10},
        },
    }
    request = {
        "schema_version": ACTION_REQUEST_SCHEMA,
        "idempotency_key": "outer-terminal-persistence-after-commit",
        "expected_ownership_generation": 7,
        "expected_board_epoch_by_board": {"4": 10, "5": 11},
        "action_id": "oem.deck.move_to_location",
        "inputs": {"target": "LOC_OC", "camera_offset": False},
    }
    admitted = store.admit_command(request, state=state)
    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    provider = _CompletedPrimitiveProvider(table)
    executor = make_deck_command_executor(
        provider_getter=lambda: provider,
        position_table_provider=lambda: table,
        command_store=store,
    )
    assert store._acquire_owner() is True
    claimed = store.claim_next()
    assert claimed is not None
    plane = OperatorCommandPlane.__new__(OperatorCommandPlane)
    plane.app = SimpleNamespace(state=SimpleNamespace(
        oem_deck_command_executor=executor,
        oem_deck_command_assessment=lambda _state: {"enabled": True},
    ))
    plane.store = store
    plane.machine_state_provider = lambda: state
    original_finish = store.finish
    failed_once = False

    def fail_completed_terminalization(command_id, *, status, **kwargs):
        nonlocal failed_once
        if status == "completed" and not failed_once:
            failed_once = True
            raise RuntimeError("injected outer terminal commit failure")
        return original_finish(command_id, status=status, **kwargs)

    store.finish = fail_completed_terminalization
    plane.store._dispatch_worker(plane._dispatch_one, claimed)

    receipt = store.get_command(admitted["command_id"])
    semantic = store.deck_semantic_state()
    assert provider.invocations == 1
    assert receipt["status"] == "ambiguous"
    assert receipt["deck_movement"]["controller_completion_verified"] is True
    assert receipt["deck_movement"]["ambiguity_state"] == "recovery_required"
    assert semantic["current_location"] == "LOC_OC"
    assert semantic["ambiguity_state"] == "none"
    assert store.recovery()["hold"] is True
    assert store.claim_next() is None
    store.stop()


@pytest.mark.parametrize(
    ("epoch_snapshots", "expected_status", "expected_force", "expected_moves"),
    [
        ([(99, 11)], "failed", 0, 0),
        ([(10, 11), (10, 11), (10, 11)], "completed", 1, 1),
        ([(10, 11), (10, 11), (99, 11)], "failed", 1, 0),
    ],
)
def test_admitted_board_epochs_are_exact_execution_fence(
    tmp_path, epoch_snapshots, expected_status, expected_force, expected_moves
) -> None:
    store = _store(tmp_path)
    state = {
        "ownership_generation": 7,
        "serial206_initialization_provider": {
            "x_authority": {"active_board_epoch": 11},
            "board4_authority": {"active_board_epoch": 10},
        },
    }
    admitted = store.admit_command({
        "schema_version": ACTION_REQUEST_SCHEMA,
        "idempotency_key": f"board-fence-{expected_status}-{expected_force}",
        "expected_ownership_generation": 7,
        "expected_board_epoch_by_board": {"4": 10, "5": 11},
        "action_id": "oem.deck.move_to_location",
        "inputs": {"target": "LOC_OC", "camera_offset": False},
    }, state=state)
    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    provider = _EpochFenceProvider(table, epoch_snapshots)
    executor = make_deck_command_executor(
        provider_getter=lambda: provider,
        position_table_provider=lambda: table,
        command_store=store,
    )
    assert store._acquire_owner() is True
    claimed = store.claim_next()
    assert claimed is not None
    assert claimed["expected_board_epoch_by_board"] == {"4": 10, "5": 11}
    plane = OperatorCommandPlane.__new__(OperatorCommandPlane)
    plane.app = SimpleNamespace(state=SimpleNamespace(
        ownership_generation=7,
        oem_deck_command_executor=executor,
        oem_deck_command_assessment=lambda _state: {"enabled": True},
    ))
    plane.store = store
    plane.machine_state_provider = lambda: state

    plane._dispatch_one(claimed)

    receipt = store.get_command(admitted["command_id"])
    assert receipt["status"] == expected_status
    assert provider.force_invocations == expected_force
    assert provider.invocations == expected_moves
    if expected_status == "failed":
        assert "board_epoch_fence_stale" in receipt["terminal_evidence"]["detail"]
        assert receipt["terminal_evidence"]["delivery_attempted"] is False
    store.stop()
