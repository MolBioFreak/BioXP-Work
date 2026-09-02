from __future__ import annotations

import json
import time
from contextlib import contextmanager, nullcontext
from types import SimpleNamespace
from typing import Any, cast

from fastapi import FastAPI, HTTPException
from fastapi.testclient import TestClient
import pytest

from bioxp import operator_controls
from bioxp.operator_command_plane import OperatorCommandStore
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.oem_deck_catalog import DeckCatalog
from bioxp.oem_deck_movement import WP8_COMPILED_CHILD_OPERATIONS
from bioxp.oem_serial206_initialization import (
    SERIAL206_INITIALIZE_MOTION_STAGE_SPECS,
    Serial206OemInitializationProvider,
    Serial206ProductionPrimitiveAdapter,
)
from bioxp import oem_serial206_initialization
from bioxp.oem_compat.position_table import PositionTable


def _operator_store(root):
    runtime_owner = OEMRuntimeStore(root)
    runtime_owner.close()
    return OperatorCommandStore(root)


@pytest.fixture(autouse=True)
def _prepare_canonical_runtime(tmp_path):
    runtime_owner = OEMRuntimeStore(tmp_path / "canonical-oem-runtime")
    runtime_owner.close()


def _complete_oem_movable_locations() -> dict[str, str]:
    # ClassMachineStatus.cs:419-453 constructor-owned defaults.
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


class FakeDeckProvider:
    def __init__(self, table: PositionTable) -> None:
        self.table = table
        self.calls: list[tuple[str, object]] = []
        self.tip_tray_reader = None
        self.tip_tray_publisher = None

    def bind_tip_tray_state_reader(self, reader):
        self.tip_tray_reader = reader

    def bind_tip_tray_state_publisher(self, publisher):
        self.tip_tray_publisher = publisher

    def deck_semantic_bootstrap_snapshot(self, *, expected_generation: int):
        return {
            "current_location": "LOC_MS", "current_well": 0, "current_tray": None,
            "tip_loaded": False, "tip_dirty": False, "tip_location": -1, "clean_path": True,
            "plate_on_gantry": None,
            "movable_plate_locations": _complete_oem_movable_locations(),
            "pseudo_z_home": 65000,
            "ownership_generation": expected_generation,
            "board_epoch_4": 10,
            "board_epoch_5": 11,
            "latch_status": True,
            "machine_latch_closed": True,
            "latch_observation_id": "fake-bootstrap-latch",
            "source_operation": "test_bootstrap",
            "source_command_id": "fake-bootstrap",
        }

    @contextmanager
    def movement_lease(self):
        self.calls.append(("lease", "acquired"))
        yield

    def deck_authority_snapshot(self, *, expected_generation: int):
        self.calls.append(("snapshot", expected_generation))
        return {
            "ownership_generation": expected_generation,
            "provider_owner_id": "fake-provider",
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
            "pseudo_z_home": 65000,
            "device_type": "BIOXP",
            "latch_status": True,
            "machine_latch_closed": True,
        }

    def force_to_high_home(self, *, command_id: str):
        self.calls.append(("force_to_high_home", command_id))
        return {"ok": True, "semantic_state_revision": 2}

    def moveTo(self, **kwargs):
        self.calls.append(("moveTo", kwargs))
        return {
            "ok": True,
            "provider_command_id": "provider-1",
            "controller_command_acknowledged": True,
            "controller_completion_verified": True,
            "hardware_postcondition_verified": True,
        }

    def moveZCamera(self, **kwargs):
        self.calls.append(("moveZCamera", kwargs))
        return self.moveTo(**kwargs)

    def parkGantry(self, **kwargs):
        self.calls.append(("parkGantry", kwargs))
        return self.moveTo(location_id=28, camera_offset=False)


class AmbiguousFakeDeckProvider(FakeDeckProvider):
    def moveTo(self, **kwargs):
        self.calls.append(("moveTo", kwargs))
        return {"ok": False, "controller_command_acknowledged": True}


class BootstrapFakeDeckProvider(FakeDeckProvider):
    def __init__(self, table: PositionTable) -> None:
        super().__init__(table)
        self.semantic_reader = None

    def bind_deck_semantic_state_reader(self, reader):
        self.semantic_reader = reader

    def deck_semantic_bootstrap_snapshot(self, *, expected_generation: int):
        return {
            "current_location": "LOC_MS", "current_well": 0, "current_tray": None,
            "tip_loaded": False, "tip_dirty": False, "tip_location": -1, "clean_path": True,
            "plate_on_gantry": None, "movable_plate_locations": _complete_oem_movable_locations(), "pseudo_z_home": 65000,
            "ownership_generation": expected_generation, "board_epoch_4": 10, "board_epoch_5": 11,
            "latch_status": True, "machine_latch_closed": True,
            "latch_observation_id": "bootstrap-latch-1",
            "source_operation": "migrated_successful_semantic_state",
            "source_command_id": "legacy-runtime-command-1",
        }

    def deck_authority_snapshot(self, *, expected_generation: int):
        assert callable(self.semantic_reader)
        semantic: dict[str, object] = self.semantic_reader()
        revision = semantic["semantic_state_revision"]
        assert type(revision) is int and revision >= 1
        result = super().deck_authority_snapshot(expected_generation=expected_generation)
        result["machine_state_revision"] = semantic["semantic_state_revision"]
        return result


class ReconciliationFakeDeckProvider(AmbiguousFakeDeckProvider):
    def __init__(self, table: PositionTable) -> None:
        super().__init__(table)
        self.semantic_reader = None
        self.reconciliation_overrides: dict[str, object] = {}
        self.final_reconciliation_overrides: dict[str, object] = {}
        self.lease_depth = 0
        self.snapshot_lease_depths: list[int] = []

    @contextmanager
    def movement_lease(self):
        self.lease_depth += 1
        self.calls.append(("lease", "acquired"))
        try:
            yield
        finally:
            self.lease_depth -= 1

    def bind_deck_semantic_state_reader(self, reader):
        self.semantic_reader = reader

    def deck_reconciliation_snapshot(self, *, expected_generation: int):
        self.snapshot_lease_depths.append(self.lease_depth)
        self.calls.append(("snapshot", expected_generation))
        semantic = self.semantic_reader()
        return {
            "ownership_generation": expected_generation,
            "provider_owner_id": "fake-provider",
            "board_epoch_4": 10,
            "board_epoch_5": 11,
            "position_table_sha256": self.table.digest,
            "machine_state_revision": semantic["semantic_state_revision"],
            "latch_status": True,
            "machine_latch_closed": True,
            "latch_observation_id": "server-latch-1",
            "controller_position_observation_id": "server-controller-position-1",
            "captured_at": 123.0,
            "current_x": 101,
            "current_y": 202,
            "current_z": 65000,
            "observed_location_id": "LOC_MS",
            "observed_well_id": 0,
            **self.reconciliation_overrides,
            **(self.final_reconciliation_overrides if len(self.snapshot_lease_depths) > 1 else {}),
        }


def _install_reconciliation_app(tmp_path, monkeypatch):
    from bioxp.oem_deck_catalog import configured_location_names

    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    provider = ReconciliationFakeDeckProvider(table)
    app = FastAPI()
    monkeypatch.setenv("BIOXP_RUNTIME_STATE_ROOT", str(tmp_path))
    monkeypatch.setattr(operator_controls.hardware_state, "project", lambda _name: {"domains": {}, "freshness": {"state": "fresh", "age_s": 0, "fresh_for_s": 30}})
    monkeypatch.setattr(operator_controls.hardware_state, "ownership_projection", lambda: {"ownership_epoch": 7, "ownership": "service"})
    operator_controls.install_operator_control_plane(
        app,
        maintenance_state_provider=lambda: {"motion_blocked": False, "recovery_required": False, "block_reason": None},
        reference_state_provider=lambda: {"rows": {}},
        lifecycle_state_provider=lambda: {"operation_state": "stopped"},
        serial206_initialization_state_provider=lambda: {
            "x_authority": {"active_board_epoch": 11},
            "board4_authority": {"active_board_epoch": 10},
        },
        oem_deck_provider=lambda: provider,
        oem_deck_position_table_provider=lambda: table,
    )
    app.state.operator_command_plane.start()
    return app, provider, table


def _admit_ambiguous_deck(client: TestClient) -> str:
    admitted = client.post("/operator/v2/actions/oem.deck.move_to_location", json={
        "schema_version": "bioxp.operator_action_request.v2",
        "idempotency_key": "deck-route-ambiguous-1",
        "expected_ownership_generation": 7,
        "expected_board_epoch_by_board": {"4": 10, "5": 11},
        "inputs": {"target": "LOC_OC", "camera_offset": False},
    })
    assert admitted.status_code == 200
    command_id = admitted.json()["command_id"]
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        receipt = client.get(f"/operator/commands/{command_id}").json()
        if receipt["status"] == "ambiguous":
            return command_id
        time.sleep(0.01)
    raise AssertionError(receipt)


def _reconciliation_body(**overrides):
    body = {
        "schema_version": "bioxp.operator_deck_reconciliation_request.v1",
        "operator_ack": "RECONCILE_DECK",
        "decision_id": "governed-decision-1",
        "approved_by": "operator-test",
        "reason": "source-compatible operator reconciliation",
        "current_location": "LOC_MS",
        "current_well": 0,
    }
    body.update(overrides)
    return body


def test_deck_provider_refreshes_after_startup_without_usb_owner(tmp_path, monkeypatch):
    from bioxp.oem_deck_catalog import configured_location_names

    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    provider_slot: dict[str, Any] = {"current": None}
    app = FastAPI()
    monkeypatch.setenv("BIOXP_RUNTIME_STATE_ROOT", str(tmp_path))
    monkeypatch.setattr(operator_controls.hardware_state, "project", lambda _name: {"domains": {}, "freshness": {"state": "fresh", "age_s": 0, "fresh_for_s": 30}})
    monkeypatch.setattr(operator_controls.hardware_state, "ownership_projection", lambda: {"ownership_epoch": 7, "ownership": "service"})
    operator_controls.install_operator_control_plane(
        app,
        maintenance_state_provider=lambda: {"motion_blocked": False, "recovery_required": False, "block_reason": None},
        reference_state_provider=lambda: {"rows": {}},
        lifecycle_state_provider=lambda: {"operation_state": "stopped"},
        serial206_initialization_state_provider=lambda: {
            "x_authority": {"active_board_epoch": 11},
            "board4_authority": {"active_board_epoch": 10},
        },
        oem_deck_provider=lambda: provider_slot["current"],
        oem_deck_position_table_provider=lambda: table,
    )
    assert app.state.oem_deck_provider is None

    provider = BootstrapFakeDeckProvider(table)
    provider_slot["current"] = provider
    response = TestClient(app).get("/operator/v2/control-catalog")

    assert response.status_code == 200
    assert app.state.oem_deck_provider is provider
    assert callable(provider.semantic_reader)
    assert callable(provider.tip_tray_reader)
    assert callable(provider.tip_tray_publisher)


def test_authenticated_deck_reconciliation_route_uses_server_provider_truth(tmp_path, monkeypatch):
    app, provider, _table = _install_reconciliation_app(tmp_path, monkeypatch)
    client = TestClient(app)
    command_id = _admit_ambiguous_deck(client)

    unauthorized = client.post(
        f"/operator/recovery/deck/{command_id}/reconcile",
        json=_reconciliation_body(operator_ack="WRONG"),
    )
    assert unauthorized.status_code == 422

    result = client.post(
        f"/operator/recovery/deck/{command_id}/reconcile",
        json=_reconciliation_body(),
    )

    assert result.status_code == 200, result.text
    body = result.json()
    assert body["command_id"] == command_id
    assert body["controller_position_observation"]["observation_id"] == "server-controller-position-1"
    assert body["controller_position_observation"]["x"] == 101
    assert provider.snapshot_lease_depths == [1, 1]
    semantic = app.state.operator_command_plane.store.deck_semantic_state()
    assert semantic["transition_provenance"]["reconciliation_decision"]["decision_id"] == "governed-decision-1"
    assert semantic["transition_provenance"]["controller_position_observation"]["observation_id"] == "server-controller-position-1"
    assert semantic["ambiguity_state"] == "none"
    assert app.state.operator_command_plane.store.recovery()["hold"] is False
    app.state.operator_command_plane.stop()


@pytest.mark.parametrize(
    ("provider_override", "body_override"),
    [
        ({"ownership_generation": 8}, {}),
        ({"board_epoch_4": 99}, {}),
        ({"machine_state_revision": 999}, {}),
        ({"position_table_sha256": "0" * 64}, {}),
        ({"controller_position_observation_id": ""}, {}),
        ({"observed_location_id": "LOC_RC"}, {}),
        ({"observed_well_id": 1}, {}),
        ({}, {"current_location": "UNKNOWN_LOCATION"}),
        ({}, {"current_well": 96}),
    ],
)
def test_deck_reconciliation_route_rejects_stale_or_malformed_truth_without_change(
    tmp_path, monkeypatch, provider_override, body_override
):
    app, provider, _table = _install_reconciliation_app(tmp_path, monkeypatch)
    client = TestClient(app)
    command_id = _admit_ambiguous_deck(client)
    store = app.state.operator_command_plane.store
    before_semantic = store.deck_semantic_state()
    before_recovery = store.recovery()
    provider.reconciliation_overrides = provider_override

    response = client.post(
        f"/operator/recovery/deck/{command_id}/reconcile",
        json=_reconciliation_body(**body_override),
    )

    assert response.status_code in {409, 422, 503}
    assert store.deck_semantic_state() == before_semantic
    assert store.recovery() == before_recovery
    app.state.operator_command_plane.stop()


def test_deck_reconciliation_final_authority_drift_rolls_back_without_clearing_hold(tmp_path, monkeypatch):
    app, provider, _table = _install_reconciliation_app(tmp_path, monkeypatch)
    client = TestClient(app)
    command_id = _admit_ambiguous_deck(client)
    store = app.state.operator_command_plane.store
    before_semantic = store.deck_semantic_state()
    before_recovery = store.recovery()
    provider.final_reconciliation_overrides = {
        "controller_position_observation_id": "drifted-position-observation",
        "observed_location_id": "LOC_RC",
    }

    response = client.post(
        f"/operator/recovery/deck/{command_id}/reconcile", json=_reconciliation_body()
    )

    assert response.status_code == 409
    assert provider.snapshot_lease_depths == [1, 1]
    assert store.deck_semantic_state() == before_semantic
    assert store.recovery() == before_recovery
    app.state.operator_command_plane.stop()


def test_deck_reconciliation_route_rejects_missing_provider_truth_and_missing_hold(tmp_path, monkeypatch):
    app, provider, _table = _install_reconciliation_app(tmp_path, monkeypatch)
    client = TestClient(app)
    command_id = _admit_ambiguous_deck(client)
    store = app.state.operator_command_plane.store
    before = store.deck_semantic_state()
    provider.deck_reconciliation_snapshot = None

    missing_truth = client.post(
        f"/operator/recovery/deck/{command_id}/reconcile",
        json=_reconciliation_body(),
    )
    assert missing_truth.status_code == 503
    assert store.deck_semantic_state() == before

    provider.deck_reconciliation_snapshot = lambda **_kwargs: {
        "ownership_generation": 7, "provider_owner_id": "fake-provider",
        "board_epoch_4": 10, "board_epoch_5": 11,
        "position_table_sha256": provider.table.digest,
        "machine_state_revision": before["semantic_state_revision"],
        "controller_position_observation_id": "server-controller-position-2",
        "captured_at": 124.0, "current_x": 1, "current_y": 2, "current_z": 3,
    }
    store.connection.execute("UPDATE operator_plane_safety SET recovery_hold=0 WHERE singleton=1")
    missing_hold = client.post(
        f"/operator/recovery/deck/{command_id}/reconcile",
        json=_reconciliation_body(),
    )
    assert missing_hold.status_code == 409
    assert store.deck_semantic_state() == before
    app.state.operator_command_plane.stop()


def test_install_binds_reachable_executor_and_queue_reaches_fake_provider_async(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_RUNTIME_STATE_ROOT", str(tmp_path))
    from bioxp.oem_deck_catalog import configured_location_names
    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    provider = BootstrapFakeDeckProvider(table)
    app = FastAPI()
    maintenance = {
        "motion_blocked": False,
        "recovery_required": False,
        "block_reason": None,
    }
    lifecycle = {"operation_state": "stopped"}
    monkeypatch.setattr(operator_controls.hardware_state, "project", lambda _name: {"domains": {}, "freshness": {"state": "fresh", "age_s": 0, "fresh_for_s": 30}})
    monkeypatch.setattr(operator_controls.hardware_state, "ownership_projection", lambda: {"ownership_epoch": 7, "ownership": "service"})

    operator_controls.install_operator_control_plane(
        app,
        maintenance_state_provider=lambda: maintenance,
        reference_state_provider=lambda: {"rows": {}},
        lifecycle_state_provider=lambda: lifecycle,
        serial206_initialization_state_provider=lambda: {
            "x_authority": {"active_board_epoch": 11},
            "board4_authority": {"active_board_epoch": 10},
        },
        oem_deck_provider=lambda: provider,
        oem_deck_position_table_provider=lambda: table,
    )
    app.state.operator_command_plane.start()
    assert callable(app.state.oem_deck_command_executor)
    assert callable(app.state.oem_mov_execution_admitter)
    assert callable(app.state.oem_wp8_operation_admitter)
    assert provider.tip_tray_reader is not None
    assert provider.tip_tray_reader(0)["tip_available"] is None
    assert provider.tip_tray_publisher is not None
    catalog_response = TestClient(app).get("/operator/v2/control-catalog")
    assert catalog_response.status_code == 200
    deck_action = next(
        row for row in catalog_response.json()["actions"]
        if row["action_id"] == "oem.deck.move_to_location"
    )
    assert deck_action["enabled"] is True
    assert deck_action["required_boards"] == [4, 5]
    assert deck_action["expected_board_epoch_by_board"] == {"4": 10, "5": 11}
    assert deck_action["position_table_revision"] == table.digest
    assert len(deck_action["destination_options"]) == 26

    assess_deck = app.state.oem_deck_command_assessment
    valid_state = {
        "ownership_generation": 7,
        "maintenance": {
            "motion_blocked": False,
            "recovery_required": False,
            "block_reason": None,
        },
        "lifecycle": {"operation_state": "stopped"},
    }
    assert assess_deck(valid_state)["enabled"] is True
    for bad_maintenance in (
        None,
        {},
        {"motion_blocked": False},
        {"recovery_required": False},
        {"motion_blocked": 0, "recovery_required": False, "block_reason": None},
        {"motion_blocked": False, "recovery_required": 0, "block_reason": None},
        {"motion_blocked": False, "recovery_required": False, "block_reason": "stale block"},
    ):
        assessment = assess_deck({**valid_state, "maintenance": bad_maintenance})
        assert assessment["enabled"] is False
        assert assessment["disabled_reason"] is not None
    for bad_lifecycle in (
        None,
        {},
        {"operation_state": None},
        {"operation_state": "ready"},
        {"operation_state": "waiting"},
        {"operation_state": "running"},
        {"operation_state": "paused"},
        {"operation_state": "error"},
        {"operation_state": "emergency"},
    ):
        assessment = assess_deck({**valid_state, "lifecycle": bad_lifecycle})
        assert assessment["enabled"] is False
        assert assessment["disabled_reason"] is not None

    maintenance["motion_blocked"] = True
    blocked_action = next(
        row for row in TestClient(app).get("/operator/v2/control-catalog").json()["actions"]
        if row["action_id"] == "oem.deck.move_to_location"
    )
    assert blocked_action["enabled"] is False
    maintenance["motion_blocked"] = False
    lifecycle["operation_state"] = "emergency"
    emergency_action = next(
        row for row in TestClient(app).get("/operator/v2/control-catalog").json()["actions"]
        if row["action_id"] == "oem.deck.move_to_location"
    )
    assert emergency_action["enabled"] is False
    lifecycle["operation_state"] = "stopped"

    payload = {
        "schema_version": "bioxp.operator_action_request.v2",
        "idempotency_key": "deck-install-e2e-1",
        "expected_ownership_generation": 7,
        "expected_board_epoch_by_board": {"4": 10, "5": 11},
        "inputs": {"target": "LOC_OC", "camera_offset": False},
    }
    response = TestClient(app).post("/operator/v2/actions/oem.deck.move_to_location", json=payload)
    assert response.status_code == 200
    command_id = response.json()["command_id"]
    deadline = time.monotonic() + 3
    detail = None
    while time.monotonic() < deadline:
        detail = TestClient(app).get(f"/operator/commands/{command_id}").json()
        if detail["status"] not in {"queued", "dispatched", "issued_pending"}:
            break
        time.sleep(0.01)
    assert detail is not None and detail["status"] == "completed", json.dumps(detail, indent=2)
    assert any(name == "moveTo" for name, _ in provider.calls)
    detail_v2 = TestClient(app).get(f"/operator/v2/actions/receipts/{command_id}?detail=true")
    assert detail_v2.status_code == 200
    assert detail_v2.json()["deck_movement"]["semantic_state_committed"] is True
    dashboard = TestClient(app).get("/operator/v2/dashboard")
    assert dashboard.status_code == 200
    assert dashboard.json()["deck"] == {
        "current_location": "LOC_OC",
        "current_well": 0,
        "semantic_state_revision": 3,
        "position_table_revision": table.digest,
        "destination_catalog_revision": deck_action["destination_catalog_revision"],
        "ambiguity_state": "none",
    }
    assert command_id in {
        row["command_id"] for row in dashboard.json()["latest_receipts"]
    }

    direct_command_id = "direct-projection-newer"
    app.state.operator_receipt_store.put({
        "schema_version": "bioxp.operator_action_receipt.v1",
        "command_id": direct_command_id,
        "idempotency_key": "direct-projection-newer-key",
        "action_id": "query.status",
        "status": "completed",
        "ownership_generation": 7,
        "started_at": str(time.time() + 100),
        "finished_at": str(time.time() + 101),
        "controller_acknowledged": False,
        "physical_effect_verified": False,
        "response": {"http_status": 200, "body": {"ok": True}},
        "stage_receipts": [],
    })
    first_page = TestClient(app).get("/operator/v2/actions/history", params={"limit": 1}).json()
    assert [row["command_id"] for row in first_page["items"]] == [direct_command_id]
    assert first_page["next_cursor"]
    second_page = TestClient(app).get(
        "/operator/v2/actions/history",
        params={"limit": 1, "cursor": first_page["next_cursor"]},
    ).json()
    assert [row["command_id"] for row in second_page["items"]] == [command_id]
    catalog_after = TestClient(app).get("/operator/v2/control-catalog").json()
    assert {direct_command_id, command_id}.issubset({
        row["command_id"] for row in catalog_after["dashboard"]["latest_receipts"]
    })

    durable_ids = set()
    for index in range(205):
        admitted_bulk = app.state.operator_command_plane.store.admit_command(
            {
                "idempotency_key": f"durable-bulk-{index:03d}",
                "expected_ownership_generation": 7,
                "expected_board_epoch_by_board": {},
                "action_id": "oem.x.move_steps",
                "inputs": {"steps": 1},
            },
            state={"ownership_generation": 7},
        )
        durable_ids.add(admitted_bulk["command_id"])
    bulk_first = TestClient(app).get(
        "/operator/v2/actions/history", params={"limit": 200},
    ).json()
    assert len(bulk_first["items"]) == 200
    bulk_second = TestClient(app).get(
        "/operator/v2/actions/history",
        params={"limit": 200, "cursor": bulk_first["next_cursor"]},
    ).json()
    paged_ids = {
        row["command_id"]
        for row in [*bulk_first["items"], *bulk_second["items"]]
    }
    assert durable_ids.issubset(paged_ids)

    app.state.operator_command_plane.stop()


class Wp8BootstrapFakeDeckProvider(BootstrapFakeDeckProvider):
    def deck_owner_authority_stamps(self):
        return {"ownership_generation": 7, "board_epoch_4": 10, "board_epoch_5": 11}

    def wp8_operation_machine_state(self, operation, intent_inputs):
        assert operation == "send_z_and_gripper_home"
        assert intent_inputs == {"run_in_parallel": False}
        return {"gripper_position": 0, "closed_position": 3000}

    def execute_wp8_child(self, child, *, command_id, child_order, plan_digest):
        self.calls.append(("wp8", {
            "child": dict(child),
            "command_id": command_id,
            "child_order": child_order,
            "plan_digest": plan_digest,
        }))
        return {
            "ok": True,
            "controller_command_acknowledged": True,
            "controller_completion_verified": True,
            "hardware_postcondition_verified": True,
        }


class Wp8PrimitiveAllowlistFake:
    def __init__(self):
        self.calls = []
        self.tester = self

    def sleep(self, milliseconds):
        self.calls.append(("sleep", milliseconds))
        return {"ok": True}

    def unrelated_mutation(self):
        self.calls.append(("unrelated_mutation", None))
        return {"ok": True}

    def rPunchFoil(self, **arguments):
        self.calls.append(("rPunchFoil", arguments))
        return {"ok": True, "controller_command_acknowledged": True, "controller_completion_verified": True}

    def CirclePunch(self, **arguments):
        self.calls.append(("CirclePunch", arguments))
        return {"ok": True, "controller_command_acknowledged": True, "controller_completion_verified": True}

    def z_move_z_home(self, *, timeout_s):
        self.calls.append(("z_move_z_home", timeout_s))
        return {"ok": True, "controller_command_acknowledged": True, "controller_completion_verified": True}

    def setGripperCurrent(self, current):
        self.calls.append(("dynamic_setGripperCurrent", current))
        return {"ok": True}

    def motor_set_axis_param(self, board, parameter, value, *, motor):
        self.calls.append(("motor_set_axis_param", board, motor, parameter, value))
        return {"ok": True}

    def motor_get_axis_param(self, board, parameter, *, motor):
        self.calls.append(("motor_get_axis_param", board, motor, parameter))
        return {"ok": True, "value": 31}


def test_production_wp8_dispatch_is_explicit_and_rejects_uncompiled_primitive() -> None:
    provider = object.__new__(Serial206OemInitializationProvider)
    provider.primitives = Wp8PrimitiveAllowlistFake()
    assert provider.execute_wp8_child({
        "order": 0, "operation": "Sleep", "arguments": {"milliseconds": 5},
    }, command_id="cmd-1", child_order=0, plan_digest="plan-1") == {"ok": True}
    homed = provider.execute_wp8_child(
        {
            "order": 1,
            "operation": "MoveZHome",
            "arguments": {},
        },
        command_id="cmd-1", child_order=1, plan_digest="plan-1",
    )
    assert homed["source_anchor"] == "ControlLib.movExecution:w:MoveZHome"
    current = provider.execute_wp8_child(
        {
            "order": 2,
            "operation": "setGripperCurrent",
            "arguments": {"current": 31},
        },
        command_id="cmd-1", child_order=2, plan_digest="plan-1",
    )
    assert current["source_anchor"] == "ClassControlInterface.setGripperCurrent"
    with pytest.raises(RuntimeError, match="source_authority_missing:unrelated_mutation"):
        provider.execute_wp8_child(
            {"order": 1, "operation": "unrelated_mutation", "arguments": {}},
            command_id="cmd-1", child_order=1, plan_digest="plan-1",
        )
    assert provider.primitives.calls == [
        ("sleep", 5),
        ("z_move_z_home", 30.0),
        ("motor_set_axis_param", 4, 2, 6, 31),
        ("motor_get_axis_param", 4, 2, 6),
    ]


def test_production_wp8_dispatch_inventory_closes_compiler_denominator() -> None:
    assert set(Serial206OemInitializationProvider.wp8_child_binding_inventory()) == set(
        WP8_COMPILED_CHILD_OPERATIONS
    )
    assert {
        operation: handler_name
        for operation, handler_name in Serial206OemInitializationProvider._WP8_CHILD_BINDINGS.items()
        if not callable(getattr(Serial206OemInitializationProvider, handler_name, None))
    } == {}


def test_production_wp8_z_location_binding_uses_canonical_location_name(monkeypatch) -> None:
    from bioxp import oem_serial206_initialization as initialization

    class Row:
        z_low = 12345

    class Table:
        def rows(self):
            return [{"location_id": "LOC_P_TC_PRESS"}]

        def resolve(self, *, location_id):
            assert location_id == "LOC_P_TC_PRESS"
            return Row()

    class Primitives:
        def __init__(self):
            self.calls = []

        def oem_move_z(self, target, **kwargs):
            self.calls.append((target, kwargs))
            return {
                "ok": True,
                "controller_command_acknowledged": True,
                "controller_completion_verified": True,
            }

    monkeypatch.setattr(initialization, "load_bound_oem_position_table", lambda: Table())
    provider = object.__new__(Serial206OemInitializationProvider)
    provider.primitives = Primitives()
    provider.mov_execution_machine_state = lambda: {"pseudo_z_home": 65000}
    result = provider.execute_wp8_child(
        {
            "order": 0,
            "operation": "moveZPress",
            "arguments": {"pressure_target": 24, "z_low_offset": -2200, "wait": True},
        },
        command_id="cmd-z", child_order=0, plan_digest="plan-z",
    )
    assert result["ok"] is True
    assert provider.primitives.calls == [
        (10145, {"pseudo_home_steps": 65000, "motor_current": 31, "wait_for_stop": True})
    ]


def test_production_mov_execution_continuation_leaves_preserve_oem_arguments() -> None:
    provider = object.__new__(Serial206OemInitializationProvider)
    provider.primitives = Wp8PrimitiveAllowlistFake()
    assert provider.rPunchFoil(plate_name=2, location_id=3)["ok"] is True
    assert provider.CirclePunch(destination=2, column=0, row=0)["ok"] is True
    assert provider.primitives.calls == [
        ("rPunchFoil", {"plate_name": 2, "location_id": 3}),
        ("CirclePunch", {"destination": 2, "column": 0, "row": 0}),
    ]


def test_production_wp8_snapshot_owns_gripper_branch_inputs(monkeypatch) -> None:
    from bioxp import oem_initialization

    monkeypatch.setattr(
        oem_initialization,
        "build_machine_calibration_manifest",
        lambda **_kwargs: {
            "ok": True,
            "gripper": {"GripperClosePOS": {"value": 27350}},
            "thermal_door": {
                "TCDoorOpen": {"value": 2000},
                "TCDoorStallGuardThreshold": {"value": 17},
                "TC_DOOR_MAX_CURRENT": {"value": 80},
            },
        },
    )
    provider = object.__new__(Serial206OemInitializationProvider)
    provider.mov_execution_machine_state = lambda: {
        "gripper_position": 0,
        "plate_locations": {1: 21, 4: 29, 5: 20},
        "plate_on_gantry": 1,
        "tip_loaded": True,
        "thermal_door_open": False,
    }
    provider.primitives = SimpleNamespace(
        tester=SimpleNamespace(motor_get_position=lambda board, motor: {"position": 12345})
    )
    snapshot = provider.wp8_operation_machine_state(
        "send_z_and_gripper_home", {"run_in_parallel": False},
    )
    assert snapshot["gripper_position"] == 12345
    assert snapshot["closed_position"] == 27350


def test_mov_execution_snapshot_carries_server_owned_wp8_branch_state(monkeypatch) -> None:
    from bioxp import oem_serial206_initialization as serial_module
    from bioxp.oem_deck_catalog import configured_location_names

    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    monkeypatch.setattr(serial_module, "load_bound_oem_position_table", lambda: table)
    provider = object.__new__(Serial206OemInitializationProvider)
    provider._canonical_deck_semantic_state = lambda: {
        "current_location": "LOC_MS",
        "current_well": 0,
        "movable_plate_locations": _complete_oem_movable_locations(),
        "semantic_state_revision": 1,
        "ownership_generation": 7,
        "board_epoch_4": 10,
        "board_epoch_5": 11,
        "tip_loaded": False,
        "tip_dirty": False,
        "tip_location": -1,
        "clean_path": True,
        "pseudo_z_home": 65000,
        "plate_on_gantry": None,
    }
    provider._lock = nullcontext()
    provider._load_state = lambda: {"machine_status": {
        "thermal_door_open": True,
        "GripperVersion": 2,
        "BoardTestMode": True,
    }}
    snapshot = provider.mov_execution_machine_state()
    assert snapshot["thermal_door_open"] is True
    assert snapshot["gripper_version"] == 2
    assert snapshot["board_test_mode"] is True


def test_install_binds_internal_wp8_fifo_and_worker_executes_durable_children(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_RUNTIME_STATE_ROOT", str(tmp_path))
    from bioxp.oem_deck_catalog import configured_location_names
    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    provider = Wp8BootstrapFakeDeckProvider(table)
    app = FastAPI()
    monkeypatch.setattr(operator_controls, "_build_catalog", lambda _app: ([], {}))
    monkeypatch.setattr(operator_controls.hardware_state, "project", lambda _name: {"domains": {}, "freshness": {"state": "fresh", "age_s": 0, "fresh_for_s": 30}})
    monkeypatch.setattr(operator_controls.hardware_state, "ownership_projection", lambda: {"ownership_epoch": 7, "ownership": "service"})
    state = {
        "ownership_generation": 7,
        "x_authority": {"active_board_epoch": 11},
        "board4_authority": {"active_board_epoch": 10},
    }
    operator_controls.install_operator_control_plane(
        app,
        maintenance_state_provider=lambda: {"motion_blocked": False, "recovery_required": False, "block_reason": None},
        reference_state_provider=lambda: {"rows": {}},
        lifecycle_state_provider=lambda: {"operation_state": "stopped"},
        serial206_initialization_state_provider=lambda: dict(state),
        oem_deck_provider=lambda: provider,
        oem_deck_position_table_provider=lambda: table,
    )
    assert callable(app.state.oem_wp8_operation_executor)
    original_snapshot = provider.wp8_operation_machine_state
    command_count_before = app.state.operator_command_plane.store.connection.execute(
        "SELECT COUNT(*) FROM operator_plane_commands"
    ).fetchone()[0]
    provider_calls_before = list(provider.calls)
    from bioxp.oem_compat.pathing import LOCATION_NAME_TO_ID
    provider.wp8_operation_machine_state = lambda operation, intent_inputs: {
        "thermal_door_open": False,
        "position_table_by_location": {
            str(LOCATION_NAME_TO_ID[row["location_id"]]): row for row in table.rows()
        },
    }
    from bioxp.oem_deck_movement import translate_oem_plate_move
    absent_target = translate_oem_plate_move("PL_OUTPUT", "LOC_RC", None)
    assert absent_target["destination"] == 32
    with pytest.raises(
        RuntimeError,
        match="machine_target_absent_from_serial206_position_table:32",
    ):
        app.state.oem_wp8_operation_admitter(
            "move_plate",
            inputs=absent_target,
            idempotency_key="wp8-position-table-absent-32",
        )
    assert app.state.operator_command_plane.store.connection.execute(
        "SELECT COUNT(*) FROM operator_plane_commands"
    ).fetchone()[0] == command_count_before
    assert provider.calls == provider_calls_before
    provider.wp8_operation_machine_state = lambda operation, intent_inputs: {"thermal_door_open": False}
    with pytest.raises(RuntimeError, match="thermal_door_must_be_open"):
        app.state.oem_wp8_operation_admitter(
            "move_plate",
            inputs={"plate": 0, "destination": 25, "press_plate": False},
            idempotency_key="wp8-preflight-reject",
        )
    assert app.state.operator_command_plane.store.connection.execute(
        "SELECT COUNT(*) FROM operator_plane_commands"
    ).fetchone()[0] == command_count_before
    provider.wp8_operation_machine_state = original_snapshot
    app.state.operator_command_plane.start()
    with pytest.raises(HTTPException) as forbidden:
        app.state.operator_command_plane.store.admit_internal_wp8_operation(
            "send_z_and_gripper_home",
            inputs={"run_in_parallel": False, "gripper_position": 999},
            state=state,
        )
    assert forbidden.value.status_code == 422
    assert forbidden.value.detail["error"] == "workflow_state_override_forbidden"
    admitted = app.state.operator_command_plane.store.admit_internal_wp8_operation(
        "send_z_and_gripper_home",
        inputs={"run_in_parallel": False},
        state=state,
        idempotency_key="wp8-install-binding",
    )
    deadline = time.monotonic() + 10
    detail = None
    while time.monotonic() < deadline:
        detail = app.state.operator_command_plane.store.get_command(admitted["command_id"])
        if detail is not None and detail["status"] not in {"queued", "dispatched", "issued_pending"}:
            break
        time.sleep(0.01)
    assert detail is not None and detail["status"] == "completed"
    evidence = app.state.operator_command_plane.store.wp8_operation_evidence(admitted["command_id"])
    assert [row["operation"] for row in evidence["children"]] == [
        "getG", "moveZPseudoHome", "sendGripperHome", "ReleaseLockGripperOperation",
    ]
    assert all(row["terminal_state"] == "completed" for row in evidence["children"])
    wp8_calls = [cast(dict[str, Any], row[1]) for row in provider.calls if row[0] == "wp8"]
    assert [row["child"]["operation"] for row in wp8_calls] == [
        "getG", "moveZPseudoHome", "sendGripperHome", "ReleaseLockGripperOperation",
    ]
    assert all(row["command_id"] == admitted["command_id"] for row in wp8_calls)
    assert [row["child_order"] for row in wp8_calls] == [0, 1, 2, 3]
    assert len({row["plan_digest"] for row in wp8_calls}) == 1
    app.state.operator_command_plane.stop()


class FakeDeckPrimitives:
    def __init__(self) -> None:
        self.calls = []
        self.latch_reads = 0

    def _read_axis_position(self, axis):
        return {"x": 100, "y": 200, "z": 65000}[axis]

    def deck_io_query_type(self, io_type):
        assert io_type == 3
        self.latch_reads += 1
        return {"ok": True, "value": 1, "ack": {"status": 100, "value": 1}}

    def read_oem_latch_status(self):
        return {"ok": True, "value": True, "observation_id": "fake-host-latch:0:1"}

    def query_all_pipette_tip_states(self, *, lifecycle_stage_id: str):
        return {"ok": True, "tip_exists": True, "channels_with_tips": [0]}

    def oem_move_to(self, x, y, z, **kwargs):
        self.calls.append((x, y, z, kwargs))
        return {
            "ok": True,
            "controller_command_acknowledged": True,
            "controller_terminal_state_verified": True,
            "hardware_postcondition_verified": True,
        }

    def oem_move_z(self, z, **kwargs):
        self.calls.append(("z", z, kwargs))
        return {
            "ok": True,
            "controller_command_acknowledged": True,
            "controller_terminal_state_verified": True,
            "hardware_postcondition_verified": True,
        }

    def oem_initialize_motion_scriptmove_to_waste(self, **kwargs):
        self.calls.append(("scriptmoveTo", kwargs))
        return {
            "ok": True,
            "controller_command_acknowledged": True,
            "controller_terminal_state_verified": True,
            "hardware_postcondition_verified": True,
        }


class FakeReferenceStore:
    def snapshot(self, axes):
        return {
            "ok": True,
            "rows": {
                axis: {"state": "referenced", "state_version": index + 1}
                for index, axis in enumerate(axes)
            },
        }


class FakeRuntimeStore:
    def __init__(self, state):
        self.state = state

    def read_oem_serial206_initialization_state(self):
        return self.state

    def write_oem_serial206_initialization_state(self, state):
        self.state = state

    def board4_authority_projection(self):
        return {"board": {"state": "active", "active_board_epoch": 10, "state_version": 4}}


def test_fresh_runtime_has_exact_oem_constructor_movable_defaults() -> None:
    machine = Serial206OemInitializationProvider._new_state()["machine_status"]
    assert machine["movable_plate_locations"] == _complete_oem_movable_locations()
    assert machine["current_tray"] is None


@pytest.mark.parametrize(
    "movable",
    [
        {},
        {**_complete_oem_movable_locations(), "UNKNOWN_OBJECT": "LOC_MS"},
        {**_complete_oem_movable_locations(), "TROUGH": "LOC_BSC"},
        {**_complete_oem_movable_locations(), "TROUGH": "UNKNOWN_LOCATION"},
    ],
)
def test_bootstrap_rejects_incomplete_unknown_or_duplicate_movable_maps(tmp_path, movable) -> None:
    store = _operator_store(tmp_path)
    snapshot = {
        "current_location": "LOC_MS", "current_well": 0, "current_tray": None,
        "tip_loaded": False, "tip_dirty": False, "tip_location": -1, "clean_path": True,
        "plate_on_gantry": None, "movable_plate_locations": movable, "pseudo_z_home": 65000,
        "ownership_generation": 7, "board_epoch_4": 10, "board_epoch_5": 11,
        "latch_status": True, "machine_latch_closed": True, "latch_observation_id": "bootstrap-latch-1",
        "source_operation": "migrated_successful_semantic_state", "source_command_id": "legacy-state-1",
    }
    with pytest.raises(ValueError, match="movable plate state is not authoritative"):
        store.bootstrap_deck_semantic_state(snapshot)
    assert store.deck_semantic_state()["semantic_state_revision"] == 0
    store.stop()


def test_plate_name_domain_matches_exact_oem_ordinals_and_canonicalizes_cover_branch() -> None:
    from bioxp.oem_deck_movement import OEM_PLATE_NAME_ORDINALS, canonical_plate_name

    assert OEM_PLATE_NAME_ORDINALS == {
        "POOL_PLATE": 0, "OUTPUT_PLATE": 1, "REAGENT_PLATE": 2,
        "BIO_SECURITY_COVER": 3, "OUTPUT_COVER": 4, "REAGENT_COVER": 5,
        "TIP_TRAY": 6, "STRIP_ONE": 7, "STRIP_TWO": 8, "STRIP_THREE": 9,
        "STRIP_FOUR": 10, "TROUGH": 11, "SYNTHESIS_PLATE": 12,
        "OLIGO_QUANTITATION_PLATE": 13, "GENE_QUANTITATION_PLATE": 14,
        "REF_QUANTITATION_PLATE": 15, "ELUTION_PLATE": 16,
        "ACCUMULATION_PLATE": 17, "TIP_HOTEL": 18, "TFF_REAGENT_BLOCK": 19,
        "VOLUME_CALCULATION": 20, "WASTE_BIN": 21,
    }
    assert canonical_plate_name("OUTPUT_COVER") == 4
    assert canonical_plate_name("REAGENT_COVER") == 5
    assert canonical_plate_name(4) == 4
    assert canonical_plate_name(None) is None
    with pytest.raises(ValueError, match="outside the source domain"):
        canonical_plate_name("4")


@pytest.mark.parametrize("plate", ["UNKNOWN_PLATE", "4", -1, 22, True, False])
def test_bootstrap_rejects_plate_on_gantry_outside_exact_oem_domain(tmp_path, plate) -> None:
    store = _operator_store(tmp_path)
    snapshot = {
        "current_location": "LOC_MS", "current_well": 0, "current_tray": None,
        "tip_loaded": False, "tip_dirty": False, "tip_location": -1, "clean_path": True,
        "plate_on_gantry": plate, "movable_plate_locations": _complete_oem_movable_locations(),
        "pseudo_z_home": 65000, "ownership_generation": 7, "board_epoch_4": 10, "board_epoch_5": 11,
        "latch_status": True, "machine_latch_closed": True, "latch_observation_id": "bootstrap-latch-1",
        "source_operation": "migrated_successful_semantic_state", "source_command_id": "legacy-state-1",
    }
    with pytest.raises(ValueError, match="plate state is not authoritative"):
        store.bootstrap_deck_semantic_state(snapshot)
    assert store.deck_semantic_state()["semantic_state_revision"] == 0
    store.stop()


def test_bootstrap_and_plate_producer_publish_only_canonical_plate_ordinals(tmp_path) -> None:
    store = _operator_store(tmp_path)
    snapshot = {
        "current_location": "LOC_MS", "current_well": 0, "current_tray": None,
        "tip_loaded": False, "tip_dirty": False, "tip_location": -1, "clean_path": True,
        "plate_on_gantry": "OUTPUT_COVER", "movable_plate_locations": _complete_oem_movable_locations(),
        "pseudo_z_home": 65000, "ownership_generation": 7, "board_epoch_4": 10, "board_epoch_5": 11,
        "latch_status": True, "machine_latch_closed": True, "latch_observation_id": "bootstrap-latch-1",
        "source_operation": "migrated_successful_semantic_state", "source_command_id": "legacy-state-1",
    }
    assert store.bootstrap_deck_semantic_state(snapshot)["plate_on_gantry"] == 4
    store.bind_deck_owner_authority_reader(
        lambda: {"ownership_generation": 7, "board_epoch_4": 10, "board_epoch_5": 11}
    )
    updated = store.publish_deck_owner_state(
        source_operation="plate_operation", source_command_id="plate-op-1",
        updates={
            "current_tray": "REAGENT_COVER", "plate_on_gantry": "REAGENT_COVER",
            "movable_plate_locations": _complete_oem_movable_locations(),
        },
        ownership_generation=7, board_epoch_4=10, board_epoch_5=11,
    )
    assert updated["plate_on_gantry"] == 5
    before = updated["semantic_state_revision"]
    with pytest.raises(ValueError, match="plate_on_gantry is outside the source domain"):
        store.publish_deck_owner_state(
            source_operation="plate_operation", source_command_id="plate-op-invalid",
            updates={
                "current_tray": None, "plate_on_gantry": "NOT_A_PLATE",
                "movable_plate_locations": _complete_oem_movable_locations(),
            },
            ownership_generation=7, board_epoch_4=10, board_epoch_5=11,
        )
    assert store.deck_semantic_state()["semantic_state_revision"] == before
    store.stop()


def test_sqlite_plate_cover_round_trip_feeds_exact_clearance_ordinals_to_movement(tmp_path, monkeypatch) -> None:
    from bioxp import oem_serial206_initialization
    from bioxp.oem_deck_catalog import configured_location_names
    from bioxp.oem_deck_movement import DeckAuthoritySnapshot

    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    store = _operator_store(tmp_path)
    snapshot = {
        "current_location": "LOC_MS", "current_well": 0, "current_tray": None,
        "tip_loaded": False, "tip_dirty": False, "tip_location": -1, "clean_path": True,
        "plate_on_gantry": "OUTPUT_COVER", "movable_plate_locations": _complete_oem_movable_locations(),
        "pseudo_z_home": 65000, "ownership_generation": 7, "board_epoch_4": 10, "board_epoch_5": 11,
        "latch_status": True, "machine_latch_closed": True, "latch_observation_id": "bootstrap-latch-1",
        "source_operation": "migrated_successful_semantic_state", "source_command_id": "legacy-state-1",
    }
    semantic = store.bootstrap_deck_semantic_state(snapshot)
    store.bind_deck_owner_authority_reader(
        lambda: {"ownership_generation": 7, "board_epoch_4": 10, "board_epoch_5": 11}
    )
    primitives = FakeDeckPrimitives()
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 7)
    monkeypatch.setattr(oem_serial206_initialization, "load_bound_oem_position_table", lambda: table)

    def movement_snapshot(plate: int, revision: int) -> DeckAuthoritySnapshot:
        return DeckAuthoritySnapshot(
            ownership_generation=7, provider_owner_id="serial206-test", board_epoch_4=10,
            board_epoch_5=11, position_table_sha256=table.digest, machine_state_revision=revision,
            reference_versions={"x": 1, "y": 1, "z": 1, "g": 1},
            safety_epochs={"global": 0, "x": 0, "y": 0, "z": 0},
            latch_observation_id="latch-1", controller_position_observation_id="position-1",
            captured_at=1.0, current_x=0, current_y=0, current_z=65000,
            current_location_id="LOC_MS", current_well_id=0, tip_loaded=False,
            tip_dirty=False, tip_location=-1, clean_path=True, plate_on_gantry=plate,
            pseudo_z_home=65000, device_type="BIOXP", latch_status=True,
            machine_latch_closed=True, semantic_state_provenance_digest="a" * 64,
        )

    assert semantic["plate_on_gantry"] == 4
    authority = movement_snapshot(semantic["plate_on_gantry"], semantic["semantic_state_revision"])
    provider.moveTo(location_id=1, authority_snapshot=authority.__dict__)
    assert primitives.calls[-1][3]["plate_on_gantry"] == 4

    semantic = store.publish_deck_owner_state(
        source_operation="plate_operation", source_command_id="plate-op-cover-5",
        updates={
            "current_tray": "REAGENT_COVER", "plate_on_gantry": "REAGENT_COVER",
            "movable_plate_locations": _complete_oem_movable_locations(),
        },
        ownership_generation=7, board_epoch_4=10, board_epoch_5=11,
    )
    assert semantic["plate_on_gantry"] == 5
    authority = movement_snapshot(semantic["plate_on_gantry"], semantic["semantic_state_revision"])
    provider.moveTo(location_id=1, authority_snapshot=authority.__dict__)
    assert primitives.calls[-1][3]["plate_on_gantry"] == 5
    store.stop()


def test_serial206_provider_exposes_complete_deck_snapshot_and_reachable_ordinary_move(monkeypatch):
    from bioxp import oem_serial206_initialization
    from bioxp.oem_deck_catalog import configured_location_names

    table = PositionTable.from_rows([
        {"name": name, "x": index * 100, "y": index * 100 + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ], source_sha256="3" * 64)
    primitives = FakeDeckPrimitives()
    state = Serial206OemInitializationProvider._new_state()
    state["x_lifecycle"].update({
        "state": "referenced_ready", "reference_state": "referenced",
        "generation": 7, "board_lifecycle_generation": 11,
    })
    state["z_lifecycle"].update({
        "state": "referenced_ready", "reference_state": "referenced",
        "generation": 7, "board_lifecycle_generation": 11,
    })
    state["machine_status"].update({
        "current_location": "LOC_MS", "current_well": 0,
        "tip_loaded": False, "tip_dirty": False, "tip_location": -1,
        "clean_path": True, "plate_on_gantry": None, "psudo_z_home_steps": 65000,
        "latch_status": True, "latch_closed": True, "latch_observation_id": "latch-1",
        "semantic_state_revision": 3,
    })
    runtime_store = FakeRuntimeStore(state)
    provider = Serial206OemInitializationProvider(
        primitives,
        state_store=runtime_store,
        reference_store=FakeReferenceStore(),
        generation_provider=lambda: 7,
    )
    provider.bind_deck_semantic_state_reader(lambda: {
        "current_location": "LOC_MS", "current_well": 0, "semantic_state_revision": 3,
        "ambiguity_state": "none", "producer_operation": "updateLocation",
        "producer_command_id": "canonical-cmd-1",
        "tip_loaded": False, "tip_dirty": False, "tip_location": -1, "clean_path": True,
        "plate_on_gantry": None, "pseudo_z_home": 65000,
        "ownership_generation": 7, "board_epoch_4": 10, "board_epoch_5": 11,
        "latch_status": True, "machine_latch_closed": True, "latch_observation_id": "latch-1",
        "transition_provenance": {"source_operation": "updateLocation", "command_id": "canonical-cmd-1"},
    })
    monkeypatch.setattr(oem_serial206_initialization, "load_bound_oem_position_table", lambda: table)

    with pytest.raises(RuntimeError, match="tray_0_tip_availability_unavailable"):
        provider.deck_authority_snapshot(expected_generation=7)
    assert primitives.latch_reads == 0
    assert primitives.calls == []
    provider.bind_tip_tray_state_reader(lambda tray_id: {
        "tray_id": tray_id, "tip_available": False,
        "operation_id": "tray-op-1", "command_id": "tray-cmd-1",
        "ownership_generation": 7, "board_epoch_4": 10, "board_epoch_5": 11,
    })

    with provider.movement_lease():
        assert provider.force_to_high_home(command_id="cmd-1")["ok"] is True
        snapshot = provider.deck_authority_snapshot(expected_generation=7)
        result = provider.moveTo(location_id=1, camera_offset=False, authority_snapshot=snapshot)

    assert snapshot["board_epoch_4"] == 10 and snapshot["board_epoch_5"] == 11
    assert snapshot["position_table_sha256"] == table.digest
    assert set(snapshot["reference_versions"]) == {"x", "y", "z", "g"}
    assert set(snapshot["safety_epochs"]) == {"global", "x", "y", "z"}
    assert primitives.latch_reads == 1
    assert snapshot["latch_status"] is True
    assert snapshot["machine_latch_closed"] is True
    assert snapshot["latch_observation_id"] != "latch-1"
    assert snapshot["latch_observation_id"].startswith("deck-latch:")
    assert result["ok"] is True and result["controller_completion_verified"] is True
    assert len(primitives.calls) == 1


class _ProductionMoveReceiptTester:
    def motor_set_axis_param(self, board, parameter, value, motor=0):
        return {"ok": True, "board": board, "parameter": parameter, "value": value, "motor": motor}


class _ProductionMoveReceiptAdapter(Serial206ProductionPrimitiveAdapter):
    def __init__(self, *, child_ack=True, child_terminal=True):
        self.tester = _ProductionMoveReceiptTester()
        self.y_provider = None
        self.child_ack = child_ack
        self.child_terminal = child_terminal
        self.motion_calls = []

    def _read_axis_position(self, axis):
        return {"x": 0, "y": 0, "z": 500}[axis]

    def _axis_profile(self, axis):
        return {"board": {"x": 5, "y": 4, "z": 4}[axis], "motor": {"x": 0, "y": 0, "z": 1}[axis]}

    def oem_move_xy(self, x, y, *, wait_timeout_s=5.0):
        self.motion_calls.append(("moveTo", x, y))
        return {
            "ok": True,
            "targets": {"x": x, "y": y},
            "controller_command_acknowledged": self.child_ack,
            "controller_terminal_state_verified": self.child_terminal,
            "target_position_verified": self.child_terminal,
        }

    def oem_move_z(self, position, *, pseudo_home_steps, motor_current=31, wait_for_stop=True):
        self.motion_calls.append(("moveZ", position, motor_current, wait_for_stop))
        return {
            "ok": True,
            "axis": "z",
            "requested": position,
            "controller_command_acknowledged": self.child_ack,
            "controller_terminal_state_verified": self.child_terminal,
            "target_position_verified": self.child_terminal,
        }

    def oem_initialize_motion_scriptmove_to_waste(self, **kwargs):
        self.motion_calls.append(("scriptmoveTo", kwargs))
        return {
            "ok": True,
            "controller_command_acknowledged": self.child_ack,
            "controller_terminal_state_verified": self.child_terminal,
            "target_position_verified": self.child_terminal,
        }


def _production_move_provider(monkeypatch, *, child_ack=True, child_terminal=True):
    from bioxp import oem_serial206_initialization
    from bioxp.oem_deck_catalog import configured_location_names

    location_names = [*configured_location_names(), "CAMERA_OFFSET"]
    table = PositionTable.from_rows([
        {"name": name, "x": index + 100, "y": index + 200, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(location_names)
    ])
    monkeypatch.setattr(oem_serial206_initialization, "load_bound_oem_position_table", lambda: table)
    adapter = _ProductionMoveReceiptAdapter(child_ack=child_ack, child_terminal=child_terminal)
    provider = Serial206OemInitializationProvider(adapter, generation_provider=lambda: 7)
    authority = {
        "tip_loaded": False, "tip_dirty": False, "tip_location": -1, "clean_path": True,
        "pseudo_z_home": 500, "ownership_generation": 7, "board_epoch_4": 10,
        "board_epoch_5": 11, "current_location_id": "LOC_MS", "current_well_id": 0,
        "machine_state_revision": 1, "semantic_state_provenance_digest": "a" * 64,
        "plate_on_gantry": None,
    }
    return provider, authority


@pytest.mark.parametrize("operation", ["ordinary", "barcode", "park"])
def test_real_production_move_to_shape_proves_named_move_children(monkeypatch, operation) -> None:
    provider, authority = _production_move_provider(monkeypatch)

    if operation == "ordinary":
        results = [provider.moveTo(location_id=1, authority_snapshot=authority)]
    elif operation == "barcode":
        results = [
            provider.moveTo(location_id=2, camera_offset=True, authority_snapshot=authority),
            provider.moveZCamera(location_id=2, authority_snapshot=authority),
        ]
    else:
        results = [provider.parkGantry(authority_snapshot=authority)]

    assert all(row["ok"] is True for row in results)
    assert all(row["controller_command_acknowledged"] is True for row in results)
    assert all(row["controller_completion_verified"] is True for row in results)
    assert all(
        (
            (row.get("primitive_result") or row["source_children"][-1]["result"]).get("controller_completion_verified") is True
            or (row.get("primitive_result") or row["source_children"][-1]["result"]).get("controller_terminal_state_verified") is True
        )
        for row in results
    )


@pytest.mark.parametrize(
    ("location_id", "location_name", "base_x", "base_y", "expected_x", "expected_y", "expected_z"),
    [
        (2, "LOC_TC", 100000, 110000, 90213, 102906, 1794),
        (3, "LOC_RC", 100, 200, -20331, 38, 3145),
    ],
)
def test_barcode_uses_raw_il_offsets_high_clamp_and_source_child_order(
    monkeypatch, location_id, location_name, base_x, base_y, expected_x, expected_y, expected_z
) -> None:
    from bioxp import oem_serial206_initialization
    from bioxp.oem_deck_catalog import configured_location_names

    rows = []
    for index, name in enumerate([*configured_location_names(), "CAMERA_OFFSET"]):
        row = {"name": name, "x": index + 100, "y": index + 200, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        if name == location_name:
            row.update({"x": base_x, "y": base_y})
        if name == "CAMERA_OFFSET":
            row.update({"x": 3499, "y": -7744, "zLow": 3145, "zDelta": 6842})
        rows.append(row)
    table = PositionTable.from_rows(rows)
    monkeypatch.setattr(oem_serial206_initialization, "load_bound_oem_position_table", lambda: table)
    adapter = _ProductionMoveReceiptAdapter()
    provider = Serial206OemInitializationProvider(adapter, generation_provider=lambda: 7)
    authority = {
        "tip_loaded": False, "tip_dirty": False, "tip_location": -1, "clean_path": True,
        "pseudo_z_home": 500, "ownership_generation": 7, "board_epoch_4": 10,
        "board_epoch_5": 11, "current_location_id": "LOC_MS", "current_well_id": 0,
        "machine_state_revision": 1, "semantic_state_provenance_digest": "a" * 64,
        "plate_on_gantry": None,
    }

    provider.moveTo(location_id=location_id, camera_offset=True, authority_snapshot=authority)
    provider.moveZCamera(location_id=location_id, authority_snapshot=authority)

    assert adapter.motion_calls[0] == ("moveTo", expected_x, expected_y)
    assert adapter.motion_calls[-1] == ("moveZ", expected_z, 31, True)


class _ParkPrimitives:
    def __init__(self, *, tips_remain: bool = False, incomplete_axis: str | None = None) -> None:
        self.calls = []
        self.tips_remain = tips_remain
        self.incomplete_axis = incomplete_axis

    @staticmethod
    def _completed(command_id):
        return {
            "ok": True, "command_id": command_id,
            "controller_command_acknowledged": True,
            "controller_terminal_state_verified": True,
            "hardware_postcondition_verified": True,
        }

    def oem_move_to(self, x, y, z, **kwargs):
        self.calls.append(("moveTo", x, y, z, kwargs))
        return self._completed("park-waste-move")

    def eject_all_tips_for_oem_park(self):
        self.calls.append(("ejectAllTips", True, True))
        return self._completed("park-eject")

    def query_all_pipette_tip_states(self, *, lifecycle_stage_id: str):
        self.calls.append(("queryTipStatus", lifecycle_stage_id))
        return {"ok": True, "tip_exists": self.tips_remain, "channels_with_tips": [0] if self.tips_remain else []}

    def oem_initialize_motion_move_absolute(self, axis, position, **kwargs):
        self.calls.append((f"move{axis.upper()}", position, kwargs))
        result = self._completed(f"park-{axis}")
        if axis == self.incomplete_axis:
            result["controller_terminal_state_verified"] = False
        return result

    def oem_initialize_motion_scriptmove_to_waste(self, **kwargs):
        self.calls.append(("scriptmoveTo", kwargs))
        return self._completed("park-final")


def _park_authority(*, tip_loaded: bool, current_location_id: str = "LOC_MS") -> dict[str, object]:
    return {
        "tip_loaded": tip_loaded, "tip_dirty": False,
        "tip_location": 0 if tip_loaded else -1, "clean_path": True,
        "pseudo_z_home": 500, "ownership_generation": 7, "board_epoch_4": 10,
        "board_epoch_5": 11, "current_location_id": current_location_id,
        "current_well_id": 4, "machine_state_revision": 1,
        "semantic_state_provenance_digest": "a" * 64, "plate_on_gantry": None,
    }


def _install_park_table(monkeypatch) -> None:
    from bioxp import oem_serial206_initialization
    from bioxp.oem_deck_catalog import configured_location_names

    table = PositionTable.from_rows([
        {"name": name, "x": index + 100, "y": index + 200, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    monkeypatch.setattr(oem_serial206_initialization, "load_bound_oem_position_table", lambda: table)


def test_park_gantry_false_preserves_loaded_tip_child_order_and_discarded_returns(monkeypatch) -> None:
    _install_park_table(monkeypatch)
    primitives = _ParkPrimitives()
    provider = Serial206OemInitializationProvider(
        primitives, generation_provider=lambda: 7,
        sleep=lambda seconds: primitives.calls.append(("sleep", seconds)),
    )
    published = []
    provider.bind_deck_semantic_state_publisher(lambda **kwargs: published.append(kwargs) or kwargs["updates"])

    result = provider.parkGantry(authority_snapshot=_park_authority(tip_loaded=True))

    assert result["ok"] is True
    assert [row[0] for row in primitives.calls] == [
        "moveTo", "ejectAllTips", "queryTipStatus", "sleep", "moveZ", "moveX", "scriptmoveTo",
    ]
    assert primitives.calls[0][-1]["run_in_parallel"] is False
    assert primitives.calls[-1][1].update({}) is None
    assert primitives.calls[-1][1]["current_location"] == 0
    assert primitives.calls[-1][1]["current_well"] == 4
    assert primitives.calls[-1][1]["target_location"] == 28
    assert primitives.calls[-1][1]["position_flag"] == 2
    assert published[-1]["updates"] == {"tip_loaded": False, "tip_dirty": False, "tip_location": -1}
    assert [row["discarded_return"] for row in result["source_children"]] == [
        True, False, True, False, False, False, True,
    ]


def test_park_gantry_already_parked_is_authoritative_no_io_noop() -> None:
    primitives = _ParkPrimitives()
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 7)

    result = provider.parkGantry(
        authority_snapshot=_park_authority(tip_loaded=False, current_location_id="LOC_PARK")
    )

    assert result["ok"] is True and result["source_noop"] is True
    assert result["controller_completion_verified"] is False
    assert primitives.calls == []


def test_park_gantry_does_not_publish_tip_state_without_terminal_z_x_proof(monkeypatch) -> None:
    _install_park_table(monkeypatch)
    primitives = _ParkPrimitives(incomplete_axis="z")
    provider = Serial206OemInitializationProvider(primitives, generation_provider=lambda: 7, sleep=lambda _s: None)
    published = []
    provider.bind_deck_semantic_state_publisher(lambda **kwargs: published.append(kwargs))

    result = provider.parkGantry(authority_snapshot=_park_authority(tip_loaded=True))

    assert result["ok"] is False
    assert result["governance_outcome"] == "park_tip_cleanup_terminal_proof_missing"
    assert published == []
    assert all(call[0] != "scriptmoveTo" for call in primitives.calls)


def test_scriptmove_production_evidence_aggregates_required_nested_children() -> None:
    execution = {
        "ok": True,
        "execution_results": [
            {"op": "moveZ", "results": [{"result": _ParkPrimitives._completed("z")}]},
            {"op": "moveX", "results": [{"result": _ParkPrimitives._completed("x")}]},
        ],
    }
    evidence = oem_serial206_initialization._aggregate_executed_controller_evidence(execution)
    assert evidence == {
        "controller_command_acknowledged": True,
        "controller_completion_verified": True,
        "hardware_postcondition_verified": True,
        "required_executed_child_count": 2,
    }
    execution["execution_results"][1]["results"][0]["result"]["controller_terminal_state_verified"] = False
    assert oem_serial206_initialization._aggregate_executed_controller_evidence(execution)["controller_completion_verified"] is False


def test_park_gantry_manual_tip_removal_is_governance_failure_without_final_move(monkeypatch) -> None:
    _install_park_table(monkeypatch)
    primitives = _ParkPrimitives(tips_remain=True)
    provider = Serial206OemInitializationProvider(
        primitives, generation_provider=lambda: 7,
        sleep=lambda seconds: primitives.calls.append(("sleep", seconds)),
    )

    result = provider.parkGantry(authority_snapshot=_park_authority(tip_loaded=True))

    assert result["ok"] is False
    assert result["governance_outcome"] == "manual_tip_removal_required"
    assert [row[0] for row in primitives.calls] == ["moveTo", "ejectAllTips", "queryTipStatus", "sleep"]
    assert all(row[0] != "scriptmoveTo" for row in primitives.calls)


def test_fresh_store_tip_tray_authority_is_nullable_until_source_producer(tmp_path) -> None:
    store = _operator_store(tmp_path)

    state = store.tip_tray_state(0)

    assert state == {
        "schema_version": "bioxp.operator_tip_tray_state.v1",
        "tray_id": 0,
        "occupancy": None,
        "tip_available": None,
        "available_count": None,
        "revision": 0,
        "operation_id": None,
        "command_id": None,
        "ownership_generation": None,
        "board_epoch_4": None,
        "board_epoch_5": None,
        "timestamp": None,
        "provenance": None,
        "provenance_sha256": None,
    }
    store.stop()


def test_tip_tray_source_transitions_preserve_occupancy_and_literal_latch_semantics(tmp_path) -> None:
    store = _operator_store(tmp_path)
    common = {
        "tray_id": 0, "operation_id": "tray-op-1", "command_id": "tray-cmd-1",
        "ownership_generation": 7, "board_epoch_4": 10, "board_epoch_5": 11,
        "provenance": {"source": "ClassTipTray", "source_anchor": "ClassTipTray.cs:119,371-482"},
    }

    full = store.publish_tip_tray_transition(transition="construct", **common)
    removed = store.publish_tip_tray_transition(
        transition="remove_group", group_index=0, **{**common, "operation_id": "tray-op-2", "command_id": "tray-cmd-2"}
    )
    empty = store.publish_tip_tray_transition(
        transition="remove_all", **{**common, "operation_id": "tray-op-3", "command_id": "tray-cmd-3"}
    )
    retipped = store.publish_tip_tray_transition(
        transition="retip", well_ids=[0, 1, 2, 3],
        **{**common, "operation_id": "tray-op-4", "command_id": "tray-cmd-4"},
    )

    assert len(full["occupancy"]) == 96 and all(full["occupancy"])
    assert full["tip_available"] is True and full["available_count"] == 24
    assert sum(removed["occupancy"]) == 92 and removed["available_count"] == 23
    assert removed["tip_available"] is True
    assert empty["tip_available"] is False and empty["available_count"] == 0
    assert sum(retipped["occupancy"]) == 4
    assert retipped["tip_available"] is False
    assert retipped["available_count"] is None
    assert retipped["revision"] == 4
    store.stop()


def test_provider_tip_tray_publisher_uses_current_owner_authority() -> None:
    state = Serial206OemInitializationProvider._new_state()
    state["x_lifecycle"]["board_lifecycle_generation"] = 11
    provider = Serial206OemInitializationProvider(
        FakeDeckPrimitives(), state_store=FakeRuntimeStore(state), generation_provider=lambda: 7,
    )
    published = []
    provider.bind_tip_tray_state_publisher(
        lambda **kwargs: published.append(kwargs) or {"tip_available": True}
    )

    result = provider.publish_tip_tray_transition(
        tray_id=0, transition="reset", operation_id="tray-op-provider",
        command_id="tray-cmd-provider", provenance={"source": "ClassTipTray.resetStatus"},
    )

    assert result["tip_available"] is True
    assert published == [{
        "tray_id": 0, "transition": "reset", "operation_id": "tray-op-provider",
        "command_id": "tray-cmd-provider", "provenance": {"source": "ClassTipTray.resetStatus"},
        "well_ids": None, "group_index": None, "ownership_generation": 7,
        "board_epoch_4": 10, "board_epoch_5": 11,
    }]


def test_tip_tray_sql_projection_and_history_are_guarded(tmp_path) -> None:
    import sqlite3

    store = _operator_store(tmp_path)
    store.publish_tip_tray_transition(
        tray_id=0, transition="reset", operation_id="tray-op-guard",
        command_id="tray-cmd-guard", ownership_generation=7,
        board_epoch_4=10, board_epoch_5=11,
        provenance={"source": "ClassTipTray.resetStatus"},
    )

    with pytest.raises(sqlite3.IntegrityError):
        store.connection.execute(
            "UPDATE operator_plane_tip_tray_state SET tip_available=0 WHERE tray_id=0"
        )
    with pytest.raises(sqlite3.IntegrityError):
        store.connection.execute(
            "UPDATE operator_plane_tip_tray_transitions SET provenance_json='{}' WHERE tray_id=0"
        )
    with pytest.raises(sqlite3.IntegrityError):
        store.connection.execute(
            "DELETE FROM operator_plane_tip_tray_transitions WHERE tray_id=0"
        )
    store.stop()


@pytest.mark.parametrize(
    ("child_ack", "child_terminal"),
    [(False, True), (True, False)],
)
def test_real_production_move_to_never_infers_completion_from_child_ok(
    monkeypatch, child_ack, child_terminal
) -> None:
    provider, authority = _production_move_provider(
        monkeypatch, child_ack=child_ack, child_terminal=child_terminal
    )

    result = provider.moveTo(location_id=1, authority_snapshot=authority)

    assert result["ok"] is True
    assert result["controller_completion_verified"] is False
    assert result["primitive_result"]["controller_completion_verified"] is False


def test_initialize_motion_pipette_query_never_establishes_tip_tray_authority() -> None:
    state = Serial206OemInitializationProvider._new_state()
    state["x_lifecycle"]["board_lifecycle_generation"] = 11
    runtime = FakeRuntimeStore(state)
    provider = Serial206OemInitializationProvider(
        FakeDeckPrimitives(), state_store=runtime, generation_provider=lambda: 7,
    )
    provider.bind_deck_semantic_state_reader(lambda: {
        "tip_location": 0, "tip_dirty": False,
    })
    provider.bind_deck_semantic_state_publisher(
        lambda *, source_operation, source_command_id, updates, **authority: {
            **dict(updates), **dict(authority), "source_operation": source_operation,
            "source_command_id": source_command_id,
        }
    )
    query_spec = next(
        spec for spec in SERIAL206_INITIALIZE_MOTION_STAGE_SPECS
        if spec.key == "initializeMotion.queryTipStatus.initial"
    )

    result = provider._execute_initialize_motion_stage(state, query_spec, timeout_s=2.0)

    assert result["ok"] is True
    assert "tip_tray_source_command_id" not in result
    assert "tip_tray_zero" not in result
    assert "source_tip_trays" not in runtime.state["machine_status"]
    assert "tip_tray_availability" not in runtime.state["machine_status"]
    assert not hasattr(provider, "_publish_tip_tray_zero_from_source_owner")


def test_clean_path_is_derived_from_current_tray_zero_availability_and_expectation_mismatch_is_atomic(tmp_path) -> None:
    store = _operator_store(tmp_path)
    store.bootstrap_deck_semantic_state({
        "current_location": "LOC_MS", "current_well": 0, "current_tray": None,
        "tip_loaded": False, "tip_dirty": False, "tip_location": -1, "clean_path": False,
        "plate_on_gantry": None, "movable_plate_locations": {
            "POOL_PLATE": "LOC_P_TC", "OUTPUT_PLATE": "LOC_P_OC", "REAGENT_PLATE": "LOC_RC",
            "STRIP_ONE": "LOC_STRIP1", "STRIP_TWO": "LOC_STRIP2", "STRIP_THREE": "LOC_STRIP3",
            "STRIP_FOUR": "LOC_STRIP4", "REAGENT_COVER": "LOC_RC_COVER_STORAGE",
            "OUTPUT_COVER": "LOC_OC_COVER_STORAGE", "BIO_SECURITY_COVER": "LOC_BSC", "TROUGH": "LOC_TROUGH",
        }, "pseudo_z_home": 65000,
        "ownership_generation": 7, "board_epoch_4": 10, "board_epoch_5": 11,
        "latch_status": True, "machine_latch_closed": True, "latch_observation_id": "real-latch-1",
        "source_operation": "migrated_successful_semantic_state", "source_command_id": "legacy-state-1",
    })
    store.publish_tip_tray_transition(
        tray_id=0, transition="construct", operation_id="tray-op-construct",
        command_id="tray-cmd-construct", ownership_generation=7,
        board_epoch_4=10, board_epoch_5=11,
        provenance={"source": "ClassTipTray..ctor"},
    )
    store.publish_tip_tray_transition(
        tray_id=0, transition="remove_all", operation_id="tray-op-empty",
        command_id="tray-cmd-empty", ownership_generation=7,
        board_epoch_4=10, board_epoch_5=11,
        provenance={"source": "ClassTipTray.RemoveAllTips"},
    )
    state = Serial206OemInitializationProvider._new_state()
    state["x_lifecycle"]["board_lifecycle_generation"] = 11
    runtime = FakeRuntimeStore(state)
    provider = Serial206OemInitializationProvider(
        FakeDeckPrimitives(), state_store=runtime, generation_provider=lambda: 7,
    )
    provider.bind_deck_semantic_state_reader(store.deck_semantic_state)
    provider.bind_deck_semantic_state_publisher(store.publish_deck_owner_state)
    provider.bind_tip_tray_state_reader(store.tip_tray_state)
    before_local = json.dumps(runtime.state, sort_keys=True)
    before_canonical = store.deck_semantic_state()

    with pytest.raises(ValueError, match="clean_path expectation does not match OEM tray-0 authority"):
        provider.publish_clean_path_state(expected_clean_path=False, source_command_id="clean-path-mismatch")

    assert json.dumps(runtime.state, sort_keys=True) == before_local
    assert store.deck_semantic_state() == before_canonical
    published = provider.publish_clean_path_state(
        expected_clean_path=True, source_command_id="clean-path-match"
    )
    assert published["clean_path"] is True
    assert published["semantic_state_revision"] == before_canonical["semantic_state_revision"] + 1
    store.stop()


def test_clean_path_rejects_missing_or_stale_tray_zero_authority_without_publication(tmp_path) -> None:
    store = _operator_store(tmp_path)
    state = Serial206OemInitializationProvider._new_state()
    state["x_lifecycle"]["board_lifecycle_generation"] = 11
    runtime = FakeRuntimeStore(state)
    provider = Serial206OemInitializationProvider(
        FakeDeckPrimitives(), state_store=runtime, generation_provider=lambda: 7,
    )
    published = []
    provider.bind_deck_semantic_state_publisher(lambda **kwargs: published.append(kwargs))

    with pytest.raises(RuntimeError, match="tray_0_tip_availability_unavailable"):
        provider.publish_clean_path_state(expected_clean_path=False, source_command_id="missing")

    provider.bind_tip_tray_state_reader(lambda _tray_id: {
        "tray_id": 0, "tip_available": True,
        "operation_id": "stale-tray-op", "command_id": "stale-tip-tray-state",
        "ownership_generation": 6, "board_epoch_4": 10, "board_epoch_5": 11,
    })
    with pytest.raises(RuntimeError, match="tray_0_tip_availability_unavailable"):
        provider.publish_clean_path_state(expected_clean_path=False, source_command_id="stale")
    assert published == []
    store.stop()


def test_production_state_owners_publish_canonical_sqlite_mutations(tmp_path) -> None:
    store = _operator_store(tmp_path)
    store.bootstrap_deck_semantic_state({
        "current_location": "LOC_MS", "current_well": 0, "current_tray": None,
        "tip_loaded": False, "tip_dirty": False, "tip_location": 2, "clean_path": True,
        "plate_on_gantry": None, "movable_plate_locations": _complete_oem_movable_locations(), "pseudo_z_home": 65000,
        "ownership_generation": 7, "board_epoch_4": 10, "board_epoch_5": 11,
        "latch_status": True, "machine_latch_closed": True, "latch_observation_id": "real-latch-1",
        "source_operation": "migrated_successful_semantic_state", "source_command_id": "legacy-state-1",
    })
    state = Serial206OemInitializationProvider._new_state()
    state["x_lifecycle"]["board_lifecycle_generation"] = 11
    state["machine_status"].update({
        "tip_loaded": False, "plate_on_gantry": None,
    })
    store.publish_tip_tray_transition(
        tray_id=0, transition="construct", operation_id="tray-op-production",
        command_id="tray-cmd-production", ownership_generation=7,
        board_epoch_4=10, board_epoch_5=11,
        provenance={"source": "ClassTipTray..ctor"},
    )
    provider = Serial206OemInitializationProvider(
        FakeDeckPrimitives(), state_store=FakeRuntimeStore(state), generation_provider=lambda: 7,
    )
    provider.bind_deck_semantic_state_reader(store.deck_semantic_state)
    provider.bind_deck_semantic_state_publisher(store.publish_deck_owner_state)
    provider.bind_tip_tray_state_reader(store.tip_tray_state)

    provider.query_and_publish_pipette_state(source_command_id="pipette-query-1")
    provider.publish_clean_path_state(expected_clean_path=False, source_command_id="clean-path-1")
    provider.load_gantry(tip_loaded=True, plate_on_gantry="OUTPUT_PLATE")
    provider.publish_plate_operation_state(
        current_tray="OUTPUT", plate_on_gantry="OUTPUT_PLATE",
        movable_plate_locations=_complete_oem_movable_locations(), source_command_id="plate-op-1",
    )

    semantic = store.deck_semantic_state()
    assert semantic["tip_loaded"] is True and semantic["tip_location"] == 2
    assert semantic["clean_path"] is False
    assert semantic["pseudo_z_home"] == 500
    assert semantic["plate_on_gantry"] == 1
    assert semantic["current_tray"] == "OUTPUT"
    assert semantic["movable_plate_locations"] == _complete_oem_movable_locations()
    assert semantic["producer_operation"] == "plate_operation"
    store.stop()


def test_fresh_store_bootstraps_canonical_semantic_state_from_bound_provider(tmp_path, monkeypatch):
    from bioxp.oem_deck_catalog import configured_location_names

    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    provider = BootstrapFakeDeckProvider(table)
    app = FastAPI()
    monkeypatch.setenv("BIOXP_RUNTIME_STATE_ROOT", str(tmp_path))
    monkeypatch.setattr(operator_controls.hardware_state, "project", lambda _name: {"domains": {}, "freshness": {"state": "fresh", "age_s": 0, "fresh_for_s": 30}})
    monkeypatch.setattr(operator_controls.hardware_state, "ownership_projection", lambda: {"ownership_epoch": 7, "ownership": "service"})

    operator_controls.install_operator_control_plane(
        app, maintenance_state_provider=lambda: {"motion_blocked": False, "recovery_required": False, "block_reason": None},
        reference_state_provider=lambda: {"rows": {}}, lifecycle_state_provider=lambda: {"operation_state": "stopped"},
        serial206_initialization_state_provider=lambda: {"x_authority": {"active_board_epoch": 11}, "board4_authority": {"active_board_epoch": 10}},
        oem_deck_provider=lambda: provider, oem_deck_position_table_provider=lambda: table,
    )

    semantic = app.state.operator_command_plane.store.deck_semantic_state()
    assert semantic["semantic_state_revision"] == 1
    assert semantic["current_location"] == "LOC_MS"
    assert semantic["producer_operation"] == "semantic_state_bootstrap"
    assert semantic["transition_provenance"]["source_operation"] == "semantic_state_bootstrap"
    assert semantic["transition_provenance"]["upstream_source_operation"] == "migrated_successful_semantic_state"
    action = next(row for row in TestClient(app).get("/operator/v2/control-catalog").json()["actions"] if row["action_id"] == "oem.deck.move_to_location")
    assert action["enabled"] is True
    app.state.operator_command_plane.stop()


def test_controller_coordinates_near_position_table_do_not_establish_semantic_location(monkeypatch):
    from bioxp import oem_serial206_initialization
    from bioxp.oem_deck_catalog import configured_location_names

    table = PositionTable.from_rows([
        {"name": name, "x": index * 100, "y": index * 100 + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ], source_sha256="3" * 64)
    state = Serial206OemInitializationProvider._new_state()
    state["z_lifecycle"].update({
        "state": "referenced_ready", "reference_state": "referenced",
        "generation": 7, "board_lifecycle_generation": 11,
    })
    state["machine_status"].update({
        "tip_loaded": False, "tip_dirty": False, "tip_location": -1,
        "clean_path": True, "current_location": None, "current_well": None,
    })
    runtime_store = FakeRuntimeStore(state)
    provider = Serial206OemInitializationProvider(
        FakeDeckPrimitives(), state_store=runtime_store,
        reference_store=FakeReferenceStore(), generation_provider=lambda: 7,
    )
    monkeypatch.setattr(oem_serial206_initialization, "load_bound_oem_position_table", lambda: table)

    authority = provider.path_planning_authority(expected_generation=7)

    assert authority == {"ok": False, "blockers": ["deck_semantic_state_reader_not_bound"]}
    persisted = runtime_store.state["machine_status"]
    assert persisted["current_location"] is None
    assert persisted["current_well"] is None
    assert "controller_position_observation" not in persisted


def test_serial206_provider_uses_bound_canonical_deck_state_over_stale_local_state(tmp_path, monkeypatch):
    from bioxp import oem_serial206_initialization
    from bioxp.oem_deck_catalog import DeckCatalog, configured_location_names
    from bioxp.oem_deck_movement import DeckAuthoritySnapshot, NamedLocationIntent, compile_named_location
    from bioxp.operator_command_plane import ACTION_REQUEST_SCHEMA, OperatorCommandStore

    table = PositionTable.from_rows([
        {"name": name, "x": index * 100, "y": index * 100 + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ], source_sha256="3" * 64)
    state = Serial206OemInitializationProvider._new_state()
    state["x_lifecycle"].update({"state": "referenced_ready", "reference_state": "referenced", "generation": 7, "board_lifecycle_generation": 11})
    state["z_lifecycle"].update({"state": "referenced_ready", "reference_state": "referenced", "generation": 7, "board_lifecycle_generation": 11})
    state["machine_status"].update({
        "current_location": "LOC_MS", "current_well": 0, "semantic_state_revision": 1,
        "tip_loaded": True, "tip_dirty": True, "tip_location": 3, "clean_path": False,
        "plate_on_gantry": 4, "psudo_z_home_steps": 500,
        "latch_status": False, "latch_closed": False, "latch_observation_id": "stale-local-latch",
    })
    primitives = FakeDeckPrimitives()
    provider = Serial206OemInitializationProvider(
        primitives, state_store=FakeRuntimeStore(state),
        reference_store=FakeReferenceStore(), generation_provider=lambda: 7,
    )
    monkeypatch.setattr(oem_serial206_initialization, "load_bound_oem_position_table", lambda: table)

    with pytest.raises(RuntimeError, match="deck_semantic_state_reader_not_bound"):
        provider.deck_authority_snapshot(expected_generation=7)

    store = _operator_store(tmp_path / "canonical-store")
    request = {
        "schema_version": ACTION_REQUEST_SCHEMA, "idempotency_key": "canonical-provider-snapshot-1",
        "expected_ownership_generation": 7, "expected_board_epoch_by_board": {"4": 10, "5": 11},
        "action_id": "oem.deck.move_to_location", "inputs": {"target": "LOC_OC", "camera_offset": False},
    }
    admitted = store.admit_command(request, state={
        "ownership_generation": 7,
        "serial206_initialization_provider": {"x_authority": {"active_board_epoch": 11}, "board4_authority": {"active_board_epoch": 10}},
    })
    initial_provenance = {
        "source_operation": "canonical_seed",
        "command_id": admitted["command_id"],
        "before_revision": 0,
        "after_revision": 1,
        "latch_status": True,
        "machine_latch_closed": True,
        "latch_observation_id": "latch-1",
    }
    with store._transaction() as conn:
        conn.execute(
            "UPDATE operator_plane_deck_semantic_state SET current_location='LOC_MS',current_well=0,tip_loaded=0,tip_dirty=0,tip_location=-1,clean_path=1,plate_on_gantry=NULL,pseudo_z_home=65000,semantic_state_revision=1,producer_operation='canonical_seed',producer_command_id=?,ownership_generation=6,board_epoch_4=8,board_epoch_5=9,transition_provenance_json=? WHERE singleton=1",
            (admitted["command_id"], json.dumps(initial_provenance, sort_keys=True, separators=(",", ":"))),
        )
    authority = DeckAuthoritySnapshot(
        7, "provider-owner", 10, 11, table.digest, 1,
        {"x": 1, "y": 1, "z": 1, "g": 1}, {"global": 0, "x": 0, "y": 0, "z": 0},
        "latch-1", "position-1", 1.0, 0, 0, 65000, "LOC_MS", 0,
        False, False, -1, True, None, 65000, "BIOXP", True, True,
    )
    plan = compile_named_location(NamedLocationIntent("LOC_OC"), DeckCatalog.from_position_table(table), table, authority)
    store.persist_deck_plan(admitted["command_id"], plan)
    store.commit_deck_success(
        admitted["command_id"], plan,
        [{"provider_command_id": "provider-canonical-1", "controller_command_acknowledged": True, "controller_completion_verified": True}],
    )
    provider.bind_deck_semantic_state_reader(store.deck_semantic_state)
    provider.bind_tip_tray_state_reader(lambda tray_id: {
        "tray_id": tray_id, "tip_available": False,
        "operation_id": "tray-op-canonical", "command_id": "tray-cmd-canonical",
        "ownership_generation": 7, "board_epoch_4": 10, "board_epoch_5": 11,
    })

    snapshot = provider.deck_authority_snapshot(expected_generation=7)
    planning = provider.path_planning_authority(expected_generation=7)
    canonical = store.deck_semantic_state()
    assert snapshot["current_location_id"] == "LOC_OC"
    assert snapshot["current_well_id"] == 0
    assert snapshot["machine_state_revision"] == canonical["semantic_state_revision"]
    assert snapshot["semantic_state_provenance_digest"]
    assert snapshot["tip_loaded"] is False
    assert snapshot["tip_dirty"] is False
    assert snapshot["tip_location"] == -1
    assert snapshot["clean_path"] is True
    assert snapshot["plate_on_gantry"] is None
    assert snapshot["pseudo_z_home"] == 65000
    assert snapshot["latch_status"] is True
    assert snapshot["machine_latch_closed"] is True
    assert snapshot["latch_observation_id"].startswith("deck-latch:")
    assert primitives.latch_reads == 2
    assert canonical["ownership_generation"] == 7
    assert canonical["board_epoch_4"] == 10
    assert canonical["board_epoch_5"] == 11
    assert planning["current_loc"] == "LOC_OC"
    assert planning["current_well"] == 0
    with pytest.raises(RuntimeError, match="semantic_observation_unavailable"):
        provider.deck_reconciliation_snapshot(expected_generation=7)
    result = provider.moveTo(location_id=1, camera_offset=False, authority_snapshot=snapshot)
    assert result["ok"] is True
    primitive_call = primitives.calls[-1]
    assert primitive_call[3]["tip_loaded"] is False
    assert primitive_call[3]["plate_on_gantry"] is None
    assert primitive_call[3]["pseudo_home_steps"] == 65000
    runtime_store_state = provider.state_store.state["machine_status"]
    assert runtime_store_state["current_location"] == "LOC_MS"
    malformed = dict(canonical)
    malformed.pop("clean_path")
    provider.bind_deck_semantic_state_reader(lambda: malformed)
    with pytest.raises(RuntimeError, match="clean_path"):
        provider.deck_authority_snapshot(expected_generation=7)
    store.stop()


def test_serial206_provider_executes_barcode_continuation_and_empty_gantry_park(monkeypatch):
    from bioxp import oem_serial206_initialization
    from bioxp.oem_deck_catalog import configured_location_names

    rows = [
        {"name": name, "x": index * 100, "y": index * 100 + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ]
    rows.append({"name": "CAMERA_OFFSET", "x": 3499, "y": -7744, "zLow": 3145, "zDelta": 6842, "inc_factor": 0})
    table = PositionTable.from_rows(rows, source_sha256="3" * 64)
    primitives = FakeDeckPrimitives()
    state = Serial206OemInitializationProvider._new_state()
    state["machine_status"].update({"tip_loaded": False, "plate_on_gantry": None, "psudo_z_home_steps": 500})
    provider = Serial206OemInitializationProvider(
        primitives, state_store=FakeRuntimeStore(state), reference_store=FakeReferenceStore(),
        generation_provider=lambda: 7,
    )
    monkeypatch.setattr(oem_serial206_initialization, "load_bound_oem_position_table", lambda: table)

    authority = {
        "tip_loaded": False, "tip_dirty": False, "tip_location": -1, "clean_path": True,
        "plate_on_gantry": None, "pseudo_z_home": 500,
        "ownership_generation": 7, "board_epoch_4": 10, "board_epoch_5": 11,
        "current_location_id": "LOC_MS", "current_well_id": 0,
        "machine_state_revision": 1, "semantic_state_provenance_digest": "a" * 64,
    }
    barcode = provider.moveZCamera(location_id=3, authority_snapshot=authority)
    park = provider.parkGantry(authority_snapshot=authority)

    assert barcode["ok"] is True and park["ok"] is True
    assert primitives.calls[0][0] == "z"
    assert primitives.calls[1][0] == "scriptmoveTo"
    assert primitives.calls[1][1]["current_location"] == 0
    assert primitives.calls[1][1]["current_well"] == 0
    assert primitives.calls[1][1]["target_location"] == 28
    assert primitives.calls[1][1]["target_well"] == 0
    assert primitives.calls[1][1]["position_flag"] == 2


def test_catalog_disables_deck_action_when_branch_provider_method_is_missing(tmp_path, monkeypatch):
    from bioxp.oem_deck_catalog import configured_location_names

    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    provider = FakeDeckProvider(table)
    provider.moveZCamera = None
    app = FastAPI()
    monkeypatch.setenv("BIOXP_RUNTIME_STATE_ROOT", str(tmp_path))
    monkeypatch.setattr(operator_controls.hardware_state, "project", lambda _name: {"domains": {}, "freshness": {"state": "fresh", "age_s": 0, "fresh_for_s": 30}})
    monkeypatch.setattr(operator_controls.hardware_state, "ownership_projection", lambda: {"ownership_epoch": 7, "ownership": "service"})
    operator_controls.install_operator_control_plane(
        app, maintenance_state_provider=lambda: {"motion_blocked": False, "recovery_required": False, "block_reason": None},
        reference_state_provider=lambda: {"rows": {}}, lifecycle_state_provider=lambda: {"operation_state": "stopped"},
        serial206_initialization_state_provider=lambda: {"x_authority": {"active_board_epoch": 11}, "board4_authority": {"active_board_epoch": 10}},
        oem_deck_provider=lambda: provider, oem_deck_position_table_provider=lambda: table,
    )
    action = next(row for row in TestClient(app).get("/operator/v2/control-catalog").json()["actions"] if row["action_id"] == "oem.deck.move_to_location")
    assert action["enabled"] is False
    assert action["disabled_reason"] == "canonical_deck_provider_incomplete:moveZCamera"
    app.state.operator_command_plane.stop()


def test_catalog_and_admission_share_durable_deck_recovery_truth(tmp_path, monkeypatch):
    from bioxp.oem_deck_catalog import configured_location_names

    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    provider = AmbiguousFakeDeckProvider(table)
    app = FastAPI()
    monkeypatch.setenv("BIOXP_RUNTIME_STATE_ROOT", str(tmp_path))
    monkeypatch.setattr(operator_controls.hardware_state, "project", lambda _name: {"domains": {}, "freshness": {"state": "fresh", "age_s": 0, "fresh_for_s": 30}})
    monkeypatch.setattr(operator_controls.hardware_state, "ownership_projection", lambda: {"ownership_epoch": 7, "ownership": "service"})
    operator_controls.install_operator_control_plane(
        app, maintenance_state_provider=lambda: {"motion_blocked": False, "recovery_required": False, "block_reason": None},
        reference_state_provider=lambda: {"rows": {}}, lifecycle_state_provider=lambda: {"operation_state": "stopped"},
        serial206_initialization_state_provider=lambda: {"x_authority": {"active_board_epoch": 11}, "board4_authority": {"active_board_epoch": 10}},
        oem_deck_provider=lambda: provider, oem_deck_position_table_provider=lambda: table,
    )
    app.state.operator_command_plane.start()
    client = TestClient(app)
    payload = {
        "schema_version": "bioxp.operator_action_request.v2",
        "idempotency_key": "deck-recovery-catalog-seam-1",
        "expected_ownership_generation": 7,
        "expected_board_epoch_by_board": {"4": 10, "5": 11},
        "inputs": {"target": "LOC_OC", "camera_offset": False},
    }
    admitted = client.post("/operator/v2/actions/oem.deck.move_to_location", json=payload)
    assert admitted.status_code == 200
    command_id = admitted.json()["command_id"]
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        receipt = client.get(f"/operator/commands/{command_id}").json()
        if receipt["status"] == "ambiguous":
            break
        time.sleep(0.01)
    assert receipt["status"] == "ambiguous"

    catalog = client.get("/operator/v2/control-catalog").json()
    action = next(row for row in catalog["actions"] if row["action_id"] == "oem.deck.move_to_location")
    assert action["enabled"] is False
    assert action["disabled_reason"] == "deck_recovery_hold"
    assert action["destination_options"]
    assert all(row["enabled"] is False and row["disabled_reason"] == "deck_recovery_hold" for row in action["destination_options"])
    blocked = client.post("/operator/v2/actions/oem.deck.move_to_location", json={**payload, "idempotency_key": "deck-recovery-catalog-seam-2"})
    assert blocked.status_code == 409
    assert blocked.json()["detail"]["error"] == action["disabled_reason"]

    store = app.state.operator_command_plane.store
    semantic = store.deck_semantic_state()
    reconciliation_authority = {
        "ownership_generation": 7, "provider_owner_id": "catalog-seam-provider",
        "board_epoch_4": 10, "board_epoch_5": 11,
        "position_table_sha256": table.digest,
        "machine_state_revision": semantic["semantic_state_revision"],
        "latch_status": True, "machine_latch_closed": True,
        "latch_observation_id": "catalog-seam-latch-1",
        "controller_position_observation_id": "catalog-seam-position-1",
        "current_x": 0, "current_y": 0, "current_z": 65000, "captured_at": 123.0,
        "observed_location_id": "LOC_MS", "observed_well_id": 0,
    }
    store.reconcile_deck_recovery(
        command_id=command_id, current_location="LOC_MS", current_well=0,
        current_authority=reconciliation_authority,
        current_position_table_revision=table.digest,
        current_destination_catalog_revision=DeckCatalog.from_position_table(table).revision,
        decision={"decision_id": "catalog-seam-decision-1", "approved_by": "operator-test", "reason": "known canonical location observed"},
        final_authority_reader=lambda: dict(reconciliation_authority),
    )
    restored = next(row for row in client.get("/operator/v2/control-catalog").json()["actions"] if row["action_id"] == "oem.deck.move_to_location")
    assert restored["enabled"] is True
    assert all(row["enabled"] is True and row["disabled_reason"] is None for row in restored["destination_options"])
    app.state.operator_command_plane.stop()


def _current_owner_provider(store, *, generation_state=None, tip_exists=True):
    generation_state = generation_state or {"value": 7}
    state = Serial206OemInitializationProvider._new_state()
    state["x_lifecycle"].update({
        "state": "referenced_ready", "reference_state": "referenced",
        "generation": 7, "board_lifecycle_generation": 11,
    })
    primitives = FakeDeckPrimitives()
    primitives.query_all_pipette_tip_states = lambda *, lifecycle_stage_id: {
        "ok": True, "tip_exists": tip_exists,
        "channels_with_tips": [0] if tip_exists else [],
    }
    provider = Serial206OemInitializationProvider(
        primitives, state_store=FakeRuntimeStore(state),
        generation_provider=lambda: generation_state["value"],
    )
    provider.bind_deck_semantic_state_reader(store.deck_semantic_state)
    store.bind_deck_owner_authority_reader(provider.deck_owner_authority_stamps)
    provider.bind_deck_semantic_state_publisher(store.publish_deck_owner_state)
    return provider, generation_state


def test_initialize_motion_real_tip_query_publishes_current_authority_atomically(tmp_path) -> None:
    store = _operator_store(tmp_path)
    store.bootstrap_deck_semantic_state({
        "current_location": "LOC_MS", "current_well": 0, "current_tray": None,
        "tip_loaded": False, "tip_dirty": False, "tip_location": 2, "clean_path": True,
        "plate_on_gantry": None, "movable_plate_locations": _complete_oem_movable_locations(), "pseudo_z_home": 65000,
        "ownership_generation": 6, "board_epoch_4": 8, "board_epoch_5": 9,
        "latch_status": True, "machine_latch_closed": True, "latch_observation_id": "latch-1",
        "source_operation": "migrated_successful_semantic_state", "source_command_id": "legacy-1",
    })
    provider, _generation = _current_owner_provider(store)
    result = provider.query_and_publish_pipette_state(
        source_command_id="initialize-motion-real-tip-query",
    )

    semantic = store.deck_semantic_state()
    assert result["ok"] is True
    assert semantic["tip_loaded"] is True and semantic["tip_location"] == 2
    assert (semantic["ownership_generation"], semantic["board_epoch_4"], semantic["board_epoch_5"]) == (7, 10, 11)
    store.stop()


def test_owner_publication_generation_drift_in_sqlite_transaction_inserts_nothing(tmp_path) -> None:
    store = _operator_store(tmp_path)
    store.bootstrap_deck_semantic_state({
        "current_location": "LOC_MS", "current_well": 0, "current_tray": None,
        "tip_loaded": False, "tip_dirty": False, "tip_location": -1, "clean_path": True,
        "plate_on_gantry": None, "movable_plate_locations": _complete_oem_movable_locations(), "pseudo_z_home": 65000,
        "ownership_generation": 7, "board_epoch_4": 10, "board_epoch_5": 11,
        "latch_status": True, "machine_latch_closed": True, "latch_observation_id": "latch-1",
        "source_operation": "migrated_successful_semantic_state", "source_command_id": "legacy-1",
    })
    provider, generation = _current_owner_provider(store, tip_exists=False)
    commands_before = store.connection.execute("SELECT COUNT(*) FROM operator_plane_commands").fetchone()[0]
    revisions_before = store.connection.execute("SELECT COUNT(*) FROM operator_plane_deck_semantic_transitions").fetchone()[0]
    real_publisher = store.publish_deck_owner_state

    def drifting_publisher(**kwargs):
        generation["value"] = 8
        return real_publisher(**kwargs)

    provider.bind_deck_semantic_state_publisher(drifting_publisher)
    with pytest.raises(RuntimeError, match="deck_owner_authority_changed"):
        provider.query_and_publish_pipette_state(source_command_id="drift-query")

    assert store.connection.execute("SELECT COUNT(*) FROM operator_plane_commands").fetchone()[0] == commands_before
    assert store.connection.execute("SELECT COUNT(*) FROM operator_plane_deck_semantic_transitions").fetchone()[0] == revisions_before
    assert store.deck_semantic_state()["semantic_state_revision"] == 1
    store.stop()


def test_wp6_success_derives_current_tray_from_oem_update_location_mapping(tmp_path) -> None:
    from bioxp.oem_deck_catalog import configured_location_names
    from bioxp.oem_deck_movement import DeckAuthoritySnapshot, NamedLocationIntent, compile_named_location

    table = PositionTable.from_rows([
        {"name": name, "x": index, "y": index + 1, "zLow": 60000, "zDelta": 10000, "inc_factor": 0}
        for index, name in enumerate(configured_location_names())
    ])
    store = _operator_store(tmp_path)
    store.bootstrap_deck_semantic_state({
        "current_location": "LOC_MS", "current_well": 0, "current_tray": None,
        "tip_loaded": False, "tip_dirty": False, "tip_location": -1, "clean_path": True,
        "plate_on_gantry": None,
        "movable_plate_locations": _complete_oem_movable_locations(), "pseudo_z_home": 65000,
        "ownership_generation": 7, "board_epoch_4": 10, "board_epoch_5": 11,
        "latch_status": True, "machine_latch_closed": True, "latch_observation_id": "latch-1",
        "source_operation": "migrated_successful_semantic_state", "source_command_id": "legacy-1",
    })
    authority = DeckAuthoritySnapshot(
        7, "provider", 10, 11, table.digest, 1,
        {"x": 1, "y": 1, "z": 1, "g": 1}, {"global": 0, "x": 0, "y": 0, "z": 0},
        "latch-1", "position-1", 1.0, 0, 0, 65000, "LOC_MS", 0,
        False, False, -1, True, None, 65000, "BIOXP", True, True,
    )
    plan = compile_named_location(NamedLocationIntent("LOC_OC"), DeckCatalog.from_position_table(table), table, authority)
    admitted = store.admit_command({
        "schema_version": "bioxp.operator_action_request.v2", "idempotency_key": "tray-map-1",
        "expected_ownership_generation": 7, "expected_board_epoch_by_board": {"4": 10, "5": 11},
        "action_id": "oem.deck.move_to_location", "inputs": {"target": "LOC_OC", "camera_offset": False},
    }, state={
        "ownership_generation": 7,
        "serial206_initialization_provider": {"x_authority": {"active_board_epoch": 11}, "board4_authority": {"active_board_epoch": 10}},
    })
    store.persist_deck_plan(admitted["command_id"], plan)
    store.commit_deck_success(admitted["command_id"], plan, [{
        "controller_command_acknowledged": True, "controller_completion_verified": True,
    }])

    semantic = store.deck_semantic_state()
    assert semantic["current_location"] == "LOC_OC" and semantic["current_well"] == 0
    assert semantic["current_tray"] == "OUTPUT_PLATE"

    authority_after_output = DeckAuthoritySnapshot(
        7, "provider", 10, 11, table.digest, semantic["semantic_state_revision"],
        {"x": 1, "y": 1, "z": 1, "g": 1}, {"global": 0, "x": 0, "y": 0, "z": 0},
        "latch-2", "position-2", 2.0, 1, 2, 65000, "LOC_OC", 0,
        False, False, -1, True, None, 65000, "BIOXP", True, True,
    )
    unmapped_plan = compile_named_location(
        NamedLocationIntent("LOC_TROUGH"), DeckCatalog.from_position_table(table),
        table, authority_after_output,
    )
    unmapped = store.admit_command({
        "schema_version": "bioxp.operator_action_request.v2", "idempotency_key": "tray-map-2",
        "expected_ownership_generation": 7, "expected_board_epoch_by_board": {"4": 10, "5": 11},
        "action_id": "oem.deck.move_to_location", "inputs": {"target": "LOC_TROUGH", "camera_offset": False},
    }, state={
        "ownership_generation": 7,
        "serial206_initialization_provider": {"x_authority": {"active_board_epoch": 11}, "board4_authority": {"active_board_epoch": 10}},
    })
    store.persist_deck_plan(unmapped["command_id"], unmapped_plan)
    store.commit_deck_success(unmapped["command_id"], unmapped_plan, [{
        "controller_command_acknowledged": True, "controller_completion_verified": True,
    }])
    assert store.deck_semantic_state()["current_tray"] == "OUTPUT_PLATE"
    store.stop()


class _RecoveryPositionTester:
    def __init__(self, coordinates, *, host_latch=True, sensor_latch=1):
        self.coordinates = dict(coordinates)
        self.reads = []
        self.host_latch = host_latch
        self.sensor_latch = sensor_latch

    def _motion_oem_axis_profile(self, axis, startup=True):
        assert startup is True
        return {"board": axis, "motor": 0}

    def motor_get_position(self, board, motor=0):
        self.reads.append((board, motor))
        return {"ok": True, "position": self.coordinates[board]}

    def deck_io_query_type(self, io_type):
        assert io_type == 3
        return {"ok": True, "value": self.sensor_latch, "ack": {"status": 100, "value": self.sensor_latch}}

    def read_oem_latch_status(self):
        return {"ok": True, "value": self.host_latch, "observation_id": f"host-latch:{int(self.host_latch)}"}


def test_production_recovery_observation_exactly_binds_position_table_well_zero(monkeypatch) -> None:
    from bioxp import oem_serial206_initialization

    table = PositionTable.from_rows([
        {"name": "LOC_MS", "x": 101, "y": 202, "zLow": 60000, "zDelta": 10000, "inc_factor": 0},
        {"name": "LOC_OC", "x": 301, "y": 402, "zLow": 60000, "zDelta": 10000, "inc_factor": 0},
    ])
    monkeypatch.setattr(oem_serial206_initialization, "load_bound_oem_position_table", lambda: table)
    monkeypatch.setattr(PositionTable, "resolve_nearest", lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("nearest forbidden")))
    tester = _RecoveryPositionTester({"x": 101, "y": 202, "z": 65000})
    primitives = Serial206ProductionPrimitiveAdapter(
        tester, object(), authority_provider=lambda: {}, generation_provider=lambda: 7,
    )
    primitives.deck_io_query_type = tester.deck_io_query_type
    state = Serial206OemInitializationProvider._new_state()
    state["x_lifecycle"]["board_lifecycle_generation"] = 11
    provider = Serial206OemInitializationProvider(
        primitives, state_store=FakeRuntimeStore(state),
        reference_store=FakeReferenceStore(), generation_provider=lambda: 7,
    )
    provider.bind_deck_semantic_state_reader(lambda: {
        "current_location": "LOC_OC", "current_well": 0, "semantic_state_revision": 3,
        "ambiguity_state": "recovery_required", "producer_operation": "updateLocation",
        "producer_command_id": "ambiguous-command", "tip_loaded": False, "tip_dirty": False,
        "tip_location": -1, "clean_path": True, "plate_on_gantry": None, "pseudo_z_home": 65000,
        "ownership_generation": 7, "board_epoch_4": 10, "board_epoch_5": 11,
        "latch_status": True, "machine_latch_closed": True, "latch_observation_id": "prior-latch",
        "transition_provenance": {"source_operation": "updateLocation", "command_id": "ambiguous-command"},
    })
    provider.bind_tip_tray_state_reader(lambda tray_id: {
        "tray_id": tray_id, "tip_available": False,
        "operation_id": "tray-op-recovery", "command_id": "tray-cmd-recovery",
        "ownership_generation": 7, "board_epoch_4": 10, "board_epoch_5": 11,
    })

    observed = provider.deck_reconciliation_snapshot(expected_generation=7)

    assert observed["observed_location_id"] == "LOC_MS" and observed["observed_well_id"] == 0
    assert (observed["current_x"], observed["current_y"], observed["current_z"]) == (101, 202, 65000)
    assert tester.reads == [("x", 0), ("y", 0), ("z", 0), ("x", 0), ("y", 0), ("z", 0)]


def test_production_recovery_observation_rejects_one_step_nearest_match(monkeypatch) -> None:
    from bioxp import oem_serial206_initialization

    table = PositionTable.from_rows([
        {"name": "LOC_MS", "x": 101, "y": 202, "zLow": 60000, "zDelta": 10000, "inc_factor": 0},
    ])
    monkeypatch.setattr(oem_serial206_initialization, "load_bound_oem_position_table", lambda: table)
    primitives = Serial206ProductionPrimitiveAdapter(
        _RecoveryPositionTester({"x": 102, "y": 202, "z": 65000}), object(),
        authority_provider=lambda: {}, generation_provider=lambda: 7,
    )

    with pytest.raises(RuntimeError, match="semantic_location_unavailable"):
        primitives.read_deck_semantic_observation()


def test_deck_authority_uses_independent_host_and_type3_latch_producers(monkeypatch) -> None:
    from bioxp import oem_serial206_initialization

    table = PositionTable.from_rows([
        {"name": "LOC_MS", "x": 101, "y": 202, "zLow": 60000, "zDelta": 10000, "inc_factor": 0},
    ])
    monkeypatch.setattr(oem_serial206_initialization, "load_bound_oem_position_table", lambda: table)
    tester = _RecoveryPositionTester(
        {"x": 101, "y": 202, "z": 65000}, host_latch=False, sensor_latch=1,
    )
    primitives = Serial206ProductionPrimitiveAdapter(
        tester, object(), authority_provider=lambda: {}, generation_provider=lambda: 7,
    )
    state = Serial206OemInitializationProvider._new_state()
    state["x_lifecycle"]["board_lifecycle_generation"] = 11
    provider = Serial206OemInitializationProvider(
        primitives, state_store=FakeRuntimeStore(state),
        reference_store=FakeReferenceStore(), generation_provider=lambda: 7,
    )
    provider.bind_deck_semantic_state_reader(lambda: {
        "current_location": "LOC_MS", "current_well": 0, "semantic_state_revision": 1,
        "ambiguity_state": "none", "producer_operation": "updateLocation", "producer_command_id": "cmd-1",
        "tip_loaded": False, "tip_dirty": False, "tip_location": -1, "clean_path": True,
        "plate_on_gantry": None, "pseudo_z_home": 65000, "ownership_generation": 7,
        "board_epoch_4": 10, "board_epoch_5": 11, "latch_status": True,
        "machine_latch_closed": True, "latch_observation_id": "prior",
        "transition_provenance": {"source_operation": "updateLocation", "command_id": "cmd-1"},
    })
    provider.bind_tip_tray_state_reader(lambda tray_id: {
        "tray_id": tray_id, "tip_available": False,
        "operation_id": "tray-op-latch", "command_id": "tray-cmd-latch",
        "ownership_generation": 7, "board_epoch_4": 10, "board_epoch_5": 11,
    })

    snapshot = provider.deck_authority_snapshot(expected_generation=7)

    assert snapshot["latch_status"] is False
    assert snapshot["machine_latch_closed"] is True
    assert "host-latch:0" in snapshot["latch_observation_id"]
    assert "type3" in snapshot["latch_observation_id"]
