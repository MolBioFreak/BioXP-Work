from __future__ import annotations

import asyncio
import sqlite3
import threading
import time
from pathlib import Path

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from httpx import ASGITransport, AsyncClient, Response
from pydantic import BaseModel, Field

import bioxp.operator_command_plane as operator_command_plane
import bioxp.operator_controls as operator_controls
from bioxp.operator_command_plane import OperatorCommandPlane
from bioxp.operator_controls import install_operator_control_plane
from bioxp.operator_receipt_store import OperatorReceiptStore
from bioxp.oem_runtime_store import OEMRuntimeStore


class CamelBody(BaseModel):
    targetPosition: int


class BoundedBody(BaseModel):
    integer_gt: int = Field(gt=0, le=10)
    number_lt: float = Field(ge=-2.5, lt=2.5)


class AckBody(BaseModel):
    operator_ack: str


class StageApproval(BaseModel):
    approval_id: str


class OptionalNestedBody(BaseModel):
    stage_approval: StageApproval | None = None


class RecoveryBody(BaseModel):
    run_homing: bool = False
    operator_ack: str
    operator_reason: str


def make_app(tmp_path: Path, monkeypatch, *, pipette_status_provider=None):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(tmp_path))
    monkeypatch.setattr(operator_controls, "current_release_identity", lambda: {
        "verified": True,
        "release_id": "test-release",
        "source": {"manifest_sha256": "1" * 64, "aggregate_sha256": "2" * 64},
    })
    monkeypatch.setattr(operator_controls, "current_authority_identity", lambda: {
        "evidence_lock_identity_verified": True,
        "evidence_lock_sha256": "3" * 64,
    })
    monkeypatch.setattr(operator_controls, "current_registry_sha256", lambda: "4" * 64)
    runtime_owner = OEMRuntimeStore(tmp_path)
    runtime_owner.close()
    app = FastAPI()
    calls: list[tuple[str, object]] = []


    @app.get("/motion/axis/{axis}/status")
    async def axis_status(axis: str, verbose: bool = False):
        calls.append(("axis_status", {"axis": axis, "verbose": verbose}))
        return {"ok": True, "axis": axis, "verbose": verbose}

    @app.post("/motion/test/home_axis")
    async def home_axis(body: dict):
        calls.append(("home_axis", body))
        override = getattr(app.state, "operator_test_home_axis_response", None)
        if override is not None:
            return override
        return {"ok": True, "controller_acknowledged": True, "stages": [{"stage_id": "home", "status": "passed"}]}

    @app.post("/motion/oem/prepare_without_motion")
    async def prepare_without_motion():
        calls.append(("prepare_without_motion", None))
        return {"ok": True, "physical_motion": False, "homing_performed": False}

    @app.post("/motion/arm/strict_startup")
    async def recover_motion_non_homing(body: RecoveryBody):
        calls.append(("recover_motion_non_homing", body.model_dump()))
        return {"ok": True, "physical_motion": False, "homing_performed": False}

    @app.post("/motion/oem/x/move_steps")
    async def x_move_steps(steps: int, wait_timeout_s: float = 5.0):
        base = getattr(app.state, "operator_test_x_position", 0)
        app.state.operator_test_x_position = base + steps
        calls.append(("x_move_steps", {"steps": steps, "base_position": base}))
        blocker = getattr(app.state, "operator_test_x_move_steps_blocker", None)
        if blocker is not None:
            await blocker()
        override = getattr(app.state, "operator_test_x_move_steps_response", None)
        if override is not None:
            return override
        return {"ok": True, "controller_acknowledged": True, "status": "completed", "physical_effect_verified": False}

    @app.post("/motion/oem/manual/home")
    async def manual_home(axis: str):
        calls.append(("manual_home", {"axis": axis}))
        return {"ok": True, "controller_acknowledged": True, "physical_effect_verified": False}

    @app.post("/motion/oem/home_xy")
    async def home_xy():
        calls.append(("home_xy", None))
        return {"ok": True, "stages": [{"stage_id": "x"}, {"stage_id": "y"}]}

    @app.post("/motion/oem/move_xy")
    async def move_xy(x: int, y: int, timeout_s: float = 120.0):
        calls.append(("move_xy", {"x": x, "y": y, "timeout_s": timeout_s}))
        return {"ok": True, "stages": [{"stage_id": "x"}, {"stage_id": "y"}]}


    @app.post("/motion/oem/x/internal/enable_xy")
    async def enable_xy_internal():
        calls.append(("enable_xy_internal", None))
        return {"ok": True}

    @app.post("/motion/oem/x/abort")
    async def abort_all():
        calls.append(("abort_all", None))
        return {"ok": True, "controller_acknowledged": True}

    @app.post("/motion/diagnostics/stop")
    async def diagnostic_stop():
        calls.append(("diagnostic_stop", None))
        return {"ok": True, "verified_stopped": True}

    @app.post("/motion/camel")
    async def camel(body: CamelBody):
        calls.append(("camel", body.model_dump()))
        return {"ok": True, "targetPosition": body.targetPosition}

    @app.post("/motion/bounded")
    async def bounded(body: BoundedBody):
        calls.append(("bounded", body.model_dump()))
        return {"ok": True, **body.model_dump()}

    @app.post("/motion/gripper/clear")
    async def gripper_clear(body: AckBody):
        calls.append(("gripper_clear", body.model_dump()))
        return {"ok": True}

    @app.post("/motion/optional-nested")
    async def optional_nested(body: OptionalNestedBody):
        calls.append(("optional_nested", body.model_dump()))
        return {"ok": True}

    @app.post("/pipette/{channel}/aspirate")
    def pipette_aspirate(channel: int, volume_ul: float):
        calls.append(("pipette_aspirate", {"channel": channel, "volume_ul": volume_ul}))
        return {"ok": True, "channel": channel, "volume_ul": volume_ul}

    @app.post("/maintenance/usb/reconnect")
    def local_maintenance_reconnect():
        calls.append(("local_maintenance_reconnect", None))
        return {"ok": True}

    class FakeHardwareState:
        ownership_epoch = 7

        def ownership_projection(self):
            return {"ownership_epoch": self.ownership_epoch, "ownership": {"transport": "owned", "usb": "service", "router": "running", "CAN_READY": True}}

        def project(self, domain):
            if domain == "power":
                observation = {"safety_valid": True}
            elif domain == "latch":
                observation = {"door_sensor": 1, "latch_sensor": 1}
            elif domain == "interlock":
                observation = {"motion_arm": {"armed": True}}
            else:
                observation = {}
            return {
                "snapshot_id": "test-snapshot",
                "freshness": {"state": "fresh", "age_s": 0.0, "fresh_for_s": 30.0},
                "domains": {domain: {"status": "observed", "observation": observation}},
            }

    maintenance = {"motion_blocked": False, "recovery_required": False, "block_reason": None}
    lifecycle = {"operation_state": "stopped", "operation_reason": "test", "door": {"door_closed": True, "latch_closed": True}}
    app.state.operator_test_maintenance = maintenance
    app.state.operator_test_lifecycle = lifecycle
    hardware = FakeHardwareState()
    app.state.operator_test_hardware = hardware
    monkeypatch.setattr(operator_controls, "hardware_state", hardware)
    serial206_state = {
        "bound": True,
        "initialize_motors_live_available": True,
        "x_authority": {
            "state": "referenced",
            "reference_state": "referenced",
            "lifecycle": {
                "state": "referenced_ready",
                "board_lifecycle_generation_fresh": True,
            },
        },
        "z_authority": {"state": "referenced", "reference_state": "referenced", "lifecycle": {"state": "referenced_ready"}},
    }
    app.state.operator_test_serial206 = serial206_state
    install_operator_control_plane(
        app,
        maintenance_state_provider=lambda: app.state.operator_test_maintenance,
        reference_state_provider=lambda: {"rows": {axis: {"state": "referenced"} for axis in ("x", "y", "z", "g", "door")}},
        lifecycle_state_provider=lambda: app.state.operator_test_lifecycle,
        serial206_initialization_state_provider=lambda: serial206_state,
        pipette_status_provider=pipette_status_provider,
    )
    return app, calls


def action_for(catalog: dict, method: str, path: str) -> dict:
    return next(action for action in catalog["actions"] if action["informational_method"] == method and action["informational_path"] == path and action["kind"] == "primitive")


def test_dashboard_uses_passive_four_channel_pipette_provider_when_projection_is_missing(tmp_path, monkeypatch):
    passive = {
        "ok": False,
        "transport": "novo_usb_can",
        "channels": [{"channel": channel, "available": False} for channel in range(4)],
        "channel_count": 4,
    }
    app, _ = make_app(tmp_path, monkeypatch, pipette_status_provider=lambda: passive)

    dashboard = TestClient(app).get("/operator/dashboard").json()

    assert dashboard["pipettes"] == passive


def test_catalog_has_every_exact_route_once_and_distinct_meta_actions(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    assert catalog["schema_name"] == "bioxp.operator_control_catalog"
    action_ids = {row["action_id"] for row in catalog["actions"]}
    assert {"oem.xy.move_absolute", "oem.xy.home"} <= action_ids
    assert {"oem.xy.move_xy", "oem.xy.home_xy"}.isdisjoint(action_ids)
    primitive_routes = [(row["informational_method"], row["informational_path"]) for row in catalog["actions"] if row["kind"] == "primitive"]
    assert primitive_routes.count(("GET", "/motion/axis/{axis}/status")) == 1
    assert primitive_routes.count(("POST", "/motion/test/home_axis")) == 1
    assert primitive_routes.count(("POST", "/motion/oem/home_xy")) == 1
    assert primitive_routes.count(("POST", "/motion/oem/move_xy")) == 1
    meta_ids = {row["action_id"] for row in catalog["actions"] if row["kind"] == "meta"}
    assert meta_ids == {
        "meta.activate_motion",
        "meta.recover_motion_non_homing",
        "meta.initialize_motors",
        "meta.initialize_motion",
    }
    motors = next(row for row in catalog["actions"] if row["action_id"] == "meta.initialize_motors")
    motion = next(row for row in catalog["actions"] if row["action_id"] == "meta.initialize_motion")
    assert motors["available"] is False
    assert motion["available"] is False
    assert len(motors["stages"]) == 19
    assert catalog["dashboard"]["snapshot"]["collection_triggered"] is False
    assert all("enabled" in row and "dependencies" in row for row in catalog["actions"])
    assert all(
        input_spec["name"] not in {"operator_ack", "operator_reason"}
        for action in catalog["actions"]
        for input_spec in action["inputs"]
    )
    optional_nested = action_for(catalog, "POST", "/motion/optional-nested")
    assert next(row for row in optional_nested["inputs"] if row["name"] == "stage_approval")["default"] is None


def test_v2_catalog_entrypoints_publish_the_same_canonical_action_set(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    client = TestClient(app)

    direct = client.get("/operator/v2/control-catalog")
    negotiated = client.get(
        "/operator/control-catalog",
        params={"schema_version": "bioxp.operator_control_catalog.v2"},
    )

    assert direct.status_code == 200
    assert negotiated.status_code == 200
    direct_ids = [row["action_id"] for row in direct.json()["actions"]]
    negotiated_ids = [row["action_id"] for row in negotiated.json()["actions"]]
    assert negotiated_ids == direct_ids


def test_v2_strict_method_route_is_the_only_admitted_composite_path(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    app.state.operator_test_serial206["board4_authority"] = {"active_board_epoch": 10}
    app.state.operator_test_serial206["x_authority"]["active_board_epoch"] = 11
    client = TestClient(app)
    payload = {
        "schema_version": "bioxp.operator_method_request.v1",
        "method_action_id": "oem.xy.move_absolute",
        "idempotency_key": "v2-strict-method-route-1",
        "expected_ownership_generation": 7,
        "expected_board_epoch_by_board": {"4": 10, "5": 11},
        "inputs": {"x_steps": 200, "y_steps": 300},
    }

    bypass = client.post(
        "/operator/v2/actions/oem.xy.move_absolute",
        json={
            "schema_version": "bioxp.operator_action_request.v2",
            "idempotency_key": "v2-strict-action-bypass-1",
            "expected_ownership_generation": 7,
            "expected_board_epoch_by_board": {"4": 10, "5": 11},
            "inputs": {"x": 200, "y": 300},
        },
    )
    assert bypass.status_code == 409
    assert bypass.json()["detail"]["error"] == "canonical_strict_method_requires_v2_route"

    admitted = client.post("/operator/v2/methods", json=payload)

    assert admitted.status_code == 200, admitted.text
    body = admitted.json()
    assert body["action_id"] == "oem.xy.move_absolute"
    assert body["status"] == "queued"
    status = client.get(f"/operator/v2/methods/{body['method_id']}")
    assert status.status_code == 200
    assert status.json()["method_id"] == body["method_id"]


def test_v2_xyz_preview_is_not_admitted_as_a_completed_strict_method(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    app.state.operator_test_serial206["board4_authority"] = {"active_board_epoch": 10}
    app.state.operator_test_serial206["x_authority"]["active_board_epoch"] = 11
    client = TestClient(app)

    catalog = client.get("/operator/v2/control-catalog").json()
    assert "oem.xyz.move_to" not in {row["action_id"] for row in catalog["actions"]}
    admitted = client.post(
        "/operator/v2/methods",
        json={
            "schema_version": "bioxp.operator_method_request.v1",
            "method_action_id": "oem.xyz.move_to",
            "idempotency_key": "v2-strict-xyz-method-1",
            "expected_ownership_generation": 7,
            "expected_board_epoch_by_board": {"4": 10, "5": 11},
            "inputs": {"x": 200, "y": 300, "z": 400, "pseudo_z_home": 500},
        },
    )

    assert admitted.status_code == 422
    assert "oem.xyz.move_to" not in app.state.operator_command_plane.dispatch
    assert app.state.operator_command_plane.store.connection.execute(
        "SELECT COUNT(*) FROM operator_plane_methods"
    ).fetchone()[0] == 0


def test_v2_abort_arms_the_durable_interrupt_fence(tmp_path, monkeypatch):
    app, direct_calls = make_app(tmp_path, monkeypatch)
    durable_calls: list[tuple[str, dict]] = []

    async def compat_invoke(action_id, payload):
        durable_calls.append((action_id, dict(payload)))
        return {
            "schema_version": "bioxp.operator_interrupt_receipt.v1",
            "action_id": action_id,
            "interrupt_id": "interrupt-1",
            "status": "completed",
            "persistence_state": "committed",
        }

    app.state.operator_command_plane.compat_invoke = compat_invoke
    response = TestClient(app).post(
        "/operator/v2/actions/oem.abort_all",
        json={
            "schema_version": "bioxp.operator_interrupt_request.v1",
            "idempotency_key": "v2-durable-abort-1",
            "reason": "operator requested abort",
            "observed_ownership_generation": 7,
            "observed_board_epoch_by_board": {"4": 10, "5": 11},
        },
    )

    assert response.status_code == 200, response.text
    assert durable_calls[0][0] == "oem.abort_all"
    assert durable_calls[0][1]["idempotency_key"] == "v2-durable-abort-1"
    assert direct_calls == []


def test_interrupt_is_durably_admitted_before_controller_delivery(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    plane = app.state.operator_command_plane
    order: list[str] = []
    real_begin = plane.store.begin_interrupt
    real_mark_attempted = plane.store.mark_interrupt_attempted

    def begin_interrupt(*args, **kwargs):
        order.append("durable_begin")
        return real_begin(*args, **kwargs)

    def mark_interrupt_attempted(*args, **kwargs):
        order.append("durable_attempted")
        return real_mark_attempted(*args, **kwargs)

    async def deliver(*_args, **_kwargs):
        order.append("controller_delivery")
        return 200, {
            "ok": True,
            "source_call_completed": True,
            "controller_command_acknowledged": True,
        }

    monkeypatch.setattr(plane.store, "begin_interrupt", begin_interrupt)
    monkeypatch.setattr(plane.store, "mark_interrupt_attempted", mark_interrupt_attempted)
    monkeypatch.setattr(plane, "_deliver_controller_interrupt_raw", deliver)
    receipt = asyncio.run(plane.compat_invoke(
        "oem.abort_all",
        {
            "schema_version": "bioxp.operator_interrupt_request.v1",
            "idempotency_key": "durable-before-delivery-1",
            "reason": "test",
            "observed_ownership_generation": 7,
            "observed_board_epoch_by_board": {"4": 10, "5": 11},
        },
    ))

    assert order[:3] == ["durable_begin", "durable_attempted", "controller_delivery"]
    assert receipt["persistence_state"] == "committed"


def test_interrupt_admission_failure_prevents_controller_delivery(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    plane = app.state.operator_command_plane
    delivered = False

    def reject_begin(*_args, **_kwargs):
        raise RuntimeError("durable interrupt admission unavailable")

    async def deliver(*_args, **_kwargs):
        nonlocal delivered
        delivered = True
        return 200, {"ok": True, "source_call_completed": True}

    monkeypatch.setattr(plane.store, "begin_interrupt", reject_begin)
    monkeypatch.setattr(plane, "_deliver_controller_interrupt_raw", deliver)
    receipt = asyncio.run(plane.compat_invoke(
        "oem.abort_all",
        {
            "schema_version": "bioxp.operator_interrupt_request.v1",
            "idempotency_key": "failed-durable-admission-1",
            "reason": "test",
            "observed_ownership_generation": 7,
            "observed_board_epoch_by_board": {"4": 10, "5": 11},
        },
    ))

    assert delivered is False
    assert receipt["controller_stop_attempted"] is False
    assert receipt["recovery_hold"] is True


def test_concurrent_identical_interrupts_share_one_durable_attempt_and_delivery(
    tmp_path, monkeypatch,
):
    app, _ = make_app(tmp_path, monkeypatch)
    plane = app.state.operator_command_plane
    barrier = threading.Barrier(2)
    deliveries: list[str] = []

    class ConcurrentAdmissionGate:
        def __enter__(self):
            barrier.wait(timeout=2.0)

        def __exit__(self, *_args):
            return False

    async def deliver(_action_id, *, interrupt_attempt_id):
        deliveries.append(interrupt_attempt_id)
        return 200, {
            "ok": True,
            "source_call_completed": True,
            "controller_command_acknowledged": True,
        }

    plane.store._interrupt_lock = ConcurrentAdmissionGate()
    monkeypatch.setattr(plane, "_deliver_controller_interrupt_raw", deliver)
    request = {
        "schema_version": "bioxp.operator_interrupt_request.v1",
        "idempotency_key": "concurrent-durable-interrupt-1",
        "reason": "test",
        "observed_ownership_generation": 7,
        "observed_board_epoch_by_board": {"4": 10, "5": 11},
    }

    async def invoke_twice():
        return await asyncio.gather(
            plane.compat_invoke("oem.abort_all", request),
            plane.compat_invoke("oem.abort_all", request),
        )

    receipts = asyncio.run(invoke_twice())
    distinct_attempts = plane.store.connection.execute(
        "SELECT COUNT(DISTINCT interrupt_attempt_id) "
        "FROM operator_plane_interrupt_attempts WHERE idempotency_key=?",
        (request["idempotency_key"],),
    ).fetchone()[0]

    assert distinct_attempts == 1
    assert len(deliveries) == 1
    assert {receipt["interrupt_attempt_id"] for receipt in receipts} == {deliveries[0]}


def test_idempotent_interrupt_replay_keeps_fence_until_live_delivery_finishes(
    tmp_path, monkeypatch,
):
    app, _ = make_app(tmp_path, monkeypatch)
    plane = app.state.operator_command_plane
    delivery_started = asyncio.Event()
    release_delivery = asyncio.Event()

    async def deliver(_action_id, *, interrupt_attempt_id):
        delivery_started.set()
        await release_delivery.wait()
        return 200, {
            "ok": True,
            "source_call_completed": True,
            "controller_command_acknowledged": True,
        }

    monkeypatch.setattr(plane, "_deliver_controller_interrupt_raw", deliver)
    request = {
        "schema_version": "bioxp.operator_interrupt_request.v1",
        "idempotency_key": "concurrent-live-delivery-fence-1",
        "reason": "test",
        "observed_ownership_generation": 7,
        "observed_board_epoch_by_board": {"4": 10, "5": 11},
    }

    async def exercise() -> None:
        first = asyncio.create_task(plane.compat_invoke("oem.abort_all", request))
        await delivery_started.wait()
        replay = await plane.compat_invoke("oem.abort_all", request)
        assert replay["controller_stop_attempted"] is True
        assert plane.store.action_fenced("oem.x.move_steps") is True
        release_delivery.set()
        await first
        assert plane.store.action_fenced("oem.x.move_steps") is False

    asyncio.run(exercise())


def test_completed_interrupt_idempotency_key_is_redelivered_for_new_safety_request(
    tmp_path, monkeypatch,
):
    app, _ = make_app(tmp_path, monkeypatch)
    plane = app.state.operator_command_plane
    deliveries: list[str] = []

    async def deliver(_action_id, *, interrupt_attempt_id):
        deliveries.append(interrupt_attempt_id)
        return 200, {
            "ok": True,
            "source_call_completed": True,
            "controller_command_acknowledged": True,
        }

    monkeypatch.setattr(plane, "_deliver_controller_interrupt_raw", deliver)
    request = {
        "schema_version": "bioxp.operator_interrupt_request.v1",
        "idempotency_key": "repeat-completed-safety-interrupt-1",
        "reason": "test",
        "observed_ownership_generation": 7,
        "observed_board_epoch_by_board": {"4": 10, "5": 11},
    }

    first = asyncio.run(plane.compat_invoke("oem.abort_all", request))
    second = asyncio.run(plane.compat_invoke("oem.abort_all", request))

    assert len(deliveries) == 2
    assert deliveries == [first["interrupt_attempt_id"], second["interrupt_attempt_id"]]
    assert first["interrupt_attempt_id"] != second["interrupt_attempt_id"]


def test_shared_aggregate_fence_waits_for_all_aggregate_interrupt_deliveries(
    tmp_path, monkeypatch,
):
    app, _ = make_app(tmp_path, monkeypatch)
    plane = app.state.operator_command_plane
    started = {"oem.abort_all": asyncio.Event(), "oem.z.abort": asyncio.Event()}
    release = {"oem.abort_all": asyncio.Event(), "oem.z.abort": asyncio.Event()}

    async def deliver(action_id, *, interrupt_attempt_id):
        started[action_id].set()
        await release[action_id].wait()
        return 200, {
            "ok": True,
            "source_call_completed": True,
            "controller_command_acknowledged": True,
        }

    monkeypatch.setattr(plane, "_deliver_controller_interrupt_raw", deliver)

    def request(key: str) -> dict:
        return {
            "schema_version": "bioxp.operator_interrupt_request.v1",
            "idempotency_key": key,
            "reason": "test",
            "observed_ownership_generation": 7,
            "observed_board_epoch_by_board": {"4": 10, "5": 11},
        }

    async def exercise() -> None:
        first = asyncio.create_task(
            plane.compat_invoke("oem.abort_all", request("aggregate-overlap-1"))
        )
        second = asyncio.create_task(
            plane.compat_invoke("oem.z.abort", request("aggregate-overlap-2"))
        )
        await asyncio.gather(*(event.wait() for event in started.values()))
        release["oem.abort_all"].set()
        await first
        assert plane.store.action_fenced("oem.x.move_steps") is True
        release["oem.z.abort"].set()
        await second
        assert plane.store.action_fenced("oem.x.move_steps") is False

    asyncio.run(exercise())


def test_interrupt_finalization_failure_retains_fence_for_reconciliation(
    tmp_path, monkeypatch,
):
    app, _ = make_app(tmp_path, monkeypatch)
    plane = app.state.operator_command_plane

    async def deliver(_action_id, *, interrupt_attempt_id):
        return 200, {
            "ok": True,
            "source_call_completed": True,
            "controller_command_acknowledged": True,
        }

    monkeypatch.setattr(plane, "_deliver_controller_interrupt_raw", deliver)
    monkeypatch.setattr(
        plane.store, "finalize_interrupt",
        lambda **_kwargs: (_ for _ in ()).throw(sqlite3.OperationalError("locked")),
    )
    response = asyncio.run(
        plane.compat_invoke(
            "oem.abort_all",
            {
                "schema_version": "bioxp.operator_interrupt_request.v1",
                "idempotency_key": "finalization-fence-hold-1",
                "reason": "test",
                "observed_ownership_generation": 7,
                "observed_board_epoch_by_board": {"4": 10, "5": 11},
            },
        )
    )

    assert response["recovery_hold"] is True
    assert plane.store.action_fenced("oem.x.move_steps") is True


def test_provider_interrupt_persistence_failure_retains_outer_fence(
    tmp_path, monkeypatch,
):
    app, _ = make_app(tmp_path, monkeypatch)
    plane = app.state.operator_command_plane
    finalized: list[dict] = []

    async def deliver(_action_id, *, interrupt_attempt_id):
        return 200, {
            "ok": True,
            "source_call_completed": True,
            "source_return_ok": True,
            "controller_command_acknowledged": True,
            "persistence_state": "recovery_required",
            "recovery_hold": True,
            "error": "interrupt_persistence_failed:OperationalError",
        }

    monkeypatch.setattr(plane, "_deliver_controller_interrupt_raw", deliver)
    monkeypatch.setattr(
        plane.store, "finalize_interrupt",
        lambda **kwargs: finalized.append(dict(kwargs)) or {},
    )
    response = asyncio.run(
        plane.compat_invoke(
            "oem.abort_all",
            {
                "schema_version": "bioxp.operator_interrupt_request.v1",
                "idempotency_key": "provider-persistence-fence-hold-1",
                "reason": "test",
                "observed_ownership_generation": 7,
                "observed_board_epoch_by_board": {"4": 10, "5": 11},
            },
        )
    )

    assert finalized == []
    assert response["recovery_hold"] is True
    assert plane.store.action_fenced("oem.x.move_steps") is True


def test_interrupt_waits_for_affected_workers_before_parent_terminalization(
    tmp_path, monkeypatch,
):
    app, _ = make_app(tmp_path, monkeypatch)
    plane = app.state.operator_command_plane
    receipt = {
        "schema_version": "bioxp.operator_interrupt_receipt.v1",
        "interrupt_id": "interrupt-worker-drain",
        "interrupt_attempt_id": "interrupt-worker-drain",
        "action_id": "oem.abort_all",
        "active_command_id": "worker-command",
        "active_command_ids": ["worker-command"],
        "controller_stop_attempted": False,
        "persistence_state": "committed",
        "recovery_hold": True,
    }
    waits: list[tuple[list[str], float]] = []
    queued: list[dict] = []

    monkeypatch.setattr(plane.store, "begin_interrupt", lambda *_args, **_kwargs: dict(receipt))
    monkeypatch.setattr(
        plane.store, "mark_interrupt_attempted",
        lambda **_kwargs: {**receipt, "controller_stop_attempted": True},
    )
    monkeypatch.setattr(
        plane.store, "wait_for_command_workers",
        lambda command_ids, timeout: waits.append((list(command_ids), timeout)) or False,
        raising=False,
    )
    monkeypatch.setattr(
        plane.store, "finalize_interrupt",
        lambda **_kwargs: (_ for _ in ()).throw(AssertionError("finalized before drain")),
    )
    monkeypatch.setattr(
        plane.store, "_append_interrupt_spool_event",
        lambda **_kwargs: (_ for _ in ()).throw(sqlite3.OperationalError("spool locked")),
    )
    monkeypatch.setattr(
        plane.store, "queue_pending_interrupt_reconciliation",
        lambda row: queued.append(dict(row)),
    )

    async def deliver(_action_id, *, interrupt_attempt_id):
        return 200, {
            "ok": True,
            "source_call_completed": True,
            "controller_command_acknowledged": True,
        }

    monkeypatch.setattr(plane, "_deliver_controller_interrupt_raw", deliver)
    response = asyncio.run(
        plane.compat_invoke(
            "oem.abort_all",
            {
                "schema_version": "bioxp.operator_interrupt_request.v1",
                "idempotency_key": "worker-drain-interrupt-1",
                "reason": "test",
                "observed_ownership_generation": 7,
                "observed_board_epoch_by_board": {"4": 10, "5": 11},
            },
        )
    )

    assert waits and waits[0][0] == ["worker-command"]
    assert queued
    assert response["recovery_hold"] is True
    assert response["error"] == "interrupt_worker_drain_timeout"
    assert plane.store.action_fenced("oem.x.move_steps") is True


def test_y_pending_terminalizer_is_registered_with_its_command_identity(monkeypatch):
    class Store:
        def __init__(self):
            self.started = []

        def mark_dispatched(self, command_id, **_kwargs):
            assert command_id == "y-pending-command"
            return {"status": "issued_pending", "command_version": 2}

        def action_fenced(self, _action_id):
            return False

        def _start_command_worker(self, worker, command_id):
            self.started.append((worker, command_id))

    async def pending_response(*_args, **_kwargs):
        return 200, {"ok": True, "state": "issued_pending"}

    monkeypatch.setattr(operator_command_plane, "_dispatch_asgi", pending_response)
    plane = OperatorCommandPlane.__new__(OperatorCommandPlane)
    plane.app = FastAPI()
    plane.store = Store()
    plane.machine_state_provider = lambda: {"ownership_generation": 7}
    plane.dispatch = {
        "oem.y.move_absolute": {
            "method": "POST", "path": "/y/move-absolute",
            "locations": {}, "fixed_inputs": {},
        }
    }
    plane._current_assessment = lambda *_args: {"enabled": True}
    plane._require_enabled_assessment = lambda *_args: None

    plane._dispatch_one(
        {
            "command_id": "y-pending-command",
            "action_id": "oem.y.move_absolute",
            "requested_inputs": {"target_steps": 123},
            "effective_inputs": {"target_steps": 123},
            "ownership_generation": 7,
            "expected_board_epoch_by_board": {"4": 10},
            "dispatch_attempt_id": "y-pending-attempt",
        }
    )

    assert len(plane.store.started) == 1
    worker, command_id = plane.store.started[0]
    assert command_id == "y-pending-command"
    assert worker.name == "bioxp-y-terminalizer-y-pending-command"


def test_y_pending_terminalizer_requires_refreshed_pending_claim() -> None:
    class Store:
        def __init__(self):
            self.finished = []
            self._worker_lock = threading.Lock()
            self._workers = set()
            self._worker_commands = {}

        def finish(self, command_id, **kwargs):
            self.finished.append((command_id, kwargs))

    plane = OperatorCommandPlane.__new__(OperatorCommandPlane)
    plane.app = FastAPI()
    plane.app.state.serial206_y_terminalizer = lambda *_args: {
        "ok": True,
        "controller_command_acknowledged": True,
        "completion_class": "event_128",
        "terminal_speed_zero": True,
    }
    plane.store = Store()
    claimed = {
        "command_id": "y-terminal-claim",
        "action_id": "oem.y.move_absolute",
        "dispatch_attempt_id": "attempt-y-terminal-claim",
        "ownership_generation": 7,
        "command_status": "issued_pending",
        "command_version": 2,
    }

    plane._terminalize_y_pending(
        "y-terminal-claim", {"target_steps": 123}, claimed,
    )

    assert plane.store.finished[0][0] == "y-terminal-claim"
    assert plane.store.finished[0][1]["claimed"] == claimed


def test_y_pending_claim_can_terminalize_issued_pending_command(
    tmp_path, monkeypatch,
) -> None:
    app, _ = make_app(tmp_path, monkeypatch)
    plane = app.state.operator_command_plane
    state = plane._state()
    admitted = plane.store.admit_command(
        {
            "schema_version": "bioxp.operator_action_request.v2",
            "action_id": "oem.y.move_absolute",
            "inputs": {"target_steps": 100},
            "expected_ownership_generation": 7,
            "idempotency_key": "y-pending-terminal-claim-1",
            "expected_board_epoch_by_board": {"4": 10},
        },
        state=state,
    )
    claimed = plane.store.claim_next()
    assert claimed is not None
    pending = plane.store.mark_dispatched(
        claimed["command_id"], payload={"pending": True}
    )
    pending_claim = {
        **claimed,
        "command_status": "issued_pending",
        "command_version": pending["command_version"],
    }
    stale_finish = plane.store.finish(
        admitted["command_id"], status="completed",
        payload={"completion_class": "event_128"}, claimed=claimed,
    )
    assert stale_finish["status"] == "issued_pending"

    finished = plane.store.finish(
        admitted["command_id"], status="completed",
        payload={"completion_class": "event_128"}, claimed=pending_claim,
    )

    assert finished["status"] == "completed"


def test_v2_strict_method_rechecks_board_epochs_before_dispatch(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    app.state.operator_test_serial206["board4_authority"] = {"active_board_epoch": 10}
    app.state.operator_test_serial206["x_authority"]["active_board_epoch"] = 11
    client = TestClient(app)
    admitted = client.post(
        "/operator/v2/methods",
        json={
            "schema_version": "bioxp.operator_method_request.v1",
            "method_action_id": "oem.xy.move_absolute",
            "idempotency_key": "v2-strict-dispatch-fence-1",
            "expected_ownership_generation": 7,
            "expected_board_epoch_by_board": {"4": 10, "5": 11},
            "inputs": {"x_steps": 200, "y_steps": 300},
        },
    )
    assert admitted.status_code == 200, admitted.text
    method_id = admitted.json()["method_id"]

    app.state.operator_test_serial206["x_authority"]["active_board_epoch"] = 12
    plane = app.state.operator_command_plane
    body: dict = {}
    plane.start()
    try:
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            body = client.get(f"/operator/v2/methods/{method_id}").json()
            if body["status"] in {"completed", "failed", "interrupted", "ambiguous"}:
                break
            time.sleep(0.01)
    finally:
        plane.stop()

    assert body["status"] == "failed"
    assert body["child_receipts"][0]["status"] == "failed"
    assert all(call[0] != "move_xy" for call in calls)


def test_v2_catalog_and_dispatch_retain_exact_oem_activation_and_recovery(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    client = TestClient(app)

    catalog = client.get("/operator/v2/control-catalog").json()
    by_id = {row["action_id"]: row for row in catalog["actions"]}
    assert list(row["action_id"] for row in catalog["actions"]).count("meta.activate_motion") == 1
    assert list(row["action_id"] for row in catalog["actions"]).count("meta.recover_motion_non_homing") == 1
    assert "oem.xy.enable" not in by_id
    assert by_id["meta.activate_motion"]["enabled"] is True
    assert by_id["meta.recover_motion_non_homing"]["enabled"] is False
    assert by_id["meta.recover_motion_non_homing"]["disabled_reason"] == "Non-homing recovery is not currently required."

    generation = catalog["dashboard"]["ownership_generation"]
    activation = client.post(
        "/operator/v2/actions/meta.activate_motion",
        json={
            "schema_version": "bioxp.operator_action_request.v2",
            "expected_ownership_generation": generation,
            "expected_board_epoch_by_board": {},
            "idempotency_key": "activate-motion-v2-123456",
            "inputs": {},
        },
    )
    assert activation.status_code == 200, activation.text
    assert calls == [("prepare_without_motion", None)]

    app.state.operator_test_maintenance.update({
        "motion_blocked": True,
        "recovery_required": True,
        "block_reason": "injected maintenance transition",
    })
    recovery_catalog = client.get("/operator/v2/control-catalog").json()
    recovery_action = next(
        row for row in recovery_catalog["actions"]
        if row["action_id"] == "meta.recover_motion_non_homing"
    )
    assert recovery_action["enabled"] is True
    recovery = client.post(
        "/operator/v2/actions/meta.recover_motion_non_homing",
        json={
            "schema_version": "bioxp.operator_action_request.v2",
            "expected_ownership_generation": generation,
            "expected_board_epoch_by_board": {},
            "idempotency_key": "recover-motion-v2-123456",
            "inputs": {},
        },
    )
    assert recovery.status_code == 200, recovery.text
    assert calls[-1] == (
        "recover_motion_non_homing",
        {
            "run_homing": False,
            "operator_ack": "RECOVER_MOTION",
            "operator_reason": "operator control invocation",
        },
    )


def test_v2_failed_receipt_preserves_bounded_home_z_provider_detail(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    client = TestClient(app)

    async def rejected_home(*_args, **_kwargs):
        return 409, {
            "detail": {
                "result": {
                    "failure": "z_manual_home_evidence_not_verified",
                    "controller_command_acknowledged": False,
                    "controller_terminal_state_verified": False,
                    "physical_effect_verified": False,
                    "home": {
                        "home": {
                            "axis": "z",
                            "board": 4,
                            "motor": 1,
                            "failure": "board_not_initialized",
                            "source_return_code": 1,
                            "physical_effect_verified": False,
                        },
                    },
                },
                "z_lifecycle": {
                    "state": "failed_latched",
                    "reference_state": "desynced",
                },
            },
        }

    monkeypatch.setattr(operator_controls, "_dispatch_asgi", rejected_home)
    catalog = client.get("/operator/v2/control-catalog").json()
    response = client.post(
        "/operator/v2/actions/oem.z.manual_home",
        json={
            "schema_version": "bioxp.operator_action_request.v2",
            "expected_ownership_generation": catalog["dashboard"]["ownership_generation"],
            "expected_board_epoch_by_board": {},
            "idempotency_key": "home-z-provider-detail-v2",
            "inputs": {},
        },
    )

    assert response.status_code == 200, response.text
    expected_error = {
        "code": "robot route returned HTTP 409",
        "message": "robot route returned HTTP 409",
        "retryable": False,
        "detail": {
            "provider_failure": "z_manual_home_evidence_not_verified",
            "failure": "board_not_initialized",
            "axis": "z",
            "board": 4,
            "motor": 1,
            "source_return_code": 1,
            "controller_acknowledged": False,
            "controller_terminal_state_verified": False,
            "physical_effect_verified": False,
            "lifecycle_state": "failed_latched",
            "reference_state": "desynced",
        },
    }
    assert response.json()["error"] == expected_error
    command_id = response.json()["command_id"]
    polled = client.get(f"/operator/v2/actions/receipts/{command_id}")
    assert polled.status_code == 200, polled.text
    assert polled.json()["error"] == expected_error


def test_v2_canonical_discovery_is_semantic_instead_of_a_handwritten_allowlist():
    assert not hasattr(operator_controls, "ALLOWED_ACTIONS")
    assert operator_controls._is_v2_canonical_action({
        "action_id": "oem.future.literal_oem_capability",
        "kind": "semantic",
        "category": "future-oem",
    }) is True
    assert operator_controls._is_v2_canonical_action({
        "action_id": "meta.future_recovery",
        "kind": "meta",
        "category": "recovery",
    }) is True
    assert operator_controls._is_v2_canonical_action({
        "action_id": "meta.internal_initialization",
        "kind": "meta",
        "category": "initialization",
    }) is False
    for action in (
        {
            "action_id": "oem.xy.enable",
            "kind": "semantic",
            "category": "motion",
            "informational_path": "/motion/oem/x/internal/enable_xy",
        },
        {
            "action_id": "oem.y.internal.set_sg_threshold",
            "kind": "semantic",
            "category": "motion",
            "informational_path": "/motion/oem/y/internal/set_sg_threshold",
        },
        {
            "action_id": "oem.xy.home_xy",
            "kind": "semantic",
            "category": "motion",
            "informational_path": "/motion/oem/home_xy",
        },
    ):
        assert operator_controls._is_v2_canonical_action(action) is False


def test_v2_canonical_discovery_deduplicates_generic_routes_without_hiding_fixed_only_capabilities():
    actions = [
        {"action_id": "oem.z.control", "informational_path": "/motion/oem/z/control"},
        {"action_id": "oem.z.set_max_speed", "informational_path": "/motion/oem/z/control"},
        {"action_id": "oem.z.restore_original_speed", "informational_path": "/motion/oem/z/control"},
        {"action_id": "oem.x.home", "informational_path": "/motion/oem/manual/home"},
        {"action_id": "oem.y.home", "informational_path": "/motion/oem/manual/home"},
    ]
    dispatch = {
        "oem.z.control": {"fixed_inputs": {}},
        "oem.z.set_max_speed": {"fixed_inputs": {"operation": "set_max_speed"}},
        "oem.z.restore_original_speed": {"fixed_inputs": {"operation": "restore_original_speed"}},
        "oem.x.home": {"fixed_inputs": {"axis": "x"}},
        "oem.y.home": {"fixed_inputs": {"axis": "y"}},
    }

    assert operator_controls._v2_canonical_action_ids(actions, dispatch) == frozenset({
        "oem.z.control",
        "oem.x.home",
        "oem.y.home",
    })


def test_operator_ack_is_not_user_input_and_is_supplied_internally(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    action = action_for(catalog, "POST", "/motion/gripper/clear")
    assert all(row["name"] != "operator_ack" for row in action["inputs"])
    response = client.post(
        f"/operator/actions/{action['action_id']}",
        json={
            "expected_generation": catalog["ownership_generation"],
            "idempotency_key": "implicit-ack-gripper-clear",
            "inputs": {},
        },
    )
    assert response.status_code == 200, response.text
    assert calls == [("gripper_clear", {"operator_ack": "GRIPPER_CLEAR"})]


def test_local_only_maintenance_action_is_listed_but_public_operator_invocation_cannot_dispatch(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    local = action_for(catalog, "POST", "/maintenance/usb/reconnect")

    assert local["provider_available"] is False
    assert local["available"] is False
    assert local["enabled"] is False
    assert local["disabled_reason"] == "Local-only maintenance route is not callable through the operator relay."

    response = client.post(
        f"/operator/actions/{local['action_id']}",
        json={
            "expected_generation": catalog["ownership_generation"],
            "idempotency_key": "local-route-must-not-dispatch",
            "inputs": {},
        },
    )
    assert response.status_code == 409
    assert response.json()["detail"]["reason"] == "Local-only maintenance route is not callable through the operator relay."
    assert calls == []


def test_primitive_invocation_dispatches_exactly_one_catalog_route_and_persists_receipt(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    generation = catalog["ownership_generation"]
    action = action_for(catalog, "POST", "/motion/test/home_axis")
    payload = {"expected_generation": generation, "idempotency_key": "home-axis-123456", "inputs": {"body": {"axis": "x"}}}
    response = client.post(f"/operator/actions/{action['action_id']}", json=payload)
    assert response.status_code == 200, response.text
    receipt = response.json()
    assert calls == [("home_axis", {"axis": "x"})]
    assert receipt["status"] == "completed"
    assert receipt["machine_assessment"] == "pass"
    assert receipt["physical_effect_verified"] is False
    assert receipt["stage_receipts"] == []
    assert (
        receipt["request_received_at"]
        <= receipt["lock_acquired_at"]
        <= receipt["admission_completed_at"]
        <= receipt["provider_entry_at"]
        <= receipt["provider_returned_at"]
        <= receipt["receipt_persist_started_at"]
    )
    stored = OperatorReceiptStore(tmp_path).by_command(receipt["command_id"])
    assert stored is not None
    assert stored["command_id"] == receipt["command_id"]
    assert stored["response"]["body"] == {
        "ok": True,
        "controller_acknowledged": True,
        "stages": [{"stage_id": "home", "status": "passed"}],
    }
    detailed = client.get(
        f"/operator/actions/receipts/{receipt['command_id']}?detail=true"
    ).json()
    assert detailed["response"]["body"]["stages"] == [
        {"stage_id": "home", "status": "passed"}
    ]

    replay = client.post(f"/operator/actions/{action['action_id']}", json=payload)
    assert replay.status_code == 200
    assert replay.json()["command_id"] == receipt["command_id"]
    assert calls == [("home_axis", {"axis": "x"})]


def test_operator_receipt_preserves_inner_completion_ambiguity(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    action = action_for(catalog, "POST", "/motion/test/home_axis")

    async def ambiguous_dispatch(*_args, **_kwargs):
        return 504, {
            "detail": {
                "error": "tester_operation_completion_ambiguous",
                "completion_ambiguous": True,
                "outcome_unknown": True,
                "reconciliation_required": True,
                "retry_forbidden": True,
            }
        }

    monkeypatch.setattr(operator_controls, "_dispatch_asgi", ambiguous_dispatch)
    response = client.post(
        f"/operator/actions/{action['action_id']}",
        json={
            "expected_generation": catalog["ownership_generation"],
            "idempotency_key": "operator-ambiguous-pipette-1",
            "inputs": {"body": {"axis": "x"}},
        },
    )
    assert response.status_code == 200
    receipt = response.json()
    assert receipt["status"] == "outcome_unknown"
    assert receipt["reconciliation_required"] is True
    assert receipt["retry_forbidden"] is True
    assert calls == []


def test_operator_receipt_preserves_explicit_no_command_ack_over_nested_readback_acks(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    app.state.operator_test_home_axis_response = {
        "ok": True,
        "result": {
            "source_noop": True,
            "physical_motion_commanded": False,
            "wait": {"last_ack": {"status": 100, "command": 6}},
            "after": {"ack": {"status": 100, "command": 6}},
        },
        "authority_receipt": {
            "command_id": "serial206-noop-command",
            "status": "completed",
            "controller_command_acknowledged": False,
            "controller_terminal_state_verified": True,
        },
    }
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    action = action_for(catalog, "POST", "/motion/test/home_axis")

    response = client.post(
        f"/operator/actions/{action['action_id']}",
        json={
            "expected_generation": catalog["ownership_generation"],
            "idempotency_key": "verified-no-command-ack-123",
            "inputs": {"body": {"axis": "x"}},
        },
    )

    assert response.status_code == 200, response.text
    receipt = response.json()
    assert receipt["status"] == "completed"
    assert receipt["controller_acknowledged"] is False
    assert receipt["authority_receipt_status"] == "completed"
    stored = OperatorReceiptStore().by_command(receipt["command_id"])
    assert stored is not None
    assert stored["controller_acknowledged"] is False
    assert calls == [("home_axis", {"axis": "x"})]


def test_idempotent_replay_returns_durable_receipt_before_mutable_admission(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    action = action_for(catalog, "POST", "/motion/test/home_axis")
    payload = {
        "expected_generation": catalog["ownership_generation"],
        "idempotency_key": "fresh-admission-replay-123",
        "inputs": {"body": {"axis": "x"}},
    }

    first = client.post(f"/operator/actions/{action['action_id']}", json=payload)
    assert first.status_code == 200
    app.state.operator_test_maintenance.update({
        "motion_blocked": True,
        "recovery_required": True,
        "block_reason": "injected current maintenance block",
    })
    replay = client.post(f"/operator/actions/{action['action_id']}", json=payload)

    assert replay.status_code == 200
    assert replay.json()["command_id"] == first.json()["command_id"]
    assert calls == [("home_axis", {"axis": "x"})]


def test_idempotent_replay_ignores_mutable_authority_fingerprint_without_redispatch(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    action = action_for(catalog, "POST", "/motion/test/home_axis")
    payload = {
        "expected_generation": catalog["ownership_generation"],
        "idempotency_key": "authority-fingerprint-replay-123",
        "inputs": {"body": {"axis": "x"}},
    }

    first = client.post(f"/operator/actions/{action['action_id']}", json=payload)
    assert first.status_code == 200
    app.state.operator_test_lifecycle["operation_reason"] = "current lifecycle authority changed"
    replay = client.post(f"/operator/actions/{action['action_id']}", json=payload)

    assert replay.status_code == 200
    assert replay.json()["command_id"] == first.json()["command_id"]
    assert calls == [("home_axis", {"axis": "x"})]


def test_post_lock_idempotency_race_rejects_stale_release_identity(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    action = action_for(catalog, "POST", "/motion/test/home_axis")
    payload = {
        "expected_generation": catalog["ownership_generation"],
        "idempotency_key": "post-lock-stale-release-race-123",
        "inputs": {"body": {"axis": "x"}},
    }

    first = client.post(f"/operator/actions/{action['action_id']}", json=payload)
    assert first.status_code == 200

    store = app.state.operator_receipt_store
    original_by_idempotency = store.by_idempotency
    lookup_count = 0
    lookup_lock = threading.Lock()

    def miss_then_find(*args, **kwargs):
        nonlocal lookup_count
        with lookup_lock:
            lookup_count += 1
            current_lookup = lookup_count
        if current_lookup == 1:
            return None
        return original_by_idempotency(*args, **kwargs)

    monkeypatch.setattr(store, "by_idempotency", miss_then_find)
    monkeypatch.setattr(operator_controls, "current_release_identity", lambda: {
        "verified": True,
        "release_id": "replacement-test-release",
        "source": {"manifest_sha256": "1" * 64, "aggregate_sha256": "2" * 64},
    })

    replay = client.post(f"/operator/actions/{action['action_id']}", json=payload)

    assert replay.status_code == 409
    assert replay.json()["detail"] == "idempotency receipt release_id is stale"
    assert lookup_count == 2
    assert calls == [("home_axis", {"axis": "x"})]


def test_idempotent_replay_is_bound_to_ownership_generation(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    action = action_for(catalog, "POST", "/motion/test/home_axis")
    payload = {
        "expected_generation": catalog["ownership_generation"],
        "idempotency_key": "generation-bound-replay-123",
        "inputs": {"body": {"axis": "x"}},
    }
    first = client.post(f"/operator/actions/{action['action_id']}", json=payload)
    assert first.status_code == 200
    app.state.operator_test_hardware.ownership_epoch += 1
    replay = client.post(
        f"/operator/actions/{action['action_id']}",
        json={**payload, "expected_generation": payload["expected_generation"] + 1},
    )
    assert replay.status_code == 409
    assert replay.json()["detail"] == "idempotency receipt ownership generation mismatch"
    assert calls == [("home_axis", {"axis": "x"})]


def test_exact_replay_uses_stored_generation_before_current_ownership(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    action = action_for(catalog, "POST", "/motion/test/home_axis")
    payload = {
        "expected_generation": catalog["ownership_generation"],
        "idempotency_key": "stored-generation-replay-123",
        "inputs": {"body": {"axis": "x"}},
    }
    first = client.post(f"/operator/actions/{action['action_id']}", json=payload)
    assert first.status_code == 200
    app.state.operator_test_hardware.ownership_epoch += 1

    replay = client.post(f"/operator/actions/{action['action_id']}", json=payload)

    assert replay.status_code == 200
    assert replay.json()["command_id"] == first.json()["command_id"]
    assert calls == [("home_axis", {"axis": "x"})]


def test_motion_admission_is_recomputed_after_waiting_for_invocation_lock(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    catalog = TestClient(app).get("/operator/control-catalog").json()
    action = action_for(catalog, "POST", "/motion/test/home_axis")
    original_dispatch = operator_controls._dispatch_asgi

    async def scenario() -> tuple[Response, Response]:
        entered_dispatch = asyncio.Event()
        release_dispatch = asyncio.Event()

        async def delayed_dispatch(*args, **kwargs):
            if not entered_dispatch.is_set():
                entered_dispatch.set()
                await asyncio.wait_for(release_dispatch.wait(), timeout=3.0)
            return await original_dispatch(*args, **kwargs)

        monkeypatch.setattr(operator_controls, "_dispatch_asgi", delayed_dispatch)
        transport = ASGITransport(app=app)
        async with AsyncClient(transport=transport, base_url="http://testserver") as client:
            def payload(key: str) -> dict:
                return {
                    "expected_generation": catalog["ownership_generation"],
                    "idempotency_key": key,
                    "inputs": {"body": {"axis": "x"}},
                }

            first_task = asyncio.create_task(
                client.post(f"/operator/actions/{action['action_id']}", json=payload("lock-race-first-123"))
            )
            await asyncio.wait_for(entered_dispatch.wait(), timeout=3.0)
            second_task = asyncio.create_task(
                client.post(f"/operator/actions/{action['action_id']}", json=payload("lock-race-second-123"))
            )
            await asyncio.sleep(0)
            app.state.operator_test_maintenance.update(
                motion_blocked=True,
                recovery_required=False,
                block_reason="Motion is inactive. Activate motion before moving this motor.",
            )
            release_dispatch.set()
            return await asyncio.wait_for(
                asyncio.gather(first_task, second_task), timeout=3.0
            )

    first, second = asyncio.run(scenario())
    assert first.status_code == 200
    assert second.status_code == 409
    assert second.json()["detail"]["reason"] == "Motion is inactive. Activate motion before moving this motor."
    assert calls == [("home_axis", {"axis": "x"})]


def test_emergency_stop_dispatch_bypasses_blocked_normal_database_claim(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    catalog = TestClient(app).get("/operator/control-catalog").json()
    normal = action_for(catalog, "POST", "/motion/test/home_axis")
    original_claim = OperatorReceiptStore.claim
    claim_entered = threading.Event()
    release_claim = threading.Event()

    def blocked_claim(store, receipt):
        if receipt.get("action_id") == normal["action_id"]:
            claim_entered.set()
            assert release_claim.wait(timeout=5)
        return original_claim(store, receipt)

    monkeypatch.setattr(OperatorReceiptStore, "claim", blocked_claim)

    async def scenario():
        transport = ASGITransport(app=app)
        async with AsyncClient(transport=transport, base_url="http://testserver") as client:
            normal_task = asyncio.create_task(
                client.post(
                    f"/operator/actions/{normal['action_id']}",
                    json={
                        "expected_generation": catalog["ownership_generation"],
                        "idempotency_key": "blocked-normal-claim",
                        "inputs": {"body": {"axis": "x"}},
                    },
                )
            )
            assert await asyncio.to_thread(claim_entered.wait, 3)
            stop = await asyncio.wait_for(
                client.post(
                    "/operator/actions/oem.abort_all",
                    json={
                        "expected_generation": catalog["ownership_generation"],
                        "idempotency_key": "interrupt-bypass-claim",
                        "inputs": {},
                    },
                ),
                timeout=3,
            )
            assert stop.status_code == 200, stop.text
            assert calls == [("abort_all", None)]
            release_claim.set()
            return await asyncio.wait_for(normal_task, timeout=3)

    normal_response = asyncio.run(scenario())
    assert normal_response.status_code == 200
    assert calls == [("abort_all", None), ("home_axis", {"axis": "x"})]


def test_repeated_emergency_stop_key_dispatches_and_persists_each_receipt(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    payload = {
        "expected_generation": catalog["ownership_generation"],
        "idempotency_key": "repeated-emergency-stop",
        "inputs": {},
    }

    first = client.post("/operator/actions/oem.abort_all", json=payload)
    second = client.post("/operator/actions/oem.abort_all", json=payload)

    assert first.status_code == 200, first.text
    assert second.status_code == 200, second.text
    assert first.json()["command_id"] != second.json()["command_id"]
    assert first.json()["idempotency_replay_enabled"] is False
    assert second.json()["idempotency_replay_enabled"] is False
    assert calls == [("abort_all", None), ("abort_all", None)]
    store = OperatorReceiptStore(tmp_path)
    assert store.connection.execute(
        "SELECT COUNT(*) FROM operator_commands WHERE idempotency_key=?",
        (payload["idempotency_key"],),
    ).fetchone()[0] == 2
    assert store.by_idempotency(payload["idempotency_key"], include_evidence=False) is None


def test_raw_emergency_route_is_nonreplayable_interrupt(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    action = action_for(catalog, "POST", "/motion/oem/x/abort")
    payload = {
        "expected_generation": catalog["ownership_generation"],
        "idempotency_key": "raw-repeated-emergency-stop",
        "inputs": {},
    }

    first = client.post(f"/operator/actions/{action['action_id']}", json=payload)
    second = client.post(f"/operator/actions/{action['action_id']}", json=payload)

    assert first.status_code == 200
    assert second.status_code == 200
    assert first.json()["command_id"] != second.json()["command_id"]
    assert calls == [("abort_all", None), ("abort_all", None)]


def test_raw_diagnostic_stop_is_nonreplayable_interrupt(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    action = action_for(catalog, "POST", "/motion/diagnostics/stop")
    payload = {
        "expected_generation": catalog["ownership_generation"],
        "idempotency_key": "raw-repeated-diagnostic-stop",
        "inputs": {},
    }

    first = client.post(f"/operator/actions/{action['action_id']}", json=payload)
    second = client.post(f"/operator/actions/{action['action_id']}", json=payload)

    assert first.status_code == 200
    assert second.status_code == 200
    assert first.json()["command_id"] != second.json()["command_id"]
    assert calls == [("diagnostic_stop", None), ("diagnostic_stop", None)]


def test_path_and_query_inputs_are_bound_from_catalog_not_browser_routes(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    action = action_for(catalog, "GET", "/motion/axis/{axis}/status")
    inputs = {row["name"]: row for row in action["inputs"]}
    assert inputs["axis"]["location"] == "path"
    assert inputs["verbose"]["location"] == "query"
    response = client.post(f"/operator/actions/{action['action_id']}", json={
        "expected_generation": catalog["ownership_generation"],
        "idempotency_key": "axis-status-123456",
        "inputs": {"axis": "y", "verbose": True},
    })
    assert response.status_code == 200, response.text
    assert calls == [("axis_status", {"axis": "y", "verbose": True})]
    receipt = response.json()
    assert receipt["requested_inputs"] == {"axis": "y", "verbose": True}
    assert receipt["response"]["body"] == {"ok": True}
    detailed = client.get(
        f"/operator/actions/receipts/{receipt['command_id']}?detail=true"
    ).json()
    assert detailed["response"]["body"]["axis"] == "y"


def test_normalized_field_keeps_original_body_wire_name(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    action = action_for(catalog, "POST", "/motion/camel")
    target = next(row for row in action["inputs"] if row["wire_name"] == "targetPosition")
    response = client.post(f"/operator/actions/{action['action_id']}", json={
        "expected_generation": catalog["ownership_generation"],
        "idempotency_key": "camel-wire-123456",
        "inputs": {target["name"]: 123},
    })
    assert response.status_code == 200, response.text
    assert calls == [("camel", {"targetPosition": 123})]


def test_catalog_preserves_inclusive_and_exclusive_numeric_bounds(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    catalog = TestClient(app).get("/operator/control-catalog").json()
    bounded = action_for(catalog, "POST", "/motion/bounded")
    inputs = {row["wire_name"]: row for row in bounded["inputs"]}
    assert inputs["integer_gt"]["minimum"] is None
    assert inputs["integer_gt"]["exclusive_minimum"] == 0
    assert inputs["integer_gt"]["maximum"] == 10
    assert inputs["integer_gt"]["exclusive_maximum"] is None
    assert inputs["number_lt"]["minimum"] == -2.5
    assert inputs["number_lt"]["exclusive_minimum"] is None
    assert inputs["number_lt"]["maximum"] is None
    assert inputs["number_lt"]["exclusive_maximum"] == 2.5


def test_public_motion_request_models_use_signed_int32_at_robot_boundaries(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    from bioxp.api import (
        MoveAbsoluteRequest,
        MoveRelativeRequest,
        OemManualAbsoluteRequest,
        OemManualRelativeRequest,
        OemHomeXYRequest,
        OemMoveXYRequest,
        OemYMoveAbsoluteRequest,
        OemYMoveStepsRequest,
        ReferenceMarkRequest,
    )

    relative = MoveRelativeRequest.model_json_schema()["properties"]["steps"]
    absolute = MoveAbsoluteRequest.model_json_schema()["properties"]["position_steps"]
    manual_relative = OemManualRelativeRequest.model_json_schema()["properties"]["steps"]
    manual_absolute = OemManualAbsoluteRequest.model_json_schema()["properties"]["position_steps"]
    reference = ReferenceMarkRequest.model_json_schema()["properties"]["position_steps"]
    for schema in (relative, absolute, reference):
        assert "minimum" not in schema
        assert "maximum" not in schema
    for schema in (manual_relative, manual_absolute):
        assert (schema["minimum"], schema["maximum"]) == (-(2**31), 2**31 - 1)
    for model, field in (
        (OemManualRelativeRequest, "steps"),
        (OemManualAbsoluteRequest, "position_steps"),
        (OemYMoveStepsRequest, "steps"),
        (OemYMoveAbsoluteRequest, "target_steps"),
    ):
        payload = {"axis": "x", field: 1} if model in {OemManualRelativeRequest, OemManualAbsoluteRequest} else {field: 1}
        model.model_validate(payload)
        with pytest.raises(Exception):
            model.model_validate({**payload, field: str(payload[field])})
        with pytest.raises(Exception):
            model.model_validate({**payload, field: 2**31})
    OemMoveXYRequest.model_validate({"operator_ack": "MOVEXY", "x": 0, "y": 0})
    with pytest.raises(Exception):
        OemMoveXYRequest.model_validate({"operator_ack": "MOVEXY", "x": -(2**31) - 1, "y": 0})
    with pytest.raises(Exception):
        OemHomeXYRequest.model_validate({"operator_ack": "HOMEXY", "unexpected": True})
    with pytest.raises(Exception):
        OemHomeXYRequest.model_validate({"operator_ack": "HOMEXY", "timeout_s": 120.0})
    with pytest.raises(Exception):
        OemHomeXYRequest.model_validate({"operator_ack": "HOMEXY", "allow_implementation_mapped_predicate": True})


def test_z_semantic_and_manual_moves_expose_signed_int32_bounds(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    from bioxp.api import app

    actions, _ = operator_controls._build_catalog(app)
    by_id = {action["action_id"]: action for action in actions}

    relative = next(row for row in by_id["oem.z.move_steps"]["inputs"] if row["name"] == "steps")
    absolute = next(row for row in by_id["oem.z.move_absolute"]["inputs"] if row["name"] == "position_steps")
    assert (relative["minimum"], relative["maximum"]) == (-(2**31), 2**31 - 1)
    assert (absolute["minimum"], absolute["maximum"]) == (-(2**31), 2**31 - 1)

    generic_relative = next(
        action for action in actions
        if action["kind"] == "primitive" and action["informational_path"] == "/motion/oem/manual/relative"
    )
    generic_absolute = next(
        action for action in actions
        if action["kind"] == "primitive" and action["informational_path"] == "/motion/oem/manual/absolute"
    )
    generic_steps = next(row for row in generic_relative["inputs"] if row["name"] == "steps")
    generic_position = next(row for row in generic_absolute["inputs"] if row["name"] == "position_steps")
    assert (generic_steps["minimum"], generic_steps["maximum"]) == (-(2**31), 2**31 - 1)
    assert (generic_position["minimum"], generic_position["maximum"]) == (-(2**31), 2**31 - 1)


def test_unknown_inputs_generation_mismatch_and_disabled_meta_fail_without_dispatch(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    action = action_for(catalog, "POST", "/motion/test/home_axis")
    base = {"expected_generation": catalog["ownership_generation"], "idempotency_key": "reject-action-1234"}
    assert client.post(f"/operator/actions/{action['action_id']}", json={**base, "expected_generation": base["expected_generation"] + 1, "inputs": {"body": {}}}).status_code == 409
    assert client.post(f"/operator/actions/{action['action_id']}", json={**base, "inputs": {"wrong": 1}}).status_code == 422
    assert client.post("/operator/actions/meta.initialize_motion", json={**base, "inputs": {}}).status_code == 409
    assert calls == []


def test_legacy_home_xy_action_requires_the_strict_method_route(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    action = action_for(catalog, "POST", "/motion/oem/home_xy")
    response = client.post(f"/operator/actions/{action['action_id']}", json={
        "expected_generation": catalog["ownership_generation"],
        "idempotency_key": "home-xy-meta-1234",
        "inputs": {},
    })
    assert response.status_code == 409
    assert response.json()["detail"]["error"] == "canonical_strict_method_requires_v2_route"
    assert calls == []


def test_legacy_move_xy_action_requires_the_strict_method_route(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    action = action_for(catalog, "POST", "/motion/oem/move_xy")
    assert action["action_id"] == "oem.xy.move_absolute"
    response = client.post(f"/operator/actions/{action['action_id']}", json={
        "expected_generation": catalog["ownership_generation"],
        "idempotency_key": "move-xy-canonical-1234",
        "inputs": {"x": 123, "y": 456},
    })
    assert response.status_code == 409
    assert response.json()["detail"]["error"] == "canonical_strict_method_requires_v2_route"
    assert calls == []


def test_generic_assessment_rejects_provider_owned_z_manual_home_receipt(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    OperatorReceiptStore(tmp_path).put(
        {
            "command_id": "operator-z-home-1",
            "idempotency_key": "operator-z-home-1",
            "action_id": "oem.z.manual_home",
            "ownership_generation": 7,
            "started_at": "1.0",
            "status": "completed",
            "authority_receipt_id": "operator-z-home-1",
        }
    )

    assessed = client.post(
        "/operator/actions/receipts/operator-z-home-1/assessment",
        json={
            "expected_generation": 7,
            "idempotency_key": "assessment-z-home-1",
            "verdict": "pass",
            "note": "Observed movement.",
        },
    )

    assert assessed.status_code == 409
    assert assessed.json()["detail"]["error"] == "provider_owned_z_observation_required"
    assert assessed.json()["detail"]["replacement_action_id"] == "oem.z.observe"


def test_human_assessment_updates_existing_receipt_and_requires_generation(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    action = action_for(catalog, "POST", "/motion/test/home_axis")
    receipt = client.post(f"/operator/actions/{action['action_id']}", json={
        "expected_generation": catalog["ownership_generation"], "idempotency_key": "assessment-run-123", "inputs": {"body": {"axis": "x"}},
    }).json()
    assessed = client.post(f"/operator/actions/receipts/{receipt['command_id']}/assessment", json={
        "expected_generation": catalog["ownership_generation"], "idempotency_key": "assessment-pass-123", "verdict": "pass", "note": "Observed both reference indicators.",
    })
    assert assessed.status_code == 200
    assert assessed.json()["operator_assessment"] == "pass"
    assert assessed.json()["operator_note"] == "Observed both reference indicators."


def test_receipt_store_keeps_compact_history_and_replaces_by_command_id(tmp_path):
    runtime_owner = OEMRuntimeStore(tmp_path)
    runtime_owner.close()
    store = OperatorReceiptStore(tmp_path)
    for index in range(520):
        store.put({
            "command_id": f"cmd-{index}",
            "idempotency_key": f"key-{index}",
            "action_id": "query.status",
            "ownership_generation": 7,
            "started_at": str(index),
            "status": "completed",
        })
    rows = store.list(200)
    assert store.connection.execute("SELECT COUNT(*) FROM operator_commands").fetchone()[0] == 520
    assert store.by_command("cmd-7") is not None
    assert rows[0]["command_id"] == "cmd-519"
    with pytest.raises(RuntimeError, match="stale operator receipt expected state"):
        store.put({
            "command_id": "cmd-519",
            "idempotency_key": "key-519",
            "action_id": "query.status",
            "ownership_generation": 7,
            "started_at": "519",
            "status": "completed",
            "operator_assessment": "fail",
        })
    assert store.connection.execute("SELECT COUNT(*) FROM operator_commands").fetchone()[0] == 520


def test_public_durable_command_route_rejects_a_disabled_machine_assessment(
    tmp_path, monkeypatch,
):
    app, _calls = make_app(tmp_path, monkeypatch)
    app.state.operator_test_maintenance["motion_blocked"] = True
    app.state.operator_test_maintenance["block_reason"] = "maintenance"
    response = TestClient(app).post("/operator/commands", json={
        "schema_version": "bioxp.operator_command_request.v1",
        "idempotency_key": "blocked-durable-x-move",
        "expected_ownership_generation": 7,
        "action_id": "oem.x.move_steps",
        "inputs": {"steps": 1},
    })
    assert response.status_code == 409
    assert response.json()["detail"]["error"] == "action_unavailable"
    assert app.state.operator_command_plane.store.queue()["pending_count"] == 0
