from __future__ import annotations

import asyncio
import threading
from pathlib import Path

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from httpx import ASGITransport, AsyncClient, Response
from pydantic import BaseModel, Field

import bioxp.operator_controls as operator_controls
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

    @app.post("/motion/oem/home_xy")
    async def home_xy():
        calls.append(("home_xy", None))
        return {"ok": True, "stages": [{"stage_id": "x"}, {"stage_id": "y"}]}

    @app.post("/motion/oem/move_xy")
    async def move_xy(x: int, y: int, timeout_s: float = 120.0):
        calls.append(("move_xy", {"x": x, "y": y, "timeout_s": timeout_s}))
        return {"ok": True, "stages": [{"stage_id": "x"}, {"stage_id": "y"}]}

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
    install_operator_control_plane(
        app,
        maintenance_state_provider=lambda: maintenance,
        reference_state_provider=lambda: {"rows": {axis: {"state": "referenced"} for axis in ("x", "y", "z", "g", "door")}},
        lifecycle_state_provider=lambda: lifecycle,
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
    meta_ids = {row["action_id"] for row in catalog["actions"] if row["kind"] == "meta"}
    assert meta_ids == {"meta.activate_motion", "meta.initialize_motors", "meta.initialize_motion"}
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


def test_home_xy_primitive_maps_to_one_visible_source_route(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    action = action_for(catalog, "POST", "/motion/oem/home_xy")
    response = client.post(f"/operator/actions/{action['action_id']}", json={
        "expected_generation": catalog["ownership_generation"],
        "idempotency_key": "home-xy-meta-1234",
        "inputs": {},
    })
    assert response.status_code == 200, response.text
    assert calls == [("home_xy", None)]
    receipt = response.json()
    assert receipt["stage_receipts"] == []


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
