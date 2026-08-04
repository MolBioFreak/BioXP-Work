from __future__ import annotations

import asyncio
import json
from pathlib import Path

from fastapi import FastAPI
from fastapi.testclient import TestClient
from httpx import ASGITransport, AsyncClient, Response
from pydantic import BaseModel, Field

import bioxp.operator_controls as operator_controls
from bioxp.operator_controls import BoundedReceiptStore, install_operator_control_plane


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


def make_app(tmp_path: Path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(tmp_path))
    app = FastAPI()
    calls: list[tuple[str, object]] = []


    @app.get("/motion/axis/{axis}/status")
    async def axis_status(axis: str, verbose: bool = False):
        calls.append(("axis_status", {"axis": axis, "verbose": verbose}))
        return {"ok": True, "axis": axis, "verbose": verbose}

    @app.post("/motion/test/home_axis")
    async def home_axis(body: dict):
        calls.append(("home_axis", body))
        return {
            "ok": True,
            "controller_command_acknowledged": True,
            "controller_terminal_state_verified": True,
            "authority_receipt": {"command_id": "provider-home-axis-1", "status": "completed"},
            "stages": [{"stage_id": "home", "status": "passed"}],
        }

    @app.post("/motion/oem/home_xy")
    async def home_xy():
        calls.append(("home_xy", None))
        return {"ok": True, "stages": [{"stage_id": "x"}, {"stage_id": "y"}]}

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
    app.state.operator_test_maintenance = maintenance
    hardware = FakeHardwareState()
    app.state.operator_test_hardware = hardware
    monkeypatch.setattr(operator_controls, "hardware_state", hardware)
    install_operator_control_plane(
        app,
        maintenance_state_provider=lambda: maintenance,
        reference_state_provider=lambda: {"rows": {axis: {"state": "referenced"} for axis in ("x", "y", "z", "g", "door")}},
        lifecycle_state_provider=lambda: {"operation_state": "stopped", "operation_reason": "test", "door": {"door_closed": True, "latch_closed": True}},
    )
    return app, calls


def action_for(catalog: dict, method: str, path: str) -> dict:
    return next(action for action in catalog["actions"] if action["informational_method"] == method and action["informational_path"] == path and action["kind"] == "primitive")


def test_catalog_has_every_exact_route_once_and_distinct_meta_actions(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    assert catalog["schema_name"] == "bioxp.operator_control_catalog"
    primitive_routes = [(row["informational_method"], row["informational_path"]) for row in catalog["actions"] if row["kind"] == "primitive"]
    assert primitive_routes.count(("GET", "/motion/axis/{axis}/status")) == 1
    assert primitive_routes.count(("POST", "/motion/test/home_axis")) == 1
    assert primitive_routes.count(("POST", "/motion/oem/home_xy")) == 1
    meta_ids = {row["action_id"] for row in catalog["actions"] if row["kind"] == "meta"}
    assert meta_ids == {"meta.activate_motion", "meta.emergency_stop", "meta.home_xy", "meta.initialize_motors", "meta.initialize_motion"}
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


def test_partial_snapshot_does_not_make_unrelated_domains_block_motion(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    hardware = app.state.operator_test_hardware
    original_project = hardware.project

    def partial_project(domain):
        row = original_project(domain)
        if domain in {"transport", "power", "interlock", "latch"}:
            return row
        return {"domains": {domain: {"status": "missing", "observation": None}}, "freshness": {"state": "missing"}}

    monkeypatch.setattr(hardware, "project", partial_project)
    catalog = TestClient(app).get("/operator/control-catalog").json()
    action = action_for(catalog, "POST", "/motion/test/home_axis")
    dependencies = {row["key"]: row for row in action["dependencies"]}
    assert dependencies["canonical_snapshot"]["met"] is True
    assert dependencies["snapshot_fresh"]["met"] is True


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


def test_motion_inactive_disables_every_cataloged_motion_and_pipette_action(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    app.state.operator_test_maintenance.update({
        "motion_blocked": True,
        "recovery_required": True,
        "block_reason": "Motion is inactive.",
    })
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    motor_actions = [
        row for row in catalog["actions"]
        if row["kind"] == "primitive" and row["safety_class"] == "motion"
    ]
    assert motor_actions
    assert all(row["enabled"] is False for row in motor_actions)
    assert all(row["disabled_reason"] == "Motion is inactive. Activate motion before moving this motor." for row in motor_actions)
    assert any("pipette" in row["informational_path"].lower() for row in motor_actions)

    target = next(row for row in motor_actions if "/motion/test/home_axis" == row["informational_path"])
    blocked = client.post(f"/operator/actions/{target['action_id']}", json={
        "expected_generation": catalog["ownership_generation"],
        "idempotency_key": "motion-blocked-123456",
        "inputs": {"body": {"axis": "x"}},
    })
    assert blocked.status_code == 409
    assert blocked.json()["detail"]["reason"] == "Motion is inactive. Activate motion before moving this motor."
    assert any(row["key"] == "motion_enabled" and row["met"] is False for row in blocked.json()["detail"]["dependencies"])
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
    assert receipt["controller_acknowledged"] is True
    assert receipt["controller_terminal_state_verified"] is True
    assert receipt["authority_receipt_id"] == "provider-home-axis-1"
    assert receipt["authority_receipt_status"] == "completed"
    assert receipt["physical_effect_verified"] is False
    assert receipt["stage_receipts"] == [{"stage_id": "home", "status": "passed"}]
    assert json.loads((tmp_path / "operator_action_receipts.json").read_text())["receipts"][0]["command_id"] == receipt["command_id"]

    replay = client.post(f"/operator/actions/{action['action_id']}", json=payload)
    assert replay.status_code == 200
    assert replay.json()["command_id"] == receipt["command_id"]
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
    assert response.json()["response"]["body"]["axis"] == "y"


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


def test_generic_motion_request_models_do_not_claim_z_specific_step_envelopes():
    from bioxp.api import (
        MoveAbsoluteRequest,
        MoveRelativeRequest,
        OemManualAbsoluteRequest,
        OemManualRelativeRequest,
        ReferenceMarkRequest,
    )

    relative = MoveRelativeRequest.model_json_schema()["properties"]["steps"]
    absolute = MoveAbsoluteRequest.model_json_schema()["properties"]["position_steps"]
    manual_relative = OemManualRelativeRequest.model_json_schema()["properties"]["steps"]
    manual_absolute = OemManualAbsoluteRequest.model_json_schema()["properties"]["position_steps"]
    reference = ReferenceMarkRequest.model_json_schema()["properties"]["position_steps"]
    for schema in (relative, absolute, manual_relative, manual_absolute, reference):
        assert "minimum" not in schema
        assert "maximum" not in schema


def test_z_semantic_moves_expose_z_bounds_without_hardening_generic_routes():
    from bioxp.api import app

    actions, _ = operator_controls._build_catalog(app)
    by_id = {action["action_id"]: action for action in actions}

    relative = next(row for row in by_id["oem.z.move_steps"]["inputs"] if row["name"] == "steps")
    absolute = next(row for row in by_id["oem.z.move_absolute"]["inputs"] if row["name"] == "position_steps")
    assert (relative["minimum"], relative["maximum"]) == (-160000, 160000)
    assert (absolute["minimum"], absolute["maximum"]) == (0, 160000)

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
    assert generic_steps["minimum"] is None and generic_steps["maximum"] is None
    assert generic_position["minimum"] is None and generic_position["maximum"] is None


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


def test_home_xy_meta_maps_to_one_visible_source_route(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    catalog = client.get("/operator/control-catalog").json()
    response = client.post("/operator/actions/meta.home_xy", json={
        "expected_generation": catalog["ownership_generation"],
        "idempotency_key": "home-xy-meta-1234",
        "inputs": {},
    })
    assert response.status_code == 200, response.text
    assert calls == [("home_xy", None)]
    assert len(response.json()["stage_receipts"]) == 2


def test_generic_assessment_rejects_provider_owned_z_manual_home_receipt(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    BoundedReceiptStore(tmp_path).put(
        {
            "command_id": "operator-z-home-1",
            "idempotency_key": "operator-z-home-1",
            "action_id": "oem.z.manual_home",
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


def test_receipt_store_is_bounded_and_replaces_by_command_id(tmp_path):
    store = BoundedReceiptStore(tmp_path)
    for index in range(520):
        store.put({"command_id": f"cmd-{index}", "idempotency_key": f"key-{index}"})
    rows = store.list(200)
    on_disk = json.loads(store.path.read_text())["receipts"]
    assert len(on_disk) == 512
    assert rows[0]["command_id"] == "cmd-519"
    store.put({"command_id": "cmd-519", "idempotency_key": "key-519", "operator_assessment": "fail"})
    assert len(json.loads(store.path.read_text())["receipts"]) == 512
    assert store.by_command("cmd-519")["operator_assessment"] == "fail"
