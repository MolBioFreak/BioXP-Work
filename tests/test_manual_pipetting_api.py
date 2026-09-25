"""Real manual REST routes and native handler binding, without hardware."""
import asyncio
import json
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient
from bioxp import api
from bioxp.operator_controls import _build_catalog, _motor_motion_action, _safety
from bioxp.protocols.models import ProtocolActionKind
from tests.test_manual_pipetting import program


def test_compile_route_uses_native_compiler_without_owner_or_idempotency(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("compile acquired hardware or canonical live owner")
    monkeypatch.setattr(api, "_get_tester", forbidden)
    monkeypatch.setattr(api, "_protocol_command_store", forbidden)
    response = TestClient(api.app).post("/liquid/manual/compile", json=program())
    assert response.status_code == 200, response.text
    compiled = response.json()
    document = compiled["document"]
    assert document["metadata"]["manual_scope"] == "explicit_steps_only"
    assert document["stages"][0]["actions"][0]["kind"] == "pipette_position"
    assert not any(a["kind"] == "oem_operation" for a in document["stages"][0]["actions"])
    assert _safety("POST", "/liquid/manual/compile") == "service"
    assert not _motor_motion_action({"informational_method": "POST", "informational_path": "/liquid/manual/compile", "safety_class": "service"})


def test_native_handler_map_is_lazy_and_contains_position_owner(monkeypatch):
    monkeypatch.setattr(api, "_protocol_command_store", lambda: pytest.fail("premature owner lookup"))
    handlers = api._protocol_live_handlers()
    assert handlers[ProtocolActionKind.PIPETTE_POSITION] is api._protocol_live_manual_position_handler


def test_manual_execute_routes_exact_intent_to_existing_native_owner(monkeypatch):
    calls = []
    async def execute(req):
        calls.append(req.model_dump(exclude_unset=True))
        return {"sentinel": "canonical-result"}
    monkeypatch.setattr(api, "protocol_execute", execute)
    request = api.ManualPipettingExecuteRequest(**program(), dry_run=False,
        idempotency_key="manual-existing-owner", live_execution_ack=True,
        live_execution={"operator_id": None, "physical_console_verified": False})
    assert asyncio.run(api.liquid_manual_execute(request)) == {"sentinel": "canonical-result"}
    assert calls[0]["source_type"] == "native"
    assert calls[0]["dry_run"] is False
    assert calls[0]["idempotency_key"] == "manual-existing-owner"
    assert calls[0]["live_execution"] == {"operator_id": None, "physical_console_verified": False}
    assert calls[0]["live_execution_ack"] is True
    assert calls[0]["document"]["metadata"]["manual_scope"] == "explicit_steps_only"


def test_real_manual_dry_run_returns_saved_native_bundle(monkeypatch, tmp_path):
    from bioxp.services import protocol_service
    store = protocol_service.ProtocolOperatorBundleStore(tmp_path)
    create = protocol_service.create_protocol_job
    monkeypatch.setattr(api, "create_protocol_job", lambda payload, **kw: create(payload, store=store, **kw))
    monkeypatch.setattr(api, "_get_tester", lambda: pytest.fail("dry run touched hardware"))
    response = TestClient(api.app).post("/liquid/manual/execute", json=program(),
        headers={"Idempotency-Key": "manual-dry-run-test"})
    assert response.status_code == 200, response.text
    payload = response.json()
    assert payload["execution"]["dry_run"] is True
    assert payload["protocol"]["document"]["stages"][0]["actions"][0]["kind"] == "pipette_position"


def test_catalog_publishes_full_discriminated_step_schema():
    actions, dispatch = _build_catalog(api.app)
    matches = [a for a in actions if a["informational_path"] == "/liquid/manual/compile"]
    assert len(matches) == 1
    fields = {field["name"]: field for field in matches[0]["inputs"]}
    schema = fields["steps"]["json_schema"]
    assert schema["type"] == "array"
    assert schema["items"]["discriminator"]["propertyName"] == "operation"
    assert schema["items"]["discriminator"]["mapping"]["move"] == "#/$defs/ManualMove"
    assert {"ManualMove", "ManualLower", "ManualLift", "ManualLiquid", "ManualMix"} <= set(schema["$defs"])
    assert schema["$defs"]["ManualLift"]["properties"]["height_steps"]["anyOf"][1] == {"type": "null"}


@pytest.mark.parametrize("change", [
    {"operation": "move", "location_id": 3, "well": "J2", "position_flag": 1},
    {"operation": "aspirate", "channels": [5], "volume_ul": 2.0, "speed": 20.0},
])
def test_bad_authoring_is_request_error_before_any_execution(monkeypatch, change):
    monkeypatch.setattr(api, "_get_tester", lambda: pytest.fail("invalid request touched hardware"))
    response = TestClient(api.app).post("/liquid/manual/compile", json={"protocol_id": "bad", "steps": [change]})
    assert response.status_code == 422, response.text
