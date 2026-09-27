"""OEM tip-type combo is host-only, not a physical pickup or inventory edit."""
import pytest
from types import SimpleNamespace
from fastapi.testclient import TestClient

from bioxp import api
from bioxp.operator_controls import _build_catalog, _motor_motion_action, _safety
from bioxp.pipette.transport import FourPipetteTransport

PATH = "/liquid/pipette/tip-type"


def test_typed_tip_selection_uses_existing_software_owner(monkeypatch):
    transport = FourPipetteTransport([
        SimpleNamespace(_driver=SimpleNamespace(_pipette_message_state=None, bus=None))
        for _ in range(4)
    ])
    monkeypatch.setattr(api, "_get_pipette_transport", lambda: transport)
    monkeypatch.setattr(api, "_pipette_receipts", None)
    monkeypatch.setattr(api, "_get_tester", lambda: pytest.fail("hardware must not be acquired"))

    async def inline(label, call, *, timeout_s):
        return call()

    monkeypatch.setattr(api, "_run_blocking", inline)
    client = TestClient(api.app, raise_server_exceptions=True)
    try:
        for index, invalid in enumerate(({}, {"tip_type": 0}, {"tip_type": True},
                         {"tip_type": "50"}, {"tip_type": 50, "well": "A1"})):
            assert client.post(PATH, json=invalid,
                headers={"Idempotency-Key": f"invalid-tip-type-{index}"}).status_code == 422
        assert transport._tip_type == 201
        for kind in (50, 200, 201):
            response = client.post(PATH, json={"tip_type": kind},
                                   headers={"Idempotency-Key": f"tip-type-{kind}"})
            assert response.status_code == 200, response.text
            body = response.json()
            assert body["tip_type"] == kind
            assert body["tip_location"] == -1
            assert body["outcome"] == "host_state_only"
            assert body["hardware_commanded"] is False
            assert body["physical_effect_verified"] is False
            assert transport._tip_type == kind
    finally:
        client.close()


def test_tip_type_is_typed_catalog_service_action():
    _, actions = _build_catalog(api.app)
    matching = [row for row in actions.values() if row.get("path") == PATH]
    assert len(matching) == 1
    assert matching[0]["method"] == "POST"
    assert not _motor_motion_action(matching[0])
    assert _safety("POST", PATH) == "service"
    field = api.PipetteTipTypeRequest.model_json_schema()["properties"]["tip_type"]
    assert field["enum"] == [50, 200, 201]
