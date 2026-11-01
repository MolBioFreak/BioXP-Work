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
    # Fake channels have no status payload; project only the software tip type.
    monkeypatch.setattr(transport, "get_status", lambda: {"tip_type": transport._tip_type, "live_query_performed": False})

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
        for kind in (10, 50, 200, 201):
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
            # Status projection follows the selection without a separate collect.
            assert api.hardware_state.project("pipette")["domains"]["pipette"]["observation"]["tip_type"] == kind
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
    assert field["enum"] == [10, 50, 200, 201]


def test_tip_type_receipt_is_completed_not_failed(tmp_path, monkeypatch):
    from bioxp.oem_runtime_store import OEMRuntimeStore
    from bioxp.pipette.receipts import PipetteReceiptStore
    import bioxp.pipette.receipts as receipts
    monkeypatch.setattr(receipts, "current_release_identity", lambda: {"verified": True, "release_id": "offline",
        "source": {"manifest_sha256": "1" * 64, "aggregate_sha256": "2" * 64}})
    monkeypatch.setattr(receipts, "current_authority_identity", lambda: {"evidence_lock_identity_verified": True,
        "evidence_lock_sha256": "3" * 64})
    monkeypatch.setattr(receipts, "current_registry_sha256", lambda: "4" * 64)
    OEMRuntimeStore(tmp_path).close()
    store = PipetteReceiptStore(root=tmp_path)
    driver = SimpleNamespace(_pipette_message_state=None, bus=None)
    transport = FourPipetteTransport([
        SimpleNamespace(_driver=driver, _get_driver=lambda: driver) for _ in range(4)
    ])
    monkeypatch.setattr(api, "_get_pipette_transport", lambda: transport)
    monkeypatch.setattr(api, "_pipette_receipts", store)

    async def inline(label, call, *, timeout_s):
        return call()

    monkeypatch.setattr(api, "_run_blocking", inline)
    client = TestClient(api.app, raise_server_exceptions=True)
    try:
        response = client.post(PATH, json={"tip_type": 50}, headers={"Idempotency-Key": "tip-type-receipt"})
        assert response.status_code == 200, response.text
        assert response.json()["hardware_commanded"] is False
        row = store.connection.execute(
            "SELECT status FROM pipette_operations ORDER BY rowid DESC LIMIT 1").fetchone()
        assert row["status"] == "completed"
    finally:
        client.close()
    # A finished completion-wait pump command is terminal even when completion
    # attribution stayed unverified (it used to be left 'acknowledged' forever).
    acked = {"controller_acknowledged": True, "delivery_verified": True, "completion_verified": False}
    for ok, status in ((True, "completed"), (False, "failed")):
        store.record(operation="device_command", requested_inputs={"command": "lld_parameter"},
                     result={**acked, "ok": ok, "wait_mode": "completion",
                             "channels": [{"channel": c, "completion": {"ok": ok}} for c in range(4)]})
        row = store.connection.execute(
            "SELECT status FROM pipette_operations ORDER BY rowid DESC LIMIT 1").fetchone()
        assert row["status"] == status
    store.record(operation="device_command", requested_inputs={"command": "lld_parameter"},
                 result={**acked, "ok": True, "wait_mode": "ack", "channels": [{"channel": 0}]})
    assert store.connection.execute(
        "SELECT status FROM pipette_operations ORDER BY rowid DESC LIMIT 1").fetchone()["status"] == "acknowledged"
