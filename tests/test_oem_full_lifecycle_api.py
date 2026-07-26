from __future__ import annotations

from fastapi import FastAPI
from fastapi.testclient import TestClient

from src.bioxp import oem_runtime_api
from src.bioxp.oem_full_lifecycle import current_registry_sha256


def _robot_inputs():
    return {
        "ownership_generation": 7,
        "saved_status": 1,
        "ship_mode": "",
        "start_mode": "WebMode",
        "tip_present": False,
        "self_test_due": True,
        "camera_required": True,
        "deck_inspection": True,
    }


def _client(tmp_path, monkeypatch):
    oem_runtime_api.configure_runtime(store_root=str(tmp_path), autostart=False)
    monkeypatch.setattr(oem_runtime_api, "_derive_full_lifecycle_inputs", _robot_inputs)
    app = FastAPI()
    app.include_router(oem_runtime_api.router)
    return TestClient(app)


def _request(**overrides):
    payload = {
        "command": "initialize_oem_movement_lifecycle",
        "operator_ack": "INITIALIZE",
        "expected_machine_serial": 206,
        "expected_registry_sha256": current_registry_sha256(),
        "idempotency_key": "api-dry-run",
        "mode": "dry_run",
    }
    payload.update(overrides)
    return payload


def test_fixed_movement_run_api_creates_robot_owned_plan_and_gets_ledger(tmp_path, monkeypatch):
    client = _client(tmp_path, monkeypatch)

    response = client.post("/oem/runtime/movement-runs", json=_request())
    assert response.status_code == 200
    created = response.json()
    assert created["run_state"] == "planned"
    assert created["ownership_generation"] == 7
    assert created["expected_next_stage"] == "initialize_environment"

    run_id = created["run_id"]
    assert client.get(f"/oem/runtime/movement-runs/{run_id}").json()["run_id"] == run_id
    ledger = client.get(f"/oem/runtime/movement-runs/{run_id}/ledger").json()
    assert ledger["run_id"] == run_id
    assert ledger["stages"][0]["source_anchor"] == "BioXPMainWindow.cs:973-1027"


def test_fixed_movement_run_api_rejects_arbitrary_motion_or_unknown_fields(tmp_path, monkeypatch):
    client = _client(tmp_path, monkeypatch)

    for field, value in (("axis", "z"), ("stage", "M01"), ("position", 100), ("raw_frame", "00ff")):
        response = client.post("/oem/runtime/movement-runs", json=_request(**{field: value}))
        assert response.status_code == 422, (field, response.text)


def test_fixed_movement_run_api_rejects_live_until_commissioned_provider_exists(tmp_path, monkeypatch):
    client = _client(tmp_path, monkeypatch)
    response = client.post("/oem/runtime/movement-runs", json=_request(mode="live", idempotency_key="live-blocked"))
    assert response.status_code == 409
    assert "commission" in response.json()["detail"].lower()


def test_fixed_movement_run_api_cancel_is_robot_owned(tmp_path, monkeypatch):
    client = _client(tmp_path, monkeypatch)
    created = client.post("/oem/runtime/movement-runs", json=_request(idempotency_key="cancel-api")).json()

    cancelled = client.post(f"/oem/runtime/movement-runs/{created['run_id']}/cancel")
    assert cancelled.status_code == 200
    assert cancelled.json()["run_state"] == "cancelled"
