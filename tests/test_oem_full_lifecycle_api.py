from __future__ import annotations

from fastapi import FastAPI
from fastapi.testclient import TestClient

from src.bioxp import oem_runtime_api
from src.bioxp.oem_full_lifecycle import current_registry_sha256


def _robot_inputs():
    return {
        "ownership_generation": 7,
        "can_ready": True,
        "enclosure_door_closed": True,
        "latch_closed": True,
        "saved_status": 1,
        "ship_mode": "",
        "start_mode": "WebMode",
        "tip_present": False,
        "self_test_due": True,
        "camera_required": True,
        "deck_inspection": True,
    }


TEST_MUTATION_TOKEN = "test-only-lifecycle-token-0000000000000000"


def _client(tmp_path, monkeypatch):
    token_path = tmp_path / "lifecycle.token"
    token_path.write_text(TEST_MUTATION_TOKEN, encoding="utf-8")
    token_path.chmod(0o600)
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_TOKEN_FILE", str(token_path))
    oem_runtime_api.configure_runtime(store_root=str(tmp_path), autostart=False)
    monkeypatch.setattr(oem_runtime_api, "_derive_full_lifecycle_inputs", _robot_inputs)
    app = FastAPI()
    app.include_router(oem_runtime_api.router)
    client = TestClient(app)
    client.headers.update({"X-BioXP-OEM-Token": TEST_MUTATION_TOKEN})
    return client


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


def test_contract_does_not_advertise_plan_when_robot_owned_predicates_are_unknown(tmp_path):
    oem_runtime_api.configure_runtime(store_root=str(tmp_path), autostart=False)
    app = FastAPI()
    app.include_router(oem_runtime_api.router)
    client = TestClient(app)
    body = client.get("/oem/runtime/movement-runs/contract").json()
    assert body["plan_available"] is False
    assert body["plan_blockers"]
    assert "not bound" in " ".join(body["plan_blockers"])


def test_contract_reports_partial_provider_truth_and_blocks_live(tmp_path, monkeypatch):
    client = _client(tmp_path, monkeypatch)
    response = client.get("/oem/runtime/movement-runs/contract")
    assert response.status_code == 200
    body = response.json()
    assert body["plan_available"] is True
    assert body["live_creation_enabled"] is False
    assert body["physical_commissioning_complete"] is False
    assert body["providers"]["initialize_motors_m01_m19"]["implemented"] is True
    assert body["providers"]["pipette_tip_query_and_remediation"]["live_bound"] is False
    assert body["providers"]["tc_rc_oc_motion_self_test"]["implemented"] == "receipt_evaluator"
    assert body["providers"]["park_gantry"]["implemented"] == "receipt_evaluator"
    assert body["safety_boundary"]["physical_effect_verified"] is False


def test_lifecycle_mutations_require_token_file_authorization(tmp_path, monkeypatch):
    authorized = _client(tmp_path, monkeypatch)
    raw = TestClient(authorized.app)

    assert raw.post("/oem/runtime/movement-runs", json=_request()).status_code == 403
    assert raw.post(
        "/oem/runtime/movement-runs",
        json=_request(),
        headers={"X-BioXP-OEM-Token": "wrong-token"},
    ).status_code == 403

    run_id = authorized.post("/oem/runtime/movement-runs", json=_request()).json()["run_id"]
    assert raw.post(f"/oem/runtime/movement-runs/{run_id}/cancel").status_code == 403


def test_fixed_movement_run_api_creates_robot_owned_plan_and_gets_ledger(tmp_path, monkeypatch):
    client = _client(tmp_path, monkeypatch)

    response = client.post("/oem/runtime/movement-runs", json=_request())
    assert response.status_code == 200
    created = response.json()
    assert created["run_state"] == "planned"
    assert created["ownership_generation"] == 7
    assert created["expected_next_stage"] == "construct_control_lib"

    run_id = created["run_id"]
    assert client.get(f"/oem/runtime/movement-runs/{run_id}").json()["run_id"] == run_id
    ledger = client.get(f"/oem/runtime/movement-runs/{run_id}/ledger").json()
    assert ledger["run_id"] == run_id
    assert ledger["stages"][0]["source_anchor"] == "BioXPMainWindow:672-675"


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


def test_configure_runtime_automatically_blocks_interrupted_lifecycle_run(tmp_path, monkeypatch):
    client = _client(tmp_path, monkeypatch)
    created = client.post("/oem/runtime/movement-runs", json=_request(idempotency_key="restart-auto")).json()
    runs = oem_runtime_api._require_full_lifecycle_runs()
    payload = runs.get(created["run_id"])
    payload["run_state"] = "running"
    payload["current_stage"] = payload["stages"][0]["stage_id"]
    payload["stages"][0]["status"] = "running"
    runs.store.write_oem_full_lifecycle_run(payload)

    oem_runtime_api.configure_runtime(store_root=str(tmp_path), autostart=False)
    recovered = oem_runtime_api._require_full_lifecycle_runs().get(created["run_id"])
    assert recovered["run_state"] == "blocked"
    assert recovered["current_stage"] is None
    assert recovered["physical_effect_verified"] is False


def test_malformed_run_id_fails_as_client_error_not_http_500(tmp_path, monkeypatch):
    _client(tmp_path, monkeypatch)
    app = FastAPI()
    app.include_router(oem_runtime_api.router)
    client = TestClient(app, raise_server_exceptions=False)
    response = client.get("/oem/runtime/movement-runs/bad%5Cname")
    assert 400 <= response.status_code < 500


def test_fixed_movement_run_api_cancel_is_robot_owned(tmp_path, monkeypatch):
    client = _client(tmp_path, monkeypatch)
    created = client.post("/oem/runtime/movement-runs", json=_request(idempotency_key="cancel-api")).json()

    cancelled = client.post(f"/oem/runtime/movement-runs/{created['run_id']}/cancel")
    assert cancelled.status_code == 200
    assert cancelled.json()["run_state"] == "cancelled"
