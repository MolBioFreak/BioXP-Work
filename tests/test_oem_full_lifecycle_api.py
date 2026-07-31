from __future__ import annotations

from threading import Event, Thread

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from src.bioxp import oem_runtime_api
from src.bioxp.hardware_status import HardwareStateOwner
from src.bioxp.oem_full_lifecycle import (
    OemFullLifecycleError,
    OemFullLifecycleRuns,
    current_authority_identity,
    current_registry_sha256,
)
from src.bioxp.oem_runtime_store import OEMRuntimeStore


def _robot_inputs():
    return {
        "ownership_generation": 7,
        "can_ready": True,
        "board_test_mode": False,
        "pipette_exists": None,
        "initialize_system_producer": "initializeEnvironment",
        "update_check_suppresses_initialize_system": False,
        "system_in_motion_at_entry": False,
        "enclosure_door_closed": True,
        "latch_closed": True,
        "saved_status": 1,
        "ship_mode": "",
        "start_mode": "WebMode",
        "tip_present": False,
        "self_test_due": True,
        "check_camera": True,
        "camera_installed": True,
        "is_development_machine": False,
        "deck_inspection": True,
    }


def _client(tmp_path, monkeypatch):
    tmp_path.mkdir(parents=True, exist_ok=True)
    oem_runtime_api.configure_runtime(store_root=str(tmp_path), autostart=False)
    monkeypatch.setattr(oem_runtime_api, "_derive_full_lifecycle_inputs", _robot_inputs)
    owner = HardwareStateOwner()
    with owner.ownership_lease():
        owner._epoch = 7
    monkeypatch.setattr(oem_runtime_api, "hardware_state", owner)
    app = FastAPI()
    app.include_router(oem_runtime_api.router)
    return TestClient(app)


def _request(**overrides):
    payload = {
        "command": "initialize_oem_movement_lifecycle",
        "operator_ack": "INITIALIZE",
        "expected_generation": 7,
        "bms_connection_generation": 77,
        "expected_machine_serial": 206,
        "expected_registry_sha256": current_registry_sha256(),
        "expected_evidence_lock_sha256": current_authority_identity()["evidence_lock_sha256"],
        "idempotency_key": "api-dry-run",
        "mode": "dry_run",
    }
    payload.update(overrides)
    return payload


def _cancel_request(**overrides):
    payload = {
        "expected_generation": 7,
        "bms_connection_generation": 77,
        "expected_machine_serial": 206,
        "expected_registry_sha256": current_registry_sha256(),
        "expected_evidence_lock_sha256": current_authority_identity()["evidence_lock_sha256"],
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
    assert body["evidence_lock_identity_verified"] is True
    assert body["source_authority_verified"] is False
    assert body["evidence_lock_sha256"] == "a69454df24e9348fd34d8c89f2a2e089576587152bdcc20754f9d700ecbaf03c"
    assert len(body["initialize_system_producers"]) == 5
    assert body["live_creation_enabled"] is False
    assert body["physical_commissioning_complete"] is False
    assert body["providers"]["initialize_motors_m01_m19"]["implemented"] is True
    assert body["providers"]["pipette_tip_query_and_remediation"]["live_bound"] is False
    assert body["providers"]["tc_rc_oc_motion_self_test"]["implemented"] == "receipt_evaluator"
    assert body["providers"]["park_gantry"]["implemented"] == "receipt_evaluator"
    assert body["safety_boundary"]["physical_effect_verified"] is False


def test_lifecycle_dry_run_and_cancel_require_machine_state_not_authentication(tmp_path, monkeypatch):
    client = _client(tmp_path, monkeypatch)
    created = client.post("/oem/runtime/movement-runs", json=_request(idempotency_key="no-auth-dry-run"))
    assert created.status_code == 200
    run_id = created.json()["run_id"]
    cancelled = client.post(f"/oem/runtime/movement-runs/{run_id}/cancel", json=_cancel_request())
    assert cancelled.status_code == 200

    live = client.post(
        "/oem/runtime/movement-runs",
        json=_request(mode="live", idempotency_key="no-auth-live-blocked"),
    )
    assert live.status_code == 409
    assert "commission" in live.json()["detail"].lower()


def test_fixed_movement_run_api_creates_robot_owned_plan_and_gets_ledger(tmp_path, monkeypatch):
    client = _client(tmp_path, monkeypatch)

    response = client.post("/oem/runtime/movement-runs", json=_request())
    assert response.status_code == 200
    created = response.json()
    assert created["run_state"] == "planned"
    assert created["machine_configuration_verified"] is True
    assert created["request"]["expected_generation"] == 7
    assert created["request"]["bms_connection_generation"] == 77
    assert created["request"]["expected_evidence_lock_sha256"] == current_authority_identity()["evidence_lock_sha256"]
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


def test_fixed_movement_run_api_rejects_unrelated_robot_ownership_generation(tmp_path, monkeypatch):
    client = _client(tmp_path, monkeypatch)
    response = client.post(
        "/oem/runtime/movement-runs",
        json=_request(expected_generation=77, bms_connection_generation=77),
    )
    assert response.status_code == 409
    assert "ownership generation" in response.json()["detail"].lower()


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

    cancelled = client.post(f"/oem/runtime/movement-runs/{created['run_id']}/cancel", json=_cancel_request())
    assert cancelled.status_code == 200
    assert cancelled.json()["run_state"] == "cancelled"


def test_cancel_rejects_mismatched_admission_generation_or_authority(tmp_path, monkeypatch):
    client = _client(tmp_path, monkeypatch)
    created = client.post("/oem/runtime/movement-runs", json=_request(idempotency_key="cancel-bound")).json()
    route = f"/oem/runtime/movement-runs/{created['run_id']}/cancel"

    assert client.post(route, json=_cancel_request(expected_generation=8)).status_code == 409
    assert client.post(route, json=_cancel_request(bms_connection_generation=78)).status_code == 409
    assert client.post(route, json=_cancel_request(expected_registry_sha256="9" * 64)).status_code == 409
    assert client.post(route, json=_cancel_request(expected_evidence_lock_sha256="8" * 64)).status_code == 409
    assert client.get(f"/oem/runtime/movement-runs/{created['run_id']}").json()["run_state"] == "planned"


def test_ownership_transition_cannot_interleave_with_durable_reservation(tmp_path, monkeypatch):
    client = _client(tmp_path, monkeypatch)
    runs = oem_runtime_api._require_full_lifecycle_runs()
    original = runs.store.create_oem_full_lifecycle_run_once
    transition_started = Event()
    transition_done = Event()
    threads = []

    def wrapped(*args, **kwargs):
        def transition():
            transition_started.set()
            oem_runtime_api.hardware_state.change_ownership(reason="reviewer-race")
            transition_done.set()

        thread = Thread(target=transition)
        threads.append(thread)
        thread.start()
        assert transition_started.wait(1)
        assert not transition_done.wait(0.05)
        return original(*args, **kwargs)

    monkeypatch.setattr(runs.store, "create_oem_full_lifecycle_run_once", wrapped)
    response = client.post("/oem/runtime/movement-runs", json=_request(idempotency_key="atomic-owner"))
    assert response.status_code == 200
    for thread in threads:
        thread.join(1)
    assert transition_done.is_set()
    assert response.json()["ownership_generation"] == 7
    assert oem_runtime_api.hardware_state.ownership_epoch == 8


def test_create_and_cancel_reject_current_robot_ownership_change(tmp_path, monkeypatch):
    client = _client(tmp_path, monkeypatch)
    runs = oem_runtime_api._require_full_lifecycle_runs()
    original = runs.store.create_oem_full_lifecycle_run_once

    def advance_before_reservation(*args, **kwargs):
        oem_runtime_api.hardware_state.change_ownership(reason="reentrant-reviewer-race")
        return original(*args, **kwargs)

    monkeypatch.setattr(runs.store, "create_oem_full_lifecycle_run_once", advance_before_reservation)
    assert client.post("/oem/runtime/movement-runs", json=_request(idempotency_key="stale-create")).status_code == 409

    client = _client(tmp_path / "cancel", monkeypatch)
    created = client.post("/oem/runtime/movement-runs", json=_request(idempotency_key="stale-cancel")).json()
    oem_runtime_api.hardware_state.change_ownership(reason="stale-before-cancel")
    response = client.post(f"/oem/runtime/movement-runs/{created['run_id']}/cancel", json=_cancel_request())
    assert response.status_code == 409


def test_cancel_rejects_persisted_stage_corruption(tmp_path):
    runs = OemFullLifecycleRuns(OEMRuntimeStore(tmp_path))
    request = _request(idempotency_key="persisted-corruption")
    request["inputs"] = _robot_inputs()
    created = runs.create(request)
    payload = runs.get(created["run_id"])
    payload["stages"][0]["status"] = "completed"
    runs.store.write_oem_full_lifecycle_run(payload)
    with pytest.raises(OemFullLifecycleError, match="stage evidence"):
        runs.cancel(created["run_id"], _cancel_request())
