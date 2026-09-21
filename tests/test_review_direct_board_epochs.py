"""Offline V2 -> retained owner -> ASGI -> SQLite epoch-binding regressions.

Only hardware/readiness providers are doubles; no robot or OEM host is used.
These tests qualify the Python request contract, not native movement parity.
"""
import asyncio
import copy
import time
from types import SimpleNamespace

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from bioxp import operator_controls as controls
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.operator_receipt_store import OperatorReceiptStore
from bioxp.runtime_audit_store import request_digest


@pytest.fixture
def direct_rig(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(tmp_path))
    OEMRuntimeStore(tmp_path).close()
    epochs = {"4": 10, "5": 20}
    delivered, contexts = [], []
    before_leaf = [None]
    monkeypatch.setattr(controls, "current_release_identity", lambda: {
        "verified": True, "release_id": "offline-review",
        "source": {"manifest_sha256": "1" * 64, "aggregate_sha256": "2" * 64}})
    monkeypatch.setattr(controls, "current_authority_identity", lambda: {
        "evidence_lock_identity_verified": True, "evidence_lock_sha256": "3" * 64})
    monkeypatch.setattr(controls, "current_registry_sha256", lambda: "4" * 64)
    # Deliberately isolate the new epoch boundary from unrelated readiness rules.
    monkeypatch.setattr(controls, "_assess_action", lambda *a, **k: {
        "enabled": True, "disabled_reason": None, "dependencies": []})
    class Hardware:
        ownership_epoch = 7
        def ownership_projection(self):
            return {"ownership_epoch": 7, "ownership": {
                "transport": "owned", "usb": "service", "router": "running", "CAN_READY": True}}
        def project(self, *domains, **kw):
            return {"snapshot_id": "offline", "domains": {},
                    "freshness": {"state": "fresh", "age_s": 0.0, "fresh_for_s": 30.0}}
    monkeypatch.setattr(controls, "hardware_state", Hardware())
    app = FastAPI()
    @app.post("/motion/oem/z/clear")
    async def leaf():
        context = controls.current_operator_dispatch_context()
        contexts.append(context)
        if before_leaf[0]:
            before_leaf[0]()
        # The actual worker lease consumes this hook; exercise that same hook
        # before our physical-leaf double, without touching the global daemon.
        precheck = context.get("motion_snapshot_precheck")
        if callable(precheck):
            precheck()
        delivered.append("clear")
        return {"ok": True, "controller_command_acknowledged": True,
                "controller_terminal_state_verified": True}
    controls.install_operator_control_plane(app,
        maintenance_state_provider=lambda: {"motion_blocked": False, "recovery_required": False},
        reference_state_provider=lambda: {"rows": {}},
        lifecycle_state_provider=lambda: {"operation_state": "stopped"},
        serial206_initialization_state_provider=lambda: {
            "bound": True, "initialize_motors_live_available": True,
            "x_authority": {"current_board_lifecycle_generation": epochs.get("5")},
            "board4_authority": {"active_board_epoch": epochs.get("4")}})
    with TestClient(app) as client:
        yield SimpleNamespace(client=client, app=app, root=tmp_path, epochs=epochs,
                              delivered=delivered, contexts=contexts, before_leaf=before_leaf)
    app.state.operator_command_plane.stop()


def request(epochs=None, key="review-board-key"):
    return {"schema_version": "bioxp.operator_action_request.v2",
        "expected_ownership_generation": 7,
        "expected_board_epoch_by_board": {"4": 10, "5": 20} if epochs is None else epochs,
        "idempotency_key": key, "inputs": {}}


def settle(rig, command_id):
    for _ in range(200):
        response = rig.client.get(f"/operator/v2/actions/receipts/{command_id}")
        assert response.status_code == 200, response.text
        if response.json()["terminal"]:
            return response.json()
        time.sleep(0.005)
    pytest.fail("offline action did not settle")


@pytest.mark.parametrize("endpoint", ["/operator/v2/actions/oem.z.clear", "/operator/actions/oem.z.clear"])
@pytest.mark.parametrize("epochs", [{"4": 9}, {"5": 19}, {"6": 1}])
def test_stale_or_unknown_supplied_epoch_rejected_before_leaf(direct_rig, endpoint, epochs):
    r = direct_rig.client.post(endpoint, json=request(epochs))
    assert r.status_code == 409, r.text
    assert r.json()["detail"]["error"] == "board_epoch_mismatch"
    assert direct_rig.delivered == []


def test_epochs_survive_durable_receipt_and_replay(direct_rig):
    r = direct_rig.client.post("/operator/v2/actions/oem.z.clear", json=request())
    assert r.status_code == 200, r.text
    row = settle(direct_rig, r.json()["command_id"])
    assert row["status"] == "completed", row
    assert row["expected_board_epoch_by_board"] == {"4": 10, "5": 20}
    assert direct_rig.contexts[0]["expected_board_epoch_by_board"] == {"4": 10, "5": 20}
    reader = OperatorReceiptStore(direct_rig.root)
    stored = reader.by_command(row["command_id"], include_evidence=False)
    assert stored["expected_board_epoch_by_board"] == {"4": 10, "5": 20}
    # Same original identity returns the old receipt after epoch drift; no reissue.
    direct_rig.epochs["4"] = 11
    replay = direct_rig.client.post("/operator/v2/actions/oem.z.clear", json=request())
    assert replay.status_code == 200, replay.text
    assert replay.json()["command_id"] == row["command_id"]
    conflict = direct_rig.client.post("/operator/v2/actions/oem.z.clear", json=request({"4": 11, "5": 20}))
    assert conflict.status_code == 409, conflict.text
    assert direct_rig.delivered == ["clear"]


def test_drift_after_admission_is_rejected_by_dispatch_hook(direct_rig):
    direct_rig.before_leaf[0] = lambda: direct_rig.epochs.update({"4": 11})
    r = direct_rig.client.post("/operator/v2/actions/oem.z.clear", json=request())
    assert r.status_code == 200, r.text
    row = settle(direct_rig, r.json()["command_id"])
    assert row["status"] != "completed", row
    assert direct_rig.delivered == []


def test_empty_map_remains_compatible(direct_rig):
    r = direct_rig.client.post("/operator/v2/actions/oem.z.clear", json=request({}))
    assert r.status_code == 200, r.text
    assert settle(direct_rig, r.json()["command_id"])["status"] == "completed"
    assert direct_rig.delivered == ["clear"]


def test_durable_claim_binds_epoch_before_any_later_receipt_write(tmp_path):
    OEMRuntimeStore(tmp_path).close()
    store = OperatorReceiptStore(tmp_path)
    receipt = {"command_id": "offline-command", "idempotency_key": "offline-claim-key",
        "action_id": "oem.z.clear", "ownership_generation": 7,
        "requested_inputs": {}, "expected_board_epoch_by_board": {"4": 10}}
    claimed, created = store.claim(receipt)
    assert created
    assert claimed["expected_board_epoch_by_board"] == {"4": 10}
    replay, created = store.claim(receipt)
    assert not created and replay["command_id"] == claimed["command_id"]
    with pytest.raises(ValueError, match="idempotency key conflict"):
        store.claim({**receipt, "expected_board_epoch_by_board": {"4": 11}})


def test_request_digest_includes_supplied_epochs_only():
    base = {"action_id": "oem.z.clear", "requested_inputs": {}}
    assert request_digest({**base, "expected_board_epoch_by_board": {"4": 10}}) != request_digest(
        {**base, "expected_board_epoch_by_board": {"4": 11}})


@pytest.mark.parametrize("epochs", [{"04": 1}, {"4": -1}, {"4": True}])
def test_internal_expected_epochs_are_validated(epochs):
    with pytest.raises(ValueError):
        controls.InvokeRequest(expected_generation=7, idempotency_key="validation-key",
                               expected_board_epoch_by_board=epochs)
