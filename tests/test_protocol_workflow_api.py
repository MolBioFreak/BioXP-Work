"""Finite API and retired ingress tests; run only with the denied-device runner."""
import pytest
from fastapi import FastAPI, HTTPException
from fastapi.testclient import TestClient
from pydantic import TypeAdapter, ValidationError

from bioxp import api
from bioxp.services.protocol_service import (
    ProtocolLiveContractError, ProtocolOperatorBundleStore, create_protocol_job,
    get_protocol_job, review_protocol_job, _job_status_from_state,
)
from bioxp.protocols.runtime_state import ProtocolRuntimeState, ProtocolWorkflowState


TARGET = {"command_id": "protocol-live-test", "idempotency_key": "original-control",
          "expected_ownership_generation": 4}


@pytest.mark.parametrize("variant", [
    {"action": "pause", "mode": "ordinary"}, {"action": "pause", "mode": "deferred"},
    {"action": "wake", "gate_id": "source:3"},
    {"action": "continue", "gate": "ordinary_pause", "gate_id": "source:3"},
    {"action": "continue", "gate": "deferred_pause", "gate_id": "source:3"},
    {"action": "continue", "gate": "delaypoint", "gate_id": "source:3"},
    {"action": "safe_stop"}, {"action": "abort"},
])
def test_finite_control_wire_roundtrip(variant):
    payload = {**TARGET, **variant}
    assert TypeAdapter(api.ProtocolControlRequest).validate_python(payload).model_dump() == payload


@pytest.mark.parametrize("invalid", [
    {"action": "abort", "waitforpipette": False},
    {"action": "continue", "gate": "review", "gate_id": "source:3"},
    {"action": "wake"}, {"action": "pause", "mode": "anything"},
    {"action": "native", "method": "parkGantry"},
    {"action": "abort", "expected_ownership_generation": True},
    {"action": "abort", "idempotency_key": "   "},
])
def test_control_rejects_extra_methods_wrong_gate_and_coerced_identity(invalid):
    with pytest.raises(ValidationError):
        TypeAdapter(api.ProtocolControlRequest).validate_python({**TARGET, **invalid})


@pytest.mark.parametrize("gate", ["ordinary_pause", "deferred_pause", "delaypoint", "review", "error_hold"])
def test_waiting_gate_is_not_terminal_or_review_status(gate):
    state = ProtocolRuntimeState(protocol_id="p", job_id="job", dry_run=False)
    state.workflow = ProtocolWorkflowState(command_id="job", phase="waiting", gate=gate, gate_id="g")
    state.paused = True
    assert _job_status_from_state(state) == "dispatched"


def test_dry_run_artifact_roundtrip_without_canonical_owner(tmp_path):
    store = ProtocolOperatorBundleStore(tmp_path)
    result = create_protocol_job({"document": {"protocol_id": "p", "stages": [
        {"stage_id": "one", "actions": [{"action_id": "note", "kind": "note"}]}]}}, store=store)
    assert result["execution"]["dry_run"] is True
    assert "command" not in result
    assert get_protocol_job(result["job_id"], store=store) == result
    assert not list(tmp_path.glob("*/idempotency-reservation.json"))


def test_no_live_execution_without_canonical_owner(tmp_path):
    with pytest.raises(ProtocolLiveContractError, match="custody"):
        create_protocol_job({"document": {"protocol_id": "p", "stages": [{"stage_id": "one", "actions": [{"action_id": "a", "kind": "note"}]}]}}, dry_run=False,
                            store=ProtocolOperatorBundleStore(tmp_path))


def test_historical_live_review_cannot_reconstruct_executor(tmp_path):
    store = ProtocolOperatorBundleStore(tmp_path)
    store.save({"job_id": "historical", "execution": {"dry_run": False}})
    with pytest.raises(ProtocolLiveContractError, match="cannot reconstruct"):
        review_protocol_job("historical", store=store)


def test_retired_worker_cannot_dispatch_live_or_write_parallel_history():
    from bioxp.oem_runtime_worker import OEMRuntimeWorker
    from bioxp.oem_runtime_types import OEMRuntimeCommand
    class NoWrites:
        def __getattr__(self, name):
            raise AssertionError("unexpected store write: " + name)
    called = []
    worker = OEMRuntimeWorker(store=NoWrites(), handlers={"validateJob": lambda command: called.append(command) or {"ok": True}}, autostart=True)
    result = worker.enqueue(OEMRuntimeCommand(name="validateJob", mode="live", operator_ack=True, artifact_root="/tmp/offline-unused"))
    assert result["ok"] is False and result["queued"] is False and called == []
    preview = worker.enqueue(OEMRuntimeCommand(name="validateJob", mode="dry_run"))
    assert preview["preview_only"] is True and preview["queued"] is False
    assert len(called) == 1
    assert worker.snapshot()["queue_depth"] == 0
    worker.stop()


def test_real_control_route_validation_and_refusal(monkeypatch):
    app = FastAPI()
    app.add_api_route("/protocol/jobs/{job_id}/control", api.protocol_job_control, methods=["POST"])
    client = TestClient(app)
    invalid = client.post("/protocol/jobs/protocol-live-test/control", json={**TARGET, "action": "continue", "gate": "review", "gate_id": "r"})
    assert invalid.status_code == 422
    def absent():
        raise HTTPException(status_code=503, detail={"error": "canonical_workflow_owner_unavailable"})
    monkeypatch.setattr(api, "_protocol_command_store", absent)
    refused = client.post("/protocol/jobs/protocol-live-test/control", json={**TARGET, "action": "abort"})
    assert refused.status_code == 503


def test_label_only_pause_does_not_modify_lifecycle():
    from bioxp.oem_runtime_events import OEMRuntimeEventRouter
    from bioxp.lifecycle_state import lifecycle_state
    before = lifecycle_state.projection()
    result = OEMRuntimeEventRouter(store=None, worker=None).handle_pause()
    assert result["ok"] is False
    assert lifecycle_state.projection() == before
