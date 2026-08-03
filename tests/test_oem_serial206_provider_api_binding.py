from __future__ import annotations

import pytest
from fastapi import HTTPException
from pydantic import ValidationError

import bioxp.api as api
from bioxp import operator_controls


class PartialProvider:
    def __init__(self) -> None:
        self.z_calls = []

    def capability_status(self):
        return {
            "initialize_motors_live_available": True,
            "initialize_motion_live_available": False,
            "initialize_motion_partial_primitives": [],
            "initialize_motion_missing_primitives": ["initializeMotion.scriptmoveTo.tip_exists"],
        }

    def initialize_motion_projection(self):
        return {"initialize_motors": {"expected_next_stage": "z-home", "terminal_state": "pending"}}

    def initialize_motors_admission_projection(self):
        return {"available": True, "expected_stage": "z-home", "blockers": []}

    def z_projection(self):
        return {"available": True, "state": "prepared_unreferenced", "authority": type(self).__name__}

    def execute_z_intent(self, intent, **kwargs):
        self.z_calls.append((intent, kwargs))
        return {"ok": True, "z_state": "prepared_unreferenced", "authority_receipt": {"command_id": "z-1"}}

    def record_z_observation(self, **kwargs):
        self.z_calls.append(("observation", kwargs))
        return {"ok": True, "z_state": "referenced_ready"}


def test_z_observation_request_is_strict():
    assert set(api.OemZObservationRequest.model_fields) == {"command_id", "verdict", "note"}
    with pytest.raises(ValidationError):
        api.OemZObservationRequest(command_id="z-1", verdict=True, note="Observed")
    with pytest.raises(ValidationError):
        api.OemZObservationRequest(command_id="z-1", verdict="pass", note="   ")


def test_provider_status_is_truthful_for_unbound_and_partial_provider(monkeypatch):
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", None)
    unbound = api.serial206_oem_initialization_provider_status()
    assert unbound["bound"] is False
    assert unbound["z_authority"]["available"] is False

    partial = api.bind_serial206_oem_initialization_provider(PartialProvider())
    assert partial["bound"] is True
    assert partial["initialize_motors_live_available"] is True
    assert partial["z_authority"]["state"] == "prepared_unreferenced"


def test_z_mutations_are_rejected_outside_operator_dispatch_context(monkeypatch):
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", PartialProvider())
    with pytest.raises(HTTPException) as exc:
        api._execute_provider_z_intent("move_steps", {"steps": 10})
    assert exc.value.status_code == 410
    assert exc.value.detail["error"] == "direct_z_mutation_retired"


def test_operator_dispatch_context_reaches_provider_with_generation_and_idempotency(monkeypatch):
    provider = PartialProvider()
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", provider)
    token = operator_controls._DISPATCH_CONTEXT.set({
        "operator_command_id": "operator-1",
        "idempotency_key": "idem-1",
        "expected_ownership_generation": 11,
        "action_id": "oem.z.move_steps",
    })
    try:
        result = api._execute_provider_z_intent("move_steps", {"steps": 10})
    finally:
        operator_controls._DISPATCH_CONTEXT.reset(token)

    assert result["ok"] is True
    assert provider.z_calls == [("move_steps", {
        "inputs": {"steps": 10},
        "expected_generation": 11,
        "idempotency_key": "idem-1",
    })]


def test_usb_ownership_sync_binds_and_clears_production_provider(monkeypatch):
    sentinel = PartialProvider()
    monkeypatch.setattr(api, "_tester", object())
    monkeypatch.setattr(api, "_pipette_transport", object())
    monkeypatch.setattr(api, "_build_serial206_oem_initialization_provider", lambda: sentinel)
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", None)

    api._sync_serial206_oem_initialization_provider(transport="owned", usb="service", router="running")
    assert api._serial206_oem_initialization_provider is sentinel

    api._sync_serial206_oem_initialization_provider(transport="unbound", usb="released", router="stopped")
    assert api._serial206_oem_initialization_provider is None
