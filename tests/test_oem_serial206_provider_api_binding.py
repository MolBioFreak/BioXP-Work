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

    def z_command_lease(self):
        provider = self

        class Lease:
            def __enter__(self):
                provider.z_calls.append(("lease_enter", {}))

            def __exit__(self, exc_type, exc, traceback):
                provider.z_calls.append(("lease_exit", {}))

        return Lease()

    def execute_z_intent(self, intent, **kwargs):
        self.z_calls.append((intent, kwargs))
        return {"ok": True, "z_state": "prepared_unreferenced", "authority_receipt": {"command_id": "z-1"}}

    def record_z_observation(self, **kwargs):
        self.z_calls.append(("observation", kwargs))
        return {"ok": True, "z_state": "referenced_ready"}


def test_z_set_home_requires_explicit_current_position_confirmation():
    assert set(api.OemZSetHomeRequest.model_fields) == {"operator_ack", "note"}
    with pytest.raises(ValidationError):
        api.OemZSetHomeRequest(operator_ack="SET_HOME", note="Confirm")
    with pytest.raises(ValidationError):
        api.OemZSetHomeRequest(
            operator_ack="SET_HOME_CURRENT_POSITION",
            note="   ",
        )


def test_z_observation_request_is_strict():
    expected = {
        "command_id",
        "verdict",
        "physical_motion_observed",
        "expected_direction_observed",
        "home_endpoint_observed",
        "stopped_observed",
        "note",
    }
    assert set(api.OemZObservationRequest.model_fields) == expected
    base = {
        "command_id": "z-1",
        "verdict": "pass",
        "physical_motion_observed": True,
        "expected_direction_observed": True,
        "home_endpoint_observed": True,
        "stopped_observed": True,
        "note": "Observed",
    }
    with pytest.raises(ValidationError):
        api.OemZObservationRequest(**{**base, "verdict": True})
    with pytest.raises(ValidationError):
        api.OemZObservationRequest(**{**base, "physical_motion_observed": 1})
    with pytest.raises(ValidationError):
        api.OemZObservationRequest(**{**base, "note": "   "})


def test_provider_status_is_truthful_for_unbound_and_partial_provider(monkeypatch):
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", None)
    unbound = api.serial206_oem_initialization_provider_status()
    assert unbound["bound"] is False
    assert unbound["z_authority"]["available"] is False

    partial = api.bind_serial206_oem_initialization_provider(PartialProvider())
    assert partial["bound"] is True
    assert partial["initialize_motors_live_available"] is True
    assert partial["z_authority"]["state"] == "prepared_unreferenced"


def test_liquid_preflight_requires_provider_z_lifecycle_authority(monkeypatch):
    class References:
        @staticmethod
        def snapshot(axes):
            return {"rows": {axis: {"state": "referenced"} for axis in axes}}

    class Provider:
        @staticmethod
        def z_projection():
            return {
                "available": True,
                "state": "executing",
                "reference_state": "referenced",
                "board_lifecycle_generation_fresh": True,
            }

    monkeypatch.setattr(api, "_reference_state_store", References())
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", Provider())

    with pytest.raises(api.PipettePreflightError) as exc:
        api._liquid_reference_preflight("aspirate")

    assert "serial206_z_lifecycle_not_referenced_ready" in exc.value.details["authority_blockers"]


def test_z_mutations_are_rejected_outside_operator_dispatch_context(monkeypatch):
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", PartialProvider())
    with pytest.raises(HTTPException) as exc:
        api._execute_provider_z_intent("move_steps", {"steps": 10})
    assert exc.value.status_code == 410
    assert exc.value.detail["error"] == "direct_z_mutation_retired"


@pytest.mark.parametrize(
    ("intent", "inputs"),
    (
        ("move_steps", {"steps": 10}),
        ("move_absolute", {"position_steps": 500}),
    ),
)
def test_manual_z_move_dispatch_never_inserts_automatic_homing(monkeypatch, intent, inputs):
    provider = PartialProvider()
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", provider)
    token = operator_controls._DISPATCH_CONTEXT.set({
        "operator_command_id": "operator-1",
        "idempotency_key": "idem-1",
        "expected_ownership_generation": 11,
        "action_id": f"oem.z.{intent}",
    })
    try:
        result = api._execute_provider_z_intent(intent, inputs)
    finally:
        operator_controls._DISPATCH_CONTEXT.reset(token)

    assert result["ok"] is True
    assert provider.z_calls == [
        ("lease_enter", {}),
        (intent, {
            "inputs": {**inputs, "command_id": "operator-1"},
            "expected_generation": 11,
            "idempotency_key": "idem-1",
        }),
        ("lease_exit", {}),
    ]


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
