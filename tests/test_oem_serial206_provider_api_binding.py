from __future__ import annotations

import asyncio

import pytest
from pydantic import ValidationError

import bioxp.api as api


class PartialProvider:
    def __init__(self) -> None:
        self.observations = []
        self.motor_calls = []

    def capability_status(self):
        return {
            "initialize_motors_live_available": True,
            "initialize_motion_live_available": False,
            "initialize_motion_partial_primitives": ["initializeMotion.queryTipStatus.initial"],
            "initialize_motion_missing_primitives": ["initializeMotion.scriptmoveTo.tip_exists"],
        }

    def record_observation(self, **kwargs):
        self.observations.append(kwargs)
        return {"ok": True, "physical_motion_commanded": False, "movement_ledger": {"expected_next_stage": "gripper-current-31"}}

    def initialize_motors(self, **kwargs):
        self.motor_calls.append(kwargs)
        return {
            "ok": True,
            "ready": False,
            "state": "awaiting_operator_observation",
            "physical_motion_commanded": True,
            "stage_receipts": [{"stage": "z-home"}],
        }


def _observation_payload(**overrides):
    payload = {
        "stage": "z-home",
        "command_id": "approval-z-home",
        "expected_generation": 11,
        "observed_pass": True,
        "note": "Christian observed the commanded stage and terminal stop.",
    }
    payload.update(overrides)
    return payload


def test_api_request_contract_is_single_stage_and_observation_is_strict():
    assert "stage_approval" in api.OemInitializationRunRequest.model_fields
    assert "stage_approvals" not in api.OemInitializationRunRequest.model_fields
    assert set(api.OemSerial206ObservationRequest.model_fields) == {
        "stage", "command_id", "expected_generation", "observed_pass", "note"
    }
    with pytest.raises(ValidationError):
        api.OemSerial206ObservationRequest(**_observation_payload(observed_pass="true"))
    with pytest.raises(ValidationError):
        api.OemSerial206ObservationRequest(**_observation_payload(note="   "))


def test_provider_status_is_truthful_for_unbound_and_partial_provider(monkeypatch):
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", None)
    unbound = api.serial206_oem_initialization_provider_status()
    assert unbound["bound"] is False
    assert unbound["initialize_motors_live_available"] is False
    assert unbound["initialize_motion_live_available"] is False

    partial = api.bind_serial206_oem_initialization_provider(PartialProvider())
    assert partial["bound"] is True
    assert partial["initialize_motors_live_available"] is True
    assert partial["initialize_motion_live_available"] is False
    assert partial["initialize_motion_missing_primitives"] == ["initializeMotion.scriptmoveTo.tip_exists"]


def test_observation_route_never_invokes_hardware_readiness_or_primitive(monkeypatch):
    provider = PartialProvider()
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", provider)
    monkeypatch.setattr(
        api,
        "_require_motion_route_ready",
        lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("hardware readiness must not run")),
    )
    request = api.OemSerial206ObservationRequest(**_observation_payload())

    result = asyncio.run(api.motion_oem_initialization_observation(request))

    assert result["ok"] is True
    assert result["physical_motion_commanded"] is False
    assert provider.observations == [_observation_payload()]


def test_initialize_motors_route_passes_only_one_stage_approval(monkeypatch):
    provider = PartialProvider()
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", provider)
    monkeypatch.setattr(api, "_require_motion_route_ready", lambda *args, **kwargs: None)
    request = api.OemInitializationRunRequest(
        operator_ack="OEM_INITIALIZATION_RUN_WITH_HOMING",
        run_homing=True,
        stage_approval={
            "approval_id": "approval-z-home",
            "expected_generation": 11,
            "expected_component": "z",
            "expected_direction": "toward-gap9-home",
            "expected_bound": 160000,
            "operator_note": "Approved exact durable next stage only.",
            "idempotency_key": "idem-z-home",
        },
    )

    result = asyncio.run(api.motion_oem_initialization_run(request))

    assert result["state"] == "awaiting_operator_observation"
    assert len(provider.motor_calls) == 1
    assert isinstance(provider.motor_calls[0]["approval"], api.Serial206StageApproval)
    assert "approvals" not in provider.motor_calls[0]


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
