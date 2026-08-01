from __future__ import annotations

import asyncio

import pytest
from fastapi import HTTPException

from bioxp import api as api_module
from bioxp.operator_controls import _build_catalog


def test_source_grounded_prepare_and_physical_stop_are_bound_operator_actions():
    actions_list, _ = _build_catalog(api_module.app)
    actions = {row["action_id"]: row for row in actions_list}

    prepare = actions["meta.activate_motion"]
    assert prepare["informational_path"] == "/motion/oem/prepare_without_motion"
    assert prepare["provider_available"] is True
    assert prepare["enabled"] is True
    assert {row["wire_name"] for row in prepare["inputs"]} == {"operator_ack", "operator_reason"}

    emergency = actions["meta.emergency_stop"]
    assert emergency["informational_path"] == "/motion/emergency_stop"
    assert emergency["provider_available"] is True
    assert emergency["enabled"] is True
    assert emergency["requires_confirmation"] is False


def test_legacy_inferred_power_routes_fail_closed_without_tester_access(monkeypatch):
    def forbidden():
        raise AssertionError("legacy quarantine must not touch USB/CAN")

    monkeypatch.setattr(api_module, "_get_tester", forbidden)
    for handler in (
        api_module.prepare_interlock,
        api_module.motion_power_enable,
        api_module.motion_power_diag,
    ):
        with pytest.raises(HTTPException) as raised:
            asyncio.run(handler())
        assert raised.value.status_code == 410
        assert raised.value.detail["physical_motion_commanded"] is False


def test_no_motion_prepare_requires_explicit_ack_before_authority_or_tester(monkeypatch):
    def forbidden():
        raise AssertionError("invalid acknowledgement must reject before provider access")

    monkeypatch.setattr(api_module.Serial206MotionAuthority, "from_active_snapshot", forbidden)
    monkeypatch.setattr(api_module, "_get_tester", forbidden)
    request = api_module.MotionPrepareWithoutMotionRequest(
        operator_ack="WRONG",
        operator_reason="commissioning preflight",
    )
    with pytest.raises(HTTPException) as raised:
        asyncio.run(api_module.motion_oem_prepare_without_motion(request))
    assert raised.value.status_code == 409
    assert raised.value.detail["physical_motion_commanded"] is False
