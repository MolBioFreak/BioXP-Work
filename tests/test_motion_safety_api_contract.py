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
    assert prepare["inputs"] == []

    emergency = actions["oem.abort_all"]
    assert emergency["informational_path"] == "/motion/oem/x/abort"
    assert emergency["provider_available"] is True
    assert emergency["enabled"] is True
    assert emergency["requires_confirmation"] is False


def test_legacy_inferred_power_routes_fail_closed_without_tester_access(monkeypatch):
    def forbidden():
        raise AssertionError("legacy quarantine must not touch USB/CAN")

    monkeypatch.setattr(api_module, "_get_tester", forbidden)
    for handler in (
        api_module.prepare_interlock,
        api_module.motion_power_diag,
    ):
        with pytest.raises(HTTPException) as raised:
            asyncio.run(handler())
        assert raised.value.status_code == 410
        assert raised.value.detail["physical_motion_commanded"] is False


def test_no_motion_prepare_and_power_alias_are_one_click_no_input_routes():
    paths = api_module.app.openapi()["paths"]
    for path in ("/motion/oem/prepare_without_motion", "/motion/power/enable"):
        operation = paths[path]["post"]
        assert "requestBody" not in operation
        assert operation.get("parameters", []) == []
