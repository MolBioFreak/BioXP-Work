from __future__ import annotations

import asyncio

from bioxp import oem_homing_routes


class ExplodingProvider:
    def __getattr__(self, name):
        raise AssertionError(f"provider I/O attempted: {name}")


def test_legacy_scriptmove_live_request_is_preview_only_without_provider_io(monkeypatch) -> None:
    async def fake_plan(**kwargs):
        return {"steps": [], "branch": "ordinary"}

    monkeypatch.setattr(oem_homing_routes, "plan_oem_scriptmove_path", fake_plan)
    result = asyncio.run(oem_homing_routes._execute_oem_scriptmove_path_impl({"mode": "live", "location_id": "LOC_OC"}, ExplodingProvider()))
    assert result["executor_status"] == "preview_only"
    assert result["motion_commanded"] is False
    assert result["opened_usb"] is False
    assert result["legacy_live_execution_retired"] is True


def test_ordinal_19_is_named_loc_rc_cover() -> None:
    from bioxp.oem_compat.pathing import LOCATION_ID_TO_NAME
    assert LOCATION_ID_TO_NAME[19] == "LOC_RC_COVER"
