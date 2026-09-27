"""No-motion OEM preparation must not erase separately collected observations."""
import asyncio
from types import SimpleNamespace

import pytest
from fastapi import HTTPException


@pytest.mark.parametrize("succeeds", [True, False])
def test_prepare_retains_collected_snapshot_and_original_age(monkeypatch, succeeds):
    from bioxp import api, hardware_status

    clock = [1000.0]
    monkeypatch.setattr(hardware_status, "time", SimpleNamespace(time=lambda: clock[0]))
    monkeypatch.setattr(
        hardware_status, "lifecycle_state",
        SimpleNamespace(transport_changed=lambda *args, **kwargs: None),
    )
    owner = hardware_status.HardwareStateOwner(fresh_for_s=30)
    owner.change_ownership(reason="test", transport="owned", usb="service", router="running")
    domains = ("transport", "boards", "latch", "chiller")
    collected = owner.collect(domains, {name: lambda context, name=name: {"domain": name} for name in domains})
    before = collected["snapshot"]
    monkeypatch.setattr(api, "hardware_state", owner)
    monkeypatch.setattr(api, "Serial206MotionAuthority", SimpleNamespace(from_active_snapshot=lambda: object()))
    tester = SimpleNamespace(motion_arm_confirm=lambda **kwargs: {"state": "armed"})
    monkeypatch.setattr(api, "_tester", tester)
    monkeypatch.setattr(api, "_get_tester", lambda: tester)
    monkeypatch.setattr(api, "_maintenance_state", {"motion_blocked": False, "recovery_required": False})
    monkeypatch.setattr(api, "_maintenance_state_payload", lambda: {})
    provider = SimpleNamespace(prepare_global_motion_without_motion=lambda *args, **kwargs: {
        "ok": succeeds, "generation": owner.ownership_epoch, "component_prepare_receipts": {},
        **({} if succeeds else {"failure": "offline_preparation_failure"}),
    })
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", provider)

    async def run_inline(label, operation, **kwargs):
        return operation()

    monkeypatch.setattr(api, "_run_blocking", run_inline)
    if succeeds:
        assert asyncio.run(api.motion_oem_prepare_without_motion())["ok"] is True
    else:
        with pytest.raises(HTTPException) as exc:
            asyncio.run(api.motion_oem_prepare_without_motion())
        assert exc.value.status_code == 409
    assert owner.completed_snapshot()["snapshot_id"] == before["snapshot_id"]
    assert owner.completed_snapshot()["domains"] == before["domains"]
    assert owner.project(*domains, include_lifecycle=False)["cache_state"] == "fresh"
    clock[0] += 31
    projection = owner.project(*domains, include_lifecycle=False)
    assert projection["snapshot_id"] == before["snapshot_id"]
    assert projection["cache_state"] == "stale"
    assert projection["freshness"]["age_s"] == 31
