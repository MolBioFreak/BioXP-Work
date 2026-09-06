from __future__ import annotations

import asyncio

from bioxp import api


def test_raw_move_to_http_route_is_preview_only(monkeypatch) -> None:
    monkeypatch.setattr(api, "_require_motion_route_ready", lambda: (_ for _ in ()).throw(AssertionError("readiness/provider touched")))
    monkeypatch.setattr(api, "_run_blocking", lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("provider touched")))
    request = api.OemMoveToRequest(operator_ack="MOVETO", x=1, y=2, z=3, pseudo_z_home=500, run_in_parallel=True, timeout_s=1.0)
    result = asyncio.run(api.motion_oem_move_to(request))
    assert result["executor_status"] == "preview_only"
    assert result["motion_commanded"] is False
