"""Offline catalog wire qualification. Capture replay is opt-in, never live IO."""
import asyncio
import copy
import json
import os
import time
from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from bioxp import operator_controls as controls
from tests.z_stop_fixtures import make_app


def compose(definitions, current):
    return [{**definition, **current["action_states"][index]}
            for definition, index in zip(definitions, current["action_state_indices"], strict=True)]


def test_actual_capture_lossless_actions_and_consumed_dashboard():
    path = os.environ.get("BIOXP_CATALOG_CAPTURE")
    if not path:
        pytest.skip("set BIOXP_CATALOG_CAPTURE to retained BMS full catalog")
    full = json.loads(Path(path).read_text())
    before = copy.deepcopy(full)
    for body in (full, full["canonical"]):
        definitions = controls._catalog_definitions(body["actions"])
        current = controls._catalog_assessment(body, controls._catalog_revision(body["actions"]))
        assert compose(definitions, current) == body["actions"]
        assert len(current["action_states"]) < len(body["actions"])
        dashboard = controls._catalog_dashboard(body["dashboard"])
        old = body["dashboard"].get("telemetry", body["dashboard"])
        new = dashboard.get("telemetry", dashboard)
        assert {k:v for k,v in new.items() if k != "pipettes"} == {k:v for k,v in old.items() if k != "pipettes"}
        for a, b in zip(old["pipettes"]["channels"], new["pipettes"]["channels"], strict=True):
            assert {k:v for k,v in a.items() if k not in {"last_transaction", "hardware_tip_status", "hardware_pressure"}} == {k:v for k,v in b.items() if k not in {"hardware_tip_status", "hardware_pressure"}}
            for key in ("hardware_tip_status", "hardware_pressure"):
                assert b[key] == {k:v for k,v in a[key].items() if k in {"ok", "hardware_truth_level", "tip_loaded", "pressure"}}
        assert new["pipettes"]["last_error"] == old["pipettes"]["last_error"]
        if "latest_receipts" in dashboard:
            assert dashboard["latest_receipts"] == body["dashboard"]["latest_receipts"]
    assert full == before


@pytest.mark.parametrize("enabled", [True, False, None])
def test_all_dynamic_fields_and_omission_survive(enabled):
    rows = [{"action_id": "a", "inputs": [{"enum": [False, 0, None]}],
             "enabled": enabled, "disabled_reason": None, "provider_available": False,
             "provider_unavailable_reason": "offline", "available": enabled,
             "unavailable_reason": "unknown", "dependencies": [{"key": "provider_available", "met": False, "reason": "offline"}],
             "snapshot_freshness": {"state": "missing", "age_s": None}, "future_dynamic": False},
            {"action_id": "b", "enabled": False}]
    revision = controls._catalog_revision(rows)
    current = controls._catalog_assessment({"actions": rows}, revision)
    assert compose(controls._catalog_definitions(rows), current) == rows
    rows[0]["enabled"] = not enabled
    rows[0].pop("snapshot_freshness")
    assert controls._catalog_revision(rows) == revision
    assert compose(controls._catalog_definitions(rows), controls._catalog_assessment({"actions": rows}, revision)) == rows


def test_compact_cache_preserves_failure_aging_and_generation():
    async def run():
        state = {"generation": 1, "fail": False}
        cache = controls._OperatorPollCache(lambda: state["generation"])
        async def view(view="assessment"):
            if state["fail"]:
                raise ValueError("provider failed")
            return controls._catalog_assessment({"ownership_generation": state["generation"], "actions": [{"action_id": "a", "enabled": True, "snapshot_freshness": {"state": "fresh", "age_s": 1, "fresh_for_s": 2}}]}, "revision")
        poll = cache.wrap(view)
        try:
            await poll(view="assessment")
            key = ("view", None, "assessment")
            body, stored = cache._cache[key]
            assert "actions" not in body
            cache._cache[key] = (body, stored - 10)
            state["fail"] = True
            result = await poll(view="assessment")
            assert result["action_states"][0]["enabled"] is True
            assert result["action_states"][0]["snapshot_freshness"]["state"] == "stale"
            # Drain the intentionally failed worker before replacing ownership.
            with pytest.raises(ValueError, match="provider failed"):
                await asyncio.wait_for(asyncio.wrap_future(cache._pending), 2)
            state.update(generation=2, fail=False)
            result = await poll(view="assessment")
            assert result["ownership_generation"] == 2
        finally:
            cache.close()
    asyncio.run(run())


def test_real_routes_metadata_does_not_collect_state_and_z_preview_unchanged(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    project = controls.hardware_state.project
    monkeypatch.setattr(controls.hardware_state, "project", lambda *args, include_lifecycle=False, **kwargs: project(*args, **kwargs))
    def get(client, path, params):
        for _ in range(100):
            response = client.get(path, params=params)
            if response.status_code != 503:
                assert response.status_code == 200, response.text
                return response.json()
            time.sleep(.01)
        pytest.fail("catalog stayed warming for the bounded test window")
    with TestClient(app) as client:
        cache = app.state.operator_poll_cache
        for path in ("/operator/control-catalog", "/operator/v2/control-catalog"):
            metadata = get(client, path, {"view": "metadata"})
            if path == "/operator/control-catalog":
                assert cache._pending is None
            full = get(client, path, {})
            current = get(client, path, {"view": "assessment"})
            assert compose(metadata["actions"], current) == full["actions"]
            assert "actions" not in current
        for z in (-2147483648, 0, 65000, 2147483647):
            full = get(client, "/operator/control-catalog", {"z_target_steps": z})
            current = get(client, "/operator/control-catalog", {"view": "assessment", "z_target_steps": z})
            assert current["dashboard"]["z_axis"]["provider"]["target_preview"] == full["dashboard"]["z_axis"]["provider"]["target_preview"]
        assert len(cache._cache) == 4  # finite full/compact V1/V2, not each Z target
