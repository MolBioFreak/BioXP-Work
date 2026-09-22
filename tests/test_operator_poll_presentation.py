"""Operator poll presentation policy: serving aged evidence must never flip
live controls to disabled (operator directive 2026-09-21). Enablement is
re-evaluated live at submit time; the poll cache only serves evidence."""

import asyncio
import copy

import pytest
from fastapi import HTTPException

from bioxp.operator_controls import _OperatorPollCache


def _body():
    return {
        "schema_version": "bioxp.operator_control_catalog.v2",
        "ownership_generation": 1,
        "actions": [
            {
                "action_id": "oem.deck.move_to_location",
                "enabled": True,
                "disabled_reason": None,
                "destination_options": [
                    {"target": "LOC_PARK", "enabled": True, "disabled_reason": None},
                ],
                "freshness": {"state": "fresh", "age_s": 0.2, "fresh_for_s": 10.0},
            },
        ],
        "freshness": {"state": "fresh", "age_s": 0.2, "fresh_for_s": 10.0},
    }


def _run(coro):
    return asyncio.run(coro)


def test_aged_serve_keeps_enablement_and_ages_evidence():
    async def scenario():
        cache = _OperatorPollCache(ownership_generation_provider=lambda: 1)

        async def catalog_view():
            return copy.deepcopy(_body())

        poll = cache.wrap(catalog_view)
        first = await poll()
        assert first["actions"][0]["enabled"] is True
        # Age the cached body past the old sealed boundary (~16s) and past the
        # evidence fresh window (10s), then serve again.
        key = (catalog_view.__name__, None)
        body, stored_at = cache._cache[key]
        cache._cache[key] = (body, stored_at - 16.0)
        second = await poll()
        action = second["actions"][0]
        assert action["enabled"] is True
        assert action["disabled_reason"] is None
        assert all(option["enabled"] is True for option in action["destination_options"])
        # Evidence ages honestly: the served freshness crosses to stale.
        assert second["freshness"]["state"] == "stale"
        assert action["freshness"]["state"] == "stale"
        assert "cached_projection_stale" not in str(second)
        assert "Canonical motion snapshot is stale." not in str(second)

    _run(scenario())


def test_never_served_view_still_reports_refresh_failure():
    async def scenario():
        cache = _OperatorPollCache(ownership_generation_provider=lambda: 1)

        async def broken_view():
            raise ValueError("provider offline")

        poll = cache.wrap(broken_view)
        # Immediate waiter sees the underlying failure...
        with pytest.raises(ValueError):
            await poll()
        # ...and the never-served view then carries the sticky 503 refusal.
        with pytest.raises(HTTPException) as excinfo:
            await poll()
        assert excinfo.value.status_code == 503

    _run(scenario())


def test_served_view_never_goes_cold_from_failed_refresh():
    async def scenario():
        cache = _OperatorPollCache(ownership_generation_provider=lambda: 1)
        state = {"fail": False}

        async def flaky_view():
            if state["fail"]:
                raise ValueError("provider offline")
            return copy.deepcopy(_body())

        poll = cache.wrap(flaky_view)
        await poll()
        state["fail"] = True
        # A failed refresh must not turn an already-served view cold; the last
        # known enablement is served unchanged.
        served = await poll()
        assert served["actions"][0]["enabled"] is True

    _run(scenario())
