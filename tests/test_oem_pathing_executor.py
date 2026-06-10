import asyncio

import pytest
from fastapi import HTTPException

from src.bioxp import oem_homing_routes as routes


PLAN_MOVE_TO = {
    "ok": True,
    "schema_version": "bioxp.oem_scriptmove_path_plan.v1",
    "steps": [
        {"op": "moveTo", "x": 1506, "y": 71, "z": 65000},
    ],
    "motion_commanded": False,
}


class FakeExecutor:
    def __init__(self, *, fail_axis=None):
        self.calls = []
        self.fail_axis = fail_axis

    def absolute(self, axis, position_steps, *, speed=None, acc=None, wait_timeout_s=12.0):
        self.calls.append(("absolute", axis, int(position_steps), speed, acc, wait_timeout_s))
        if axis == self.fail_axis:
            return {"ok": False, "axis": axis, "motion_failure": {"category": "fixture_failure"}}
        return {
            "ok": True,
            "axis": axis,
            "target_position": int(position_steps),
            "motion_truth": {"evidence_level": "controller_only", "physical_motion_confirmed": False},
        }

    def relative(self, axis, steps, *, speed=None, acc=None, wait_timeout_s=12.0):
        self.calls.append(("relative", axis, int(steps), speed, acc, wait_timeout_s))
        return {"ok": True, "axis": axis, "position_delta": int(steps)}

    def sleep(self, milliseconds):
        self.calls.append(("sleep", int(milliseconds)))
        return {"ok": True, "milliseconds": int(milliseconds)}


@pytest.fixture(autouse=True)
def patch_plan(monkeypatch):
    async def fake_plan(**kwargs):
        return dict(PLAN_MOVE_TO)

    monkeypatch.setattr(routes, "plan_oem_scriptmove_path", fake_plan)


def test_scriptmove_execute_dry_run_remains_no_motion_preview():
    executor = FakeExecutor()

    payload = asyncio.run(
        routes._execute_oem_scriptmove_path_impl(
            {"location_id": "LOC_PARK", "mode": "dry_run"},
            motion_executor=executor,
        )
    )

    assert payload["ok"] is True
    assert payload["executor_status"] == "preview_only"
    assert payload["motion_commanded"] is False
    assert executor.calls == []


def test_scriptmove_execute_live_rejects_without_ack_before_executor():
    executor = FakeExecutor()

    with pytest.raises(HTTPException) as exc:
        asyncio.run(
            routes._execute_oem_scriptmove_path_impl(
                {"location_id": "LOC_PARK", "mode": "live", "reason": "commissioning"},
                motion_executor=executor,
            )
        )

    assert exc.value.status_code == 409
    assert executor.calls == []


def test_scriptmove_execute_live_requires_reason_before_executor():
    executor = FakeExecutor()

    with pytest.raises(HTTPException) as exc:
        asyncio.run(
            routes._execute_oem_scriptmove_path_impl(
                {"location_id": "LOC_PARK", "mode": "live", "operator_ack": "OEM_PATH_EXECUTE"},
                motion_executor=executor,
            )
        )

    assert exc.value.status_code == 409
    assert "reason" in str(exc.value.detail).lower()
    assert executor.calls == []


def test_scriptmove_execute_live_runs_planned_move_steps_in_order():
    executor = FakeExecutor()

    payload = asyncio.run(
        routes._execute_oem_scriptmove_path_impl(
            {
                "location_id": "LOC_PARK",
                "mode": "live",
                "operator_ack": "OEM_PATH_EXECUTE",
                "reason": "supervised OEM parity commissioning",
                "wait_timeout_s": 7.5,
            },
            motion_executor=executor,
        )
    )

    assert payload["ok"] is True
    assert payload["executor_status"] == "live_step_execution_complete"
    assert payload["opened_usb"] is True
    assert payload["motion_commanded"] is True
    assert payload["physical_motion"] is False
    assert payload["execution_results"][0]["ok"] is True
    assert executor.calls == [
        ("absolute", "x", 1506, None, None, 7.5),
        ("absolute", "y", 71, None, None, 7.5),
        ("absolute", "z", 65000, None, None, 7.5),
    ]


def test_scriptmove_execute_live_fails_closed_on_first_step_failure():
    executor = FakeExecutor(fail_axis="y")

    payload = asyncio.run(
        routes._execute_oem_scriptmove_path_impl(
            {
                "location_id": "LOC_PARK",
                "mode": "live",
                "operator_ack": "OEM_PATH_EXECUTE",
                "reason": "supervised OEM parity commissioning",
            },
            motion_executor=executor,
        )
    )

    assert payload["ok"] is False
    assert payload["failed_closed"] is True
    assert payload["executor_status"] == "failed_closed_step_error"
    assert payload["motion_commanded"] is True
    assert [call[1] for call in executor.calls] == ["x", "y"]
