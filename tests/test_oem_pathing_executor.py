import asyncio
from types import SimpleNamespace

import pytest

from src.bioxp import oem_homing_routes as routes


PLAN_MOVE_TO = {
    "ok": True,
    "schema_version": "bioxp.oem_scriptmove_path_plan.v1",
    "steps": [{
        "op": "moveTo",
        "x": 1506,
        "y": 71,
        "z": 65000,
        "move_to_authority": {
            "gripper_confirmed": True,
            "tip_loaded": False,
            "plate_on_gantry": None,
            "location19_y": 19000,
        },
    }],
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

    def oem_move_to(self, x, y, z, **kwargs):
        self.calls.append(("move_to", int(x), int(y), int(z), dict(kwargs)))
        return {"ok": True, "branch": "fixture_move_to"}

    def path_planning_authority(self):
        return {
            "ok": True,
            "current_loc": "LOC_MS",
            "current_well": 0,
            "current_x": 100,
            "current_y": 200,
            "current_z": 65000,
            "tip_loaded": False,
            "tip_dirty": False,
            "clean_path": False,
            "tip_location": -1,
            "gripper_confirmed": True,
            "plate_on_gantry": None,
            "pseudo_z_home": 65000,
            "source": "test-provider-authority",
        }


@pytest.fixture(autouse=True)
def patch_plan(monkeypatch):
    async def fake_plan(**kwargs):
        return dict(PLAN_MOVE_TO)

    monkeypatch.setattr(routes, "plan_oem_scriptmove_path", fake_plan)
    monkeypatch.setattr(
        routes,
        "load_bound_oem_position_table",
        lambda: SimpleNamespace(resolve=lambda **_kwargs: SimpleNamespace(coordinates={"y": 19000})),
    )


def test_scriptmove_execute_dry_run_remains_no_motion_preview():
    executor = FakeExecutor()
    payload = asyncio.run(
        routes._execute_oem_scriptmove_path_impl(
            {"location_id": "LOC_PARK", "mode": "dry_run"}, motion_executor=executor
        )
    )
    assert payload["ok"] is True
    assert payload["executor_status"] == "preview_only"
    assert payload["motion_commanded"] is False
    assert executor.calls == []


def test_scriptmove_execute_live_without_ack_is_preview_only_before_executor():
    executor = FakeExecutor()
    result = asyncio.run(
        routes._execute_oem_scriptmove_path_impl(
            {"location_id": "LOC_PARK", "mode": "live", "reason": "commissioning"},
            motion_executor=executor,
        )
    )
    assert result["executor_status"] == "preview_only"
    assert result["motion_commanded"] is False
    assert result["opened_usb"] is False
    assert result["legacy_live_execution_retired"] is True
    assert executor.calls == []


def test_scriptmove_execute_live_without_reason_is_preview_only_before_executor():
    executor = FakeExecutor()
    result = asyncio.run(
        routes._execute_oem_scriptmove_path_impl(
            {"location_id": "LOC_PARK", "mode": "live", "operator_ack": "OEM_PATH_EXECUTE"},
            motion_executor=executor,
        )
    )
    assert result["executor_status"] == "preview_only"
    assert result["motion_commanded"] is False
    assert result["opened_usb"] is False
    assert result["legacy_live_execution_retired"] is True
    assert executor.calls == []


def test_scriptmove_execute_live_with_legacy_ack_is_preview_only_and_zero_io():
    executor = FakeExecutor()
    result = asyncio.run(
        routes._execute_oem_scriptmove_path_impl(
            {
                "location_id": "LOC_PARK",
                "mode": "live",
                "operator_ack": "OEM_PATH_EXECUTE",
                "reason": "provider-owned commissioning path",
            },
            motion_executor=executor,
        )
    )
    assert result["ok"] is True
    assert result["executor_status"] == "preview_only"
    assert result["motion_commanded"] is False
    assert result["opened_usb"] is False
    assert result["legacy_live_execution_retired"] is True
    assert executor.calls == []


def test_scriptmove_live_preview_never_reads_or_calls_provider_authority(monkeypatch):
    captured = {}

    async def capture_plan(**kwargs):
        captured.update(kwargs)
        return dict(PLAN_MOVE_TO)

    monkeypatch.setattr(routes, "plan_oem_scriptmove_path", capture_plan)
    executor = FakeExecutor()
    result = asyncio.run(
        routes._execute_oem_scriptmove_path_impl(
            {
                "location_id": "LOC_PARK",
                "mode": "live",
                "operator_ack": "OEM_PATH_EXECUTE",
                "reason": "authority override proof",
                "current_x": 999999,
                "current_y": 999999,
                "current_z": 999999,
                "tip_loaded": True,
                "tip_dirty": True,
                "clean_path": True,
                "tip_location": 7,
                "gripper_confirmed": False,
                "plate_on_gantry": 5,
            },
            motion_executor=executor,
        )
    )
    assert result["executor_status"] == "preview_only"
    assert result["motion_commanded"] is False
    assert result["opened_usb"] is False
    assert result["legacy_live_execution_retired"] is True
    assert captured["current_x"] == 999999
    assert executor.calls == []
