import asyncio
import importlib
import sys
import types

from src.bioxp.services.reference_service import (
    MarkAxisDesyncedCommand,
    MarkAxisReferencedCommand,
    ReferenceStateStore,
)


def load_api(monkeypatch):
    usb_pkg = types.ModuleType("usb")
    usb_core = types.ModuleType("usb.core")
    usb_util = types.ModuleType("usb.util")
    usb_pkg.core = usb_core
    usb_pkg.util = usb_util
    monkeypatch.setitem(sys.modules, "usb", usb_pkg)
    monkeypatch.setitem(sys.modules, "usb.core", usb_core)
    monkeypatch.setitem(sys.modules, "usb.util", usb_util)
    for name in [
        "src.bioxp.api",
        "src.bioxp.usb_driver",
        "src.bioxp.services.motion_service",
        "src.bioxp.services.reference_service",
        "src.bioxp.services",
        "src.bioxp",
    ]:
        sys.modules.pop(name, None)
    return importlib.import_module("src.bioxp.api")


def test_reference_state_store_tracks_reference_and_desync():
    store = ReferenceStateStore()

    referenced = store.mark_referenced(
        MarkAxisReferencedCommand(axis="x", position_steps=0, source="manual", note="camera aligned")
    )
    desynced = store.mark_desynced(
        MarkAxisDesyncedCommand(axis="x", reason="belt slip suspected", source="operator")
    )
    snapshot = store.snapshot(["x", "y"])

    assert referenced["state"] == "referenced"
    assert referenced["origin_position_steps"] == 0
    assert desynced["state"] == "desynced"
    assert desynced["note"] == "belt slip suspected"
    assert snapshot["rows"]["x"]["state"] == "desynced"
    assert snapshot["rows"]["y"]["state"] == "unknown"


def test_reference_state_store_records_motion_without_forcing_reference():
    store = ReferenceStateStore()

    row = store.record_motion("g", "relative")

    assert row["state"] == "unknown"
    assert row["last_motion_kind"] == "relative"
    assert store.snapshot(["g"])["rows"]["g"]["state"] == "unknown"


def test_reference_state_store_reset_clears_rows():
    store = ReferenceStateStore()
    store.mark_referenced(MarkAxisReferencedCommand(axis="x", position_steps=0))

    store.reset()

    assert store.snapshot(["x"])["rows"]["x"]["state"] == "unknown"


def test_reference_routes_delegate_to_store(monkeypatch):
    api = load_api(monkeypatch)
    calls = []

    class FakeStore:
        def snapshot(self, axes):
            axes = list(axes)
            calls.append(("snapshot", axes))
            return {"axes": axes, "rows": {axis.value: {"state": "unknown"} for axis in axes}}

        def mark_referenced(self, command):
            calls.append(("mark_referenced", command))
            return {"axis": command.axis.value, "state": "referenced"}

        def mark_desynced(self, command):
            calls.append(("mark_desynced", command))
            return {"axis": command.axis.value, "state": "desynced"}

    monkeypatch.setattr(api, "_reference_state_store", FakeStore())

    status = asyncio.run(api.motion_reference_status("x,y"))
    marked = asyncio.run(
        api.motion_reference_mark_referenced(
            api.ReferenceMarkRequest(axis=api.AxisName.X, position_steps=0, source="manual", note="trusted")
        )
    )
    desynced = asyncio.run(
        api.motion_reference_mark_desynced(
            api.ReferenceDesyncRequest(axis=api.AxisName.Y, reason="manual override", source="operator")
        )
    )

    assert status["axes"] == [api.AxisName.X, api.AxisName.Y]
    assert marked["state"] == "referenced"
    assert desynced["state"] == "desynced"
    assert calls[0][0] == "snapshot"
    assert calls[1][0] == "mark_referenced"
    assert calls[2][0] == "mark_desynced"


def test_motion_arm_strict_startup_marks_axes_referenced_when_homing_runs(monkeypatch):
    api = load_api(monkeypatch)
    recorded = []

    async def fake_run_blocking(label, func, timeout_s=30.0):
        del label, timeout_s
        return {"ok": True, "homing": {"x_home": {"ok": True}}}

    class FakeStore:
        def mark_referenced(self, command):
            recorded.append(command)
            return {"ok": True}

    monkeypatch.setattr(api, "_run_blocking", fake_run_blocking)
    monkeypatch.setattr(api, "_get_tester", lambda: object())
    monkeypatch.setattr(api, "_reference_state_store", FakeStore())

    result = asyncio.run(api.motion_arm_strict_startup(api.MotionArmStartupRequest(run_homing=True)))

    assert result["ok"] is True
    assert {command.axis for command in recorded} == set(api.AxisName)
    assert all(command.source == "motion_arm_strict_startup" for command in recorded)


def test_motion_hard_reset_marks_all_axes_desynced(monkeypatch):
    api = load_api(monkeypatch)
    recorded = []

    async def fake_run_blocking(label, func, timeout_s=30.0):
        del label, timeout_s
        return {"ok": True, "rounds": 2}

    class FakeStore:
        def mark_desynced(self, command):
            recorded.append(command)
            return {"ok": True}

    monkeypatch.setattr(api, "_run_blocking", fake_run_blocking)
    monkeypatch.setattr(api, "_get_tester", lambda: object())
    monkeypatch.setattr(api, "_reference_state_store", FakeStore())

    result = asyncio.run(api.motion_hard_reset(api.MotionHardResetRequest(rounds=2)))

    assert result["ok"] is True
    assert {command.axis for command in recorded} == set(api.AxisName)
    assert all(command.source == "motion_hard_reset" for command in recorded)
