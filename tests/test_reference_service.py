import asyncio
import fcntl
import importlib
import sys
import types

import pytest
from fastapi import HTTPException

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



def test_reference_state_file_lock_propagates_inner_exceptions(tmp_path):
    store = ReferenceStateStore(state_path=tmp_path / "reference-state.json")

    with pytest.raises(RuntimeError, match="boom"):
        with store._state_file_lock(fcntl.LOCK_EX):
            raise RuntimeError("boom")



def test_reference_state_store_reset_clears_persisted_rows(tmp_path):
    state_path = tmp_path / "reference-state.json"
    store = ReferenceStateStore(state_path=state_path)
    store.mark_referenced(MarkAxisReferencedCommand(axis="x", position_steps=0, source="manual"))

    store.reset()

    reloaded = ReferenceStateStore(state_path=state_path)
    assert reloaded.snapshot(["x"])["rows"]["x"]["state"] == "unknown"



def test_reference_state_store_persist_failure_is_best_effort(monkeypatch, tmp_path):
    state_path = tmp_path / "reference-state.json"
    store = ReferenceStateStore(state_path=state_path)

    monkeypatch.setattr(type(state_path), "write_text", lambda self, text, *args, **kwargs: (_ for _ in ()).throw(OSError("disk full")))

    row = store.mark_referenced(MarkAxisReferencedCommand(axis="x", position_steps=0, source="manual"))

    assert row["state"] == "referenced"
    assert row["persisted"] is False
    assert store.snapshot(["x"])["rows"]["x"]["state"] == "referenced"
    assert not state_path.exists()



def test_reference_state_store_keeps_in_memory_updates_after_persist_failure(monkeypatch, tmp_path):
    state_path = tmp_path / "reference-state.json"
    state_path.write_text(
        '{"rows": {"x": {"axis": "x", "state": "desynced", "source": "stale", "note": "old", "updated_at": "2026-01-01T00:00:00+00:00"}}}'
    )
    store = ReferenceStateStore(state_path=state_path)

    monkeypatch.setattr(type(state_path), "write_text", lambda self, text, *args, **kwargs: (_ for _ in ()).throw(OSError("disk full")))

    row = store.mark_referenced(MarkAxisReferencedCommand(axis="x", position_steps=0, source="manual"))
    snapshot = store.snapshot(["x"])

    assert row["state"] == "referenced"
    assert snapshot["rows"]["x"]["state"] == "referenced"
    assert snapshot["rows"]["x"]["source"] == "manual"



def test_reference_state_store_persists_rows_to_disk(tmp_path):
    state_path = tmp_path / "reference-state.json"
    store = ReferenceStateStore(state_path=state_path)
    row = store.mark_referenced(MarkAxisReferencedCommand(axis="x", position_steps=0, source="startup_home"))

    reloaded = ReferenceStateStore(state_path=state_path)

    assert row["persisted"] is True
    assert reloaded.snapshot(["x"])["rows"]["x"]["state"] == "referenced"
    assert reloaded.snapshot(["x"])["rows"]["x"]["source"] == "startup_home"



def test_reference_state_persists_across_api_lifespan_restart(monkeypatch, tmp_path):
    api = load_api(monkeypatch)
    state_path = tmp_path / "reference-state.json"
    store = ReferenceStateStore(state_path=state_path)
    store.mark_referenced(MarkAxisReferencedCommand(axis="x", position_steps=0, source="manual"))

    monkeypatch.setattr(api, "_reference_state_store", store)
    monkeypatch.setattr(api, "BioXpTester", lambda alt=1: object())
    monkeypatch.setattr(api, "build_default_pipette_transport", lambda **kwargs: types.SimpleNamespace(close=lambda: None))

    async def exercise_lifespan():
        async with api.lifespan(None):
            assert api._reference_state_store.snapshot(["x"])["rows"]["x"]["state"] == "referenced"

    asyncio.run(exercise_lifespan())

    reloaded = ReferenceStateStore(state_path=state_path)
    assert reloaded.snapshot(["x"])["rows"]["x"]["state"] == "referenced"



def test_reference_state_store_ignores_invalid_persisted_rows(tmp_path):
    state_path = tmp_path / "reference-state.json"
    state_path.write_text('{"rows": {"x": {"axis": "x", "state": "bogus"}}}')

    store = ReferenceStateStore(state_path=state_path)

    assert store.snapshot(["x"])["rows"]["x"]["state"] == "unknown"



def test_reference_state_store_merges_interleaved_writers(tmp_path):
    state_path = tmp_path / "reference-state.json"
    store_a = ReferenceStateStore(state_path=state_path)
    store_b = ReferenceStateStore(state_path=state_path)

    store_a.mark_referenced(MarkAxisReferencedCommand(axis="x", position_steps=0, source="manual"))
    store_b.mark_referenced(MarkAxisReferencedCommand(axis="y", position_steps=10, source="manual"))

    snapshot_a = store_a.snapshot(["x", "y"])
    snapshot_b = store_b.snapshot(["x", "y"])
    reloaded = ReferenceStateStore(state_path=state_path)
    persisted = reloaded.snapshot(["x", "y"])

    for snapshot in (snapshot_a, snapshot_b, persisted):
        assert snapshot["rows"]["x"]["state"] == "referenced"
        assert snapshot["rows"]["y"]["state"] == "referenced"



def test_reference_state_store_merges_dirty_recovery_with_new_disk_rows(monkeypatch, tmp_path):
    state_path = tmp_path / "reference-state.json"
    original_write_text = type(state_path).write_text
    failure_count = {"remaining": 1}

    def flaky_write_text(self, text, *args, **kwargs):
        if self == state_path.with_suffix(".json.tmp") and failure_count["remaining"] > 0:
            failure_count["remaining"] -= 1
            raise OSError("disk full")
        return original_write_text(self, text, *args, **kwargs)

    monkeypatch.setattr(type(state_path), "write_text", flaky_write_text)

    store_a = ReferenceStateStore(state_path=state_path)
    store_b = ReferenceStateStore(state_path=state_path)

    store_a.mark_referenced(MarkAxisReferencedCommand(axis="x", position_steps=0, source="manual"))
    store_b.mark_referenced(MarkAxisReferencedCommand(axis="y", position_steps=10, source="manual"))
    store_a.record_motion("z", "jog")

    reloaded = ReferenceStateStore(state_path=state_path)
    persisted = reloaded.snapshot(["x", "y", "z"])

    assert persisted["rows"]["x"]["state"] == "referenced"
    assert persisted["rows"]["y"]["state"] == "referenced"
    assert persisted["rows"]["z"]["last_motion_kind"] == "jog"



def test_reference_state_store_dirty_recovery_prefers_newer_same_axis_disk_row(monkeypatch, tmp_path):
    state_path = tmp_path / "reference-state.json"
    original_write_text = type(state_path).write_text
    failure_count = {"remaining": 1}

    def flaky_write_text(self, text, *args, **kwargs):
        if self == state_path.with_suffix(".json.tmp") and failure_count["remaining"] > 0:
            failure_count["remaining"] -= 1
            raise OSError("disk full")
        return original_write_text(self, text, *args, **kwargs)

    monkeypatch.setattr(type(state_path), "write_text", flaky_write_text)

    store_a = ReferenceStateStore(state_path=state_path)
    store_b = ReferenceStateStore(state_path=state_path)

    store_a.mark_referenced(MarkAxisReferencedCommand(axis="x", position_steps=0, source="manual"))
    store_b.mark_desynced(MarkAxisDesyncedCommand(axis="x", reason="belt slip", source="operator"))
    row = store_a.record_motion("x", "jog")

    reloaded = ReferenceStateStore(state_path=state_path)
    persisted = reloaded.snapshot(["x"])

    assert row["persisted"] is True
    assert persisted["rows"]["x"]["state"] == "desynced"
    assert persisted["rows"]["x"]["note"] == "belt slip"
    assert persisted["rows"]["x"]["last_motion_kind"] == "jog"



def test_reference_state_store_failed_reset_recovery_preserves_disk_rows(monkeypatch, tmp_path):
    state_path = tmp_path / "reference-state.json"
    original_write_text = type(state_path).write_text

    seed = ReferenceStateStore(state_path=state_path)
    seed.mark_referenced(MarkAxisReferencedCommand(axis="legacy", position_steps=5, source="manual"))

    failure_count = {"remaining": 1}

    def flaky_write_text(self, text, *args, **kwargs):
        if self == state_path.with_suffix(".json.tmp") and failure_count["remaining"] > 0:
            failure_count["remaining"] -= 1
            raise OSError("disk full")
        return original_write_text(self, text, *args, **kwargs)

    monkeypatch.setattr(type(state_path), "write_text", flaky_write_text)

    store_a = ReferenceStateStore(state_path=state_path)
    store_b = ReferenceStateStore(state_path=state_path)

    store_a.reset()
    store_b.mark_referenced(MarkAxisReferencedCommand(axis="y", position_steps=10, source="manual"))
    store_a.mark_referenced(MarkAxisReferencedCommand(axis="x", position_steps=0, source="manual"))

    reloaded = ReferenceStateStore(state_path=state_path)
    persisted = reloaded.snapshot(["legacy", "x", "y"])

    assert persisted["rows"]["legacy"]["state"] == "referenced"
    assert persisted["rows"]["x"]["state"] == "referenced"
    assert persisted["rows"]["y"]["state"] == "referenced"



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


def test_motion_arm_strict_startup_rejects_monolithic_homing(monkeypatch):
    api = load_api(monkeypatch)

    async def fake_run_blocking(label, func, timeout_s=30.0):
        raise AssertionError("run_homing=True must be rejected before dispatch")

    monkeypatch.setattr(api, "_run_blocking", fake_run_blocking)
    monkeypatch.setattr(api, "_get_tester", lambda: object())

    with pytest.raises(HTTPException) as exc_info:
        asyncio.run(api.motion_arm_strict_startup(api.MotionArmStartupRequest(run_homing=True)))

    assert exc_info.value.status_code == 409
    assert "Monolithic strict_startup run_homing is disabled" in str(exc_info.value.detail)


def test_motion_hard_reset_marks_all_axes_desynced(monkeypatch):
    api = load_api(monkeypatch)
    recorded = []

    class FakeTester:
        def motor_hard_reset(self, *, rounds):
            return {"ok": True, "rounds": rounds}

    class FakeStore:
        def mark_desynced(self, command):
            recorded.append(command)
            return {"ok": True}

    monkeypatch.setattr(api, "_get_tester", lambda: FakeTester())
    monkeypatch.setattr(api, "_reference_state_store", FakeStore())
    monkeypatch.setattr(api, "_ownership_changed", lambda **kwargs: None)
    monkeypatch.setattr(api, "_mark_post_maintenance_motion_block", lambda **kwargs: {})

    result = asyncio.run(api.motion_hard_reset(api.MotionHardResetRequest(rounds=2)))

    assert result["ok"] is True
    assert {command.axis for command in recorded} == set(api.AxisName)
    assert all(command.source == "motion_hard_reset" for command in recorded)
