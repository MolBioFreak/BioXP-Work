import asyncio
import fcntl
import importlib
import sys
import tempfile
import types

import pytest
from fastapi import HTTPException

from src.bioxp.oem_runtime_store import OEMRuntimeStore
from src.bioxp.services.reference_service import (
    MarkAxisDesyncedCommand,
    MarkAxisReferencedCommand,
    ReferenceStateStore,
)


@pytest.fixture(autouse=True)
def _canonical_runtime_state(tmp_path):
    runtime = OEMRuntimeStore(tmp_path)
    runtime.close()


def load_api(monkeypatch):
    runtime_root = tempfile.mkdtemp(prefix="bioxp-reference-api-")
    monkeypatch.setenv("BIOXP_RUNTIME_STATE_ROOT", runtime_root)
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", runtime_root)
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


def test_reference_state_store_tracks_reference_and_desync(tmp_path):
    store = ReferenceStateStore(state_path=tmp_path / "bioxp_runtime.db")

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


def test_reference_state_store_records_motion_without_forcing_reference(tmp_path):
    store = ReferenceStateStore(state_path=tmp_path / "bioxp_runtime.db")

    row = store.record_motion("g", "relative")

    assert row["state"] == "unknown"
    assert row["last_motion_kind"] == "relative"
    assert store.snapshot(["g"])["rows"]["g"]["state"] == "unknown"


def test_reference_state_store_reset_clears_rows(tmp_path):
    store = ReferenceStateStore(state_path=tmp_path / "bioxp_runtime.db")
    store.mark_referenced(MarkAxisReferencedCommand(axis="x", position_steps=0))

    store.reset()

    assert store.snapshot(["x"])["rows"]["x"]["state"] == "unknown"



def test_reference_state_file_lock_propagates_inner_exceptions(tmp_path):
    store = ReferenceStateStore(state_path=tmp_path / "bioxp_runtime.db")

    with pytest.raises(RuntimeError, match="boom"):
        with store._state_file_lock(fcntl.LOCK_EX):
            raise RuntimeError("boom")



def test_reference_state_store_reset_clears_persisted_rows(tmp_path):
    state_path = tmp_path / "bioxp_runtime.db"
    store = ReferenceStateStore(state_path=state_path)
    store.mark_referenced(MarkAxisReferencedCommand(axis="x", position_steps=0, source="manual"))

    store.reset()

    reloaded = ReferenceStateStore(state_path=state_path)
    assert reloaded.snapshot(["x"])["rows"]["x"]["state"] == "unknown"



def test_reference_state_store_persists_rows_to_disk(tmp_path):
    state_path = tmp_path / "bioxp_runtime.db"
    store = ReferenceStateStore(state_path=state_path)
    row = store.mark_referenced(MarkAxisReferencedCommand(axis="x", position_steps=0, source="startup_home"))

    reloaded = ReferenceStateStore(state_path=state_path)

    assert row["persisted"] is True
    assert reloaded.snapshot(["x"])["rows"]["x"]["state"] == "referenced"
    assert reloaded.snapshot(["x"])["rows"]["x"]["source"] == "startup_home"



def test_reference_state_persists_across_api_lifespan_restart(monkeypatch, tmp_path):
    api = load_api(monkeypatch)
    state_path = tmp_path / "bioxp_runtime.db"
    store = ReferenceStateStore(state_path=state_path)
    store.mark_referenced(MarkAxisReferencedCommand(axis="x", position_steps=0, source="manual"))

    monkeypatch.setattr(api, "_reference_state_store", store)
    monkeypatch.setattr(api, "BioXpTester", lambda alt=1: object())
    monkeypatch.setattr(api, "build_default_pipette_transport", lambda **kwargs: types.SimpleNamespace(close=lambda: None))

    async def exercise_lifespan():
        async with api.lifespan(api.app):
            assert api._reference_state_store.snapshot(["x"])["rows"]["x"]["state"] == "referenced"

    asyncio.run(exercise_lifespan())

    reloaded = ReferenceStateStore(state_path=state_path)
    assert reloaded.snapshot(["x"])["rows"]["x"]["state"] == "referenced"



def test_reference_state_store_merges_interleaved_writers(tmp_path):
    state_path = tmp_path / "bioxp_runtime.db"
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


def test_reference_store_without_durable_path_is_untrusted():
    store = ReferenceStateStore()

    result = store.mark_referenced(
        MarkAxisReferencedCommand(axis="x", position_steps=0, source="manual")
    )
    snapshot = store.snapshot(["x"])

    assert result["ok"] is False
    assert result["durable_clean"] is False
    assert snapshot["ok"] is False
    assert snapshot["durable_clean"] is False
    assert snapshot["rows"]["x"]["state"] == "unknown"


def test_reference_store_lock_failure_is_fail_closed(monkeypatch, tmp_path):
    state_path = tmp_path / "bioxp_runtime.db"
    store = ReferenceStateStore(state_path=state_path)

    def fail_open(*args, **kwargs):
        raise OSError("lock unavailable")

    monkeypatch.setattr(type(store._state_lock_path), "open", fail_open)

    with pytest.raises(RuntimeError, match="lock unavailable"):
        with store._state_file_lock(fcntl.LOCK_EX):
            pass


def test_reference_store_batch_publishes_once_and_rereads(tmp_path):
    store = ReferenceStateStore(state_path=tmp_path / "bioxp_runtime.db")
    calls = {"persist": 0}
    original = store._persist_locked

    def counted_persist():
        calls["persist"] += 1
        return original()

    store._persist_locked = counted_persist
    result = store.mark_referenced_many([
        MarkAxisReferencedCommand(axis="x", position_steps=0, source="home_xy"),
        MarkAxisReferencedCommand(axis="y", position_steps=0, source="home_xy"),
    ])

    assert result["ok"] is True
    assert result["durable_clean"] is True
    assert calls["persist"] == 1
    assert {row["axis"] for row in result["rows"]} == {"x", "y"}
