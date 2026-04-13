import asyncio
import importlib
import json
import sys
import types
from pathlib import Path

import pytest
from fastapi import HTTPException

import src.bioxp.services.motion_service as motion_service_module
from src.bioxp.services.artifact_service import BIOXP_VALIDATION_ARTIFACT_ROOT_ENV
from src.bioxp.services.motion_service import (
    AbsoluteMoveCommand,
    HomeAxisCommand,
    MotionArtifactOptions,
    RelativeMoveCommand,
    dry_run_motion_response,
    run_absolute_motion_command,
    run_home_axis_command,
    run_relative_motion_command,
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


async def _fake_run_blocking(label, func, timeout_s=30.0):
    del label, timeout_s
    return func()


def test_run_relative_motion_command_requires_capture_for_dry_run():
    command = RelativeMoveCommand(
        axis="x",
        steps=5,
        wait_timeout_s=2.0,
        artifact=MotionArtifactOptions(capture_bundle=False, dry_run_bundle=True),
    )

    with pytest.raises(HTTPException) as exc_info:
        asyncio.run(
            run_relative_motion_command(
                command,
                get_tester=lambda: object(),
                run_blocking=_fake_run_blocking,
                execute_relative_move=lambda *args, **kwargs: {"ok": True},
            )
        )

    assert exc_info.value.status_code == 400
    assert "capture_bundle=true" in str(exc_info.value.detail)


def test_run_relative_motion_command_dry_run_writes_bundle(monkeypatch, tmp_path):
    root = tmp_path / "validation"
    monkeypatch.setenv(BIOXP_VALIDATION_ARTIFACT_ROOT_ENV, str(root))

    command = RelativeMoveCommand(
        axis="x",
        steps=12,
        wait_timeout_s=3.0,
        artifact=MotionArtifactOptions(
            capture_bundle=True,
            dry_run_bundle=True,
            operator_note="service dry-run",
            snapshot_refs=("before.png", "after.png"),
        ),
    )

    result = asyncio.run(
        run_relative_motion_command(
            command,
            get_tester=lambda: (_ for _ in ()).throw(AssertionError("hardware should not be touched")),
            run_blocking=_fake_run_blocking,
            execute_relative_move=lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("executor should not run")),
            dry_run_response_factory=dry_run_motion_response,
        )
    )

    assert result["dry_run"] is True
    assert result["artifact_bundle"]["dry_run"] is True
    metadata = json.loads(Path(result["artifact_bundle"]["metadata_path"]).read_text())
    assert metadata["operator_note"] == "service dry-run"
    assert metadata["snapshot_ref_count"] == 2


def test_run_relative_motion_command_dry_run_records_reuse_prepared_intent(monkeypatch, tmp_path):
    root = tmp_path / "validation"
    monkeypatch.setenv(BIOXP_VALIDATION_ARTIFACT_ROOT_ENV, str(root))

    command = RelativeMoveCommand(
        axis="x",
        steps=12,
        wait_timeout_s=3.0,
        reuse_prepared=True,
        artifact=MotionArtifactOptions(capture_bundle=True, dry_run_bundle=True),
    )

    result = asyncio.run(
        run_relative_motion_command(
            command,
            get_tester=lambda: (_ for _ in ()).throw(AssertionError("hardware should not be touched")),
            run_blocking=_fake_run_blocking,
            execute_relative_move=lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("executor should not run")),
            dry_run_response_factory=dry_run_motion_response,
        )
    )

    assert result["dry_run"] is True
    assert result["prep_policy"]["reuse_requested"] is True
    assert "audit only" in result["prep_policy"]["note"]


def test_run_relative_motion_command_preserves_live_success_when_bundle_write_fails(monkeypatch):
    command = RelativeMoveCommand(
        axis="x",
        steps=7,
        wait_timeout_s=2.5,
        artifact=MotionArtifactOptions(capture_bundle=True, dry_run_bundle=False),
    )

    def fake_bundle_writer(**kwargs):
        raise OSError("disk full")

    monkeypatch.setattr(motion_service_module, "create_motion_validation_bundle", fake_bundle_writer)

    result = asyncio.run(
        run_relative_motion_command(
            command,
            get_tester=lambda: "tester-object",
            run_blocking=_fake_run_blocking,
            execute_relative_move=lambda *args, **kwargs: {"axis": "x", "move": {"ok": True}},
        )
    )

    assert result["move"]["ok"] is True
    assert "artifact_bundle" not in result
    assert "disk full" in result["artifact_bundle_error"]


def test_run_absolute_motion_command_executes_blocking_move():
    observed = {}

    async def fake_run_blocking(label, func, timeout_s=30.0):
        observed["label"] = label
        observed["timeout_s"] = timeout_s
        return func()

    def fake_execute(tester, axis, position_steps, wait_timeout_s):
        observed["tester"] = tester
        observed["axis"] = axis
        observed["position_steps"] = position_steps
        observed["wait_timeout_s"] = wait_timeout_s
        return {"axis": "z", "target_position": position_steps, "wait": {"ok": True}}

    command = AbsoluteMoveCommand(axis="z", position_steps=321, wait_timeout_s=7.5)
    result = asyncio.run(
        run_absolute_motion_command(
            command,
            get_tester=lambda: "tester-object",
            run_blocking=fake_run_blocking,
            execute_absolute_move=fake_execute,
        )
    )

    assert result["target_position"] == 321
    assert observed["tester"] == "tester-object"
    assert observed["axis"] == "z"
    assert observed["position_steps"] == 321
    assert observed["wait_timeout_s"] == 7.5
    assert observed["timeout_s"] == 35.0


def test_run_home_axis_command_executes_blocking_move():
    observed = {}

    async def fake_run_blocking(label, func, timeout_s=30.0):
        observed["label"] = label
        observed["timeout_s"] = timeout_s
        return func()

    def fake_execute(tester, axis, speed, timeout_s):
        observed["tester"] = tester
        observed["axis"] = axis
        observed["speed"] = speed
        observed["timeout_s_arg"] = timeout_s
        return {"axis": "y", "home": {"ok": True}}

    command = HomeAxisCommand(axis="y", speed=44, timeout_s=8.0)
    result = asyncio.run(
        run_home_axis_command(
            command,
            get_tester=lambda: "tester-object",
            run_blocking=fake_run_blocking,
            execute_home_axis=fake_execute,
        )
    )

    assert result["home"]["ok"] is True
    assert observed["axis"] == "y"
    assert observed["speed"] == 44
    assert observed["timeout_s_arg"] == 8.0
    assert observed["timeout_s"] == 35.0


def test_run_relative_motion_command_dry_run_bundle_failure_raises(monkeypatch):
    command = RelativeMoveCommand(
        axis="x",
        steps=3,
        wait_timeout_s=1.0,
        artifact=MotionArtifactOptions(capture_bundle=True, dry_run_bundle=True),
    )

    def fake_bundle_writer(**kwargs):
        raise OSError("disk full")

    monkeypatch.setattr(motion_service_module, "create_motion_validation_bundle", fake_bundle_writer)

    with pytest.raises(OSError, match="disk full"):
        asyncio.run(
            run_relative_motion_command(
                command,
                get_tester=lambda: "tester-object",
                run_blocking=_fake_run_blocking,
                execute_relative_move=lambda *args, **kwargs: {"axis": "x", "move": {"ok": True}},
                dry_run_response_factory=dry_run_motion_response,
            )
        )


def test_move_axis_relative_route_delegates_to_motion_service(monkeypatch):
    api = load_api(monkeypatch)
    captured = {}

    async def fake_runner(command, **kwargs):
        captured["command"] = command
        captured["kwargs"] = kwargs
        return {"ok": True, "axis": "x"}

    monkeypatch.setattr(api, "run_relative_motion_command", fake_runner)

    req = api.MoveRelativeRequest(
        axis=api.AxisName.X,
        steps=9,
        wait_timeout_s=4.0,
        capture_bundle=True,
        operator_note="route smoke",
    )
    result = asyncio.run(api.move_axis_relative(req))

    assert result == {"ok": True, "axis": "x"}
    assert captured["command"].steps == 9
    assert captured["command"].artifact.capture_bundle is True
    assert captured["command"].artifact.operator_note == "route smoke"
    assert captured["kwargs"]["get_tester"] is api._get_tester
    assert captured["kwargs"]["run_blocking"] is api._run_blocking


def test_move_axis_absolute_route_records_motion_after_success(monkeypatch):
    api = load_api(monkeypatch)
    recorded = []

    async def fake_runner(command, **kwargs):
        return {"axis": getattr(command.axis, "value", command.axis), "target_position": 100, "wait": {"ok": True}}

    class FakeStore:
        def record_motion(self, axis, motion_kind):
            recorded.append((axis, motion_kind))
            return {"ok": True}

    monkeypatch.setattr(api, "run_absolute_motion_command", fake_runner)
    monkeypatch.setattr(api, "_reference_state_store", FakeStore())

    req = api.MoveAbsoluteRequest(axis=api.AxisName.GRIPPER, position_steps=100, wait_timeout_s=5.0)
    result = asyncio.run(api.move_axis_absolute(req))

    assert result["target_position"] == 100
    assert recorded == [(api.AxisName.GRIPPER, "absolute")]


def test_move_axis_relative_route_skips_reference_updates_for_dry_run(monkeypatch):
    api = load_api(monkeypatch)
    recorded = []

    async def fake_runner(command, **kwargs):
        return {"axis": getattr(command.axis, "value", command.axis), "dry_run": True, "move": {"ok": True}}

    class FakeStore:
        def record_motion(self, axis, motion_kind):
            recorded.append((axis, motion_kind))
            return {"ok": True}

    monkeypatch.setattr(api, "run_relative_motion_command", fake_runner)
    monkeypatch.setattr(api, "_reference_state_store", FakeStore())

    req = api.MoveRelativeRequest(axis=api.AxisName.X, steps=5, capture_bundle=True, dry_run_bundle=True)
    result = asyncio.run(api.move_axis_relative(req))

    assert result["dry_run"] is True
    assert recorded == []


def test_home_axis_route_marks_reference_after_success(monkeypatch):
    api = load_api(monkeypatch)
    recorded = []

    async def fake_runner(command, **kwargs):
        return {"axis": getattr(command.axis, "value", command.axis), "home": {"ok": True}}

    class FakeStore:
        def mark_referenced(self, command):
            recorded.append(command)
            return {"ok": True}

    monkeypatch.setattr(api, "run_home_axis_command", fake_runner)
    monkeypatch.setattr(api, "_reference_state_store", FakeStore())

    req = api.HomeAxisRequest(axis=api.AxisName.Y, timeout_s=6.0)
    result = asyncio.run(api.home_axis(req))

    assert result["home"]["ok"] is True
    assert len(recorded) == 1
    assert recorded[0].axis is api.AxisName.Y
    assert recorded[0].source == "home_axis"


def test_home_axis_route_skips_reference_mark_for_dry_run(monkeypatch):
    api = load_api(monkeypatch)
    recorded = []

    async def fake_runner(command, **kwargs):
        return {"axis": getattr(command.axis, "value", command.axis), "dry_run": True, "home": {"ok": True}}

    class FakeStore:
        def mark_referenced(self, command):
            recorded.append(command)
            return {"ok": True}

    monkeypatch.setattr(api, "run_home_axis_command", fake_runner)
    monkeypatch.setattr(api, "_reference_state_store", FakeStore())

    req = api.HomeAxisRequest(axis=api.AxisName.Z, capture_bundle=True, dry_run_bundle=True)
    result = asyncio.run(api.home_axis(req))

    assert result["dry_run"] is True
    assert recorded == []
