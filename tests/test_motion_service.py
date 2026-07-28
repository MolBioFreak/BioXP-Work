import asyncio
import importlib
import json
import sys
import threading
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
    assert result["artifact_bundle_error"]["category"] == "validation_artifact_unavailable"
    assert "disk full" in result["artifact_bundle_error"]["message"]
    assert result["artifact_bundle_error"]["hardware_motion_commanded"] is True


def test_run_absolute_motion_command_executes_blocking_move_with_profile_override():
    observed = {}

    async def fake_run_blocking(label, func, timeout_s=30.0):
        observed["label"] = label
        observed["timeout_s"] = timeout_s
        return func()

    def fake_execute(tester, axis, position_steps, wait_timeout_s, *, speed=None, acc=None):
        observed["tester"] = tester
        observed["axis"] = axis
        observed["position_steps"] = position_steps
        observed["wait_timeout_s"] = wait_timeout_s
        observed["speed"] = speed
        observed["acc"] = acc
        return {"axis": "z", "target_position": position_steps, "wait": {"ok": True}}

    command = AbsoluteMoveCommand(axis="z", position_steps=321, wait_timeout_s=7.5, speed=200, acc=80)
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
    assert observed["speed"] == 200
    assert observed["acc"] == 80
    assert observed["timeout_s"] == 35.0


def test_run_home_axis_command_executes_blocking_move():
    observed = {}

    async def fake_run_blocking(label, func, timeout_s=30.0):
        observed["label"] = label
        observed["timeout_s"] = timeout_s
        return func()

    def fake_execute(tester, axis, speed, timeout_s, **kwargs):
        observed["tester"] = tester
        observed["axis"] = axis
        observed["speed"] = speed
        observed["timeout_s_arg"] = timeout_s
        observed["execute_kwargs"] = kwargs
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


def test_run_relative_motion_command_dry_run_bundle_failure_is_structured_not_500(monkeypatch):
    command = RelativeMoveCommand(
        axis="x",
        steps=3,
        wait_timeout_s=1.0,
        artifact=MotionArtifactOptions(capture_bundle=True, dry_run_bundle=True),
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
            dry_run_response_factory=dry_run_motion_response,
        )
    )

    assert result["dry_run"] is True
    assert result.get("ok") is False
    assert "artifact_bundle" not in result
    assert result["artifact_bundle_error"]["category"] == "validation_artifact_unavailable"
    assert "disk full" in result["artifact_bundle_error"]["message"]


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
        speed=200,
        acc=80,
        capture_bundle=True,
        operator_note="route smoke",
    )
    result = asyncio.run(api.move_axis_relative(req))

    assert result == {"ok": True, "axis": "x"}
    assert captured["command"].steps == 9
    assert captured["command"].speed == 200
    assert captured["command"].acc == 80
    assert captured["command"].artifact.capture_bundle is True
    assert captured["command"].artifact.operator_note == "route smoke"
    assert captured["kwargs"]["get_tester"] is api._get_tester
    assert captured["kwargs"]["run_blocking"] is api._run_blocking


def test_move_axis_absolute_route_records_motion_after_success(monkeypatch):
    api = load_api(monkeypatch)
    recorded = []

    async def fake_runner(command, **kwargs):
        assert command.speed == 200
        assert command.acc == 80
        return {"axis": getattr(command.axis, "value", command.axis), "target_position": 100, "wait": {"ok": True}, "motion_evidence": {"classification": {"controller_motion_evidence": True}}}

    class FakeStore:
        def record_motion(self, axis, motion_kind):
            recorded.append((axis, motion_kind))
            return {"ok": True}

    monkeypatch.setattr(api, "run_absolute_motion_command", fake_runner)
    monkeypatch.setattr(api, "_reference_state_store", FakeStore())

    req = api.MoveAbsoluteRequest(axis=api.AxisName.GRIPPER, position_steps=100, wait_timeout_s=5.0, speed=200, acc=80)
    result = asyncio.run(api.move_axis_absolute(req))

    assert result["target_position"] == 100
    assert recorded == [(api.AxisName.GRIPPER, "absolute")]


def test_execute_absolute_move_applies_profile_override_before_motion(monkeypatch):
    api = load_api(monkeypatch)

    class FakeReferenceStore:
        def snapshot(self, axes):
            return {"rows": {ax.value if hasattr(ax, "value") else ax: {"axis": ax.value if hasattr(ax, "value") else ax, "state": "referenced"} for ax in axes}}

    monkeypatch.setattr(api, "_reference_state_store", FakeReferenceStore())

    class FakeTester:
        def motor_function_preset(self, key):
            assert key == "x"
            return {
                "board": 5,
                "motor": 0,
                "speed": 100,
                "acc": 50,
                "run_current": 31,
                "standby_current": 20,
            }

        def motor_normalize_speed_acc(self, board, *, motor=0, speed=None, acc=None):
            assert (board, motor, speed, acc) == (5, 0, 200, 80)
            return {"axis_key": "x", "speed": 200, "acc": 80, "changes": []}

        def motion_arm_state(self):
            return {"armed": True, "reason": "test"}

        def motion_gate_live_snapshot(self):
            return {"ok": True}

        def activate_boards(self, expect_reply=True):
            return {5: "ok"}

        def motor_prepare_motion_interlock(self, force_lock=True):
            return {"ok": True}

        def motor_prepare_axis(self, board, **kwargs):
            assert board == 5
            assert kwargs["speed"] == 200
            assert kwargs["acc"] == 80
            return {"ok": True, "kwargs": kwargs}

        def motor_get_position(self, board, *, motor=0):
            return {"position": 0}

        def motor_get_switch_activity(self, board, *, motor=0):
            return {"left_active": False, "right_active": False}

        def motor_move_absolute(self, board, position, *, motor=0):
            assert (board, position, motor) == (5, 91869, 0)
            return {"ok": True}

    monkeypatch.setattr(
        api,
        "_wait_for_motion_with_guardrails",
        lambda tester, board, motor, timeout_s: {
            "ok": True,
            "position_after": {"position": 91869},
            "switch_activity_after": {"left_active": False, "right_active": True},
        },
    )

    result = api._execute_absolute_move(FakeTester(), api.AxisName.X, 91869, 30.0, speed=200, acc=80)

    assert result["motion_profile"]["speed"] == 200
    assert result["motion_profile"]["acc"] == 80
    assert result["motion_profile"]["requested_speed"] == 200
    assert result["motion_profile"]["requested_acc"] == 80


def test_protocol_live_move_handler_executes_absolute_profile_override(monkeypatch):
    api = load_api(monkeypatch)
    recorded = []

    class Action:
        action_id = "move-x-limit"
        params = {"axis": "x", "position_steps": 91869, "wait_timeout_s": 30, "speed": 200, "acc": 80}

    monkeypatch.setattr(api, "_get_tester", lambda: "tester")

    def fake_execute_absolute(tester, axis, position_steps, wait_timeout_s, *, speed=None, acc=None):
        assert tester == "tester"
        assert axis == api.AxisName.X
        assert position_steps == 91869
        assert wait_timeout_s == 30.0
        assert speed == 200
        assert acc == 80
        return {"ok": True, "motion_profile": {"speed": 200, "acc": 80}}

    class FakeStore:
        def record_motion(self, axis, motion_kind):
            recorded.append((axis, motion_kind))
            return {"ok": True}

    monkeypatch.setattr(api, "_execute_absolute_move", fake_execute_absolute)
    monkeypatch.setattr(api, "_reference_state_store", FakeStore())

    result = api._protocol_live_move_handler(Action(), None)

    assert result["ok"] is True
    assert result["move_mode"] == "absolute"
    assert result["move"]["motion_profile"] == {"speed": 200, "acc": 80}
    assert recorded == [(api.AxisName.X, "protocol_absolute")]


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


def test_move_axis_zero_route_moves_to_absolute_zero(monkeypatch):
    api = load_api(monkeypatch)
    recorded = []

    async def fake_runner(command, **kwargs):
        recorded.append(command)
        return {"axis": getattr(command.axis, "value", command.axis), "move": {"ok": True, "mode": "absolute"}, "motion_evidence": {"classification": {"controller_motion_evidence": True}}}

    class FakeStore:
        def record_motion(self, axis, motion_kind):
            recorded.append((axis, motion_kind))
            return {"ok": True}

    monkeypatch.setattr(api, "run_absolute_motion_command", fake_runner)
    monkeypatch.setattr(api, "_reference_state_store", FakeStore())

    req = api.MoveAxisZeroRequest(axis=api.AxisName.Y, wait_timeout_s=6.0)
    result = asyncio.run(api.move_axis_zero(req))

    assert recorded[0].position_steps == 0
    assert result["move"]["ok"] is True
    assert result["route_semantics"]["home_semantics"] == "not_homing_absolute_zero_only"
    assert result["route_semantics"]["oem_switch_search_homing_executed"] is False
    assert recorded[1] == (api.AxisName.Y, "absolute_zero")


def test_home_axis_route_preserves_oem_switch_search_and_marks_reference(monkeypatch):
    api = load_api(monkeypatch)
    recorded = []

    async def fake_runner(command, **kwargs):
        recorded.append(command)
        return {"axis": getattr(command.axis, "value", command.axis), "home": {"ok": True}, "motion_evidence": {"classification": {"controller_motion_evidence": True}}}

    class FakeStore:
        def mark_referenced(self, command):
            recorded.append(command)
            return {"ok": True}

    monkeypatch.setattr(api, "run_home_axis_command", fake_runner)
    monkeypatch.setattr(api, "_reference_state_store", FakeStore())

    req = api.HomeAxisRequest(axis=api.AxisName.Y, timeout_s=6.0)
    result = asyncio.run(api.home_axis(req))

    assert result["home"]["ok"] is True
    assert result["route_semantics"]["source_command"] == "home_axis_manual_button_goHome_guarded"
    assert result["route_semantics"]["home_semantics"] == "manual_goHome_style_switch_search_not_startup_axisSearchHome_not_zero"
    assert result["route_semantics"]["oem_switch_search_homing_executed"] is True
    assert recorded[0].axis is api.AxisName.Y
    assert recorded[1].axis is api.AxisName.Y
    assert recorded[1].source == "home_axis"
    assert recorded[1].motion_kind == "home"


def test_home_axis_route_skips_reference_mark_for_dry_run(monkeypatch):
    api = load_api(monkeypatch)
    recorded = []

    async def fake_runner(command, **kwargs):
        recorded.append(command)
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
    assert result["route_semantics"]["source_command"] == "home_axis_manual_button_goHome_guarded"
    assert result["route_semantics"]["home_semantics"] == "manual_goHome_style_switch_search_not_startup_axisSearchHome_not_zero"
    assert result["route_semantics"]["oem_switch_search_homing_executed"] is False
    assert recorded == [recorded[0]]


def test_dual_limit_switch_state_is_readback_only_for_supervised_motion(monkeypatch):
    api = load_api(monkeypatch)

    api._guard_direction(
        api.AxisName.Z,
        0,
        {"left_active": True, "right_active": True},
        {"disable_left": True, "disable_right": True},
    )
    api._guard_direction(
        api.AxisName.X,
        100,
        {"left_active": True, "right_active": True},
        {"disable_left": False, "disable_right": False},
    )


def test_execute_relative_move_returns_structured_no_motion_failure_instead_of_409(monkeypatch):
    api = load_api(monkeypatch)

    class FakeReferenceStore:
        def snapshot(self, axes):
            return {"rows": {ax.value if hasattr(ax, "value") else ax: {"axis": ax.value if hasattr(ax, "value") else ax, "state": "referenced"} for ax in axes}}

    monkeypatch.setattr(api, "_reference_state_store", FakeReferenceStore())

    class FakeTester:
        def motor_function_preset(self, key):
            assert key == "x"
            return {
                "board": 5,
                "motor": 0,
                "speed": 100,
                "acc": 50,
                "run_current": 31,
                "standby_current": 31,
            }

        def motion_arm_state(self):
            return {"armed": True, "reason": "test"}

        def motion_gate_live_snapshot(self):
            return {"ok": True}

        def activate_boards(self, expect_reply=True):
            return {5: {"status": 100}}

        def motor_prepare_motion_interlock(self, force_lock=True):
            return {"ok": True}

        def motor_prepare_axis(self, board, **kwargs):
            return {"ok": True, "board": board, "kwargs": kwargs}

        def motor_get_position(self, board, *, motor=0):
            return {"position": 1234}

        def motor_get_switch_activity(self, board, *, motor=0):
            return {"left_active": False, "right_active": True}

        def motor_move_relative(self, board, steps, *, motor=0):
            return {"ok": True, "board": board, "motor": motor, "steps": steps}

    monkeypatch.setattr(
        api,
        "_wait_for_motion_with_guardrails",
        lambda tester, board, motor, timeout_s: {
            "ok": False,
            "error": "motion command produced no nonzero speed before reporting stopped; treating as ambiguous/no physical motion.",
            "seen_nonzero": False,
            "position_after": {"position": 1234},
            "switch_activity_after": {"left_active": False, "right_active": True},
        },
    )

    result = api._execute_relative_move(FakeTester(), api.AxisName.X, 100, 12.0)

    assert result["ok"] is False
    assert result["motion_failure"]["category"] == "guardrail_no_motion"
    assert "no nonzero speed" in result["motion_failure"]["message"]
    assert result["motion_truth"]["physical_motion_confirmed"] is False
    assert result["wait"]["ok"] is False



def test_wait_for_motion_polls_oem_gap8_reached_position(monkeypatch):
    api = load_api(monkeypatch)

    class FakeTester:
        def __init__(self):
            self.speed_values = [100, 100, 0]
            self.position_values = [0, 500, 1000, 1000]
            self.gap8_calls = 0

        def motor_get_position(self, board, *, motor=0):
            value = self.position_values.pop(0) if self.position_values else 1000
            return {"board": board, "motor": motor, "position": value, "ok": True}

        def motor_get_speed(self, board, *, motor=0):
            value = self.speed_values.pop(0) if self.speed_values else 0
            return {"board": board, "motor": motor, "speed": value, "ok": True}

        def motor_get_switch_activity(self, board, *, motor=0):
            return {"left_active": False, "right_active": False}

        def motor_get_reached_position(self, board, *, motor=0):
            self.gap8_calls += 1
            return {"board": board, "motor": motor, "target_reached": 1, "ok": True}

    tester = FakeTester()
    result = api._wait_for_motion_with_guardrails(tester, 5, 0, timeout_s=5.0, poll_s=0.01)

    assert result["ok"] is True
    assert tester.gap8_calls >= 1
    assert result["reached_position_after"]["target_reached"] == 1
    assert result["log_tail"][-1]["target_reached"] == 1


def test_motion_evidence_reports_gap8_from_wait(monkeypatch):
    api = load_api(monkeypatch)

    evidence = api._build_motion_evidence(
        preset={"board": 5, "motor": 0},
        prep={},
        interlock={},
        position_before={"position": 0},
        switch_before={"left_active": False, "right_active": False},
        position_after={"position": 1000},
        switch_after={"left_active": False, "right_active": False},
        move={"ok": True},
        wait={
            "ok": True,
            "seen_nonzero": True,
            "last_speed": 0,
            "reached_position_after": {"target_reached": 1, "ok": True},
            "log_tail": [{"speed": 100, "position": 500, "target_reached": 0}, {"speed": 0, "position": 1000, "target_reached": 1}],
        },
        motion_profile={"speed": 200, "acc": 200},
        raw_events=[],
        event_capture_attempted=True,
    )

    assert evidence["telemetry"]["after"]["gap8_target_reached"]["value"] == 1
    assert evidence["classification"]["gap8_target_reached"] == 1
    assert evidence["classification"]["controller_target_confirmed"] is False
    assert evidence["classification"]["controller_motion_evidence"] is False


def test_home_motion_evidence_rejects_fake_position_speed_without_switch_transition(monkeypatch):
    api = load_api(monkeypatch)

    evidence = api._build_home_motion_evidence(
        preset={"board": 5, "motor": 0},
        prep={},
        interlock={},
        home={
            "ok": False,
            "position_before": {"position": 0},
            "position_after": {"position": 93241},
            "switch_activity_before": {"left_active": False, "right_active": False},
            "switch_activity_after": {"left_active": False, "right_active": False},
            "move_left": {"ok": True},
            "switch_transition": False,
            "log_tail": [
                {"speed": -200, "position": -10000, "home": 0},
                {"speed": -200, "position": -50000, "home": 0},
                {"speed": -200, "position": -93241, "home": 0},
            ],
        },
        motion_profile={"speed": 200, "acc": 200},
    )

    assert evidence["classification"]["nonzero_speed_seen"] is True
    assert evidence["classification"]["position_delta"] == 93241
    assert evidence["classification"]["controller_target_confirmed"] is False
    assert evidence["classification"]["controller_motion_evidence"] is False
    assert evidence["switch_transition"] is False


def test_home_motion_evidence_accepts_switch_transition_as_physical_proof(monkeypatch):
    api = load_api(monkeypatch)

    evidence = api._build_home_motion_evidence(
        preset={"board": 5, "motor": 0},
        prep={},
        interlock={},
        home={
            "ok": True,
            "position_before": {"position": 0},
            "position_after": {"position": -1000},
            "switch_activity_before": {"left_active": False, "right_active": False},
            "switch_activity_after": {"left_active": True, "right_active": False},
            "move_left": {"ok": True},
            "home_active_value": 1,
            "home_after": {"value": 1},
            "set_home": {"ok": True},
            "switch_transition": True,
            "log_tail": [
                {"speed": -200, "position": -500, "home": 0},
                {"speed": -200, "position": -1000, "home": 1},
            ],
        },
        motion_profile={"speed": 200, "acc": 200},
    )

    assert evidence["classification"]["controller_motion_evidence"] is True
    assert evidence["classification"]["switch_transition"] is True
    assert evidence["switch_transition"] is True


def test_motion_error_events_fail_relative_move_even_with_counter_delta(monkeypatch):
    api = load_api(monkeypatch)

    class FakeTester:
        def motor_function_preset(self, key):
            return {"board": 5, "motor": 0, "speed": 200, "acc": 200, "run_current": 31, "standby_current": 10, "stall_guard": 16}

        def motor_normalize_speed_acc(self, board, *, motor=0, speed=None, acc=None):
            return {"axis_key": "x", "speed": speed or 200, "acc": acc or 200, "changes": []}

        def motion_arm_state(self):
            return {"armed": True}

        def motion_gate_live_snapshot(self):
            return {"ok": True}

        def activate_boards(self, expect_reply=True):
            return {5: {"status": 100}}

        def motor_prepare_motion_interlock(self, force_lock=True):
            return {"ok": True}

        def motor_prepare_axis(self, board, **kwargs):
            return {"ok": True, "ops": []}

        def motor_get_position(self, board, *, motor=0):
            return {"position": 0}

        def motor_get_switch_activity(self, board, *, motor=0):
            return {"left_active": False, "right_active": False}

        def motor_move_relative(self, board, steps, *, motor=0):
            return {"ok": True, "steps": steps}

    class FakeReferenceStore:
        def snapshot(self, axes):
            return {"rows": {"x": {"axis": "x", "state": "referenced"}}}

    monkeypatch.setattr(api, "_reference_state_store", FakeReferenceStore())
    monkeypatch.setattr(
        api,
        "_wait_for_motion_with_guardrails",
        lambda tester, board, motor, timeout_s: {
            "ok": True,
            "seen_nonzero": True,
            "position_after": {"position": 1000},
            "switch_activity_after": {"left_active": False, "right_active": False},
            "reached_position_after": {"target_reached": 1},
            "log_tail": [{"speed": 100, "position": 500, "target_reached": 0}, {"speed": 0, "position": 1000, "target_reached": 1}],
        },
    )
    monkeypatch.setattr(api, "_collect_motion_event_capture", lambda tester: ([{"status": 14, "board": 5, "motor": 0}], True))

    result = api._execute_relative_move(FakeTester(), api.AxisName.X, 1000, 5.0)

    assert result["ok"] is False
    assert result["motion_failure"]["category"] == "controller_motion_error"
    assert result["motion_failure"]["event_status"] == 14


def test_move_axis_relative_skips_reference_update_for_structured_motion_failure(monkeypatch):
    api = load_api(monkeypatch)
    recorded = []

    async def fake_runner(command, **kwargs):
        return {
            "axis": getattr(command.axis, "value", command.axis),
            "ok": False,
            "motion_failure": {"category": "guardrail_no_motion", "message": "no position delta detected"},
        }

    class FakeStore:
        def record_motion(self, axis, motion_kind):
            recorded.append((axis, motion_kind))
            return {"ok": True}

    monkeypatch.setattr(api, "run_relative_motion_command", fake_runner)
    monkeypatch.setattr(api, "_reference_state_store", FakeStore())

    req = api.MoveRelativeRequest(axis=api.AxisName.X, steps=5)
    result = asyncio.run(api.move_axis_relative(req))

    assert result["ok"] is False
    assert recorded == []


class _LocalRequest:
    client = types.SimpleNamespace(host="127.0.0.1")


def test_maintenance_usb_release_blocks_live_axis_motion_until_recovery(monkeypatch):
    api = load_api(monkeypatch)
    disconnected = []

    class FakeTester:
        def _disconnect(self):
            disconnected.append("disconnect")
            return {"ok": True, "release_interface_ok": True, "dispose_resources_ok": True}

    async def fake_runner(*args, **kwargs):
        raise AssertionError("live motion runner must not be called while maintenance recovery is required")

    monkeypatch.setattr(api, "_tester", FakeTester())
    monkeypatch.setattr(api, "_tester_quarantine", None)
    monkeypatch.setattr(api, "_pipette_transport", None)
    result = asyncio.run(api.maintenance_usb_release(_LocalRequest()))

    assert disconnected == ["disconnect"]
    assert result["maintenance_state"]["motion_blocked"] is True
    assert result["maintenance_state"]["usb_owner"] == "released"

    monkeypatch.setattr(api, "run_relative_motion_command", fake_runner)
    with pytest.raises(HTTPException) as exc_info:
        asyncio.run(api.move_axis_relative(api.MoveRelativeRequest(axis=api.AxisName.X, steps=5)))

    assert exc_info.value.status_code == 409
    assert exc_info.value.detail["error"] == "post_maintenance_motion_recovery_required"
    assert exc_info.value.detail["hardware_motion_commanded"] is False


def test_maintenance_block_allows_validation_only_dry_run_bundle(monkeypatch):
    api = load_api(monkeypatch)
    api._mark_post_maintenance_motion_block(source="test", reason="blocked for dry-run test")

    async def fake_runner(command, **kwargs):
        return {"axis": getattr(command.axis, "value", command.axis), "dry_run": True, "move": {"ok": False, "skipped": True}}

    monkeypatch.setattr(api, "run_relative_motion_command", fake_runner)

    result = asyncio.run(
        api.move_axis_relative(
            api.MoveRelativeRequest(
                axis=api.AxisName.X,
                steps=5,
                capture_bundle=True,
                dry_run_bundle=True,
            )
        )
    )

    assert result["dry_run"] is True


def _successful_non_homing_recovery_result():
    return {
        "ok": True,
        "run_homing": False,
        "homing": None,
        "arm_state": {"armed": True},
        "final_gate": {"ok": True},
    }


def test_post_maintenance_recover_motion_requires_ack_and_clears_block_without_homing(monkeypatch):
    api = load_api(monkeypatch)
    calls = []

    class FakeTester:
        def motion_arm_strict_startup(self, *, run_homing=False):
            calls.append(("strict_startup", run_homing))
            return _successful_non_homing_recovery_result()

    api._tester = FakeTester()
    api._mark_post_maintenance_motion_block(source="test", reason="blocked before recovery")

    with pytest.raises(HTTPException) as exc_info:
        asyncio.run(
            api.maintenance_usb_recover_motion(
                api.MaintenanceRecoverMotionRequest(operator_ack="NOPE", include_diag=False),
                _LocalRequest(),
            )
        )
    assert exc_info.value.status_code == 409
    assert calls == []
    assert api._maintenance_state_payload()["motion_blocked"] is True

    result = asyncio.run(
        api.maintenance_usb_recover_motion(
            api.MaintenanceRecoverMotionRequest(operator_ack=api.MAINTENANCE_RECOVERY_ACK, include_diag=False),
            _LocalRequest(),
        )
    )

    assert calls == [("strict_startup", False)]
    assert result["maintenance_state"]["motion_blocked"] is False
    assert result["maintenance_state"]["last_recovery"]["evidence"]["strict_startup"]["run_homing"] is False


@pytest.mark.parametrize(
    "recovery",
    [
        None,
        {},
        {"ok": 1, "homing": None, "arm_state": {"armed": True}, "final_gate": {"ok": True}},
        {"ok": True, "homing": None, "arm_state": {"armed": 1}, "final_gate": {"ok": True}},
        {"ok": True, "homing": None, "arm_state": {"armed": True}, "final_gate": {"ok": 1}},
        {"ok": True, "homing": {"z_home": {}}, "arm_state": {"armed": True}, "final_gate": {"ok": True}},
    ],
)
def test_post_maintenance_recovery_failure_preserves_block_and_fails_closed(monkeypatch, recovery):
    api = load_api(monkeypatch)

    class FakeTester:
        def motion_arm_strict_startup(self, *, run_homing=False):
            assert run_homing is False
            return recovery

    api._tester = FakeTester()
    api._mark_post_maintenance_motion_block(source="test", reason="blocked before failed recovery")

    with pytest.raises(HTTPException) as exc_info:
        asyncio.run(
            api.maintenance_usb_recover_motion(
                api.MaintenanceRecoverMotionRequest(
                    operator_ack=api.MAINTENANCE_RECOVERY_ACK,
                    include_diag=False,
                ),
                _LocalRequest(),
            )
        )

    assert exc_info.value.status_code == 409
    assert exc_info.value.detail["error"] == "motion_recovery_failed_closed"
    assert exc_info.value.detail["hardware_motion_commanded"] is False
    assert exc_info.value.detail["maintenance_state"]["motion_blocked"] is True
    assert api._maintenance_state_payload()["motion_blocked"] is True


def test_remote_strict_startup_requires_recover_motion_ack_before_hardware(monkeypatch):
    api = load_api(monkeypatch)
    api._tester = object()
    api._mark_post_maintenance_motion_block(source="test", reason="blocked before remote recovery")
    monkeypatch.setattr(api, "_get_tester", lambda: (_ for _ in ()).throw(AssertionError("hardware accessed")))

    for request in (
        api.MotionArmStartupRequest(
            run_homing=False,
            operator_ack="WRONG",
            operator_reason="supervised post-maintenance recovery",
        ),
        api.MotionArmStartupRequest(
            run_homing=False,
            operator_ack="RECOVER_MOTION",
            operator_reason=None,
        ),
    ):
        with pytest.raises(HTTPException) as exc_info:
            asyncio.run(api.motion_arm_strict_startup(request))

        assert exc_info.value.status_code == 409
        assert exc_info.value.detail["expected_operator_ack"] == "RECOVER_MOTION"
        assert exc_info.value.detail["operator_reason_required"] is True
        assert exc_info.value.detail["hardware_motion_commanded"] is False
    assert api._maintenance_state_payload()["motion_blocked"] is True


def test_remote_strict_startup_rejects_when_recovery_is_not_pending_before_hardware(monkeypatch):
    api = load_api(monkeypatch)
    monkeypatch.setattr(api, "_get_tester", lambda: (_ for _ in ()).throw(AssertionError("hardware accessed")))

    with pytest.raises(HTTPException) as exc_info:
        asyncio.run(
            api.motion_arm_strict_startup(
                api.MotionArmStartupRequest(
                    run_homing=False,
                    operator_ack="RECOVER_MOTION",
                    operator_reason="invalid duplicate recovery attempt",
                )
            )
        )

    assert exc_info.value.status_code == 409
    assert exc_info.value.detail["error"] == "motion_recovery_not_required"
    assert exc_info.value.detail["hardware_motion_commanded"] is False


def test_remote_strict_startup_failed_result_preserves_block(monkeypatch):
    api = load_api(monkeypatch)

    class FakeTester:
        def motion_arm_strict_startup(self, *, run_homing=False):
            assert run_homing is False
            return {
                "ok": False,
                "homing": None,
                "arm_state": {"armed": False},
                "final_gate": {"ok": False},
            }

    api._tester = FakeTester()
    api._mark_post_maintenance_motion_block(source="test", reason="blocked before remote recovery")

    with pytest.raises(HTTPException) as exc_info:
        asyncio.run(
            api.motion_arm_strict_startup(
                api.MotionArmStartupRequest(
                    run_homing=False,
                    operator_ack="RECOVER_MOTION",
                    operator_reason="supervised failed recovery proof test",
                )
            )
        )

    assert exc_info.value.status_code == 409
    assert exc_info.value.detail["error"] == "motion_recovery_failed_closed"
    assert exc_info.value.detail["hardware_motion_commanded"] is False
    assert api._maintenance_state_payload()["motion_blocked"] is True


def test_remote_strict_startup_valid_result_clears_block_and_capability_is_advertised(monkeypatch):
    api = load_api(monkeypatch)

    class FakeTester:
        def motion_arm_strict_startup(self, *, run_homing=False):
            assert run_homing is False
            return _successful_non_homing_recovery_result()

    api._tester = FakeTester()
    api._mark_post_maintenance_motion_block(source="test", reason="blocked before remote recovery")

    result = asyncio.run(
        api.motion_arm_strict_startup(
            api.MotionArmStartupRequest(
                run_homing=False,
                operator_ack="RECOVER_MOTION",
                operator_reason="supervised non-homing recovery after maintenance",
            )
        )
    )

    assert result["maintenance_state"]["motion_blocked"] is False
    assert "recover_motion_non_homing" in api.BMS_COMMISSIONING_CAPABILITIES
    assert "recover_motion" not in api.BMS_COMMISSIONING_CAPABILITIES
    assert result["maintenance_state"]["last_recovery"]["evidence"]["operator_reason"] == (
        "supervised non-homing recovery after maintenance"
    )


def test_remote_strict_startup_cannot_clear_newer_maintenance_block(monkeypatch):
    api = load_api(monkeypatch)
    recovery_started = threading.Event()
    allow_recovery_to_finish = threading.Event()

    class FakeTester:
        def motion_arm_strict_startup(self, *, run_homing=False):
            assert run_homing is False
            recovery_started.set()
            assert allow_recovery_to_finish.wait(timeout=2.0)
            return _successful_non_homing_recovery_result()

    api._tester = FakeTester()
    api._mark_post_maintenance_motion_block(source="older-maintenance", reason="older latch")

    async def scenario():
        task = asyncio.create_task(
            api.motion_arm_strict_startup(
                api.MotionArmStartupRequest(
                    run_homing=False,
                    operator_ack="RECOVER_MOTION",
                    operator_reason="recover only the observed older latch",
                )
            )
        )
        assert await asyncio.to_thread(recovery_started.wait, 1.0)
        api._mark_post_maintenance_motion_block(source="newer-maintenance", reason="newer latch")
        allow_recovery_to_finish.set()
        with pytest.raises(HTTPException) as exc_info:
            await task
        return exc_info.value

    exc = asyncio.run(scenario())
    assert exc.status_code == 409
    assert exc.detail["error"] == "motion_recovery_latch_changed"
    assert exc.detail["hardware_motion_commanded"] is False
    state = api._maintenance_state_payload()
    assert state["motion_blocked"] is True
    assert state["recovery_required"] is True
    assert state["blocked_by"] == "newer-maintenance"


def test_protocol_live_move_is_blocked_after_maintenance_before_executor(monkeypatch):
    api = load_api(monkeypatch)
    api._mark_post_maintenance_motion_block(source="test", reason="blocked before protocol move")

    class Action:
        action_id = "blocked-move"
        params = {"axis": "x", "position_steps": 100}

    monkeypatch.setattr(api, "_execute_absolute_move", lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("executor must not run")))

    with pytest.raises(HTTPException) as exc_info:
        api._protocol_live_move_handler(Action(), None)

    assert exc_info.value.status_code == 409
    assert exc_info.value.detail["hardware_motion_commanded"] is False



def test_controller_only_motion_truth_does_not_update_reference(monkeypatch):
    api = load_api(monkeypatch)

    assert api._motion_response_allows_reference_update({
        "axis": "x",
        "position_delta": 1000,
        "wait": {"ok": True, "seen_nonzero": True},
        "motion_truth": {"physical_motion_confirmed": False, "evidence_level": "controller_only"},
    }) is False

    assert api._motion_response_allows_reference_update({
        "axis": "x",
        "position_delta": 1000,
        "motion_truth": {"physical_motion_confirmed": True, "evidence_level": "operator_or_camera"},
    }) is True


def test_operator_desynced_axis_blocks_motion_prepare(monkeypatch):
    api = load_api(monkeypatch)

    class FakeStore:
        def snapshot(self, axes):
            return {"rows": {"x": {"axis": "x", "state": "desynced", "note": "operator reported no physical movement"}}}

    monkeypatch.setattr(api, "_reference_state_store", FakeStore())

    try:
        api._require_axis_not_operator_desynced(api.AxisName.X, command="axis motion/prepare")
    except api.HTTPException as exc:
        assert exc.status_code == 409
        assert exc.detail["axis"] == "x"
        assert exc.detail["reason"] == "operator_physical_truth_over_controller_counter_truth"
    else:
        raise AssertionError("desynced axis was not blocked")



def test_axis_limit_guard_blocks_current_position_outside_oem_envelope(monkeypatch):
    api = load_api(monkeypatch)
    monkeypatch.setattr(api, "harmonized_motion_config", lambda: {"axis_limits": {"x": {"min_steps": 0, "max_steps": 90263}}})

    try:
        api._guard_axis_position_within_oem_limits(api.AxisName.X, {"position": -307111}, command="home search")
    except api.HTTPException as exc:
        assert exc.status_code == 409
        assert exc.detail["reason"] == "controller_position_outside_oem_axis_limits"
        assert exc.detail["position_steps"] == -307111
    else:
        raise AssertionError("out-of-range X position was not blocked")


def test_axis_limit_guard_blocks_relative_target_outside_oem_envelope(monkeypatch):
    api = load_api(monkeypatch)
    monkeypatch.setattr(api, "harmonized_motion_config", lambda: {"axis_limits": {"x": {"min_steps": 0, "max_steps": 90263}}})

    try:
        api._guard_absolute_target(api.AxisName.X, {"position": 1000}, -1, {"left_active": False, "right_active": False}, {})
    except api.HTTPException as exc:
        assert exc.status_code == 409
        assert exc.detail["reason"] == "target_outside_oem_axis_limits"
        assert exc.detail["target_position_steps"] == -1
    else:
        raise AssertionError("out-of-range X target was not blocked")


def test_z_guard_transforms_oem_source_limits_into_live_signed_controller_envelope(monkeypatch):
    api = load_api(monkeypatch)
    monkeypatch.setattr(
        api,
        "harmonized_motion_config",
        lambda: {"axis_limits": {"z": {"min_steps": 0, "max_steps": 160000, "source": "test_oem_source"}}},
    )

    api._guard_absolute_target(
        api.AxisName.Z,
        {"position": 0},
        -1,
        {"left_active": False, "right_active": False},
        {},
    )

    for unsafe_target in (1, -160001):
        try:
            api._guard_absolute_target(
                api.AxisName.Z,
                {"position": 0},
                unsafe_target,
                {"left_active": False, "right_active": False},
                {},
            )
        except api.HTTPException as exc:
            assert exc.status_code == 409
            assert exc.detail["reason"] == "target_outside_oem_axis_limits"
            assert exc.detail["configured_limit"]["coordinate_contract"] == "observed_live_signed_z"
        else:
            raise AssertionError(f"unsafe signed-Z target {unsafe_target} was not blocked")


def test_target_validation_runs_before_missing_current_position_fails_closed(monkeypatch):
    api = load_api(monkeypatch)
    monkeypatch.setattr(
        api,
        "harmonized_motion_config",
        lambda: {"axis_limits": {"z": {"min_steps": 0, "max_steps": 160000}}},
    )

    try:
        api._guard_absolute_target(
            api.AxisName.Z,
            None,
            1,
            {"left_active": False, "right_active": False},
            {},
        )
    except api.HTTPException as exc:
        assert exc.detail["reason"] == "target_outside_oem_axis_limits"
    else:
        raise AssertionError("out-of-range target bypassed validation when current position was absent")

    try:
        api._guard_absolute_target(
            api.AxisName.Z,
            None,
            -1,
            {"left_active": False, "right_active": False},
            {},
        )
    except api.HTTPException as exc:
        assert exc.detail["reason"] == "current_position_unavailable_for_motion_guard"
        assert exc.detail["motion_blocked"] is True
    else:
        raise AssertionError("missing current position did not fail closed")



def test_target_reached_event_for_wrong_motor_does_not_confirm_motion(monkeypatch):
    api = load_api(monkeypatch)

    evidence = api._build_motion_evidence(
        preset={"board": 5, "motor": 0},
        prep={},
        interlock={},
        position_before={"position": 0},
        switch_before={"left_active": False, "right_active": False},
        position_after={"position": 1000},
        switch_after={"left_active": False, "right_active": False},
        move={"ok": True},
        wait={"ok": True, "seen_nonzero": True, "log_tail": [{"speed": 100, "position": 500}]},
        raw_events=[{"board": 5, "motor": 1, "status": 128, "cmd": 138, "value": 0}],
        event_capture_attempted=True,
    )

    assert evidence["events"]["target_reached_128"] == []
    assert evidence["classification"]["target_reached_event_seen"] is False
    assert evidence["classification"]["controller_motion_evidence"] is False


def test_target_reached_event_for_matching_motor_confirms_motion(monkeypatch):
    api = load_api(monkeypatch)

    evidence = api._build_motion_evidence(
        preset={"board": 5, "motor": 0},
        prep={},
        interlock={},
        position_before={"position": 0},
        switch_before={"left_active": False, "right_active": False},
        position_after={"position": 1000},
        switch_after={"left_active": False, "right_active": False},
        move={"ok": True},
        wait={"ok": True, "seen_nonzero": True, "log_tail": [{"speed": 100, "position": 500}]},
        raw_events=[{"board": 5, "motor": 0, "status": 128, "cmd": 138, "value": 0}],
        event_capture_attempted=True,
    )

    assert len(evidence["events"]["target_reached_128"]) == 1
    assert evidence["classification"]["target_reached_event_seen"] is True
    assert evidence["classification"]["controller_motion_evidence"] is True


def test_gap8_query_reports_oem_return_semantics_not_raw_target_value(monkeypatch):
    from src.bioxp.usb_driver import BioXpTester

    tester = BioXpTester.__new__(BioXpTester)
    tester.motor_get_axis_param = lambda board, param, motor=0: {
        "board": board,
        "param": param,
        "motor": motor,
        "value": 0,
        "ack": {"status": 100, "status_str": "Success"},
    }
    tester._tmcl_success = lambda ack: isinstance(ack, dict) and ack.get("status") == 100

    row = tester.motor_get_reached_position(5, motor=0)

    assert row["gap_param"] == 8
    assert row["oem_query_reached_position_return"] == 0
    assert row["ack_success"] is True
    assert row["target_reached"] is None



def test_relative_move_fails_without_matching_oem_target_event(monkeypatch):
    api = load_api(monkeypatch)
    store = api.ReferenceStateStore()
    store.mark_referenced(api.MarkAxisReferencedCommand(axis=api.AxisName.X, position_steps=0, source="test", motion_kind="test"))
    monkeypatch.setattr(api, "_reference_state_store", store)
    monkeypatch.setattr(api, "harmonized_motion_config", lambda: {"axis_limits": {"x": {"min_steps": 0, "max_steps": 90263}}})

    class FakeTester:
        MOTOR_SWITCH_ACTIVE_VALUE = 1
        def __init__(self):
            self.speeds = [100, 100, 0, 0]
            self.positions = [0, 500, 1000, 1000, 1000]
        def motor_function_preset(self, key):
            return {"board": 5, "motor": 0, "label": "X", "speed": 200, "acc": 200, "run_current": 31, "standby_current": 10, "stall_guard": 16}
        def motor_normalize_speed_acc(self, board, *, motor=0, speed=None, acc=None):
            return {"axis_key": "x", "speed": speed or 200, "acc": acc or 200, "changes": []}
        def motion_arm_state(self): return {"armed": True}
        def motion_gate_live_snapshot(self): return {"ok": True}
        def activate_boards(self, expect_reply=True): return {5: {"status": 100}}
        def motor_prepare_motion_interlock(self, force_lock=True): return {"ok": True}
        def motor_prepare_axis(self, board, **kwargs): return {"ok": True, "ops": []}
        def motor_get_position(self, board, *, motor=0):
            value = self.positions.pop(0) if self.positions else 1000
            return {"position": value}
        def motor_get_switch_activity(self, board, *, motor=0): return {"left_active": False, "right_active": False}
        def clear_bus_event_buffer(self): pass
        def drain(self, **kwargs): return []
        def motor_move_relative(self, board, steps, *, motor=0): return {"ok": True}
        def motor_get_speed(self, board, *, motor=0):
            value = self.speeds.pop(0) if self.speeds else 0
            return {"speed": value}
        def motor_get_reached_position(self, board, *, motor=0): return {"gap_param": 8, "oem_query_reached_position_return": 0, "target_reached": None, "ok": True}
        def motor_stop(self, board, *, motor=0): return {"ok": True}
        def motor_wait_stopped(self, board, *, motor=0, timeout_s=2.0, poll_s=0.06): return {"ok": True}
        def pop_bus_event_buffer(self): return []
        def collect_bus_events(self, **kwargs): return []

    result = api._execute_relative_move(FakeTester(), api.AxisName.X, 1000, 5.0)

    assert result.get("ok") is False
    assert result["motion_failure"]["category"] == "oem_motion_evidence_missing"
    assert result["motion_evidence"]["classification"]["controller_motion_evidence"] is False



def test_home_evidence_uses_oem_queryhome_sethome_not_required_transition(monkeypatch):
    api = load_api(monkeypatch)

    evidence = api._build_home_motion_evidence(
        preset={"board": 5, "motor": 0},
        prep={},
        interlock={},
        motion_profile={},
        home={
            "ok": True,
            "home_active_value": 1,
            "position_before": {"position": -307111},
            "position_after": {"position": 0},
            "switch_activity_before": {"left_active": True, "right_active": True},
            "switch_activity_after": {"left_active": True, "right_active": True},
            "move_left": {"ok": True},
            "home_after": {"value": 1},
            "set_home": {"ok": True},
            "switch_transition": False,
        },
    )

    assert evidence["home_predicate_confirmed"] is True
    assert evidence["switch_transition"] is False
    assert evidence["classification"]["home_predicate_confirmed"] is True
    assert evidence["classification"]["controller_motion_evidence"] is True


def test_home_route_does_not_block_prehome_outside_oem_envelope(monkeypatch):
    api = load_api(monkeypatch)
    monkeypatch.setattr(api, "harmonized_motion_config", lambda: {"axis_limits": {"x": {"min_steps": 0, "max_steps": 90263}}})
    store = api.ReferenceStateStore()
    store.mark_referenced(api.MarkAxisReferencedCommand(axis=api.AxisName.X, position_steps=0, source="test", motion_kind="test"))
    monkeypatch.setattr(api, "_reference_state_store", store)
    monkeypatch.setattr(api, "_home_predicate_snapshot", lambda tester, axis: {"interpreted": {"confidence": "source_anchored"}})

    class FakeTester:
        MOTOR_SWITCH_ACTIVE_VALUE = 1
        def _motion_oem_axis_profile(self, key):
            return {"board": 5, "motor": 0, "label": "X", "speed": 200, "home_speed": 200, "acc": 200, "run_current": 31, "standby_current": 10}
        def motor_function_preset(self, key):
            return self._motion_oem_axis_profile(key)
        def motion_arm_state(self): return {"armed": True}
        def motion_gate_live_snapshot(self): return {"ok": True}
        def activate_boards(self, expect_reply=True): return {5: {"status": 100}}
        def motor_prepare_motion_interlock(self, force_lock=True): return {"ok": True}
        def motor_prepare_axis(self, board, **kwargs): return {"ok": True, "ops": []}
        def motor_get_position(self, board, *, motor=0): return {"position": -307111}
        def motor_oem_switch_search_home_axis(self, axis_key, *, speed, timeout_s):
            return {
                "ok": True,
                "axis": axis_key,
                "board": 5,
                "motor": 0,
                "home_active_value": 1,
                "position_before": {"position": -307111},
                "position_after": {"position": 0},
                "switch_activity_before": {"left_active": True, "right_active": True},
                "switch_activity_after": {"left_active": True, "right_active": True},
                "move_left": {"ok": True},
                "home_after": {"value": 1},
                "set_home": {"ok": True},
                "switch_transition": False,
            }

    result = api._execute_home_axis(FakeTester(), api.AxisName.X, 200, 10.0)

    assert result["ok"] is True
    assert result["motion_profile"]["position_before_home_search"] == {"position": -307111}
    assert result["motion_profile"]["pre_home_position_outside_limit_policy"] == "tracked_not_blocked_home_reestablishes_reference"
    assert result["motion_evidence"]["classification"]["controller_motion_evidence"] is True
    assert api._motion_response_allows_reference_update(result) is True
