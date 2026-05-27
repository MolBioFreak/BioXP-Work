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
    assert result["ok"] is False
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
        return {"axis": getattr(command.axis, "value", command.axis), "target_position": 100, "wait": {"ok": True}}

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
        return {"axis": getattr(command.axis, "value", command.axis), "move": {"ok": True, "mode": "absolute"}}

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

    async def fake_runner(*args, **kwargs):
        raise AssertionError("live motion runner must not be called while maintenance recovery is required")

    api._tester = FakeTester()
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


def test_post_maintenance_recover_motion_requires_ack_and_clears_block_without_homing(monkeypatch):
    api = load_api(monkeypatch)
    calls = []

    class FakeTester:
        def motion_arm_strict_startup(self, *, run_homing=False):
            calls.append(("strict_startup", run_homing))
            return {"ok": True, "run_homing": run_homing}

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
