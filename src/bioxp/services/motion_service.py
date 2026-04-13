from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Awaitable, Callable

from fastapi import HTTPException

from .artifact_service import create_motion_validation_bundle

MOTION_PREP_REUSE_DEBUG_ENV = "BIOXP_ENABLE_PREP_REUSE_DEBUG"


@dataclass(frozen=True)
class MotionArtifactOptions:
    capture_bundle: bool = False
    dry_run_bundle: bool = False
    operator_note: str | None = None
    snapshot_refs: tuple[str, ...] = ()

    @classmethod
    def from_request(cls, req: Any) -> "MotionArtifactOptions":
        return cls(
            capture_bundle=bool(getattr(req, "capture_bundle", False)),
            dry_run_bundle=bool(getattr(req, "dry_run_bundle", False)),
            operator_note=getattr(req, "operator_note", None),
            snapshot_refs=tuple(getattr(req, "snapshot_refs", ()) or ()),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "capture_bundle": bool(self.capture_bundle),
            "dry_run_bundle": bool(self.dry_run_bundle),
            "operator_note": self.operator_note,
            "snapshot_refs": list(self.snapshot_refs),
        }


@dataclass(frozen=True)
class RelativeMoveCommand:
    axis: Any
    steps: int
    wait_timeout_s: float
    reuse_prepared: bool = False
    artifact: MotionArtifactOptions = field(default_factory=MotionArtifactOptions)

    @classmethod
    def from_request(cls, req: Any) -> "RelativeMoveCommand":
        return cls(
            axis=req.axis,
            steps=int(req.steps),
            wait_timeout_s=float(req.wait_timeout_s),
            reuse_prepared=bool(getattr(req, "reuse_prepared", False)),
            artifact=MotionArtifactOptions.from_request(req),
        )

    def to_request_payload(self) -> dict[str, Any]:
        return {
            "axis": _axis_value(self.axis),
            "steps": int(self.steps),
            "wait_timeout_s": float(self.wait_timeout_s),
            "reuse_prepared": bool(self.reuse_prepared),
            **self.artifact.to_payload(),
        }


@dataclass(frozen=True)
class AbsoluteMoveCommand:
    axis: Any
    position_steps: int
    wait_timeout_s: float
    artifact: MotionArtifactOptions = field(default_factory=MotionArtifactOptions)

    @classmethod
    def from_request(cls, req: Any) -> "AbsoluteMoveCommand":
        return cls(
            axis=req.axis,
            position_steps=int(req.position_steps),
            wait_timeout_s=float(req.wait_timeout_s),
            artifact=MotionArtifactOptions.from_request(req),
        )

    def to_request_payload(self) -> dict[str, Any]:
        return {
            "axis": _axis_value(self.axis),
            "position_steps": int(self.position_steps),
            "wait_timeout_s": float(self.wait_timeout_s),
            **self.artifact.to_payload(),
        }


@dataclass(frozen=True)
class HomeAxisCommand:
    axis: Any
    speed: int | None
    timeout_s: float
    artifact: MotionArtifactOptions = field(default_factory=MotionArtifactOptions)

    @classmethod
    def from_request(cls, req: Any) -> "HomeAxisCommand":
        return cls(
            axis=req.axis,
            speed=None if getattr(req, "speed", None) is None else int(req.speed),
            timeout_s=float(req.timeout_s),
            artifact=MotionArtifactOptions.from_request(req),
        )

    def to_request_payload(self) -> dict[str, Any]:
        return {
            "axis": _axis_value(self.axis),
            "speed": None if self.speed is None else int(self.speed),
            "timeout_s": float(self.timeout_s),
            **self.artifact.to_payload(),
        }


BlockingRunner = Callable[..., Awaitable[dict[str, Any]]]
DryRunResponseFactory = Callable[[str, Any], dict[str, Any]]


def _axis_value(axis: Any) -> str:
    return str(getattr(axis, "value", axis))


def _normalized_snapshot_refs(values: tuple[str, ...] | list[str] | None) -> list[str]:
    return [str(value).strip() for value in (values or ()) if str(value).strip()]


def validate_motion_artifact_options(artifact: MotionArtifactOptions) -> None:
    if bool(artifact.dry_run_bundle) and not bool(artifact.capture_bundle):
        raise HTTPException(status_code=400, detail="dry_run_bundle requires capture_bundle=true")


def dry_run_motion_truth_payload() -> dict[str, Any]:
    return {
        "evidence_level": "artifact_only",
        "controller_reported_position": False,
        "controller_reported_switches": False,
        "physical_motion_confirmed": False,
        "independent_evidence_required": True,
        "summary": (
            "No hardware motion executed. This dry-run bundle only captures operator metadata, "
            "request JSON, and placeholder response state."
        ),
        "recommended_next_evidence": [
            "perform supervised physical move",
            "capture before/after images",
            "record operator confirmation",
        ],
    }


def dry_run_prep_policy(axis: Any, *, reuse_requested: bool = False) -> dict[str, Any]:
    debug_flag_enabled = str(os.environ.get(MOTION_PREP_REUSE_DEBUG_ENV, "")).strip().lower() in {"1", "true", "yes", "on"}
    note = "Dry-run bundle only; no board activation, interlock wake, or axis prep executed."
    if reuse_requested:
        note = f"{note} reuse_prepared was requested and recorded for audit only."
    return {
        "axis": _axis_value(axis),
        "reuse_requested": bool(reuse_requested),
        "armed_and_live": False,
        "debug_flag_required": True,
        "debug_flag_enabled": debug_flag_enabled,
        "reuse_allowed": False,
        "reuse_used": False,
        "interlock_reused": False,
        "board_activation_skipped": True,
        "axis_prep_skipped": True,
        "note": note,
    }


def dry_run_motion_response(command: str, axis: Any, *, reuse_requested: bool = False) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "axis": _axis_value(axis),
        "dry_run": True,
        "skipped_hardware_io": True,
        "message": "Dry-run validation bundle created; hardware command skipped.",
        "prep_policy": dry_run_prep_policy(axis, reuse_requested=reuse_requested),
        "motion_truth": dry_run_motion_truth_payload(),
    }
    if command == "home":
        payload["home"] = {"ok": True, "skipped": True}
    else:
        payload["move"] = {"ok": True, "skipped": True, "mode": command}
        payload["wait"] = {"ok": True, "skipped": True}
    return payload


def maybe_attach_motion_artifact_bundle(
    *,
    command_name: str,
    command: RelativeMoveCommand | AbsoluteMoveCommand | HomeAxisCommand,
    response_payload: dict[str, Any],
    dry_run: bool,
) -> dict[str, Any]:
    if not bool(command.artifact.capture_bundle):
        return response_payload

    request_payload = command.to_request_payload()
    bundle = create_motion_validation_bundle(
        command=command_name,
        axis=_axis_value(command.axis),
        request_payload=request_payload,
        response_payload=dict(response_payload),
        motion_truth=response_payload.get("motion_truth"),
        prep_policy=response_payload.get("prep_policy"),
        operator_note=request_payload.get("operator_note"),
        snapshot_refs=_normalized_snapshot_refs(command.artifact.snapshot_refs),
        dry_run=dry_run,
    )
    enriched = dict(response_payload)
    enriched["artifact_bundle"] = bundle
    return enriched


def attach_motion_artifact_bundle_safe(
    *,
    command_name: str,
    command: RelativeMoveCommand | AbsoluteMoveCommand | HomeAxisCommand,
    response_payload: dict[str, Any],
    dry_run: bool,
) -> dict[str, Any]:
    try:
        return maybe_attach_motion_artifact_bundle(
            command_name=command_name,
            command=command,
            response_payload=response_payload,
            dry_run=dry_run,
        )
    except Exception as exc:
        if dry_run:
            raise
        enriched = dict(response_payload)
        enriched["artifact_bundle_error"] = str(exc)
        return enriched


async def run_relative_motion_command(
    command: RelativeMoveCommand,
    *,
    get_tester: Callable[[], Any],
    run_blocking: BlockingRunner,
    execute_relative_move: Callable[..., dict[str, Any]],
    dry_run_response_factory: DryRunResponseFactory = dry_run_motion_response,
) -> dict[str, Any]:
    validate_motion_artifact_options(command.artifact)
    if command.artifact.capture_bundle and command.artifact.dry_run_bundle:
        return attach_motion_artifact_bundle_safe(
            command_name="relative",
            command=command,
            response_payload=dry_run_response_factory(
                "relative",
                command.axis,
                reuse_requested=bool(command.reuse_prepared),
            ),
            dry_run=True,
        )

    tester = get_tester()
    response = await run_blocking(
        f"Axis {_axis_value(command.axis)} relative move",
        lambda: execute_relative_move(
            tester,
            command.axis,
            int(command.steps),
            float(command.wait_timeout_s),
            reuse_prepared=bool(command.reuse_prepared),
        ),
        timeout_s=max(25.0, float(command.wait_timeout_s) + 10.0),
    )
    return attach_motion_artifact_bundle_safe(
        command_name="relative",
        command=command,
        response_payload=response,
        dry_run=False,
    )


async def run_absolute_motion_command(
    command: AbsoluteMoveCommand,
    *,
    get_tester: Callable[[], Any],
    run_blocking: BlockingRunner,
    execute_absolute_move: Callable[..., dict[str, Any]],
    dry_run_response_factory: DryRunResponseFactory = dry_run_motion_response,
) -> dict[str, Any]:
    validate_motion_artifact_options(command.artifact)
    if command.artifact.capture_bundle and command.artifact.dry_run_bundle:
        return attach_motion_artifact_bundle_safe(
            command_name="absolute",
            command=command,
            response_payload=dry_run_response_factory("absolute", command.axis),
            dry_run=True,
        )

    tester = get_tester()
    response = await run_blocking(
        f"Axis {_axis_value(command.axis)} absolute move",
        lambda: execute_absolute_move(
            tester,
            command.axis,
            int(command.position_steps),
            float(command.wait_timeout_s),
        ),
        timeout_s=max(35.0, float(command.wait_timeout_s) + 10.0),
    )
    return attach_motion_artifact_bundle_safe(
        command_name="absolute",
        command=command,
        response_payload=response,
        dry_run=False,
    )


async def run_home_axis_command(
    command: HomeAxisCommand,
    *,
    get_tester: Callable[[], Any],
    run_blocking: BlockingRunner,
    execute_home_axis: Callable[..., dict[str, Any]],
    dry_run_response_factory: DryRunResponseFactory = dry_run_motion_response,
) -> dict[str, Any]:
    validate_motion_artifact_options(command.artifact)
    if command.artifact.capture_bundle and command.artifact.dry_run_bundle:
        return attach_motion_artifact_bundle_safe(
            command_name="home",
            command=command,
            response_payload=dry_run_response_factory("home", command.axis),
            dry_run=True,
        )

    tester = get_tester()
    response = await run_blocking(
        f"Axis {_axis_value(command.axis)} home",
        lambda: execute_home_axis(
            tester,
            command.axis,
            command.speed,
            float(command.timeout_s),
        ),
        timeout_s=max(35.0, float(command.timeout_s) + 10.0),
    )
    return attach_motion_artifact_bundle_safe(
        command_name="home",
        command=command,
        response_payload=response,
        dry_run=False,
    )
