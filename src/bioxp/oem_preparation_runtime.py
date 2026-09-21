"""Trusted composition for the existing admitted OEM preparation lifecycle.

This module owns neither a workflow runner nor camera/device discovery.
Physical camera leaves run through canonical lifecycle children; preflight
only probes the shared owner's read-only UVC capabilities.
"""
from __future__ import annotations

import copy
import hashlib
import json
from dataclasses import asdict
from pathlib import Path
from typing import Any, Mapping

from .camera_provider import OemInspectionCameraSettings
from .oem_job_preparation import (
    build_prepare_handler, build_preparation_inspection_reader,
    capture_preparation_input, capture_preparation_settings,
    load_declared_diagnostic_job, selected_diagnostic_job_trays,
)
from .protocols.runtime_state import ProtocolSourceModel


LED_DEPENDENCY = (
    "source_preparation_led_backend_unavailable:shared_selected_camera_type0_uvc"
)


def snapshot_evidence_missing(value: Any) -> bool:
    """A caught OEM SnapshotImage exception is not a captured host artifact."""
    if isinstance(value, Mapping):
        if str(value.get("source_anchor", "")).startswith("ControlLib.SnapshotImage:"):
            artifact = value.get("result")
            return not (isinstance(artifact, Mapping) and artifact.get("ok") is True
                        and artifact.get("sha256") and artifact.get("path")
                        and type(artifact.get("size_bytes")) is int and artifact["size_bytes"] > 0)
        return any(snapshot_evidence_missing(child) for child in value.values())
    if isinstance(value, (list, tuple)):
        return any(snapshot_evidence_missing(child) for child in value)
    return False


def capture_selected_preparation(*, snapshot: Any, metadata: Mapping[str, Any]) -> dict[str, Any]:
    """Export exact selected job inputs, without constructing historical state."""
    requirements = metadata.get("source_preparation_requirements")
    if not isinstance(requirements, Mapping):
        raise ValueError("source_preparation_requirements_required")
    if not isinstance(metadata.get("source_model"), Mapping):
        raise ValueError("source_preparation_model_required")
    policy = metadata.get("source_settings")
    if not isinstance(policy, Mapping):
        raise ValueError("source_preparation_settings_required")
    captured_settings = capture_preparation_settings(snapshot, operation_parameters=policy)
    captured = capture_preparation_input(
        settings=captured_settings["settings"],
        source_model=ProtocolSourceModel.from_payload(copy.deepcopy(dict(metadata["source_model"]))),
        requirements=requirements, source_identity=captured_settings["source_identity"],
    )
    if metadata.get("oem_job_load") is True:
        selected_job = metadata.get("source_job_metadata")
        if not isinstance(selected_job, Mapping):
            raise ValueError("source_job_metadata_required_for_new_job_load")
        selected_diagnostic_job_trays(selected_job)
        captured.pop("capture_sha256")
        captured["source_job_metadata"] = copy.deepcopy(dict(selected_job))
        captured["capture_sha256"] = hashlib.sha256(json.dumps(captured, sort_keys=True,
            separators=(",", ":"), allow_nan=False).encode()).hexdigest()
    return captured


class PreparationCameraRuntime:
    """Uses only the existing shared CameraProvider; never stops preview."""

    def __init__(self, camera: Any, *, artifact_root: Path):
        self.camera = camera
        self.artifact_root = Path(artifact_root)

    def require_illumination(self) -> None:
        probe = getattr(self.camera, "probe_illumination", None)
        if not callable(probe):
            raise RuntimeError(LED_DEPENDENCY)
        try:
            result = probe()  # GET_LEN/INFO only, never DSP discovery or bank writes.
        except Exception as exc:
            raise RuntimeError(f"{LED_DEPENDENCY}:{exc}") from exc
        if not isinstance(result, Mapping) or result.get("ok") is not True:
            raise RuntimeError(LED_DEPENDENCY)

    def initialize_illumination(self) -> Mapping[str, Any]:
        return self.camera.initialize_illumination()

    def led(self, *, channel: int, on: bool) -> Mapping[str, Any]:
        if type(channel) is not int or channel not in (1, 2, 3) or type(on) is not bool:
            raise ValueError("source_preparation_led_arguments_invalid")
        return self.camera.set_illumination(channel=channel, on=on)

    def exposure(self, *, value: float | None) -> Mapping[str, Any]:
        settings = OemInspectionCameraSettings(gain=1000, exposure=0 if value is None else value)
        settings.validate()
        captured = self.camera.capture_inspection(settings)
        return {"ok": True, "frame_sha256": captured.frame.content_sha256,
                "source_frames_discarded": captured.source_frames_discarded,
                "delivery_attempted": True, "physical_effect_verified": False}

    def capture_image(self, method: str, state: Any) -> bytes:
        result = self.camera.capture_inspection(OemInspectionCameraSettings(gain=1000, exposure=1000))
        frame = result.frame
        artifact = self._save_capture(result, condition=method,
            artifact_id=f"inspection:{frame.provider_generation}:{frame.sequence}:{frame.captured_at.isoformat()}",
            root=self.artifact_root / state.job_id / "source-images")
        state.record_event("source_inspection_image", detail=artifact)
        return result.frame.content

    def snapshot_image(self, *, condition: str, artifact_id: str) -> dict[str, Any]:
        result = self.camera.capture_inspection(OemInspectionCameraSettings(gain=1000, exposure=1000))
        return self._save_capture(result, condition=condition, artifact_id=artifact_id, root=self.artifact_root)

    @staticmethod
    def _save_capture(result: Any, *, condition: str, artifact_id: str, root: Path) -> dict[str, Any]:
        frame = result.frame
        if not frame.content:
            raise RuntimeError("source_snapshot_frame_empty")
        digest = hashlib.sha256(frame.content).hexdigest()
        # Conditions are descriptive source text, never filesystem paths.
        name = hashlib.sha256(artifact_id.encode()).hexdigest() + ".jpg"
        path = root / name
        artifact = {"ok": True, "capture_ok": True, "condition": condition, "artifact_id": artifact_id,
                "path": str(path), "sha256": digest, "size_bytes": len(frame.content),
                "provider_generation": frame.provider_generation,
                "sequence": frame.sequence, "captured_at": frame.captured_at.isoformat(),
                "camera_identity": asdict(frame.identity),
                "source_frames_discarded": result.source_frames_discarded}
        try:
            root.mkdir(parents=True, exist_ok=True)
            with path.open("xb") as output:
                output.write(frame.content)
            with path.with_suffix(".json").open("x") as output:
                json.dump(artifact, output, sort_keys=True, allow_nan=False)
            artifact["artifact_saved"] = True
        except (OSError, TypeError, ValueError) as exc:
            # Image logging is nonblocking. CV still consumes the real acquired
            # bytes, and the existing event/result explicitly records the loss.
            artifact.update(ok=False, artifact_saved=False,
                            error="source_image_logging_failed:" + str(exc))
        return artifact


def bind_selected_preparation(*, metadata: Mapping[str, Any], snapshot: Any,
                              camera_runtime: PreparationCameraRuntime,
                              execute_native: Any, execute_control: Any, sleep: Any):
    """Build a callback only; all physical leaves stay after workflow admission."""
    captured = capture_selected_preparation(snapshot=snapshot, metadata=metadata)
    settings = captured["source_settings"]
    selected_load = metadata.get("oem_job_load", False)
    if type(selected_load) is not bool:
        raise ValueError("source_job_load_selection_invalid")
    if settings["DeckInspection"]:
        for profile in settings["InspectionSettings"].values():
            OemInspectionCameraSettings(gain=profile["Gain"], exposure=profile["Exposure"]).validate()
        camera_runtime.require_illumination()
    model = ProtocolSourceModel.from_payload(captured["source_model"])
    if selected_load:
        selected_trays = selected_diagnostic_job_trays(captured["source_job_metadata"])
        # Dependency selection uses declared future strips, not fictitious wells.
        model.strips = selected_trays["strips"]
    reader = build_preparation_inspection_reader(snapshot=snapshot, settings=settings,
        source_model=model, capture_image=camera_runtime.capture_image)
    native = build_prepare_handler(captured=captured, execute_native=execute_native,
        execute_control=execute_control, inspect_image=reader, sleep=sleep)

    def prepare(state: Any) -> dict[str, Any]:
        if settings["DeckInspection"]:
            initialized = execute_control("preparation_camera_initialize", {}, state)
            if not isinstance(initialized, Mapping) or initialized.get("ok") is not True:
                return {"ok": False, "failure": "source_preparation_camera_initialization_failed",
                        "camera_initialization": initialized}
        loads = load_declared_diagnostic_job(state, selected=captured["source_job_metadata"],
            execute_native=execute_native) if selected_load else []
        result = native(state)
        result["source_preparation_input"] = copy.deepcopy(captured)
        result["source_job_load_children"] = loads
        failed_children = [row for row in result.get("preparation_evidence", ())
                           if isinstance(row, Mapping) and (
                               ("operation" in row and "result" in row
                                and (not isinstance(row["result"], Mapping)
                                     or row["result"].get("ok") is not True))
                               or ("operation" in row and "exception" in row))]
        if failed_children:
            # A source catch cannot erase a failed/uncertain canonical child.
            # Inspection warnings remain the separate OEM Ignore/Abort choice.
            result.update(ok=False, failure="source_preparation_child_failed",
                          failed_children=copy.deepcopy(failed_children))
        if settings["DeckInspection"]:
            # Native inspection outcomes drive the existing explicit Ignore/Abort
            # gate. They are not human physical attestation or observed tip stock.
            result["deck_manifest"] = {
                "physical_observations": False,
                "inspections": copy.deepcopy(result.get("inspections", [])),
                "inspection_issues": copy.deepcopy(result.get("inspection_issues", [])),
                "artifacts": [copy.deepcopy(event.detail) for event in state.events
                              if event.event == "source_inspection_image"],
            }
        return result
    return prepare
