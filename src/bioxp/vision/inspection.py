from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping


@dataclass(frozen=True)
class InspectionCommand:
    device: str = "/dev/video0"
    location_id: str | None = None
    requested_checks: tuple[str, ...] = ()
    include_image_data: bool = True

    def __post_init__(self) -> None:
        checks = tuple(
            str(value).strip().lower()
            for value in self.requested_checks
            if str(value).strip()
        )
        object.__setattr__(self, "requested_checks", checks)
        object.__setattr__(self, "device", str(self.device or "/dev/video0"))
        object.__setattr__(self, "location_id", None if self.location_id is None else str(self.location_id))
        object.__setattr__(self, "include_image_data", bool(self.include_image_data))

    @classmethod
    def from_request(cls, req: Any) -> "InspectionCommand":
        return cls(
            device=getattr(req, "device", "/dev/video0"),
            location_id=getattr(req, "location_id", None),
            requested_checks=tuple(getattr(req, "requested_checks", ()) or ()),
            include_image_data=bool(getattr(req, "include_image_data", True)),
        )


@dataclass(frozen=True)
class InspectionResult:
    ok: bool
    device: str | None
    location_id: str | None
    requested_checks: tuple[str, ...] = ()
    observations: tuple[str, ...] = ()
    capability_used: str = "inspection"
    snapshot: Mapping[str, Any] = field(default_factory=dict)

    def to_payload(self) -> dict[str, Any]:
        return {
            "ok": bool(self.ok),
            "device": self.device,
            "location_id": self.location_id,
            "requested_checks": list(self.requested_checks),
            "observations": list(self.observations),
            "capability_used": self.capability_used,
            "snapshot": dict(self.snapshot),
        }

    @classmethod
    def from_snapshot(
        cls,
        command: InspectionCommand,
        snapshot: Mapping[str, Any],
        *,
        capability_used: str,
    ) -> "InspectionResult":
        observations = []
        if bool(snapshot.get("ok")):
            observations.append("snapshot_captured")
        else:
            observations.append("snapshot_capture_failed")
        observations.extend(f"requested_check:{name}" for name in command.requested_checks)
        snapshot_payload = {
            "path": snapshot.get("path"),
            "size": snapshot.get("size"),
            "image_b64": snapshot.get("image_b64") if command.include_image_data else None,
            "image_error": snapshot.get("image_error"),
            "pick": dict(snapshot.get("pick") or {}),
            "metadata": dict(snapshot.get("metadata") or {}),
        }
        return cls(
            ok=bool(snapshot.get("ok")),
            device=snapshot.get("device"),
            location_id=command.location_id,
            requested_checks=command.requested_checks,
            observations=tuple(observations),
            capability_used=str(capability_used),
            snapshot=snapshot_payload,
        )
