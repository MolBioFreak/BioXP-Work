from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

from .frames import OemCommandFrame, OemReplyFrame


class MissingDryRunReply(RuntimeError):
    pass


class ReplayMismatch(RuntimeError):
    pass


class ReplyMismatch(RuntimeError):
    pass


class SafetyContractViolation(RuntimeError):
    pass


def assert_transport_safety(*, mode: str, opened_usb: bool, physical_motion: bool) -> None:
    mode_text = str(mode)
    if mode_text == "dry_run" and opened_usb:
        raise SafetyContractViolation("dry_run transport must not open USB")
    if mode_text in {"dry_run", "shadow"} and physical_motion:
        raise SafetyContractViolation(f"{mode_text} transport must not report physical motion")


@dataclass
class DryRunTransport:
    require_configured_replies: bool = False
    configured_replies: dict[OemCommandFrame, OemReplyFrame] = field(default_factory=dict)
    frames: list[OemCommandFrame] = field(default_factory=list)
    opened_usb: bool = False

    def transmit(self, frame: OemCommandFrame) -> OemReplyFrame:
        self.frames.append(frame)
        if frame in self.configured_replies:
            reply = self.configured_replies[frame]
            if not reply.matches_command(frame):
                raise ReplyMismatch(f"Configured reply does not match {frame!r}: {reply!r}")
            return reply
        if self.require_configured_replies:
            raise MissingDryRunReply(f"No dry-run reply configured for {frame!r}")
        return OemReplyFrame(category=frame.category, status=100, synthetic=True)


def _frame_to_json(frame: OemCommandFrame) -> dict:
    return {
        "sidh": frame.sidh,
        "sidl": frame.sidl,
        "category": frame.category.value,
        "message": frame.message,
        "timeout_ms": frame.timeout_ms,
        "command": frame.command,
        "cmd_type": frame.cmd_type,
        "motor": frame.motor,
        "value": frame.value,
        "oem_payload_hex": frame.oem_payload.hex(),
        "raw_hex": frame.raw.hex(),
    }


@dataclass
class RecordingTransport(DryRunTransport):
    artifact_path: Path | str | None = None
    mode: str = "dry_run"

    def close(self) -> None:
        if self.artifact_path is None:
            return
        path = Path(self.artifact_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "format": "bioxp-oem-compat-trace-v1",
            "mode": self.mode,
            "frames": [_frame_to_json(f) for f in self.frames],
        }
        path.write_text(json.dumps(payload, indent=2, sort_keys=True))


@dataclass
class ReplayTransport(DryRunTransport):
    expected: list[dict] = field(default_factory=list)
    position: int = 0

    @classmethod
    def from_file(cls, path: Path | str) -> "ReplayTransport":
        payload = json.loads(Path(path).read_text())
        return cls(expected=list(payload.get("frames", [])))

    def transmit(self, frame: OemCommandFrame) -> OemReplyFrame:
        if self.position >= len(self.expected):
            raise ReplayMismatch(f"No replay frame at position {self.position}")
        expected = self.expected[self.position]
        actual = _frame_to_json(frame)
        for key in ("sidh", "sidl", "oem_payload_hex"):
            if actual[key] != expected[key]:
                raise ReplayMismatch(f"Replay mismatch at {self.position} for {key}: {actual[key]} != {expected[key]}")
        self.position += 1
        self.frames.append(frame)
        return OemReplyFrame(category=frame.category, status=100, synthetic=True)
