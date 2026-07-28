from __future__ import annotations

import base64
import hashlib
import io
import re
import secrets
import subprocess
import threading
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

from PIL import Image

CAMERA_STATUS_SCHEMA = "bioxp.camera_status.v1"
EXPECTED_CAMERA_CARDS = frozenset(
    {
        "IZONE UVC 5M CAMERA",
        "SMI: IZONE UVC 5M CAMERA",
    }
)
EXPECTED_USB_VID = "2084"
EXPECTED_USB_PID = "f37d"
MAX_VIDEO_NODES = 64
DISCOVERY_TIMEOUT_SECONDS = 3.0
CAPTURE_TIMEOUT_SECONDS = 12.0


class CameraError(RuntimeError):
    pass


class CameraUnavailable(CameraError):
    pass


class CameraAmbiguous(CameraUnavailable):
    pass


class CameraFrameUnavailable(CameraUnavailable):
    pass


@dataclass(frozen=True)
class CameraIdentity:
    device: str
    card: str
    usb_vid: str
    usb_pid: str


@dataclass(frozen=True)
class CameraFrame:
    content: bytes
    sequence: int
    captured_at: datetime
    provider_generation: int
    content_sha256: str
    identity: CameraIdentity


@dataclass(frozen=True)
class CameraStatus:
    available: bool
    frame_sequence: int | None
    frame_captured_at: datetime | None
    frame_age_seconds: float | None
    freshness_budget_seconds: float
    provider_generation: int
    dropped_frames: int
    content_sha256: str | None
    detail: str

    def to_payload(self) -> dict[str, Any]:
        return {
            "schema_version": CAMERA_STATUS_SCHEMA,
            "available": self.available,
            "frame_sequence": self.frame_sequence,
            "frame_captured_at": (
                None if self.frame_captured_at is None else self.frame_captured_at.isoformat()
            ),
            "frame_age_seconds": self.frame_age_seconds,
            "freshness_budget_seconds": self.freshness_budget_seconds,
            "provider_generation": self.provider_generation,
            "dropped_frames": self.dropped_frames,
            "content_sha256": self.content_sha256,
            "detail": self.detail,
        }


class CameraProvider:
    """Single owner for serial-206 camera discovery, capture, and frame state.

    Every camera operation is coordinated by this provider's one lock and one
    generation. The lock is private to camera work and is never shared with
    motion or emergency-stop handling.
    """

    def __init__(
        self,
        *,
        sysfs_root: str | Path = "/sys/class/video4linux",
        dev_root: str | Path = "/dev",
        runner: Callable[..., Any] | None = None,
        generation: int | None = None,
        clock: Callable[[], datetime] | None = None,
        freshness_budget_seconds: float = 30.0,
    ) -> None:
        budget = float(freshness_budget_seconds)
        if not 0.0 < budget <= 60.0:
            raise ValueError("camera freshness budget must be greater than zero and at most 60 seconds")
        self._sysfs_root = Path(sysfs_root)
        self._dev_root = Path(dev_root)
        if generation is not None and (type(generation) is not int or generation < 0):
            raise ValueError("camera provider generation must be a non-negative integer")
        self._runner = runner or subprocess.run
        self._generation = generation if generation is not None else max(1, secrets.randbits(63))
        self._clock = clock or (lambda: datetime.now(timezone.utc))
        self._freshness_budget_seconds = budget
        self._lock = threading.RLock()
        self._latest: CameraFrame | None = None
        self._sequence = 0
        self._dropped_frames = 0

    @property
    def generation(self) -> int:
        return self._generation

    def discover(self) -> CameraIdentity:
        with self._lock:
            matches: list[CameraIdentity] = []
            for sysfs_node in self._video_nodes():
                card = self._read_text(sysfs_node / "name")
                if card not in EXPECTED_CAMERA_CARDS:
                    continue
                vid, pid = self._usb_identity(sysfs_node)
                if vid != EXPECTED_USB_VID or pid != EXPECTED_USB_PID:
                    continue
                device = self._dev_root / sysfs_node.name
                if not device.exists():
                    continue
                if not self._v4l2_confirms_capture(device, card):
                    continue
                matches.append(
                    CameraIdentity(
                        device=str(device),
                        card=card,
                        usb_vid=vid,
                        usb_pid=pid,
                    )
                )
            if not matches:
                raise CameraUnavailable(
                    "exact serial-206 camera identity is unavailable or lacks finite V4L2 capture evidence"
                )
            if len(matches) != 1:
                devices = ", ".join(identity.device for identity in matches)
                raise CameraAmbiguous(f"serial-206 camera identity is ambiguous: {devices}")
            return matches[0]

    def capture(self) -> CameraFrame:
        with self._lock:
            identity = self.discover()
            argv = [
                "ffmpeg",
                "-hide_banner",
                "-loglevel",
                "error",
                "-f",
                "video4linux2",
                "-input_format",
                "mjpeg",
                "-video_size",
                "640x480",
                "-i",
                identity.device,
                "-frames:v",
                "1",
                "-f",
                "image2pipe",
                "-vcodec",
                "mjpeg",
                "pipe:1",
            ]
            try:
                completed = self._runner(
                    argv,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=CAPTURE_TIMEOUT_SECONDS,
                    check=False,
                    shell=False,
                )
            except (OSError, subprocess.TimeoutExpired) as exc:
                raise CameraUnavailable(f"bounded camera capture failed: {exc}") from exc
            if int(completed.returncode) != 0:
                error = self._as_text(completed.stderr).strip() or "ffmpeg capture failed"
                raise CameraUnavailable(error)
            content = bytes(completed.stdout or b"")
            self._validate_jpeg(content)
            captured_at = self._aware_now()
            self._sequence += 1
            frame = CameraFrame(
                content=content,
                sequence=self._sequence,
                captured_at=captured_at,
                provider_generation=self._generation,
                content_sha256=hashlib.sha256(content).hexdigest(),
                identity=identity,
            )
            self._latest = frame
            return frame

    def capture_snapshot(self) -> dict[str, Any]:
        """Capture one frame and expose only immutable, provider-owned pixels.

        The provider-selected device is report-only identity. No path, device,
        control, reset, or stream input is accepted from callers.
        """
        frame = self.capture()
        return {
            "ok": True,
            "device": frame.identity.device,
            "path": None,
            "size": len(frame.content),
            "image_bytes": frame.content,
            "image_b64": base64.b64encode(frame.content).decode("ascii"),
            "image_error": None,
            "metadata": {
                "frame_sequence": frame.sequence,
                "frame_captured_at": frame.captured_at.isoformat(),
                "provider_generation": frame.provider_generation,
                "content_sha256": frame.content_sha256,
            },
        }

    def latest(self) -> CameraFrame:
        with self._lock:
            if self._latest is None:
                raise CameraFrameUnavailable("no camera frame is available")
            age = self._frame_age(self._latest)
            if age > self._freshness_budget_seconds:
                raise CameraFrameUnavailable(
                    f"latest camera frame is stale ({age:.3f}s > {self._freshness_budget_seconds:.3f}s)"
                )
            return self._latest

    def status(self) -> CameraStatus:
        with self._lock:
            frame = self._latest
            if frame is None:
                return CameraStatus(
                    available=False,
                    frame_sequence=None,
                    frame_captured_at=None,
                    frame_age_seconds=None,
                    freshness_budget_seconds=self._freshness_budget_seconds,
                    provider_generation=self._generation,
                    dropped_frames=self._dropped_frames,
                    content_sha256=None,
                    detail="latest_frame_unavailable; streaming_not_supported",
                )
            age = round(self._frame_age(frame), 3)
            fresh = age <= self._freshness_budget_seconds
            return CameraStatus(
                available=True,
                frame_sequence=frame.sequence,
                frame_captured_at=frame.captured_at,
                frame_age_seconds=age,
                freshness_budget_seconds=self._freshness_budget_seconds,
                provider_generation=self._generation,
                dropped_frames=self._dropped_frames,
                content_sha256=frame.content_sha256,
                detail=(
                    "latest_frame_available; streaming_not_supported"
                    if fresh
                    else "latest_frame_stale; streaming_not_supported"
                ),
            )

    def _video_nodes(self) -> tuple[Path, ...]:
        try:
            candidates = [
                path
                for path in self._sysfs_root.iterdir()
                if re.fullmatch(r"video[0-9]+", path.name)
            ]
        except OSError:
            return ()
        candidates.sort(key=lambda path: int(path.name[5:]))
        return tuple(candidates[:MAX_VIDEO_NODES])

    def _usb_identity(self, sysfs_node: Path) -> tuple[str | None, str | None]:
        current = sysfs_node.resolve(strict=False)
        for _ in range(12):
            vid = self._read_text(current / "idVendor")
            pid = self._read_text(current / "idProduct")
            if vid is not None or pid is not None:
                return self._normalize_hex(vid), self._normalize_hex(pid)
            if current.parent == current:
                break
            current = current.parent
        return None, None

    def _v4l2_confirms_capture(self, device: Path, expected_card: str) -> bool:
        argv = ["v4l2-ctl", "--device", str(device), "--all"]
        try:
            completed = self._runner(
                argv,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=DISCOVERY_TIMEOUT_SECONDS,
                check=False,
                shell=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return False
        if int(completed.returncode) != 0:
            return False
        output = self._as_text(completed.stdout)
        card_values = [
            line.split(":", 1)[1].strip()
            for line in output.splitlines()
            if ":" in line and line.split(":", 1)[0].strip().lower() == "card type"
        ]
        device_caps: set[str] = set()
        in_device_caps = False
        for line in output.splitlines():
            stripped = line.strip()
            if stripped.lower().startswith("device caps") and ":" in stripped:
                in_device_caps = True
                continue
            if not in_device_caps:
                continue
            if line.startswith(("\t\t", "        ")):
                if stripped:
                    device_caps.add(stripped)
                continue
            if stripped:
                break
        return card_values == [expected_card] and "Video Capture" in device_caps

    def _aware_now(self) -> datetime:
        value = self._clock()
        if value.tzinfo is None or value.utcoffset() is None:
            raise CameraError("camera clock must return an aware UTC datetime")
        return value.astimezone(timezone.utc)

    def _frame_age(self, frame: CameraFrame) -> float:
        return max(0.0, (self._aware_now() - frame.captured_at).total_seconds())

    @staticmethod
    def _validate_jpeg(content: bytes) -> None:
        if not content.startswith(b"\xff\xd8") or not content.endswith(b"\xff\xd9"):
            raise CameraUnavailable("camera capture did not return a complete JPEG")
        try:
            with Image.open(io.BytesIO(content)) as image:
                image.load()
                if image.format != "JPEG" or image.size != (640, 480):
                    raise CameraUnavailable("camera capture is not a 640x480 JPEG")
        except CameraUnavailable:
            raise
        except Exception as exc:
            raise CameraUnavailable("camera capture returned an invalid JPEG") from exc

    @staticmethod
    def _read_text(path: Path) -> str | None:
        try:
            return path.read_text(encoding="utf-8", errors="strict").strip()
        except (OSError, UnicodeError):
            return None

    @staticmethod
    def _normalize_hex(value: str | None) -> str | None:
        if value is None:
            return None
        normalized = value.strip().lower()
        return normalized.removeprefix("0x")

    @staticmethod
    def _as_text(value: Any) -> str:
        if isinstance(value, bytes):
            return value.decode("utf-8", errors="replace")
        return str(value or "")
