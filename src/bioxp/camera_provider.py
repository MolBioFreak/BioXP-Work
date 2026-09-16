from __future__ import annotations

import base64
import hashlib
import io
import os
import stat
from uuid import UUID
import re
import secrets
import subprocess
import threading
import time
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
MAX_JPEG_BYTES = 2 * 1024 * 1024
MAX_PROVIDER_GENERATION = (1 << 53) - 1


class CameraJpegBuffer:
    """Bounded incremental MJPEG framing; decoding remains provider-owned."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.dropped = 0

    def feed(self, chunk: bytes):
        # Accept arbitrary caller chunk sizes without growing an unbounded buffer.
        for offset in range(0, len(chunk), 16384):
            self.buffer.extend(chunk[offset:offset + 16384])
            while self.buffer:
                start = self.buffer.find(b"\xff\xd8")
                if start < 0:
                    self.buffer[:] = b"\xff" if self.buffer[-1:] == b"\xff" else b""
                    break
                if start:
                    del self.buffer[:start]
                end = self.buffer.find(b"\xff\xd9", 2)
                if end >= 0:
                    frame = bytes(self.buffer[:end + 2])
                    del self.buffer[:end + 2]
                    if len(frame) <= MAX_JPEG_BYTES:
                        yield frame
                    else:
                        self.dropped += 1
                    continue
                if len(self.buffer) > MAX_JPEG_BYTES:
                    self.dropped += 1
                    self.buffer[:] = b"\xff" if self.buffer[-1:] == b"\xff" else b""
                break


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
class OemInspectionCameraSettings:
    """Values from CameraControlParameter, NOT arbitrary V4L2 controls.

    ControlLib.AdjustCamera (1883-1920) skips Exposure=1000, selects auto
    for Exposure=0, and never applies Gain. The serial-206 Settings3200
    bundle uses 1000 for both fields. Manual exposure units are NOT proven
    by the supplied CGrabThread (it passes property 15 to OpenCV without
    identifying the backend); do not invent a log2-to-V4L2 conversion.
    """

    gain: float = 1000
    exposure: float = 1000

    def validate(self) -> None:
        if type(self.gain) not in (int, float) or self.gain != 1000:
            raise CameraUnavailable(
                "OEM inspection Gain must be the 1000 unchanged sentinel; "
                "CGrabThread.setGain writes OpenCV property 10, not V4L2 gain")
        if type(self.exposure) not in (int, float) or self.exposure not in (0, 1000):
            raise CameraUnavailable(
                "OEM manual exposure conversion is unqualified: only 1000 "
                "(unchanged) and 0 (auto exposure) are supported")


@dataclass(frozen=True)
class CameraInspectionControls:
    brightness: int
    gain: int
    auto_exposure: int
    exposure_time_absolute: int  # V4L2 100-microsecond units, not OEM units.


@dataclass(frozen=True)
class CameraInspectionFrame:
    frame: CameraFrame
    settings: OemInspectionCameraSettings
    controls_before: CameraInspectionControls
    controls_configured: CameraInspectionControls
    controls_after: CameraInspectionControls
    acquisition_started_at: datetime
    source_frames_discarded: int
    source_policy: str = "CGrabThread.GetSingleFrame: discard one, read one"
    width: int = 640
    height: int = 480
    flip: str = "none"


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
        if generation is not None and (type(generation) is not int or not 0 <= generation <= MAX_PROVIDER_GENERATION):
            raise ValueError("camera provider generation must be a non-negative JSON-safe integer")
        self._runner = runner or subprocess.run
        self._generation = generation if generation is not None else max(1, secrets.randbits(52))
        self._clock = clock or (lambda: datetime.now(timezone.utc))
        self._freshness_budget_seconds = budget
        self._lock = threading.RLock()
        self._latest: CameraFrame | None = None
        self._sequence = 0
        self._dropped_frames = 0
        self._stream_owner: str | None = None
        self._stream_identity: CameraIdentity | None = None
        self._stream_accepting = False
        self._led_fd: int | None = None
        self._led_leaf: Any = None
        self._led_binding: Any = None

    def _close_illumination(self) -> None:
        # Caller holds provider RLock, including stream generation transitions.
        fd, self._led_fd = self._led_fd, None
        self._led_leaf = self._led_binding = None
        if fd is not None:
            os.close(fd)

    def close(self) -> None:
        """Release the control-only fd; capture owner lifecycle is unchanged."""
        with self._lock:
            self._close_illumination()

    def _illumination_selection(self):
        from .vision.oem_camera_led import SMI_XU_GUID
        identity = self.discover()
        if self._stream_owner is not None and (
                not self._stream_accepting or identity != self._stream_identity):
            raise CameraUnavailable("camera illumination stream identity changed")
        node = self._sysfs_root / Path(identity.device).name
        current = node.resolve(strict=True)
        usb = None
        for parent in (current, *current.parents):
            if (parent / "idVendor").exists():
                usb = parent
                break
        if usb is None or (self._normalize_hex(self._read_text(usb / "idVendor")),
                           self._normalize_hex(self._read_text(usb / "idProduct"))) != (
                               EXPECTED_USB_VID, EXPECTED_USB_PID):
            raise CameraUnavailable("selected camera USB descriptor identity unavailable")
        raw = (usb / "descriptors").read_bytes()
        if (len(raw) < 18 or raw[:2] != b"\x12\x01"
                or raw[8:12] != bytes.fromhex("84207df3")):
            raise CameraUnavailable("selected camera USB device descriptor mismatch")
        descriptors, offset, video_control = [], 0, False
        while offset < len(raw):
            size = raw[offset]
            if size < 2 or offset + size > len(raw):
                raise CameraUnavailable("malformed selected USB descriptors")
            d = raw[offset:offset + size]
            if d[1] == 4:
                video_control = len(d) >= 9 and d[5:7] == b"\x0e\x01"
            if (video_control and len(d) >= 20 and d[1:3] == b"\x24\x06"
                    and d[4:20] == UUID(SMI_XU_GUID).bytes_le):
                descriptors.append(d)
            offset += size
        if len(descriptors) != 1:
            raise CameraUnavailable("selected camera source XU missing or ambiguous")
        device_stat = os.stat(identity.device)
        expected_dev = self._read_text(node / "dev")
        if (not stat.S_ISCHR(device_stat.st_mode) or expected_dev !=
                f"{os.major(device_stat.st_rdev)}:{os.minor(device_stat.st_rdev)}"):
            raise CameraUnavailable("selected camera device node identity mismatch")
        binding = (identity, self._generation, str(current), str(usb),
                   device_stat.st_dev, device_stat.st_ino, device_stat.st_rdev,
                   hashlib.sha256(raw).hexdigest())
        return binding, descriptors[0]

    def _illumination(self, operation: str, *, channel=None, on=None) -> dict[str, Any]:
        from .vision.oem_camera_led import SmiUvcLed
        # This is explicitly an RLock. The leaf re-enters it for the ENTIRE
        # transaction; no separate capture owner or nonreentrant io_lock nesting.
        with self._lock:
            try:
                binding, descriptor = self._illumination_selection()
                if self._led_binding != binding:
                    self._close_illumination()
                if self._led_leaf is None:
                    if operation == "set":
                        raise CameraUnavailable("camera illumination initialization required")
                    fd = os.open(binding[0].device, os.O_RDWR | os.O_NONBLOCK | os.O_CLOEXEC)
                    self._led_fd = fd
                    opened = os.fstat(fd)
                    if (opened.st_dev, opened.st_ino, opened.st_rdev) != binding[4:7]:
                        raise CameraUnavailable("camera control fd selection changed during open")
                    self._led_leaf = SmiUvcLed(fd, self._lock, descriptor)
                    self._led_binding = binding
                leaf = self._led_leaf
                if operation == "probe":
                    length, info = leaf.probe()
                    result = {"control_length": length, "control_info": info}
                elif operation == "initialize":
                    leaf.probe()
                    result = {"dsp_type": leaf.discover_dsp_type()}
                else:
                    leaf.set_led(channel, on)
                    result = {"channel": channel, "on": on}
                return {"ok": True, **result, "provider_generation": self._generation,
                        "delivery_attempted": operation != "probe",
                        "physical_effect_verified": False}
            except Exception:
                self._close_illumination()
                raise

    def probe_illumination(self) -> dict[str, Any]:
        """Read-only GET_LEN/INFO preflight; never register/bank writes."""
        return self._illumination("probe")

    def initialize_illumination(self) -> dict[str, Any]:
        """Physical source discovery: invoke only inside canonical camera child."""
        return self._illumination("initialize")

    def set_illumination(self, *, channel: int, on: bool) -> dict[str, Any]:
        """Canonical camera child leaf; no implicit discovery or retry."""
        if type(channel) is not int or channel not in (1, 2, 3) or type(on) is not bool:
            raise ValueError("source_preparation_led_arguments_invalid")
        return self._illumination("set", channel=channel, on=on)

    def begin_stream(self, owner: str) -> CameraIdentity:
        """Reserve the device under the same lock as still capture/discovery."""
        with self._lock:
            if self._stream_owner is not None:
                raise CameraUnavailable("camera stream already owns the device")
            identity = self.discover()
            self._close_illumination()
            self._stream_owner = owner
            self._stream_identity = identity
            self._stream_accepting = True
            self._generation = self._generation % MAX_PROVIDER_GENERATION + 1
            self._latest = None
            return identity

    def invalidate_stream(self, owner: str) -> None:
        with self._lock:
            if self._stream_owner == owner:
                self._close_illumination()
                self._stream_accepting = False
                self._latest = None

    def end_stream(self, owner: str) -> None:
        """Release only after the corresponding process is reaped."""
        with self._lock:
            if self._stream_owner == owner:
                self._close_illumination()
                self._stream_owner = None
                self._stream_identity = None
                self._stream_accepting = False
                self._latest = None
                self._generation = self._generation % MAX_PROVIDER_GENERATION + 1

    def drop_stream_frame(self, owner: str, *, invalid: bool = False, count: int = 1) -> None:
        with self._lock:
            if self._stream_owner == owner and self._stream_accepting:
                self._dropped_frames += count
                if invalid:
                    self._latest = None

    def publish_stream_frame(self, owner: str, content: bytes) -> CameraFrame:
        with self._lock:
            if self._stream_owner != owner or not self._stream_accepting:
                raise CameraFrameUnavailable("obsolete camera stream owner")
            assert self._stream_identity is not None
            # Only exact immutable bytes from this owner's retained, validated
            # frame qualify. Changed/untrusted payloads still receive full decode.
            latest = self._latest
            if (type(content) is bytes and latest is not None
                    and type(latest.content) is bytes
                    and latest.provider_generation == self._generation
                    and latest.identity == self._stream_identity
                    and content == latest.content):
                return self._publish(latest.content, self._stream_identity)
            try:
                self._validate_jpeg(content)
            except CameraError:
                self.drop_stream_frame(owner, invalid=True)
                raise
            return self._publish(content, self._stream_identity)

    def _publish(self, content: bytes, identity: CameraIdentity) -> CameraFrame:
        captured_at = self._aware_now()
        self._sequence += 1
        frame = CameraFrame(
            content=content, sequence=self._sequence, captured_at=captured_at,
            provider_generation=self._generation,
            content_sha256=hashlib.sha256(content).hexdigest(), identity=identity,
        )
        self._latest = frame
        return frame

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
            if self._stream_owner is not None:
                return self.latest()
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
            return self._publish(content, identity)

    def capture_inspection(self, settings: OemInspectionCameraSettings) -> CameraInspectionFrame:
        """Fresh source-typed acquisition; NEVER returns the preview cache.

        The native caller owns movement/LED sequencing and must call this only
        after its source settle point. A preview owner must be explicitly stopped
        and reaped by its existing service before entry. We neither stop nor
        restart that service, including after failure. All work uses our normal
        lock, discovery, generation, decode and publication authority.
        """
        if type(settings) is not OemInspectionCameraSettings:
            raise CameraUnavailable("inspection requires OemInspectionCameraSettings")
        settings.validate()  # Reject unqualified mappings before any device access.
        with self._lock:
            if self._stream_owner is not None:
                raise CameraUnavailable("inspection requires preview owner to be stopped and reaped")
            # An unsuccessful inspection must not leave an older frame looking
            # like the result of the current post-move request.
            self._latest = None
            identity = self.discover()
            before = self._inspection_controls(identity)
            configured = before
            frames = 2
            if settings.exposure == 0:
                self._inspection_run([
                    "v4l2-ctl", "--device", identity.device, "--set-ctrl", "auto_exposure=3",
                ])
                configured = self._inspection_controls(identity)
                if configured.auto_exposure != 3:
                    raise CameraUnavailable("OEM auto exposure readback disagrees with requested mode")
                self._check_inspection_controls(before, configured, allow_mode_change=True)
                # CGrabThread.setAutoExposure: waitKey(50), then two
                # GetSingleFrame calls. Each request reads twice (90-140).
                time.sleep(0.050)
                frames += 4
            started = self._aware_now()
            completed = self._inspection_run([
                "ffmpeg", "-hide_banner", "-loglevel", "error",
                "-f", "video4linux2", "-input_format", "mjpeg",
                "-video_size", "640x480", "-i", identity.device,
                "-frames:v", str(frames), "-c:v", "copy",
                "-fs", str(frames * MAX_JPEG_BYTES + 1),
                "-f", "image2pipe", "pipe:1",
            ], capture=True)
            content = bytes(completed.stdout or b"")
            if len(content) > frames * MAX_JPEG_BYTES:
                raise CameraFrameUnavailable("inspection capture exceeds bounded frame size")
            buffer = CameraJpegBuffer()
            captured = list(buffer.feed(content))
            if (len(captured) != frames or buffer.dropped or buffer.buffer
                    or b"".join(captured) != content):
                raise CameraFrameUnavailable("inspection capture did not return the required fresh frames")
            for image in captured:
                self._validate_jpeg(image)
            after = self._inspection_controls(identity)
            self._check_inspection_controls(configured, after)
            return CameraInspectionFrame(
                frame=self._publish(captured[-1], identity), settings=settings,
                controls_before=before, controls_configured=configured,
                controls_after=after, acquisition_started_at=started,
                source_frames_discarded=frames - 1,
            )

    def _inspection_run(self, argv: list[str], *, capture: bool = False) -> Any:
        try:
            completed = self._runner(
                argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                timeout=CAPTURE_TIMEOUT_SECONDS if capture else DISCOVERY_TIMEOUT_SECONDS,
                check=False, shell=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise CameraUnavailable(f"bounded inspection camera operation failed: {exc}") from exc
        if int(completed.returncode) != 0:
            raise CameraUnavailable(self._as_text(completed.stderr).strip() or "inspection camera operation failed")
        return completed

    def _inspection_controls(self, identity: CameraIdentity) -> CameraInspectionControls:
        output = self._as_text(self._inspection_run([
            "v4l2-ctl", "--device", identity.device, "--list-ctrls-menus",
        ]).stdout)
        values: dict[str, int] = {}
        # Narrow to the actual IZONE control domains, not generic control input.
        for name, low, high in (("brightness", 0, 15), ("gain", 0, 9),
                                ("exposure_time_absolute", 39, 5000)):
            rows = re.findall(rf"^\s*{name}\s+0x[0-9a-f]+\s+\(int\)\s*:\s*(.*)$", output, re.M)
            if len(rows) != 1:
                raise CameraUnavailable(f"inspection control evidence missing or ambiguous: {name}")
            fields = dict(re.findall(r"(min|max|step|value)=(-?\d+)", rows[0]))
            if (fields.get("min"), fields.get("max"), fields.get("step")) != (str(low), str(high), "1"):
                raise CameraUnavailable(f"inspection control domain changed: {name}")
            if "value" not in fields or not low <= int(fields["value"]) <= high:
                raise CameraUnavailable(f"inspection control readback invalid: {name}")
            values[name] = int(fields["value"])
        modes = re.findall(r"^\s*auto_exposure\s+0x[0-9a-f]+\s+\(menu\)\s*:\s*([^\n]+)\n((?:[ \t]+[13]:[^\n]+\n?)+)", output, re.M)
        if len(modes) != 1:
            raise CameraUnavailable("inspection auto exposure menu is missing or ambiguous")
        mode = re.search(r"\bvalue=(\d+)\b", modes[0][0])
        entries = dict(re.findall(r"([13]):\s*([^\n]+)", modes[0][1]))
        if entries != {"1": "Manual Mode", "3": "Aperture Priority Mode"} or mode is None or mode[1] not in entries:
            raise CameraUnavailable("inspection auto exposure mode is unsupported")
        return CameraInspectionControls(auto_exposure=int(mode[1]), **values)

    @staticmethod
    def _check_inspection_controls(
        before: CameraInspectionControls, after: CameraInspectionControls,
        *, allow_mode_change: bool = False,
    ) -> None:
        if (before.brightness != after.brightness or before.gain != after.gain
                or (not allow_mode_change and before.auto_exposure != after.auto_exposure)
                or (after.auto_exposure == 1
                    and before.exposure_time_absolute != after.exposure_time_absolute)):
            raise CameraUnavailable("inspection controls changed unexpectedly during acquisition")

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
                    detail="latest_frame_unavailable",
                )
            age = round(self._frame_age(frame), 3)
            fresh = age <= self._freshness_budget_seconds
            return CameraStatus(
                available=fresh,
                frame_sequence=frame.sequence,
                frame_captured_at=frame.captured_at,
                frame_age_seconds=age,
                freshness_budget_seconds=self._freshness_budget_seconds,
                provider_generation=self._generation,
                dropped_frames=self._dropped_frames,
                content_sha256=frame.content_sha256,
                detail=(
                    "latest_frame_available"
                    if fresh
                    else "latest_frame_stale"
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
        if len(content) > MAX_JPEG_BYTES:
            raise CameraUnavailable("camera JPEG exceeds bounded frame size")
        if not content.startswith(b"\xff\xd8") or not content.endswith(b"\xff\xd9"):
            raise CameraUnavailable("camera capture did not return a complete JPEG")
        try:
            with Image.open(io.BytesIO(content)) as image:
                if image.format != "JPEG" or image.size != (640, 480):
                    raise CameraUnavailable("camera capture is not a 640x480 JPEG")
                image.load()
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
