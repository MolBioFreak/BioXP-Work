"""Unit-only OEM camera adapter; no real camera or subprocess is used.

Controls below are verbatim lines from serial206 live-readonly-v4l2.txt.
BIOXP_CAMERA_V4L2_EVIDENCE optionally verifies the captured full artifact too.
Images are generated fixtures, not scientific or live acceptance evidence.
"""
from dataclasses import FrozenInstanceError
import io
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace

from PIL import Image
import pytest

from src.bioxp import camera_provider as camera


CONTROLS = """
                     brightness 0x00980900 (int)    : min=0 max=15 step=1 default=8 value=8
                           gain 0x00980913 (int)    : min=0 max=9 step=1 default=3 value=3
                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=3 value=3 (Aperture Priority Mode)
				1: Manual Mode
				3: Aperture Priority Mode
         exposure_time_absolute 0x009a0902 (int)    : min=39 max=5000 step=1 default=1250 value=1250 flags=inactive
"""
DISCOVERY = """Driver Info:
	Card type        : SMI: IZONE UVC 5M CAMERA
	Device Caps      : 0x04200001
		Video Capture
		Streaming
		Extended Pix Format
Media Driver Info:
"""


def jpeg(color="red", size=(640, 480)):
    out = io.BytesIO()
    Image.new("RGB", size, color).save(out, "JPEG")
    return out.getvalue()


@pytest.fixture
def rig(tmp_path, monkeypatch):
    sysfs, dev = tmp_path / "sys", tmp_path / "dev"
    node = sysfs / "video0"
    node.mkdir(parents=True)
    dev.mkdir()
    (node / "name").write_text("SMI: IZONE UVC 5M CAMERA")
    (sysfs / "idVendor").write_text("2084")
    (sysfs / "idProduct").write_text("f37d")
    (dev / "video0").touch()
    state = SimpleNamespace(calls=[], controls=CONTROLS, discovery=DISCOVERY,
                            frames=None, fail=None, sleeps=[], after=None)
    evidence = os.environ.get("BIOXP_CAMERA_V4L2_EVIDENCE")
    if evidence:
        raw = Path(evidence).read_text()
        state.discovery = raw.split("\nUser Controls\n", 1)[0]
        state.controls = raw.rsplit("\nUser Controls\n", 1)[1].split("ioctl:", 1)[0]
        for line in CONTROLS.strip().splitlines():
            assert line.strip() in raw

    def run(argv, **kwargs):
        state.calls.append(argv)
        assert kwargs["shell"] is False and kwargs["timeout"] > 0
        if state.fail:
            raise state.fail
        if argv[0] == "ffmpeg":
            count = int(argv[argv.index("-frames:v") + 1])
            content = state.frames if state.frames is not None else jpeg("red") * (count - 1) + jpeg("blue")
            if state.after is not None:
                state.controls = state.after
            return SimpleNamespace(returncode=0, stdout=content, stderr=b"")
        assert argv[0] == "v4l2-ctl"
        assert argv[2] == str(dev / "video0")
        if argv[-1] == "--all":
            return SimpleNamespace(returncode=0, stdout=state.discovery, stderr="")
        if "--set-ctrl" in argv:
            assert argv[-1] == "auto_exposure=3"
            state.controls = state.controls.replace("value=1 (Manual Mode)", "value=3 (Aperture Priority Mode)")
            return SimpleNamespace(returncode=0, stdout=b"", stderr=b"")
        assert argv[-1] == "--list-ctrls-menus"
        return SimpleNamespace(returncode=0, stdout=state.controls.encode(), stderr=b"")

    monkeypatch.setattr(camera.time, "sleep", state.sleeps.append)
    p = camera.CameraProvider(sysfs_root=sysfs, dev_root=dev, runner=run, generation=17)
    return p, state


def test_actual_domains_and_fresh_final_frame_without_control_writes(rig):
    p, state = rig
    p._publish(jpeg("green"), p.discover())  # Prior post/preview frame must not qualify.
    old = p.latest()
    result = p.capture_inspection(camera.OemInspectionCameraSettings())
    assert result.frame.content == jpeg("blue") != old.content
    assert result.frame.sequence == old.sequence + 1
    assert result.frame.provider_generation == 17
    assert result.frame.captured_at >= result.acquisition_started_at
    assert result.source_frames_discarded == 1
    assert (result.width, result.height, result.flip) == (640, 480, "none")
    assert result.controls_before == result.controls_configured == result.controls_after
    assert result.controls_after == camera.CameraInspectionControls(8, 3, 3, 1250)
    assert not any("--set-ctrl" in c for c in state.calls)
    assert state.sleeps == []
    ff = next(c for c in state.calls if c[0] == "ffmpeg")
    assert ff[ff.index("-c:v") + 1] == "copy"  # No lossy re-encoding/flip/rescale.
    assert ff[ff.index("-frames:v") + 1] == "2"
    assert type(result.frame.content) is bytes
    with pytest.raises(FrozenInstanceError):
        result.controls_after.gain = 4
    with pytest.raises(FrozenInstanceError):
        result.frame.content = b"changed"


def test_auto_mode_source_wait_and_two_settle_requests(rig):
    p, state = rig
    state.controls = state.controls.replace("value=3 (Aperture Priority Mode)", "value=1 (Manual Mode)")
    result = p.capture_inspection(camera.OemInspectionCameraSettings(exposure=0))
    assert result.controls_before.auto_exposure == 1
    assert result.controls_configured.auto_exposure == result.controls_after.auto_exposure == 3
    assert result.source_frames_discarded == 5
    assert state.sleeps == [0.050]
    assert sum("--set-ctrl" in c for c in state.calls) == 1


@pytest.mark.parametrize("settings", [
    {"gain": 0}, {"gain": 9}, {"gain": 1001}, {"gain": True},
    {"exposure": -1}, {"exposure": -2}, {"exposure": -6},
    {"exposure": 1250}, {"exposure": float("nan")},
    {"exposure": float("inf")}, {"exposure": "1000"}, {"exposure": False},
])
def test_unqualified_settings_rejected_before_discovery(rig, settings):
    p, state = rig
    with pytest.raises(camera.CameraUnavailable):
        p.capture_inspection(camera.OemInspectionCameraSettings(**settings))
    assert state.calls == []


def test_rejects_untyped_input(rig):
    p, state = rig
    with pytest.raises(camera.CameraUnavailable, match="requires Oem"):
        p.capture_inspection({"gain": 1000, "exposure": 1000})
    assert not state.calls


def test_preview_must_stop_and_reap_and_is_never_restarted(rig):
    p, state = rig
    p.begin_stream("preview")
    prior = p.publish_stream_frame("preview", jpeg())
    state.calls.clear()
    for invalidated in (False, True):
        if invalidated:
            p.invalidate_stream("preview")
        with pytest.raises(camera.CameraUnavailable, match="stopped and reaped"):
            p.capture_inspection(camera.OemInspectionCameraSettings())
        assert state.calls == []
    p.end_stream("preview")
    generation = p.generation
    result = p.capture_inspection(camera.OemInspectionCameraSettings())
    assert result.frame.provider_generation == generation != prior.provider_generation
    assert p._stream_owner is None
    with pytest.raises(camera.CameraFrameUnavailable):
        p.publish_stream_frame("preview", jpeg())


@pytest.mark.parametrize("bad", [b"", jpeg(), jpeg() * 3, jpeg() + b"\xff\xd8bad\xff\xd9",
                                 jpeg() + jpeg(size=(320, 240)), b"junk" + jpeg() * 2,
                                 jpeg() * 2 + b"junk"])
def test_bad_or_missing_fresh_frames_cannot_publish_cached_image(rig, bad):
    p, state = rig
    p._publish(jpeg(), p.discover())
    state.frames = bad
    with pytest.raises(camera.CameraUnavailable):
        p.capture_inspection(camera.OemInspectionCameraSettings())
    with pytest.raises(camera.CameraFrameUnavailable):
        p.latest()


@pytest.mark.parametrize("old,new", [("max=9", "max=10"), ("min=39", "min=1"),
                                      ("value=1250", "value=5001"),
                                      ("3: Aperture Priority Mode", "3: Unsupported")])
def test_changed_domains_refused_before_controls_or_capture(rig, old, new):
    p, state = rig
    state.controls = state.controls.replace(old, new)
    with pytest.raises(camera.CameraUnavailable):
        p.capture_inspection(camera.OemInspectionCameraSettings(exposure=0))
    assert not any(c[0] == "ffmpeg" or "--set-ctrl" in c for c in state.calls)


def test_readback_drift_refuses_result_but_auto_time_changes_are_observed(rig):
    p, state = rig
    state.after = state.controls.replace("value=1250", "value=1000")
    result = p.capture_inspection(camera.OemInspectionCameraSettings())
    assert result.controls_after.exposure_time_absolute == 1000
    state.after = state.controls.replace("default=3 value=3\n", "default=3 value=4\n")
    with pytest.raises(camera.CameraUnavailable, match="changed unexpectedly"):
        p.capture_inspection(camera.OemInspectionCameraSettings())
    assert not p.status().available


def test_generic_capture_still_uses_preview_cache(rig):
    p, state = rig
    p.begin_stream("preview")
    frame = p.publish_stream_frame("preview", jpeg())
    state.calls.clear()
    assert p.capture() is frame
    assert not state.calls


def test_inspection_and_stream_reservation_share_the_provider_lock(rig):
    from concurrent.futures import ThreadPoolExecutor
    import threading
    p, state = rig
    entered, release, attempted = threading.Event(), threading.Event(), threading.Event()
    original = p._runner
    def blocked(argv, **kwargs):
        if argv[0] == "ffmpeg":
            entered.set()
            assert release.wait(2)
        return original(argv, **kwargs)
    p._runner = blocked
    def reserve():
        attempted.set()
        return p.begin_stream("next-preview")
    with ThreadPoolExecutor(max_workers=2) as workers:
        capture = workers.submit(p.capture_inspection, camera.OemInspectionCameraSettings())
        assert entered.wait(2)
        reservation = workers.submit(reserve)
        assert attempted.wait(2)
        assert not reservation.done()
        release.set()
        result = capture.result(timeout=2)
        reservation.result(timeout=2)
    assert result.frame.provider_generation != p.generation
    assert p._stream_owner == "next-preview"
    assert not p.status().available


def test_manual_retained_exposure_must_not_drift(rig):
    p, state = rig
    state.controls = state.controls.replace("value=3 (Aperture Priority Mode)", "value=1 (Manual Mode)")
    state.after = state.controls.replace("value=1250", "value=1000")
    with pytest.raises(camera.CameraUnavailable, match="changed unexpectedly"):
        p.capture_inspection(camera.OemInspectionCameraSettings())
    assert not p.status().available


def test_failed_device_discovery_clears_old_nonstream_frame(rig):
    p, state = rig
    p._publish(jpeg(), p.discover())
    state.discovery = state.discovery.replace("Video Capture", "Metadata Capture")
    with pytest.raises(camera.CameraUnavailable):
        p.capture_inspection(camera.OemInspectionCameraSettings())
    assert not p.status().available


def test_auto_readback_mismatch_prevents_frame_capture(rig):
    p, state = rig
    state.controls = state.controls.replace("value=3 (Aperture Priority Mode)", "value=1 (Manual Mode)")
    original = p._runner
    def reject_write(argv, **kwargs):
        if "--set-ctrl" in argv:
            return SimpleNamespace(returncode=0, stdout=b"", stderr=b"")
        return original(argv, **kwargs)
    p._runner = reject_write
    with pytest.raises(camera.CameraUnavailable, match="readback disagrees"):
        p.capture_inspection(camera.OemInspectionCameraSettings(exposure=0))
    assert not any(c[0] == "ffmpeg" for c in state.calls)


def test_timeout_fails_closed(rig):
    p, state = rig
    original = p._runner
    def timed_out(argv, **kwargs):
        if argv[0] == "ffmpeg":
            raise subprocess.TimeoutExpired(argv, kwargs["timeout"])
        return original(argv, **kwargs)
    p._runner = timed_out
    with pytest.raises(camera.CameraUnavailable, match="bounded inspection"):
        p.capture_inspection(camera.OemInspectionCameraSettings())
    assert not p.status().available
