"""Offline UVC packet framing regressions; no device or real subprocess access."""
import hashlib

import pytest

from src.bioxp import camera_provider as camera
from tests.test_camera_oem_inspection import jpeg, rig  # noqa: F401


@pytest.mark.parametrize("between,after", [(b"", b""), (b"\x00", b""),
                                             (b"", b"\x00"),
                                             (b"\x00" * 1908, b"\x00" * 1905)])
@pytest.mark.parametrize("exposure,count", [(1000, 2), (0, 6)])
def test_only_post_jpeg_zero_padding_is_accepted(rig, between, after, exposure, count):
    provider, state = rig
    first, last = jpeg("red"), jpeg("blue")
    assert b"\x00" in first and b"\x00" in last
    state.frames = between.join([first] * (count - 1) + [last]) + after
    old = provider._publish(jpeg("green"), provider.discover())
    result = provider.capture_inspection(camera.OemInspectionCameraSettings(exposure=exposure))
    assert result.frame.content == last
    assert result.frame.content_sha256 == hashlib.sha256(last).hexdigest()
    assert result.frame.sequence == old.sequence + 1
    assert result.source_frames_discarded == count - 1
    assert result.frame.provider_generation == old.provider_generation
    assert result.frame.captured_at >= result.acquisition_started_at
    ffmpeg = [call for call in state.calls if call[0] == "ffmpeg"]
    assert len(ffmpeg) == 1
    argv = ffmpeg[0]
    assert argv[argv.index("-frames:v") + 1] == str(count)
    assert argv[argv.index("-c:v") + 1] == "copy"
    assert argv[argv.index("-fs") + 1] == str(count * camera.MAX_JPEG_BYTES + 1)


_FIRST, _LAST = jpeg("red"), jpeg("blue")
_PAD = b"\x00" * 7


@pytest.mark.parametrize("content", [
    b"\x00" + _FIRST + _PAD + _LAST,
    b"junk" + _FIRST + _PAD + _LAST,
    _FIRST + _PAD + b"junk" + _LAST,
    _FIRST + b"\x01" + _LAST + _PAD,
    _FIRST + _PAD + _LAST + _PAD + b"junk",
    _FIRST + _PAD + _LAST + b"\x01",
    _FIRST + _PAD + _LAST[:-2] + _PAD,
    _FIRST + _PAD + b"\xff\xd8bad\xff\xd9" + _PAD,
    b"\xff\xd8bad\xff\xd9" + _PAD + _LAST + _PAD,
    _FIRST + _PAD + _LAST + _PAD + _FIRST,
    _FIRST + _PAD + _LAST + _PAD + _FIRST[:-2],
    _FIRST + _PAD + _LAST + _PAD + b"\xff",
    _FIRST + _PAD,
    _PAD,
], ids=["leading-zero", "leading-garbage", "between-garbage", "between-nonzero",
        "trailing-garbage", "trailing-nonzero", "truncated-second", "malformed-second",
        "malformed-first", "extra-frame", "truncated-extra-frame", "partial-marker",
        "missing-second", "only-padding"])
def test_padding_does_not_hide_invalid_framing_or_decode(rig, content):
    provider, state = rig
    old = provider._publish(jpeg("green"), provider.discover())
    state.frames = content
    with pytest.raises(camera.CameraUnavailable):
        provider.capture_inspection(camera.OemInspectionCameraSettings())
    assert provider._sequence == old.sequence
    with pytest.raises(camera.CameraFrameUnavailable):
        provider.latest()
