"""Exact-byte reuse only; full decode, owner fencing and publication remain real."""
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timedelta, timezone
import io
from unittest.mock import Mock

from PIL import Image
import pytest

from src.bioxp.camera_provider import (
    CameraFrameUnavailable, CameraIdentity, CameraProvider, CameraUnavailable,
)


def jpeg(color="red", size=(640, 480)):
    output = io.BytesIO()
    Image.new("RGB", size, color).save(output, "JPEG")
    return output.getvalue()


def corrupt_scan():
    content = bytearray(jpeg())
    # Header remains readable, but decoding needs an absent quantization table.
    content[content.index(b"\xff\xc0") + 12] = 3
    return bytes(content)


@pytest.fixture
def provider(monkeypatch):
    p = CameraProvider(generation=17)
    monkeypatch.setattr(p, "discover", lambda: CameraIdentity(
        "/synthetic/video7", "IZONE UVC 5M CAMERA", "2084", "f37d"))
    monkeypatch.setattr(p, "_validate_jpeg", Mock(wraps=p._validate_jpeg))
    p.begin_stream("owner")
    return p


def test_equal_bytes_retain_pixels_but_publish_new_sequence_and_time(provider):
    now = datetime(2026, 1, 1, tzinfo=timezone.utc)
    provider._clock = lambda: now
    first = provider.publish_stream_frame("owner", jpeg())
    now += timedelta(seconds=31)
    with pytest.raises(CameraFrameUnavailable, match="stale"):
        provider.latest()
    equal = bytes(bytearray(first.content))
    assert equal is not first.content
    second = provider.publish_stream_frame("owner", equal)
    assert second.content is first.content
    assert second.sequence == first.sequence + 1
    assert second.captured_at == now
    assert second.content_sha256 == first.content_sha256
    assert second.provider_generation == first.provider_generation
    assert provider.status().available
    assert provider._validate_jpeg.call_count == 1
    provider.publish_stream_frame("owner", jpeg("blue"))
    provider.publish_stream_frame("owner", equal)  # Only latest, not a history cache.
    assert provider._validate_jpeg.call_count == 3


@pytest.mark.parametrize("bad", [
    b"", b"\xff\xd8bad\xff\xd9", jpeg()[:-2], corrupt_scan(),
    jpeg()[:200] + b"\xff\xd9", jpeg(size=(320, 240)),
    b"\xff\xd8" + b"x" * (2 * 1024 * 1024) + b"\xff\xd9",
], ids=["empty", "invalid", "missing-eoi", "decode-corrupt", "truncated-header", "wrong-size", "oversized"])
def test_changed_invalid_payload_clears_reuse_and_still_fails(provider, bad):
    good = jpeg()
    provider.publish_stream_frame("owner", good)
    for _ in range(2):
        with pytest.raises(CameraUnavailable):
            provider.publish_stream_frame("owner", bad)
        assert not provider.status().available
    assert provider.status().dropped_frames == 2
    provider.publish_stream_frame("owner", good)
    assert provider._validate_jpeg.call_count == 4


def test_replacement_and_invalidations_require_validation(provider):
    content = jpeg()
    first = provider.publish_stream_frame("owner", content)
    with pytest.raises(CameraFrameUnavailable, match="obsolete"):
        provider.publish_stream_frame("other", content)
    assert provider._validate_jpeg.call_count == 1
    provider.invalidate_stream("owner")
    with pytest.raises(CameraFrameUnavailable):
        provider.publish_stream_frame("owner", content)
    provider.end_stream("owner")
    provider.begin_stream("replacement")
    provider.end_stream("owner")  # Obsolete cleanup cannot remove new owner.
    second = provider.publish_stream_frame("replacement", content)
    assert second.provider_generation != first.provider_generation
    provider.drop_stream_frame("replacement", invalid=True)
    provider.publish_stream_frame("replacement", content)
    assert provider._validate_jpeg.call_count == 3


def test_mutable_input_cannot_supply_reuse_authority(provider):
    content = bytearray(jpeg())
    provider.publish_stream_frame("owner", content)
    content[200:] = b"\xff\xd9"
    with pytest.raises(CameraUnavailable):
        provider.publish_stream_frame("owner", bytes(content))
    assert provider._validate_jpeg.call_count == 2
    provider.publish_stream_frame("owner", jpeg())
    provider.publish_stream_frame("owner", bytearray(jpeg()))
    assert provider._validate_jpeg.call_count == 4


def test_parallel_publications_share_only_locked_latest(provider):
    content = jpeg()
    with ThreadPoolExecutor(max_workers=8) as workers:
        frames = list(workers.map(
            lambda _: provider.publish_stream_frame("owner", bytes(bytearray(content))), range(32)))
    assert sorted(frame.sequence for frame in frames) == list(range(1, 33))
    assert len({id(frame.content) for frame in frames}) == 1
    assert provider._validate_jpeg.call_count == 1
