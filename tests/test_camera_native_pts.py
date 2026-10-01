"""Native V4L2 clock mapping; all inputs offline, never a device."""
import asyncio
from datetime import datetime, timezone
from fractions import Fraction
import os
import time

import pytest
from tests.test_camera_capture_cadence import rig
from tests.test_camera_post_request_capture import jpeg, production_argv

_REAL_SPAWN = asyncio.create_subprocess_exec


def test_production_bypasses_ffmpeg_delay_locked_loop():
    argv = production_argv()
    assert argv[argv.index("-timestamps") + 1] == "default"
    assert "-copyts" in argv
    assert argv[argv.index("-c:v") + 1] == "copy"
    assert argv[argv.index("-vsync") + 1] == "0"


def test_clock_mapping_retains_packet_age_and_bounds_sampling(monkeypatch):
    from src.bioxp import api
    monkeypatch.setattr(api.time, "time", lambda: 1790634882.0)
    samples = iter([619784.42, 619784.44])
    monkeypatch.setattr(api.time, "clock_gettime", lambda _: next(samples))
    actual = api._camera_packet_utc(619779420000, Fraction(1, 1000000))
    assert actual is not None
    assert actual.timestamp() == pytest.approx(1790634876.98)


def test_mapping_does_not_accumulate_dequeue_cadence_error(monkeypatch):
    from src.bioxp import api
    for index in range(761):
        mono = 619784.42 + index * 3
        wall = 1790634882.0 + index * 3
        monkeypatch.setattr(api.time, "time", lambda: wall)
        samples = iter([mono, mono + (0.01 if index % 2 else 0)])
        monkeypatch.setattr(api.time, "clock_gettime", lambda _: next(samples))
        actual = api._camera_packet_utc(int((mono - 0.05) * 1000000), Fraction(1, 1000000))
        assert actual is not None
        assert wall - 0.061 <= actual.timestamp() <= wall - 0.049


@pytest.mark.parametrize("pts", [1791046249888043, 239976206748619630,
                                -9223372036854775808, 0, 619785420000])
def test_bad_future_unknown_pts_is_not_source_evidence(monkeypatch, pts):
    from src.bioxp import api
    monkeypatch.setattr(api.time, "time", lambda: 1790634882.0)
    monkeypatch.setattr(api.time, "clock_gettime", lambda _: 619784.42)
    assert api._camera_packet_utc(pts, Fraction(1, 1000000)) is None


def test_native_realtime_is_not_shifted(monkeypatch):
    from src.bioxp import api
    monkeypatch.setattr(api.time, "time", lambda: 1790634882.0)
    monkeypatch.setattr(api.time, "clock_gettime", lambda _: 619784.42)
    actual = api._camera_packet_utc(1790634881000000, Fraction(1, 1000000))
    assert actual is not None and actual.timestamp() == 1790634881.0


def test_bad_pts_preview_continues_but_snapshot_waits(rig):
    api, camera, processes, _ = rig
    async def scenario():
        await api._start_owned_camera_session({})
        proc = processes[0]
        proc.stderr.feed_data(b"#tb 0: 1/1000000\n")
        content = jpeg()
        async def publish(seconds):
            sequence = camera._sequence
            pts = int(seconds * 1000000)
            proc.stderr.feed_data(f"0, {pts}, {pts}, 33333, {len(content)}, 0x0\n".encode())
            proc.stdout.feed_data(content)
            for _ in range(1000):
                if camera._sequence > sequence:
                    return camera.latest()
                await asyncio.sleep(.001)
            pytest.fail("packet did not publish")
        try:
            await publish(time.clock_gettime(time.CLOCK_MONOTONIC) - 5)
            pending = asyncio.create_task(api.camera_snapshot())
            await asyncio.sleep(.03)
            for seconds in [1791046249.888043, 239976206748.61963]:
                frame = await publish(seconds)
                assert frame.content == content and frame.source_captured_at is None
                assert not pending.done()
            old = await publish(time.clock_gettime(time.CLOCK_MONOTONIC) - 5)
            assert old.source_captured_at < datetime.now(timezone.utc)
            assert not pending.done()
            await publish(time.clock_gettime(time.CLOCK_MONOTONIC))
            response = await asyncio.wait_for(pending, 1)
            assert response.body == content
            assert "x-camera-source-frame-at" in response.headers
            assert api._camera_session["frames_emitted"] == 5
        finally:
            await api._stop_owned_camera_session(reason="offline native PTS cleanup")
        assert proc.returncode is not None
    asyncio.run(scenario())


@pytest.mark.parametrize("clock", ["monotonic", "realtime"])
@pytest.mark.parametrize("large", [False, True])
def test_real_ffmpeg_copy_native_pts_snapshot_and_preview(rig, monkeypatch, clock, large):
    from PIL import Image
    from io import BytesIO
    from starlette.requests import Request
    api, camera, processes, _ = rig
    content = jpeg()
    if large:
        output = BytesIO()
        Image.effect_noise((640, 480), 100).convert("RGB").save(output, format="JPEG", quality=95)
        content = output.getvalue()
        assert len(content) > 65536
    writers = []
    async def spawn(*argv, **kwargs):
        assert argv[argv.index("-timestamps") + 1] == "default"
        output = list(argv[argv.index("-i") + 2:])
        # Real ffmpeg timestamps paced MJPEG stdin at realtime; translate that
        # packet clock to monotonic only in the offline fixture using setts.
        if clock == "monotonic":
            offset = time.time() - time.clock_gettime(time.CLOCK_MONOTONIC)
            output = ["-bsf:v", f"setts=ts=PTS-{offset}/TB", *output]
        command = [os.environ.get("CAMERA_TEST_FFMPEG", "ffmpeg"), "-hide_banner",
                   "-loglevel", "error", "-copyts", "-probesize", "32", "-analyzeduration", "0",
                   "-use_wallclock_as_timestamps", "1", "-f", "mjpeg", "-framerate", "30",
                   "-i", "pipe:0", *output]
        proc = await _REAL_SPAWN(*command, stdin=asyncio.subprocess.PIPE, **kwargs)
        processes.append(proc)
        async def write():
            try:
                for _ in range(120):
                    proc.stdin.write(content)
                    await proc.stdin.drain()
                    await asyncio.sleep(.05)
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                proc.stdin.close()
        writers.append(asyncio.create_task(write()))
        return proc
    monkeypatch.setattr(api.asyncio, "create_subprocess_exec", spawn)
    async def scenario():
        await api._start_owned_camera_session({})
        owner = api._camera_session
        preview = await api.camera_mjpeg(Request({"type": "http", "query_string": b""}))
        iterator = preview.body_iterator
        try:
            assert content in await asyncio.wait_for(anext(iterator), 5)
            for _ in range(3):
                requested = datetime.now(timezone.utc)
                response = await asyncio.wait_for(api.camera_snapshot(), 5)
                frame = camera.latest()
                assert response.body == content
                assert requested < frame.source_captured_at <= frame.captured_at
                assert api._camera_session is owner
            print({"clock": clock, "large": large, "sequence": frame.sequence,
                   "source": frame.source_captured_at.isoformat(), "publication": frame.captured_at.isoformat()})
        finally:
            await iterator.aclose()
            await api._stop_owned_camera_session(reason="offline real ffmpeg cleanup")
            for task in writers:
                task.cancel()
            await asyncio.gather(*writers, return_exceptions=True)
        assert len(processes) == 1 and processes[0].returncode is not None
    asyncio.run(scenario())
