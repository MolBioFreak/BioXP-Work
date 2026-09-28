"""Offline camera regression: real ffmpeg, no devices or network."""
import ast
import asyncio
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timedelta, timezone
import io
import os
from pathlib import Path
import subprocess
import threading

from PIL import Image
import pytest

from src.bioxp.camera_provider import CameraIdentity, CameraProvider, CameraJpegBuffer, CameraFrameUnavailable
from tests.test_camera_capture_cadence import rig

_REAL_SPAWN = asyncio.create_subprocess_exec


def jpeg():
    output = io.BytesIO()
    Image.new("RGB", (640, 480), "white").save(output, format="JPEG")
    return output.getvalue()


def provider():
    camera = CameraProvider(generation=17)
    camera.discover = lambda: CameraIdentity("/synthetic/video7", "IZONE UVC 5M CAMERA", "2084", "f37d")
    camera.begin_stream("owner")
    return camera


def production_argv():
    source = Path(__file__).parents[1] / "src/bioxp/api.py"
    tree = ast.parse(source.read_text())
    function = next(n for n in tree.body if isinstance(n, ast.AsyncFunctionDef) and n.name == "_start_owned_camera_session_locked")
    assignment = next(n for n in function.body if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "cmd" for t in n.targets))
    return eval(compile(ast.Expression(assignment.value), str(source), "eval"), {
        "fps": 30, "quality": 7, "width": 640, "height": 480, "device": "/synthetic/video7"})


def test_timestamp_gaps_do_not_synthesize_output_frames(tmp_path):
    ffmpeg = os.environ.get("CAMERA_TEST_FFMPEG", "ffmpeg")
    argv = production_argv()
    output = argv[argv.index("-i") + 2:]
    # Lavfi supplies raw frames rather than camera MJPEG: encode only for this
    # timestamp-gap probe, keeping production output synchronization/muxers.
    if "copy" in output:
        output[output.index("copy")] = "mjpeg"
    result = subprocess.run([ffmpeg, "-hide_banner", "-loglevel", "error", "-copyts",
                             "-f", "lavfi", "-i", "testsrc=size=640x480:rate=30:duration=0.1",
                             "-vf", "setpts=30*PTS", "-threads", "1", *output],
                            capture_output=True, check=True, timeout=15)
    frames = list(CameraJpegBuffer().feed(result.stdout))
    print({"source_frames": 3, "output_frames": len(frames), "output_args": output,
           "packet_metadata": result.stderr.decode()})
    assert len(frames) == 3


def test_snapshot_waits_for_post_request_source_not_cached_preview():
    camera = provider()
    old = camera.publish_stream_frame("owner", jpeg())
    started = threading.Event()
    def capture():
        started.set()
        return camera.capture()
    pool = ThreadPoolExecutor(1)
    try:
        future = pool.submit(capture)
        assert started.wait(1)
        # Baseline returns the preview immediately (the regression).
        with pytest.raises(TimeoutError):
            future.result(timeout=0.05)
        # A later host publication of an old source frame must not satisfy it.
        camera.publish_stream_frame("owner", old.content,
                                    source_captured_at=datetime.now(timezone.utc) - timedelta(seconds=5))
        with pytest.raises(TimeoutError):
            future.result(timeout=0.05)
        fresh = camera.publish_stream_frame("owner", old.content,
                                           source_captured_at=datetime.now(timezone.utc))
        assert future.result(timeout=1) is fresh
        assert fresh.content == old.content  # No uniqueness gate for stationary scenes.
        assert fresh.provider_generation == old.provider_generation
        assert camera._stream_owner == "owner"
    finally:
        camera.end_stream("owner")
        pool.shutdown(wait=True)


@pytest.mark.parametrize("large_packet", [False, True])
def test_connected_real_ffmpeg_owner_snapshot_and_preview(rig, monkeypatch, tmp_path, large_packet):
    from starlette.requests import Request
    api, camera, processes, _ = rig
    source = tmp_path / "stationary.jpg"
    if large_packet:
        Image.effect_noise((640, 480), 100).convert("RGB").save(source, format="JPEG", quality=95)
        assert source.stat().st_size > 65536
    else:
        source.write_bytes(jpeg())
    commands = []
    source_tasks = []

    async def spawn(*argv, **kwargs):
        assert argv[argv.index("-i") + 1] == "/synthetic/video7"
        assert "-copyts" in argv and argv[argv.index("-timestamps") + 1] == "abs"
        output = list(argv[argv.index("-i") + 2:])
        assert output[output.index("-c:v") + 1] == "copy"
        # Replace ONLY the camera input. Actual subprocess/pipes, sideband,
        # reader, provider, multipart preview, snapshot route and cleanup run.
        command = [os.environ.get("CAMERA_TEST_FFMPEG", "ffmpeg"), "-hide_banner",
                   "-loglevel", "error", "-copyts", "-probesize", "32", "-analyzeduration", "0",
                   "-use_wallclock_as_timestamps", "1", "-f", "mjpeg", "-framerate", "30",
                   "-i", "pipe:0", *output]
        commands.append(command)
        process = await _REAL_SPAWN(*command, stdin=asyncio.subprocess.PIPE, **kwargs)
        processes.append(process)
        stdin = process.stdin
        assert stdin is not None
        async def source_frames():
            try:
                for _ in range(100):
                    stdin.write(source.read_bytes())
                    await stdin.drain()
                    await asyncio.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                stdin.close()
        source_tasks.append(asyncio.create_task(source_frames()))
        return process
    monkeypatch.setattr(api.asyncio, "create_subprocess_exec", spawn)

    async def scenario():
        await api._start_owned_camera_session({})
        owner = api._camera_session
        generation = camera.generation
        preview = await api.camera_mjpeg(Request({"type": "http", "query_string": b""}))
        iterator = preview.body_iterator
        try:
            part = await asyncio.wait_for(anext(iterator), 5)
            assert source.read_bytes() in part
            for _ in range(3):
                requested_at = datetime.now(timezone.utc)
                before = camera.latest()
                response = await asyncio.wait_for(api.camera_snapshot(), 5)
                after = camera.latest()
                assert response.body == before.content == source.read_bytes()
                assert after.source_captured_at > requested_at
                assert after.sequence > before.sequence
                assert api._camera_session is owner and camera.generation == generation
                assert response.headers["cache-control"] == "no-store"
            print({"real_processes": len(processes), "snapshots": 3,
                   "sequence": camera.latest().sequence,
                   "source_timestamp": camera.latest().source_captured_at.isoformat(),
                   "host_publication": camera.latest().captured_at.isoformat(),
                   "argv": commands[0]})
        finally:
            await iterator.aclose()
            await api._stop_owned_camera_session(reason="offline test cleanup")
            for task in source_tasks:
                task.cancel()
            await asyncio.gather(*source_tasks, return_exceptions=True)
        assert len(processes) == 1 and processes[0].returncode is not None
        assert not camera.status().available
    asyncio.run(scenario())


@pytest.mark.parametrize("ending", ["invalidate_stream", "end_stream"])
def test_snapshot_owner_loss_wakes_waiter_without_restarting(ending):
    camera = provider()
    with ThreadPoolExecutor(1) as pool:
        pending = pool.submit(camera.capture)
        with pytest.raises(TimeoutError):
            pending.result(timeout=0.05)
        getattr(camera, ending)("owner")
        with pytest.raises(CameraFrameUnavailable, match="ended during capture"):
            pending.result(timeout=1)


def test_snapshot_timeout_does_not_restart_or_return_cache(monkeypatch):
    import src.bioxp.camera_provider as module
    monkeypatch.setattr(module, "CAPTURE_TIMEOUT_SECONDS", 0.02)
    camera = provider()
    old = camera.publish_stream_frame("owner", jpeg())
    with pytest.raises(CameraFrameUnavailable, match="timed out"):
        camera.capture()
    assert camera.latest() is old and camera._stream_owner == "owner"


def test_connected_backlog_uses_packet_pts_not_read_time(rig):
    api, camera, processes, _ = rig
    async def scenario():
        await api._start_owned_camera_session({})
        process = processes[0]
        owner = api._camera_session
        generation = camera.generation
        process.stderr.feed_data(b"#tb 0: 1/1000000\n")
        content = jpeg()
        async def publish(source_at, padding=b""):
            sequence = camera._sequence
            packet = content + padding
            pts = int(source_at.timestamp() * 1000000)
            process.stderr.feed_data(f"0, {pts}, {pts}, 33333, {len(packet)}, 0x00000000\n".encode())
            # Deliberately fragmented stdout, independent of sideband chunking.
            for offset in range(0, len(packet), 997):
                process.stdout.feed_data(packet[offset:offset + 997])
            for _ in range(1000):
                if camera._sequence > sequence:
                    return
                await asyncio.sleep(0.001)
            pytest.fail(f"packet did not publish: {owner}")
        try:
            old_at = datetime.now(timezone.utc) - timedelta(seconds=5)
            await publish(old_at)
            request = asyncio.create_task(api.camera_snapshot())
            await asyncio.sleep(0.02)
            await publish(old_at, padding=b"\x00" * 12)
            assert not request.done(), "new host publication incorrectly qualified old source pixels"
            fresh_at = datetime.now(timezone.utc)
            await publish(fresh_at)
            response = await asyncio.wait_for(request, 1)
            assert response.body == content
            assert response.headers["x-camera-source-frame-at"] == fresh_at.isoformat()
            assert response.headers["x-camera-source-timestamp-kind"] == "v4l2_packet_utc"
            assert camera.generation == generation and api._camera_session is owner
            assert len(processes) == 1
        finally:
            await api._stop_owned_camera_session(reason="offline backlog test cleanup")
    asyncio.run(scenario())


def test_connected_metadata_backpressure_stop_reaps(rig):
    api, camera, processes, _ = rig
    async def scenario():
        await api._start_owned_camera_session({})
        process = processes[0]
        content = jpeg()
        # Metadata producer fills its two-record queue while pixels are absent.
        process.stderr.feed_data(b"#tb 0: 1/1000000\n" +
                                 f"0, 0, 0, 1, {len(content)}, 0x00000000\n".encode() * 20)
        await asyncio.sleep(0.02)
        await asyncio.wait_for(api._stop_owned_camera_session(reason="offline blocked pipe cleanup"), 1)
        assert process.returncode is not None
        assert camera._stream_owner is None
    asyncio.run(scenario())
