"""Networkless producer-path tests: synthetic sysfs and fake async ffmpeg."""
import asyncio
import hashlib
import io
from types import SimpleNamespace

import pytest
from PIL import Image

from test_camera_provider import _add_video_node, _v4l2_output


def jpeg(color="red"):
    output = io.BytesIO()
    Image.new("RGB", (640, 480), color).save(output, format="JPEG")
    return output.getvalue()


class Process:
    def __init__(self):
        self.stdout = asyncio.StreamReader()
        self.stderr = asyncio.StreamReader()
        self.returncode = None
        self.terminated = 0
        self.killed = 0
        self.waited = 0
        self.done = asyncio.Event()

    def terminate(self):
        self.terminated += 1
        self.returncode = -15
        self.stdout.feed_eof()
        self.stderr.feed_eof()
        self.done.set()

    def kill(self):
        self.killed += 1
        self.terminate()

    async def wait(self):
        self.waited += 1
        await self.done.wait()
        return self.returncode


@pytest.fixture
def rig(monkeypatch, tmp_path):
    from src.bioxp import api
    from src.bioxp.camera_provider import CameraProvider
    card = "IZONE UVC 5M CAMERA"
    sysfs, dev = tmp_path / "sys" / "video4linux", tmp_path / "dev"
    _add_video_node(sysfs, dev, "video7", card)
    calls, processes = [], []

    def runner(argv, **kwargs):
        calls.append(argv)
        assert argv[0] == "v4l2-ctl", "second device capture process attempted"
        return SimpleNamespace(returncode=0, stdout=_v4l2_output(card, device_caps=("Video Capture", "Streaming")), stderr="")

    async def spawn(*argv, **kwargs):
        assert argv[argv.index("-i") + 1] == str(dev / "video7")
        proc = Process()
        processes.append(proc)
        return proc

    provider = CameraProvider(sysfs_root=sysfs, dev_root=dev, runner=runner, generation=17)
    monkeypatch.setattr(api, "_camera_provider", provider)
    monkeypatch.setattr(api, "_camera_session", None)
    monkeypatch.setattr(api, "_camera_stream_state", {})
    monkeypatch.setattr(api, "_camera_projection_epoch", 0)
    monkeypatch.setattr(api, "_pick_stream_device", lambda _: {"ok": True, "device": str(dev / "video7")})
    monkeypatch.setattr(api.shutil, "which", lambda _: "/fake/ffmpeg")
    monkeypatch.setattr(api.asyncio, "create_subprocess_exec", spawn)
    monkeypatch.setattr(api.hardware_state, "invalidate", lambda **_: None)
    monkeypatch.setattr(api.lifecycle_state, "record_camera_evidence", lambda _: None)
    if hasattr(api, "_camera_owner_lock"):
        monkeypatch.setattr(api, "_camera_owner_lock", asyncio.Lock())
    return api, provider, processes, calls


async def settle():
    for _ in range(20):
        await asyncio.sleep(0.001)


def test_actual_stream_reader_publishes_status_latest_and_snapshot(rig):
    api, provider, processes, calls = rig

    async def scenario():
        await api._start_owned_camera_session({})
        proc = processes[-1]
        try:
            first = jpeg()
            proc.stdout.feed_data(first)
            await settle()
            status = await api.camera_status()
            assert status["frame_sequence"] == 1, "stream pixels never reached provider status"
            assert status["available"] is True
            assert (await api.camera_frame_latest()).body == first
            assert (await api.camera_snapshot()).body == first
            proc.stdout.feed_data(jpeg("blue"))
            await settle()
            status2 = await api.camera_status()
            assert status2["frame_sequence"] == 2
            assert status2["content_sha256"] == hashlib.sha256(jpeg("blue")).hexdigest()
            assert status2["provider_generation"] == status["provider_generation"]
            assert len(processes) == 1
        finally:
            await api._stop_owned_camera_session(reason="test cleanup")
            await settle()

    asyncio.run(scenario())


def test_stream_requires_progress_and_expires_without_fabrication(rig):
    from datetime import datetime, timedelta, timezone
    api, provider, processes, _ = rig
    now = [datetime(2026, 9, 7, tzinfo=timezone.utc)]
    provider._clock = lambda: now[0]

    async def scenario():
        start = await api._start_owned_camera_session({})
        try:
            assert start["state"] == "starting"
            assert (await api._start_owned_camera_session({}))["state"] == "starting"
            with pytest.raises(api.HTTPException) as exc:
                await api.camera_snapshot()
            assert exc.value.status_code == 503
            processes[0].stdout.feed_data(jpeg())
            await settle()
            assert (await api._start_owned_camera_session({}))["state"] == "starting"
            processes[0].stdout.feed_data(jpeg())  # identical pixels are valid actual frames
            await settle()
            assert (await api._start_owned_camera_session({}))["state"] == "live"
            before = await api.camera_status()
            now[0] += timedelta(seconds=31)
            after = await api.camera_status()
            assert after["frame_sequence"] == before["frame_sequence"] == 2
            assert after["available"] is False
            assert (await api._start_owned_camera_session({}))["state"] == "starting"
            with pytest.raises(api.HTTPException):
                await api.camera_snapshot()
            assert len(processes) == 1
        finally:
            await api._stop_owned_camera_session(reason="cleanup")
    asyncio.run(scenario())


def test_invalid_jpeg_never_advances_or_reaches_viewer(rig):
    api, provider, processes, _ = rig

    async def scenario():
        await api._start_owned_camera_session({})
        try:
            processes[0].stdout.feed_data(b"\xff\xd8invalid\xff\xd9")
            await settle()
            status = await api.camera_status()
            assert status["available"] is False
            assert status["frame_sequence"] is None
            assert status["dropped_frames"] == 1
            assert api._camera_session["queue"].empty()
            processes[0].stdout.feed_data(jpeg() * 4)
            await settle()
            assert provider.latest().sequence == 4
            assert provider.status().dropped_frames == 3  # invalid + two evictions
            assert api._camera_session["queue"].qsize() == 2
        finally:
            await api._stop_owned_camera_session(reason="cleanup")
    asyncio.run(scenario())


def test_partial_buffer_bounded_and_split_markers_recover():
    from src.bioxp.camera_provider import CameraJpegBuffer, MAX_JPEG_BYTES
    parser = CameraJpegBuffer()
    assert list(parser.feed(b"\xff\xd8" + b"x" * (MAX_JPEG_BYTES * 4))) == []
    assert len(parser.buffer) <= MAX_JPEG_BYTES
    assert parser.dropped == 1
    content = jpeg()
    assert list(parser.feed(b"garbage\xff")) == []
    assert list(parser.feed(content[1:-1])) == []
    assert list(parser.feed(content[-1:])) == [content]
    assert len(parser.buffer) == 0


@pytest.mark.parametrize("rejection", ["wrong_usb", "metadata", "ambiguous"])
def test_stream_uses_exact_provider_admission_not_video0(rig, monkeypatch, rejection):
    api, provider, processes, _ = rig
    monkeypatch.setattr(api, "_pick_stream_device", lambda _: pytest.fail("legacy arbitrary selector used"))
    if rejection == "wrong_usb":
        (provider._sysfs_root / "video7").resolve().parents[1].joinpath("idVendor").write_text("ffff")
    elif rejection == "metadata":
        provider._runner = lambda *_, **__: SimpleNamespace(returncode=0, stdout=_v4l2_output("IZONE UVC 5M CAMERA", device_caps=("Metadata Capture",)), stderr="")
    else:
        _add_video_node(provider._sysfs_root, provider._dev_root, "video8", "IZONE UVC 5M CAMERA")
    async def scenario():
        with pytest.raises(api.HTTPException) as exc:
            await api._start_owned_camera_session({})
        assert exc.value.status_code == 503
        assert not processes
        assert provider.status().available is False
    asyncio.run(scenario())


def test_concurrent_start_stop_restart_owns_one_process_and_generation(rig):
    api, provider, processes, _ = rig
    async def scenario():
        results = await asyncio.gather(*(api._start_owned_camera_session({}) for _ in range(6)))
        assert len(processes) == 1
        assert len({r["stream_id"] for r in results}) == 1
        processes[0].stdout.feed_data(jpeg())
        await settle()
        old = provider.latest()
        old_session = api._camera_session
        await api._stop_owned_camera_session(reason="explicit stop")
        assert processes[0].waited > 0
        assert old_session["reader_task"].done()
        assert old_session["stderr_task"].done()
        assert provider.status().available is False
        await api._start_owned_camera_session({})
        try:
            assert provider.generation != old.provider_generation
            with pytest.raises(Exception, match="obsolete"):
                provider.publish_stream_frame(old_session["session_id"], jpeg())
            provider.end_stream(old_session["session_id"])
            assert api._camera_stream_state["active"] is True
            processes[1].stdout.feed_data(jpeg("blue"))
            await settle()
            assert provider.latest().content == jpeg("blue")
        finally:
            await api._stop_owned_camera_session(reason="cleanup")
    asyncio.run(scenario())


def test_old_reader_finally_cannot_clear_replacement_state(rig):
    api, provider, processes, _ = rig
    async def scenario():
        await api._start_owned_camera_session({})
        old = api._camera_session
        # Simulate a stale callback after another ownership generation was installed.
        replacement = {"session_id": "replacement"}
        api._camera_session = replacement
        api._camera_projection_epoch += 1
        api._camera_stream_state.update(active=True, last_frame_at=123)
        old["reader_task"].cancel()
        await asyncio.gather(old["reader_task"], return_exceptions=True)
        await api._reap_camera_session(old)
        assert api._camera_session is replacement
        assert api._camera_stream_state["active"] is True
        assert api._camera_stream_state["last_frame_at"] == 123
        api._camera_session = None
    asyncio.run(scenario())


def test_eof_reaps_process_drains_bounded_stderr_and_invalidates(rig):
    api, provider, processes, _ = rig
    async def scenario():
        await api._start_owned_camera_session({})
        session = api._camera_session
        proc = processes[0]
        proc.stdout.feed_data(jpeg() * 2)
        proc.stderr.feed_data(b"e" * 100000)
        await settle()
        assert len(session["stderr_tail"]) <= 4096
        proc.stdout.feed_eof()
        await asyncio.wait_for(session["reader_task"], 1)
        assert proc.terminated == 1 and proc.waited > 0
        assert session["stderr_task"].done()
        assert not provider.status().available
        assert api._camera_stream_phase(session) == "off"
        await api._stop_owned_camera_session(reason="cleanup")
    asyncio.run(scenario())


def test_cancelled_start_finishes_spawn_then_reaps(rig, monkeypatch):
    api, provider, processes, _ = rig
    original = api.asyncio.create_subprocess_exec
    async def scenario():
        entered, release = asyncio.Event(), asyncio.Event()
        async def delayed(*args, **kwargs):
            entered.set()
            await release.wait()
            return await original(*args, **kwargs)
        monkeypatch.setattr(api.asyncio, "create_subprocess_exec", delayed)
        task = asyncio.create_task(api._start_owned_camera_session({}))
        await entered.wait()
        task.cancel()
        release.set()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert processes[0].returncode is not None
        assert processes[0].waited > 0
        assert api._camera_session is None
        assert provider._stream_owner is None
    asyncio.run(scenario())


def test_spawn_failure_releases_reserved_identity(rig, monkeypatch):
    api, provider, _, _ = rig
    async def fail(*_, **__):
        raise OSError("synthetic spawn failure")
    monkeypatch.setattr(api.asyncio, "create_subprocess_exec", fail)
    async def scenario():
        with pytest.raises(OSError, match="synthetic"):
            await api._start_owned_camera_session({})
        assert provider._stream_owner is None
        assert not provider.status().available
    asyncio.run(scenario())


@pytest.mark.parametrize("route", ["camera_snapshot", "camera_frame_latest", "camera_status"])
def test_old_generation_read_response_rejected(rig, monkeypatch, route):
    api, provider, processes, _ = rig
    original = api.run_in_threadpool
    async def scenario():
        await api._start_owned_camera_session({})
        processes[0].stdout.feed_data(jpeg())
        await settle()
        async def delayed(func, *args, **kwargs):
            value = await original(func, *args, **kwargs)
            if func.__name__ in {"capture", "latest", "status"}:
                await api._stop_owned_camera_session(reason="racing response")
            return value
        monkeypatch.setattr(api, "run_in_threadpool", delayed)
        with pytest.raises(api.HTTPException) as exc:
            await getattr(api, route)()
        assert exc.value.status_code == 503
    asyncio.run(scenario())


def test_discovery_off_event_loop_and_no_shared_motion_lock(rig):
    import threading
    api, provider, _, _ = rig
    original = provider._runner
    runner_threads = []
    def runner(*args, **kwargs):
        runner_threads.append(threading.get_ident())
        return original(*args, **kwargs)
    provider._runner = runner
    async def scenario():
        loop_thread = threading.get_ident()
        await api._start_owned_camera_session({})
        await api._stop_owned_camera_session(reason="cleanup")
        assert runner_threads and all(t != loop_thread for t in runner_threads)
    asyncio.run(scenario())


def test_cancelled_stop_waits_for_kill_reap_before_releasing(rig, monkeypatch):
    api, provider, processes, _ = rig
    async def scenario():
        await api._start_owned_camera_session({})
        proc = processes[0]
        entered = asyncio.Event()
        real_terminate = proc.terminate
        def ignore_terminate():
            entered.set()
        def kill():
            proc.killed += 1
            real_terminate()
        proc.terminate = ignore_terminate
        proc.kill = kill
        real_wait_for = asyncio.wait_for
        async def fast_timeout(awaitable, timeout):
            return await real_wait_for(awaitable, 0.02 if timeout == 3.0 else timeout)
        monkeypatch.setattr(api.asyncio, "wait_for", fast_timeout)
        task = asyncio.create_task(api._stop_owned_camera_session(reason="explicit stop"))
        await entered.wait()
        task.cancel()
        assert provider._stream_owner is not None
        with pytest.raises(asyncio.CancelledError):
            await task
        assert proc.killed == 1
        assert proc.waited >= 2
        assert provider._stream_owner is None
    asyncio.run(scenario())


def test_start_provider_replacement_rejected_and_process_reaped(rig, monkeypatch):
    api, provider, processes, _ = rig
    original = api.asyncio.create_subprocess_exec
    async def replaced(*args, **kwargs):
        proc = await original(*args, **kwargs)
        monkeypatch.setattr(api, "_camera_provider", object())
        return proc
    monkeypatch.setattr(api.asyncio, "create_subprocess_exec", replaced)
    async def scenario():
        with pytest.raises(api.HTTPException) as exc:
            await api._start_owned_camera_session({})
        assert exc.value.status_code == 503
        assert processes[0].returncode is not None
        assert processes[0].waited > 0
        assert provider._stream_owner is None
    asyncio.run(scenario())


def test_missing_stdout_reaped(rig, monkeypatch):
    api, provider, processes, _ = rig
    original = api.asyncio.create_subprocess_exec
    async def no_stdout(*args, **kwargs):
        proc = await original(*args, **kwargs)
        stdout = proc.stdout
        original_terminate = proc.terminate
        def terminate():
            proc.stdout = stdout
            original_terminate()
            proc.stdout = None
        proc.terminate = terminate
        proc.stdout = None
        return proc
    monkeypatch.setattr(api.asyncio, "create_subprocess_exec", no_stdout)
    async def scenario():
        with pytest.raises(api.HTTPException) as exc:
            await api._start_owned_camera_session({})
        assert exc.value.status_code == 500
        assert processes[0].waited > 0
        assert provider._stream_owner is None
    asyncio.run(scenario())


def test_asgi_stream_status_latest_snapshot_share_pixels_and_state_is_json(rig):
    import httpx
    api, provider, processes, _ = rig
    async def scenario():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=api.app), base_url="http://offline") as client:
            assert (await client.post("/camera/stream/start", json={})).status_code == 200
            try:
                processes[0].stdout.feed_data(jpeg() * 2)
                await settle()
                response = await client.get("/camera/status")
                assert response.status_code == 200
                assert response.json()["frame_sequence"] == 2
                state = await client.get("/camera/stream/state")
                assert state.status_code == 200
                assert state.json()["state"] == "live"
                assert (await client.get("/camera/frame/latest")).content == jpeg()
                assert (await client.post("/camera/snapshot", json={})).content == jpeg()
                assert len(processes) == 1
            finally:
                assert (await client.post("/camera/stream/stop", json={})).status_code == 200
            assert (await client.get("/camera/status")).json()["available"] is False
            assert (await client.get("/camera/frame/latest")).status_code == 503
    asyncio.run(scenario())


def test_delayed_viewer_stop_cannot_stop_replacement_or_new_viewer(rig):
    api, provider, processes, _ = rig
    async def scenario():
        first = await api._start_owned_camera_session({})
        await api._stop_owned_camera_session(reason="explicit stop")
        second = await api._start_owned_camera_session({})
        try:
            await api._stop_owned_camera_session(reason="viewer grace expired", expected_session_id=first["stream_id"])
            assert api._camera_session["session_id"] == second["stream_id"]
            assert processes[1].returncode is None
            api._camera_session["viewers"] = 1
            await api._stop_owned_camera_session(reason="viewer grace expired", expected_session_id=second["stream_id"])
            assert processes[1].returncode is None
        finally:
            await api._stop_owned_camera_session(reason="cleanup")
    asyncio.run(scenario())


def test_generation_changes_survive_json_number_precision(rig):
    from src.bioxp.camera_provider import MAX_PROVIDER_GENERATION
    api, provider, _, _ = rig
    provider._generation = MAX_PROVIDER_GENERATION
    async def scenario():
        old = provider.generation
        await api._start_owned_camera_session({})
        assert float(provider.generation) != float(old)
        assert 0 < provider.generation <= MAX_PROVIDER_GENERATION
        await api._stop_owned_camera_session(reason="cleanup")
        assert provider.generation == 2
    asyncio.run(scenario())


def test_evidence_callback_failure_does_not_leak_device(rig, monkeypatch):
    api, provider, processes, _ = rig
    def fail(**kwargs):
        raise RuntimeError("synthetic evidence failure")
    monkeypatch.setattr(api.hardware_state, "invalidate", fail)
    async def scenario():
        with pytest.raises(RuntimeError, match="synthetic"):
            await api._start_owned_camera_session({})
        assert processes[0].returncode is not None
        assert processes[0].waited > 0
        assert provider._stream_owner is None
        assert api._camera_session is None
    asyncio.run(scenario())
