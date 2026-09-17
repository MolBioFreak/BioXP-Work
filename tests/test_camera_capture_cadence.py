"""Exercise actual producer argv/reader using finite FFmpeg, never a device.

Run with the existing camera rig inside a route-free, device-free sandbox.
The sole input substitution is synthetic lavfi at a known capture cadence;
all output/filter/encoder arguments come from the real production caller.
"""
import asyncio
import os
import subprocess

import pytest
from starlette.requests import Request

# Same synthetic-process qualification mechanism as test_camera_stream_producer;
# kept local so this focused regression can run from a clean release worktree.
class Process:
    def __init__(self):
        self.stdout = asyncio.StreamReader()
        self.stderr = asyncio.StreamReader()
        self.returncode = None
        self.done = asyncio.Event()

    def terminate(self):
        self.returncode = -15
        self.stdout.feed_eof()
        self.stderr.feed_eof()
        self.done.set()

    kill = terminate

    async def wait(self):
        await self.done.wait()
        return self.returncode


async def settle():
    for _ in range(20):
        await asyncio.sleep(0.001)


@pytest.fixture
def rig(monkeypatch):
    from src.bioxp import api
    from src.bioxp.camera_provider import CameraIdentity, CameraProvider

    provider = CameraProvider(generation=17)
    # Device discovery is the hardware double; provider publication is real.
    monkeypatch.setattr(provider, "discover", lambda: CameraIdentity(
        "/synthetic/video7", "IZONE UVC 5M CAMERA", "2084", "f37d"))
    processes = []
    async def spawn(*argv, **kwargs):
        assert argv[argv.index("-i") + 1] == "/synthetic/video7"
        process = Process()
        processes.append(process)
        return process
    monkeypatch.setattr(api, "_camera_provider", provider)
    monkeypatch.setattr(api, "_camera_session", None)
    monkeypatch.setattr(api, "_camera_stream_state", {})
    monkeypatch.setattr(api, "_camera_projection_epoch", 0)
    monkeypatch.setattr(api, "_camera_owner_lock", asyncio.Lock())
    monkeypatch.setattr(api.shutil, "which", lambda _: "/fake/ffmpeg")
    monkeypatch.setattr(api.asyncio, "create_subprocess_exec", spawn)
    monkeypatch.setattr(api.hardware_state, "invalidate", lambda **_: None)
    monkeypatch.setattr(api.lifecycle_state, "record_camera_evidence", lambda _: None)
    return api, provider, processes, []


@pytest.mark.parametrize("capture_fps,scene", [(2, "testsrc2"), (30, "testsrc2"), (30, "color")])
def test_real_encoder_does_not_invent_capture_progress(rig, monkeypatch, tmp_path, capture_fps, scene):
    from src.bioxp.camera_provider import CameraJpegBuffer

    api, provider, processes, _ = rig
    spawn = api.asyncio.create_subprocess_exec
    encoded = []
    argv_seen = []
    from unittest.mock import Mock
    validation = Mock(wraps=provider._validate_jpeg)
    monkeypatch.setattr(provider, "_validate_jpeg", validation)

    async def synthetic_input(*argv, **kwargs):
        argv_seen.append(argv)
        output_args = list(argv[argv.index("-i") + 2:])
        completed = subprocess.run(
            [os.environ["CAMERA_TEST_FFMPEG"], "-hide_banner", "-loglevel", "error",
             "-f", "lavfi", "-i", f"{scene}=size=640x480:rate={capture_fps}:duration=2",
             *output_args],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=15, check=True,
        )
        encoded.extend(CameraJpegBuffer().feed(completed.stdout))
        return await spawn(*argv, **kwargs)

    monkeypatch.setattr(api.asyncio, "create_subprocess_exec", synthetic_input)

    async def scenario():
        await api._start_owned_camera_session({})
        response = await api.camera_mjpeg(Request({"type": "http", "query_string": b""}))
        iterator = response.body_iterator
        delivered = []
        try:
            for content in encoded:
                processes[0].stdout.feed_data(content)
                delivered.append(await asyncio.wait_for(anext(iterator), 2))
            status = await api.camera_status()
            print({"capture_fps": capture_fps, "input_frames": capture_fps * 2,
                   "encoded_frames": len(encoded), "published_sequence": status["frame_sequence"],
                   "delivered_parts": len(delivered), "output_args": list(argv_seen[0][argv_seen[0].index("-i")+2:])})
            assert len(encoded) == capture_fps * 2, "fps filter invented fresh capture frames"
            assert status["frame_sequence"] == len(encoded)
            expected_decodes = 1 + sum(a != b for a, b in zip(encoded, encoded[1:]))
            print({"scene": scene, "published": len(encoded), "full_decodes": validation.call_count,
                   "expected_decodes": expected_decodes})
            assert validation.call_count == expected_decodes
            if scene == "color":
                assert expected_decodes == 1
            assert (await api.camera_frame_latest()).body == encoded[-1]
            assert all(content in part for content, part in zip(encoded, delivered))
            # Actual producer multipart bytes cross the unchanged strict proxy parser.
            # Optional path supplied by the connected qualification runner, not a live server.
            if os.environ.get("CAMERA_TEST_BMS_API"):
                import sys
                sys.path.insert(0, os.environ["CAMERA_TEST_BMS_API"])
                import httpx
                sys.path.insert(0, os.path.join(os.environ["CAMERA_TEST_BMS_API"], "tests"))
                from test_bioxp_camera_boundary import Boundary

                class EncodedStream(httpx.AsyncByteStream):
                    async def __aiter__(self):
                        for part in delivered:
                            for offset in range(0, len(part), 997):
                                yield part[offset:offset + 997]

                boundary = Boundary(tmp_path / "bms")
                original_transport = boundary.transport
                async def transport(request):
                    if request.url.path == "/camera/mjpeg":
                        return httpx.Response(200, headers={
                            "Content-Type": "multipart/x-mixed-replace; boundary=frame"
                        }, stream=EncodedStream())
                    return await original_transport(request)
                boundary.transport = transport
                generation = await boundary.connect()
                messages = []
                async def receive():
                    await asyncio.Event().wait()
                async def send(message):
                    messages.append(message)
                await boundary.app({
                    "type": "http", "asgi": {"version": "3.0", "spec_version": "2.4"},
                    "method": "GET", "path": "/camera/mjpeg", "raw_path": b"/camera/mjpeg",
                    "query_string": f"expected_generation={generation}".encode(),
                    "headers": [], "scheme": "http", "server": ("bms", 80),
                    "client": ("browser", 1), "http_version": "1.1",
                }, receive, send)
                assert messages[0]["status"] == 200
                assert [m["body"] for m in messages if m.get("body")] == delivered
                assert boundary.connection._generation_leases[generation].lease_count == 0
                await boundary.connection.disconnect()
            sequence = status["frame_sequence"]
            await settle()
            assert (await api.camera_status())["frame_sequence"] == sequence
        finally:
            await iterator.aclose()
            await api._stop_owned_camera_session(reason="test cleanup")
            await settle()
        assert not provider.status().available
        assert len(processes) == 1

    asyncio.run(scenario())
