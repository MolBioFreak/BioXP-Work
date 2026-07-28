from __future__ import annotations

import asyncio
import hashlib
import time
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

import pytest
from pydantic import ValidationError


def _v4l2_output(card: str, *, device_caps: tuple[str, ...], top_level_capture: bool = True) -> str:
    capabilities = "\t\tVideo Capture\n" if top_level_capture else ""
    caps = "".join(f"\t\t{value}\n" for value in device_caps)
    return (
        "Driver Info:\n"
        f"\tCard type        : {card}\n"
        "\tCapabilities     : 0x84a00001\n"
        f"{capabilities}"
        "\tDevice Caps      : 0x04200000\n"
        f"{caps}"
        "Priority: 2\n"
    )


def _add_video_node(root: Path, dev_root: Path, name: str, card: str) -> None:
    usb = root.parent / "devices" / "usb-camera"
    node = usb / "video4linux" / name
    node.mkdir(parents=True, exist_ok=True)
    (usb / "idVendor").write_text("2084\n", encoding="utf-8")
    (usb / "idProduct").write_text("f37d\n", encoding="utf-8")
    (node / "name").write_text(card, encoding="utf-8")
    root.mkdir(parents=True, exist_ok=True)
    (root / name).symlink_to(node, target_is_directory=True)
    dev_root.mkdir(parents=True, exist_ok=True)
    (dev_root / name).touch()


def test_serial206_discovery_rejects_metadata_node_and_selects_capture_node(tmp_path):
    from src.bioxp.camera_provider import CameraProvider, EXPECTED_CAMERA_CARDS

    card = sorted(EXPECTED_CAMERA_CARDS)[0]
    sysfs = tmp_path / "sys" / "class" / "video4linux"
    dev = tmp_path / "dev"
    _add_video_node(sysfs, dev, "video0", card)
    _add_video_node(sysfs, dev, "video1", card)

    calls: list[list[str]] = []

    def runner(argv, **kwargs):
        calls.append(list(argv))
        device = Path(argv[argv.index("--device") + 1]).name
        if device == "video0":
            output = _v4l2_output(
                card,
                device_caps=("Metadata Capture", "Streaming"),
                top_level_capture=True,
            )
        else:
            output = _v4l2_output(
                card,
                device_caps=("Video Capture", "Streaming"),
                top_level_capture=True,
            )
        return SimpleNamespace(returncode=0, stdout=output, stderr="")

    provider = CameraProvider(sysfs_root=sysfs, dev_root=dev, runner=runner, generation=7)

    identity = provider.discover()

    assert identity.device == str(dev / "video1")
    assert [Path(call[2]).name for call in calls] == ["video0", "video1"]


def test_camera_routes_are_finite_and_callers_cannot_select_device(monkeypatch):
    from src.bioxp import api

    camera_routes = {
        route.path
        for route in api.app.routes
        if getattr(route, "path", "").startswith("/camera")
    }
    assert camera_routes == {
        "/camera/status",
        "/camera/frame/latest",
        "/camera/snapshot",
    }

    with pytest.raises(ValidationError):
        api.CameraSnapshotRequest(device="/dev/video99")

    class Provider:
        def capture(self):
            raise AssertionError("provider must not run for a rejected caller-selected device")

    monkeypatch.setattr(api, "_camera_provider", Provider())
    from fastapi.testclient import TestClient

    response = TestClient(api.app).post("/camera/snapshot", json={"device": "/dev/video99"})
    assert response.status_code == 422


def test_snapshot_route_returns_exact_provider_owned_jpeg_and_integrity_headers(monkeypatch):
    from src.bioxp import api
    from src.bioxp.camera_provider import CameraFrame, CameraIdentity

    content = b"\xff\xd8provider-owned-pixels\xff\xd9"
    digest = hashlib.sha256(content).hexdigest()
    frame = CameraFrame(
        content=content,
        sequence=3,
        captured_at=datetime(2026, 7, 28, tzinfo=timezone.utc),
        provider_generation=9,
        content_sha256=digest,
        identity=CameraIdentity(
            device="/dev/video7",
            card="IZONE UVC 5M CAMERA",
            usb_vid="2084",
            usb_pid="f37d",
        ),
    )

    class Provider:
        def capture(self):
            return frame

    monkeypatch.setattr(api, "_camera_provider", Provider())
    response = asyncio.run(api.camera_snapshot(api.CameraSnapshotRequest()))

    assert response.body == content
    assert response.media_type == "image/jpeg"
    assert response.headers["content-length"] == str(len(content))
    assert response.headers["etag"] == f'"{digest}"'
    assert response.headers["x-content-sha256"] == digest


def test_camera_status_does_not_block_event_loop(monkeypatch):
    from src.bioxp import api

    class Provider:
        def status(self):
            time.sleep(0.2)
            return SimpleNamespace(to_payload=lambda: {"available": False})

    monkeypatch.setattr(api, "_camera_provider", Provider())

    async def scenario():
        started = time.perf_counter()
        task = asyncio.create_task(api.camera_status())
        await asyncio.sleep(0.02)
        event_loop_delay = time.perf_counter() - started
        await task
        return event_loop_delay

    assert asyncio.run(scenario()) < 0.1
