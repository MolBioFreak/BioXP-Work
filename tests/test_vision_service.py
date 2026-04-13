import asyncio
import importlib
import sys
import types

import pytest
from fastapi import HTTPException

from src.bioxp.domain.capabilities import CapabilityName, CapabilityRegistry
from src.bioxp.services.vision_service import run_barcode_read_command, run_inspection_command
from src.bioxp.vision.barcode import BarcodeReadCommand
from src.bioxp.vision.inspection import InspectionCommand


async def _fake_run_blocking(label, func, timeout_s=30.0):
    del label, timeout_s
    return func()


def _capabilities(*enabled_names):
    registry = CapabilityRegistry()
    for name in enabled_names:
        registry.register(name, enabled=True)
    return registry


def load_api(monkeypatch):
    usb_pkg = types.ModuleType("usb")
    usb_core = types.ModuleType("usb.core")
    usb_util = types.ModuleType("usb.util")
    usb_pkg.core = usb_core
    usb_pkg.util = usb_util
    monkeypatch.setitem(sys.modules, "usb", usb_pkg)
    monkeypatch.setitem(sys.modules, "usb.core", usb_core)
    monkeypatch.setitem(sys.modules, "usb.util", usb_util)
    for name in [
        "src.bioxp.api",
        "src.bioxp.usb_driver",
        "src.bioxp.vision.barcode",
        "src.bioxp.vision.inspection",
        "src.bioxp.vision",
        "src.bioxp.services.vision_service",
        "src.bioxp.services",
        "src.bioxp",
    ]:
        sys.modules.pop(name, None)
    return importlib.import_module("src.bioxp.api")


def test_run_inspection_command_returns_snapshot_backed_result():
    command = InspectionCommand(device="/dev/video0", location_id="reagent_rack", requested_checks=("focus", "framing"))
    snapshot = {
        "ok": True,
        "device": "/dev/video0",
        "path": "/tmp/capture.jpg",
        "size": 2048,
        "image_b64": "ZmFrZQ==",
        "image_error": None,
        "pick": {"ok": True},
    }

    result = asyncio.run(
        run_inspection_command(
            command,
            get_capabilities=lambda: _capabilities(CapabilityName.INSPECTION),
            capture_snapshot=lambda device: {**snapshot, "device": device},
            run_blocking=_fake_run_blocking,
        )
    )

    assert result["ok"] is True
    assert result["device"] == "/dev/video0"
    assert result["location_id"] == "reagent_rack"
    assert result["snapshot"]["path"] == "/tmp/capture.jpg"
    assert "snapshot_captured" in result["observations"]


def test_run_barcode_read_command_extracts_candidates_from_snapshot_metadata():
    command = BarcodeReadCommand(device="/dev/video0", location_id="reagent_rack", symbologies=("qr",))
    snapshot = {
        "ok": True,
        "device": "/dev/video0",
        "path": "/tmp/capture.jpg",
        "size": 2048,
        "image_b64": None,
        "image_error": None,
        "metadata": {
            "barcodes": [
                {"value": "ABC123", "symbology": "qr", "confidence": 0.98},
            ]
        },
    }

    result = asyncio.run(
        run_barcode_read_command(
            command,
            get_capabilities=lambda: _capabilities(CapabilityName.INSPECTION),
            capture_snapshot=lambda device: {**snapshot, "device": device},
            run_blocking=_fake_run_blocking,
        )
    )

    assert result["ok"] is True
    assert result["barcode_count"] == 1
    assert result["barcodes"][0]["value"] == "ABC123"
    assert result["capability_used"] == "inspection"


def test_run_barcode_read_command_rejects_when_capability_missing():
    command = BarcodeReadCommand(device="/dev/video0")

    with pytest.raises(HTTPException) as exc_info:
        asyncio.run(
            run_barcode_read_command(
                command,
                get_capabilities=lambda: _capabilities(),
                capture_snapshot=lambda device: {"ok": True, "device": device},
                run_blocking=_fake_run_blocking,
            )
        )

    assert exc_info.value.status_code == 409
    assert "barcode" in str(exc_info.value.detail).lower() or "inspection" in str(exc_info.value.detail).lower()


def test_vision_routes_delegate_to_service(monkeypatch):
    api = load_api(monkeypatch)
    calls = []

    async def fake_inspect(command, **kwargs):
        calls.append(("inspect", command))
        return {"ok": True, "location_id": command.location_id}

    async def fake_barcode(command, **kwargs):
        calls.append(("barcode", command))
        return {"ok": True, "barcode_count": 1}

    monkeypatch.setattr(api, "run_inspection_command", fake_inspect)
    monkeypatch.setattr(api, "run_barcode_read_command", fake_barcode)

    inspect_result = asyncio.run(
        api.vision_inspect(api.InspectionRequest(device="/dev/video0", location_id="reagent_rack", requested_checks=["focus"]))
    )
    barcode_result = asyncio.run(
        api.vision_barcode_read(api.BarcodeReadRequest(device="/dev/video0", location_id="reagent_rack", symbologies=["qr"]))
    )

    assert inspect_result["location_id"] == "reagent_rack"
    assert barcode_result["barcode_count"] == 1
    assert [name for name, _ in calls] == ["inspect", "barcode"]
