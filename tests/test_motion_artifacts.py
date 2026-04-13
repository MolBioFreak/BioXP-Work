import asyncio
import importlib
import json
import sys
import types
from pathlib import Path

import pytest


BIOXP_VALIDATION_ARTIFACT_ROOT_ENV = "BIOXP_VALIDATION_ARTIFACT_ROOT"


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
        "src.bioxp.services.artifact_service",
        "src.bioxp.services",
        "src.bioxp",
    ]:
        sys.modules.pop(name, None)
    return importlib.import_module("src.bioxp.api")


def test_create_motion_validation_bundle_writes_expected_files(monkeypatch, tmp_path):
    root = tmp_path / "bioxp_validation"
    monkeypatch.setenv(BIOXP_VALIDATION_ARTIFACT_ROOT_ENV, str(root))
    api = load_api(monkeypatch)

    bundle = api.create_motion_validation_bundle(
        command="relative",
        axis="x",
        request_payload={
            "axis": "x",
            "steps": 120,
            "wait_timeout_s": 5.0,
            "capture_bundle": True,
            "dry_run_bundle": False,
            "operator_note": "Marker aligned",
            "snapshot_refs": ["before.jpg", "after.jpg"],
        },
        response_payload={
            "axis": "x",
            "position_delta": 120,
            "motion_truth": {"evidence_level": "controller_only"},
            "prep_policy": {"note": "Fresh board activation and axis prep executed."},
        },
        motion_truth={"evidence_level": "controller_only"},
        prep_policy={"note": "Fresh board activation and axis prep executed."},
        operator_note="Marker aligned",
        snapshot_refs=["before.jpg", "after.jpg"],
        dry_run=False,
    )

    bundle_dir = Path(bundle["bundle_dir"])
    metadata_path = Path(bundle["metadata_path"])
    request_path = Path(bundle["request_path"])
    response_path = Path(bundle["response_path"])

    assert bundle_dir.is_dir()
    assert metadata_path.is_file()
    assert request_path.is_file()
    assert response_path.is_file()
    assert str(root) in str(bundle_dir)

    metadata = json.loads(metadata_path.read_text())
    request_payload = json.loads(request_path.read_text())
    response_payload = json.loads(response_path.read_text())

    assert metadata["schema_version"] == "bioxp.motion_validation_bundle/v1"
    assert metadata["bundle_kind"] == "motion_validation"
    assert metadata["request_summary"]["axis"] == "x"
    assert metadata["response_summary"]["position_delta"] == 120
    assert metadata["response_summary"]["truth_evidence_level"] == "controller_only"
    assert metadata["snapshot_ref_count"] == 2
    assert metadata["operator_note"] == "Marker aligned"
    assert request_payload["snapshot_refs"] == ["before.jpg", "after.jpg"]
    assert response_payload["axis"] == "x"


def test_create_motion_validation_bundle_omits_empty_operator_note(monkeypatch, tmp_path):
    root = tmp_path / "bioxp_validation"
    monkeypatch.setenv(BIOXP_VALIDATION_ARTIFACT_ROOT_ENV, str(root))
    api = load_api(monkeypatch)

    bundle = api.create_motion_validation_bundle(
        command="home",
        axis="y",
        request_payload={"axis": "y", "capture_bundle": True, "dry_run_bundle": True},
        response_payload={"axis": "y", "dry_run": True, "skipped_hardware_io": True},
        operator_note=None,
        snapshot_refs=None,
        dry_run=True,
    )

    metadata = json.loads(Path(bundle["metadata_path"]).read_text())
    assert metadata["operator_note"] is None
    assert bundle["operator_note"] is None
    assert metadata["snapshot_refs"] == []


def test_relative_move_dry_run_bundle_skips_hardware_io(monkeypatch, tmp_path):
    root = tmp_path / "dry_run_validation"
    monkeypatch.setenv(BIOXP_VALIDATION_ARTIFACT_ROOT_ENV, str(root))
    api = load_api(monkeypatch)
    monkeypatch.setattr(api, "_get_tester", lambda: (_ for _ in ()).throw(AssertionError("hardware should not be touched")))

    req = api.MoveRelativeRequest(
        axis=api.AxisName.X,
        steps=12,
        capture_bundle=True,
        dry_run_bundle=True,
        operator_note="Dry run before camera capture",
        snapshot_refs=["before.png", "after.png"],
    )

    result = asyncio.run(api.move_axis_relative(req))

    assert result["dry_run"] is True
    assert result["skipped_hardware_io"] is True
    assert result["motion_truth"]["evidence_level"] == "artifact_only"
    assert result["artifact_bundle"]["dry_run"] is True

    bundle_dir = Path(result["artifact_bundle"]["bundle_dir"])
    metadata = json.loads((bundle_dir / "metadata.json").read_text())
    request_payload = json.loads((bundle_dir / "request.json").read_text())
    response_payload = json.loads((bundle_dir / "response.json").read_text())

    assert metadata["dry_run"] is True
    assert metadata["response_summary"]["skipped_hardware_io"] is True
    assert request_payload["operator_note"] == "Dry run before camera capture"
    assert request_payload["snapshot_refs"] == ["before.png", "after.png"]
    assert response_payload["message"].startswith("Dry-run validation bundle created")


def test_dry_run_bundle_requires_capture_bundle(monkeypatch):
    api = load_api(monkeypatch)
    req = api.MoveRelativeRequest(axis=api.AxisName.X, steps=5, capture_bundle=False, dry_run_bundle=True)

    with pytest.raises(api.HTTPException) as exc_info:
        asyncio.run(api.move_axis_relative(req))

    assert exc_info.value.status_code == 400
    assert "capture_bundle=true" in str(exc_info.value.detail)
