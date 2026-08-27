from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
CONTROLLER_PATH = ROOT / "scripts" / "bioxp_handlerctl.py"
INSTALLER_PATH = ROOT / "scripts" / "install_bioxp_handler_lifecycle.sh"
EMERGENCY_PATH = ROOT / "scripts" / "bioxp_emergency_motor_kill.sh"
RUNTIME_STORE_PATH = ROOT / "src" / "bioxp" / "runtime_audit_store.py"
API_PATH = ROOT / "src" / "bioxp" / "api.py"

spec = importlib.util.spec_from_file_location("bioxp_handlerctl", CONTROLLER_PATH)
assert spec is not None and spec.loader is not None
handlerctl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(handlerctl)


def test_running_stage_detection_and_refusal():
    status = {
        "startup": {
            "stages": {
                "constructor_pipette_stage": {"state": "passed"},
                "initialization_without_motion": {"state": "running"},
            }
        }
    }
    assert handlerctl.running_stages(status) == ["initialization_without_motion"]
    details = {"ActiveState": "active", "SubState": "running"}
    with pytest.raises(RuntimeError, match="refusing lifecycle mutation"):
        handlerctl.refuse_lifecycle_mutation(
            status,
            details,
            listener_open=True,
            recover_stuck=False,
        )
    handlerctl.refuse_lifecycle_mutation(
        status,
        details,
        listener_open=True,
        recover_stuck=True,
    )


def test_single_owner_handoff_is_unconditionally_denied(monkeypatch):
    monkeypatch.setattr(handlerctl, "systemctl", lambda *args, **kwargs: pytest.fail("handoff must not touch systemd"))
    with pytest.raises(RuntimeError, match="sole runtime owner"):
        handlerctl.handoff(force=True)


def test_controller_enforces_exact_loaded_unit_launcher_and_no_dropins(tmp_path, monkeypatch):
    unit = tmp_path / "bioxp-api.service"
    launcher = tmp_path / "bioxp-release-container-run"
    receipt = tmp_path / "release-identity.json"
    unit.write_bytes(b"unit-bytes\n")
    launcher.write_bytes(b"launcher-bytes\n")
    receipt.write_text(json.dumps({
        "schema": "bioxp.release.identity.v1",
        "status": "verified",
        "binding": {
            "unit_path": str(unit),
            "launcher_path": str(launcher),
            "unit_sha256": hashlib.sha256(unit.read_bytes()).hexdigest(),
            "launcher_sha256": hashlib.sha256(launcher.read_bytes()).hexdigest(),
        },
    }))
    monkeypatch.setattr(handlerctl, "UNIT_PATH", unit)
    monkeypatch.setattr(handlerctl, "LAUNCHER_PATH", launcher)
    monkeypatch.setattr(handlerctl, "RECEIPT_PATH", receipt)
    details = {
        "FragmentPath": str(unit),
        "DropInPaths": "",
        "ExecStart": f"{{ path={launcher} ; argv[]={launcher} ; ignore_errors=no ; }}",
    }
    handlerctl.verify_loaded_authority(details)
    details["DropInPaths"] = "/etc/systemd/system/bioxp-api.service.d/override.conf"
    with pytest.raises(RuntimeError, match="forbidden DropInPaths"):
        handlerctl.verify_loaded_authority(details)


def test_installer_consumes_verified_exact_materialization_and_verifies_loaded_authority():
    text = INSTALLER_PATH.read_text()
    assert 'source.get("mode") == "exact_commit_materialization"' in text
    assert 'root == f"/opt/bioxp/releases/{commit}"' in text
    assert "bioxp_source_manifest.py" in text
    assert "systemctl daemon-reload" in text
    assert "FragmentPath --value" in text
    assert "DropInPaths --value" in text
    assert "ExecStart --value" in text
    assert "installed unit digest mismatch after daemon-reload" in text
    assert "installed launcher digest mismatch after daemon-reload" in text
    assert 'action.lookup("unit") !== "bioxp-api.service"' in text
    assert "sudoers" not in text.lower()
    assert "NOPASSWD" not in text


def test_runtime_release_receipt_is_v5_append_only_and_readiness_gated():
    store = RUNTIME_STORE_PATH.read_text()
    api = API_PATH.read_text()
    assert "SCHEMA_VERSION = 5" in store
    assert "CREATE TABLE IF NOT EXISTS runtime_release_receipts" in store
    assert '"runtime_release_receipts",' in store
    for field in (
        "systemd_invocation_id",
        "application_pid",
        "application_cgroup",
        "application_start_time_ticks",
        "configuration_sha256",
        "oem_lock_sha256",
        "unit_sha256",
        "launcher_sha256",
        "source_manifest_sha256",
        "source_aggregate_sha256",
        "image_id",
        "image_inspection_receipt_sha256",
        "declared_listener_json",
        "observed_listener_json",
    ):
        assert field in store
    assert "record_runtime_release_start" in api
    assert "publish_runtime_release_receipt" in api
    assert "runtime_release_start_receipt_unavailable" in api


def test_emergency_preemption_failure_continues_to_physical_usb_stop_and_restarts_only_canonical_service():
    text = EMERGENCY_PATH.read_text()
    assert "preempt_api_warning=canonical_unit_stop_failed" in text
    assert "continuing_physical_usb_stop=true" in text
    assert text.index("preempt_api") < text.index("PYTHONPATH=src python3")
    assert "systemctl restart bioxp-api.service" in text
    assert "systemctl --user" not in text
    assert "bioxp-recovery" not in text


def test_controller_never_invokes_sudo_or_arbitrary_unit():
    text = CONTROLLER_PATH.read_text()
    assert 'UNIT = "bioxp-api.service"' in text
    assert '"sudo"' not in text
    assert "shell=True" not in text
