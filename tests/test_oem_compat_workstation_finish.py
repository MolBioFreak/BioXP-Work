import json
import subprocess
from pathlib import Path

from fastapi.testclient import TestClient


def test_oem_compat_api_startup_dry_run_can_export_trace_artifact(tmp_path):
    from fastapi import FastAPI
    from src.bioxp.oem_compat.api import router

    app = FastAPI()
    app.include_router(router)
    client = TestClient(app)
    artifact = tmp_path / "startup_trace.json"

    resp = client.post(
        "/oem-compat/startup/dry-run",
        json={"run_homing": True, "artifact_path": str(artifact)},
    )

    assert resp.status_code == 200
    body = resp.json()
    assert body["artifact_path"] == str(artifact)
    assert body["replay_ok"] is True
    assert artifact.exists()
    payload = json.loads(artifact.read_text())
    assert payload["format"] == "bioxp-oem-compat-trace-v1"
    assert payload["frame_count"] == body["frame_count"]
    assert payload["frames"][0]["oem_payload_hex"]


def test_oem_compat_api_rejects_trace_artifact_outside_allowed_roots(tmp_path):
    from fastapi import FastAPI
    from src.bioxp.oem_compat.api import router

    app = FastAPI()
    app.include_router(router)
    client = TestClient(app)

    resp = client.post(
        "/oem-compat/startup/dry-run",
        json={"run_homing": False, "artifact_path": "/etc/bioxp-should-not-write.json"},
    )

    assert resp.status_code == 400
    assert "artifact_path" in resp.json()["detail"]


def test_workstation_readiness_script_writes_report_and_artifacts(tmp_path):
    repo = Path("/home/dalab/Desktop/BioXP 3200 Development Work/bioxp_re")
    report = tmp_path / "readiness.json"
    result = subprocess.run(
        ["python3", "scripts/oem_compat_workstation_readiness.py", "--output", str(report), "--skip-pytest"],
        cwd=repo,
        text=True,
        capture_output=True,
        timeout=120,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    payload = json.loads(report.read_text())
    assert payload["ok"] is True
    assert payload["mode"] == "workstation_dry_run"
    assert payload["startup_trace"]["replay_ok"] is True
    assert payload["startup_trace"]["frame_count"] > 30
    assert Path(payload["startup_trace"]["artifact_path"]).exists()
