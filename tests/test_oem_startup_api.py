from fastapi.testclient import TestClient


def test_oem_startup_api_dry_run_request_and_status(monkeypatch, tmp_path):
    import src.bioxp.api as api

    monkeypatch.setenv("BIOXP_OEM_STARTUP_ARTIFACT_BASE", str(tmp_path))
    api._oem_startup_program = None
    client = TestClient(api.app)

    resp = client.post("/oem/startup/request", json={"mode": "dry_run", "require_config": False})

    assert resp.status_code == 200
    body = resp.json()
    assert body["ok"] is True
    assert body["session_id"]
    assert body["startup_status_url"].endswith(body["session_id"])

    latest = client.get("/oem/startup/status/latest")
    assert latest.status_code == 200
    assert latest.json()["session_id"] == body["session_id"]


def test_oem_startup_api_live_requires_ack_and_artifact(monkeypatch, tmp_path):
    import src.bioxp.api as api

    monkeypatch.setenv("BIOXP_OEM_STARTUP_ARTIFACT_BASE", str(tmp_path))
    api._oem_startup_program = None
    client = TestClient(api.app)

    resp = client.post("/oem/startup/request", json={"mode": "live"})

    assert resp.status_code == 409
    assert "operator_ack" in resp.json()["detail"]


def test_oem_switch_audit_api_status_mode(monkeypatch, tmp_path):
    import src.bioxp.api as api
    from src.bioxp.oem_switch_audit import OfflineSwitchAuditFixture

    monkeypatch.setenv("BIOXP_OEM_STARTUP_ARTIFACT_BASE", str(tmp_path))
    monkeypatch.setattr(api, "_tester", object())
    monkeypatch.setattr(api, "_startup_error", None)
    monkeypatch.setattr(api, "_BioXpSwitchAuditHardware", lambda tester: OfflineSwitchAuditFixture())
    client = TestClient(api.app)

    resp = client.post("/oem/switch_audit", json={"mode": "status", "axes": ["z"]})

    assert resp.status_code == 200
    body = resp.json()
    assert body["ok"] is True
    assert body["axes"][0]["axis"] == "z"


def test_oem_motion_worker_api_run_next_and_status(monkeypatch, tmp_path):
    import src.bioxp.api as api

    from src.bioxp.oem_startup_program import OEMStartupProgram, DryRunStartupHardware

    monkeypatch.setenv("BIOXP_OEM_STARTUP_ARTIFACT_BASE", str(tmp_path))
    api._oem_startup_program = OEMStartupProgram(hardware=DryRunStartupHardware(door_closed=True, latch_closed=True), artifact_base=tmp_path)
    client = TestClient(api.app)

    req = client.post("/oem/startup/request", json={"mode": "dry_run", "require_config": False})
    assert req.status_code == 200

    status_before = client.get("/oem/motion_worker/status")
    assert status_before.status_code == 200
    assert status_before.json()["queue_depth"] == 1

    ran = client.post("/oem/motion_worker/run_next")
    assert ran.status_code == 200
    assert ran.json()["ok"] is True

    latest = client.get("/oem/startup/status/latest").json()
    assert latest["state"] == "diagnostic_complete"
    assert latest["ready"] is False


def test_oem_initial_check_live_requires_ack(monkeypatch, tmp_path):
    import src.bioxp.api as api

    monkeypatch.setenv("BIOXP_OEM_STARTUP_ARTIFACT_BASE", str(tmp_path))
    api._oem_startup_program = None
    client = TestClient(api.app)

    resp = client.post("/oem/initial_check", json={"mode": "live"})

    assert resp.status_code == 409
    assert "operator_ack" in resp.json()["detail"]
