from fastapi.testclient import TestClient
import pytest


@pytest.fixture(autouse=True)
def _reset_canonical_startup_lifecycle():
    from src.bioxp.lifecycle_state import lifecycle_state

    lifecycle_state.transport_changed(None, reason="startup_test_setup")
    yield
    lifecycle_state.transport_changed(None, reason="startup_test_teardown")


def _reset_startup_state(api, tmp_path, monkeypatch):
    from src.bioxp.lifecycle_state import CanonicalLifecycleOwner

    monkeypatch.setenv("BIOXP_OEM_STARTUP_ARTIFACT_BASE", str(tmp_path))
    monkeypatch.setattr(api, "lifecycle_state", CanonicalLifecycleOwner())
    api._oem_startup_program = None
    return TestClient(api.app)


def test_oem_startup_api_binds_canonical_session_without_queueing(monkeypatch, tmp_path):
    import src.bioxp.api as api

    client = _reset_startup_state(api, tmp_path, monkeypatch)
    response = client.post(
        "/oem/startup/request",
        json={"mode": "dry_run", "require_config": False},
    )

    assert response.status_code == 200
    body = response.json()
    assert body["ok"] is True
    assert body["session_id"] == "canonical"
    assert body["queued"] is False
    assert body["next_action"] == "POST /oem/startup/constructor_pipettes"
    assert body["startup_status_url"] == "/oem/startup/status/canonical"
    assert body["status"] == "not_run"
    assert body["state"] == "waiting"
    stages = body["lifecycle"]["startup"]["stages"]
    assert stages["constructor_pipette_stage"]["state"] == "not_run"
    assert stages["initialization_without_motion"]["state"] == "blocked"
    assert stages["initial_check"]["state"] == "blocked"

    latest = client.get("/oem/startup/status/latest")
    assert latest.status_code == 200
    latest_body = latest.json()
    assert latest_body["state"] == "not_run"
    assert latest_body["lifecycle"]["operation_state"] == "waiting"
    assert "session_id" not in latest_body

    canonical = client.get(body["startup_status_url"])
    assert canonical.status_code == 200
    assert canonical.json()["session_id"] == "canonical"
    assert canonical.json()["state"] == "not_run"

    missing = client.get("/oem/startup/status/not-canonical")
    assert missing.status_code == 404


def test_oem_startup_api_live_requires_ack_and_artifact_root(monkeypatch, tmp_path):
    import src.bioxp.api as api

    client = _reset_startup_state(api, tmp_path, monkeypatch)

    missing_ack = client.post(
        "/oem/startup/request",
        json={"mode": "live", "artifact_root": str(tmp_path / "live")},
    )
    assert missing_ack.status_code == 409
    assert "operator_ack" in missing_ack.json()["detail"]

    missing_artifact = client.post(
        "/oem/startup/request",
        json={"mode": "live", "operator_ack": "INITIALIZE"},
    )
    assert missing_artifact.status_code == 409
    assert "artifact_root" in missing_artifact.json()["detail"]


def test_oem_motion_worker_remains_empty_after_generic_startup(monkeypatch, tmp_path):
    import src.bioxp.api as api

    client = _reset_startup_state(api, tmp_path, monkeypatch)
    request = client.post(
        "/oem/startup/request",
        json={"mode": "dry_run", "require_config": False},
    )
    assert request.status_code == 200

    status = client.get("/oem/motion_worker/status")
    assert status.status_code == 200
    assert status.json()["queue_depth"] == 0

    run_next = client.post("/oem/motion_worker/run_next")
    assert run_next.status_code == 200
    assert run_next.json()["ok"] is False
    assert run_next.json()["result"] is None


def test_oem_door_event_only_records_canonical_state(monkeypatch, tmp_path):
    import src.bioxp.api as api

    client = _reset_startup_state(api, tmp_path, monkeypatch)
    client.post(
        "/oem/startup/request",
        json={"mode": "dry_run", "require_config": False},
    )

    response = client.post(
        "/oem/startup/door_event",
        json={"session_id": "canonical", "door_closed": True, "latch_closed": True},
    )
    assert response.status_code == 200
    body = response.json()
    assert body["door"]["door_closed"] is True
    assert body["door"]["latch_closed"] is True
    assert body["lifecycle"]["operation_state"] == "stopped"
    assert body["lifecycle"]["startup"]["state"] == "not_run"
    assert body["lifecycle"]["startup"]["stages"]["constructor_pipette_stage"]["state"] == "not_run"


def test_oem_initial_check_shadow_is_non_executing(monkeypatch, tmp_path):
    import src.bioxp.api as api

    client = _reset_startup_state(api, tmp_path, monkeypatch)
    response = client.post("/oem/initial_check", json={"mode": "shadow"})

    assert response.status_code == 200
    body = response.json()
    assert body["ok"] is True
    assert body["executed"] is False
    assert body["mode"] == "shadow"
    assert body["lifecycle"]["startup"]["state"] == "not_run"
    assert "cannot claim completion" in body["reason"]
    assert "physical_motion" not in body
    assert "hardware_reads" not in body
    assert "hardware_writes" not in body


def test_oem_initial_check_live_requires_ack(monkeypatch, tmp_path):
    import src.bioxp.api as api

    client = _reset_startup_state(api, tmp_path, monkeypatch)
    response = client.post("/oem/initial_check", json={"mode": "live"})

    assert response.status_code == 409
    assert "operator_ack INITIALIZE" in response.json()["detail"]


def test_status_advertises_only_current_bms_commissioning_capabilities(monkeypatch, tmp_path):
    import src.bioxp.api as api

    client = _reset_startup_state(api, tmp_path, monkeypatch)
    response = client.get("/status")

    assert response.status_code == 200
    assert response.json()["capabilities"] == [
        "collect_hardware_snapshot",
        "construct_pipettes",
        "initialize_without_motion",
        "run_initial_check",
        "recover_motion_non_homing",
        "run_oem_motor_stage",
        "collect_axis_diagnostics",
        "run_axis_diagnostic",
        "stop_axis_diagnostic",
    ]


def test_api_composes_oem_runtime_and_finite_commissioning_routes_once():
    import src.bioxp.api as api

    route_counts = {}
    for path, operations in api.app.openapi()["paths"].items():
        for method in operations:
            upper = method.upper()
            if upper in {"GET", "POST", "PUT", "PATCH", "DELETE", "OPTIONS", "HEAD"}:
                route_counts[(upper, path)] = route_counts.get((upper, path), 0) + 1

    expected = {
        ("GET", "/oem/runtime/movement-runs/contract"),
        ("POST", "/oem/runtime/movement-runs"),
        ("GET", "/oem/runtime/movement-runs/{run_id}"),
        ("GET", "/oem/runtime/movement-runs/{run_id}/ledger"),
        ("POST", "/oem/runtime/movement-runs/{run_id}/cancel"),
        ("GET", "/motion/diagnostics/status"),
        ("POST", "/motion/diagnostics/execute"),
        ("POST", "/motion/diagnostics/stop"),
        ("POST", "/motion/gripper/clear"),
        ("POST", "/motion/gripper/home"),
        ("POST", "/motion/oem/startup_step"),
    }
    assert {route for route in expected if route_counts.get(route) == 1} == expected
    assert ("POST", "/oem/runtime/activate_service") not in route_counts
    assert route_counts.get(("POST", "/oem/startup/initialize_environment")) == 1


def test_oem_startup_step_dispatches_semantic_gripper_commission(monkeypatch):
    import src.bioxp.api as api

    calls = []

    class FakeTester:
        def motion_arm_state(self):
            return {"armed": True}

        def motion_gate_live_snapshot(self):
            return {"ok": True}

        def activate_boards(self, *, expect_reply):
            assert expect_reply is True
            return {"ok": True}

        def motor_prepare_motion_interlock(self, *, force_lock):
            assert force_lock is True
            return {"ok": True}

    def commission(tester, **kwargs):
        calls.append((tester, kwargs))
        return {"ok": True, "physical_motion_commanded": False}

    monkeypatch.setattr(api, "_require_motion_not_blocked_by_maintenance", lambda: None)
    monkeypatch.setattr(api, "gripper_commission_home", commission)

    tester = FakeTester()
    result = api._execute_oem_startup_step(tester, "gripper-commission-home", 12.5)

    assert result["ok"] is True
    assert result["step"] == "gripper-commission-home"
    assert calls == [
        (
            tester,
            {
                "operator_ack": "GRIPPER_COMMISSION_HOME",
                "reason": "OEM startup_step gripper-commission-home",
                "timeout_s": 12.5,
            },
        )
    ]


def test_runtime_factory_upgrades_a_cached_dry_startup_provider(monkeypatch):
    from src.bioxp import api as api_module
    from src.bioxp.oem_startup_program import BioXpStartupHardware, DryRunStartupHardware

    monkeypatch.setattr(api_module, "_oem_startup_program", None)
    monkeypatch.setattr(api_module, "_tester", None)
    dry = api_module._get_oem_startup_program(dry_safe=True)
    assert isinstance(dry.hardware, DryRunStartupHardware)

    monkeypatch.setattr(api_module, "_tester", object())
    live = api_module._get_live_oem_startup_program()

    assert live is not dry
    assert isinstance(live.hardware, BioXpStartupHardware)


def test_oem_switch_audit_status_mode_is_read_only(monkeypatch, tmp_path):
    import src.bioxp.api as api
    from src.bioxp.oem_switch_audit import OfflineSwitchAuditFixture

    monkeypatch.setenv("BIOXP_OEM_STARTUP_ARTIFACT_BASE", str(tmp_path))
    monkeypatch.setattr(api, "_tester", object())
    monkeypatch.setattr(api, "_tester_quarantine", None)
    monkeypatch.setattr(api, "_startup_error", None)
    monkeypatch.setattr(api, "_BioXpSwitchAuditHardware", lambda tester: OfflineSwitchAuditFixture())
    client = TestClient(api.app)

    response = client.post("/oem/switch_audit", json={"mode": "status", "axes": ["z"]})

    assert response.status_code == 200
    assert response.json()["ok"] is True
    assert response.json()["axes"][0]["axis"] == "z"
