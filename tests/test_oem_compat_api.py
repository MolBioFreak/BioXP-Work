from fastapi.testclient import TestClient
from types import SimpleNamespace


def test_oem_compat_api_startup_dry_run_exposes_trace_without_physical_motion():
    from fastapi import FastAPI
    from src.bioxp.oem_compat.api import router

    app = FastAPI()
    app.include_router(router)
    client = TestClient(app)

    resp = client.post("/oem-compat/startup/dry-run", json={"run_homing": True})

    assert resp.status_code == 200
    body = resp.json()
    assert body["ok"] is True
    assert body["mode"] == "dry_run"
    assert body["physical_motion"] is False
    assert body["trace_names"] == ["initialize_motors_without_motion", "startup_homing"]
    assert body["frame_count"] > 30
    assert body["opened_usb"] is False


def test_oem_compat_api_startup_dry_run_fails_closed_on_safety_violation(monkeypatch):
    from fastapi import FastAPI
    import src.bioxp.oem_compat.api as oem_api

    class UnsafeControl:
        transport = SimpleNamespace(frames=[], opened_usb=False)

        def startup(self, *, run_homing: bool):
            return SimpleNamespace(
                ok=True,
                mode="dry_run",
                physical_motion=True,
                trace_names=[],
                traces=[],
            )

    monkeypatch.setattr(oem_api.BioXPControlLib, "dry_run", classmethod(lambda cls: UnsafeControl()))
    app = FastAPI()
    app.include_router(oem_api.router)
    client = TestClient(app)

    resp = client.post("/oem-compat/startup/dry-run", json={"run_homing": False})

    assert resp.status_code == 500
    assert "safety contract" in resp.json()["detail"]


def test_oem_compat_api_script_translate_dry_run_accepts_oem_xml():
    from fastapi import FastAPI
    from src.bioxp.oem_compat.api import router

    app = FastAPI()
    app.include_router(router)
    client = TestClient(app)

    xml = """
    <WpfGenBotCommonLib>
      <script>
        <line1 cmd="LED 0 0 0" />
        <line2 cmd="WAIT 1" />
      </script>
    </WpfGenBotCommonLib>
    """

    resp = client.post("/oem-compat/scripts/translate/dry-run", json={"xml": xml})

    assert resp.status_code == 200
    body = resp.json()
    assert body["mode"] == "dry_run"
    assert body["executed"] is False
    assert [a["verb"] for a in body["actions"]] == ["LED", "WAIT"]
    assert all(a["status"] == "planned" for a in body["actions"])


def test_oem_compat_api_import_protocol_dry_run_applies_virtual_state_without_motion():
    from fastapi import FastAPI
    from src.bioxp.oem_compat.api import router

    app = FastAPI()
    app.include_router(router)
    client = TestClient(app)

    xml = """
    <WpfGenBotCommonLib>
      <experiment><data name="ApiDryRun" /></experiment>
      <script>
        <step step="1" />
        <line1 cmd="ST AMP PL_POOL Z5 V14 T50 NTY" />
        <line2 cmd="ZW PL_POOL Z5 PL_OUTPUT Z9 V120 W10 MR5 DM2 T200 AMP" />
      </script>
    </WpfGenBotCommonLib>
    """

    resp = client.post("/oem-compat/protocols/import/dry-run", json={"xml": xml, "source_name": "api-dry.xml"})

    assert resp.status_code == 200
    body = resp.json()
    assert body["ok"] is True
    assert body["coverage"]["unsupported_command_count"] == 0
    assert body["job"]["mode"] == "dry_run"
    assert body["job"]["executed"] is False
    assert body["job"]["physical_motion"] is False
    assert body["job"]["artifact"]["opened_usb"] is False
    assert body["job"]["virtual_state"]["materials"]["AMP"]["location_id"] == "PL_POOL"
    assert body["job"]["virtual_state"]["materials"]["AMP"]["wash_count"] == 10


def test_main_bioxp_api_includes_oem_compat_dry_run_router_without_touching_usb(monkeypatch):
    import src.bioxp.api as api

    client = TestClient(api.app)

    resp = client.post("/oem-compat/startup/dry-run", json={"run_homing": False})

    assert resp.status_code == 200
    body = resp.json()
    assert body["ok"] is True
    assert body["mode"] == "dry_run"
    assert body["physical_motion"] is False
    assert body["trace_names"] == ["initialize_motors_without_motion"]
    assert body["opened_usb"] is False


def test_maintenance_usb_release_is_localhost_only_and_disconnects_owned_runtime(monkeypatch):
    import src.bioxp.api as api

    class FakeTester:
        def __init__(self):
            self.disconnected = False

        def _disconnect(self):
            self.disconnected = True

    fake = FakeTester()
    monkeypatch.setattr(api, "_tester", fake)
    monkeypatch.setattr(api, "_startup_error", None)
    client = TestClient(api.app)

    resp = client.post("/maintenance/usb/release")

    assert resp.status_code == 200
    body = resp.json()
    assert body["ok"] is True
    assert body["mode"] == "maintenance"
    assert body["usb_owner"] == "released"
    assert fake.disconnected is True
    assert api._tester is None
    assert "manually released" in api._startup_error


def test_maintenance_usb_guard_rejects_non_local_clients():
    from types import SimpleNamespace
    import pytest
    from fastapi import HTTPException
    import src.bioxp.api as api

    request = SimpleNamespace(client=SimpleNamespace(host="10.0.0.20"))

    with pytest.raises(HTTPException) as excinfo:
        api._require_local_maintenance_client(request)
    assert excinfo.value.status_code == 403
    assert "localhost-only" in excinfo.value.detail


def test_maintenance_usb_reconnect_is_localhost_only_and_recreates_runtime(monkeypatch):
    import src.bioxp.api as api

    created = []

    class FakeTester:
        def __init__(self, *, alt):
            self.alt = alt
            created.append(alt)

    monkeypatch.setattr(api, "BioXpTester", FakeTester)
    monkeypatch.setattr(api, "_tester", None)
    monkeypatch.setattr(api, "_startup_error", "released")
    client = TestClient(api.app)

    resp = client.post("/maintenance/usb/reconnect")

    assert resp.status_code == 200
    body = resp.json()
    assert body["ok"] is True
    assert body["mode"] == "maintenance"
    assert body["usb_owner"] == "service"
    assert created == [1]
    assert isinstance(api._tester, FakeTester)
    assert api._startup_error is None
