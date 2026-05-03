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
