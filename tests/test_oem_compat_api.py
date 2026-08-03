from fastapi.testclient import TestClient
from types import SimpleNamespace
from typing import Any

import pytest


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


def test_oem_compat_api_import_protocol_live_request_fails_closed_with_proof_ladder():
    from fastapi import FastAPI
    from src.bioxp.oem_compat.api import router

    app = FastAPI()
    app.include_router(router)
    client = TestClient(app)

    xml = """
    <WpfGenBotCommonLib>
      <experiment><data name="ApiLiveGate" /></experiment>
      <script>
        <line1 cmd="ST AMP PL_POOL Z5 V14 T50 NTY" />
      </script>
    </WpfGenBotCommonLib>
    """

    resp = client.post(
        "/oem-compat/protocols/import/dry-run",
        json={"xml": xml, "source_name": "api-live.xml", "requested_mode": "live"},
    )

    assert resp.status_code == 200
    body = resp.json()
    assert body["ok"] is False
    assert body["job"]["mode"] == "live"
    assert body["job"]["artifact"]["fail_closed"] is True
    assert body["job"]["physical_motion"] is False
    assert "operator_ack required for live mode" in body["job"]["preflight_errors"]
    assert "live transport validation not complete" in body["job"]["proof_ladder"]["blockers"]


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
            return {"ok": True, "release_interface_ok": True, "dispose_resources_ok": True}

    fake = FakeTester()
    monkeypatch.setattr(api, "_tester", fake)
    monkeypatch.setattr(api, "_tester_quarantine", None)
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
    monkeypatch.setattr(api, "_tester_quarantine", None)
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


def test_remote_reconnect_bootstraps_intentionally_unbound_runtime(monkeypatch):
    import src.bioxp.api as api

    created = []

    class FakeTester:
        def __init__(self, *, alt):
            self.alt = alt
            created.append(alt)

    monkeypatch.setattr(api, "BioXpTester", FakeTester)
    monkeypatch.setattr(api, "build_default_pipette_transport", lambda *, shared_usb: {"tester": shared_usb})
    monkeypatch.setattr(api, "_tester", None)
    monkeypatch.setattr(api, "_tester_quarantine", None)
    monkeypatch.setattr(api, "_pipette_transport", None)
    monkeypatch.setattr(api, "_startup_error", "USB transport is intentionally unbound")
    monkeypatch.setattr(api, "_generic_lifespan_claim_pending", True, raising=False)
    monkeypatch.setattr(
        api,
        "_status_payload",
        lambda: {
            "available": False,
            "ownership": {"transport": "owned", "usb": "service", "router": "running"},
        },
    )
    client = TestClient(api.app)

    resp = client.post("/reconnect")

    assert resp.status_code == 200
    body = resp.json()
    assert body["ok"] is True
    assert body["ownership"] == {"transport": "owned", "usb": "service", "router": "running"}
    assert body["maintenance_state"]["motion_blocked"] is True
    assert body["maintenance_state"]["recovery_required"] is True
    assert created == [1]
    assert isinstance(api._tester, FakeTester)
    assert api._startup_error is None


@pytest.mark.parametrize(
    "startup_error",
    [
        "OEM machine/runtime-state authority unavailable: bad lock",
        "BioXP USB runtime manually released for direct maintenance testing.",
    ],
)
def test_remote_reconnect_does_not_claim_non_generic_absent_owner(monkeypatch, startup_error):
    import src.bioxp.api as api

    created = []

    class FakeTester:
        def __init__(self, *, alt):
            created.append(alt)

    monkeypatch.setattr(api, "BioXpTester", FakeTester)
    monkeypatch.setattr(api, "_tester", None)
    monkeypatch.setattr(api, "_tester_quarantine", None)
    monkeypatch.setattr(api, "_pipette_transport", None)
    monkeypatch.setattr(api, "_startup_error", startup_error)
    monkeypatch.setattr(api, "_generic_lifespan_claim_pending", False, raising=False)

    response = TestClient(api.app).post("/reconnect")

    assert response.status_code == 503
    assert created == []
    assert api._tester is None
    assert api._startup_error == startup_error


def test_remote_reconnect_refuses_quarantined_owner_before_constructor(monkeypatch):
    import src.bioxp.api as api

    created = []

    class FakeTester:
        def __init__(self, *, alt):
            created.append(alt)

    quarantined = object()
    monkeypatch.setattr(api, "BioXpTester", FakeTester)
    monkeypatch.setattr(api, "_tester", None)
    monkeypatch.setattr(api, "_tester_quarantine", quarantined)
    monkeypatch.setattr(api, "_startup_error", "quarantined partial owner")
    monkeypatch.setattr(api, "_generic_lifespan_claim_pending", True, raising=False)

    response = TestClient(api.app).post("/reconnect")

    assert response.status_code == 503
    assert created == []
    assert api._tester is None
    assert api._tester_quarantine is quarantined


def test_remote_reconnect_quarantines_incompletely_cleaned_constructor_owner(monkeypatch):
    import src.bioxp.api as api

    class PartialOwner:
        def _disconnect(self):
            return {
                "ok": False,
                "release_interface_ok": False,
                "dispose_resources_ok": False,
            }

    partial_owner = PartialOwner()

    class ConstructionFailure(RuntimeError):
        def __init__(self):
            super().__init__("endpoint discovery failed after USB claim")
            self.partial_owner = partial_owner

    class FailingTester:
        def __init__(self, *, alt):
            raise ConstructionFailure()

    monkeypatch.setattr(api, "BioXpTester", FailingTester)
    monkeypatch.setattr(api, "_tester", None)
    monkeypatch.setattr(api, "_tester_quarantine", None)
    monkeypatch.setattr(api, "_pipette_transport", None)
    monkeypatch.setattr(api, "_startup_error", "USB transport is intentionally unbound")
    monkeypatch.setattr(api, "_generic_lifespan_claim_pending", True)

    response = TestClient(api.app).post("/reconnect")

    assert response.status_code == 503
    assert api._tester is None
    assert api._tester_quarantine is partial_owner
    assert api._generic_lifespan_claim_pending is True
    assert api.hardware_state.ownership_projection()["ownership"]["transport"] == "quarantined"


def test_lifespan_shutdown_drains_cancelled_reconnect_before_publishing_stopped(monkeypatch):
    import asyncio
    import threading
    from types import SimpleNamespace

    import src.bioxp.api as api

    constructor_entered = threading.Event()
    allow_constructor_exit = threading.Event()
    disconnected = []

    class LateTester:
        def __init__(self, *, alt):
            constructor_entered.set()
            assert allow_constructor_exit.wait(timeout=5)

        def _disconnect(self):
            disconnected.append("late-owner")
            return {
                "ok": True,
                "release_interface_ok": True,
                "dispose_resources_ok": True,
            }

    monkeypatch.setattr(
        api,
        "configure_oem_machine_snapshot_from_env",
        lambda **kwargs: SimpleNamespace(startup_mode="normal", operation_parameters={"CheckCamera": False}),
    )
    monkeypatch.setattr(api, "configure_oem_runtime_state_from_env", lambda snapshot: None)
    monkeypatch.setattr(api, "BioXpTester", LateTester)
    monkeypatch.setattr(api, "build_default_pipette_transport", lambda **kwargs: SimpleNamespace(close=lambda: None))
    monkeypatch.setattr(api, "shutdown_oem_runtime", lambda: None)

    async def no_camera(**kwargs):
        return None

    monkeypatch.setattr(api, "_stop_owned_camera_session", no_camera)
    monkeypatch.setattr(api, "_tester", None)
    monkeypatch.setattr(api, "_tester_quarantine", None)
    monkeypatch.setattr(api, "_pipette_transport", None)

    async def scenario():
        context = api.lifespan(api.app)
        await context.__aenter__()
        reconnect = asyncio.create_task(api.reconnect_runtime())
        assert await asyncio.to_thread(constructor_entered.wait, 2)
        reconnect.cancel()
        with pytest.raises(asyncio.CancelledError):
            await reconnect

        shutdown = asyncio.create_task(context.__aexit__(None, None, None))
        await asyncio.sleep(0.05)
        shutdown_finished_before_constructor = shutdown.done()
        allow_constructor_exit.set()
        await shutdown
        await asyncio.sleep(0.05)

        assert not shutdown_finished_before_constructor
        assert api._tester is None
        assert disconnected == ["late-owner"]
        ownership = api.hardware_state.ownership_projection()["ownership"]
        assert ownership == {
            **ownership,
            "transport": "unbound",
            "usb": "unbound",
            "router": "stopped",
        }

    asyncio.run(scenario())


def test_existing_owner_reconnect_failure_quarantines_and_blocks_motion(monkeypatch):
    import asyncio

    from fastapi import HTTPException
    import src.bioxp.api as api

    class UncertainOwner:
        def reconnect(self):
            raise RuntimeError("rebind failed after releasing old interface")

    owner = UncertainOwner()
    monkeypatch.setattr(api, "_tester", owner)
    monkeypatch.setattr(api, "_tester_quarantine", None)
    monkeypatch.setattr(api, "_pipette_transport", SimpleNamespace(close=lambda: None))
    monkeypatch.setattr(api, "_generic_lifespan_claim_pending", False)
    api._ownership_changed(reason="test_owner", transport="owned", usb="service", router="running")

    with pytest.raises(HTTPException) as captured:
        asyncio.run(api.reconnect_runtime())

    assert captured.value.status_code == 503
    assert api._tester is None
    assert api._tester_quarantine is owner
    assert api._pipette_transport is None
    assert api.hardware_state.ownership_projection()["ownership"]["transport"] == "quarantined"
    maintenance = api._maintenance_state_payload()
    assert maintenance["motion_blocked"] is True
    assert maintenance["recovery_required"] is True


@pytest.mark.parametrize("disconnect_failure", ["incomplete_report", "exception"])
def test_lifespan_shutdown_quarantines_incomplete_disconnect(monkeypatch, disconnect_failure):
    import asyncio

    import src.bioxp.api as api

    class IncompletelyReleasedOwner:
        def _disconnect(self):
            if disconnect_failure == "exception":
                raise RuntimeError("disconnect crashed after partial cleanup")
            return {
                "ok": False,
                "release_interface_ok": False,
                "dispose_resources_ok": True,
            }

    owner = IncompletelyReleasedOwner()
    monkeypatch.setattr(
        api,
        "configure_oem_machine_snapshot_from_env",
        lambda **kwargs: SimpleNamespace(startup_mode="normal", operation_parameters={"CheckCamera": False}),
    )
    monkeypatch.setattr(api, "configure_oem_runtime_state_from_env", lambda snapshot: None)
    monkeypatch.setattr(api, "shutdown_oem_runtime", lambda: None)
    monkeypatch.setattr(api, "_tester", None)
    monkeypatch.setattr(api, "_tester_quarantine", None)
    monkeypatch.setattr(api, "_pipette_transport", None)

    async def no_camera(**kwargs):
        return None

    monkeypatch.setattr(api, "_stop_owned_camera_session", no_camera)

    async def scenario():
        context = api.lifespan(api.app)
        await context.__aenter__()
        api._tester = owner
        api._ownership_changed(reason="test_owner", transport="owned", usb="service", router="running")
        await context.__aexit__(None, None, None)

    asyncio.run(scenario())

    assert api._tester is None
    assert api._tester_quarantine is owner
    assert api.hardware_state.ownership_projection()["ownership"]["transport"] == "quarantined"
    assert api._maintenance_state_payload()["motion_blocked"] is True


def test_lifespan_oem_shutdown_error_cannot_skip_usb_owner_cleanup(monkeypatch):
    import asyncio

    import src.bioxp.api as api

    disconnected = []

    class Owner:
        def _disconnect(self):
            disconnected.append(True)
            return {
                "ok": True,
                "release_interface_ok": True,
                "dispose_resources_ok": True,
            }

    owner = Owner()
    monkeypatch.setattr(
        api,
        "configure_oem_machine_snapshot_from_env",
        lambda **kwargs: SimpleNamespace(startup_mode="normal", operation_parameters={"CheckCamera": False}),
    )
    monkeypatch.setattr(api, "configure_oem_runtime_state_from_env", lambda snapshot: None)
    monkeypatch.setattr(api, "shutdown_oem_runtime", lambda: (_ for _ in ()).throw(RuntimeError("OEM shutdown failed")))
    monkeypatch.setattr(api, "_tester", None)
    monkeypatch.setattr(api, "_tester_quarantine", None)
    monkeypatch.setattr(api, "_pipette_transport", None)

    async def no_camera(**kwargs):
        return None

    monkeypatch.setattr(api, "_stop_owned_camera_session", no_camera)

    async def scenario():
        context = api.lifespan(api.app)
        await context.__aenter__()
        api._tester = owner
        api._ownership_changed(reason="test_owner", transport="owned", usb="service", router="running")
        await context.__aexit__(None, None, None)

    asyncio.run(scenario())

    assert disconnected == [True]
    assert api._tester is None
    assert api._tester_quarantine is None
    assert api.hardware_state.ownership_projection()["ownership"]["transport"] == "unbound"


@pytest.mark.parametrize("teardown_path", ["lifespan", "maintenance"])
def test_pipette_close_error_cannot_skip_authoritative_usb_disconnect(monkeypatch, teardown_path):
    import asyncio

    import src.bioxp.api as api

    disconnected = []

    class Owner:
        def _disconnect(self):
            disconnected.append(True)
            return {"ok": True, "release_interface_ok": True, "dispose_resources_ok": True}

    class FailingPipette:
        def close(self):
            raise RuntimeError("pipette close failed")

    owner = Owner()
    monkeypatch.setattr(api, "_tester", None)
    monkeypatch.setattr(api, "_tester_quarantine", None)
    monkeypatch.setattr(api, "_pipette_transport", None)

    if teardown_path == "lifespan":
        monkeypatch.setattr(
            api,
            "configure_oem_machine_snapshot_from_env",
            lambda **kwargs: SimpleNamespace(startup_mode="normal", operation_parameters={"CheckCamera": False}),
        )
        monkeypatch.setattr(api, "configure_oem_runtime_state_from_env", lambda snapshot: None)
        monkeypatch.setattr(api, "shutdown_oem_runtime", lambda: None)

        async def no_camera(**kwargs):
            return None

        monkeypatch.setattr(api, "_stop_owned_camera_session", no_camera)

        async def scenario():
            context = api.lifespan(api.app)
            await context.__aenter__()
            api._tester = owner
            api._pipette_transport = FailingPipette()
            api._ownership_changed(reason="test_owner", transport="owned", usb="service", router="running")
            await context.__aexit__(None, None, None)

        asyncio.run(scenario())
    else:
        api._tester = owner
        api._pipette_transport = FailingPipette()
        api._ownership_changed(reason="test_owner", transport="owned", usb="service", router="running")
        request: Any = SimpleNamespace(client=SimpleNamespace(host="127.0.0.1"))
        result = asyncio.run(api.maintenance_usb_release(request))
        warnings = result["maintenance_state"].get("cleanup_warnings", ())
        assert any("pipette transport close" in warning for warning in warnings)

    assert disconnected == [True]
    assert api._tester is None
    assert api._tester_quarantine is None
    assert api.hardware_state.ownership_projection()["ownership"]["transport"] == "unbound"


def test_lifespan_configures_oem_runtime_lazily_without_terminal_snapshot_hook(monkeypatch, tmp_path):
    import asyncio
    from types import SimpleNamespace

    import src.bioxp.api as api

    configured = []
    shutdown = []
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    monkeypatch.setattr(
        api,
        "configure_oem_machine_snapshot_from_env",
        lambda **kwargs: SimpleNamespace(startup_mode="normal", operation_parameters={"CheckCamera": False}),
    )
    monkeypatch.setattr(api, "configure_oem_runtime_state_from_env", lambda snapshot: None)
    monkeypatch.setattr(api, "configure_oem_runtime", lambda **kwargs: configured.append(kwargs))
    monkeypatch.setattr(api, "shutdown_oem_runtime", lambda: shutdown.append(True))
    monkeypatch.setattr(api, "_tester", None)
    monkeypatch.setattr(api, "_tester_quarantine", None)
    monkeypatch.setattr(api, "_pipette_transport", None)

    async def no_camera(**kwargs):
        return None

    monkeypatch.setattr(api, "_stop_owned_camera_session", no_camera)

    async def scenario():
        async with api.lifespan(api.app):
            assert api._tester is None

    asyncio.run(scenario())

    assert configured == [
        {
            "store_root": str(tmp_path),
            "autostart": True,
        }
    ]
    assert "terminal_snapshot_hook" not in configured[0]
    assert shutdown == [True]
