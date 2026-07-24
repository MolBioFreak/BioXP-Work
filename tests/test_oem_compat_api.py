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
    monkeypatch.setattr(api, "_quarantined_tester", None)
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


def test_service_usb_activation_claims_ownership_without_snapshot_recovery_or_motion(monkeypatch):
    import src.bioxp.api as api

    created = []
    shared_transport = object()

    class FakeTester:
        def __init__(self, *, alt):
            self.alt = alt
            created.append(alt)

    monkeypatch.setattr(api, "BioXpTester", FakeTester)
    monkeypatch.setattr(api, "build_default_pipette_transport", lambda *, shared_usb: shared_transport)
    monkeypatch.setattr(api, "_tester", None)
    monkeypatch.setattr(api, "_quarantined_tester", None)
    monkeypatch.setattr(api, "_pipette_transport", None)
    monkeypatch.setattr(api, "_startup_error", "generic lifespan unbound")
    client = TestClient(api.app)

    response = client.post("/oem/runtime/activate_service")

    assert response.status_code == 200
    body = response.json()
    assert body["ok"] is True
    assert body["usb_owner"] == "service"
    assert body["snapshot_collected"] is False
    assert body["motion_recovered"] is False
    assert body["motion_commanded"] is False
    assert created == [1]
    assert isinstance(api._tester, FakeTester)
    assert api._pipette_transport is shared_transport
    assert api._startup_error is None
    first_epoch = api.hardware_state.ownership_epoch
    status = api._status_payload()
    assert status["runtime_available"] is True
    assert status["hardware_connected"] is None

    repeated = client.post("/oem/runtime/activate_service")

    assert repeated.status_code == 200
    assert repeated.json()["already_active"] is True
    assert api.hardware_state.ownership_epoch == first_epoch
    assert created == [1]


def test_service_usb_activation_disconnects_partial_owner_when_transport_build_fails(monkeypatch):
    import src.bioxp.api as api

    created = []
    cleanup_wait_timeouts = []
    original_wait_for = api.asyncio.wait_for

    async def track_wait_for(awaitable, *, timeout):
        if timeout == 20.0:
            cleanup_wait_timeouts.append(timeout)
        return await original_wait_for(awaitable, timeout=timeout)

    class FakeTester:
        def __init__(self, *, alt):
            self.alt = alt
            self.disconnected = False
            created.append(self)

        def _disconnect(self):
            self.disconnected = True

    def fail_transport(*, shared_usb):
        assert shared_usb is created[0]
        raise RuntimeError("pipette transport failed")

    monkeypatch.setattr(api, "BioXpTester", FakeTester)
    monkeypatch.setattr(api, "build_default_pipette_transport", fail_transport)
    monkeypatch.setattr(api.asyncio, "wait_for", track_wait_for)
    monkeypatch.setattr(api, "_tester", None)
    monkeypatch.setattr(api, "_quarantined_tester", None)
    monkeypatch.setattr(api, "_pipette_transport", None)
    monkeypatch.setattr(api, "_startup_error", "generic lifespan unbound")
    client = TestClient(api.app)

    response = client.post("/oem/runtime/activate_service")

    assert response.status_code == 503
    assert created[0].disconnected is True
    assert api._tester is None
    assert api._pipette_transport is None
    assert api._startup_error == "pipette transport failed"
    assert cleanup_wait_timeouts == []


def test_service_usb_activation_tracks_partial_owner_when_disconnect_fails(monkeypatch):
    import pytest
    import src.bioxp.api as api

    created = []

    class FakeTester:
        def __init__(self, *, alt):
            self.alt = alt
            created.append(self)

        def _disconnect(self):
            raise RuntimeError("USB release failed")

    def fail_transport(*, shared_usb):
        raise RuntimeError("pipette transport failed")

    monkeypatch.setattr(api, "BioXpTester", FakeTester)
    monkeypatch.setattr(api, "build_default_pipette_transport", fail_transport)
    monkeypatch.setattr(api, "_tester", None)
    monkeypatch.setattr(api, "_quarantined_tester", None)
    monkeypatch.setattr(api, "_pipette_transport", None)
    client = TestClient(api.app)

    response = client.post("/oem/runtime/activate_service")

    assert response.status_code == 503
    assert api._tester is None
    assert api._quarantined_tester is created[0]
    assert api._pipette_transport is None
    assert api._startup_error is not None
    assert "partial USB owner cleanup failed: USB release failed" in api._startup_error

    with pytest.raises(api.HTTPException, match="cleanup failed"):
        api._get_tester()

    repeated = client.post("/oem/runtime/activate_service")
    assert repeated.status_code == 503
    assert "cleanup failed" in repeated.json()["detail"]
    assert len(created) == 1


def test_service_usb_activation_rejects_inconsistent_partial_runtime(monkeypatch):
    import src.bioxp.api as api

    orphan_transport = object()
    monkeypatch.setattr(api, "_tester", None)
    monkeypatch.setattr(api, "_quarantined_tester", None)
    monkeypatch.setattr(api, "_pipette_transport", orphan_transport)
    monkeypatch.setattr(
        api,
        "BioXpTester",
        lambda **_kwargs: (_ for _ in ()).throw(AssertionError("must not construct over partial state")),
    )
    client = TestClient(api.app)

    response = client.post("/oem/runtime/activate_service")

    assert response.status_code == 503
    assert "internally inconsistent" in response.json()["detail"]


def test_bioxp_tester_constructor_disconnects_if_connect_fails_after_usb_claim(monkeypatch):
    import pytest
    from src.bioxp.usb_driver import BioXpTester

    disconnected = []

    def fail_connect(self):
        self.dev = object()
        raise RuntimeError("connect failed after claim")

    def record_disconnect(self):
        disconnected.append(self.dev)
        self.dev = None

    monkeypatch.setattr(BioXpTester, "_connect", fail_connect)
    monkeypatch.setattr(BioXpTester, "_disconnect", record_disconnect)

    with pytest.raises(RuntimeError, match="connect failed after claim"):
        BioXpTester(alt=1)

    assert len(disconnected) == 1


def test_bioxp_tester_constructor_exposes_quarantinable_owner_when_cleanup_fails(monkeypatch):
    import pytest
    import src.bioxp.usb_driver as usb_driver

    class Router:
        def shutdown(self):
            raise RuntimeError("router did not stop")

    device = object()
    released = []

    def fail_connect(self):
        self.dev = device
        self.ep_in = object()
        self.ep_out = object()
        self.novo_router = Router()
        raise RuntimeError("connect failed")

    monkeypatch.setattr(usb_driver.BioXpTester, "_connect", fail_connect)
    monkeypatch.setattr(usb_driver.usb.util, "release_interface", lambda dev, interface: released.append((dev, interface)), raising=False)
    monkeypatch.setattr(usb_driver.usb.util, "dispose_resources", lambda dev: released.append((dev, "disposed")), raising=False)

    with pytest.raises(usb_driver.BioXpConstructionCleanupError) as caught:
        usb_driver.BioXpTester(alt=1)

    owner = caught.value.partial_owner
    assert owner.dev is None
    assert owner.novo_router is None
    assert owner._construction_cleanup_failed is True
    assert released == [(device, 0), (device, "disposed")]


def test_service_usb_activation_cancellation_drains_constructor_and_disconnects_candidate(monkeypatch):
    import asyncio
    import threading
    import pytest
    import src.bioxp.api as api

    constructor_started = threading.Event()
    constructor_release = threading.Event()
    instances = []

    class SlowTester:
        def __init__(self, *, alt):
            self.alt = alt
            self.disconnected = False
            instances.append(self)
            constructor_started.set()
            constructor_release.wait(timeout=2.0)

        def _disconnect(self):
            self.disconnected = True

    async def scenario():
        monkeypatch.setattr(api, "BioXpTester", SlowTester)
        monkeypatch.setattr(api, "build_default_pipette_transport", lambda *, shared_usb: object())
        monkeypatch.setattr(api, "_tester", None)
        monkeypatch.setattr(api, "_quarantined_tester", None)
        monkeypatch.setattr(api, "_pipette_transport", None)
        task = asyncio.create_task(api.oem_runtime_activate_service())
        assert await asyncio.to_thread(constructor_started.wait, 1.0)

        task.cancel()
        await asyncio.sleep(0)
        assert api._tester_lock.locked() is True
        assert task.done() is False

        constructor_release.set()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert instances[0].disconnected is True
        assert api._tester is None
        assert api._pipette_transport is None

    asyncio.run(scenario())


def test_service_usb_activation_cancellation_drains_failure_cleanup(monkeypatch):
    import asyncio
    import threading
    import pytest
    import src.bioxp.api as api

    cleanup_started = threading.Event()
    cleanup_release = threading.Event()
    instances = []

    class FakeTester:
        def __init__(self, *, alt):
            self.alt = alt
            self.disconnected = False
            instances.append(self)

        def _disconnect(self):
            cleanup_started.set()
            cleanup_release.wait(timeout=2.0)
            self.disconnected = True

    def fail_transport(*, shared_usb):
        raise RuntimeError("transport build failed")

    async def scenario():
        monkeypatch.setattr(api, "BioXpTester", FakeTester)
        monkeypatch.setattr(api, "build_default_pipette_transport", fail_transport)
        monkeypatch.setattr(api, "_tester", None)
        monkeypatch.setattr(api, "_quarantined_tester", None)
        monkeypatch.setattr(api, "_pipette_transport", None)
        task = asyncio.create_task(api.oem_runtime_activate_service())
        assert await asyncio.to_thread(cleanup_started.wait, 1.0)

        task.cancel()
        await asyncio.sleep(0)
        assert api._tester_lock.locked() is True
        assert task.done() is False

        cleanup_release.set()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert instances[0].disconnected is True
        assert api._tester is None
        assert api._pipette_transport is None

    asyncio.run(scenario())
