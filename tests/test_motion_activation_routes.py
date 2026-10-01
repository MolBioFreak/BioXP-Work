"""Route identity and real native recovery gates, not activation-wire proof."""
import asyncio
from types import SimpleNamespace

import pytest
from fastapi import HTTPException
from starlette.requests import Request
from tests.test_wake_setup_debloat import rig


@pytest.fixture
def routes(monkeypatch):
    from bioxp import api
    from bioxp.usb_driver import BioXpTester

    tester = BioXpTester.__new__(BioXpTester)
    tester._motion_arm = {}
    calls = []
    state = {"io": {1: 1, 2: 1, 3: 1}, "rail": {"no24v": False},
             "ack": {"status": 100}, "provider_ok": True, "published": True,
             "override": {}, "gate_index": 0, "bad_gate": None}
    def io_snapshot(board):
        state["gate_index"] += 1
        if state["bad_gate"] == state["gate_index"]:
            return {1: 0, 2: 1, 3: 1}
        return state["io"]
    monkeypatch.setattr(tester, "io_snapshot", io_snapshot)
    monkeypatch.setattr(tester, "motor_query_24v_sensor", lambda: state["rail"])
    monkeypatch.setattr(tester, "motion_latch_override_state", lambda: state["override"])
    def latch(lock, *, activate_first):
        assert activate_first is False
        calls.append("latch")
        return {"ack": {"status": 1} if state.get("bad_second_lock") and calls.count("latch") == 2 else state["ack"]}
    monkeypatch.setattr(tester, "latch_oem", latch)
    def forbidden(*args, **kwargs):
        pytest.fail("legacy reconnect/activation/current-pulse path invoked")
    for name in ("reconnect", "activate_boards", "motion_arm_strict_startup", "motor_prepare_motion_interlock"):
        monkeypatch.setattr(tester, name, forbidden)
    authority = object()
    monkeypatch.setattr(api, "Serial206MotionAuthority", SimpleNamespace(from_active_snapshot=lambda: authority))
    def prepare(actual, *, authority):
        assert actual is tester
        calls.append(("provider", authority))
        return {"ok": state["provider_ok"], "generation": 7, "failure": None if state["provider_ok"] else "provider_failure",
                "component_prepare_receipts": {}, "physical_motion_commanded": False}
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", SimpleNamespace(prepare_global_motion_without_motion=prepare))
    monkeypatch.setattr(api, "_tester", tester)
    monkeypatch.setattr(api, "_get_tester", lambda: tester)
    monkeypatch.setattr(api, "_maintenance_state", {"motion_blocked": True, "recovery_required": True})
    monkeypatch.setattr(api, "_maintenance_latch_generation", 11)
    monkeypatch.setattr(api, "_maintenance_state_payload", lambda: dict(api._maintenance_state))
    state["real_clear"] = api._clear_post_maintenance_motion_block
    state["real_run_blocking"] = api._run_blocking
    def clear(**kwargs):
        calls.append(("clear", kwargs["expected_latch_generation"]))
        assert kwargs["expected_latch_generation"] == api._maintenance_latch_generation
        api._maintenance_state.update(motion_blocked=False, recovery_required=False)
        return dict(api._maintenance_state)
    monkeypatch.setattr(api, "_clear_post_maintenance_motion_block", clear)
    def publish(**kwargs):
        calls.append("publish")
        assert kwargs["expected_ownership_epoch"] == 7
        return {"published": state["published"]}
    monkeypatch.setattr(api, "hardware_state", SimpleNamespace(publish_can_ready_from_preparation=publish))
    async def inline(label, operation, **kwargs):
        return operation()
    monkeypatch.setattr(api, "_run_blocking", inline)
    monkeypatch.setattr(api.time, "sleep", lambda seconds: None)
    return api, tester, calls, state, authority


def invoke(api, route, **changes):
    if route == "direct":
        return asyncio.run(api.motion_oem_prepare_without_motion())
    if route == "power":
        return asyncio.run(api.motion_power_enable())
    if route == "maintenance":
        data = {"operator_ack": api.MAINTENANCE_RECOVERY_ACK, "include_diag": False, **changes}
        return asyncio.run(api.maintenance_usb_recover_motion(api.MaintenanceRecoverMotionRequest(**data),
            Request({"type": "http", "client": ("127.0.0.1", 123)})))
    data = {"run_homing": False, "operator_ack": api.MOTION_RECOVERY_ACK, "operator_reason": "offline test", **changes}
    return asyncio.run(api.motion_arm_strict_startup(api.MotionArmStartupRequest(**data)))


@pytest.mark.parametrize("route", ["direct", "power", "maintenance", "strict"])
def test_each_supported_route_uses_shared_provider_once(routes, route):
    api, tester, calls, state, authority = routes
    response = invoke(api, route)
    assert response["ok"] is True
    assert calls.count(("provider", authority)) == 1
    assert calls.count(("clear", 11)) == 1
    assert calls.count("publish") == 1
    assert tester.motion_arm_state()["armed"] is True
    if route in {"strict", "maintenance"}:
        report = response.get("recovery", response)
        assert report["homing"] is None
        assert report["final_gate"]["ok"] is True
        assert calls.count("latch") == 2
        assert report["interlock"]["ops"] == []
    else:
        assert "Home Z" not in response["next_required_action"]


@pytest.mark.parametrize("route", ["maintenance", "strict"])
@pytest.mark.parametrize("failure", ["door", "solenoid", "latch", "missing_io", "rail", "lock_ack", "second_lock_ack", "post_gate", "final_gate", "provider", "publication"])
def test_recovery_preserves_native_physical_failures_and_latch(routes, route, failure):
    api, tester, calls, state, authority = routes
    if failure in {"door", "solenoid", "latch"}:
        state["io"] [{"door": 1, "solenoid": 2, "latch": 3}[failure]] = 0
    elif failure == "missing_io": state["io"] = {}
    elif failure == "rail": state["rail"] = {"no24v": True}
    elif failure == "lock_ack": state["ack"] = {"status": 1}
    elif failure == "second_lock_ack": state["bad_second_lock"] = True
    elif failure == "post_gate": state["bad_gate"] = 2
    elif failure == "final_gate": state["bad_gate"] = 5
    elif failure == "provider": state["provider_ok"] = False
    elif failure == "publication": state["published"] = False
    with pytest.raises(HTTPException) as exc:
        invoke(api, route)
    assert exc.value.status_code == 409
    assert exc.value.detail["error"] == "motion_recovery_failed_closed"
    assert api._maintenance_state["recovery_required"] is True
    assert not any(isinstance(c, tuple) and c[0] == "clear" for c in calls)
    assert tester.motion_arm_state()["armed"] is False
    if failure != "publication": assert "publish" not in calls


def test_prelock_gate_remains_diagnostic(routes):
    api, tester, calls, state, authority = routes
    state["bad_gate"] = 1
    result = invoke(api, "strict")
    assert result["ok"] is True
    assert result["checks"][0]["ok"] is False
    assert result["checks"][0]["critical"] is False


@pytest.mark.parametrize("route,changes", [("maintenance", {"operator_ack": "wrong"}),
    ("strict", {"operator_ack": "wrong"}), ("strict", {"operator_reason": " "}),
    ("strict", {"run_homing": True})])
def test_existing_authorization_before_provider(routes, route, changes):
    api, tester, calls, state, authority = routes
    with pytest.raises(HTTPException) as exc: invoke(api, route, **changes)
    assert exc.value.status_code == 409
    assert calls == []


@pytest.mark.parametrize("route", ["maintenance", "strict"])
def test_recovery_requires_existing_pending_latch(routes, route):
    api, tester, calls, state, authority = routes
    api._maintenance_state.update(motion_blocked=False, recovery_required=False)
    with pytest.raises(HTTPException) as exc: invoke(api, route)
    assert exc.value.detail["error"] == "motion_recovery_not_required"
    assert calls == []


def test_maintenance_local_client_only(routes):
    api, tester, calls, state, authority = routes
    with pytest.raises(HTTPException) as exc:
        asyncio.run(api.maintenance_usb_recover_motion(
            api.MaintenanceRecoverMotionRequest(operator_ack=api.MAINTENANCE_RECOVERY_ACK),
            Request({"type": "http", "client": ("192.0.2.1", 123)})))
    assert exc.value.status_code == 403
    assert calls == []


def test_meta_catalog_dispatches_to_exact_shared_routes(routes):
    from bioxp.operator_controls import _build_catalog
    api, *_ = routes
    actions, dispatch = _build_catalog(api.app)
    activation = next(row for row in actions if row["action_id"] == "meta.activate_motion")
    assert dispatch["meta.activate_motion"]["path"] == "/motion/oem/prepare_without_motion"
    assert dispatch["meta.recover_motion_non_homing"]["path"] == "/motion/arm/strict_startup"
    assert "cmd64=0" not in str(activation)
    assert "conditional activation" in activation["description"]
    assert "torque are not verified" in activation["description"]

@pytest.mark.parametrize("route", ["direct", "maintenance", "strict"])
def test_newer_pending_generation_is_not_cleared(routes, monkeypatch, route):
    api, tester, calls, state, authority = routes
    monkeypatch.setattr(api, "_clear_post_maintenance_motion_block", state["real_clear"])
    original = api._serial206_oem_initialization_provider.prepare_global_motion_without_motion
    def prepare(*args, **kwargs):
        api._maintenance_latch_generation += 1
        return original(*args, **kwargs)
    monkeypatch.setattr(api._serial206_oem_initialization_provider, "prepare_global_motion_without_motion", prepare)
    with pytest.raises(HTTPException) as exc: invoke(api, route)
    assert exc.value.detail["error"] == "motion_recovery_latch_changed"
    assert api._maintenance_state["recovery_required"] is True


@pytest.mark.parametrize("route", ["direct", "maintenance", "strict"])
def test_disconnect_does_not_replay_provider_or_clear_latch(routes, monkeypatch, route):
    import threading
    api, tester, calls, state, authority = routes
    entered, release = threading.Event(), threading.Event()
    original = api._serial206_oem_initialization_provider.prepare_global_motion_without_motion
    def prepare(*args, **kwargs):
        entered.set()
        assert release.wait(5)
        return original(*args, **kwargs)
    monkeypatch.setattr(api._serial206_oem_initialization_provider, "prepare_global_motion_without_motion", prepare)
    monkeypatch.setattr(api, "_run_blocking", state["real_run_blocking"])
    async def exercise():
        monkeypatch.setattr(api, "_tester_lock", asyncio.Lock())
        if route == "direct": coroutine = api.motion_oem_prepare_without_motion()
        elif route == "strict": coroutine = api.motion_arm_strict_startup(api.MotionArmStartupRequest(
            operator_ack=api.MOTION_RECOVERY_ACK, operator_reason="offline cancellation"))
        else: coroutine = api.maintenance_usb_recover_motion(api.MaintenanceRecoverMotionRequest(
            operator_ack=api.MAINTENANCE_RECOVERY_ACK, include_diag=False),
            Request({"type": "http", "client": ("127.0.0.1", 123)}))
        task = asyncio.create_task(coroutine)
        assert await asyncio.to_thread(entered.wait, 5)
        task.cancel()
        with pytest.raises(asyncio.CancelledError): await task
        release.set()
        workers = [t for t in asyncio.all_tasks() if t.get_name().startswith("bioxp-tester:")]
        await asyncio.gather(*workers)
    try: asyncio.run(exercise())
    finally: release.set()
    assert calls.count(("provider", authority)) == 1
    assert not any(isinstance(c, tuple) and c[0] == "clear" for c in calls)
    assert api._maintenance_state["recovery_required"] is True


@pytest.mark.parametrize("route", ["direct", "power", "maintenance", "strict"])
def test_routes_connected_to_real_provider_native_and_sqlite(rig, monkeypatch, route):
    # This integration proves real provider publication and route compatibility.
    # Exact non-cycling wire assertions belong to the integrated core repair.
    from bioxp import api
    driver, provider, frames, fault, root = rig
    driver._motor_last_tx_ts = {}
    driver._motion_arm = {}
    driver._motion_latch_override = {}
    wire = driver.send_tmcl
    def sensors(board, command, typ, motor, value, **kwargs):
        result = wire(board, command, typ, motor, value, **kwargs)
        if board == 5 and command == 15 and typ in (1, 2, 3):
            result["value"] = 1
        return result
    monkeypatch.setattr(driver, "send_tmcl", sensors)
    monkeypatch.setattr(driver, "send_tmcl_retry", sensors)
    monkeypatch.setattr(api, "_tester", driver)
    monkeypatch.setattr(api, "_get_tester", lambda: driver)
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", provider)
    monkeypatch.setattr(api, "_maintenance_state", {"motion_blocked": True, "recovery_required": True})
    monkeypatch.setattr(api, "_maintenance_latch_generation", 11)
    monkeypatch.setattr(api, "hardware_state", SimpleNamespace(
        publish_can_ready_from_preparation=lambda **kwargs: {"published": kwargs["expected_ownership_epoch"] == 7}))
    async def inline(label, operation, **kwargs): return operation()
    monkeypatch.setattr(api, "_run_blocking", inline)
    monkeypatch.setattr(api.time, "sleep", lambda seconds: None)
    try:
        result = invoke(api, route)
        report = result.get("recovery", result)
        assert report["ok"] is True
        assert report["board_lifecycle_generation"]
        assert "z" in report["component_prepare_receipts"]
        persisted = provider.state_store.read_oem_serial206_initialization_state()
        assert persisted["z_lifecycle"]["state"] == "prepared_unreferenced"
        assert persisted["x_lifecycle"]["state"] == "prepared_unreferenced"
        assert not any(command in (3, 4, 13) or (command == 5 and typ in (0, 1))
                       for board, command, typ, motor, value in frames)
        assert provider.state_store.read_oem_serial206_initialization_state()["preparation"]
        assert [row for row in frames if row[1] == 64] == [
            (board, 64, 0, 0, 1) for board in driver.BOARDS]
        assert [row for row in frames if row[0] == 4 and row[1] == 5 and row[3] == 1] == [
            (4, 5, 4, 1, 1791), (4, 5, 5, 1, 576),
            (4, 5, 6, 1, 31), (4, 5, 205, 1, 3)]
        first_generation = report["board_lifecycle_generation"]
        first_board = provider.state_store.board4_authority_projection()["board"]
        start = len(frames)
        api._maintenance_state.update(motion_blocked=True, recovery_required=True)
        repeated = invoke(api, route)
        warm = repeated.get("recovery", repeated)
        assert warm["ok"] is True
        assert warm["board_lifecycle_generation"] == first_generation
        assert provider.state_store.board4_authority_projection()["board"] == first_board
        assert not any(row[1] == 64 for row in frames[start:])
        assert not any(command in (3, 4, 13) or (command == 5 and typ in (0, 1))
                       for board, command, typ, motor, value in frames[start:])
    finally:
        for service in driver._oem_fan_services().values():
            service["running"] = False
            timer = service.get("timer")
            if timer is not None:
                timer.cancel()
                timer.join(2)
                assert not timer.is_alive()
