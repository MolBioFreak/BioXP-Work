"""Offline real driver -> API binding -> command-store pose publication."""
import ast
import json
import os
from pathlib import Path
import socket
from types import SimpleNamespace

import pytest
import usb.core

from bioxp.usb_driver import BioXpTester
from bioxp.hardware_status import HardwareStateOwner
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.operator_command_plane import OperatorCommandStore


@pytest.fixture(autouse=True)
def no_hardware(monkeypatch):
    def blocked(*args, **kwargs):
        raise AssertionError("hardware/network forbidden in pose qualification")
    monkeypatch.setattr(socket.socket, "connect", blocked)
    monkeypatch.setattr(socket, "create_connection", blocked)
    monkeypatch.setattr(usb.core, "find", blocked)
    monkeypatch.setattr(BioXpTester, "_connect", blocked)
    monkeypatch.setattr(BioXpTester, "motor_get_axis_param", blocked)
    monkeypatch.setattr(BioXpTester, "_send_motor", blocked)
    monkeypatch.setattr(BioXpTester, "_send_thermal", blocked)
    monkeypatch.setattr(BioXpTester, "_send_chiller", blocked)


@pytest.fixture
def connected(tmp_path):
    # Execute the production binding function without importing API lifespan or
    # opening its unrelated process-global databases.
    tree = ast.parse((Path(__file__).parents[1] / "src/bioxp/api.py").read_text())
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                    and n.name == "_bind_axis_position_observer")
    hardware = HardwareStateOwner()
    hardware.change_ownership(reason="offline", transport="owned", usb="service", router="running")
    OEMRuntimeStore(tmp_path)
    store = OperatorCommandStore(tmp_path)
    driver = object.__new__(BioXpTester)
    namespace = dict(_tester=driver, hardware_state=hardware,
                     app=SimpleNamespace(state=SimpleNamespace(operator_command_plane=SimpleNamespace(store=store))))
    exec(compile(ast.Module(body=[function], type_ignores=[]), "api-pose-binding", "exec"), namespace)
    bind = namespace["_bind_axis_position_observer"]
    bind(epoch=hardware.ownership_epoch, owned=True)
    yield driver, store, hardware, bind, namespace
    store.stop()


def reply(driver, value, *, ack=None):
    calls = []
    def read(board, param, motor=0):
        calls.append((board, param, motor))
        return {"ack": {"status": 100} if ack is None else ack, "value": value}
    driver.motor_get_axis_param = read
    return calls


@pytest.mark.parametrize("board,motor,axis", [(5, 0, "x"), (4, 0, "y"), (4, 1, "z")])
def test_actual_reads_publish_immediately_without_extra_queries(connected, monkeypatch, board, motor, axis):
    driver, store, _, _, _ = connected
    calls = reply(driver, 12345)
    monkeypatch.setattr("bioxp.usb_driver.time.time", lambda: 100.25)
    result = driver.motor_get_position(board, motor=motor)
    assert result["position"] == 12345
    assert calls == [(board, 1, motor)]
    assert store._updates_snapshot(None, None)["pose"]["axes"] == [
        {"axis": axis, "position_steps": 12345, "observed_at": 100.25}]


def test_partial_progress_final_and_no_freshness_laundering(connected, monkeypatch):
    driver, store, hardware, _, _ = connected
    exports = []
    # Transport-replaced real observations, not selected targets or animation.
    for board, motor, value, timestamp in [(4, 0, 100, 10.0), (5, 0, 200, 11.0),
                                           (5, 0, 300, 12.0), (5, 0, 499, 13.0)]:
        reply(driver, value)
        monkeypatch.setattr("bioxp.usb_driver.time.time", lambda t=timestamp: t)
        driver.motor_get_position(board, motor=motor)
        exports.append(store._updates_snapshot(None, None))
    axes = {row["axis"]: row for row in exports[-1]["pose"]["axes"]}
    assert axes["x"] == dict(axis="x", position_steps=499, observed_at=13.0)
    assert axes["y"] == dict(axis="y", position_steps=100, observed_at=10.0)
    assert hardware.completed_snapshot() is None  # no full sweep or admission publication
    path = os.environ.get("BIOXP_POSE_EXPORT")
    if path:
        Path(path).write_text(json.dumps({"evidence_kind": "offline_transport_replaced_producer", "updates": exports}, indent=2))


@pytest.mark.parametrize("row", [{"ack": None, "value": None},
                                  {"ack": {"status": 2}, "value": 999},
                                  {"ack": {"status": 100}, "value": True}])
def test_failed_cached_or_malformed_reply_never_publishes(connected, row):
    driver, store, _, _, _ = connected
    reply(driver, 55)
    driver.motor_get_position(5)
    before = store._updates_snapshot(None, None)
    driver.motor_get_axis_param = lambda *args, **kwargs: row
    driver.motor_get_position(5)
    assert store._updates_snapshot(None, None) == before


def test_other_motor_not_pose_and_observer_failure_not_motion_failure(connected):
    driver, store, _, _, _ = connected
    reply(driver, 77)
    driver.motor_get_position(4, motor=2)
    assert store._updates_snapshot(None, None)["pose"] is None
    def fail(**kwargs):
        raise RuntimeError("display unavailable")
    driver.set_axis_position_observer(fail, ownership_generation=1)
    assert driver.motor_get_position(5)["ok"] is True


def test_reconnect_during_query_cannot_relabel_old_read(connected):
    driver, store, hardware, bind, namespace = connected
    reply(driver, 10)
    driver.motor_get_position(5)
    def reconnect(*args, **kwargs):
        epoch = hardware.change_ownership(reason="replaced")
        bind(epoch=epoch, owned=True)
        return {"ack": {"status": 100}, "value": 999}
    driver.motor_get_axis_param = reconnect
    assert driver.motor_get_position(5)["ok"] is True
    assert store._updates_snapshot(None, None)["pose"] is None
    reply(driver, 20)
    driver.motor_get_position(4)
    assert store._updates_snapshot(None, None)["pose"]["axes"][0]["axis"] == "y"
    namespace["_tester"] = None
    epoch = hardware.change_ownership(reason="released")
    bind(epoch=epoch, owned=False)
    driver.motor_get_position(5)
    assert store._updates_snapshot(None, None)["pose"] is None


@pytest.mark.parametrize("command,param,publishes", [(6, 1, True), (6, 3, False), (5, 1, False)])
def test_snapshot_query_read_uses_same_sink_without_extra_traffic(connected, command, param, publishes):
    driver, store, _, _, _ = connected
    calls = []
    def send(*args, **kwargs):
        calls.append((args, kwargs))
        return {"status": 100, "value": 876}
    driver._send_motor = send
    assert driver.query_only_tmcl(5, command, param)["value"] == 876
    assert len(calls) == 1
    assert calls[0][1]["allow_recover"] is False
    assert (store._updates_snapshot(None, None)["pose"] is not None) is publishes


def test_existing_stop_wait_publishes_final_read_not_speed_as_pose(connected, monkeypatch):
    driver, store, _, _, _ = connected
    calls = reply(driver, 42)
    speeds = iter((100, 0))
    def speed(*args, **kwargs):
        # No coordinates exist while the existing loop reads speed only.
        assert store._updates_snapshot(None, None)["pose"] is None
        return {"ack": {"status": 100}, "speed": next(speeds), "speed_reply_valid": True}
    driver.motor_get_speed = speed
    monkeypatch.setattr("bioxp.usb_driver.time.sleep", lambda seconds: None)
    result = driver.motor_wait_stopped(5, min_polls=1, target_position=42)
    assert result["stopped"] is True and result["last_position"] == 42
    assert calls == [(5, 1, 0)]
    assert store._updates_snapshot(None, None)["pose"]["axes"][0]["position_steps"] == 42


def test_query_binding_captures_generation_before_read():
    driver = object.__new__(BioXpTester)
    observations = []
    driver.set_axis_position_observer(lambda **row: observations.append(row), ownership_generation=1)
    def read(*args, **kwargs):
        driver.set_axis_position_observer(lambda **row: observations.append(row), ownership_generation=2)
        return {"ack": {"status": 100}, "value": 123}
    driver.motor_get_axis_param = read
    driver.motor_get_position(5)
    assert observations[0]["ownership_generation"] == 1
