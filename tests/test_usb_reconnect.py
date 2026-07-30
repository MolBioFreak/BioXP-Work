from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from bioxp import usb_driver


class FakeDev:
    def __init__(self, label: str):
        self.label = label
        self.reset_calls = 0

    def reset(self):
        self.reset_calls += 1


class ConnectFailure(RuntimeError):
    pass


def test_constructor_endpoint_failure_releases_claimed_interface(monkeypatch):
    released = []
    disposed = []

    class PartialDev:
        label = "partial-constructor"

        def is_kernel_driver_active(self, interface):
            return False

        def set_configuration(self):
            return None

        def set_interface_altsetting(self, *, interface, alternate_setting):
            return None

        def get_active_configuration(self):
            return {(0, 1): object()}

    dev = PartialDev()
    monkeypatch.setattr(usb_driver.usb.core, "find", lambda **kwargs: dev)
    monkeypatch.setattr(usb_driver.usb.util, "claim_interface", lambda value, iface: None)
    monkeypatch.setattr(usb_driver.usb.util, "find_descriptor", lambda *args, **kwargs: None)
    monkeypatch.setattr(
        usb_driver.usb.util,
        "release_interface",
        lambda value, iface: released.append((value, iface)),
    )
    monkeypatch.setattr(usb_driver.usb.util, "dispose_resources", lambda value: disposed.append(value))

    with pytest.raises(ValueError, match="endpoints were not found"):
        usb_driver.BioXpTester(alt=1)

    assert released == [(dev, 0)]
    assert disposed == [dev]


def test_constructor_router_start_failure_shuts_down_and_releases_usb(monkeypatch):
    released = []
    disposed = []
    router_events = []

    class PartialDev:
        label = "partial-router"

        def is_kernel_driver_active(self, interface):
            return False

        def set_configuration(self):
            return None

        def set_interface_altsetting(self, *, interface, alternate_setting):
            return None

        def get_active_configuration(self):
            return {(0, 1): object()}

    class PartialRouter:
        def __init__(self, **kwargs):
            router_events.append("constructed")

        def start(self):
            router_events.append("start")
            raise RuntimeError("router startup failed")

        def shutdown(self):
            router_events.append("shutdown")

    dev = PartialDev()
    endpoints = iter([object(), object()])
    monkeypatch.setattr(usb_driver.usb.core, "find", lambda **kwargs: dev)
    monkeypatch.setattr(usb_driver.usb.util, "claim_interface", lambda value, iface: None)
    monkeypatch.setattr(usb_driver.usb.util, "find_descriptor", lambda *args, **kwargs: next(endpoints))
    monkeypatch.setattr(usb_driver.usb.util, "release_interface", lambda value, iface: released.append((value, iface)))
    monkeypatch.setattr(usb_driver.usb.util, "dispose_resources", lambda value: disposed.append(value))
    monkeypatch.setattr(usb_driver, "NovoRouter", PartialRouter)

    with pytest.raises(RuntimeError, match="router startup failed"):
        usb_driver.BioXpTester(alt=1)

    assert router_events == ["constructed", "start", "shutdown"]
    assert released == [(dev, 0)]
    assert disposed == [dev]


def test_constructor_incomplete_cleanup_exposes_partial_owner_for_quarantine(monkeypatch):
    owners = []

    def fail_after_claim(self):
        owners.append(self)
        self.dev = FakeDev("partial-owner")
        raise RuntimeError("constructor failed after claim")

    def incomplete_cleanup(self):
        return {
            "ok": False,
            "release_interface_ok": False,
            "dispose_resources_ok": True,
        }

    monkeypatch.setattr(usb_driver.BioXpTester, "_connect", fail_after_claim)
    monkeypatch.setattr(usb_driver.BioXpTester, "_disconnect", incomplete_cleanup)

    with pytest.raises(usb_driver.BioXpConstructionError) as captured:
        usb_driver.BioXpTester(alt=1)

    assert captured.value.partial_owner is owners[0]
    assert captured.value.cleanup_report["ok"] is False


def _make_tester(connect_outcomes):
    tester = usb_driver.BioXpTester.__new__(usb_driver.BioXpTester)
    tester.dev = FakeDev("initial")
    tester.ep_out = object()
    tester.ep_in = object()
    tester.alt = 1
    tester._chiller_noresp_streak = 7
    tester._chiller_last_tx_ts = 1.25
    tester._thermal_noresp_streak = 5
    tester._thermal_last_tx_ts = 2.5
    tester._motor_last_tx_ts = {4: 9.0, 5: 8.0, 6: 7.0}
    tester._motor_noresp_streak = {4: 3, 5: 2, 6: 1}
    connect_calls = []
    outcomes = list(connect_outcomes)

    def fake_connect():
        connect_calls.append("connect")
        outcome = outcomes.pop(0)
        tester.dev = FakeDev(f"connected-{len(connect_calls)}")
        if isinstance(outcome, Exception):
            raise outcome

    tester._connect = fake_connect
    return tester, connect_calls


@pytest.mark.parametrize("failed_step", ["release", "dispose"])
def test_disconnect_report_is_not_ok_when_any_usb_release_step_fails(monkeypatch, failed_step):
    tester, _ = _make_tester([])

    def release(dev, iface):
        if failed_step == "release":
            raise RuntimeError("release failed")

    def dispose(dev):
        if failed_step == "dispose":
            raise RuntimeError("dispose failed")

    monkeypatch.setattr(usb_driver.usb.util, "release_interface", release)
    monkeypatch.setattr(usb_driver.usb.util, "dispose_resources", dispose)

    report = tester._disconnect()

    assert report["ok"] is False
    assert report["release_interface_ok"] is (failed_step != "release")
    assert report["dispose_resources_ok"] is (failed_step != "dispose")


def test_failed_release_retains_device_handle_for_authoritative_cleanup_retry(monkeypatch):
    tester, _ = _make_tester([])
    original = tester.dev
    release_calls = []

    def release(dev, iface):
        release_calls.append((dev, iface))
        if len(release_calls) == 1:
            raise RuntimeError("transient release failure")

    monkeypatch.setattr(usb_driver.usb.util, "release_interface", release)
    monkeypatch.setattr(usb_driver.usb.util, "dispose_resources", lambda dev: None)

    first = tester._disconnect()
    assert first["ok"] is False
    assert tester.dev is original

    second = tester._disconnect()
    assert second["ok"] is True
    assert tester.dev is None
    assert release_calls == [(original, 0), (original, 0)]


def test_failed_router_shutdown_retains_router_handle_for_cleanup_retry(monkeypatch):
    tester, _ = _make_tester([])

    class RetryableRouter:
        def __init__(self):
            self.calls = 0

        def shutdown(self):
            self.calls += 1
            if self.calls == 1:
                raise RuntimeError("transient router shutdown failure")

    router = RetryableRouter()
    tester.novo_router = router
    monkeypatch.setattr(usb_driver.usb.util, "release_interface", lambda dev, iface: None)
    monkeypatch.setattr(usb_driver.usb.util, "dispose_resources", lambda dev: None)

    first = tester._disconnect()
    assert first["ok"] is False
    assert tester.novo_router is router

    second = tester._disconnect()
    assert second["ok"] is True
    assert tester.novo_router is None
    assert tester.dev is None
    assert router.calls == 2


def test_reconnect_prefers_soft_rebind_before_usb_reset(monkeypatch):
    released = []
    disposed = []
    slept = []

    monkeypatch.setattr(usb_driver.usb.util, "release_interface", lambda dev, iface: released.append((dev.label, iface)))
    monkeypatch.setattr(usb_driver.usb.util, "dispose_resources", lambda dev: disposed.append(dev.label))
    monkeypatch.setattr(usb_driver.time, "sleep", lambda seconds: slept.append(seconds))

    tester, connect_calls = _make_tester([None])
    initial_dev = tester.dev

    result = tester.reconnect()

    assert isinstance(result, dict)
    assert result["reset_provenance"]["schema_version"] == "bioxp.reset_provenance.v1"
    assert result["reset_provenance"]["subsystem"] == "usb_runtime"
    assert result["reset_provenance"]["hardware_usb_reset_performed"] is False
    assert result["reset_provenance"]["attempts"][0]["hard_reset"] is False
    assert connect_calls == ["connect"]
    assert initial_dev.reset_calls == 0
    assert released == [("initial", 0)]
    assert disposed == ["initial"]
    assert slept == []
    assert tester._chiller_noresp_streak == 0
    assert tester._chiller_last_tx_ts == 0.0
    assert tester._thermal_noresp_streak == 0
    assert tester._thermal_last_tx_ts == 0.0
    assert tester._motor_last_tx_ts == {4: 0.0, 5: 0.0, 6: 0.0}
    assert tester._motor_noresp_streak == {4: 0, 5: 0, 6: 0}


def test_reconnect_falls_back_to_hard_reset_after_soft_failure(monkeypatch):
    released = []
    disposed = []
    slept = []

    monkeypatch.setattr(usb_driver.usb.util, "release_interface", lambda dev, iface: released.append((dev.label, iface)))
    monkeypatch.setattr(usb_driver.usb.util, "dispose_resources", lambda dev: disposed.append(dev.label))
    monkeypatch.setattr(usb_driver.time, "sleep", lambda seconds: slept.append(seconds))

    tester, connect_calls = _make_tester([ConnectFailure("claim failed"), None])
    initial_dev = tester.dev

    result = tester.reconnect()

    assert result["reset_provenance"]["hardware_usb_reset_performed"] is True
    assert [attempt["hard_reset"] for attempt in result["reset_provenance"]["attempts"]] == [False, True]
    assert result["reset_provenance"]["attempts"][0]["ok"] is False
    assert "claim failed" in result["reset_provenance"]["attempts"][0]["error"]
    assert connect_calls == ["connect", "connect"]
    assert initial_dev.reset_calls == 0
    assert released == [("initial", 0), ("connected-1", 0)]
    assert disposed == ["initial", "connected-1"]
    assert slept == [0.12]
    assert tester.dev.label == "connected-2"
    assert tester._motor_noresp_streak == {4: 0, 5: 0, 6: 0}


def test_reconnect_hard_fallback_resets_original_device_when_soft_connect_fails_early(monkeypatch):
    released = []
    disposed = []
    slept = []

    monkeypatch.setattr(usb_driver.usb.util, "release_interface", lambda dev, iface: released.append((dev.label, iface)))
    monkeypatch.setattr(usb_driver.usb.util, "dispose_resources", lambda dev: disposed.append(dev.label))
    monkeypatch.setattr(usb_driver.time, "sleep", lambda seconds: slept.append(seconds))

    tester = usb_driver.BioXpTester.__new__(usb_driver.BioXpTester)
    tester.dev = FakeDev("initial")
    tester.ep_out = object()
    tester.ep_in = object()
    tester.alt = 1
    tester._chiller_noresp_streak = 7
    tester._chiller_last_tx_ts = 1.25
    tester._thermal_noresp_streak = 5
    tester._thermal_last_tx_ts = 2.5
    tester._motor_last_tx_ts = {4: 9.0, 5: 8.0, 6: 7.0}
    tester._motor_noresp_streak = {4: 3, 5: 2, 6: 1}
    connect_calls = []

    def fake_connect():
        connect_calls.append("connect")
        if len(connect_calls) == 1:
            raise ConnectFailure("device not found")
        tester.dev = FakeDev("connected-2")

    tester._connect = fake_connect
    initial_dev = tester.dev

    tester.reconnect()

    assert connect_calls == ["connect", "connect"]
    assert initial_dev.reset_calls == 1
    assert released == [("initial", 0), ("initial", 0)]
    assert disposed == ["initial", "initial"]
    assert slept == [0.12]
    assert tester.dev.label == "connected-2"
    assert tester._motor_noresp_streak == {4: 0, 5: 0, 6: 0}


def test_explicit_hard_reconnect_still_resets_usb(monkeypatch):
    released = []
    disposed = []
    slept = []

    monkeypatch.setattr(usb_driver.usb.util, "release_interface", lambda dev, iface: released.append((dev.label, iface)))
    monkeypatch.setattr(usb_driver.usb.util, "dispose_resources", lambda dev: disposed.append(dev.label))
    monkeypatch.setattr(usb_driver.time, "sleep", lambda seconds: slept.append(seconds))

    tester, connect_calls = _make_tester([None])
    initial_dev = tester.dev

    tester.reconnect(hard_reset=True)

    assert connect_calls == ["connect"]
    assert initial_dev.reset_calls == 1
    assert released == [("initial", 0)]
    assert disposed == ["initial"]
    assert slept == [0.12]
