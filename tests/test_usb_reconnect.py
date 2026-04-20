from pathlib import Path
import sys

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


def test_reconnect_prefers_soft_rebind_before_usb_reset(monkeypatch):
    released = []
    disposed = []
    slept = []

    monkeypatch.setattr(usb_driver.usb.util, "release_interface", lambda dev, iface: released.append((dev.label, iface)))
    monkeypatch.setattr(usb_driver.usb.util, "dispose_resources", lambda dev: disposed.append(dev.label))
    monkeypatch.setattr(usb_driver.time, "sleep", lambda seconds: slept.append(seconds))

    tester, connect_calls = _make_tester([None])
    initial_dev = tester.dev

    tester.reconnect()

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

    tester.reconnect()

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
