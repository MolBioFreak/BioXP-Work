import json

from src.bioxp import usb_driver
from src.bioxp.usb_driver import BioXpTester


class FakeUsbDevice:
    label = "fake-novo-usb"

    def __repr__(self):
        return "<FakeUsbDevice fake-novo-usb>"


def test_disconnect_default_result_is_json_serializable_for_artifacts():
    tester = object.__new__(BioXpTester)
    fake = FakeUsbDevice()
    tester.dev = fake
    tester.ep_out = object()
    tester.ep_in = object()

    result = tester._disconnect()

    json.dumps({"disconnect": result}, sort_keys=True)
    assert result["device"] == "fake-novo-usb"
    assert result["hard_reset_requested"] is False
    assert tester.dev is None
    assert tester.ep_out is None
    assert tester.ep_in is None


def test_disconnect_can_preserve_device_return_for_reconnect_recovery(monkeypatch):
    tester = object.__new__(BioXpTester)
    fake = FakeUsbDevice()
    tester.dev = fake
    tester.ep_out = object()
    tester.ep_in = object()
    monkeypatch.setattr(usb_driver.usb.util, "release_interface", lambda dev, iface: None)
    monkeypatch.setattr(usb_driver.usb.util, "dispose_resources", lambda dev: None)

    result = tester._disconnect(return_device=True)

    assert result is fake
    assert tester.dev is None
    assert tester.ep_out is None
    assert tester.ep_in is None
