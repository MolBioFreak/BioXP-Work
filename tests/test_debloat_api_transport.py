"""A5 optional capture absent; normal TMCL and router events survive."""
import contextlib
from types import SimpleNamespace
import pytest
import usb.core
from bioxp.usb_driver import BioXpTester


@pytest.mark.parametrize('status', [100, 2])
def test_tmcl_ack_and_controller_error_without_sniff(status):
    driver = BioXpTester.__new__(BioXpTester)
    driver._transport_guard = contextlib.nullcontext
    calls = []
    def transact(frame, **kwargs):
        calls.append((frame, kwargs))
        return {'ok': True, 'frames': [{'data': [4, status, 6, 0, 0, 0, 0, 42]}]}
    driver.novo_router = SimpleNamespace(transact=transact, tmcl_matcher=lambda **kwargs: kwargs)
    result = driver.send_tmcl(4, 6, 1, 0, 0)
    assert result['status'] == status
    assert len(calls) == 1
    assert calls[0][0] == bytes(driver._build_frame(4, 6, 1, 0, 0))
    assert not hasattr(driver, '_usb_sniff_ledger_path')


@pytest.mark.parametrize('error', [usb.core.USBTimeoutError('offline timeout'), usb.core.USBError('offline disconnect')])
def test_tmcl_transport_failure_without_sniff(error):
    driver = BioXpTester.__new__(BioXpTester)
    driver._transport_guard = contextlib.nullcontext
    def transact(*args, **kwargs):
        raise error
    driver.novo_router = SimpleNamespace(transact=transact, tmcl_matcher=lambda **kwargs: kwargs)
    assert driver.send_tmcl(4, 6, 1, 0, 0) is None
