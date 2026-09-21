"""Offline source semantics and OS-ioctl leaf tests; no hardware acceptance."""
import ctypes
import errno
import struct

import pytest

from bioxp.vision import oem_camera_led as smi

from bioxp.vision.oem_camera_led import (
    GPIO0_REGISTER, LED3_REGISTER, SMI_REGISTER_PROPERTY_SET,
    register_bank_prefix, updated_gpio0,
    windows_register_read_calls, windows_register_write_calls,
)


@pytest.mark.parametrize("led,mask", [(1, 1), (2, 128)])
@pytest.mark.parametrize("enabled", [True, False])
def test_all_gpio_bytes_preserve_every_unaddressed_bit(led, mask, enabled):
    for before in range(256):
        after = updated_gpio0(before, led, enabled)
        assert (after & ~mask) == (before & ~mask)
        assert bool(after & mask) is enabled


@pytest.mark.parametrize("led", [1, 2, 3])
@pytest.mark.parametrize("dsp_type", range(8))
def test_source_conditional_bank_prefix(led, dsp_type):
    assert register_bank_prefix(led, dsp_type) == (
        () if dsp_type == 0 else ((0xD160, 1 if led == 3 else 2), (0xD164, 1)))


def test_exact_native_windows_read_packet_not_usb_packet():
    calls = windows_register_read_calls(GPIO0_REGISTER)
    assert [(c.method, c.property_id) for c in calls] == [("Set", 2), ("Get", 1)]
    for call in calls:
        assert call.property_set == "1b593e4f-f836-4256-8bb3-1f11cd246b6a"
        assert call.buffer.hex() == "00" * 32 + "07020000" + "00" * 4


@pytest.mark.parametrize("enabled", [False, True])
def test_led3_exact_native_windows_write_packets(enabled):
    assert LED3_REGISTER == 0xD87D
    calls = windows_register_write_calls(LED3_REGISTER, int(enabled))
    assert [(c.method, c.property_id) for c in calls] == [("Set", 1), ("Set", 1)]
    assert calls[0].buffer == bytes(32) + bytes.fromhex("7dd80000") + bytes(4)
    assert calls[1].buffer == bytes(32) + struct.pack("<I", enabled) + bytes(4)
    assert all(c.property_set == SMI_REGISTER_PROPERTY_SET for c in calls)


@pytest.mark.parametrize("value,led,enabled", [(-1, 1, True), (256, 1, True),
    (None, 1, True), (True, 1, True), (0, 3, True), (0, 1, 1)])
def test_unknown_readback_or_invalid_input_is_not_synthetic_gpio(value, led, enabled):
    with pytest.raises(ValueError):
        updated_gpio0(value, led, enabled)


@pytest.mark.parametrize("value", [-1, 2**32, None, True])
def test_windows_packet_dword_validation(value):
    with pytest.raises(ValueError):
        windows_register_write_calls(GPIO0_REGISTER, value)


XU = bytes.fromhex("1a24060292423946d10ce34a87833133f9eaaa3b08010301ff00")


class IoctlScript:
    """Doubles only fcntl.ioctl; verifies real native ABI and pointer data."""
    def __init__(self, monkeypatch):
        self.pending = []
        self.seen = []
        self.held = False
        self.entries = 0
        monkeypatch.setattr(smi.fcntl, "ioctl", self.ioctl)
        self.led = smi.SmiUvcLed(71, self, XU)

    def __enter__(self):
        assert not self.held, "nested lock acquisition"
        self.held = True
        self.entries += 1

    def __exit__(self, *args):
        self.held = False

    def ioctl(self, fd, operation, header, mutate):
        assert self.held
        assert fd == 71 and mutate is True
        # Independent native layout decoder; not the production ctypes class.
        assert struct.calcsize("@BBBxHP") == len(header)
        unit, selector, query, size, pointer = struct.unpack("@BBBxHP", header)
        assert operation == 0xC0007521 | (len(header) << 16)
        assert (unit, selector) == (2, 1)
        assert pointer != 0
        actual = ctypes.string_at(pointer, size)
        expected_query, expected_data, response = self.pending.pop(0)
        assert (query, actual) == (expected_query, expected_data)
        self.seen.append((query, actual))
        if isinstance(response, Exception):
            raise response
        if response is not None:
            assert len(response) == size
            ctypes.memmove(pointer, response, size)
        return 0

    def probe(self, length=2, info=3):
        self.pending += [(0x85, b"\0\0", struct.pack("<H", length)),
                         (0x86, b"\0", bytes([info]))]
        return self.led.probe()

    def write(self, address, value):
        self.pending += [(1, struct.pack("<H", address), None),
                         (1, struct.pack("<H", value), None)]

    def read(self, address, value):
        self.pending += [(1, struct.pack("<H", address), None),
                         (0x81, b"\0\0", struct.pack("<H", value))]

    def discovery(self, first=0x2018, second=None):
        self.write(0xD160, 2)
        self.read(0x213, first)
        if first != 0x2018:
            self.read(0xD160, 2)
            self.read(0xD164, 9)
            self.write(0xD160, 2)
            self.write(0xD164, 1)
            self.read(0x213, second)
        return self.led.discover_dsp_type()


def test_real_ioctl_abi_probe_has_only_standard_gets(monkeypatch):
    script = IoctlScript(monkeypatch)
    assert script.seen == []  # constructor performs no query/open
    assert script.probe() == (2, 3)
    assert script.seen == [(0x85, b"\0\0"), (0x86, b"\0")]
    assert not script.pending and not script.held


@pytest.mark.parametrize("length,info", [(64, 3), (40, 3), (0, 3), (2, 0), (2, 1), (2, 2), (2, 7)])
def test_probe_refuses_unmatched_wire_size_or_capability(monkeypatch, length, info):
    script = IoctlScript(monkeypatch)
    with pytest.raises(RuntimeError, match="incompatible"):
        script.probe(length, info)
    with pytest.raises(RuntimeError):
        script.led.discover_dsp_type()
    assert len(script.seen) == 2 and not script.pending


@pytest.mark.parametrize("led", [1, 2, 3])
@pytest.mark.parametrize("enabled", [False, True])
@pytest.mark.parametrize("banked", [False, True])
def test_exact_os_ioctl_led_transactions(monkeypatch, led, enabled, banked):
    script = IoctlScript(monkeypatch)
    script.probe()
    assert script.discovery(0x9999 if banked else 0x2018, 0x3510) == (4 if banked else 0)
    before = script.entries
    if banked:
        script.write(0xD160, 1 if led == 3 else 2)
        script.write(0xD164, 1)
    if led == 3:
        script.write(0xD87D, int(enabled))
    else:
        # Native GPIO caller takes low byte, not the entire WORD.
        script.read(0x0207, 0xA55A)
        if banked:
            script.write(0xD160, 2)
            script.write(0xD164, 1)
        mask = 1 if led == 1 else 0x80
        script.write(0x0207, 0x5A | mask if enabled else 0x5A & ~mask)
    script.led.set_led(led, enabled)
    assert not script.pending and not script.held
    assert script.entries == before + 1
    assert all(len(data) == 2 for request, data in script.seen if request in (1, 0x81))


@pytest.mark.parametrize("chip,dsp", [(0x3052, 1), (0x3084, 2), (0x3100, 2), (0x3199, 2),
    (0x3200, 3), (0x3299, 3), (0x3300, 4), (0x3510, 4), (0x4000, 3),
    (0x4199, 3), (0x41A0, 4), (0x4FFF, 4), (0x3469, 6), (0x347F, 7)])
def test_dsp_classification_is_source_register_not_usb_revision(chip, dsp):
    assert smi.dsp_type_from_chip_id(chip) == dsp
    assert smi.dsp_type_from_chip_id(0x2018, initial=True) == 0


@pytest.mark.parametrize("chip", [4, 0x2018, 0x319A, 0x329A, 0x5000])
def test_unclassified_bank_id_is_not_defaulted(chip):
    with pytest.raises(ValueError):
        smi.dsp_type_from_chip_id(chip)


@pytest.mark.parametrize("offset,value", [(0, 25), (2, 5), (3, 0), (4, 0), (20, 0), (21, 8), (23, 0), (24, 0)])
def test_wrong_selected_xu_refused_before_any_ioctl(monkeypatch, offset, value):
    script = IoctlScript(monkeypatch)
    descriptor = bytearray(XU)
    descriptor[offset] = value
    with pytest.raises(ValueError):
        smi.SmiUvcLed(71, script, descriptor)
    assert not script.seen


@pytest.mark.parametrize("fail_at", range(12))
def test_each_banked_gpio_ioctl_failure_stops_without_retry(monkeypatch, fail_at):
    script = IoctlScript(monkeypatch)
    script.probe()
    script.discovery(0x9999, 0x3510)
    script.write(0xD160, 2)
    script.write(0xD164, 1)
    script.read(0x207, 0x80)
    script.write(0xD160, 2)
    script.write(0xD164, 1)
    script.write(0x207, 0x81)
    request, data, _ = script.pending[fail_at]
    script.pending[fail_at] = (request, data, OSError(errno.EIO, "leaf failure"))
    before = len(script.seen)
    with pytest.raises(OSError):
        script.led.set_led(1, True)
    assert len(script.seen) == before + fail_at + 1
    assert not script.held
    with pytest.raises(RuntimeError, match="DSP discovery"):
        script.led.set_led(1, True)
    assert len(script.seen) == before + fail_at + 1


def test_requires_discovery_and_does_not_infer_it_from_probe(monkeypatch):
    script = IoctlScript(monkeypatch)
    script.probe()
    with pytest.raises(RuntimeError, match="DSP discovery"):
        script.led.set_led(1, True)
    assert len(script.seen) == 2


def test_unknown_dsp_restores_only_source_saved_bank_values(monkeypatch):
    script = IoctlScript(monkeypatch)
    script.probe()
    script.write(0xD160, 2)
    script.read(0x213, 0)
    script.read(0xD160, 2)
    script.read(0xD164, 9)
    script.write(0xD160, 2)
    script.write(0xD164, 1)
    script.read(0x213, 0)
    script.write(0xD160, 2)
    script.write(0xD164, 9)
    with pytest.raises(ValueError, match="unclassified"):
        script.led.discover_dsp_type()
    assert not script.pending
    with pytest.raises(RuntimeError):
        script.led.set_led(3, True)


@pytest.mark.parametrize("chip", [0x3469, 0x347F])
def test_special_dsp_is_not_claimed_fully_initialized(monkeypatch, chip):
    script = IoctlScript(monkeypatch)
    script.probe()
    with pytest.raises(RuntimeError, match="extra A390"):
        script.discovery(0, chip)
    with pytest.raises(RuntimeError):
        script.led.set_led(3, True)
    assert not script.pending
