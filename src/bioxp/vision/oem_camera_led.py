"""Recovered SMI illumination semantics and source-proven type0 Linux leaf.

Pinned SMIUtility.dll SHA256:
a02a4e4ff6942a9a9690343b83ea0edd6cd3be1e5d4c0f02196a6b6dda7c5052

The type1 Windows descriptions remain offline-only and are NOT USB payloads.
The Microsoft usbvideo service selects type0 (RVA6B64-6B76, 686B-6887), whose
IKsControl/KSP_NODE GUID matches the selected camera's actual XU descriptor.
RVA6EA0/70A0 uses control1 and a 64-byte DWORD-first Windows data buffer.
The selected camera's actual GET_LEN is2: only its little-endian WORD is wire
payload; Windows DataLength is capacity, not the USB control length.
Microsoft's extension-unit documentation proves the direct property/control
mapping. Linux uses UVCIOC_CTRL_QUERY, never the separate KSP_NODE header.

SmiUvcLed borrows the existing selected fd and lock; it NEVER opens/closes a
camera. Its owner must fence identity/generation across calls and supply the
selected descriptor. Construction does no I/O. probe() is GET-only; discovery
and register reads DO send source-required SET_CUR address/bank selections.
No Linux transport-error replay is attempted (Windows retries HRESULT writes,
but an ioctl error does not prove non-delivery). Live qualification is separate.
"""
from __future__ import annotations

import ctypes
import fcntl
import struct
from uuid import UUID
from dataclasses import dataclass
from typing import Literal

SMI_REGISTER_PROPERTY_SET = "1b593e4f-f836-4256-8bb3-1f11cd246b6a"
IKS_PROPERTY_SET_IID = "31efac30-515c-11d0-a9aa-00aa0061be93"
GPIO0_REGISTER = 0x0207
LED3_REGISTER = 55421


@dataclass(frozen=True)
class WindowsKsCall:
    """IKsPropertySet call, deliberately not a UVC transfer description.

    Native code passes the same 40-byte buffer as instance and property data.
    For a GET, the returned DWORD at offset 32 is the register value.
    """

    method: Literal["Set", "Get"]
    property_id: int
    buffer: bytes
    property_set: str = SMI_REGISTER_PROPERTY_SET


def _dword_buffer(value: int) -> bytes:
    if type(value) is not int or not 0 <= value <= 0xFFFFFFFF:
        raise ValueError("SMI value must be an unsigned DWORD")
    return bytes(32) + struct.pack("<I", value) + bytes(4)


def windows_register_read_calls(address: int) -> tuple[WindowsKsCall, ...]:
    """RVA 7610: Set(property 2, address), Get(property 1, same buffer)."""
    data = _dword_buffer(address)
    return (WindowsKsCall("Set", 2, data), WindowsKsCall("Get", 1, data))


def windows_register_write_calls(address: int, value: int) -> tuple[WindowsKsCall, ...]:
    """RVA 78B0: Set(property 1, address), Set(property 1, value).

    The second call is issued only if the first HRESULT succeeds. This pure
    description is not an executor and must not conceal failure propagation.
    """
    return (WindowsKsCall("Set", 1, _dword_buffer(address)),
            WindowsKsCall("Set", 1, _dword_buffer(value)))


def updated_gpio0(value: int, led: int, enabled: bool) -> int:
    """ClassLEDControl bit preservation on a genuinely read GPIO byte."""
    if type(value) is not int or not 0 <= value <= 255:
        raise ValueError("GPIO readback must be a byte")
    if type(led) is not int or led not in (1, 2):
        raise ValueError("only led1 and led2 are GPIO0 bits")
    if type(enabled) is not bool:
        raise ValueError("enabled must be boolean")
    mask = 1 if led == 1 else 0x80
    return value | mask if enabled else value & (0xFF ^ mask)


def register_bank_prefix(led: int, dsp_type: int) -> tuple[tuple[int, int], ...]:
    """Recovered bank writes preceding EACH GPIO Get/Set or LED3 SetValue.

    DSP type is native object+0xC0, not USB revision, VID, PID or XU unit ID.
    It must come from recovered discovery, never a default or guess.
    """
    if type(led) is not int or led not in (1, 2, 3):
        raise ValueError("LED must be 1, 2 or 3")
    if type(dsp_type) is not int or not 0 <= dsp_type <= 7:
        raise ValueError("a recovered OEM DSP type is required")
    if dsp_type == 0:
        return ()
    return ((0xD160, 1 if led == 3 else 2), (0xD164, 1))


SMI_XU_GUID = "46394292-0cd1-4ae3-8783-3133f9eaaa3b"
UVC_SET_CUR = 0x01
UVC_GET_CUR = 0x81
UVC_GET_LEN = 0x85
UVC_GET_INFO = 0x86


class _XuQuery(ctypes.Structure):
    _fields_ = [("unit", ctypes.c_uint8), ("selector", ctypes.c_uint8),
                ("query", ctypes.c_uint8), ("size", ctypes.c_uint16),
                ("data", ctypes.c_void_p)]


# Linux asm-generic _IOWR('u', 0x21, struct uvc_xu_control_query).
UVCIOC_CTRL_QUERY = 0xC0000000 | (ctypes.sizeof(_XuQuery) << 16) | (ord("u") << 8) | 0x21


def _uvc_query(fd: int, unit: int, request: int, payload: bytes) -> bytes:
    data = (ctypes.c_ubyte * len(payload)).from_buffer_copy(payload)
    query = _XuQuery(unit, 1, request, len(data), ctypes.addressof(data))
    header = bytearray(ctypes.string_at(ctypes.addressof(query), ctypes.sizeof(query)))
    fcntl.ioctl(fd, UVCIOC_CTRL_QUERY, header, True)
    return bytes(data)


def type0_register_payload(value: int) -> bytes:
    """Selected GET_LEN=2 control: low WORD of native DWORD-first buffer."""
    if type(value) is not int or not 0 <= value <= 0xFFFF:
        raise ValueError("selected SMI register control requires an unsigned WORD")
    return struct.pack("<H", value)


def dsp_type_from_chip_id(chip_id: int, *, initial: bool = False) -> int:
    """ACEC-AEAC branch ordering; ID is register0213, never USB revision.

    Unknown IDs are not assigned a fabricated constructor/default outcome.
    Special IDs6/7 are classified here, but their native extra initialization
    at A390 is outside this LED leaf and discover_dsp_type refuses them.
    """
    type0_register_payload(chip_id)
    if initial:
        if chip_id == 0x2018:
            return 0
        raise ValueError("non-2018 ID requires the banked second read")
    if chip_id == 0x3052:
        return 1
    if chip_id in (0x3469, 0x347F):
        return 6 if chip_id == 0x3469 else 7
    if chip_id == 0x3084 or 0x3100 <= chip_id <= 0x3199:
        return 2
    if 0x4000 <= chip_id <= 0x4199 or 0x3200 <= chip_id <= 0x3299:
        return 3
    if 0x3300 <= chip_id <= 0x4FFF:
        return 4
    raise ValueError(f"unclassified source DSP register0213: {chip_id:#x}")


class SmiUvcLed:
    """Borrowed selected-camera leaf. No parallel camera owner or device open.

    Pass the owner's fd, its existing context-manager lock, and raw selected
    CS_EXTENSION_UNIT descriptor. Keep this object within that fd generation.
    All public I/O methods acquire the supplied lock for the whole transaction;
    callers already holding a nonreentrant lock must not call these methods.
    """

    def __init__(self, selected_fd: int, lock, extension_descriptor: bytes):
        if type(selected_fd) is not int or selected_fd < 0:
            raise ValueError("an existing selected camera fd is required")
        d = bytes(extension_descriptor)
        if (len(d) < 24 or d[0] != len(d) or d[1:3] != b"\x24\x06"
                or d[4:20] != UUID(SMI_XU_GUID).bytes_le or not d[3]):
            raise ValueError("selected XU descriptor does not match source type0 GUID")
        control_size_at = 22 + d[21]
        if control_size_at >= len(d) - 1:
            raise ValueError("malformed XU descriptor")
        control_size = d[control_size_at]
        if (not control_size or control_size_at + control_size + 2 != len(d)
                or d[20] < 1 or not (d[control_size_at + 1] & 1)):
            raise ValueError("selected XU does not expose register control1")
        if not hasattr(lock, "__enter__") or not hasattr(lock, "__exit__"):
            raise ValueError("existing owner transaction lock is required")
        self._fd, self._lock, self._unit = selected_fd, lock, d[3]
        self._qualified = False
        self._dsp_type = None

    def _query(self, request: int, payload: bytes) -> bytes:
        try:
            return _uvc_query(self._fd, self._unit, request, payload)
        except OSError:
            self._qualified = False
            self._dsp_type = None
            raise

    def probe(self) -> tuple[int, int]:
        """GET_LEN/GET_INFO only; never performs SET_CUR or LED writes."""
        with self._lock:
            self._qualified = False
            self._dsp_type = None
            length = struct.unpack("<H", self._query(UVC_GET_LEN, bytes(2)))[0]
            info = self._query(UVC_GET_INFO, bytes(1))[0]
            if length != 2 or info & 3 != 3 or info & 4:
                raise RuntimeError(f"SMI control1 incompatible: length={length}, info={info:#x}")
            self._qualified = True
            return length, info

    def _require_probe(self):
        if not self._qualified:
            raise RuntimeError("matching control1 GET_LEN/GET_INFO required")

    def _read(self, address: int) -> int:
        self._require_probe()
        self._query(UVC_SET_CUR, type0_register_payload(address))
        data = self._query(UVC_GET_CUR, bytes(2))
        return struct.unpack("<H", data)[0]

    def _write(self, address: int, value: int):
        self._require_probe()
        address_data, value_data = type0_register_payload(address), type0_register_payload(value)
        self._query(UVC_SET_CUR, address_data)
        self._query(UVC_SET_CUR, value_data)

    def read_register(self, address: int) -> int:
        """Not GET-only: selecting a register sends SET_CUR first."""
        with self._lock:
            return self._read(address)

    def discover_dsp_type(self) -> int:
        """Source AC50 bank/0213 discovery, not complete CreateDeviceObject.

        This changes bank registers; it is NOT part of passive probe(). Unknown
        IDs restore the source-saved banks and fail. Special3469/347F require
        additional native initialization, so are not silently accepted here.
        """
        with self._lock:
            self._dsp_type = None
            self._write(0xD160, 2)
            first = self._read(0x0213)
            if first == 0x2018:
                self._dsp_type = 0
                return 0
            saved = self._read(0xD160), self._read(0xD164)
            self._write(0xD160, 2)
            self._write(0xD164, 1)
            chip_id = self._read(0x0213)
            try:
                dsp = dsp_type_from_chip_id(chip_id)
            except ValueError:
                self._write(0xD160, saved[0])
                self._write(0xD164, saved[1])
                raise
            if dsp in (6, 7):
                raise RuntimeError("DSP3469/347F extra A390 initialization is not implemented")
            self._dsp_type = dsp
            return dsp

    def set_led(self, led: int, enabled: bool) -> None:
        """Exact ClassLEDControl GPIO RMW / LED3 write, under owner's lock.

        Return attests successful ioctl delivery only, not optical verification.
        On any transport failure no later write is sent and discovery is invalidated.
        """
        if type(led) is not int or led not in (1, 2, 3) or type(enabled) is not bool:
            raise ValueError("LED1/2/3 and boolean enabled required")
        with self._lock:
            if self._dsp_type is None:
                raise RuntimeError("source register0213 DSP discovery required")
            prefix = register_bank_prefix(led, self._dsp_type)
            for address, value in prefix:
                self._write(address, value)
            if led == 3:
                self._write(LED3_REGISTER, int(enabled))
            else:
                value = updated_gpio0(self._read(GPIO0_REGISTER) & 0xFF, led, enabled)
                for address, bank_value in prefix:
                    self._write(address, bank_value)
                self._write(GPIO0_REGISTER, value)
