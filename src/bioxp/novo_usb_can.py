from __future__ import annotations

import time
from types import SimpleNamespace
from typing import Any

try:
    import usb.core  # type: ignore
    import usb.util  # type: ignore
except ModuleNotFoundError as exc:  # pragma: no cover - exercised when pyusb absent
    usb = None  # type: ignore[assignment]
    _usb_import_error: ModuleNotFoundError | None = exc
else:  # pragma: no cover - import itself is environment-dependent
    _usb_import_error = None

from .can_driver import BioXpCanDriver

NOVO_USB_VENDOR_ID = 0x03EB
NOVO_USB_PRODUCT_ID = 0x2423
NOVO_USB_DEFAULT_ALT = 1
NOVO_FRAME = 0x7E
NOVO_ESCAPE = 0x7D
NOVO_ESCAPE_XOR = 0x20


class NovoUsbCanError(RuntimeError):
    """Raised when the Novo USB-CAN transport cannot be opened or decoded."""


def novo_checksum(payload: bytes | bytearray | list[int]) -> int:
    return sum(int(byte) & 0xFF for byte in payload) & 0xFF


def novo_encode(payload: bytes | bytearray | list[int]) -> bytes:
    """Encode the OEM Novo USB-CAN payload using NovoEncoding.cs framing.

    The payload is the unescaped CAN record used by OEM CanInterfaceBoard:
    4-byte big-endian module/arbitration id, 1-byte DLC, then DLC data bytes.
    NovoEncoding appends an 8-bit additive checksum, escapes 0x7e/0x7d, and
    wraps the result in 0x7e frame delimiters.
    """

    src = bytes(int(byte) & 0xFF for byte in payload)
    out = bytearray([NOVO_FRAME])
    for byte in src + bytes([novo_checksum(src)]):
        if byte in (NOVO_FRAME, NOVO_ESCAPE):
            out.append(NOVO_ESCAPE)
            out.append(byte ^ NOVO_ESCAPE_XOR)
        else:
            out.append(byte)
    out.append(NOVO_FRAME)
    return bytes(out)


def novo_decode(frame: bytes | bytearray | list[int], *, verify_checksum: bool = True) -> bytes:
    """Decode one OEM Novo USB-CAN framed packet.

    Unlike the decompiled NovoEncoding.Decode, this validates the delimiter and
    checksum so bad reads are reported explicitly instead of poisoning demux.
    """

    raw = bytes(int(byte) & 0xFF for byte in frame)
    try:
        start = raw.index(NOVO_FRAME)
        end = raw.rindex(NOVO_FRAME)
    except ValueError as exc:
        raise NovoUsbCanError("Novo USB frame is missing 0x7e delimiter") from exc
    if end <= start + 1:
        raise NovoUsbCanError("Novo USB frame is empty")
    body = raw[start + 1 : end]
    decoded = bytearray()
    i = 0
    while i < len(body):
        byte = body[i]
        if byte == NOVO_ESCAPE:
            i += 1
            if i >= len(body):
                raise NovoUsbCanError("Novo USB frame has a dangling escape byte")
            decoded.append(body[i] ^ NOVO_ESCAPE_XOR)
        else:
            decoded.append(byte)
        i += 1
    if len(decoded) < 2:
        raise NovoUsbCanError("Novo USB frame is too short for payload+checksum")
    payload = bytes(decoded[:-1])
    checksum = decoded[-1]
    expected = novo_checksum(payload)
    if verify_checksum and checksum != expected:
        raise NovoUsbCanError(f"Novo USB checksum mismatch: got 0x{checksum:02x}, expected 0x{expected:02x}")
    return payload


class NovoUsbCanBus:
    """PyUSB-backed bus adapter exposing the tiny python-can API we need.

    This mirrors OEM Novo.Devices.CanInterfaceBoard:
    - find VID:PID 03eb:2423
    - use interface 0 alternate setting 1
    - bulk OUT writes NovoEncoding( module_id[4] + dlc[1] + data[dlc] )
    - bulk IN reads the same record shape and returns a message-like object

    If ``shared_usb`` is a live BioXpTester, its already-claimed endpoints are
    reused instead of trying to claim the interface a second time.
    """

    def __init__(
        self,
        *,
        shared_usb: Any | None = None,
        vendor_id: int = NOVO_USB_VENDOR_ID,
        product_id: int = NOVO_USB_PRODUCT_ID,
        alt: int = NOVO_USB_DEFAULT_ALT,
        read_size: int = 256,
        write_timeout_ms: int = 2000,
        claim_interface: bool = True,
    ) -> None:
        if _usb_import_error is not None and shared_usb is None:
            raise NovoUsbCanError(f"pyusb is required for Novo USB-CAN access: {_usb_import_error}")
        self.vendor_id = int(vendor_id)
        self.product_id = int(product_id)
        self.alt = int(alt)
        self.read_size = int(read_size)
        self.write_timeout_ms = int(write_timeout_ms)
        self.claim_interface = bool(claim_interface)
        self.shared_usb = shared_usb
        self.dev: Any | None = None
        self.ep_out: Any | None = None
        self.ep_in: Any | None = None
        self._owns_device = shared_usb is None
        self._connect()

    def _connect(self) -> None:
        if self.shared_usb is not None:
            self.dev = getattr(self.shared_usb, "dev", None)
            self.ep_out = getattr(self.shared_usb, "ep_out", None)
            self.ep_in = getattr(self.shared_usb, "ep_in", None)
            if self.ep_out is None or self.ep_in is None:
                raise NovoUsbCanError("shared BioXpTester does not expose live USB endpoints")
            return

        assert usb is not None
        self.dev = usb.core.find(idVendor=self.vendor_id, idProduct=self.product_id)
        if self.dev is None:
            raise NovoUsbCanError(
                f"Novo USB-CAN device not found (vid=0x{self.vendor_id:04x}, pid=0x{self.product_id:04x})"
            )
        try:
            if self.dev.is_kernel_driver_active(0):
                try:
                    self.dev.detach_kernel_driver(0)
                except usb.core.USBError:
                    pass
        except (NotImplementedError, usb.core.USBError):
            pass
        self.dev.set_configuration()
        if self.claim_interface:
            usb.util.claim_interface(self.dev, 0)
        self.dev.set_interface_altsetting(interface=0, alternate_setting=self.alt)
        cfg = self.dev.get_active_configuration()
        intf = cfg[(0, self.alt)]
        self.ep_out = usb.util.find_descriptor(
            intf,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT,
        )
        self.ep_in = usb.util.find_descriptor(
            intf,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN,
        )
        if self.ep_out is None or self.ep_in is None:
            raise NovoUsbCanError("Novo USB-CAN bulk endpoints were not found")

    @staticmethod
    def build_payload(arbitration_id: int, data: list[int] | bytes | bytearray, dlc: int | None = None) -> bytes:
        payload_data = [int(byte) & 0xFF for byte in data]
        data_len = len(payload_data) if dlc is None else int(dlc)
        data_len = max(0, min(255, data_len))
        payload_data = payload_data[:data_len]
        module_id = int(arbitration_id) & 0xFFFFFFFF
        return module_id.to_bytes(4, "big") + bytes([data_len]) + bytes(payload_data)

    def send(self, msg: Any) -> None:
        if self.ep_out is None:
            raise NovoUsbCanError("Novo USB-CAN OUT endpoint is not connected")
        payload = self.build_payload(
            int(getattr(msg, "arbitration_id")),
            list(getattr(msg, "data", [])),
            int(getattr(msg, "dlc", len(getattr(msg, "data", [])))),
        )
        frame = novo_encode(payload)
        self.ep_out.write(frame, timeout=self.write_timeout_ms)

    def recv(self, timeout: float | None = None) -> Any | None:
        if self.ep_in is None:
            raise NovoUsbCanError("Novo USB-CAN IN endpoint is not connected")
        timeout_ms = None if timeout is None else max(1, int(float(timeout) * 1000.0))
        try:
            raw = self.ep_in.read(self.read_size, timeout=timeout_ms)
        except Exception as exc:
            if exc.__class__.__name__ == "USBTimeoutError" or isinstance(exc, TimeoutError):
                return None
            raise
        decoded = novo_decode(list(raw))
        if len(decoded) < 5:
            raise NovoUsbCanError(f"Novo USB-CAN decoded payload too short: {decoded!r}")
        arbitration_id = int.from_bytes(decoded[0:4], "big")
        dlc = int(decoded[4])
        data = list(decoded[5 : 5 + dlc])
        return SimpleNamespace(
            arbitration_id=arbitration_id,
            data=data,
            dlc=len(data),
            is_extended_id=False,
            timestamp=time.time(),
        )

    def shutdown(self) -> None:
        if not self._owns_device or self.dev is None or usb is None:
            return
        try:
            usb.util.release_interface(self.dev, 0)
        except Exception:
            pass
        try:
            usb.util.dispose_resources(self.dev)
        except Exception:
            pass
        self.dev = None
        self.ep_out = None
        self.ep_in = None


class BioXpNovoUsbDriver(BioXpCanDriver):
    """BioXpCanDriver-compatible pipette driver over OEM Novo USB-CAN."""

    def __init__(
        self,
        *,
        shared_usb: Any | None = None,
        alt: int = NOVO_USB_DEFAULT_ALT,
        pipette_id: int = 0,
        response_timeout_s: float = 1.0,
        vendor_id: int = NOVO_USB_VENDOR_ID,
        product_id: int = NOVO_USB_PRODUCT_ID,
    ) -> None:
        self.bus = NovoUsbCanBus(
            shared_usb=shared_usb,
            alt=alt,
            vendor_id=vendor_id,
            product_id=product_id,
        )
        self.channel = "novo-usb-shared" if shared_usb is not None else "novo-usb"
        self.bitrate = 0
        self.pipette_id = int(pipette_id)
        self.response_timeout_s = float(response_timeout_s)
        self.usb = {
            "vendor_id": int(vendor_id),
            "product_id": int(product_id),
            "alt": int(alt),
            "shared_usb": shared_usb is not None,
            "framing": "NovoEncoding.cs",
        }

    def close(self):
        shutdown = getattr(self.bus, "shutdown", None)
        if callable(shutdown):
            shutdown()
