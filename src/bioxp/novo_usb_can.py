from __future__ import annotations

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
from .novo_router import NovoRouter

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

    Live use requires the router owned by the shared ``BioXpTester``. This
    adapter never claims USB or reads the IN endpoint itself.
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
        if shared_usb is None:
            raise NovoUsbCanError("Novo USB-CAN requires the shared BioXpTester/NovoRouter owner")
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
        self.router: NovoRouter | None = None
        self._owns_device = False
        self._connect()

    def _connect(self) -> None:
        if self.shared_usb is not None:
            self.dev = getattr(self.shared_usb, "dev", None)
            self.ep_out = getattr(self.shared_usb, "ep_out", None)
            self.ep_in = getattr(self.shared_usb, "ep_in", None)
            if self.ep_out is None or self.ep_in is None:
                raise NovoUsbCanError("shared BioXpTester does not expose live USB endpoints")
            self.router = getattr(self.shared_usb, "novo_router", None)
            if self.router is None or not self.router.running:
                raise NovoUsbCanError("shared BioXpTester does not expose one running NovoRouter")
            return

    @staticmethod
    def build_payload(arbitration_id: int, data: list[int] | bytes | bytearray, dlc: int | None = None) -> bytes:
        payload_data = [int(byte) & 0xFF for byte in data]
        data_len = len(payload_data) if dlc is None else int(dlc)
        if data_len < 0 or data_len > 8:
            raise NovoUsbCanError(f"invalid classic CAN DLC: {data_len}")
        if data_len != len(payload_data):
            raise NovoUsbCanError(
                f"CAN DLC/data mismatch: dlc={data_len}, data_length={len(payload_data)}; padding is forbidden"
            )
        module_id = int(arbitration_id) & 0xFFFFFFFF
        return module_id.to_bytes(4, "big") + bytes([data_len]) + bytes(payload_data)

    def send(self, msg: Any) -> None:
        router = self._current_router()
        payload = self.build_payload(
            int(getattr(msg, "arbitration_id")),
            list(getattr(msg, "data", [])),
            int(getattr(msg, "dlc", len(getattr(msg, "data", [])))),
        )
        frame = novo_encode(payload)
        router.transact(
            frame,
            matcher=None,
            matcher_name="tx_only",
            timeout_s=0.0,
            write_timeout_ms=self.write_timeout_ms,
            provenance={
                "tx_id": int(getattr(msg, "arbitration_id")),
                "tx_dlc": int(getattr(msg, "dlc", len(getattr(msg, "data", [])))),
                "tx_data": list(getattr(msg, "data", [])),
            },
        )

    def recv(self, timeout: float | None = None) -> Any | None:
        raise NovoUsbCanError("direct receive is forbidden; register a transaction with NovoRouter")

    def _current_router(self) -> NovoRouter:
        router = getattr(self.shared_usb, "novo_router", None) if self.shared_usb is not None else None
        if router is None or not router.running:
            raise NovoUsbCanError("shared Novo router is unavailable")
        self.router = router
        self.ep_out = getattr(self.shared_usb, "ep_out", None)
        self.ep_in = getattr(self.shared_usb, "ep_in", None)
        return router

    def transact_can(
        self,
        msg: Any,
        *,
        channel: int,
        expected_function: int,
        timeout_s: float,
        matcher_name: str,
        initialization: bool = False,
        completion_timeout_s: float = 60.0,
    ) -> dict[str, Any]:
        router = self._current_router()
        tx_id = int(getattr(msg, "arbitration_id"))
        tx_data = list(getattr(msg, "data", []))
        tx_dlc = int(getattr(msg, "dlc", len(tx_data)))
        payload = self.build_payload(tx_id, tx_data, tx_dlc)
        expected_rx_id = tx_id | 0x400
        if initialization:
            router.prepare_pipette_completion(int(channel), float(completion_timeout_s))
        try:
            return router.transact(
                novo_encode(payload),
                matcher=router.pipette_matcher(
                    channel=int(channel),
                    expected_function=int(expected_function),
                    initialization=bool(initialization),
                ),
                matcher_name=matcher_name,
                timeout_s=float(timeout_s),
                write_timeout_ms=self.write_timeout_ms,
                provenance={
                    "channel": int(channel),
                    "command_family": int(expected_function),
                    "tx_id": tx_id,
                    "tx_dlc": tx_dlc,
                    "tx_data": tx_data[:tx_dlc],
                    "expected_rx_id": expected_rx_id,
                    "completion_timeout_ms": int(round(float(completion_timeout_s) * 1000.0)) if initialization else None,
                },
            )
        except Exception:
            if initialization:
                router.wait_pipette_completion(int(channel), 0.0)
            raise

    def wait_pipette_completion(self, channel: int, timeout_s: float) -> dict[str, Any]:
        return self._current_router().wait_pipette_completion(int(channel), float(timeout_s))

    def shutdown(self) -> None:
        # The shared BioXpTester is the lifecycle owner; a client cannot stop
        # its reader or release its USB interface.
        self.dev = None
        self.ep_out = None
        self.ep_in = None
        self.router = None


class BioXpNovoUsbDriver(BioXpCanDriver):
    """BioXpCanDriver-compatible pipette driver over OEM Novo USB-CAN."""

    def __init__(
        self,
        *,
        shared_usb: Any | None = None,
        alt: int = NOVO_USB_DEFAULT_ALT,
        pipette_id: int = 0,
        response_timeout_s: float = 60.0,
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
        self.response_timeout_s = 60.0
        self.usb = {
            "vendor_id": int(vendor_id),
            "product_id": int(product_id),
            "alt": int(alt),
            "shared_usb": shared_usb is not None,
            "framing": "NovoEncoding.cs",
            "requested_response_timeout_s": float(response_timeout_s),
            "transaction_timeout_ms": 60_000,
        }

    def close(self):
        shutdown = getattr(self.bus, "shutdown", None)
        if callable(shutdown):
            shutdown()
