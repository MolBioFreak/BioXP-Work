from __future__ import annotations

import time
from typing import Any, Callable

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


def _validate_novo_payload(payload: bytes) -> bytes:
    if len(payload) < 5:
        raise NovoUsbCanError("Novo CAN payload is shorter than module-id plus DLC")
    dlc = int(payload[4])
    if dlc > 8:
        raise NovoUsbCanError(f"Novo CAN payload has invalid DLC: {dlc}")
    expected_length = 5 + dlc
    if len(payload) != expected_length:
        raise NovoUsbCanError(
            f"Novo CAN payload/DLC mismatch: dlc={dlc}, payload_length={len(payload) - 5}"
        )
    return payload


def novo_encode(payload: bytes | bytearray | list[int]) -> bytes:
    """Encode one validated OEM Novo USB-CAN CAN record."""
    src = _validate_novo_payload(bytes(int(byte) & 0xFF for byte in payload))
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
    """Decode exactly one validated OEM Novo USB-CAN framed packet.

    The vendor decoder is permissive; Linux deliberately rejects concatenated,
    partial, malformed, or structurally invalid frames before router demux.
    """
    raw = bytes(int(byte) & 0xFF for byte in frame)
    if len(raw) < 4 or raw[0] != NOVO_FRAME or raw[-1] != NOVO_FRAME:
        raise NovoUsbCanError("Novo USB frame must start and end with 0x7e")
    if raw.count(NOVO_FRAME) != 2:
        raise NovoUsbCanError("Novo USB read contains more than one frame")
    body = raw[1:-1]
    if not body:
        raise NovoUsbCanError("Novo USB frame is empty")
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
            if byte == NOVO_FRAME:
                raise NovoUsbCanError("Novo USB frame contains an unescaped delimiter")
            decoded.append(byte)
        i += 1
    if len(decoded) < 2:
        raise NovoUsbCanError("Novo USB frame is too short for payload+checksum")
    payload = bytes(decoded[:-1])
    checksum = decoded[-1]
    expected = novo_checksum(payload)
    if verify_checksum and checksum != expected:
        raise NovoUsbCanError(f"Novo USB checksum mismatch: got 0x{checksum:02x}, expected 0x{expected:02x}")
    return _validate_novo_payload(payload)


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

    def _finish_pipette_response(
        self,
        *,
        router: NovoRouter,
        response: dict[str, Any],
        channel: int,
        completion_owner_token: str | None,
        completion_timeout_s: float,
        wait_for_completion: bool,
    ) -> dict[str, Any]:
        if completion_owner_token is None:
            query_verified = bool(
                response.get("ok")
                and response.get("outcome") in {"completion", "multipart_completion"}
            )
            return {
                **response,
                "completion_received": False,
                "completion_deferred": False,
                "semantic_query_response_verified": query_verified,
            }
        immediate_ack = bool(response.get("outcome") == "ack" and response.get("ack_received"))
        if not immediate_ack:
            completion = router.wait_pipette_completion(
                int(channel),
                0.0,
                owner_token=completion_owner_token,
            )
            return {
                **response,
                "ok": False,
                "immediate_ack_received": False,
                "controller_acknowledged": False,
                "completion_received": False,
                "completion": completion,
            }
        if not wait_for_completion:
            return {
                **response,
                "ok": True,
                "outcome": "ack",
                "immediate_ack_received": True,
                "controller_acknowledged": True,
                "completion_received": False,
                "completion_deferred": True,
            }
        completion = router.wait_pipette_completion(
            int(channel),
            float(completion_timeout_s),
            owner_token=completion_owner_token,
        )
        return {
            **response,
            "ok": bool(completion.get("ok")),
            "outcome": completion.get("outcome"),
            "immediate_ack_received": True,
            "controller_acknowledged": True,
            "completion_received": bool(completion.get("ok")),
            "completion_deferred": False,
            "completion": completion,
        }

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
        allow_multipart: bool = False,
        wait_for_completion: bool = True,
        replace_completion_owner_token: str | None = None,
        completion_replacement_reason: str | None = None,
    ) -> dict[str, Any]:
        router = self._current_router()
        tx_id = int(getattr(msg, "arbitration_id"))
        tx_data = list(getattr(msg, "data", []))
        tx_dlc = int(getattr(msg, "dlc", len(tx_data)))
        payload = self.build_payload(tx_id, tx_data, tx_dlc)
        expected_tx_id = 0x100 | (int(channel) << 3) | int(expected_function)
        if tx_id != expected_tx_id:
            raise NovoUsbCanError(
                f"pipette TX tuple is not the exact TX-domain ID: tx=0x{tx_id:03x} expected=0x{expected_tx_id:03x}"
            )
        expected_rx_id = tx_id | 0x400
        derived_rx_id = 0x500 | (int(channel) << 3) | int(expected_function)
        if expected_rx_id != derived_rx_id:
            raise NovoUsbCanError(
                f"pipette TX tuple does not derive expected RX ID: tx=0x{tx_id:03x} expected=0x{derived_rx_id:03x}"
            )
        completion_owner_token: str | None = None
        if int(expected_function) in {0, 1}:
            completion_owner_token = router.prepare_pipette_completion(
                int(channel),
                float(completion_timeout_s),
                command_family=int(expected_function),
                command_name=str(matcher_name),
                expected_rx_id=expected_rx_id,
                replace_owner_token=replace_completion_owner_token,
                replacement_reason=completion_replacement_reason,
            )
        try:
            response = router.transact(
                novo_encode(payload),
                matcher=router.pipette_matcher(
                    channel=int(channel),
                    expected_function=int(expected_function),
                    initialization=completion_owner_token is not None,
                    allow_multipart=bool(allow_multipart),
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
                    "completion_timeout_ms": int(round(float(completion_timeout_s) * 1000.0)) if completion_owner_token is not None else None,
                    "completion_deferred": bool(completion_owner_token is not None and not wait_for_completion),
                    "completion_owner_token": completion_owner_token,
                },
            )
            return self._finish_pipette_response(
                router=router,
                response=response,
                channel=int(channel),
                completion_owner_token=completion_owner_token,
                completion_timeout_s=float(completion_timeout_s),
                wait_for_completion=bool(wait_for_completion),
            )
        except Exception:
            if completion_owner_token is not None:
                router.wait_pipette_completion(
                    int(channel),
                    0.0,
                    owner_token=completion_owner_token,
                )
            raise

    def transact_can_many(
        self,
        messages: list[Any] | tuple[Any, ...],
        *,
        channel: int,
        expected_function: int,
        timeout_s: float,
        matcher_name: str,
        initialization: bool = False,
        completion_timeout_s: float = 60.0,
        allow_multipart: bool = True,
        wait_for_completion: bool = True,
    ) -> dict[str, Any]:
        router = self._current_router()
        if len(messages) < 2:
            raise NovoUsbCanError("multi-frame CAN transaction requires at least first and final messages")
        payloads: list[bytes] = []
        tx_frames: list[dict[str, Any]] = []
        expected_ids = [
            0x103 | (int(channel) << 3),
            *([0x104 | (int(channel) << 3)] * max(0, len(messages) - 2)),
            0x100 | (int(channel) << 3) | int(expected_function),
        ]
        for index, message in enumerate(messages):
            tx_id = int(getattr(message, "arbitration_id"))
            tx_data = list(getattr(message, "data", []))
            tx_dlc = int(getattr(message, "dlc", len(tx_data)))
            if tx_id != expected_ids[index]:
                raise NovoUsbCanError(
                    "pipette multipart TX tuple has the wrong first/middle/final TX-domain ID: "
                    f"index={index} tx=0x{tx_id:03x} expected=0x{expected_ids[index]:03x}"
                )
            payloads.append(novo_encode(self.build_payload(tx_id, tx_data, tx_dlc)))
            tx_frames.append({"tx_id": tx_id, "tx_dlc": tx_dlc, "tx_data": tx_data[:tx_dlc]})
        expected_rx_id = int(getattr(messages[-1], "arbitration_id")) | 0x400
        derived_rx_id = 0x500 | (int(channel) << 3) | int(expected_function)
        if expected_rx_id != derived_rx_id:
            raise NovoUsbCanError(
                f"pipette final TX tuple does not derive expected RX ID: expected=0x{derived_rx_id:03x}"
            )
        completion_owner_token: str | None = None
        if int(expected_function) in {0, 1}:
            completion_owner_token = router.prepare_pipette_completion(
                int(channel),
                float(completion_timeout_s),
                command_family=int(expected_function),
                command_name=str(matcher_name),
                expected_rx_id=expected_rx_id,
            )
        try:
            response = router.transact_many(
                payloads,
                matcher=router.pipette_matcher(
                    channel=int(channel),
                    expected_function=int(expected_function),
                    initialization=completion_owner_token is not None,
                    allow_multipart=bool(allow_multipart),
                ),
                matcher_name=matcher_name,
                timeout_s=float(timeout_s),
                write_timeout_ms=self.write_timeout_ms,
                provenance={
                    "channel": int(channel),
                    "command_family": int(expected_function),
                    "expected_rx_id": expected_rx_id,
                    "tx_frames": tx_frames,
                    "completion_timeout_ms": int(round(float(completion_timeout_s) * 1000.0)) if completion_owner_token is not None else None,
                    "completion_deferred": bool(completion_owner_token is not None and not wait_for_completion),
                    "completion_owner_token": completion_owner_token,
                },
            )
            return self._finish_pipette_response(
                router=router,
                response=response,
                channel=int(channel),
                completion_owner_token=completion_owner_token,
                completion_timeout_s=float(completion_timeout_s),
                wait_for_completion=bool(wait_for_completion),
            )
        except Exception:
            if completion_owner_token is not None:
                router.wait_pipette_completion(
                    int(channel),
                    0.0,
                    owner_token=completion_owner_token,
                )
            raise

    def wait_pipette_completion(
        self,
        channel: int,
        timeout_s: float,
        *,
        owner_token: str | None = None,
    ) -> dict[str, Any]:
        return self._current_router().wait_pipette_completion(
            int(channel),
            float(timeout_s),
            owner_token=owner_token,
        )

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
        pipette_error_callback: Callable[[int, int], None] | None = None,
    ) -> None:
        self.bus = NovoUsbCanBus(
            shared_usb=shared_usb,
            alt=alt,
            vendor_id=vendor_id,
            product_id=product_id,
        )
        self.channel = "novo-usb-shared" if shared_usb is not None else "novo-usb"
        self.bitrate = 0
        self.pipette_id = self._validate_pipette_id(pipette_id)
        # This shared-USB subclass cannot call BioXpCanDriver.__init__ (it would
        # claim SocketCAN), but its OEM pipette commands retain the base 100 ms
        # post-wake delay contract.
        self._sleep = time.sleep
        self.response_timeout_s = 60.0
        # Do not invoke BioXpCanDriver.__init__: it opens SocketCAN, whereas
        # this subclass is owned by the shared OEM Novo USB router.  Retain
        # only the inherited driver's injectable timing dependency so its
        # OEM wake -> 100 ms -> WR sequence can run.
        self._sleep = time.sleep
        self._pipette_message_state: dict[str, Any] = {}
        self._pipette_last_command: str | None = None
        self._pipette_error_callback = pipette_error_callback
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
