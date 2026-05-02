from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import Enum


class TrafficCategory(str, Enum):
    MOTION = "motion"
    CANOPEN_UIM = "canopen_uim"
    PIPETTE = "pipette"


@dataclass(frozen=True)
class OemCommandFrame:
    """Logical equivalent of OEM InterfaceCAN.TransmitMessage arguments.

    OEM ClassCanLib sends a 7-byte command payload via InterfaceCAN.TransmitMessage.
    The current Linux USB driver wraps equivalent motor operations in a 0x7E-framed
    raw envelope, so this object stores both:
    - oem_payload: source-grounded 7-byte OEM payload
    - cmd/raw: Linux raw dry-run envelope or raw transport payload
    """

    sidh: int
    sidl: int
    cmd: bytes
    message: str = ""
    timeout_ms: int = 60000
    command: int | None = None
    cmd_type: int | None = None
    motor: int | None = None
    value: int | None = None
    oem_payload_bytes: bytes | None = None

    @property
    def oem_payload(self) -> bytes:
        return self.oem_payload_bytes if self.oem_payload_bytes is not None else self.cmd

    @property
    def category(self) -> TrafficCategory:
        if 4 <= self.sidh <= 9:
            return TrafficCategory.MOTION
        if self.sidh == 0 or self.sidh > 512:
            return TrafficCategory.CANOPEN_UIM
        return TrafficCategory.PIPETTE

    @property
    def raw(self) -> bytes:
        return self.cmd

    @classmethod
    def from_payload(
        cls,
        board_id: int,
        payload: bytes,
        *,
        message: str = "",
        timeout_ms: int = 60000,
        raw: bytes | None = None,
    ) -> "OemCommandFrame":
        return cls(
            sidh=int(board_id),
            sidl=0,
            cmd=bytes(raw if raw is not None else payload),
            message=message,
            timeout_ms=int(timeout_ms),
            command=payload[0] if len(payload) > 0 else None,
            cmd_type=payload[1] if len(payload) > 1 else None,
            motor=payload[2] if len(payload) > 2 else None,
            value=int.from_bytes(payload[3:7], "big", signed=True) if len(payload) >= 7 else None,
            oem_payload_bytes=bytes(payload),
        )

    @classmethod
    def tmcl(
        cls,
        board_id: int,
        command: int,
        cmd_type: int,
        motor: int,
        value: int,
        *,
        message: str = "",
        timeout_ms: int = 60000,
    ) -> "OemCommandFrame":
        payload = build_oem_motor_payload(command, cmd_type, motor, value)
        raw = build_tmcl_frame(board_id, command, cmd_type, motor, value)
        return cls.from_payload(
            board_id,
            payload,
            message=message,
            timeout_ms=timeout_ms,
            raw=raw,
        )


@dataclass(frozen=True)
class OemReplyFrame:
    category: TrafficCategory
    payload: bytes = b""
    status: int | None = None
    synthetic: bool = False


def build_oem_motor_payload(command: int, cmd_type: int, motor: int, value: int) -> bytes:
    return bytes([int(command) & 0xFF, int(cmd_type) & 0xFF, int(motor) & 0xFF]) + struct.pack(">i", int(value))


def build_tmcl_frame(board_id: int, command: int, cmd_type: int, motor: int, value: int) -> bytes:
    val = struct.pack(">i", int(value))
    inner = bytearray(
        [
            0x00,
            0x00,
            0x00,
            int(board_id),
            0x08,
            int(command),
            int(cmd_type),
            int(motor),
            val[0],
            val[1],
            val[2],
            val[3],
            0x00,
        ]
    )
    chk = sum(inner) & 0xFF
    return bytes(bytearray([0x7E]) + inner + bytearray([chk, 0x7E]))
