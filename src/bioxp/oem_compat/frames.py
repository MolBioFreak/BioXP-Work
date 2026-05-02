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

    For TMCL-style motor frames, command/cmd_type/motor/value are also retained
    so tests can verify source-level OEM semantics as well as raw bytes.
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

    @property
    def category(self) -> TrafficCategory:
        # Mirrors ClassNovoCANUSB.transmitCommand traffic classification.
        if 4 <= self.sidh <= 9:
            return TrafficCategory.MOTION
        if self.sidh == 0 or self.sidh > 512:
            return TrafficCategory.CANOPEN_UIM
        return TrafficCategory.PIPETTE

    @property
    def raw(self) -> bytes:
        return self.cmd

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
        raw = build_tmcl_frame(board_id, command, cmd_type, motor, value)
        return cls(
            sidh=int(board_id),
            sidl=0,
            cmd=raw,
            message=message,
            timeout_ms=int(timeout_ms),
            command=int(command),
            cmd_type=int(cmd_type),
            motor=int(motor),
            value=int(value),
        )


@dataclass(frozen=True)
class OemReplyFrame:
    category: TrafficCategory
    payload: bytes = b""
    status: int | None = None
    synthetic: bool = False


def build_tmcl_frame(board_id: int, command: int, cmd_type: int, motor: int, value: int) -> bytes:
    """Build the same TMCL-like raw frame shape used by src/bioxp/usb_driver.py."""

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
