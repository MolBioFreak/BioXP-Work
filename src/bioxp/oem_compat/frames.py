from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class TrafficCategory(str, Enum):
    MOTION = "motion"
    CANOPEN_UIM = "canopen_uim"
    PIPETTE = "pipette"


@dataclass(frozen=True)
class OemCommandFrame:
    """Logical equivalent of OEM InterfaceCAN.TransmitMessage arguments."""

    sidh: int
    sidl: int
    cmd: bytes
    message: str = ""
    timeout_ms: int = 60000

    @property
    def category(self) -> TrafficCategory:
        # Mirrors ClassNovoCANUSB.transmitCommand traffic classification.
        if 4 <= self.sidh <= 9:
            return TrafficCategory.MOTION
        if self.sidh == 0 or self.sidh > 512:
            return TrafficCategory.CANOPEN_UIM
        return TrafficCategory.PIPETTE


@dataclass(frozen=True)
class OemReplyFrame:
    category: TrafficCategory
    payload: bytes = b""
    status: int | None = None
    synthetic: bool = False
