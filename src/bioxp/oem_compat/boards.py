from __future__ import annotations

from dataclasses import dataclass

from .motor import Motor
from .transport import DryRunTransport

BOARD_HEAD = 0x04
BOARD_DECK = 0x05
BOARD_THERMAL = 0x06
BOARD_CHILLER = 0x07


@dataclass(frozen=True)
class AxisProfile:
    key: str
    label: str
    board_id: int
    motor: int
    speed: int
    acc: int
    run_current: int
    standby_current: int
    stall_guard: int
    disable_right: bool | None = None
    disable_left: bool | None = None
    rdiv: int | None = None
    pdiv: int | None = None


AXIS_PROFILES: dict[str, AxisProfile] = {
    "x": AxisProfile("x", "X", BOARD_DECK, 0, 1700, 350, 31, 10, 16, True, False),
    "y": AxisProfile("y", "Y", BOARD_HEAD, 0, 1800, 400, 31, 10, 16, True, False),
    "z": AxisProfile("z", "Z", BOARD_HEAD, 1, 1791, 576, 31, 10, 16, False, False),
    "g": AxisProfile("g", "GRIPPER", BOARD_HEAD, 2, 600, 20, 20, 10, 5, None, None, 6, 2),
    "door": AxisProfile("door", "THERMAL_DOOR", BOARD_THERMAL, 0, 600, 200, 20, 10, 6, True, True),
}

ALIASES = {
    "gripper": "g",
    "thermaldoor": "door",
    "thermal_door": "door",
    "d": "door",
}


@dataclass
class Board:
    board_id: int
    label: str
    transport: DryRunTransport

    def motor(self, motor: int, label: str = "") -> Motor:
        return Motor(board_id=self.board_id, motor=int(motor), transport=self.transport, label=label or self.label)


@dataclass
class BioXPBoards:
    transport: DryRunTransport
    head: Board
    deck: Board
    thermal: Board
    chiller: Board

    @classmethod
    def dry_run(cls) -> "BioXPBoards":
        transport = DryRunTransport()
        return cls.from_transport(transport)

    @classmethod
    def from_transport(cls, transport: DryRunTransport) -> "BioXPBoards":
        return cls(
            transport=transport,
            head=Board(BOARD_HEAD, "HEAD", transport),
            deck=Board(BOARD_DECK, "DECK", transport),
            thermal=Board(BOARD_THERMAL, "THERMAL", transport),
            chiller=Board(BOARD_CHILLER, "CHILLER", transport),
        )

    def profile(self, axis: str) -> AxisProfile:
        key = ALIASES.get(str(axis).strip().lower(), str(axis).strip().lower())
        try:
            return AXIS_PROFILES[key]
        except KeyError as exc:
            raise KeyError(f"Unknown OEM BioXP axis {axis!r}") from exc

    def axis(self, axis: str) -> Motor:
        profile = self.profile(axis)
        return Motor(profile.board_id, profile.motor, self.transport, profile.label)
