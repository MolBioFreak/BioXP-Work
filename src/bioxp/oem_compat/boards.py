from __future__ import annotations

from dataclasses import dataclass

from .frames import OemCommandFrame
from .motor import Motor
from .transport import DryRunTransport

BOARD_HEAD = 0x04
BOARD_DECK = 0x05
BOARD_THERMAL = 0x06
BOARD_CHILLER = 0x07
BOARD_TFF_DECK = 0x08
BOARD_TFF_CHILLER = 0x09


def _be(value: int) -> bytes:
    return int(value).to_bytes(4, "big", signed=True)


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
    "x": AxisProfile("x", "X", BOARD_DECK, 0, 1700, 350, 31, 10, 16, False, False),
    "y": AxisProfile("y", "Y", BOARD_HEAD, 0, 1800, 400, 31, 10, 16, True, False),
    "z": AxisProfile("z", "Z", BOARD_HEAD, 1, 1791, 576, 31, 10, 3, False, False),
    "g": AxisProfile("g", "GRIPPER", BOARD_HEAD, 2, 1500, 20, 10, 10, 20, None, None, 6, 2),
    "door": AxisProfile("door", "THERMAL_DOOR", BOARD_THERMAL, 0, 600, 200, 20, 10, 6, True, True),
}

OEM_HOME_PROFILES = {
    "x": {"startup_home_speed": 250, "manual_home_speed": 500, "home_method": "goHome"},
    "y": {"startup_home_speed": 250, "manual_home_speed": 500, "home_method": "goHome"},
    "z": {"startup_home_speed": 1791, "manual_home_speed": 1791, "home_method": "goHome"},
    "g": {"startup_home_speed": 200, "manual_home_speed": 200, "gripper_version_1_home_speed": 200, "gripper_default_home_speed": 600, "home_method": "goHome"},
    "door": {"startup_home_speed": 600, "manual_home_speed": 600, "home_method": "doorSearchHome"},
}

OEM_SOURCE_ANCHOR = "BioXPControlLib/ClassControlInterface.cs initializeMotors, initializeMotorsWithoutMotion, btnHome*_Click"


def axis_profile_matrix() -> dict[str, dict]:
    """Return source-anchored motor profile data used by the test handoff matrix."""
    rows: dict[str, dict] = {}
    for key, profile in AXIS_PROFILES.items():
        rows[key] = {
            "key": key,
            "label": profile.label,
            "board_id": profile.board_id,
            "motor": profile.motor,
            "speed": profile.speed,
            "acc": profile.acc,
            "run_current": profile.run_current,
            "standby_current": profile.standby_current,
            "stall_guard": profile.stall_guard,
            "disable_right": profile.disable_right,
            "disable_left": profile.disable_left,
            "rdiv": profile.rdiv,
            "pdiv": profile.pdiv,
            **OEM_HOME_PROFILES[key],
            "source_anchor": OEM_SOURCE_ANCHOR,
        }
    return rows

ALIASES = {"gripper": "g", "thermaldoor": "door", "thermal_door": "door", "d": "door"}


@dataclass
class Board:
    board_id: int
    label: str
    transport: DryRunTransport

    def motor(self, motor: int, label: str = "") -> Motor:
        return Motor(board_id=self.board_id, motor=int(motor), transport=self.transport, label=label or self.label)

    def _send_payload(self, payload: bytes, message: str) -> None:
        self.transport.transmit(OemCommandFrame.from_payload(self.board_id, payload, message=message))

    def activate(self) -> None:
        self._send_payload(bytes([64, 0, 0, 0, 0, 0, 1]), f"{self.label}.activateBoard")

    def deactivate(self) -> None:
        self._send_payload(bytes([64, 0, 0, 0, 0, 0, 0]), f"{self.label}.deactivateBoard")

    def query_firmware_version(self) -> None:
        self._send_payload(bytes([173, 0, 0, 0, 0, 0, 0]), f"{self.label}.queryFirmwareVersion")


@dataclass
class DeckBoard(Board):
    def set_led(self, mask: int, intensity: int) -> None:
        pwm = int(1024 * int(intensity) / 255)
        self._send_payload(bytes([50, 0, int(mask) & 0xFF]) + _be(pwm), f"DECK.setLED({mask},{intensity})")

    def set_color(self, r: int, g: int, b: int) -> None:
        self.set_led(0, r)
        self.set_led(1, g)
        self.set_led(2, b)

    def query_24v_sensor(self) -> None:
        self._send_payload(bytes([15, 0, 0, 0, 0, 0, 0]), "DECK.query24VSensor")

    def query_door_sensor(self) -> None:
        self._send_payload(bytes([15, 1, 0, 0, 0, 0, 0]), "DECK.queryDoorSensor")

    def query_solenoid_control(self) -> None:
        self._send_payload(bytes([15, 2, 0, 0, 0, 0, 0]), "DECK.querySolenoidControl")

    def query_latch_sensor(self) -> None:
        self._send_payload(bytes([15, 3, 0, 0, 0, 0, 0]), "DECK.queryLatchSensor")

    def set_solenoid_control(self, onoff: int) -> None:
        self._send_payload(bytes([14, 2, 0, 0, 0, 0, int(onoff) & 0xFF]), f"DECK.setSolenoidControl({onoff})")


@dataclass
class BioXPBoards:
    transport: DryRunTransport
    head: Board
    deck: DeckBoard
    thermal: Board
    chiller: Board

    @classmethod
    def dry_run(cls) -> "BioXPBoards":
        return cls.from_transport(DryRunTransport())

    @classmethod
    def from_transport(cls, transport: DryRunTransport) -> "BioXPBoards":
        return cls(
            transport=transport,
            head=Board(BOARD_HEAD, "HEAD", transport),
            deck=DeckBoard(BOARD_DECK, "DECK", transport),
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
        p = self.profile(axis)
        return Motor(p.board_id, p.motor, self.transport, p.label)
