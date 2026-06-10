from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .boards import AXIS_PROFILES, BioXPBoards
from .transport import DryRunTransport


@dataclass(frozen=True)
class TraceOperation:
    name: str
    axis: str | None = None
    value: Any = None
    details: dict[str, Any] = field(default_factory=dict)


@dataclass
class OperationTrace:
    name: str
    operations: list[TraceOperation] = field(default_factory=list)

    def add(self, name: str, axis: str | None = None, value: Any = None, **details: Any) -> TraceOperation:
        op = TraceOperation(name=name, axis=axis, value=value, details=details)
        self.operations.append(op)
        return op


class BioXPControlInterface:
    def __init__(self, boards: BioXPBoards):
        self.boards = boards
        self.transport = boards.transport

    @classmethod
    def dry_run(cls) -> "BioXPControlInterface":
        return cls(BioXPBoards.dry_run())

    def wait_for_board(self, trace: OperationTrace | None = None) -> None:
        if trace:
            trace.add("wait_for_board", details={"boards": [4, 5, 6, 7]})

    def activate_boards(self, trace: OperationTrace | None = None) -> None:
        for b in (self.boards.head, self.boards.deck, self.boards.thermal, self.boards.chiller):
            b.activate()
        if trace:
            trace.add("activate_boards", details={"boards": [4, 5, 6, 7]})

    def deactivate_boards(self, trace: OperationTrace | None = None) -> None:
        for b in (self.boards.head, self.boards.deck, self.boards.thermal, self.boards.chiller):
            b.deactivate()
        if trace:
            trace.add("deactivate_boards", details={"boards": [4, 5, 6, 7]})

    def initialize_motors_without_motion(self) -> OperationTrace:
        trace = OperationTrace("initialize_motors_without_motion")
        self.wait_for_board(trace)
        trace.add("turn_off_heater")
        trace.add("set_chiller_pwm", value="oem_default")
        trace.add("set_chiller_rates", value="oem_default")
        for key in ("x", "y", "z", "g", "door"):
            self._configure_axis(key, trace)
        trace.add("set_chiller_cool_rate", value="OC")
        trace.add("set_chiller_cool_rate", value="RC")
        trace.add("set_tc_heat_rate", value=2.5)
        trace.add("set_tc_cool_rate", value=-2.0)
        self.boards.deck.set_color(255, 255, 255)
        trace.add("set_deck_color", value={"r": 255, "g": 255, "b": 255})
        return trace

    def _configure_axis(self, axis: str, trace: OperationTrace) -> None:
        p = AXIS_PROFILES[axis]
        m = self.boards.axis(axis)
        m.set_max_speed(p.speed)
        m.set_max_acc(p.acc)
        trace.add("set_speed_acc", axis, {"speed": p.speed, "acc": p.acc})
        m.set_max_current(p.run_current)
        trace.add("set_run_current", axis, p.run_current)
        m.set_standby_current(p.standby_current)
        trace.add("set_standby_current", axis, p.standby_current)
        if axis == "z":
            m.read_max_current()
            trace.add("read_max_current", axis)
        m.set_stall_guard_threshold(p.stall_guard)
        trace.add("set_stall_guard", axis, p.stall_guard)
        if p.disable_right:
            m.disable_right_limit_switch()
            trace.add("disable_right_switch", axis)
        if p.disable_left:
            m.disable_left_limit_switch()
            trace.add("disable_left_switch", axis)
        if p.rdiv is not None or p.pdiv is not None:
            if p.rdiv is not None:
                m.set_rdiv(p.rdiv)
            if p.pdiv is not None:
                m.set_pdiv(p.pdiv)
            trace.add("set_rdiv_pdiv", axis, {"rdiv": p.rdiv, "pdiv": p.pdiv})

    def initialize_motors(self) -> OperationTrace:
        trace = OperationTrace("initialize_motors")
        trace.operations.extend(self.initialize_motors_without_motion().operations)
        trace.operations.extend(self.startup_homing().operations)
        return trace

    def home_axis(self, axis: str, *, startup: bool = False) -> OperationTrace:
        key = {"gripper": "g", "d": "door"}.get(str(axis).lower(), str(axis).lower())
        trace = OperationTrace(f"home_{key}")
        speed = self._home_speed(key, startup=startup)
        m = self.boards.axis(key)
        if key == "door":
            trace.add("door_search_home", key, {"speed": speed, "stall_guard": AXIS_PROFILES["door"].stall_guard})
            m.set_stall_guard_threshold(AXIS_PROFILES["door"].stall_guard)
            m.move_left(speed)
            m.stop()
            m.set_home()
            return trace
        if key == "z":
            m.set_max_current(31)
            trace.add("set_run_current", key, 31)
        if key == "g":
            m.set_max_current(31)
            trace.add("set_run_current", key, 31)
            m.set_stall_guard_threshold(5)
            trace.add("set_stall_guard", key, 5)
        m.set_home()
        m.move_left(speed)
        trace.add("home_axis", key, speed, startup=bool(startup))
        m.stop()
        trace.add("stop", key)
        m.set_home()
        trace.add("set_home", key, 0)
        return trace

    @staticmethod
    def _home_speed(axis: str, *, startup: bool = False) -> int:
        if axis == "x":
            return 250
        if axis == "y":
            return 250 if startup else 250
        if axis == "z":
            return 1791 if startup else 597
        if axis == "g":
            return 200
        if axis == "door":
            return 600
        return AXIS_PROFILES[axis].speed

    def startup_homing(self) -> OperationTrace:
        trace = OperationTrace("startup_homing")
        g = self.boards.axis("g")
        g.set_max_current(31)
        trace.add("set_run_current", "g", 31)
        g.move_relative(10000)
        trace.add("move_relative", "g", 10000, reason="OEM gripper pre-home clearance")
        for axis in ("z", "g", "x"):
            trace.operations.extend(self.home_axis(axis, startup=True).operations)
        x = self.boards.axis("x")
        x.set_home()
        trace.add("set_home", "x", 0)
        x.set_max_speed(AXIS_PROFILES["x"].speed)
        trace.add("set_max_speed", "x", AXIS_PROFILES["x"].speed)
        x.move_absolute(6000)
        trace.add("move_absolute", "x", 6000, reason="OEM post-home X offset")
        trace.operations.extend(self.home_axis("y", startup=True).operations)
        trace.operations.extend(self.home_axis("door", startup=True).operations)
        return trace

    def enable_xyz(self, enable: bool) -> OperationTrace:
        trace = OperationTrace("enable_xyz")
        trace.add("enable_xyz", value=bool(enable))
        return trace

    def enable_xy(self, enable: bool) -> OperationTrace:
        trace = OperationTrace("enable_xy")
        trace.add("enable_xy", value=bool(enable))
        return trace

    def reset_xy_limits(self) -> OperationTrace:
        trace = OperationTrace("reset_xy_limits")
        for axis in ("x", "y"):
            p = AXIS_PROFILES[axis]
            self.boards.axis(axis).set_limits(disable_right=p.disable_right, disable_left=p.disable_left)
            trace.add("set_limits", axis, {"disable_right": p.disable_right, "disable_left": p.disable_left})
        return trace

    def force_abort_motion(self) -> OperationTrace:
        trace = OperationTrace("force_abort_motion")
        for axis in ("x", "y", "z", "g", "door"):
            self.boards.axis(axis).stop()
            trace.add("stop", axis)
        return trace
