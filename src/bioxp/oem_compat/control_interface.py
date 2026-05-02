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
    """Near-1:1 Linux-compatible ClassControlInterface skeleton.

    This class is intentionally dry-run/workstation-safe for now. It emits the
    OEM-order motor/board sequence into DryRunTransport and a semantic trace.
    """

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
        if trace:
            trace.add("activate_boards", details={"boards": [4, 5, 6, 7]})

    def deactivate_boards(self, trace: OperationTrace | None = None) -> None:
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
        return trace

    def _configure_axis(self, axis: str, trace: OperationTrace) -> None:
        profile = AXIS_PROFILES[axis]
        motor = self.boards.axis(axis)
        motor.set_max_speed(profile.speed)
        trace.add("set_max_speed", axis, profile.speed)
        motor.set_max_acc(profile.acc)
        trace.add("set_max_acc", axis, profile.acc)
        motor.set_max_current(profile.run_current)
        trace.add("set_run_current", axis, profile.run_current)
        motor.set_standby_current(profile.standby_current)
        trace.add("set_standby_current", axis, profile.standby_current)
        motor.set_stall_guard_threshold(profile.stall_guard)
        trace.add("set_stall_guard", axis, profile.stall_guard)
        if profile.disable_right is not None or profile.disable_left is not None:
            motor.set_limits(disable_right=profile.disable_right, disable_left=profile.disable_left)
            trace.add("set_limits", axis, {"disable_right": profile.disable_right, "disable_left": profile.disable_left})
        if profile.rdiv is not None:
            motor.set_rdiv(profile.rdiv)
            trace.add("set_rdiv", axis, profile.rdiv)
        if profile.pdiv is not None:
            motor.set_pdiv(profile.pdiv)
            trace.add("set_pdiv", axis, profile.pdiv)

    def initialize_motors(self) -> OperationTrace:
        trace = OperationTrace("initialize_motors")
        trace.operations.extend(self.initialize_motors_without_motion().operations)
        trace.operations.extend(self.startup_homing().operations)
        return trace

    def home_axis(self, axis: str, *, startup: bool = False) -> OperationTrace:
        key = "g" if str(axis).lower() == "gripper" else str(axis).lower()
        trace = OperationTrace(f"home_{key}")
        speed = self._home_speed(key, startup=startup)
        motor = self.boards.axis(key)
        motor.move_home(speed=speed)
        trace.add("home_axis", key, speed, startup=bool(startup))
        motor.stop()
        trace.add("stop", key)
        motor.set_home()
        trace.add("set_home", key, 0)
        return trace

    @staticmethod
    def _home_speed(axis: str, *, startup: bool = False) -> int:
        # Startup X uses OEM search speed 250; diagnostic X/Y home buttons use 500.
        if axis == "x":
            return 250 if startup else 500
        if axis == "y":
            return 500
        if axis == "z":
            return 1791
        if axis == "g":
            return 200
        if axis == "door":
            return 600
        return AXIS_PROFILES[axis].speed

    def startup_homing(self) -> OperationTrace:
        trace = OperationTrace("startup_homing")
        g = self.boards.axis("g")
        g.move_relative(10000)
        trace.add("move_relative", "g", 10000, reason="OEM gripper pre-home clearance")

        for axis in ("z", "g", "x"):
            trace.operations.extend(self.home_axis(axis, startup=True).operations)

        x = self.boards.axis("x")
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
            profile = AXIS_PROFILES[axis]
            self.boards.axis(axis).set_limits(disable_right=profile.disable_right, disable_left=profile.disable_left)
            trace.add("set_limits", axis, {"disable_right": profile.disable_right, "disable_left": profile.disable_left})
        return trace

    def force_abort_motion(self) -> OperationTrace:
        trace = OperationTrace("force_abort_motion")
        for axis in ("x", "y", "z", "g", "door"):
            self.boards.axis(axis).stop()
            trace.add("stop", axis)
        return trace
