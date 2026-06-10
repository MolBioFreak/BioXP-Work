from __future__ import annotations

from dataclasses import dataclass

from .frames import OemCommandFrame, OemReplyFrame
from .transport import DryRunTransport

CMD_ROR = 1
CMD_ROL = 2
CMD_MST = 3
CMD_MVP = 4
CMD_SAP = 5
CMD_GAP = 6
CMD_RFS = 13
CMD_QUERY_STOP = 138

MVP_ABS = 0
MVP_REL = 1
RFS_SEARCH_HOME = 0
RFS_SET_HOME = 2

PARAM_ACTUAL_POSITION = 1
PARAM_ACTUAL_SPEED = 3
PARAM_MAX_SPEED = 4
PARAM_MAX_ACC = 5
PARAM_RUN_CURRENT = 6
PARAM_STANDBY_CURRENT = 7
PARAM_TARGET_REACHED = 8
PARAM_LEFT_SWITCH = 9
PARAM_RIGHT_SWITCH = 10
PARAM_DISABLE_RIGHT_SWITCH = 12
PARAM_DISABLE_LEFT_SWITCH = 13
PARAM_RAMP_MODE = 138
PARAM_RDIV = 153
PARAM_PDIV = 154
PARAM_STALL_GUARD_THRESHOLD = 205


def _clamp_current(current: int) -> int:
    return max(0, min(31, int(current)))


@dataclass
class Motor:
    board_id: int
    motor: int
    transport: DryRunTransport
    label: str = ""

    def _send(self, command: int, cmd_type: int, value: int, message: str, *, timeout_ms: int = 60000) -> OemReplyFrame:
        frame = OemCommandFrame.tmcl(
            self.board_id,
            command,
            cmd_type,
            self.motor,
            value,
            message=message,
            timeout_ms=timeout_ms,
        )
        return self.transport.transmit(frame)

    def move_right(self, speed: int = 200) -> OemReplyFrame:
        return self._send(CMD_ROR, 0, speed, f"{self.label}.ROR({speed})")

    def move_left(self, speed: int = 200) -> OemReplyFrame:
        return self._send(CMD_ROL, 0, speed, f"{self.label}.ROL({speed})")

    def set_axis_param(self, param: int, value: int) -> OemReplyFrame:
        return self._send(CMD_SAP, int(param), int(value), f"{self.label}.SAP{param}={value}")

    def get_axis_param(self, param: int) -> OemReplyFrame:
        return self._send(CMD_GAP, int(param), 0, f"{self.label}.GAP{param}")

    def set_max_speed(self, speed: int) -> OemReplyFrame:
        return self.set_axis_param(PARAM_MAX_SPEED, speed)

    def set_max_acc(self, acc: int) -> OemReplyFrame:
        return self.set_axis_param(PARAM_MAX_ACC, acc)

    def set_max_current(self, current: int) -> OemReplyFrame:
        return self.set_axis_param(PARAM_RUN_CURRENT, _clamp_current(current))

    def set_standby_current(self, current: int) -> OemReplyFrame:
        return self.set_axis_param(PARAM_STANDBY_CURRENT, _clamp_current(current))

    def read_max_current(self) -> OemReplyFrame:
        return self.get_axis_param(PARAM_RUN_CURRENT)

    def set_stall_guard_threshold(self, threshold: int) -> OemReplyFrame:
        return self.set_axis_param(PARAM_STALL_GUARD_THRESHOLD, threshold)

    def disable_right_limit_switch(self) -> list[OemReplyFrame]:
        # OEM ClassMotor sends this twice; first with short timeout.
        return [
            self._send(CMD_SAP, PARAM_DISABLE_RIGHT_SWITCH, 1, f"{self.label}.disable_right", timeout_ms=1000),
            self._send(CMD_SAP, PARAM_DISABLE_RIGHT_SWITCH, 1, f"{self.label}.disable_right"),
        ]

    def disable_left_limit_switch(self) -> list[OemReplyFrame]:
        return [
            self._send(CMD_SAP, PARAM_DISABLE_LEFT_SWITCH, 1, f"{self.label}.disable_left", timeout_ms=1000),
            self._send(CMD_SAP, PARAM_DISABLE_LEFT_SWITCH, 1, f"{self.label}.disable_left"),
        ]

    def set_limits(self, *, disable_right: bool | None = None, disable_left: bool | None = None) -> list[OemReplyFrame]:
        replies: list[OemReplyFrame] = []
        if disable_right:
            replies.extend(self.disable_right_limit_switch())
        if disable_left:
            replies.extend(self.disable_left_limit_switch())
        return replies

    def set_rdiv(self, value: int) -> OemReplyFrame:
        return self.set_axis_param(PARAM_RDIV, value)

    def set_pdiv(self, value: int) -> OemReplyFrame:
        return self.set_axis_param(PARAM_PDIV, value)

    def set_ramp_mode(self, value: int) -> OemReplyFrame:
        return self.set_axis_param(PARAM_RAMP_MODE, value)

    def move_relative(self, steps: int) -> OemReplyFrame:
        return self._send(CMD_MVP, MVP_REL, int(steps), f"{self.label}.MVP_REL({steps})")

    def move_absolute(self, position: int) -> OemReplyFrame:
        return self._send(CMD_MVP, MVP_ABS, max(0, int(position)), f"{self.label}.MVP_ABS({position})")

    def move_home(self, speed: int | None = None) -> OemReplyFrame:
        # Board axisSearchHome uses moveLeft after setting speed; ClassMotor.MoveHome is MVP ABS 0.
        if speed is not None:
            self.set_max_speed(int(speed))
        return self.move_left(speed or 200)

    def move_to_home_position(self) -> OemReplyFrame:
        return self._send(CMD_MVP, MVP_ABS, 0, f"{self.label}.MoveHome")

    def set_home(self) -> OemReplyFrame:
        # OEM ClassMotor.setHome payload is SAP param 1, not RFS.
        return self._send(CMD_SAP, 1, 0, f"{self.label}.SET_HOME")

    def stop(self) -> list[OemReplyFrame]:
        # OEM StopMotor sends twice and checks the second response.
        return [self._send(CMD_MST, 0, 0, f"{self.label}.MST"), self._send(CMD_MST, 0, 0, f"{self.label}.MST")]

    def query_actual_position(self) -> OemReplyFrame:
        return self.get_axis_param(PARAM_ACTUAL_POSITION)

    def query_motor_speed(self) -> OemReplyFrame:
        return self.get_axis_param(PARAM_ACTUAL_SPEED)

    def query_reached_position(self) -> OemReplyFrame:
        return self.get_axis_param(PARAM_TARGET_REACHED)

    def query_left_switch_status(self) -> OemReplyFrame:
        return self.get_axis_param(PARAM_LEFT_SWITCH)

    def query_right_switch_status(self) -> OemReplyFrame:
        return self.get_axis_param(PARAM_RIGHT_SWITCH)

    def query_motor_stop(self, axis: int = -1) -> OemReplyFrame:
        value = 1 << self.motor if axis == -1 else 0
        motor = self.motor if axis == -1 else int(axis)
        return self._send(CMD_QUERY_STOP, 0, value, f"{self.label}.QUERY_STOP(axis={axis})")
