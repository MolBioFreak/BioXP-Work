from src.bioxp.usb_driver import BioXpTester


class FakeHomeXYTester(BioXpTester):
    BOARD_HEAD = 0x04
    BOARD_DECK = 0x05
    BOARD_THERMAL = 0x06
    MOTOR_SWITCH_ACTIVE_VALUE = 1

    def __init__(self):
        self.positions = {("x", 0x05, 0): 1253, ("y", 0x04, 0): 0}
        self.calls = []

    def _axis_for(self, board):
        return "x" if board == 0x05 else "y"

    def motor_set_axis_param(self, board, param, value, motor=0):
        self.calls.append(("sap", board, motor, param, value))
        return {"ok": True, "param": param, "value": value}

    def motor_oem_go_home(self, axis, **kwargs):
        self.calls.append(("go_home", axis, kwargs))
        return {"ok": True, "axis": axis}

    def motor_axis_status(self, board, motor=0):
        axis = self._axis_for(board)
        return {
            "position": {"position": self.positions[(axis, board, motor)]},
            "speed": {"speed": 0},
            "switches": {"left_raw_active": True, "left_state": 1, "right_raw_active": True, "right_state": 1},
        }

    def motor_set_home(self, board, motor=0):
        axis = self._axis_for(board)
        self.calls.append(("set_home", axis, board, motor))
        self.positions[(axis, board, motor)] = 0 if axis == "y" else -1
        return {"ok": True, "readback": {"value": self.positions[(axis, board, motor)]}}

    def motor_get_position(self, board, motor=0):
        axis = self._axis_for(board)
        return {"position": self.positions[(axis, board, motor)]}


def test_oem_home_xy_sets_home_after_switch_confirmed():
    tester = FakeHomeXYTester()

    result = tester.motor_oem_home_xy(timeout_s=30)

    assert result["ok"] is True
    assert result["home_rebase"]["x"]["status_before_rebase"]["position"]["position"] == 1253
    assert result["home_rebase"]["x"]["home_switch_confirmed"] is True
    assert result["home_rebase"]["x"]["home_rebased"] is True
    assert result["home_rebase"]["x"]["position_after_set_home"]["position"] == -1
    assert result["home_rebase"]["y"]["home_rebased"] is True
    assert ("set_home", "x", 0x05, 0) in tester.calls
    assert ("set_home", "y", 0x04, 0) in tester.calls
