import threading

from src.bioxp.usb_driver import BioXpTester
from support_oem_machine_bundle import serial_206_immutable_machine_bundle


class FakeHomeXYTester(BioXpTester):
    BOARD_HEAD = 0x04
    BOARD_DECK = 0x05
    BOARD_THERMAL = 0x06
    MOTOR_SWITCH_ACTIVE_VALUE = 1

    def __init__(self):
        self.positions = {("x", 0x05, 0): 1253, ("y", 0x04, 0): 0}
        self.calls = []

    def _machine_config_bundle(self):
        return serial_206_immutable_machine_bundle()

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



def test_oem_homexy_starts_both_axis_tasks_before_either_completes():
    calls = []
    calls_lock = threading.Lock()
    both_started = threading.Event()

    class Tester(BioXpTester):
        def __init__(self):
            pass

        def _motion_oem_axis_profile(self, axis, startup=True):
            return {"x": {"board": 5, "motor": 0, "speed": 1700, "acc": 350}, "y": {"board": 4, "motor": 0, "speed": 1800, "acc": 400}}[axis]

        def motor_set_axis_param(self, board, param, value, motor=0):
            return {"ok": True, "board": board, "param": param, "value": value}

        def motor_oem_go_home(self, axis, **kwargs):
            assert kwargs["require_switch_transition"] is True
            with calls_lock:
                calls.append(("started", axis))
                if len([row for row in calls if row[0] == "started"]) == 2:
                    both_started.set()
            assert both_started.wait(timeout=0.5), "other HomeXY task did not start concurrently"
            with calls_lock:
                calls.append(("finished", axis))
            return {"ok": True, "axis": axis}

        def motor_axis_status(self, board, motor=0):
            return {"speed": {"speed": 0}, "switches": {"left_raw_active": True}}

        def motor_set_home(self, board, motor=0):
            return {"ok": True}

        def motor_get_position(self, board, motor=0):
            return {"position": 0}

    result = Tester().motor_oem_home_xy(timeout_s=1)

    assert result["ok"] is True
    assert result["live_parallel_execution"] is True
    assert result["implementation_note"] == "oem_task_run_waitall_with_transaction_serialized_usb"
    first_finish = next(index for index, row in enumerate(calls) if row[0] == "finished")
    assert {row[1] for row in calls[:first_finish] if row[0] == "started"} == {"x", "y"}
