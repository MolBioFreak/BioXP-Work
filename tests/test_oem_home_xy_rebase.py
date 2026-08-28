import threading

import pytest

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
        return {
            "ok": True,
            "axis": axis,
            "controller_terminal_state_verified": True,
            "controller_home_proof_verified": True,
        }

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


def test_oem_home_xy_uses_the_source_shaped_home_kernel_without_outer_rebase():
    tester = FakeHomeXYTester()

    result = tester.motor_oem_home_xy(timeout_s=30)

    assert result["ok"] is True
    assert "home_rebase" not in result
    assert not any(call[0] == "set_home" for call in tester.calls)
    home_calls = [call for call in tester.calls if call[0] == "go_home"]
    assert {call[1] for call in home_calls} == {"x", "y"}
    assert all(call[2]["require_switch_transition"] is False for call in home_calls)



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
            assert kwargs["require_switch_transition"] is False
            with calls_lock:
                calls.append(("started", axis))
                if len([row for row in calls if row[0] == "started"]) == 2:
                    both_started.set()
            assert both_started.wait(timeout=0.5), "other HomeXY task did not start concurrently"
            with calls_lock:
                calls.append(("finished", axis))
            return {
                "ok": True,
                "axis": axis,
                "controller_terminal_state_verified": True,
                "controller_home_proof_verified": True,
            }

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


@pytest.mark.parametrize("missing_board", [4, 5])
def test_oem_homexy_returns_source_noop_when_either_board_is_absent(missing_board):
    class MissingBoardTester(BioXpTester):
        def _motion_oem_axis_profile(self, axis_key, startup=True):
            return {
                "x": {"board": 5, "motor": 0, "speed": 1700, "acc": 350},
                "y": {"board": 4, "motor": 0, "speed": 1800, "acc": 400},
            }[axis_key]

        def _oem_board_present(self, board_id):
            return int(board_id) != missing_board

        def motor_oem_go_home(self, axis_key, **kwargs):
            raise AssertionError(f"HomeXY dispatched {axis_key} with an absent OEM board")

    result = MissingBoardTester.__new__(MissingBoardTester).motor_oem_home_xy(timeout_s=1)

    assert result["ok"] is True
    assert result["source_noop"] is True
    assert result["source_return"] is None
    assert result["command_issued"] is False
    assert result["physical_motion_commanded"] is False
