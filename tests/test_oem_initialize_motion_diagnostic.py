from src.bioxp.oem_startup_program import BioXpStartupHardware


class MotionDiagTester:
    BOARD_HEAD = 4
    BOARD_DECK = 5
    BOARD_THERMAL = 6

    def __init__(self):
        self.calls = []

    def motor_get_position(self, board, motor=0):
        self.calls.append(("position", board, motor))
        return {"ok": True, "position": 1000 + board * 10 + motor}

    def motor_get_speed(self, board, motor=0):
        self.calls.append(("speed", board, motor))
        return {"ok": True, "speed": 0}

    def motor_get_switch_activity(self, board, motor=0):
        self.calls.append(("switch", board, motor))
        return {"left_state": 0, "right_state": 1, "left_active": False, "right_active": True}

    def motor_query_24v_sensor(self):
        self.calls.append(("rail",))
        return {"raw": 0, "no24v": False}

    def io_snapshot(self, board):
        self.calls.append(("io", board))
        return {1: 1, 3: 1}


def test_initialize_motion_diagnostic_is_passive_axis_snapshot():
    tester = MotionDiagTester()
    result = BioXpStartupHardware(lambda: tester).initialize_motion_diagnostic(mode="live", run_homing=False)
    assert result["ok"] is True
    assert result["physical_motion"] is False
    assert result["run_homing"] is False
    assert sorted(result["axis_snapshots"].keys()) == ["door", "g", "x", "y", "z"]
    assert result["axis_snapshots"]["z"]["board"] == 4
    assert result["axis_snapshots"]["z"]["motor"] == 1
    assert result["rail_24v"]["no24v"] is False
    assert result["latch"]["door_closed"] is True
    assert not any(call[0].startswith("home") or call[0].startswith("move") for call in tester.calls)


def test_initialize_motion_diagnostic_rejects_homing():
    result = BioXpStartupHardware(lambda: MotionDiagTester()).initialize_motion_diagnostic(mode="live", run_homing=True)
    assert result["ok"] is False
    assert result["physical_motion"] is False
    assert result["error"] == "run_homing_not_allowed_in_diagnostic"
