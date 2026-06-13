from src.bioxp.usb_driver import BioXpTester


class FakeDoorTester(BioXpTester):
    def __init__(self, *, closed=False, opened=False):
        self.closed = closed
        self.opened = opened
        self.calls = []
        self.position = 0

    def _motion_oem_axis_profile(self, axis_key, startup=False):
        assert axis_key == "door"
        return dict(BioXpTester.MOTOR_FUNCTION_PRESETS["door"])

    def motor_get_position(self, board, motor=0):
        return {"ok": True, "position": self.position, "board": board, "motor": motor}

    def motor_get_speed(self, board, motor=0):
        return {"ok": True, "speed": 0, "board": board, "motor": motor}

    def motor_query_home_switch(self, board, motor=0):
        return {"ok": True, "value": 1 if self.closed else 0, "board": board, "motor": motor}

    def motor_get_switch_activity(self, board, motor=0):
        return {
            "ok": True,
            "left_state": 1 if self.closed else 0,
            "right_state": 1 if self.opened else 0,
            "left_active": self.closed,
            "right_active": self.opened,
            "board": board,
            "motor": motor,
        }

    def motor_prepare_axis(self, board, **kwargs):
        self.calls.append(("prepare", board, kwargs))
        return {"ok": True, "board": board, **kwargs}

    def motor_move_absolute(self, board, position, motor=0):
        self.calls.append(("move_absolute", board, position, motor))
        self.position = int(position)
        if position == 16000:
            self.opened = True
            self.closed = False
        elif position == 0:
            self.closed = True
            self.opened = False
        return {"ok": True, "ack": {"ok": True, "reply": "Success"}}

    def motor_wait_stopped(self, board, motor=0, timeout_s=20.0, require_seen_nonzero=False):
        self.calls.append(("wait", board, motor, timeout_s, require_seen_nonzero))
        return {"stopped": True, "seen_nonzero": True}

    def motor_move_left(self, board, speed, motor=0):
        self.calls.append(("move_left", board, speed, motor))
        self.closed = True
        self.opened = False
        return {"ok": True, "ack": {"ok": True, "reply": "Success"}}

    def motor_move_relative(self, board, steps, motor=0):
        self.calls.append(("move_relative", board, steps, motor))
        self.closed = False
        return {"ok": True, "ack": {"ok": True, "reply": "Success"}}

    def motor_stop(self, board, motor=0):
        return {"ok": True}

    def motor_set_home(self, board, motor=0):
        self.calls.append(("set_home", board, motor))
        self.position = 0
        return {"ok": True}

    def motor_set_axis_param(self, board, param, value, motor=0):
        self.calls.append(("set_axis_param", board, param, value, motor))
        return {"ok": True, "param": param, "value": value}

    def _tmcl_success(self, ack):
        return True


class FakeDoorTesterNoOpen(FakeDoorTester):
    def motor_move_absolute(self, board, position, motor=0):
        self.calls.append(("move_absolute", board, position, motor))
        self.position = int(position)
        return {"ok": True, "ack": {"ok": True, "reply": "Success"}}


def test_thermal_door_status_maps_oem_closed_and_opened_predicates():
    tester = FakeDoorTester(closed=True, opened=False)
    status = tester.motor_thermal_door_status()

    assert status["closed"] is True
    assert status["opened"] is False
    assert status["oem_predicates"]["tcDoorClosed"] is True
    assert status["oem_predicates"]["closed_source"] == "queryHome(ThermalDoor)"


def test_open_thermal_door_uses_oem_target_and_requires_open_predicate():
    tester = FakeDoorTester(closed=True, opened=False)
    result = tester.motor_oem_open_thermal_door(timeout_s=5)

    assert result["ok"] is True
    assert result["target"] == 16000
    assert result["after"]["opened"] is True
    assert ("move_absolute", BioXpTester.BOARD_THERMAL, 16000, 0) in tester.calls
    prepare_call = next(call for call in tester.calls if call[0] == "prepare")
    assert prepare_call[2]["run_current"] == 31
    assert prepare_call[2]["speed"] == 50
    assert prepare_call[2]["acc"] == 20
    assert prepare_call[2]["stall_guard"] == 8


def test_open_thermal_door_coordinate_without_open_predicate_is_not_success():
    tester = FakeDoorTesterNoOpen(closed=True, opened=False)
    result = tester.motor_oem_open_thermal_door(timeout_s=5)

    assert result["ok"] is False
    assert result["failure"] == "door_open_predicate_not_confirmed"
    assert result["target"] == 16000


def test_close_thermal_door_only_moves_when_opened_and_requires_closed_predicate():
    tester = FakeDoorTester(closed=False, opened=True)
    result = tester.motor_oem_close_thermal_door(timeout_s=5)

    assert result["ok"] is True
    assert result["target"] == 0
    assert result["after"]["closed"] is True
    assert ("move_absolute", BioXpTester.BOARD_THERMAL, 0, 0) in tester.calls


def test_close_thermal_door_does_not_move_when_open_predicate_false():
    tester = FakeDoorTester(closed=True, opened=False)
    result = tester.motor_oem_close_thermal_door(timeout_s=5)

    assert result["ok"] is True
    assert result["skipped"] is True
    assert not any(call[0] == "move_absolute" for call in tester.calls)


def test_door_search_home_reports_before_after_predicates_and_sets_home_when_closed():
    tester = FakeDoorTester(closed=False, opened=True)
    result = tester.motor_oem_door_search_home(timeout_s=5, startup=True)

    assert result["ok"] is True
    assert result["closed_before"] is False
    assert result["opened_before"] is True
    assert result["closed_after"] is True
    assert result["opened_after"] is False
    assert any(call[0] == "set_home" for call in tester.calls)
