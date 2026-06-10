from src.bioxp.usb_driver import BioXpTester


class BoardCycleTester(BioXpTester):
    def __init__(self):
        self.BOARDS = [4, 5]
        self.BOARD_DECK = 5
        self.sent = []
        self.motor_power_enabled = 0

    def enable_motor_power(self):
        self.motor_power_enabled += 1

    def send_tmcl_retry(self, board_id, command, cmd_type, motor, value, **kwargs):
        self.sent.append((board_id, command, cmd_type, motor, value, kwargs))
        return {"board": board_id, "cmd": command, "value": value, "status": 100}


def test_activate_boards_uses_command_64_value_1_and_enables_motor_power():
    tester = BoardCycleTester()
    result = tester.activate_boards(expect_reply=True)
    assert tester.motor_power_enabled == 1
    assert [row[:5] for row in tester.sent] == [(4, 64, 0, 0, 1), (5, 64, 0, 0, 1)]
    assert result[4]["value"] == 1


def test_deactivate_boards_uses_command_64_value_0_without_enabling_motor_power():
    tester = BoardCycleTester()
    result = tester.deactivate_boards(expect_reply=True)
    assert tester.motor_power_enabled == 0
    assert [row[:5] for row in tester.sent] == [(4, 64, 0, 0, 0), (5, 64, 0, 0, 0)]
    assert result[5]["value"] == 0
