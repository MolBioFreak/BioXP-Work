from src.bioxp.usb_driver import BioXpTester


class BoardCycleTester(BioXpTester):
    def __init__(self):
        self.BOARDS = [4, 5]
        self.BOARD_DECK = 5
        self.sent = []

    def send_tmcl_retry(self, board_id, command, cmd_type, motor, value, **kwargs):
        self.sent.append((board_id, command, cmd_type, motor, value, kwargs))
        return {"board": board_id, "cmd": command, "value": value, "status": 100}


def test_activate_boards_uses_only_oem_command_64_value_1():
    tester = BoardCycleTester()
    result = tester.activate_boards(expect_reply=True)
    assert [row[:5] for row in tester.sent] == [(4, 64, 0, 0, 1), (5, 64, 0, 0, 1)]
    assert result[4]["value"] == 1
    assert tester._oem_no_motion_profile_ready is False


def test_deactivate_boards_uses_command_64_value_0():
    tester = BoardCycleTester()
    result = tester.deactivate_boards(expect_reply=True)
    assert [row[:5] for row in tester.sent] == [(4, 64, 0, 0, 0), (5, 64, 0, 0, 0)]
    assert result[5]["value"] == 0
    assert tester._oem_no_motion_profile_ready is False
