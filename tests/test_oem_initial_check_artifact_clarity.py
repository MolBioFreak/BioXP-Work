from src.bioxp.usb_driver import BioXpTester


class InitialCheckTester(BioXpTester):
    BOARD_DECK = 5
    BOARDS = [4, 5]

    def __init__(self):
        self.calls = []

    def io_snapshot(self, board):
        self.calls.append(("io_snapshot", board))
        return {0: 0, 1: 1, 2: 1, 3: 1}

    def strip_set_rgb(self, r, g, b):
        self.calls.append(("strip_set_rgb", r, g, b))
        return {"ok": True, "rgb": [r, g, b]}

    def deactivate_boards(self, expect_reply=True):
        self.calls.append(("deactivate_boards", expect_reply))
        return {4: {"status": 100, "value": 0}, 5: {"status": 100, "value": 0}}

    def activate_boards(self, expect_reply=True):
        self.calls.append(("activate_boards", expect_reply))
        return {4: {"status": 100, "value": 0}, 5: {"status": 100, "value": 0}}


def test_oem_initial_check_live_artifact_preserves_requested_board_cycle_values():
    tester = InitialCheckTester()
    result = tester.oem_initial_check(mode="live")
    assert result["ok"] is True
    assert result["deactivate_boards"]["requested_value"] == 0
    assert result["activate_boards"]["requested_value"] == 1
    assert result["activate_boards"]["boards"]["4"]["requested_value"] == 1
    assert result["sequence"] == [
        "backend_ready",
        "led_white",
        "door_latch_before",
        "deactivate_boards",
        "activate_boards",
        "door_latch_final",
    ]
