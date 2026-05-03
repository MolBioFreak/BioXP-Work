def test_bioxp_startup_hardware_initial_check_shadow_fallback_queries_only():
    from src.bioxp.oem_startup_program import BioXpStartupHardware

    calls = []

    class Tester:
        BOARD_DECK = 5

        def io_snapshot(self, board):
            calls.append(("io_snapshot", board))
            return {1: 1, 3: 1}

        def strip_set_rgb(self, *args):
            calls.append(("led", args))
            return {"ok": True}

        def deactivate_boards(self):
            calls.append(("deactivate",))
            return {"ok": True}

        def activate_boards(self):
            calls.append(("activate",))
            return {"ok": True}

    hw = BioXpStartupHardware(lambda: Tester())
    result = hw.initial_check(mode="shadow")
    assert result["ok"] is True
    assert result["sequence"] == ["backend_ready", "door_latch_before", "door_latch_final"]
    assert [c[0] for c in calls] == ["io_snapshot"]


def test_bioxp_startup_hardware_initial_check_live_fallback_matches_oem_order():
    from src.bioxp.oem_startup_program import BioXpStartupHardware

    calls = []

    class Tester:
        BOARD_DECK = 5

        def io_snapshot(self, board):
            calls.append(("io_snapshot", board))
            return {1: 1, 3: 1}

        def strip_set_rgb(self, *args):
            calls.append(("led", args))
            return {"ok": True}

        def deactivate_boards(self):
            calls.append(("deactivate",))
            return {"ok": True}

        def activate_boards(self):
            calls.append(("activate",))
            return {"ok": True}

    hw = BioXpStartupHardware(lambda: Tester())
    result = hw.initial_check(mode="live")
    assert result["ok"] is True
    assert result["sequence"] == ["backend_ready", "led_white", "door_latch_before", "deactivate_boards", "activate_boards", "door_latch_final"]
    assert [c[0] for c in calls] == ["led", "io_snapshot", "deactivate", "activate", "io_snapshot"]


def test_bioxp_startup_hardware_initial_check_live_fails_if_board_cycle_unavailable():
    from src.bioxp.oem_startup_program import BioXpStartupHardware

    class Tester:
        BOARD_DECK = 5

        def io_snapshot(self, board):
            return {1: 1, 3: 1}

        def strip_set_rgb(self, *args):
            return {"ok": True}

        def activate_boards(self):
            return {"ok": True}

    result = BioXpStartupHardware(lambda: Tester()).initial_check(mode="live")
    assert result["ok"] is False
    assert result["deactivate_boards"]["error"] == "deactivate_boards_unavailable"
