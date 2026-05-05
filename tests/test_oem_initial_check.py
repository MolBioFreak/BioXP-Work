def test_oem_initial_check_shadow_queries_only_no_writes():
    from src.bioxp.usb_driver import BioXpTester

    calls = []
    tester = BioXpTester.__new__(BioXpTester)
    tester.BOARD_DECK = 5
    tester.io_snapshot = lambda board: calls.append(("io_snapshot", board)) or {1: 1, 3: 1, "no24v": False}
    tester.strip_set_rgb = lambda *args, **kwargs: calls.append(("led", args)) or {"ok": True}
    tester.deactivate_boards = lambda **kwargs: calls.append(("deactivate", kwargs)) or {"ok": True}
    tester.activate_boards = lambda **kwargs: calls.append(("activate", kwargs)) or {"ok": True}

    result = tester.oem_initial_check(mode="shadow")

    assert result["ok"] is True
    assert result["sequence"] == ["backend_ready", "door_latch_before", "door_latch_final"]
    assert [c[0] for c in calls] == ["io_snapshot", "io_snapshot"]
    assert result["led_white"]["skipped"] is True


def test_oem_initial_check_live_order_includes_led_board_cycle_and_final_snapshot():
    from src.bioxp.usb_driver import BioXpTester

    calls = []
    tester = BioXpTester.__new__(BioXpTester)
    tester.BOARD_DECK = 5
    tester.io_snapshot = lambda board: calls.append(("io_snapshot", board)) or {1: 1, 3: 1, "no24v": False}
    tester.strip_set_rgb = lambda *args, **kwargs: calls.append(("led", args)) or {"ok": True}
    tester.deactivate_boards = lambda **kwargs: calls.append(("deactivate", kwargs)) or {"ok": True}
    tester.activate_boards = lambda **kwargs: calls.append(("activate", kwargs)) or {"ok": True}

    result = tester.oem_initial_check(mode="live")

    assert result["ok"] is True
    assert result["sequence"] == ["backend_ready", "led_white", "door_latch_before", "deactivate_boards", "activate_boards", "door_latch_final"]
    assert [c[0] for c in calls] == ["led", "io_snapshot", "deactivate", "activate", "io_snapshot"]
