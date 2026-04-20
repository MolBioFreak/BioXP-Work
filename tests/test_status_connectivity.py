import importlib
import sys
import types


class FakeStatusTester:
    BOARD_DECK = 0x05

    def __init__(self, board_status, *, chiller_activate=None, chiller_firmware=None):
        self._board_status = dict(board_status)
        self._chiller_activate = {"ack": None, "ok": False} if chiller_activate is None else chiller_activate
        self._chiller_firmware = {"ack": None, "ok": False, "fw_hex": None} if chiller_firmware is None else chiller_firmware
        self.activate_boards_calls = 0
        self.reconnect_calls = 0
        self.io_snapshot_calls = 0

    def activate_boards(self, expect_reply: bool = True):
        self.activate_boards_calls += 1
        return dict(self._board_status)

    def reconnect(self, *args, **kwargs):
        self.reconnect_calls += 1
        return {"ok": True}

    def io_snapshot(self, board_id):
        self.io_snapshot_calls += 1
        return {0: 0, 1: 1, 2: 1, 3: 1, "board_id": board_id}

    def chiller_activate(self):
        return dict(self._chiller_activate)

    def chiller_query_firmware(self):
        return dict(self._chiller_firmware)

    def motor_query_24v_sensor(self):
        return {"raw": 0, "no24v": False}

    def motion_arm_state(self):
        return {"armed": True, "reason": "strict_init_pass"}

    def motion_latch_override_state(self):
        return {"enabled": False, "reason": "startup"}



def load_api(monkeypatch):
    usb_pkg = types.ModuleType("usb")
    usb_core = types.ModuleType("usb.core")
    usb_util = types.ModuleType("usb.util")
    usb_pkg.core = usb_core
    usb_pkg.util = usb_util
    monkeypatch.setitem(sys.modules, "usb", usb_pkg)
    monkeypatch.setitem(sys.modules, "usb.core", usb_core)
    monkeypatch.setitem(sys.modules, "usb.util", usb_util)
    for name in ["src.bioxp.api", "src.bioxp.usb_driver", "src.bioxp"]:
        sys.modules.pop(name, None)
    return importlib.import_module("src.bioxp.api")



def test_status_payload_does_not_reconnect_when_board_wake_is_all_none(monkeypatch):
    api = load_api(monkeypatch)
    tester = FakeStatusTester({4: None, 5: None, 6: None, 7: None})
    monkeypatch.setattr(api, "_tester", tester)

    result = api._status_payload()

    assert result["hardware_connected"] is False
    assert tester.reconnect_calls == 0
    assert tester.io_snapshot_calls == 0



def test_motion_power_status_payload_does_not_reconnect_when_board_wake_is_all_none(monkeypatch):
    api = load_api(monkeypatch)
    tester = FakeStatusTester({4: None, 5: None, 6: None, 7: None})

    result = api._motion_power_status_payload(tester)

    assert result["hardware_connected"] is False
    assert tester.reconnect_calls == 0
    assert tester.io_snapshot_calls == 0



def test_status_payload_passively_retries_one_extra_board_wake_before_reporting_degraded(monkeypatch):
    api = load_api(monkeypatch)

    class SequencedStatusTester(FakeStatusTester):
        def __init__(self):
            super().__init__({4: None, 5: None, 6: None, 7: None})
            self._board_statuses = [
                {4: None, 5: None, 6: None, 7: None},
                {4: {"status": 100}, 5: {"status": 100}, 6: {"status": 100}, 7: None},
            ]

        def activate_boards(self, expect_reply: bool = True):
            self.activate_boards_calls += 1
            idx = min(self.activate_boards_calls - 1, len(self._board_statuses) - 1)
            return dict(self._board_statuses[idx])

    tester = SequencedStatusTester()
    monkeypatch.setattr(api, "_tester", tester)

    result = api._status_payload()

    assert result["hardware_connected"] is True
    assert tester.activate_boards_calls == 2
    assert tester.reconnect_calls == 0
    assert tester.io_snapshot_calls == 1



def test_motion_power_status_payload_passively_retries_one_extra_board_wake_before_reporting_degraded(monkeypatch):
    api = load_api(monkeypatch)

    class SequencedStatusTester(FakeStatusTester):
        def __init__(self):
            super().__init__({4: None, 5: None, 6: None, 7: None})
            self._board_statuses = [
                {4: None, 5: None, 6: None, 7: None},
                {4: {"status": 100}, 5: {"status": 100}, 6: {"status": 100}, 7: None},
            ]

        def activate_boards(self, expect_reply: bool = True):
            self.activate_boards_calls += 1
            idx = min(self.activate_boards_calls - 1, len(self._board_statuses) - 1)
            return dict(self._board_statuses[idx])

    tester = SequencedStatusTester()

    result = api._motion_power_status_payload(tester)

    assert result["hardware_connected"] is True
    assert tester.activate_boards_calls == 2
    assert tester.reconnect_calls == 0
    assert tester.io_snapshot_calls == 1



def test_status_payload_reports_dedicated_chiller_probe_when_board7_wake_is_missing(monkeypatch):
    api = load_api(monkeypatch)
    tester = FakeStatusTester(
        {4: {"status": 100}, 5: {"status": 100}, 6: {"status": 100}, 7: None},
        chiller_activate={"ack": {"status": 2}, "ok": True},
        chiller_firmware={"ack": {"status": 100}, "ok": True, "fw_hex": "01-1C-90-A8"},
    )
    monkeypatch.setattr(api, "_tester", tester)

    result = api._status_payload()

    assert result["board_status"][7] is None
    assert result["chiller_status"]["alive"] is True
    assert result["chiller_status"]["activate"]["ok"] is True
    assert result["chiller_status"]["firmware"]["fw_hex"] == "01-1C-90-A8"



def test_status_payload_reports_dedicated_chiller_probe_when_all_generic_wakes_are_missing(monkeypatch):
    api = load_api(monkeypatch)
    tester = FakeStatusTester(
        {4: None, 5: None, 6: None, 7: None},
        chiller_activate={"ack": {"status": 2}, "ok": True},
        chiller_firmware={"ack": {"status": 100}, "ok": True, "fw_hex": "01-1C-90-A8"},
    )
    monkeypatch.setattr(api, "_tester", tester)

    result = api._status_payload()

    assert tester.reconnect_calls == 0
    assert result["chiller_status"]["alive"] is True
    assert result["chiller_status"]["firmware"]["fw_hex"] == "01-1C-90-A8"



def test_motion_power_status_payload_reports_dedicated_chiller_probe_when_board7_wake_is_missing(monkeypatch):
    api = load_api(monkeypatch)
    tester = FakeStatusTester(
        {4: {"status": 100}, 5: {"status": 100}, 6: {"status": 100}, 7: None},
        chiller_activate={"ack": {"status": 2}, "ok": True},
        chiller_firmware={"ack": {"status": 100}, "ok": True, "fw_hex": "01-1C-90-A8"},
    )

    result = api._motion_power_status_payload(tester)

    assert result["board_status"][7] is None
    assert result["chiller_status"]["alive"] is True
    assert result["chiller_status"]["firmware"]["fw_hex"] == "01-1C-90-A8"
