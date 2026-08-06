import importlib
import sys
import types


class ForbiddenPassiveTester:
    """Records any hidden status collection or connection mutation."""

    def __init__(self):
        self.calls = []

    def activate_boards(self, *args, **kwargs):
        self.calls.append(("activate_boards", args, kwargs))
        raise AssertionError("passive status must not activate boards")

    def reconnect(self, *args, **kwargs):
        self.calls.append(("reconnect", args, kwargs))
        raise AssertionError("passive status must not reconnect")

    def io_snapshot(self, *args, **kwargs):
        self.calls.append(("io_snapshot", args, kwargs))
        raise AssertionError("passive status must not collect deck I/O")

    def chiller_activate(self, *args, **kwargs):
        self.calls.append(("chiller_activate", args, kwargs))
        raise AssertionError("passive status must not activate the chiller")

    def chiller_query_firmware(self, *args, **kwargs):
        self.calls.append(("chiller_query_firmware", args, kwargs))
        raise AssertionError("passive status must not query the chiller")

    def motor_query_24v_sensor(self, *args, **kwargs):
        self.calls.append(("motor_query_24v_sensor", args, kwargs))
        raise AssertionError("passive status must not query power")

    def motion_arm_state(self, *args, **kwargs):
        self.calls.append(("motion_arm_state", args, kwargs))
        raise AssertionError("passive status must not collect arm state")

    def motion_latch_override_state(self, *args, **kwargs):
        self.calls.append(("motion_latch_override_state", args, kwargs))
        raise AssertionError("passive status must not collect override state")


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


def new_owner(*, fresh_for_s=30.0):
    from src.bioxp.hardware_status import HardwareStateOwner

    return HardwareStateOwner(fresh_for_s=fresh_for_s)


def canonical_owner(observations):
    owner = new_owner()
    owner.change_ownership(reason="test_owned", transport="owned", usb="service", router="running")
    result = owner.collect(
        observations,
        {domain: (lambda _context, value=value: value) for domain, value in observations.items()},
    )
    assert result["published"] is True
    return owner


def status_observations(*, boards=None, chiller=None):
    return {
        "transport": {"runtime_available": True, "CAN_READY": True},
        "boards": {4: None, 5: None, 6: None, 7: None} if boards is None else boards,
        "latch": {"snapshot": {0: 0, 1: 1, 2: 1, 3: 1}},
        "chiller": {"alive": False} if chiller is None else chiller,
    }


def motion_power_observations(*, boards=None, chiller=None):
    return {
        **status_observations(boards=boards, chiller=chiller),
        "power": {"rail_24v": {"safety_valid": True, "payload_raw": 0}},
        "interlock": {
            "motion_arm": {"armed": True, "reason": "strict_init_pass"},
            "latch_override": {"enabled": False, "reason": "startup"},
        },
    }


def test_status_payload_missing_snapshot_is_unknown_without_hidden_collection(monkeypatch):
    api = load_api(monkeypatch)
    tester = ForbiddenPassiveTester()
    monkeypatch.setattr(api, "hardware_state", new_owner())
    monkeypatch.setattr(api, "_tester", tester)

    result = api._status_payload()

    assert result["hardware_connected"] is None
    assert result["available"] is False
    assert result["cache_state"] == "missing"
    assert result["board_status"] is None
    assert result["chiller_status"] is None
    assert tester.calls == []


def test_status_payload_reports_managed_runtime_after_reclaim_without_collecting(monkeypatch):
    api = load_api(monkeypatch)
    tester = ForbiddenPassiveTester()
    owner = new_owner()
    owner.change_ownership(
        reason="managed_reclaim",
        transport="owned",
        usb="service",
        router="running",
    )
    monkeypatch.setattr(api, "hardware_state", owner)
    monkeypatch.setattr(api, "_tester", tester)
    monkeypatch.setattr(api, "_pipette_transport", object())

    result = api._status_payload()

    assert result["runtime_available"] is True
    assert result["hardware_connected"] is None
    assert result["available"] is False
    assert result["cache_state"] == "missing"
    assert tester.calls == []


def test_motion_power_status_missing_snapshot_is_unknown_without_hidden_collection(monkeypatch):
    api = load_api(monkeypatch)
    tester = ForbiddenPassiveTester()
    monkeypatch.setattr(api, "hardware_state", new_owner())

    result = api._motion_power_status_payload(tester)

    assert result["hardware_connected"] is None
    assert result["available"] is False
    assert result["cache_state"] == "missing"
    assert result["board_status"] is None
    assert result["chiller_status"] is None
    assert tester.calls == []


def test_status_payload_projects_one_completed_snapshot_without_retry(monkeypatch):
    api = load_api(monkeypatch)
    tester = ForbiddenPassiveTester()
    observations = status_observations(
        boards={4: {"ack": {"status": 100}}, 5: {"ack": {"status": 100}}, 6: {"ack": {"status": 100}}, 7: None}
    )
    monkeypatch.setattr(api, "hardware_state", canonical_owner(observations))
    monkeypatch.setattr(api, "_tester", tester)

    result = api._status_payload()

    assert result["hardware_connected"] is True
    assert result["board_status"] == observations["boards"]
    assert result["snapshot_id"] is not None
    assert tester.calls == []


def test_motion_power_status_projects_one_completed_snapshot_without_retry(monkeypatch):
    api = load_api(monkeypatch)
    tester = ForbiddenPassiveTester()
    observations = motion_power_observations(
        boards={4: {"ack": {"status": 100}}, 5: {"ack": {"status": 100}}, 6: {"ack": {"status": 100}}, 7: None}
    )
    monkeypatch.setattr(api, "hardware_state", canonical_owner(observations))

    result = api._motion_power_status_payload(tester)

    assert result["hardware_connected"] is True
    assert result["board_status"] == observations["boards"]
    assert result["rail_24v"] == observations["power"]["rail_24v"]
    assert result["motion_arm"] == observations["interlock"]["motion_arm"]
    assert tester.calls == []


def test_status_payload_projects_prior_chiller_evidence_without_probing(monkeypatch):
    api = load_api(monkeypatch)
    tester = ForbiddenPassiveTester()
    chiller = {"alive": True, "firmware": {"fw_hex": "01-1C-90-A8"}}
    observations = status_observations(chiller=chiller)
    monkeypatch.setattr(api, "hardware_state", canonical_owner(observations))
    monkeypatch.setattr(api, "_tester", tester)

    result = api._status_payload()

    assert result["chiller_status"] == chiller
    assert tester.calls == []


def test_status_payload_keeps_completed_chiller_evidence_separate_from_board_evidence(monkeypatch):
    api = load_api(monkeypatch)
    tester = ForbiddenPassiveTester()
    chiller = {"alive": True, "firmware": {"fw_hex": "01-1C-90-A8"}}
    observations = status_observations(
        boards={4: None, 5: None, 6: None, 7: None},
        chiller=chiller,
    )
    monkeypatch.setattr(api, "hardware_state", canonical_owner(observations))
    monkeypatch.setattr(api, "_tester", tester)

    result = api._status_payload()

    assert result["board_status"] == observations["boards"]
    assert result["chiller_status"] == chiller
    assert tester.calls == []


def test_motion_power_status_projects_prior_chiller_evidence_without_probing(monkeypatch):
    api = load_api(monkeypatch)
    tester = ForbiddenPassiveTester()
    chiller = {"alive": True, "firmware": {"fw_hex": "01-1C-90-A8"}}
    observations = motion_power_observations(chiller=chiller)
    monkeypatch.setattr(api, "hardware_state", canonical_owner(observations))

    result = api._motion_power_status_payload(tester)

    assert result["chiller_status"] == chiller
    assert tester.calls == []
