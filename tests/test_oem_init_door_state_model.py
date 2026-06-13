from src.bioxp.oem_initialization import classify_thermal_door_state, build_thermal_door_state_restore_plan
from src.bioxp.usb_driver import BioXpTester


def test_classifies_closed_door_from_oem_predicates():
    classified = classify_thermal_door_state({"oem_predicates": {"tcDoorClosed": True, "tcDoorOpened": False}})
    assert classified["state"] == "closed"
    assert classified["safe"] is True
    assert classified["closed_source"] == "queryHome(ThermalDoor)"


def test_classifies_open_door_from_raw_right_predicate():
    classified = classify_thermal_door_state({"oem_predicates": {"tcDoorClosed": False, "tcDoorOpened": True}})
    assert classified["state"] == "open"
    assert classified["safe"] is True


def test_ambiguous_door_state_fails_closed_restore_plan():
    plan = build_thermal_door_state_restore_plan({"oem_predicates": {"tcDoorClosed": True, "tcDoorOpened": True}})
    assert plan["ok"] is False
    assert plan["recommended_action"] == "fail_closed"
    assert plan["not_silent"] is True


def test_open_door_restore_is_explicit_not_silent_auto_restore():
    plan = build_thermal_door_state_restore_plan({"oem_predicates": {"tcDoorClosed": False, "tcDoorOpened": True}})
    assert plan["ok"] is True
    assert plan["implemented"] is False
    assert plan["recommended_action"] == "explicit_open_restore_required_after_init"


def test_driver_capture_uses_thermal_door_status_predicates():
    class Tester(BioXpTester):
        def motor_thermal_door_status(self):
            return {"oem_predicates": {"tcDoorClosed": True, "tcDoorOpened": False}, "position": {"position": 0}}

    tester = Tester.__new__(Tester)
    result = tester.motor_plan_thermal_door_restore(restore_requested=False)

    assert result["capture"]["classified"]["state"] == "closed"
    assert result["restore_plan"]["recommended_action"] == "leave_closed_after_initialization"
