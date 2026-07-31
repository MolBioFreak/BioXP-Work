from types import SimpleNamespace

import pytest

from src.bioxp import oem_startup_program
from src.bioxp.oem_startup_program import BioXpStartupHardware


class _NoHardwareDoorPredicateTester:
    """Fake-only tester: records calls and never creates a hardware transport."""

    BOARD_HEAD = 4
    BOARD_DECK = 5
    BOARD_THERMAL = 6

    def __init__(self, *, closed: bool):
        self.closed = closed
        self.calls: list[tuple] = []

    def motor_get_position(self, board, motor=0):
        self.calls.append(("position", board, motor))
        return {"ok": True, "position": 0}

    def motor_get_speed(self, board, motor=0):
        self.calls.append(("speed", board, motor))
        return {"ok": True, "speed": 0}

    def motor_get_switch_activity(self, board, motor=0):
        self.calls.append(("switch", board, motor))
        return {"left_state": 0, "right_state": 1}

    def motor_query_24v_sensor(self):
        self.calls.append(("rail",))
        return {"ok": True, "no24v": False}

    def io_snapshot(self, board):
        self.calls.append(("io", board))
        return {1: 1, 3: 1}

    def motor_thermal_door_status(self) -> dict:
        self.calls.append(("thermal_status",))
        return {
            "oem_predicates": {
                "tcDoorClosed": self.closed,
                "closed_source": "queryHome(ThermalDoor)",
            }
        }

    def motor_oem_open_thermal_door(self, *, timeout_s=20.0):
        self.calls.append(("open_thermal_door", timeout_s))
        return {"ok": True, "operation": "openThermalDoor", "fake_no_hardware": True}


def _sealed_snapshot_config(*, serial_number: int, camera_calibrated: bool):
    return SimpleNamespace(
        calibration_source="immutable_oem_machine_snapshot",
        values={
            "SerialNumber": serial_number,
            "CameraCalibrated": camera_calibrated,
        },
        blockers=[],
    )


def test_door_closed_predicate_opens_then_returns_literal_source_failure_only_when_condition_is_true(monkeypatch):
    tester = _NoHardwareDoorPredicateTester(closed=False)
    monkeypatch.setattr(
        oem_startup_program,
        "load_oem_parity_config",
        lambda _snapshot: _sealed_snapshot_config(serial_number=206, camera_calibrated=True),
        raising=False,
    )

    result = BioXpStartupHardware(lambda: tester).startup_homing_stepwise(
        mode="live",
        step="door-closed-predicate",
        execute=True,
    )

    assert result["ok"] is False
    assert result["failed_closed"] is True
    assert result["error"] == "Cannot close thermal cycler door!"
    assert result["source_condition"] == "SerialNumber>9 && !confirmAxis(tcDoorClosed) && CameraCalibrated"
    assert result["sealed_config_binding"] == {
        "SerialNumber": 206,
        "CameraCalibrated": True,
        "source": "immutable_oem_machine_snapshot",
    }
    assert result["thermal_closed_predicate"] == {
        "tcDoorClosed": False,
        "source": "queryHome(ThermalDoor)",
    }
    assert tester.calls.count(("thermal_status",)) == 1
    assert tester.calls.count(("open_thermal_door", 20.0)) == 1
    assert all(call[0] != "hardware_transport" for call in tester.calls)


@pytest.mark.parametrize(
    ("serial_number", "camera_calibrated", "closed"),
    [
        (9, True, False),
        (206, False, False),
        (206, True, True),
    ],
)
def test_door_closed_predicate_never_opens_when_any_literal_source_term_is_false(
    monkeypatch,
    serial_number,
    camera_calibrated,
    closed,
):
    tester = _NoHardwareDoorPredicateTester(closed=closed)
    monkeypatch.setattr(
        oem_startup_program,
        "load_oem_parity_config",
        lambda _snapshot: _sealed_snapshot_config(
            serial_number=serial_number,
            camera_calibrated=camera_calibrated,
        ),
        raising=False,
    )

    result = BioXpStartupHardware(lambda: tester).startup_homing_stepwise(
        mode="live",
        step="door-closed-predicate",
        execute=True,
    )

    assert result["ok"] is True
    assert result["failed_closed"] is False
    assert result["source_condition_active"] is False
    assert not any(call[0] == "open_thermal_door" for call in tester.calls)
    assert all(call[0] != "hardware_transport" for call in tester.calls)


def test_door_closed_predicate_rejects_generic_closed_field_without_tc_door_closed_predicate(monkeypatch):
    class GenericStatusTester(_NoHardwareDoorPredicateTester):
        def motor_thermal_door_status(self) -> dict:
            self.calls.append(("thermal_status",))
            return {"closed": False}

    tester = GenericStatusTester(closed=False)
    monkeypatch.setattr(
        oem_startup_program,
        "load_oem_parity_config",
        lambda _snapshot: _sealed_snapshot_config(serial_number=206, camera_calibrated=True),
        raising=False,
    )

    result = BioXpStartupHardware(lambda: tester).startup_homing_stepwise(
        mode="live",
        step="door-closed-predicate",
        execute=True,
    )

    assert result["ok"] is False
    assert result["failed_closed"] is True
    assert result["error"] == "tcDoorClosed_source_predicate_required"
    assert not any(call[0] == "open_thermal_door" for call in tester.calls)


def test_door_closed_predicate_rejects_unsealed_config_without_source_default_fallback(monkeypatch):
    tester = _NoHardwareDoorPredicateTester(closed=False)
    monkeypatch.setattr(
        oem_startup_program,
        "load_oem_parity_config",
        lambda _snapshot: SimpleNamespace(
            calibration_source="unbound_fail_closed",
            values={"SerialNumber": 206, "CameraCalibrated": True},
            blockers=["OemMachineSnapshot_not_bound"],
        ),
        raising=False,
    )

    result = BioXpStartupHardware(lambda: tester).startup_homing_stepwise(
        mode="live",
        step="door-closed-predicate",
        execute=True,
    )

    assert result["ok"] is False
    assert result["failed_closed"] is True
    assert result["error"] == "sealed_config_machine_snapshot_required"
    assert not any(call[0] == "thermal_status" for call in tester.calls)
    assert not any(call[0] == "open_thermal_door" for call in tester.calls)
