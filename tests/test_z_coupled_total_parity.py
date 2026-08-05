from types import SimpleNamespace

from src.bioxp import oem_serial206_initialization as subject
from src.bioxp.usb_driver import BioXpTester


class _HomeGZTester:
    def motor_oem_home_gz(self, **kwargs):
        return {
            "ok": False,
            "branch": "caught_plate_recovery",
            "failure": "Plate caught in gripper after plate press",
            "z_current": {
                "ok": True,
                "ack": {"status": 100},
                "readback": {"ack": {"status": 100}, "value": 31},
            },
            "recovery": {"x_home": {"ok": True}, "solenoid": {"ok": True}},
        }


def test_home_gz_caught_plate_failure_retains_verified_z_current_override(monkeypatch):
    adapter = subject.Serial206ProductionPrimitiveAdapter.__new__(
        subject.Serial206ProductionPrimitiveAdapter
    )
    adapter.tester = _HomeGZTester()
    adapter._z_profile_overrides = {}
    adapter._z_profile = lambda: {"board": 4, "motor": 1, "run_current": 31}
    monkeypatch.setattr(
        subject,
        "load_oem_parity_config",
        lambda _path: SimpleNamespace(blockers=[], values={"GripperVersion": 1}),
    )

    result = adapter.z_home_gz(pseudo_z_home_steps=500)

    assert result["ok"] is False
    assert result["result"]["branch"] == "caught_plate_recovery"
    assert adapter._z_profile_overrides == {6: 31}


class _PositionTable:
    source = "test-position-table"

    def resolve(self, *, location_id):
        assert location_id == "LOC_TROUGH"
        return SimpleNamespace(z_low=1000, z_high=200)


def test_pipette_methods_keep_exact_source_names_and_targets(monkeypatch):
    calls = []
    adapter = subject.Serial206ProductionPrimitiveAdapter.__new__(
        subject.Serial206ProductionPrimitiveAdapter
    )
    adapter._z_profile = lambda: {"board": 4, "motor": 1}
    adapter.oem_move_axis_absolute = lambda axis, target, *, wait_for_stop: calls.append(
        (axis, target, wait_for_stop)
    ) or {"ok": True}
    monkeypatch.setattr(subject, "load_bound_oem_position_table", lambda: _PositionTable())
    monkeypatch.setattr(subject.time, "sleep", lambda _seconds: None)

    lower = adapter.z_pipette_position(
        location_id="LOC_TROUGH", operation="lower_pipette", overpress=True
    )
    lift = adapter.z_pipette_position(
        location_id="LOC_TROUGH", operation="lift_pipette"
    )

    assert lower["source_method"] == "ClassControlInterface.lowerPipette"
    assert lower["target_position_steps"] == 5030
    assert lift["source_method"] == "ClassControlInterface.liftPipette"
    assert lift["target_position_steps"] == 200
    assert calls == [("z", 5030, True), ("z", 200, True)]


def test_production_move_gz_sends_both_nonwaiting_targets_and_verifies_terminal_state():
    driver = object.__new__(BioXpTester)
    targets = {}
    driver._motion_oem_axis_profile = lambda axis, startup=False: {  # type: ignore[method-assign]
        "board": 4,
        "motor": 1 if axis == "z" else 0,
    }
    driver._oem_board_present = lambda board: True  # type: ignore[method-assign]
    def move(board, position, *, motor=0):
        targets[motor] = position
        return {"ok": True, "ack": {"status": 100}}
    driver.motor_move_absolute = move  # type: ignore[method-assign]
    driver.motor_wait_stopped = lambda board, *, motor=0, target_position=None, **kwargs: {  # type: ignore[method-assign]
        "stopped": True,
        "last_speed": 0,
        "last_ack": {"status": 100},
        "target_reached": targets[motor] == target_position,
    }
    driver.motor_get_position = lambda board, *, motor=0: {  # type: ignore[method-assign]
        "ok": True,
        "ack": {"status": 100},
        "position": targets[motor],
    }
    driver.motor_get_speed = lambda board, *, motor=0: {  # type: ignore[method-assign]
        "ok": True,
        "ack": {"status": 100},
        "speed": 0,
    }

    result = driver.motor_oem_move_gz(gripper_position=27350, z_position=65000)

    assert result["ok"] is True
    assert targets == {0: 27350, 1: 65000}
