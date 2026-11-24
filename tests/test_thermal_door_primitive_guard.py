"""Every XY primitive goes around the closed thermal door with a long tip (offline)."""
import pytest

from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot

MS = (26213, 9241)       # magnet stand, below the cycler on the left
WASTE = (92049, 93211)
STRIP4 = (68625, 83403)
BYPASS_Y = 45256


class _Tester:
    def __init__(self, door):
        self.door = door
        self.reads = 0

    def motor_thermal_door_status(self):
        self.reads += 1
        if self.door == "error":
            raise RuntimeError("no reply")
        if self.door == "travel":
            return {"opened": False, "closed": False}
        return {"opened": self.door == "open", "closed": self.door == "closed"}


class _Channel:
    def __init__(self, tip):
        self._tip_loaded = tip is not None
        self._tip_type = tip


def _adapter(monkeypatch, *, tip, door, start=MS, z=500):
    bind_serial206_oem_snapshot(monkeypatch)
    a = object.__new__(Serial206ProductionPrimitiveAdapter)
    a.tester = _Tester(door)
    a.pipette_transport = _Channel(tip)
    a.pos = {"x": start[0], "y": start[1], "z": z}
    a.moves = []

    def direct(x, y, **_):
        a.moves.append((x, y))
        a.pos.update(x=x, y=y)
        return {"ok": True}

    def move_z(position, **_):
        a.moves.append(("z", position))
        a.pos["z"] = position
        return {"ok": True}

    a._move_xy_direct = direct
    a._read_axis_position = lambda axis: a.pos[axis]
    a.oem_move_z = move_z
    return a


@pytest.mark.parametrize("tip", [50, 200])
@pytest.mark.parametrize("door", ["closed", "travel", "error"])
def test_long_tip_move_xy_detours_unless_sensors_say_open(monkeypatch, tip, door):
    a = _adapter(monkeypatch, tip=tip, door=door)
    out = a.move_xy(*STRIP4, wait_timeout_s=5.0)
    assert a.moves == [(MS[0], BYPASS_Y), STRIP4]
    assert out["closed_door_bypass"]["door_open"] is (False if door == "closed" else None)


def test_long_tip_open_door_and_ten_ul_go_direct(monkeypatch):
    for tip, door in ((50, "open"), (10, "closed"), (None, "closed")):
        a = _adapter(monkeypatch, tip=tip, door=door)
        a.move_xy(*STRIP4, wait_timeout_s=5.0)
        assert a.moves == [STRIP4]
    assert _adapter(monkeypatch, tip=10, door="closed").tester.reads == 0


def test_door_sensors_not_read_when_the_leg_misses_the_box(monkeypatch):
    a = _adapter(monkeypatch, tip=50, door="closed", start=WASTE)
    a.move_xy(*STRIP4, wait_timeout_s=5.0)
    assert a.moves == [STRIP4] and a.tester.reads == 0


def test_detour_raises_z_first_when_low(monkeypatch):
    a = _adapter(monkeypatch, tip=50, door="closed", z=80000)
    a.move_xy(*STRIP4, wait_timeout_s=5.0)
    assert a.moves == [("z", 500), (MS[0], BYPASS_Y), STRIP4]


def test_open_flag_is_ignored_when_sensors_say_closed(monkeypatch):
    """The stored flag is not consulted: only the sensors decide."""
    a = _adapter(monkeypatch, tip=50, door="closed")
    assert a.thermal_door_open_from_sensors() is False
    a.tester.door = "open"
    assert a.thermal_door_open_from_sensors() is True
    a.tester.door = "travel"
    assert a.thermal_door_open_from_sensors() is None


def test_park_inside_door_operation_binds_its_pipette_parent(monkeypatch):
    """Door Park's tip cleanup had no dispatch context: lifecycle_pipette_attempt_identity_not_bound."""
    a = _adapter(monkeypatch, tip=50, door="open")
    seen = {}
    a.pipette_audit_runner = lambda name, op, **kw: seen.update(kw) or {"ok": True}
    with pytest.raises(RuntimeError, match="lifecycle_pipette_attempt_identity_not_bound"):
        a._run_audited_pipette("q", lambda t: None, lifecycle_stage_id="serial206.park")
    a._wp8_parent_command_id = "door-op-1"
    a._run_audited_pipette("q", lambda t: None, lifecycle_stage_id="serial206.park")
    assert seen["lifecycle_attempt_id"] == "door-op-1:serial206.park"

