"""Source loadTips subprocedure exercised with synthetic machine callbacks."""
from dataclasses import dataclass, field, replace

import pytest

from bioxp.pipette.oem_load_tips import LoadTipsBindings, load_tips


@dataclass
class Rig:
    groups: list[int] = field(default_factory=lambda: [0, 24, 24, 24])
    types: list[int] = field(default_factory=lambda: [50, 50, 50, 50])
    tip_exists: bool = False
    tip_missing: bool = False
    tip_type: int = 201
    camera: bool = False
    inspect_result: bool = True
    lost: list[int] = field(default_factory=list)
    calls: list = field(default_factory=list)

    def step(self, name, *args):
        self.calls.append((name, *args))
        return {"ok": True}

    def bind(self):
        def query():
            self.calls.append(("query",))
            return {"ok": True, "tip_exists": self.tip_exists,
                    "tip_missing": self.tip_missing, "tip_type": self.tip_type}

        def remove(index, group):
            self.groups[index] = group + 1
            return self.step("remove", index, group)

        def home():
            value = self.lost.pop(0) if self.lost else 0
            self.calls.append(("home", value))
            return {"ok": True, "source_return": value}

        def lift(location):
            self.tip_exists, self.tip_missing = True, False
            return self.step("lift", location)

        def eject(check):
            self.tip_exists, self.tip_missing = False, True
            return self.step("eject", check)

        return LoadTipsBindings(
            facts=lambda: {}, query=query, light=lambda: self.step("light"),
            stall_default=lambda: self.step("stall_default"),
            tray=lambda index: dict(tip_type=self.types[index], tip_available=self.groups[index] < 24,
                                    next_group=self.groups[index], location=10 + index),
            move=lambda loc, col, row: self.step("move", loc, col, row),
            publish=lambda loc, well: self.step("publish", loc, well), home=home,
            lower=lambda loc: self.step("lower", loc), lift=lift, eject=eject,
            tip_state=lambda changes: self.step("state", dict(changes)), remove=remove,
            load_type=lambda kind: self.step("load_type", kind),
            inspect=lambda tray, group: {**self.step("inspect", tray, group), "source_return": self.inspect_result},
            current_max=lambda: self.step("current_max"), log_missing=lambda: self.step("log_missing"),
            error_event=lambda msg: self.step("error", msg), unlock_door=lambda: self.step("unlock"),
            start_mode=lambda: 0, overpress_checked_has_value=lambda: True,
            camera_ready=lambda: self.camera)


def test_same_type_early_return_does_not_pick_or_restore_stall():
    rig = Rig(tip_exists=True, tip_type=50)
    result = load_tips(50, rig.bind())
    assert result["source_return"] is True
    assert rig.calls == [("query",), ("light",)]


def test_force_new_tip_ejects_then_picks_and_consumes_exact_group():
    rig = Rig(tip_exists=True, tip_type=50)
    result = load_tips(50, rig.bind(), force_new_tip=True)
    assert result["source_return"] is True
    assert result["selected_wells"] == (0, 24, 48, 72)
    assert rig.groups[0] == 1
    assert rig.calls[:6] == [("query",), ("light",), ("state", {"tip_loaded": True}),
                              ("move", 6, 0, 0), ("publish", 6, 0), ("eject", True)]
    assert ("move", 10, 0, 0) in rig.calls
    assert rig.calls[-1] == ("current_max",)


def test_camera_false_returns_false_after_pickup_and_ejection():
    rig = Rig(camera=True, inspect_result=False)
    result = load_tips(50, rig.bind(), force_new_tip=True)
    assert result["source_return"] is False
    assert rig.groups[0] == 1
    assert rig.calls[-6:] == [("inspect", 0, 0), ("move", 6, 0, 0),
                               ("publish", 6, 0), ("eject", False),
                               ("state", {"tip_loaded": False}), ("current_max",)]
    assert ("eject", False) in rig.calls


def test_lost_z_steps_retries_before_success_without_consuming_extra_group():
    rig = Rig(lost=[0, 300, 0])
    result = load_tips(50, rig.bind(), force_new_tip=True)
    assert result["source_return"] is True
    assert [c for c in rig.calls if c[0] == "lower"] == [("lower", 10), ("lower", 10)]
    assert rig.groups[0] == 1


def test_callback_exception_retains_prefix_and_does_not_run_cleanup():
    from bioxp.pipette.oem_calibration import CalibrationExecutionError

    rig = Rig()
    bindings = rig.bind()
    def broken_lower(location):
        raise RuntimeError("transport interrupted")
    with pytest.raises(CalibrationExecutionError) as caught:
        load_tips(50, replace(bindings, lower=broken_lower), force_new_tip=True)
    assert caught.value.evidence["steps"][-1]["operation"] == "lowerPipette"
    assert ("move", 10, 0, 0) in rig.calls
    assert not any(c[0] in {"remove", "current_max", "eject"} for c in rig.calls)


def test_b_row_preserves_numeric_then_parsed_source_publication():
    rig = Rig(groups=[1, 24, 24, 24])
    result = load_tips(50, rig.bind(), force_new_tip=True)
    assert result["selected_group"] == 1
    assert result["selected_wells"] == (12, 36, 60, 84)
    assert ("move", 10, 0, 1) in rig.calls
    assert [(c[1], c[2]) for c in rig.calls if c[0] == "publish"] == [(10, 1), (10, 12)]


def test_no_matching_inventory_preserves_source_error_event():
    rig = Rig(types=[200] * 4, tip_missing=True)
    with pytest.raises(RuntimeError, match="Tips are not available"):
        load_tips(50, rig.bind(), force_new_tip=True)
    assert ("error", "Tips are not available") in rig.calls
    assert not any(c[0] == "move" for c in rig.calls)
