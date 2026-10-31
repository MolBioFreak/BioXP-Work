"""Liquid search + surface aspirate: per-channel contact, level grouping, missing-liquid hold."""
import time
from types import SimpleNamespace

import pytest

from bioxp.pipette import cavro_application as app
from bioxp.pipette.cavro_liquid import compile_liquid_recipe
from bioxp.pipette.liquid_detection import aspirate_target, contact_from_timestamps, level_groups, search_window


def test_search_window_applies_tip_offset_and_floor_clearance():
    assert search_window(60000, 80000, 45757, 2016) == (105757, 123741)
    with pytest.raises(ValueError):
        search_window(60000, 61000, 0, 2016)


def test_contact_interpolates_over_the_descent():
    assert contact_from_timestamps(start_steps=1000, stop_steps=3000, started_at=10.0, stopped_at=12.0, stamp=11.0) == (2000, "interpolated_descent")
    assert contact_from_timestamps(start_steps=1000, stop_steps=3000, started_at=10.0, stopped_at=12.0, stamp=None)[0] is None
    assert contact_from_timestamps(start_steps=1000, stop_steps=3000, started_at=10.0, stopped_at=12.0, stamp=20.0)[0] is None


def test_level_groups_highest_first_and_modes():
    contacts = {0: 5000, 1: 5100, 2: 9000, 3: 9050}
    assert level_groups(contacts, "by_level", 1008) == [([0, 1], 5100), ([2, 3], 9050)]
    assert level_groups(contacts, "together", 1008) == [([0, 1, 2, 3], 9050)]
    assert level_groups(contacts, "by_level", -1) == [([0], 5000), ([1], 5100), ([2], 9000), ([3], 9050)]
    assert aspirate_target(9050, 2016, 10000) == 10000  # never below the floor


def _recipe(**detection):
    return {"mode": "single", "channels": [0, 1, 2], "timeout_ms": 5000, "target_liquid_ul": "5",
            "commanded_aspiration_ul": "5.5", "aspiration_speed_ul_s": 10, "aspiration_delay_ms": 200,
            "leading_air": None, "trailing_air": None, "before_leading_air": [], "before_liquid": [],
            "after_liquid": [], "before_dispense": [], "after_dispense": [], "dispense_segments": [],
            "final_empty_speed_ul_s": None, "final_empty_before": [], "multi": None,
            "liquid_detection": {"location_id": 3, **detection}}


def test_recipe_emits_search_then_surface_aspirate():
    ops = compile_liquid_recipe(_recipe(mode="by_level"))["application"]["operations"]
    kinds = [o["operation"] for o in ops]
    assert "aspirate" not in kinds and kinds.index("liquid_search") + 1 == kinds.index("surface_aspirate")
    search = ops[kinds.index("liquid_search")]
    assert search["expected_channels"] == [0, 1, 2] and search["location_id"] == 3
    assert "expected_channels" not in compile_liquid_recipe(_recipe(expected=False))["application"]["operations"][kinds.index("liquid_search")]


class _Rig:
    """Shared Z head with four pumps; channels touch at the given descent fractions (None = no contact)."""
    def __init__(self, monkeypatch, touch):
        self.z, self.moves, self.aspirated = 0, [], []
        row = SimpleNamespace(z_high=60000, z_low=80000)
        monkeypatch.setattr("bioxp.oem_compat.position_table.load_bound_oem_position_table",
                            lambda: SimpleNamespace(resolve=lambda **_: row))
        rig = self

        class Group:
            _interrupt_epoch = 0
            _fluid_detection_timestamps = {}
            _transports = [SimpleNamespace() for _ in range(4)]
            def _forceabort(self):
                return False
            def _run_group_liquid_operation(self, effect, channels, send, *, timeout_ms, after_sends=None, set_allow_to_stop=True):
                if after_sends:
                    after_sends()
                rows = []
                if effect == "fluid detection":
                    t0 = time.monotonic()
                    for c in channels:
                        f = touch.get(c)
                        stamp = None if f is None else t0 + 0.02 * f
                        rows.append({"channel": c, "completion": {"ok": f is not None,
                                     "pipette_message_state": {"fluid_timestamp": stamp}}})
                    time.sleep(0.021)
                    rig.z = 80000 - 2016  # descent ended at the floor
                    return {"ok": all(r["completion"]["ok"] for r in rows), "channels": rows, "interrupted_by_terminate": False}
                if effect == "aspirate":
                    rig.aspirated.append((list(channels), rig.z))
                return {"ok": True, "interrupted_by_terminate": False,
                        "channels": [{"channel": c, "completion": {"ok": True}} for c in channels]}

        def move(target, **_):
            rig.z = target; rig.moves.append(target); return {"ok": True}
        primitives = SimpleNamespace(pipette_transport=Group(), z_set_max_speed=lambda v: {"ok": True},
            oem_move_z=lambda target, **kw: {"ok": True, "target": target}, z_stop=lambda: {"ok": True},
            _read_axis_position=lambda axis: rig.z, _loaded_tip_z_offset=lambda: {"offset_steps": 0})
        self.provider = SimpleNamespace(primitives=primitives, moveZ=lambda t: move(t),
            _wp8_execution_fence_checker=lambda *a, **k: None,
            _manual_pipette_receipt_runner=lambda name, call, *a: call(primitives.pipette_transport),
            _offset_deck_semantic_state=lambda **_: {"pseudo_z_home": 0}, wp8_wait_z=lambda *a: {"ok": True})

    def run(self, recipe):
        application = compile_liquid_recipe(recipe)["application"]
        return app.run_application_inline(self.provider, application, command_id="c", owner_identity={"source_identity": "s"})


def test_each_level_aspirates_at_its_own_depth(monkeypatch):
    rig = _Rig(monkeypatch, {0: 0.25, 1: 0.25, 2: 0.75})
    result = rig.run(_recipe(mode="by_level"))
    assert result["ok"] is True, result.get("error")
    level = result["liquid_levels"][0]
    assert level["start_steps"] == 60000 and level["floor_steps"] == 80000 - 2016
    contacts = {c: level["channels"][str(c)]["contact_steps"] for c in "012"}
    assert contacts["0"] < contacts["2"]
    groups = result["surface_aspirations"][0]["groups"]
    assert [g["channels"] for g in groups] == [[0, 1], [2]]
    assert [ch for ch, _ in rig.aspirated] == [[0, 1], [2]]
    assert all(g["aspirate_steps"] == min(g["surface_steps"] + 2016, level["floor_steps"]) for g in groups)


def test_missing_liquid_in_listed_well_holds_and_aspirates_nothing(monkeypatch):
    rig = _Rig(monkeypatch, {0: 0.25, 1: None, 2: 0.5})
    result = rig.run(_recipe())
    assert result["ok"] is False and result["missing_liquid_channels"] == [1]
    assert result["requested_control"] == "pause_for_operator"
    assert rig.aspirated == []


def test_unlisted_well_without_liquid_is_skipped_not_held(monkeypatch):
    rig = _Rig(monkeypatch, {0: 0.25, 1: None, 2: 0.5})
    result = rig.run(_recipe(expected=False))
    assert result["ok"] is True and result["surface_aspirations"][0]["skipped_channels"] == [1]
    assert sorted(c for ch, _ in rig.aspirated for c in ch) == [0, 2]


def test_liquid_levels_survive_the_operational_record(monkeypatch):
    from bioxp.protocols.operational_results import application_outcome
    rig = _Rig(monkeypatch, {0: 0.25, 1: None, 2: 0.5})
    kept = application_outcome(rig.run(_recipe()))
    assert kept["missing_liquid_channels"] == [1]
    assert kept["liquid_levels"][0]["channels"]["0"]["found"] is True
