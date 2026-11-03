"""Saved depth offsets by tip type, station and plate type; default no offset."""
import json
from types import SimpleNamespace
from bioxp.pipette import depth_offsets as D


def test_no_entry_is_no_offset_and_most_specific_match_wins(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    assert D.resolve(tip_type=10, location="LOC_RC")["offset_steps"] == 0
    D.save(tip_type=10, location="LOC_RC", plate_type=None, offset_mm=4.4, source="creep")
    D.save(tip_type=10, location=None, plate_type="ONT Barcode 96", offset_mm=5.0)
    D.save(tip_type=None, location="LOC_STRIP1", plate_type=None, offset_mm=1.0)
    assert D.resolve(tip_type=10, location="LOC_RC")["offset_mm"] == 4.4
    assert D.resolve(tip_type=10, location="LOC_RC", plate_type="ont barcode 96")["offset_mm"] == 5.0
    assert D.resolve(tip_type=50, location="LOC_RC")["offset_steps"] == 0
    assert D.resolve(tip_type=50, location="LOC_STRIP1")["offset_steps"] == round(D.Z_STEPS_PER_MM)
    D.save(tip_type=10, location="LOC_RC", plate_type=None, offset_mm=4.0)  # replaces, not duplicates
    assert [e["offset_mm"] for e in D.entries() if e["location"] == "LOC_RC"] == [4.0]
    D.remove(tip_type=10, location="LOC_RC", plate_type=None)
    assert D.resolve(tip_type=10, location="LOC_RC")["offset_steps"] == 0


def test_saves_survive_a_damaged_file_and_refuse_bad_values(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    D.save(tip_type=10, location="LOC_RC", plate_type=None, offset_mm=4.4)
    D.save(tip_type=10, location="LOC_OC", plate_type=None, offset_mm=2.0)
    (tmp_path / D.SAVED_NAME).write_text("{not json")
    assert D.resolve(tip_type=10, location="LOC_RC")["offset_mm"] == 4.4  # last good copy
    for bad in ({"offset_mm": 99}, {"location": "TECANRACK1"}, {"location": "NOWHERE"}, {"tip_type": 7}):
        args = {"tip_type": 10, "location": "LOC_RC", "plate_type": None, "offset_mm": 1.0, **bad}
        try:
            D.save(**args)
            raise AssertionError(f"accepted {bad}")
        except ValueError:
            pass


def test_lowering_adds_depth_offset_but_never_at_tip_racks(tmp_path, monkeypatch):
    import bioxp.oem_serial206_initialization as m
    from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter as P
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    D.save(tip_type=10, location=None, plate_type=None, offset_mm=4.4)
    rows = {"TECANRACK1": SimpleNamespace(z_low=141531, z_high=10000), "LOC_RC": SimpleNamespace(z_low=87671, z_high=44571)}
    monkeypatch.setattr(m, "load_bound_oem_position_table", lambda: SimpleNamespace(source="fake", resolve=lambda location_id: rows[location_id]))
    monkeypatch.setattr(m.time, "sleep", lambda s: None)
    moves = []
    group = SimpleNamespace(_tip_type=10, _transports=[SimpleNamespace(_tip_loaded=True) for _ in range(4)])
    fake = SimpleNamespace(pipette_transport=group, _z_profile=lambda: None,
        oem_move_axis_absolute=lambda axis, target, wait_for_stop: moves.append(target) or {"ok": True})
    fake._loaded_tip_z_offset = lambda: P._loaded_tip_z_offset(fake)
    fake._depth_offset = lambda name, plate=None: P._depth_offset(fake, name, plate)
    P.z_pipette_position(fake, location_id="TECANRACK1", operation="lower_pipette")
    out = P.z_pipette_position(fake, location_id="LOC_RC", operation="lower_pipette")
    tip = P._loaded_tip_z_offset(fake)["offset_steps"]
    assert moves == [141531, 87671 + tip + round(4.4 * D.Z_STEPS_PER_MM)]
    assert out["depth_offset"]["offset_mm"] == 4.4
