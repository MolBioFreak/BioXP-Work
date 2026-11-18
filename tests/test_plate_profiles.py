"""One plate record holds gripper heights and depth offsets; OEM height when a plate has none."""
import json

from bioxp import plate_profiles as P
from bioxp.oem_deck_movement import _plate_gripper_offset
from bioxp.pipette import depth_offsets as D

FAST = "thermo-4483485-enduraplate-fast"


def test_enduraplate_fast_default_on_magnet_block(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    assert _plate_gripper_offset(1, 25) == -10236          # 143882 - 10236 = 133646
    P.set_loaded(movable_object="OUTPUT_PLATE", plate_id="other-plate")
    assert P.loaded()["OUTPUT_PLATE"] == "other-plate"
    assert _plate_gripper_offset(1, 25) == -30236          # no entry: OEM height
    P.save_gripper(plate_id="other-plate", location="LOC_P_MS", z_low_offset_steps=-5000)
    assert _plate_gripper_offset(1, 25) == -5000
    P.save_gripper(plate_id="other-plate", location="LOC_P_MS", z_low_offset_steps=None)
    assert _plate_gripper_offset(1, 25) == -30236


def test_depth_offsets_nest_under_plates_and_fold_in_legacy(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    (tmp_path / "depth_offsets.json").write_text(json.dumps({"entries": [
        {"tip_type": 10, "location": "LOC_RC", "plate_type": None, "offset_mm": 5.4}]}))
    assert D.resolve(tip_type=10, location="LOC_RC")["offset_mm"] == 5.4
    D.save(tip_type=10, location="LOC_RC", plate_type=FAST, offset_mm=2.0)
    assert not (tmp_path / "depth_offsets.json").exists()
    saved = json.loads((tmp_path / P.SAVED_NAME).read_text())
    assert [r["offset_mm"] for r in saved["station_depth_offsets"]] == [5.4]
    assert [r["offset_mm"] for r in saved["plates"][FAST]["depth_offsets"]] == [2.0]
    assert D.resolve(tip_type=10, location="LOC_RC", plate_type=FAST)["offset_mm"] == 2.0
    assert D.resolve(tip_type=10, location="LOC_RC", plate_type="other")["offset_mm"] == 5.4


def test_station_follows_the_deck_record_for_manual_and_workflow_moves(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    where = {"OUTPUT_PLATE": "LOC_P_MS"}
    P.set_location_reader(lambda: where)
    assert P.plate_at("LOC_MS") == FAST           # manual catch/release only updates the deck record
    where["OUTPUT_PLATE"] = "LOC_P_OC"
    assert P.plate_at("LOC_MS") is None and P.plate_at("LOC_OC") == FAST
    where["OUTPUT_PLATE"] = "LOC_GANTRY"
    assert P.plate_at("LOC_OC") is None
    P.record_plate_move(movable_object="OUTPUT_PLATE", plate_id=FAST, destination="LOC_P_MS")
    assert P.loaded()["OUTPUT_PLATE"] == FAST
    P.set_station(location="LOC_RC", plate_id="4483485")
    assert P.plate_at("LOC_RC") == FAST
    P.set_location_reader(None)
