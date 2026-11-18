"""One plate record holds gripper heights and depth offsets; OEM height when a plate has none."""
import json

from bioxp import plate_profiles as P
from bioxp.oem_deck_movement import _plate_gripper_offset
from bioxp.pipette import depth_offsets as D


def test_enduraplate_fast_default_on_magnet_block(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    assert P.gripper_z_offset("OUTPUT_PLATE", "LOC_P_MS")["z_low_offset_steps"] == -10236
    assert _plate_gripper_offset(1, 25) == -10236          # 143882 - 10236 = 133646
    P.set_loaded(movable_object="OUTPUT_PLATE", plate_id="4483354")  # catalog number resolves to id
    assert P.loaded()["OUTPUT_PLATE"] == "thermo-4483354-enduraplate"
    assert _plate_gripper_offset(1, 25) == -30236          # no entry: OEM height
    P.save_gripper(plate_id="thermo-4483354-enduraplate", location="LOC_P_MS", z_low_offset_steps=-5000)
    assert _plate_gripper_offset(1, 25) == -5000


def test_depth_offsets_live_in_the_plate_file_and_fold_in_legacy(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    (tmp_path / "depth_offsets.json").write_text(json.dumps({"entries": [
        {"tip_type": 10, "location": "LOC_RC", "plate_type": None, "offset_mm": 5.4}]}))
    assert D.resolve(tip_type=10, location="LOC_RC")["offset_mm"] == 5.4
    D.save(tip_type=10, location="LOC_OC", plate_type=None, offset_mm=2.0)
    assert not (tmp_path / "depth_offsets.json").exists()
    saved = json.loads((tmp_path / P.SAVED_NAME).read_text())
    assert {r["location"] for r in saved["depth_offsets"]} == {"LOC_RC", "LOC_OC"}


def test_workflow_plate_type_sets_loaded(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    P.set_loaded(movable_object="OUTPUT_PLATE", plate_id="thermo-4483485-enduraplate-fast")
    assert _plate_gripper_offset(1, 25) == -10236
    P.set_loaded(movable_object="OUTPUT_PLATE", plate_id="unlisted-plate")
    assert _plate_gripper_offset(1, 25) == -30236
