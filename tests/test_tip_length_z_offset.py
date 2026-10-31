"""Per-tip Z correction: measured offsets only, applied to calibrated heights with a tip loaded."""
import json

from bioxp.oem_compat.machine_state import OemMachineState
from bioxp.oem_compat.pathing import OemPathPlanner
from bioxp.pipette.tip_length import TIP_LENGTH_MM, tip_z_offset


class _Row:
    def __init__(self):
        self.z_low, self.z_high, self.inc_factor = 80000, 60000, 0
        self.base_coordinates = {"x": 1000, "y": 2000}


class _Table:
    def resolve(self, location_id):
        return _Row()


def _z(flag, *, loaded, offset):
    planner = OemPathPlanner(_Table())
    state = OemMachineState.from_query(tip_loaded=loaded, tip_z_offset_steps=offset, pseudo_z_home=65000)
    return planner._xy_z_for_script_target("3", column=0, row=0, positionflag=flag, state=state)["z"]


def test_unmeasured_offsets_are_zero_oem_behaviour(tmp_path):
    path = tmp_path / "offsets.json"
    path.write_text(json.dumps({"offsets_steps": {}}))
    for tip in (10, 50, 200, 201, None):
        assert tip_z_offset(tip, path=path)["offset_steps"] == 0
    assert tip_z_offset(10, path=path)["datasheet_delta_mm"] == 22.7
    assert tip_z_offset(200, path=path)["datasheet_delta_mm"] == -0.5
    assert TIP_LENGTH_MM[50] == 58.0


def test_measured_offset_is_used_and_non_integer_is_ignored(tmp_path):
    path = tmp_path / "offsets.json"
    path.write_text(json.dumps({"offsets_steps": {"10": 5321, "200": "-117"}, "source": "pLLD 2026-11"}))
    measured = tip_z_offset(10, path=path)
    assert measured["offset_steps"] == 5321 and measured["measured"] is True
    assert tip_z_offset(200, path=path)["offset_steps"] == 0  # string is not a measurement


def test_offset_applies_to_calibrated_heights_only_with_a_tip():
    assert _z(2, loaded=True, offset=5000) == 85000
    assert _z(1, loaded=True, offset=5000) == 65000
    assert _z(0, loaded=True, offset=5000) == 65000   # clearance (pseudo home) untouched
    assert _z(2, loaded=False, offset=5000) == 80000  # no tip, no correction
    assert _z(2, loaded=True, offset=0) == 80000


def test_shipped_offsets_match_datasheet_lengths_at_oem_z_scale():
    for tip, steps in ((10, 45757), (50, 0), (200, -1008), (201, -1008)):
        info = tip_z_offset(tip)
        assert info["measured"] is True and info["offset_steps"] == steps == info["datasheet_delta_steps"]


def test_confirmed_offset_overrides_default_and_is_listed(tmp_path, monkeypatch):
    from bioxp.pipette import tip_length
    monkeypatch.setenv(tip_length.CONFIRMED_ENV, str(tmp_path / "confirmed.json"))
    before = {r["tip_type"]: r["offset_steps"] for r in tip_length.offsets_table()["tips"]}
    table = tip_length.save_confirmed(10, 45000, "check move 2026-10-31", at="2026-10-31T00:00:00Z")
    row = next(r for r in table["tips"] if r["tip_type"] == 10)
    assert row["offset_steps"] == 45000 and row["confirmed"]["source"] == "check move 2026-10-31"
    assert next(r for r in table["tips"] if r["tip_type"] == 50)["offset_steps"] == before[50]
    import pytest
    with pytest.raises(ValueError):
        tip_length.save_confirmed(10, 999999, "x", at="t")
