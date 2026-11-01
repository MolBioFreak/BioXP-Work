"""pLLD settings follow the loaded tip type; filtered/unfiltered x 10/50/200."""
from bioxp.pipette import lld_profiles as L


def test_six_defaults_follow_tip_type_and_saved_edits_win(tmp_path, monkeypatch):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_STATE_ROOT", str(tmp_path))
    rows = L.profiles()["profiles"]
    assert sorted(rows) == ["10", "200", "50"] and all(sorted(r) == ["filtered", "unfiltered"] for r in rows.values())
    assert L.profile_for(50)["threshold_adc"] == 200 and L.profile_for(50)["kind"] == "filtered"
    assert L.profile_for(201)["tip_type"] == 200
    assert L.profile_for(None) is None and L.profile_for(7) is None
    assert L.wires(L.profile_for(10)) == ["p0,200R", "p1,5R", "p3,150R"]
    L.save(tip_type=10, kind="filtered", values={"threshold_adc": 120}, source="water test")
    assert L.profile_for(10)["threshold_adc"] == 120 and L.profile_for(10)["measured"] is True
    L.save(filtered_tips=False)
    assert L.profile_for(10) == {**L.profile_for(10), "kind": "unfiltered", "threshold_adc": 15}
    try:
        L.save(tip_type=10, kind="filtered", values={"threshold_adc": 0})
        raise AssertionError("out of range accepted")
    except ValueError:
        pass
