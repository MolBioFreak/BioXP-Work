"""Any subset of the four guns can carry tips; eject and Park handle every subset (offline wire)."""
import pytest

from tests.test_new_method_launch_tips import head


def _loaded(group):
    return [ch for ch, t in enumerate(group._transports) if t._tip_loaded]


@pytest.mark.parametrize("mounted", [[0], [1], [3], [0, 2], [1, 2, 3], [0, 1, 2, 3]])
def test_eject_all_ejects_exactly_the_loaded_guns_and_never_refuses(monkeypatch, mounted):
    group, wire = head(monkeypatch, mounted)
    group.query_tip_status_all()
    assert group._tip_location_channels() == mounted
    wire.clear()
    out = group.eject_all_tips(check_missing_tip=True, wait=True, channels=None)
    assert out["ok"] and out["channels_ejected"] == mounted
    assert [ch for cmd, ch in wire if cmd == "E1R"] == mounted
    assert out["lost_tip_channels"] == []
    assert _loaded(group) == [] and group._tip_location == -1
    assert group._expected_tip_channels is None and group._tip_type == 201


def test_empty_head_eject_is_a_clean_noop(monkeypatch):
    group, wire = head(monkeypatch, [])
    out = group.eject_all_tips(check_missing_tip=True, wait=True, channels=None)
    assert out["ok"] and out["channels_ejected"] == []
    assert not [c for c, _ in wire if c == "E1R"]


def test_pick_four_release_two_then_pipette_and_eject_only_the_kept(monkeypatch):
    group, wire = head(monkeypatch, [0, 1, 2, 3])
    group.query_tip_status_all()
    # Release guns 0 and 2 (the manual-pickup release step).
    group._eject_tip_channels_once([0, 2], wait_for_completion=True)
    group._note_tips_released([0, 2])
    assert group._tip_location_channels() == [1, 3]
    assert group._primary_tip_channel() == 1
    wire.clear()
    out = group.eject_all_tips(check_missing_tip=True, wait=True, channels=None)
    assert out["channels_ejected"] == [1, 3]
    assert [ch for cmd, ch in wire if cmd == "E1R"] == [1, 3]
    assert group._expected_tip_channels is None


def test_lost_tip_is_reported_not_refused(monkeypatch):
    # Guns 1 and 3 were loaded; gun 3's tip fell off, so the head reads only gun 1.
    group2, _ = head(monkeypatch, [1])
    group2._expected_tip_channels = [1, 3]
    out = group2.eject_all_tips(check_missing_tip=True, wait=True, channels=None)
    assert out["ok"] and out["channels_ejected"] == [1]
    assert out["lost_tip_channels"] == [3]


def test_keep_one_tip_tracks_that_gun(monkeypatch):
    group, wire = head(monkeypatch, [0, 1, 2, 3])
    out = group.KeepTip(2)
    assert out["ok"] and group._tip_location_channels() == [2]
    assert group._expected_tip_channels == [2]


def test_tips_added_by_a_second_pickup_join_the_tracked_set(monkeypatch):
    group, _ = head(monkeypatch, [1])
    group.query_tip_status_all()
    assert group._tip_location_channels() == [1]
    group2, _ = head(monkeypatch, [1, 2])
    group2._expected_tip_channels = [1]
    group2.query_tip_status_all()
    assert group2._tip_location_channels() == [1, 2]


def test_collection_snapshot_matches_head_after_eject(monkeypatch):
    """Park reads this snapshot; it must not still show an ejected tip."""
    group, _ = head(monkeypatch, [1])
    group.query_tip_status_all()
    assert any(c["tip_loaded"] for c in group.collection_source_snapshot()["channels"])
    group.eject_all_tips(check_missing_tip=True, wait=True, channels=None)
    assert not any(c["tip_loaded"] for c in group.collection_source_snapshot()["channels"])

