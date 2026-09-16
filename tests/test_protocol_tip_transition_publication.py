"""Source tipLocation/wellID mapping through the real SQLite publisher.

Synthetic authority/inventory only. No controller connection or physical leaves.
"""
import pytest

from bioxp.operator_command_plane import OperatorCommandStore
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider


@pytest.fixture
def publisher(tmp_path, monkeypatch):
    from bioxp.oem_runtime_store import OEMRuntimeStore
    preparation = OEMRuntimeStore(tmp_path / 'store')
    preparation.close()
    store = OperatorCommandStore(tmp_path / 'store')
    stamps = dict(ownership_generation=1, board_epoch_4=1, board_epoch_5=1)
    import threading
    # Synthetic authority remains stable under its own actual owner lock.
    owner_lock = threading.RLock()
    store.bind_deck_owner_authority_reader(lambda: stamps, scope=lambda: owner_lock)
    store.publish_tip_tray_transition(tray_id=0, transition='construct',
        operation_id='synthetic-constructor', command_id='synthetic-constructor',
        provenance={'synthetic_inventory': True}, **stamps)
    provider = object.__new__(Serial206OemInitializationProvider)
    provider._tip_tray_state_publisher = store.publish_tip_tray_transition
    monkeypatch.setattr(provider, 'deck_owner_authority_stamps', lambda: stamps)
    monkeypatch.setattr(provider, 'invalidate_deck_authority_cache', lambda **_: None)
    yield store, provider, stamps
    store.stop()


@pytest.mark.parametrize('group', range(24))
def test_source_group_uses_interleaved_well_ids(publisher, group):
    store, provider, stamps = publisher
    # ClassTipTray.removeTip(tipLocation,-1): column=location/2,
    # row=location%2; the other pipettes are two rows apart.
    expected = [group // 2 + (group % 2 + 2 * channel) * 12 for channel in range(4)]
    row = store.publish_tip_tray_transition(tray_id=0, transition='remove_group',
        group_index=group, operation_id='remove', command_id='remove',
        provenance={'source': 'ClassTipTray.removeTip'}, **stamps)
    assert [i for i, value in enumerate(row['occupancy']) if not value] == expected
    assert row['available_count'] == 23
    assert row['tip_available'] is (group != 23)
    reopened = OperatorCommandStore(store.root)
    try:
        assert reopened.tip_tray_state(0) == row
    finally:
        reopened.stop()


@pytest.mark.parametrize('wells', [[0], [95], [0,24,48,72], [12,36,60,84], [23,47,71,95]])
def test_source_remove_callback_uses_existing_canonical_transitions(publisher, wells):
    store, provider, _ = publisher
    row = provider.publish_tip_tray_transition(tray_id=0, transition='remove',
        well_ids=wells, operation_id='remove', command_id='remove', provenance={'synthetic': True})
    assert [i for i, value in enumerate(row['occupancy']) if not value] == wells
    provenance = row['provenance']
    assert provenance['transition'] == ('remove_well' if len(wells) == 1 else 'remove_group')
    assert provenance['well_ids'] == wells
    assert row['tip_available'] is (wells != [23,47,71,95])
    assert row['available_count'] == (None if len(wells) == 1 else 23)
    restored = provider.publish_tip_tray_transition(tray_id=0, transition='restore',
        well_ids=wells, operation_id='restore', command_id='restore', provenance={'synthetic': True})
    assert all(restored['occupancy'])
    # Retip does not reset the OEM tray-empty latch.
    assert restored['tip_available'] == row['tip_available']


def test_camera_missing_counts_source_groups_not_adjacent_slots(publisher):
    store, _, stamps = publisher
    row = store.publish_tip_tray_transition(tray_id=0, transition='camera_missing',
        well_ids=[0,1], operation_id='camera', command_id='camera', provenance={}, **stamps)
    assert row['available_count'] == 22


@pytest.mark.parametrize('wells', [[], [0,1], [0,1,2,3], [0,24,48,73], [24,48,72,96], [True], [0,0,0,0]])
def test_invalid_source_removal_cannot_publish(publisher, wells):
    store, provider, _ = publisher
    before = store.tip_tray_state(0)
    with pytest.raises(ValueError):
        provider.publish_tip_tray_transition(tray_id=0, transition='remove', well_ids=wells,
            operation_id='invalid', command_id='invalid', provenance={})
    assert store.tip_tray_state(0) == before
