"""Door flag write-through: one software flag, both stores consistent.

ControlLib.doorOpen keeps one door flag on the machine status object; every
later door decision and every operation that branches on the door reads it.
Observed live defect (2026-09-21): TCD DO published the canonical transition
``updateThermalDoorOpen {thermal_door_open: true}`` while the provider working
state kept the stale value, so the following TCD DC no-opped against
"already closed" while the door was physically open. These tests pin the
working-state mirror and the machine-state consumer.
"""
import pytest
from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot
from tests.test_deck_scoped_authority import Primitive


def door_rig(tmp_path, monkeypatch):
    from pathlib import Path
    from bioxp.oem_runtime_store import OEMRuntimeStore
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider
    from bioxp.services.reference_service import ReferenceStateStore

    bind_serial206_oem_snapshot(monkeypatch)
    runtime = OEMRuntimeStore(tmp_path)
    references = ReferenceStateStore(Path(tmp_path) / 'bioxp_runtime.db')
    provider = Serial206OemInitializationProvider(
        Primitive(home=True), state_store=runtime, reference_store=references,
        generation_provider=lambda: 1,
    )
    published = []

    def publish(**kwargs):
        published.append(kwargs)
        return {"ok": True}

    provider.bind_deck_semantic_state_publisher(publish)
    provider.deck_owner_authority_stamps = lambda: {
        "ownership_generation": 1, "board_epoch_4": 87, "board_epoch_5": 1,
    }
    return provider, published, runtime


def test_door_flag_update_mirrors_into_working_state(tmp_path, monkeypatch):
    provider, published, runtime = door_rig(tmp_path, monkeypatch)
    try:
        before = provider.wp8_operation_machine_state('thermal_door', {'open': True})
        assert before.get('door_is_open') is not True

        out = provider.wp8_update_thermal_door_open(
            'updateThermalDoorOpen', {'value': True}, command_id='c1', child_order=0, plan_digest='d1')

        # Canonical publication (audit record) carries the commanded value...
        assert published and published[0]['updates'] == {'thermal_door_open': True}
        assert published[0]['source_operation'] == 'updateThermalDoorOpen'
        assert out.get('ok') is True

        # ...and the working state the door decision reads receives it.
        assert provider._load_state()['machine_status']['thermal_door_open'] is True
        after = provider.wp8_operation_machine_state('thermal_door', {'open': True})
        assert after['door_is_open'] is True

        # Closing mirrors back the same way.
        provider.wp8_update_thermal_door_open(
            'updateThermalDoorOpen', {'value': False}, command_id='c2', child_order=0, plan_digest='d2')
        assert provider._load_state()['machine_status']['thermal_door_open'] is False
        assert provider.wp8_operation_machine_state('thermal_door', {'open': False})['door_is_open'] is False
    finally:
        runtime.close()


@pytest.mark.parametrize('tip_location', [None, -1, 0, 3])
def test_door_cycles_publish_with_retained_tip_metadata(tmp_path, monkeypatch, tip_location):
    """Real provider + both SQLite owners; no physical controller operations.

    OEM ControlLib.doorOpen assigns ThermalDoorOpen without inspecting tips.
    A loaded-tip observation with unknown location must survive that assignment.
    """
    from contextlib import nullcontext
    from bioxp.operator_command_plane import OperatorCommandStore

    provider, _, runtime = door_rig(tmp_path, monkeypatch)
    store = OperatorCommandStore(tmp_path)
    stamps = provider.deck_owner_authority_stamps()
    store.bind_deck_owner_authority_reader(lambda: stamps, scope=nullcontext)
    provider.bind_deck_semantic_state_publisher(store.publish_deck_owner_state)
    try:
        initial = store.publish_deck_owner_state(
            source_operation='pipette_owner', source_command_id='offline-tip-observation',
            updates={'tip_loaded': True, 'tip_dirty': None, 'tip_location': tip_location},
            **stamps,
        )
        current = initial
        for cycle in range(3):
            for opened in (True, False):
                out = provider.execute_wp8_child(
                    {'order': 10, 'operation': 'updateThermalDoorOpen', 'arguments': {'value': opened}},
                    command_id=f'offline-door-{cycle}-{opened}', child_order=10,
                    plan_digest='offline-door-publication',
                )
                assert out['ok'] is True
                current = store.deck_semantic_state()
                assert current['thermal_door_open'] is opened
                assert provider.wp8_operation_machine_state(
                    'thermal_door', {'open': not opened})['door_is_open'] is opened
                for field in ('tip_loaded', 'tip_dirty', 'tip_location',
                              'current_location', 'current_well', 'plate_on_gantry'):
                    assert current[field] == initial[field]
                assert current['transition_provenance']['source_operation'] == 'updateThermalDoorOpen'
        assert current['semantic_state_revision'] == initial['semantic_state_revision'] + 6
        store.stop()
        store = OperatorCommandStore(tmp_path)
        assert store.deck_semantic_state() == current
    finally:
        store.stop()
        runtime.close()


def test_door_flag_update_rejects_non_boolean(tmp_path, monkeypatch):
    provider, published, runtime = door_rig(tmp_path, monkeypatch)
    try:
        with pytest.raises(RuntimeError, match='updateThermalDoorOpen:value'):
            provider.wp8_update_thermal_door_open(
                'updateThermalDoorOpen', {'value': 'yes'}, command_id='c', child_order=0, plan_digest='d')
        assert not published
    finally:
        runtime.close()


def test_working_state_mirror_precedes_publication(tmp_path, monkeypatch):
    """A failed canonical publish must not leave the next decision on a stale flag."""
    provider, published, runtime = door_rig(tmp_path, monkeypatch)
    try:
        def failing_publish(**kwargs):
            raise RuntimeError('canonical publication unavailable')

        provider.bind_deck_semantic_state_publisher(failing_publish)
        with pytest.raises(RuntimeError):
            provider.wp8_update_thermal_door_open(
                'updateThermalDoorOpen', {'value': True}, command_id='c', child_order=0, plan_digest='d')
        # The read store is already correct even though the audit write failed.
        assert provider._load_state()['machine_status']['thermal_door_open'] is True
    finally:
        runtime.close()
