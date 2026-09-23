"""Observed covers through the real provider/publisher and retained SQLite.

No inventory defaults or publication/semantic-reader doubles. The existing
retained fixture supplies isolated controller observations, never live IO.
"""
import json
import subprocess
import sys

import pytest

from bioxp.oem_deck_movement import (
    canonical_movable_object_locations, compile_finite_plate_operation,
)
from tests.test_deck_scoped_authority import retained_rig, qualify_test_references


def test_observed_cover_publication_from_empty_inventory(retained_rig, monkeypatch):
    provider, primitive, runtime, references, store, root = retained_rig
    qualify_test_references(references)
    from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
    from tests.test_cover_inspection_movement import OfflineNative
    from tests.test_cover_inspection_flow import _SETTINGS, _jpeg, _GRAY_WITH_COVER
    native = OfflineNative()
    adapter = object.__new__(Serial206ProductionPrimitiveAdapter)
    adapter.tester, adapter.y_provider, adapter.reference_store = native, None, references
    provider.primitives = adapter
    monkeypatch.setattr('bioxp.oem_serial206_initialization.time.sleep', lambda seconds: None)
    frame = _jpeg(_GRAY_WITH_COVER)
    barcode_frames = []
    provider.bind_oem_cover_inspection_callbacks(
        settings=lambda: _SETTINGS,
        capture=lambda **kwargs: {'frame': frame, 'capture_evidence': {}},
        save=lambda **kwargs: {'ok': True},
        led=lambda **kwargs: None, rgb=lambda *args: None,
        barcode=lambda frame: barcode_frames.append(frame) or '',
    )
    stamps = provider.deck_owner_authority_stamps()
    # Explicit offline predecessor observations, through the canonical owner.
    # In particular do not bootstrap the constructor's complete inventory.
    for operation, updates in (
        ('updateLocation', {'current_location': 'LOC_OC_COVER', 'current_well': 0}),
        ('pipette_owner', {'tip_loaded': False, 'tip_dirty': False, 'tip_location': -1}),
        ('clean_path_calculation', {'clean_path': False}),
        ('sourceUnlatch', {'latch_closed': False}),
        ('sourceForceToHighHome', {'pseudo_z_home': 500}),
    ):
        store.publish_deck_owner_state(source_operation=operation,
            source_command_id='isolated-predecessor-' + operation, updates=updates, **stamps)
    provider.wp8_update_thermal_door_open('updateThermalDoorOpen', {'value': False},
        command_id='isolated-door-observation', child_order=0, plan_digest='isolated')
    before = store.deck_semantic_state()
    assert before['movable_plate_locations'] == {}
    assert before['plate_on_gantry'] is None
    plan = compile_finite_plate_operation('cover_inspection', source_leaf_available=True,
        deck_inspection=True, screen_resolution_high=False, inspection_log_only=False)
    publications = [child for child in plan['children'] if child['operation'] == 'updatePlateLocation']
    expected = {}
    for child, name, location, plate in zip(publications, ('OUTPUT_COVER', 'REAGENT_COVER'),
                                           ('LOC_OC_COVER', 'LOC_RC_COVER'), (4, 5), strict=True):
        inspection = plan['children'][child['order'] - 1]
        assert inspection['operation'] == 'inspectCoverAt'
        observed = provider.execute_wp8_child(inspection, command_id='isolated-inspection',
            child_order=inspection['order'], plan_digest=plan['plan_digest'])
        assert observed['cover_detected'] is True
        before = store.deck_semantic_state()
        result = provider.execute_wp8_child(child, command_id='isolated-inspection',
            child_order=child['order'], plan_digest=plan['plan_digest'])
        assert result['ok'] is True
        expected[name] = location
        after = store.deck_semantic_state()
        assert after['movable_plate_locations'] == expected
        assert after['plate_on_gantry'] is None and after['clean_path'] is False
        assert after['semantic_state_revision'] == before['semantic_state_revision'] + 1
        assert after['transition_provenance']['source_operation'] == 'updatePlateLocation'
        # The next inspection's real semantic read, then the real catch compiler
        # consume only the observed source, not a fictional full inventory.
        assert provider.mov_execution_machine_state()['plate_locations'][plate] == child['arguments']['location']
        machine = provider.wp8_operation_machine_state('catch_plate', {'plate': plate})
        catch = compile_finite_plate_operation('catch_plate', source_leaf_available=True,
            **machine, plate=plate)
        assert catch['resolved_location'] == child['arguments']['location']
        if plate == 4:
            missing = provider.wp8_operation_machine_state('catch_plate', {'plate': 5})
            assert missing['plate_location'] is None
            prior_moves = list(native.moves)
            with pytest.raises((TypeError, ValueError)):
                provider.wp8_catch_plate('catchPlate', {'plate': 5},
                    command_id='missing-cover', owner_identity={})
            assert native.moves == prior_moves
        code = ('import json,sys; from bioxp.operator_command_plane import OperatorCommandStore; '
                's=OperatorCommandStore(sys.argv[1]); print(json.dumps(s.deck_semantic_state())); s.stop()')
        reopened = json.loads(subprocess.check_output([sys.executable, '-c', code, str(root)], text=True))
        assert reopened == after
        before = after
    assert not any(row[0] == 'move' for row in primitive.calls)
    assert [axis for axis, _, _ in native.moves].count('x') == 4
    assert [axis for axis, _, _ in native.moves].count('y') == 4
    assert barcode_frames == [frame, frame]
    # Stale owner publication still refuses and cannot alter retained inventory.
    with pytest.raises(RuntimeError, match='deck_owner_authority_changed'):
        store.publish_deck_owner_state(source_operation='updatePlateLocation', source_command_id='stale',
            updates={'movable_plate_locations': expected}, **{**stamps, 'board_epoch_5': stamps['board_epoch_5'] + 1})
    assert store.deck_semantic_state() == before


@pytest.mark.parametrize('value', [None, [], {'UNKNOWN_COVER': 'LOC_OC_COVER'},
    {'OUTPUT_COVER': 'unknown'}, {'OUTPUT_COVER': 17}, {4: 'LOC_OC_COVER'},
    {'OUTPUT_COVER': 'LOC_OC_COVER', 'REAGENT_COVER': 'LOC_OC_COVER'}])
def test_partial_inventory_still_validates_every_entry(value):
    with pytest.raises(ValueError, match='not authoritative'):
        canonical_movable_object_locations(value, require_complete=False)


@pytest.mark.parametrize('value', [{}, {'OUTPUT_COVER': 'LOC_OC_COVER'}])
def test_bootstrap_still_requires_complete_inventory(value):
    with pytest.raises(ValueError, match='not authoritative'):
        canonical_movable_object_locations(value)
    assert canonical_movable_object_locations(value, require_complete=False) == value


def test_custody_location_is_not_a_machine_travel_target(monkeypatch):
    from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot
    from bioxp.oem_compat.position_table import load_bound_oem_position_table
    from bioxp.oem_deck_movement import validate_compiled_wp8_machine_targets
    bind_serial206_oem_snapshot(monkeypatch)
    table = load_bound_oem_position_table()
    validate_compiled_wp8_machine_targets(
        {'operation': 'updatePlateLocation', 'arguments': {'plate': 4, 'location': 29}}, table)
    for operation in ('scriptmoveTo', 'moveZ'):
        with pytest.raises(RuntimeError, match='machine_target_absent_from_serial206_position_table:29'):
            validate_compiled_wp8_machine_targets(
                {'operation': operation, 'arguments': {'location': 29}}, table)
