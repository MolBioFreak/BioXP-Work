"""Real calibration SQLite/compiler/provider; existing inert-wire rig only."""
import copy
import json

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from bioxp.oem_calibration_settings import CalibrationSettingsPatch, CalibrationSettingsService, load_saved_calibration
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.oem_compat.position_table import load_bound_oem_position_table
from bioxp.oem_deck_catalog import DeckCatalog
from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot
from tests.test_movement_modes import connected, rig, cycle, bind_deck, submit, z_key
from tests.test_debloat_core import named

HOLDER = dict(micro_row_1=dict(x=85555, y=63278), micro_row_2=dict(x=90262, y=63278),
              pcr_row_1=dict(x=80162, y=61978), pcr_row_2=dict(x=82262, y=61978), pcr_y_pitch=2132)


def save_owner(tmp_path, monkeypatch, snapshot=None):
    from bioxp.oem_machine_bundle import apply_owned_calibration_snapshot
    snapshot = snapshot or bind_serial206_oem_snapshot(monkeypatch)
    store = OEMRuntimeStore(tmp_path)
    service = CalibrationSettingsService(store, snapshot, publish=apply_owned_calibration_snapshot)
    return store, service, snapshot


def test_api_persistence_cold_read_and_unchanged_oem(tmp_path, monkeypatch):
    from bioxp import api
    store, service, baseline = save_owner(tmp_path, monkeypatch)
    before = load_bound_oem_position_table(baseline)
    original_rows = copy.deepcopy(before.rows())
    monkeypatch.setattr(api.app.state, 'calibration_settings', service, raising=False)
    app = FastAPI()
    app.add_api_route('/motion/oem/calibration_settings', api.calibration_settings_save, methods=['PATCH'])
    app.add_api_route('/motion/oem/calibration_settings', api.calibration_settings_read, methods=['GET'])
    with TestClient(app) as http:
        result = http.patch('/motion/oem/calibration_settings', json={'bulk_reagent_holder': HOLDER})
        assert result.status_code == 200, result.text
        response = result.json()
        assert response['active_bulk_reagent_holder'] == HOLDER
        assert not response['pending_restart'] and not response['motion_commanded']
        assert response['active_positions'] == response['saved_positions']
        assert http.get('/motion/oem/calibration_settings').json()['saved_bulk_reagent_holder'] == HOLDER
        revision = response['saved_revision']
        assert revision['positions'] == []
        assert all('z' not in key.lower() for row in revision['bulk_reagent_holder'].values() if isinstance(row, dict) for key in row)
        assert '146000' not in json.dumps(revision)
    # Unrelated settings save must retain the holder definition.
    service.save(CalibrationSettingsPatch.model_validate({'positions': [{'name': 'CAMERA_OFFSET', 'x': 10}]}))
    assert service.read()['active_bulk_reagent_holder'] == HOLDER
    store.close()
    cold = OEMRuntimeStore(tmp_path)
    restored = load_saved_calibration(baseline, cold)
    table = load_bound_oem_position_table(restored)
    assert table.bulk_reagent_holder == HOLDER
    assert restored.position_table[0]['raw_attributes'] == baseline.position_table[0]['raw_attributes']
    holder_only = load_bound_oem_position_table(service.active_snapshot.calibration_baseline)
    assert holder_only.rows() == original_rows
    # The only explicitly changed OEM row was CAMERA_OFFSET; all others identical.
    for old, new in zip(before.rows(), table.rows()):
        if old['location_id'] != 'CAMERA_OFFSET':
            assert old == new
    trough = table.resolve(location_id='LOC_TROUGH')
    assert trough.base_coordinates['x'] == 85555 and trough.base_coordinates['y'] == 63278
    assert trough.inc_factor == 0
    assert trough.to_payload() == before.resolve(location_id='LOC_TROUGH').to_payload()
    cold.close()


@pytest.mark.parametrize('field,value', [('z', 146000), ('zLow', 55000), ('zDelta', 106754)])
def test_reject_holder_z(field, value):
    holder = copy.deepcopy(HOLDER)
    holder['pcr_row_1'][field] = value
    with pytest.raises(ValueError):
        CalibrationSettingsPatch.model_validate({'bulk_reagent_holder': holder})


@pytest.mark.parametrize('payload', [{}, {'positions': []}, {'bulk_reagent_holder': None}])
def test_empty_updates_rejected(payload):
    with pytest.raises(ValueError):
        CalibrationSettingsPatch.model_validate(payload)


def test_catalog_and_all_provider_targets(tmp_path, monkeypatch):
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider
    store, service, baseline = save_owner(tmp_path, monkeypatch)
    unconfigured = DeckCatalog.from_position_table(load_bound_oem_position_table(baseline))
    with pytest.raises(KeyError):
        unconfigured.resolve('LOC_TROUGH_PCR_ROW_1_WELL_1')
    service.save(CalibrationSettingsPatch.model_validate({'bulk_reagent_holder': HOLDER}))
    table = load_bound_oem_position_table(service.active_snapshot)
    catalog = DeckCatalog.from_position_table(table)
    assert catalog.resolve('LOC_TROUGH').panel_label == 'Bulk reagent holder'
    assert len(catalog.rows()) == len(unconfigured.rows()) + 20
    for kind in ('micro', 'pcr'):
        for row in (1, 2):
            anchor = HOLDER[f'{kind}_row_{row}']
            key = f'LOC_TROUGH_{kind.upper()}_ROW_{row}'
            destination = catalog.resolve(key)
            assert destination.xy == (anchor['x'], anchor['y'])
            *_, coords = Serial206OemInitializationProvider._named_move_xy(16, False, False, key)
            assert (coords['x'], coords['y']) == destination.xy
            if kind == 'pcr':
                for well in range(1, 9):
                    destination = catalog.resolve(f'{key}_WELL_{well}')
                    *_, coords = Serial206OemInitializationProvider._named_move_xy(16, False, False, destination.target)
                    assert (coords['x'], coords['y']) == (anchor['x'], 61978 + (well - 1) * 2132)
                    if well == 2:
                        assert coords['y'] == 64110 and destination.channel_wells == (2, 4, 6, 8)
                    if well == 1:
                        assert destination.channel_wells == (1, 3, 5, 7)
    # No new holder geometry borrows OEM Z values in catalog metadata.
    for row in catalog.rows():
        if 'holder_row' in row:
            assert set(row['xy']) == {'x', 'y'} and row['calibration_axes'] == ['x', 'y']
    leaked = table.bulk_reagent_holder
    leaked['pcr_row_1']['x'] = 1
    assert table.bulk_reagent_holder == HOLDER
    store.close()


def test_connected_named_action_and_sequence_workflow(connected, tmp_path, monkeypatch):
    from bioxp.oem_machine_bundle import get_active_oem_machine_snapshot
    from bioxp.manual_pipetting import compile_manual_pipetting
    http, provider, frames, fault, driver, positions = connected
    store, service, baseline = save_owner(tmp_path, monkeypatch, get_active_oem_machine_snapshot())
    service.save(CalibrationSettingsPatch.model_validate({'bulk_reagent_holder': HOLDER}))
    bind_deck(http, provider)
    zb, zm = z_key(provider)
    positions[zb, zm] = 146000  # alignment probe is not saved; ordinary raise only
    target = {'target': 'LOC_TROUGH_PCR_ROW_2_WELL_2', 'camera_offset': False}
    # Real workflow compiler accepts the same finite target, no embedded XY.
    document = compile_manual_pipetting({'protocol_id': 'holder', 'steps': [
        {'operation': 'move_sequence', 'targets': [target]}]})
    assert document.stages[0].actions
    frames.clear()
    targets = [{'target': 'LOC_TROUGH_MICRO_ROW_2', 'camera_offset': False}, target]
    _, receipt = submit(http, 'oem.xy.move_sequence', 'holder-seq-0001', {'targets': targets})
    assert receipt['status'] == 'completed', receipt
    result = receipt['source_receipt']['xy_sequence']
    assert [c['resolved_xy'] for c in result['children']] == [dict(x=90262,y=63278), dict(x=82262,y=64110)]
    z_moves = [v for b, c, t, m, v in frames if c == 4 and (b, m) == (zb, zm)]
    assert z_moves == [500], z_moves
    store.close()


def test_all_named_destinations_through_existing_executor(named, tmp_path, monkeypatch):
    from bioxp.oem_machine_bundle import get_active_oem_machine_snapshot
    run, provider, command_store, counts, frames, fault = named
    store, service, baseline = save_owner(tmp_path, monkeypatch, get_active_oem_machine_snapshot())
    service.save(CalibrationSettingsPatch.model_validate({'bulk_reagent_holder': HOLDER}))
    catalog = DeckCatalog.from_position_table(load_bound_oem_position_table())
    zb, zm = z_key(provider)
    rows = [row for row in catalog.rows() if 'holder_row' in row]
    assert len(rows) == 20
    for number, row in enumerate(rows):
        result, cid = run(row['target'], key=f'holder-native-{number:04d}')
        assert result['ok'], result
        xy_moves = [(b, v) for b, c, t, m, v in frames if c == 4 and (b, m) != (zb, zm)]
        assert provider.primitives._read_axis_position('x') == row['xy']['x'], (row, xy_moves)
        assert provider.primitives._read_axis_position('y') == row['xy']['y'], (row, xy_moves)
        z_moves = [v for b, c, t, m, v in frames if c == 4 and (b, m) == (zb, zm)]
        assert all(v == 500 for v in z_moves), z_moves
    store.close()


HOLDER_Z = {**HOLDER, 'micro_row_1': dict(x=85555, y=63278, z_low=120000)}


def test_holder_row_z_persists_and_xy_only_digest_unchanged(tmp_path, monkeypatch):
    store, service, baseline = save_owner(tmp_path, monkeypatch)
    table = load_bound_oem_position_table(baseline)
    service.save(CalibrationSettingsPatch.model_validate({'bulk_reagent_holder': HOLDER}))
    # XY-only holders publish exactly as before (no null Z keys).
    assert service.read()['active_bulk_reagent_holder'] == HOLDER
    service.save(CalibrationSettingsPatch.model_validate({'bulk_reagent_holder': HOLDER_Z}))
    assert service.read()['active_bulk_reagent_holder']['micro_row_1'] == dict(x=85555, y=63278, z_low=120000)
    store.close()
    cold = load_bound_oem_position_table(load_saved_calibration(baseline, OEMRuntimeStore(tmp_path)))
    assert cold.holder_row_z('micro_row_1') == {'z_low': 120000, 'z_high': None}
    assert cold.holder_row_z('pcr_row_1') == {'z_low': None, 'z_high': None}
    # Legacy trough heights are untouched.
    assert cold.resolve(location_id='LOC_TROUGH').z_low == table.resolve(location_id='LOC_TROUGH').z_low


@pytest.mark.parametrize('row', [dict(x=1, y=2, z_high=500), dict(x=1, y=2, z_low=1000, z_high=1000)])
def test_holder_row_z_high_must_be_above_low(row):
    with pytest.raises(Exception):
        CalibrationSettingsPatch.model_validate({'bulk_reagent_holder': {**HOLDER, 'micro_row_1': row}})


def test_manual_lower_lift_carry_holder_row_to_finite_leaf():
    from bioxp.manual_pipetting import compile_manual_pipetting, manual_position_plan
    doc = compile_manual_pipetting({'protocol_id': 'h', 'steps': [
        {'operation': 'lower', 'location_id': 16, 'holder_row': 'micro_row_1'},
        {'operation': 'lift', 'location_id': 16, 'holder_row': 'micro_row_1', 'height_steps': None},
        {'operation': 'lower', 'location_id': 16}]})
    params = [a.params for a in doc.stages[0].actions]
    assert params[2] == {'operation': 'lower', 'location_id': 16}
    lower = manual_position_plan(params[0])['children'][0]
    assert lower['operation'] == 'sourceLowerTo' and lower['arguments'] == {'location': 16, 'holder_row': 'micro_row_1'}
    lift = manual_position_plan(params[1])['children'][0]
    assert lift['arguments'] == {'location': 16, 'height': None, 'holder_row': 'micro_row_1'}
    with pytest.raises(Exception):
        compile_manual_pipetting({'protocol_id': 'h', 'steps': [{'operation': 'lower', 'location_id': 3, 'holder_row': 'micro_row_1'}]})


def test_lower_lift_leaf_uses_holder_row_z(tmp_path, monkeypatch):
    import bioxp.oem_serial206_initialization as mod
    from types import SimpleNamespace
    store, service, baseline = save_owner(tmp_path, monkeypatch)
    service.save(CalibrationSettingsPatch.model_validate({'bulk_reagent_holder': HOLDER_Z}))
    table = load_bound_oem_position_table(service.active_snapshot)
    monkeypatch.setattr(mod, 'load_bound_oem_position_table', lambda *a: table)
    provider = object.__new__(mod.Serial206OemInitializationProvider)
    moves = []
    provider.moveZ = lambda target: moves.append(target) or {'ok': True}
    provider.primitives = SimpleNamespace(pipette_transport=SimpleNamespace(_tip_type=50, _tip_loaded=True))
    leaf = lambda op, **a: provider.wp8_pipette_source_leaf(op, a, command_id='c', child_order=0, plan_digest='d')
    leaf('sourceLowerTo', location=16, holder_row='micro_row_1')
    leaf('sourceLiftTo', location=16, holder_row='micro_row_1', height=None)
    leaf('sourceLiftTo', location=16, holder_row='micro_row_1', height=20000)
    assert moves == [120000, 500, 100000]
    with pytest.raises(RuntimeError):
        leaf('sourceLowerTo', location=16, holder_row='pcr_row_1')
    store.close()


def test_liquid_detection_passes_holder_row():
    from bioxp.pipette.cavro_liquid import LiquidDetection
    from bioxp.pipette.cavro_application import LiquidSearch
    assert LiquidDetection.model_validate({'location_id': 16, 'holder_row': 'micro_row_1'}).holder_row == 'micro_row_1'
    assert LiquidSearch.model_fields['holder_row'].default is None
