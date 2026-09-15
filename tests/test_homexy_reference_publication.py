"""Prepared/unreferenced XY must be published by paired Home, not prior Homes."""
import copy
import json
import sqlite3
import subprocess
import sys
import pytest
from fastapi.testclient import TestClient
from tests.test_homexy_recovery_handoff import (
    retained_rig, installed_retained, native_routes, canonical_xy_route,
    stopped_failure, homed_replacement, paired_home, run_home, reconcile,
    raw_command, test_canonical_homexy_dispatch_recovery_and_warm_consumers as canonical_check,
)

pytestmark = pytest.mark.parametrize('homed_replacement', ['paired_only'], indirect=True)


def starting_state(rig):
    app, provider, primitive, refs, root, leaf, data = rig
    assert data['home_results']['x'] is None and data['home_results']['y'] is None
    assert provider._load_state()['x_lifecycle']['state'] == 'prepared_unreferenced'
    assert provider.state_store.board4_authority_projection()['axes']['y']['lifecycle_state'] == 'prepared_unreferenced'
    assert provider._load_state()['z_lifecycle']['state'] == 'referenced_ready'
    rows = refs.snapshot(('x','y'))['rows']
    assert all(r['state'] == 'desynced' for r in rows.values())
    return copy.deepcopy(rows)


def test_cold_paired_home_publishes_both_references_and_recovery(paired_home):
    before = starting_state(paired_home)
    app, provider, primitive, refs, root, leaf, data = paired_home
    failed_before = raw_command(app.state.operator_command_plane.store, data['command_id'])
    outcome = run_home(paired_home)
    assert outcome['ok'] is True
    assert outcome['result']['source_return'] == {'x': -30, 'y': 7}
    assert outcome['result']['home'] == data['paired_native']
    assert all(v['position_after_sethome'] is None for v in data['paired_native'].values())
    rows = refs.snapshot(('x','y'))['rows']
    assert all(rows[a]['state'] == 'referenced' and rows[a]['origin_position_steps'] == 0
               and rows[a]['state_version'] > before[a]['state_version'] for a in ('x','y'))
    assert provider._load_state()['x_lifecycle']['state'] == 'referenced_ready'
    assert provider.state_store.board4_authority_projection()['axes']['y']['lifecycle_state'] == 'referenced_ready'
    response = reconcile(paired_home)
    assert response.status_code == 200, response.text
    assert raw_command(app.state.operator_command_plane.store, data['command_id']) == failed_before
    assert sorted(a for a, _ in data['paired_calls']) == ['x', 'y'] and leaf.moves == []
    assert app.state.operator_command_plane.store.deck_recovery_blocker() is None


def test_cold_paired_home_canonical_dispatch_and_warm_consumers(paired_home):
    starting_state(paired_home)
    canonical_check(paired_home)


@pytest.mark.parametrize('axis', ['x', 'y'])
@pytest.mark.parametrize('fault', ['position_ack', 'position_cache', 'position_bool', 'nonzero',
                                  'speed_ack', 'moving', 'home_ack', 'stop_ack', 'sethome_ack', 'clipped', 'cached'])
def test_cold_proof_failure_never_publishes_references(paired_home, monkeypatch, axis, fault):
    before = starting_state(paired_home)
    app, provider, primitive, refs, root, leaf, data = paired_home
    board = 5 if axis == 'x' else 4
    home, position, speed = leaf.motor_oem_go_home, leaf.motor_get_position, leaf.motor_get_speed
    def native(a, **kw):
        result = home(a, **kw)
        if a == axis:
            if fault == 'home_ack': result['home_hit']['ack'] = {'status': 1}
            if fault == 'stop_ack': result['stop']['first_delivery'] = {'status': 1}
            if fault == 'sethome_ack': result['set_home']['ack'] = {'status': 1}
            if fault == 'clipped': return {'omitted': 'item_limit'}
            if fault == 'cached': result['source_noop'] = True
        return result
    def read_position(b, **kw):
        result = copy.deepcopy(position(b, **kw))
        if b == board:
            if fault == 'position_ack': result['ack'] = {'status': 1}
            if fault == 'position_cache': result['ok'] = False
            if fault == 'position_bool': result['position'] = False
            if fault == 'nonzero': result['position'] = 8
        return result
    def read_speed(b, **kw):
        result = copy.deepcopy(speed(b, **kw))
        if b == board:
            if fault == 'speed_ack': result['ack'] = {'status': 1}
            if fault == 'moving': result['speed'] = 1
        return result
    monkeypatch.setattr(leaf, 'motor_oem_go_home', native)
    monkeypatch.setattr(leaf, 'motor_get_position', read_position)
    monkeypatch.setattr(leaf, 'motor_get_speed', read_speed)
    run_home(paired_home)
    assert refs.snapshot(('x','y'))['rows'] == before
    assert provider.state_store.board4_authority_projection()['axes']['y']['lifecycle_state'] == 'prepared_unreferenced'
    assert reconcile(paired_home).status_code == 409
    assert leaf.moves == []


@pytest.mark.parametrize('point', ['reference', 'receipts', 'state'])
def test_cold_atomic_rollback_and_publication_only_retry(paired_home, monkeypatch, point):
    before = starting_state(paired_home)
    app, provider, primitive, refs, root, leaf, data = paired_home
    owner, method = (refs, '_write_reference_payload') if point == 'reference' else (
        provider.state_store, '_append_serial206_receipts_locked' if point == 'receipts' else '_append_serial206_authority_snapshot_locked')
    original = getattr(owner, method)
    hit = []
    def fail(*args, **kwargs):
        # Allow the initial executing checkpoint; reject after publication has
        # attempted to write both the Y and shared reference authorities.
        if point != 'state' or args[0]['x_lifecycle']['state'] == 'referenced_ready':
            hit.append(True)
            raise OSError('isolated coordinated publication failure')
        return original(*args, **kwargs)
    monkeypatch.setattr(owner, method, fail)
    outcome = run_home(paired_home)
    assert hit and outcome['ok'] is False, outcome
    assert outcome['result']['ok'] is True  # source completion preserved
    assert refs.snapshot(('x','y'))['rows'] == before
    assert provider.state_store.board4_authority_projection()['axes']['y']['lifecycle_state'] == 'prepared_unreferenced'
    assert all(provider.state_store.read_serial206_receipt(a, 'paired-home') is None for a in ('x','y'))
    calls = copy.deepcopy(data['paired_calls'])
    monkeypatch.setattr(owner, method, original)
    replay = run_home(paired_home)
    assert replay['ok'] is True and replay['replayed'] is True, replay
    assert data['paired_calls'] == calls
    response = reconcile(paired_home)
    assert response.status_code == 200, response.text
    assert leaf.moves == []


@pytest.mark.parametrize('fault', ['interrupt', 'owner', 'board'])
def test_late_authority_change_rolls_back_the_whole_publication(paired_home, monkeypatch, fault):
    before = starting_state(paired_home)
    app, provider, primitive, refs, root, leaf, data = paired_home
    store = provider.state_store
    append = store._append_serial206_receipts_locked
    hit = []
    def change(rows):
        result = append(rows)
        hit.append(True)
        if fault == 'interrupt': provider._x_interrupt_epoch += 1
        elif fault == 'owner': provider._home_recovery_owner_id = 'new-owner'
        else:
            epoch = provider.preparation_provider.current_board_lifecycle_generation()
            provider.preparation_provider.current_board_lifecycle_generation = lambda: epoch + 1
        return result
    monkeypatch.setattr(store, '_append_serial206_receipts_locked', change)
    outcome = run_home(paired_home)
    assert hit and outcome['ok'] is False, outcome
    assert refs.snapshot(('x','y'))['rows'] == before
    assert store.board4_authority_projection()['axes']['y']['lifecycle_state'] == 'prepared_unreferenced'
    assert store.read_oem_serial206_initialization_state()['x_lifecycle']['state'] != 'referenced_ready'
    assert store.read_serial206_receipt('x', 'paired-home') is None or 'recovery_home' not in store.read_serial206_receipt('x', 'paired-home')
    assert leaf.moves == []


def test_new_reference_is_not_overwritten_by_old_home_publication(paired_home, monkeypatch):
    from bioxp.services.reference_service import MarkAxisDesyncedCommand
    starting_state(paired_home)
    app, provider, primitive, refs, root, leaf, data = paired_home
    speed = leaf.motor_get_speed
    changed = []
    def drift(board, **kw):
        result = speed(board, **kw)
        if board == 4 and not changed:
            refs.mark_desynced(MarkAxisDesyncedCommand('x', reason='newer reference owner'))
            changed.append(copy.deepcopy(refs.snapshot(('x','y'))['rows']))
        return result
    monkeypatch.setattr(leaf, 'motor_get_speed', drift)
    outcome = run_home(paired_home)
    assert changed and outcome['ok'] is False
    assert refs.snapshot(('x','y'))['rows'] == changed[0]
    assert reconcile(paired_home).status_code == 409


def test_independent_writer_new_command_is_not_overwritten(paired_home, monkeypatch):
    starting_state(paired_home)
    app, provider, primitive, refs, root, leaf, data = paired_home
    original = provider.state_store.finalize_homexy_reference
    seen = []
    def replace(*args, **kwargs):
        # An independent connection/process changes current command ownership
        # after observations but before the actual atomic writer is acquired.
        code = ('from bioxp.oem_runtime_store import OEMRuntimeStore;import sys;'
                's=OEMRuntimeStore(sys.argv[1]);p=s.read_oem_serial206_initialization_state();'
                'p["x_lifecycle"]["active_receipt"]["command_id"]="newer-command";'
                's.write_oem_serial206_initialization_state(p);s.close()')
        subprocess.run([sys.executable, '-c', code, str(root)], check=True, timeout=10)
        seen.append(True)
        return original(*args, **kwargs)
    monkeypatch.setattr(provider.state_store, 'finalize_homexy_reference', replace)
    outcome = run_home(paired_home)
    assert seen and outcome['ok'] is False, outcome
    current = provider.state_store.read_oem_serial206_initialization_state()
    assert current['x_lifecycle']['active_receipt']['command_id'] == 'newer-command'
    assert all(r['state'] == 'desynced' for r in refs.snapshot(('x','y'))['rows'].values())
    assert leaf.moves == []


def test_repeat_home_and_replay_use_committed_current_references(paired_home):
    starting_state(paired_home)
    app, provider, primitive, refs, root, leaf, data = paired_home
    assert run_home(paired_home)['ok'] is True
    first = refs.snapshot(('x','y'))['rows']
    assert run_home(paired_home, 'second-paired-home')['ok'] is True
    second = refs.snapshot(('x','y'))['rows']
    assert all(second[a]['state_version'] > first[a]['state_version'] for a in ('x','y'))
    calls = copy.deepcopy(data['paired_calls'])
    replay = run_home(paired_home, 'second-paired-home')
    assert replay['ok'] is True and replay['replayed'] is True
    assert data['paired_calls'] == calls
    assert refs.snapshot(('x','y'))['rows'] == second
    with pytest.raises(sqlite3.Error):
        provider.state_store._db.execute('UPDATE reference_state_authority SET payload_json=payload_json')
    assert refs.snapshot(('x','y'))['rows'] == second
