"""Bounded exact-byte memo; live SQLite authority reads remain mandatory."""
import hashlib
import json
import sqlite3

import pytest

from bioxp.oem_runtime_store import OEMRuntimeStore
from tests.test_protocol_workflow_child_admission import store, start


@pytest.fixture
def current(store):
    source = OEMRuntimeStore(store.root)
    state = {'movement_ledger': [], 'used_approvals': [], 'initialize_motion_ledger': [],
             'x_lifecycle': {'board_lifecycle_generation': 11}, 'label': 'é'}
    source.write_oem_serial206_initialization_state(state)
    start(store, {'5': 11})
    store.assert_workflow_current('parent')
    yield store, source, state
    source.close()


def test_unchanged_bytes_decode_and_dump_once_but_read_live_every_time(current, monkeypatch):
    owner, _, _ = current
    owner._workflow_verified_snapshot = None
    encoded = owner.connection.execute('SELECT state_json FROM serial206_authority_snapshots '
                                       'ORDER BY sequence DESC LIMIT 1').fetchone()[0]
    loads, dumps = json.loads, json.dumps
    calls = {'snapshot_load': 0, 'dump': 0}
    statements = []
    def load(value, *args, **kwargs):
        if value == encoded:
            calls['snapshot_load'] += 1
        return loads(value, *args, **kwargs)
    def dump(*args, **kwargs):
        calls['dump'] += 1
        return dumps(*args, **kwargs)
    monkeypatch.setattr(json, 'loads', load)
    monkeypatch.setattr(json, 'dumps', dump)
    owner.connection.set_trace_callback(statements.append)
    try:
        for _ in range(12):
            owner.assert_workflow_current('parent')
    finally:
        owner.connection.set_trace_callback(None)
    assert calls == {'snapshot_load': 1, 'dump': 1}
    for table in ('serial206_authority_snapshots', 'operator_plane_lane',
                  'operator_plane_safety', 'operator_plane_commands'):
        assert sum(table in sql for sql in statements) == 12
    key, generation = owner._workflow_verified_snapshot
    assert key[0] == encoded and generation == 11


@pytest.mark.parametrize('corruption', ['changed_bytes_reused_digest', 'changed_hash',
                                      'noncanonical_valid_hash', 'malformed', 'missing'])
def test_warm_memo_never_masks_snapshot_corruption(current, corruption):
    owner, _, _ = current
    memo = owner._workflow_verified_snapshot
    real = owner.connection
    # Model corrupt selected bytes without disabling append-only DB triggers.
    # Every SELECT still executes on the actual live connection first.
    class Selected:
        def execute(self, sql, *args):
            cursor = real.execute(sql, *args)
            if 'FROM serial206_authority_snapshots' not in sql:
                return cursor
            row = list(cursor.fetchone())
            if corruption == 'changed_bytes_reused_digest':
                row[0] = row[0].replace('é', 'e').replace('\\u00e9', 'e')
                assert row[0] != memo[0][0]
            elif corruption == 'changed_hash':
                row[1] = '0' * 64
            elif corruption == 'noncanonical_valid_hash':
                row[0] += ' '
                row[1] = hashlib.sha256(row[0].encode('utf-8')).hexdigest()
            elif corruption == 'malformed':
                row[0] = '{'
            elif corruption == 'missing':
                row = None
            return type('Cursor', (), {'fetchone': lambda self: row})()
    for _ in range(2):
        with owner._lock, pytest.raises(ValueError):
            owner._workflow_current(Selected(), 'parent')
        assert owner._workflow_verified_snapshot == memo
    owner.assert_workflow_current('parent')


@pytest.mark.parametrize('generation', [12, None, True, '11'])
def test_external_valid_snapshot_generation_change_still_fails(current, generation):
    owner, source, state = current
    state['x_lifecycle']['board_lifecycle_generation'] = generation
    source.write_oem_serial206_initialization_state(state)
    for _ in range(2):
        with pytest.raises(ValueError, match='workflow_board_epoch_changed'):
            owner.assert_workflow_current('parent')


@pytest.mark.parametrize('sql,error', [
    ('UPDATE operator_plane_safety SET global_epoch=global_epoch+1', 'workflow_interrupted'),
    ('UPDATE operator_plane_safety SET x_epoch=x_epoch+1', 'workflow_interrupted'),
    ('UPDATE operator_plane_lane SET owner_lease_until=0', 'workflow_authority_lost'),
    ("UPDATE operator_plane_lane SET owner_id='other-owner'", 'workflow_authority_lost'),
    ('UPDATE operator_plane_lane SET dispatcher_epoch=dispatcher_epoch+1', 'workflow_authority_lost'),
    ('UPDATE operator_plane_lane SET workflow_command_id=NULL', 'workflow_authority_lost'),
])
def test_warm_memo_does_not_cache_live_safety_lease_or_owner(current, sql, error):
    owner, _, _ = current
    lane = dict(owner.connection.execute('SELECT * FROM operator_plane_lane').fetchone())
    try:
        with sqlite3.connect(owner.path) as external:
            external.execute(sql)
        with pytest.raises(ValueError, match=error):
            owner.assert_workflow_current('parent')
    finally:
        # Return fixture custody before its normal shutdown path.
        with sqlite3.connect(owner.path) as external:
            external.execute('UPDATE operator_plane_lane SET ' +
                             ','.join(key + '=?' for key in lane), tuple(lane.values()))


@pytest.mark.parametrize('axis', [None, 'x'])
def test_warm_memo_preserves_in_memory_priority_fences(current, axis):
    owner, _, _ = current
    fence = owner._priority_fence if axis is None else owner._axis_priority_fences[axis]
    fence.set()
    try:
        with pytest.raises(ValueError, match='workflow_interrupted'):
            owner.assert_workflow_current('parent')
    finally:
        fence.clear()


def test_new_valid_bytes_replace_single_entry_and_rollback_is_visible(current):
    owner, source, state = current
    old = owner._workflow_verified_snapshot
    state['label'] = 'new snapshot, same generation'
    with owner._lock, source._authority_write():
        source._db.execute('BEGIN IMMEDIATE')
        try:
            source._append_serial206_authority_snapshot_locked(state)
            owner._workflow_current(source._db, 'parent')
            assert owner._workflow_verified_snapshot != old
        finally:
            source._db.execute('ROLLBACK')
    owner.assert_workflow_current('parent')
    assert owner._workflow_verified_snapshot == old
