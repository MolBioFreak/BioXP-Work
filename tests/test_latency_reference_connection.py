"""Connection reuse never substitutes for a current, verified reference read."""
import os
import sqlite3
from concurrent.futures import ThreadPoolExecutor

import pytest
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.services.reference_service import ReferenceStateStore, MarkAxisReferencedCommand, MarkAxisDesyncedCommand


@pytest.fixture
def rig(tmp_path):
    runtime = OEMRuntimeStore(tmp_path)
    path = tmp_path / 'bioxp_runtime.db'
    reference = ReferenceStateStore(path)
    yield runtime, reference, path
    reference.close()
    runtime.close()


def test_repeated_reads_reuse_handle_without_transaction(rig):
    _, ref, _ = rig
    assert ref.mark_referenced(MarkAxisReferencedCommand('x', 0))['durable_clean']
    handle = ref._connection
    trace = []
    handle.set_trace_callback(trace.append)
    for _ in range(12):
        assert ref.snapshot(('x',))['rows']['x']['state'] == 'referenced'
        assert ref._connection is handle
        assert not handle.in_transaction
    assert sum('SELECT payload_json' in s for s in trace) == 12
    assert not any('sqlite_master' in s or 'table_info' in s for s in trace)


def test_other_writer_desync_visible_immediately(rig):
    _, ref, path = rig
    other = ReferenceStateStore(path)
    try:
        ref.mark_referenced(MarkAxisReferencedCommand('x', 0))
        version = ref.snapshot(('x',))['rows']['x']['state_version']
        other.mark_desynced(MarkAxisDesyncedCommand('x', 'test'))
        row = ref.snapshot(('x',))['rows']['x']
        assert row['state'] == 'desynced' and row['state_version'] == version + 1
    finally:
        other.close()


def test_schema_change_rechecks_triggers_and_fails_closed(rig):
    _, ref, path = rig
    ref.mark_referenced(MarkAxisReferencedCommand('x', 0))
    with sqlite3.connect(path) as db:
        db.execute('DROP TRIGGER reference_state_authority_no_delete_v1')
    result = ref.snapshot(('x',))
    assert not result['ok'] and result['rows']['x']['state'] == 'unknown'
    assert ref._connection is None


def test_thread_handoff_is_serialized(rig):
    _, ref, _ = rig
    ref.mark_referenced(MarkAxisReferencedCommand('x', 0))
    with ThreadPoolExecutor(max_workers=4) as pool:
        result = list(pool.map(lambda _: ref.snapshot(('x',)), range(32)))
    assert all(r['durable_clean'] and r['rows']['x']['state'] == 'referenced' for r in result)
    assert not ref._connection.in_transaction


def test_failed_write_cannot_leave_transaction_or_false_authority(rig, monkeypatch):
    _, ref, _ = rig
    ref.mark_referenced(MarkAxisReferencedCommand('x', 0))
    def fail(*args): raise OSError('injected failure')
    monkeypatch.setattr(ref, '_write_reference_payload', fail)
    result = ref.mark_referenced(MarkAxisReferencedCommand('x', 1))
    assert not result['ok']
    assert ref._connection is None
    assert not ref.snapshot(('x',))['durable_clean']


def test_replacement_inode_does_not_silently_adopt_authority(rig, tmp_path):
    _, ref, path = rig
    ref.mark_referenced(MarkAxisReferencedCommand('x', 0))
    # No mutation to a live DB: move this fixture's main path out of the way.
    moved = path.with_name('old.db')
    os.rename(path, moved)
    path.touch()
    try:
        result = ref.snapshot(('x',))
        assert not result['ok'] and result['rows']['x']['state'] == 'unknown'
        assert ref._connection is None
    finally:
        path.unlink()
        os.rename(moved, path)


def test_close_and_reopen_still_reads_durable_bytes(rig):
    _, ref, _ = rig
    ref.mark_referenced(MarkAxisReferencedCommand('x', 0))
    ref.close()
    assert ref._connection is None
    assert ref.snapshot(('x',))['rows']['x']['state'] == 'referenced'
