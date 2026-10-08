"""Retirement uses actual SQLite migration, readers and reopen; no hardware."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import sqlite3

import pytest

from bioxp import oem_runtime_store as owner
from bioxp import runtime_method_retirement_v15 as migration
from bioxp.operator_command_plane import OperatorCommandStore
from bioxp.runtime_retention import retain_runtime_rows


def records(db, table, excluded=()):
    columns = [row[1] for row in db.execute(f'PRAGMA table_info("{table}")') if row[1] not in excluded]
    sql = 'SELECT ' + ','.join('"'+c+'"' for c in columns) + ' FROM "'+table+'"'
    return sorted((tuple(row) for row in db.execute(sql)), key=repr)


def legacy_store(root, monkeypatch):
    with monkeypatch.context() as scoped:
        scoped.setattr(owner, 'migrate_runtime_database_v2', owner._migrate_runtime_database_through_v14)
        store = owner.OEMRuntimeStore(root)
    assert store._db.execute('PRAGMA user_version').fetchone()[0] == 14
    return store


def restore_method_fixture(db, *, queued=False):
    # Restore a known historical row set, not an execution or fabricated live
    # result. Reinstall all frozen triggers before the migration validates it.
    triggers = list(db.execute("SELECT name,sql FROM sqlite_master WHERE type='trigger'"))
    for name, _ in triggers:
        db.execute(f'DROP TRIGGER "{name}"')
    state = 'queued' if queued else 'completed'
    method_id = 'restored-method'
    source = '{"steps":[{"action_id":"oem.x.move_steps","inputs":{"steps":10}}],"unknown":null}'
    db.execute('INSERT INTO operator_plane_methods(method_id,name,source_json,digest,failure_policy,status,version,ownership_generation,expanded_count,first_stream_sequence,last_stream_sequence,queued_at,updated_at) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?)',
        (method_id,'retained batch',source,hashlib.sha256(source.encode()).hexdigest(),'fail_fast',state,1,7,1,1,1,1,2))
    db.execute('INSERT INTO serial206_movement_methods(method_id,idempotency_key,action_id,canonical_inputs_sha256,state,state_version,failure_policy,child_count,accepted_at) VALUES(?,?,?,?,?,?,?,?,?)',
        (method_id,'restored-key','operator.method','a'*64,state,1,'require_completed',1,1))
    db.execute('INSERT INTO operator_commands(command_id,idempotency_key,action_id,status,started_at,updated_at,receipt_json) VALUES(?,?,?,?,?,?,?)',
        ('restored-child','restored-child-key','oem.x.move_steps',state,'1',2,'{"retained":"command evidence"}'))
    db.execute('INSERT INTO operator_plane_commands(command_id,stream_sequence,method_id,method_sequence,action_id,requested_json,effective_json,status,version,ownership_generation,queued_at,updated_at,terminal_json) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?)',
        ('restored-child',1,method_id,1,'oem.x.move_steps','{"steps":10}','{"steps":10}',state,1,7,1,2,'{"retained":"terminal evidence"}'))
    db.execute('INSERT INTO serial206_movement_commands(command_id,idempotency_key,action_id,method_id,method_order,board_scope_json,ownership_generation,expected_board_epochs_json,canonical_inputs_sha256,state,state_version,admitted_interrupt_epochs_json,accepted_at,queued_at) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?)',
        ('restored-child','restored-child-key','oem.x.move_steps',method_id,1,'{}',7,'{}','b'*64,state,1,'{}',1,1))
    db.execute('INSERT INTO operator_plane_idempotency(operation_kind,idempotency_key,fingerprint,method_id,response_json,created_at) VALUES(?,?,?,?,?,?)',
        ('method','restored-key','c'*64,method_id,'{"method_id":"restored-method"}',1))
    db.execute('INSERT INTO operator_plane_transitions(event_kind,command_id,method_id,state,payload_json,created_at) VALUES(?,?,?,?,?,?)',
        ('command_admitted','restored-child',method_id,state,'{"retained":true}',1))
    db.execute('INSERT INTO operator_plane_snapshots(token,method_id,watermark,expires_at) VALUES(?,?,?,?)', ('token',method_id,1,999))
    original = dict(db.execute('SELECT * FROM operator_plane_commands WHERE command_id=?', ('restored-child',)).fetchone())
    db.execute('INSERT INTO operator_plane_command_versions(command_id,source_sequence,row_json,versioned_at) VALUES(?,?,?,?)',
        ('restored-child',1,json.dumps(original),2))
    for _, sql in triggers:
        db.execute(sql)
    owner.verify_canonical_runtime_database(db, full_data_check=True)
    return original


@pytest.mark.parametrize('queued', [False, True])
def test_nonempty_restore_preserves_history_without_replay(tmp_path, monkeypatch, queued):
    root = tmp_path/'restored'
    old = legacy_store(root, monkeypatch)
    original = restore_method_fixture(old._db, queued=queued)
    old._db.execute("UPDATE sqlite_sequence SET seq=100 WHERE name='serial206_movement_commands'")
    versions = records(old._db, 'operator_plane_command_versions')
    prefix = records(old._db, 'runtime_schema_migrations')
    old.close()
    store = owner.OEMRuntimeStore(root)
    db = store._db
    assert db.execute('PRAGMA user_version').fetchone()[0] == 15
    assert db.execute("SELECT seq FROM sqlite_sequence WHERE name='serial206_movement_commands'").fetchone()[0] == 100
    assert [r for r in records(db, 'operator_plane_command_versions') if r[0] <= max(v[0] for v in versions)] == versions
    if queued:
        latest = db.execute("SELECT row_json FROM operator_plane_command_versions WHERE command_id='restored-child' ORDER BY version_sequence DESC LIMIT 1").fetchone()[0]
        assert json.loads(latest)['status'] == 'cancelled'
    assert [r for r in records(db, 'runtime_schema_migrations') if r[0] <= 14] == prefix
    assert db.execute('PRAGMA foreign_key_check').fetchall() == []
    assert db.execute('PRAGMA integrity_check').fetchone()[0] == 'ok'
    for table in migration.RETIRED_TABLES:
        assert db.execute('SELECT 1 FROM sqlite_master WHERE name=?',(table,)).fetchone() is None
    for table, removed in migration.REMOVED_COLUMNS.items():
        assert not removed & {r[1] for r in db.execute(f'PRAGMA table_info("{table}")')}
    history = migration.command_history(db, 'restored-child')
    assert history['original_command'] == original
    assert set(history['parents']) == {'operator_plane_methods','serial206_movement_methods'}
    assert json.loads(history['parents']['operator_plane_methods']['source_json'])['unknown'] is None
    assert db.execute("SELECT COUNT(*) FROM operator_plane_idempotency WHERE operation_kind='method'").fetchone()[0] == 0
    assert db.execute("SELECT receipt_json FROM operator_commands WHERE command_id='restored-child'").fetchone()[0] == '{"retained":"command evidence"}'
    assert db.execute("SELECT status FROM operator_plane_commands WHERE command_id='restored-child'").fetchone()[0] == ('cancelled' if queued else 'completed')
    with pytest.raises(sqlite3.IntegrityError, match='immutable'):
        db.execute('DELETE FROM runtime_retired_records')
    store.close()
    commands = OperatorCommandStore(root)
    try:
        receipt = commands.get_command('restored-child')
        assert receipt['retired_method_history'] == history
        assert receipt['method_id'] == 'restored-method'
        assert commands.get_command_summary('restored-child')['method_id'] == 'restored-method'
        feed = commands.transitions(after=0, limit=200)
        assert next(event for event in feed['events'] if event['command_id']=='restored-child')['method_id'] == 'restored-method'
        from bioxp.operator_receipt_store import OperatorHistoryReader
        reader = OperatorHistoryReader(root)
        try:
            assert reader.get_command('restored-child')['retired_method_history'] == history
        finally:
            reader.close()
        from bioxp.operator_history import read_history_page
        page, _ = read_history_page(root, 20)
        assert any(row['command_id'] == 'restored-child' for row in page)
        assert commands.claim_next() is None
        assert not hasattr(commands, 'admit_method')
    finally:
        commands.connection.close()
    reopened = owner.OEMRuntimeStore(root)
    owner.verify_canonical_runtime_database(reopened._db, full_data_check=True)
    assert migration.command_history(reopened._db, 'restored-child') == history
    reopened.close()
    # Retention cannot discard archived source associations or snapshots.
    assert retain_runtime_rows(root, as_of=1900000000)['removed_rows'] == 0


def test_fresh_schema_repeat_open_and_exact_manifest(tmp_path, monkeypatch):
    store = owner.OEMRuntimeStore(tmp_path)
    assert store._db.execute('PRAGMA user_version').fetchone()[0] == 15
    owner.verify_canonical_runtime_database(store._db, full_data_check=True)
    prefix = records(store._db, 'runtime_schema_migrations')
    assert max(prefix, key=lambda row: row[0])[1] == migration.migration_identity().name
    store.close()
    monkeypatch.setattr(owner, '_verified_sqlite_backup', lambda *a, **k: pytest.fail('stable reopen backed up'))
    for _ in range(2):
        store = owner.OEMRuntimeStore(tmp_path)
        assert records(store._db, 'runtime_schema_migrations') == prefix
        store.close()


def test_retained_v14_copy_migration_preserves_every_command_row(tmp_path):
    source_name = os.environ.get('METHOD_RETIREMENT_RETAINED_DB')
    if not source_name:
        pytest.skip('set METHOD_RETIREMENT_RETAINED_DB to immutable captured v14 database')
    source = Path(source_name)
    source_digest = hashlib.sha256(source.read_bytes()).hexdigest()
    root = tmp_path/'capture';root.mkdir()
    shutil.copyfile(source, root/'bioxp_runtime.db')
    with sqlite3.connect('file:'+str(source)+'?mode=ro',uri=True) as db:
        assert db.execute('PRAGMA user_version').fetchone()[0] == 14
        tables = [row[0] for row in db.execute("SELECT name FROM sqlite_master WHERE type='table' AND name NOT LIKE 'sqlite_%'")
                  if row[0] not in migration.RETIRED_TABLES | {'runtime_schema_migrations','runtime_store_identity'}]
        before = {t: records(db,t,migration.REMOVED_COLUMNS.get(t,())) for t in tables}
        prefix = records(db,'runtime_schema_migrations')
    store = owner.OEMRuntimeStore(root)
    owner.verify_canonical_runtime_database(store._db,full_data_check=True)
    for table in tables:
        assert records(store._db,table) == before[table], table
    assert [r for r in records(store._db,'runtime_schema_migrations') if r[0] <= 14] == prefix
    store.close()
    reopened = owner.OEMRuntimeStore(root)
    owner.verify_canonical_runtime_database(reopened._db,full_data_check=True)
    reopened.close()
    assert hashlib.sha256(source.read_bytes()).hexdigest() == source_digest


def test_failed_upgrade_rolls_back_all_history_and_schema(tmp_path, monkeypatch):
    store = legacy_store(tmp_path, monkeypatch)
    restore_method_fixture(store._db)
    before = records(store._db, 'operator_plane_commands')
    ledger = records(store._db, 'runtime_schema_migrations')
    store.close()
    actual = migration.apply
    def fail_after_rebuild(db):
        actual(db)
        raise RuntimeError('injected migration failure')
    with monkeypatch.context() as scoped:
        scoped.setattr(migration, 'apply', fail_after_rebuild)
        with pytest.raises(RuntimeError, match='injected migration failure'):
            owner.OEMRuntimeStore(tmp_path)
    db = sqlite3.connect(tmp_path/'bioxp_runtime.db')
    assert db.execute('PRAGMA user_version').fetchone()[0] == 14
    assert records(db, 'operator_plane_commands') == before
    assert records(db, 'runtime_schema_migrations') == ledger
    assert db.execute('SELECT COUNT(*) FROM operator_plane_methods').fetchone()[0] == 1
    assert db.execute('PRAGMA foreign_key_check').fetchall() == []
    db.close()
    upgraded = owner.OEMRuntimeStore(tmp_path)
    assert upgraded._db.execute('PRAGMA user_version').fetchone()[0] == 15
    upgraded.close()
