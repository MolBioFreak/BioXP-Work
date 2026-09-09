"""Disposable SQLite v7 contracts, including a store built by unmodified parent code."""
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys

import pytest
import bioxp.oem_runtime_store as runtime
from bioxp.oem_deck_schema_v6 import verify_deck_schema_v6
from bioxp.oem_deck_schema_v7 import GROUP_TRIGGER_NAME, verify_deck_schema_v7
from bioxp.operator_command_plane import OperatorCommandStore
from test_r4_deck_producer_refresh import owners, complete_predecessor
from test_r4_named_destination_vectors import rig

PARENT = Path('/home/dalab/robot/rectify-oem-completion')
SEED_SCRIPT = '''
import sys, json
from pathlib import Path
from bioxp.oem_runtime_store import OEMRuntimeStore, canonical_runtime_migration_registry
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider as Provider
from test_oem_deck_install_binding import FakeRuntimeStore, FakeDeckPrimitives, _operator_store
from test_r4_deck_producer_refresh import complete_predecessor
root = Path(sys.argv[1])
store = _operator_store(root)
state = Provider._new_state()
state['x_lifecycle']['board_lifecycle_generation'] = 11
owner = FakeRuntimeStore(state)
provider = Provider(FakeDeckPrimitives(), state_store=owner, generation_provider=lambda: 7)
provider.bind_deck_semantic_state_reader(store.deck_semantic_state)
provider.bind_deck_semantic_state_publisher(store.publish_deck_owner_state)
complete_predecessor(owner, channel=3)
store.bootstrap_deck_semantic_state(provider.deck_semantic_bootstrap_snapshot(expected_generation=7))
store.publish_deck_owner_state(source_operation='pipette_owner', source_command_id='seed-v6-owner',
    updates={'tip_loaded': True, 'tip_dirty': True, 'tip_location': 2},
    **provider.deck_owner_authority_stamps())
print(json.dumps([(i.version,i.name,i.ddl_sha256) for i in canonical_runtime_migration_registry()]))
store.stop()
'''

@pytest.fixture
def old_v6(tmp_path):
    env = dict(os.environ, PYTHONPATH=f'{PARENT}/src:{PARENT}/tests:{PARENT}', PYTHONDONTWRITEBYTECODE='1')
    result = subprocess.run([sys.executable, '-c', SEED_SCRIPT, str(tmp_path)],
                            cwd=PARENT, env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    identities = json.loads(result.stdout.strip().splitlines()[-1])
    assert identities == [[i.version, i.name, i.ddl_sha256]
                          for i in runtime.canonical_runtime_migration_registry()[:6]]
    with sqlite3.connect(tmp_path / 'bioxp_runtime.db') as db:
        assert db.execute('PRAGMA user_version').fetchone()[0] == 6
        verify_deck_schema_v6(db)
    return tmp_path


def snapshot(db):
    tables = [r[0] for r in db.execute("SELECT name FROM sqlite_master WHERE type='table'")]
    return {t: sorted((tuple(r) for r in db.execute(f'SELECT * FROM "{t}"')), key=repr)
            for t in tables}


def test_real_parent_v6_preserved_upgraded_and_reopened(old_v6):
    with sqlite3.connect(old_v6 / 'bioxp_runtime.db') as db:
        before = snapshot(db)
        old_schema = dict(db.execute("SELECT name,sql FROM sqlite_master WHERE sql IS NOT NULL"))
    store = runtime.OEMRuntimeStore(old_v6)
    runtime.verify_canonical_runtime_database(store._db)
    after = snapshot(store._db)
    assert set(before) == set(after)
    for table in before.keys() - {'runtime_store_identity', 'runtime_schema_migrations'}:
        assert after[table] == before[table], table
    assert after['runtime_schema_migrations'][:6] == before['runtime_schema_migrations']
    assert len(after['runtime_schema_migrations']) == 7
    schema = dict(store._db.execute("SELECT name,sql FROM sqlite_master WHERE sql IS NOT NULL"))
    assert {k for k in schema if schema[k] != old_schema[k]} == {GROUP_TRIGGER_NAME}
    assert store._db.execute('PRAGMA integrity_check').fetchone()[0] == 'ok'
    assert store._db.execute('PRAGMA foreign_key_check').fetchall() == []
    store.close()
    operator = OperatorCommandStore(old_v6)
    try:
        state = operator.deck_semantic_state()
        operator.publish_deck_owner_state(
            source_operation='pipette_owner', source_command_id='after-v7-group',
            updates={'tip_loaded': True, 'tip_dirty': True, 'tip_location': -1},
            ownership_generation=state['ownership_generation'],
            board_epoch_4=state['board_epoch_4'], board_epoch_5=state['board_epoch_5'],
        )
    finally:
        operator.stop()
    reopened = OperatorCommandStore(old_v6)
    try:
        assert reopened.deck_semantic_state()['tip_location'] == -1
        assert reopened.deck_semantic_state()['tip_loaded'] is True
        runtime.verify_canonical_runtime_database(reopened.connection)
    finally:
        reopened.stop()


@pytest.mark.parametrize('failure_point', ['apply', 'record', 'verify'])
def test_upgrade_failure_rolls_back_schema_ledger_identity_and_rows(old_v6, monkeypatch, failure_point):
    with sqlite3.connect(old_v6 / 'bioxp_runtime.db') as db:
        before = snapshot(db)
        schema = list(db.execute('SELECT type,name,sql FROM sqlite_master ORDER BY name'))
    if failure_point == 'apply':
        original = runtime._deck_v7.apply_deck_schema_v7
        def fail(db):
            original(db)
            raise RuntimeError('injected-v7')
        monkeypatch.setattr(runtime._deck_v7, 'apply_deck_schema_v7', fail)
    elif failure_point == 'record':
        original = runtime._record_runtime_migration
        def fail(db, **kw):
            original(db, **kw)
            if kw['identity'].version == 7:
                raise RuntimeError('injected-v7')
        monkeypatch.setattr(runtime, '_record_runtime_migration', fail)
    else:
        original = runtime.verify_canonical_runtime_database
        def fail(db, **kw):
            original(db, **kw)
            if db.execute('PRAGMA user_version').fetchone()[0] == 7:
                raise RuntimeError('injected-v7')
        monkeypatch.setattr(runtime, 'verify_canonical_runtime_database', fail)
    with pytest.raises(RuntimeError, match='injected-v7'):
        runtime.OEMRuntimeStore(old_v6)
    with sqlite3.connect(old_v6 / 'bioxp_runtime.db') as db:
        assert snapshot(db) == before
        assert list(db.execute('SELECT type,name,sql FROM sqlite_master ORDER BY name')) == schema
        assert db.execute('PRAGMA user_version').fetchone()[0] == 6
        assert db.execute('PRAGMA integrity_check').fetchone()[0] == 'ok'
        verify_deck_schema_v6(db)
    target, symbol = {
        'apply': (runtime._deck_v7, 'apply_deck_schema_v7'),
        'record': (runtime, '_record_runtime_migration'),
        'verify': (runtime, 'verify_canonical_runtime_database'),
    }[failure_point]
    monkeypatch.setattr(target, symbol, original)
    runtime.OEMRuntimeStore(old_v6).close()


@pytest.mark.parametrize('corruption', ['trigger', 'ledger', 'index', 'foreign_key', 'loaded_null'])
def test_corrupt_v6_refused_without_repair(old_v6, corruption):
    with sqlite3.connect(old_v6 / 'bioxp_runtime.db') as db:
        if corruption == 'trigger':
            db.execute(f'DROP TRIGGER {GROUP_TRIGGER_NAME}')
            db.execute(f'CREATE TRIGGER {GROUP_TRIGGER_NAME} BEFORE UPDATE ON operator_plane_deck_semantic_state BEGIN SELECT 1; END')
        elif corruption == 'ledger':
            for name, in db.execute("SELECT name FROM sqlite_master WHERE type='trigger' AND tbl_name='runtime_schema_migrations'").fetchall():
                db.execute(f'DROP TRIGGER "{name}"')
            db.execute("UPDATE runtime_schema_migrations SET ddl_sha256=? WHERE version=6", ('0'*64,))
        elif corruption == 'index':
            db.execute('CREATE INDEX counterfeit ON operator_plane_deck_semantic_state(tip_location)')
        else:
            db.create_function('authority_write_allowed', 0, lambda: 1)
            db.create_function('canonical_json', 1, lambda x: x)
            # Inject a preexisting FK violation while restoring exact trigger SQL.
            sql = db.execute('SELECT sql FROM sqlite_master WHERE name=?', (GROUP_TRIGGER_NAME,)).fetchone()[0]
            db.execute(f'DROP TRIGGER {GROUP_TRIGGER_NAME}')
            if corruption == 'foreign_key':
                db.execute("UPDATE operator_plane_deck_semantic_state SET producer_command_id='missing-command'")
            else:
                db.execute("UPDATE operator_plane_deck_semantic_state SET tip_location=NULL")
            db.execute(sql)
    with sqlite3.connect(old_v6 / 'bioxp_runtime.db') as db:
        before = snapshot(db)
        schema = list(db.execute('SELECT type,name,sql FROM sqlite_master ORDER BY name'))
    with pytest.raises(RuntimeError):
        runtime.OEMRuntimeStore(old_v6)
    with sqlite3.connect(old_v6 / 'bioxp_runtime.db') as db:
        assert snapshot(db) == before
        assert list(db.execute('SELECT type,name,sql FROM sqlite_master ORDER BY name')) == schema
        assert db.execute('PRAGMA user_version').fetchone()[0] == 6


@pytest.mark.parametrize('value', [-2, 4, 99, None, 1.5, 'invalid'])
def test_loaded_sql_domain_rejects_invalid_and_null_atomically(owners, value):
    provider, owner, store = owners
    complete_predecessor(owner)
    provider._canonical_deck_semantic_state()
    before = snapshot(store.connection)
    with store._authority_write():
        with pytest.raises(sqlite3.IntegrityError, match='unauthorized or incoherent'):
            store.connection.execute(
                'UPDATE operator_plane_deck_semantic_state SET tip_location=?, semantic_state_revision=semantic_state_revision+1',
                (value,))
    assert snapshot(store.connection) == before


def test_group_authorization_revision_and_append_only_guards(owners):
    provider, owner, store = owners
    complete_predecessor(owner, channel=-1)
    provider._canonical_deck_semantic_state()
    for authorized, sql in [
        (False, 'UPDATE operator_plane_deck_semantic_state SET semantic_state_revision=semantic_state_revision+1'),
        (True, 'UPDATE operator_plane_deck_semantic_state SET tip_location=-1'),
        (True, 'DELETE FROM operator_plane_deck_semantic_state'),
        (True, 'DELETE FROM operator_plane_deck_semantic_transitions'),
        (True, 'UPDATE operator_plane_deck_semantic_transitions SET created_at=0'),
    ]:
        before = snapshot(store.connection)
        if authorized:
            with store._authority_write(), pytest.raises(sqlite3.IntegrityError):
                store.connection.execute(sql)
        else:
            with pytest.raises(sqlite3.IntegrityError):
                store.connection.execute(sql)
        assert snapshot(store.connection) == before
    verify_deck_schema_v7(store.connection)


def test_loaded_missing_location_rejected_without_publication(owners):
    provider, owner, store = owners
    complete_predecessor(owner)
    provider._canonical_deck_semantic_state()
    with store._authority_write():
        store.connection.execute(
            'UPDATE operator_plane_deck_semantic_state SET tip_loaded=0, tip_location=NULL, '
            'semantic_state_revision=semantic_state_revision+1')
    before = snapshot(store.connection)
    with store._authority_write(), pytest.raises(sqlite3.IntegrityError):
        store.connection.execute(
            'UPDATE operator_plane_deck_semantic_state SET tip_loaded=1, '
            'semantic_state_revision=semantic_state_revision+1')
    with pytest.raises(ValueError, match='valid tip location'):
        store.publish_deck_owner_state(
            source_operation='pipette_owner', source_command_id='missing-tip-location',
            updates={'tip_loaded': True}, **provider.deck_owner_authority_stamps())
    assert snapshot(store.connection) == before


@pytest.mark.parametrize('value', [-2, 4, None, 1.5, True, '1'])
def test_owner_invalid_domain_cannot_append_history(owners, value):
    provider, owner, store = owners
    complete_predecessor(owner)
    provider._canonical_deck_semantic_state()
    before = snapshot(store.connection)
    with pytest.raises(ValueError, match='source domain'):
        store.publish_deck_owner_state(
            source_operation='pipette_owner', source_command_id='invalid-location',
            updates={'tip_loaded': True, 'tip_location': value},
            **provider.deck_owner_authority_stamps())
    assert snapshot(store.connection) == before


def test_v6_active_command_requires_quiescence(old_v6):
    with sqlite3.connect(old_v6 / 'bioxp_runtime.db') as db:
        db.row_factory = sqlite3.Row
        db.create_function('authority_write_allowed', 0, lambda: 1)
        row = dict(db.execute('SELECT * FROM operator_plane_commands LIMIT 1').fetchone())
        row.update(command_id='quiescence-fixture', status='queued', version=1,
                   stream_sequence=1000, dispatched_at=None, finished_at=None, terminal_json=None)
        columns = ','.join(row)
        db.execute(f"INSERT INTO operator_plane_commands({columns}) VALUES({','.join('?' for _ in row)})",
                   tuple(row.values()))
    with sqlite3.connect(old_v6 / 'bioxp_runtime.db') as db:
        before = snapshot(db)
    with pytest.raises(RuntimeError, match='quiesced operator mutation admission'):
        runtime.OEMRuntimeStore(old_v6)
    with sqlite3.connect(old_v6 / 'bioxp_runtime.db') as db:
        assert snapshot(db) == before
        assert db.execute('PRAGMA user_version').fetchone()[0] == 6
        verify_deck_schema_v6(db)


def test_v7_identity_binds_actual_trigger(monkeypatch):
    before = runtime.oem_deck_schema_v7_migration_identity()
    monkeypatch.setattr(runtime._deck_v7, 'DECK_SCHEMA_V7_TRIGGER_SQL',
                        runtime._deck_v7.DECK_SCHEMA_V7_TRIGGER_SQL + '\n-- sentinel')
    assert runtime.oem_deck_schema_v7_migration_identity().ddl_sha256 != before.ddl_sha256
