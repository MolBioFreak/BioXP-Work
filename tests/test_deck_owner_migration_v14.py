"""V13 retained rows/ledger survive the two-trigger V14 delta unchanged."""
import hashlib
import os
from pathlib import Path
import sqlite3

import pytest

from bioxp import oem_deck_schema_v14 as migration
from bioxp.oem_runtime_store import (
    OEMRuntimeStore, canonical_runtime_schema_manifest,
    _migrate_runtime_database_through_v14 as migrate_runtime_database_v2,
    verify_canonical_runtime_database, _runtime_physical_schema_sha256,
    _RUNTIME_PHYSICAL_SCHEMA_SHA256_BY_VERSION,
)


@pytest.fixture(autouse=True)
def v14_migration_boundary(monkeypatch):
    # Exercise this historical migration in isolation; V15 has its own suite.
    from bioxp import oem_runtime_store as owner
    monkeypatch.setattr(owner, 'migrate_runtime_database_v2', owner._migrate_runtime_database_through_v14)


def records(db):
    return {name: sorted((tuple(row) for row in db.execute('SELECT * FROM "'+name+'"')), key=repr)
        for (name,) in db.execute("SELECT name FROM sqlite_master WHERE type='table' AND name NOT LIKE 'sqlite_%'")
        if name not in {'runtime_schema_migrations', 'runtime_store_identity'}}


def test_retained_v13_rows_ledger_triggers_and_reopen(tmp_path):
    baseline = os.environ.get('DECK_RETAINED_BASELINE')
    if not baseline:
        pytest.skip('requires retained V13 DECK_RETAINED_BASELINE')
    source = Path(baseline) / 'bioxp_runtime.db'
    original = hashlib.sha256(source.read_bytes()).hexdigest()
    target = tmp_path / 'retained'
    target.mkdir()
    with sqlite3.connect('file:'+str(source)+'?mode=ro', uri=True) as ro:
        assert ro.execute('PRAGMA user_version').fetchone()[0] == 13
        before = records(ro)
        prefix = ro.execute('SELECT * FROM runtime_schema_migrations ORDER BY version').fetchall()
        objects = dict(ro.execute('SELECT name,sql FROM sqlite_master WHERE sql IS NOT NULL'))
        with sqlite3.connect(target / 'bioxp_runtime.db') as copy:
            ro.backup(copy)
    store = OEMRuntimeStore(target)
    try:
        db = store._db
        assert db.execute('PRAGMA user_version').fetchone()[0] == 14
        verify_canonical_runtime_database(db, full_data_check=True)
        assert records(db) == before
        assert [tuple(row) for row in db.execute('SELECT * FROM runtime_schema_migrations WHERE version<=13 ORDER BY version')] == prefix
        after = dict(db.execute('SELECT name,sql FROM sqlite_master WHERE sql IS NOT NULL'))
        assert set(after) == set(objects)
        assert {name for name in objects if objects[name] != after[name]} == {
            migration.DELIVERY_TRIGGER, migration.BACKGROUND_TRIGGER}
        assert _runtime_physical_schema_sha256(db) == _RUNTIME_PHYSICAL_SCHEMA_SHA256_BY_VERSION[14]
        ledger = [tuple(row) for row in db.execute('SELECT * FROM runtime_schema_migrations ORDER BY version')]
        assert db.execute('SELECT name FROM runtime_schema_migrations WHERE version=14').fetchone()[0] == migration.migration_identity().name
        migrate_runtime_database_v2(db, target)
        migration.migrate(db, target, migration.migration_identity())
        assert [tuple(row) for row in db.execute('SELECT * FROM runtime_schema_migrations ORDER BY version')] == ledger
    finally:
        store.close()
    reopened = OEMRuntimeStore(target)
    try:
        verify_canonical_runtime_database(reopened._db, full_data_check=True)
        assert records(reopened._db) == before
        assert [tuple(row) for row in reopened._db.execute('SELECT * FROM runtime_schema_migrations ORDER BY version')] == ledger
    finally:
        reopened.close()
    assert hashlib.sha256(source.read_bytes()).hexdigest() == original


def test_fresh_v14_manifest_and_frozen_v8_identity(tmp_path):
    from bioxp import oem_deck_schema_v8 as v8
    store = OEMRuntimeStore(tmp_path / 'fresh')
    try:
        assert store._db.execute('PRAGMA user_version').fetchone()[0] == 14
        verify_canonical_runtime_database(store._db, full_data_check=True)
        old = canonical_runtime_schema_manifest(version=13)
        new = canonical_runtime_schema_manifest(version=14)
        assert set(old) == set(new)
        assert {key for key in old if old[key] != new[key]} == {
            ('trigger', migration.DELIVERY_TRIGGER), ('trigger', migration.BACKGROUND_TRIGGER)}
        for trigger in (migration.DELIVERY_TRIGGER, migration.BACKGROUND_TRIGGER):
            assert 'semantic' not in new['trigger', trigger].lower()
            assert 'deck_owner_authority_current' in new['trigger', trigger].lower()
        assert store._db.execute('SELECT ddl_sha256 FROM runtime_schema_migrations WHERE version=8').fetchone()[0] == v8.migration_identity().ddl_sha256
    finally:
        store.close()
