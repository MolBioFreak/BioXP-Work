"""Normal V12 migration on a read-only captured actual retained database copy."""
import hashlib
import json
import os
import shutil
import sqlite3
from pathlib import Path


def test_actual_retained_normal_v12_migration(tmp_path):
    from bioxp.oem_runtime_store import OEMRuntimeStore, verify_canonical_runtime_database
    import pytest
    captured = os.environ.get('DECK_CAPTURED_V11_SOURCE')
    if not captured:
        pytest.skip('requires an explicitly supplied read-only captured V11 database')
    source = Path(captured)
    def file_digest(path):
        with path.open('rb') as handle:
            return hashlib.file_digest(handle, 'sha256').hexdigest()
    original = file_digest(source)
    target = tmp_path / 'actual-retained'
    target.mkdir()
    shutil.copyfile(source, target / 'bioxp_runtime.db')
    def records(connection):
        tables = [r[0] for r in connection.execute("SELECT name FROM sqlite_master WHERE type='table' AND name NOT LIKE 'sqlite_%' ORDER BY name")]
        result = {}
        for table in tables:
            if table in {'runtime_schema_migrations', 'runtime_store_identity'}:
                continue
            digest = hashlib.sha256(); count = 0
            for row in connection.execute('SELECT * FROM "'+table+'"'):
                digest.update(repr(tuple(row)).encode()); digest.update(b'\n'); count += 1
            result[table] = {'count': count, 'sha256': digest.hexdigest()}
        return result
    with sqlite3.connect('file:'+str(source)+'?mode=ro', uri=True) as ro:
        ro.execute('PRAGMA query_only=ON')
        assert ro.execute('PRAGMA user_version').fetchone()[0] == 11
        before = records(ro)
        prefix = ro.execute('SELECT * FROM runtime_schema_migrations ORDER BY version').fetchall()
        objects = dict(ro.execute("SELECT name,sql FROM sqlite_master WHERE sql IS NOT NULL"))
    store = OEMRuntimeStore(target)
    connection = store.connection if hasattr(store, 'connection') else store._db
    assert connection.execute('PRAGMA user_version').fetchone()[0] == 12
    verify_canonical_runtime_database(connection, full_data_check=True)
    assert records(connection) == before
    assert [tuple(r) for r in connection.execute('SELECT * FROM runtime_schema_migrations WHERE version<=11 ORDER BY version')] == prefix
    after_objects = dict(connection.execute("SELECT name,sql FROM sqlite_master WHERE sql IS NOT NULL"))
    changed = [name for name in objects if objects[name] != after_objects[name]]
    assert set(objects) == set(after_objects)
    assert changed == ['operator_plane_deck_recovery_decisions_authorized_insert_v1']
    migration = dict(connection.execute('SELECT * FROM runtime_schema_migrations WHERE version=12').fetchone())
    store.close()
    reopened = OEMRuntimeStore(target)
    verify_canonical_runtime_database(reopened._db, full_data_check=True)
    assert records(reopened._db) == before
    reopened.close()
    assert file_digest(source) == original
    Path(os.environ['BIOXP_WORKFLOW_EXPORT']+'.retained-v12.json').write_text(json.dumps({
        'source': str(source), 'source_sha256': original, 'source_unchanged': True,
        'from_version': 11, 'to_version': 12, 'changed_schema_objects': changed,
        'all_operational_tables_unchanged': before, 'migration': migration,
        'normal_reopen_verified': True}, indent=2))
