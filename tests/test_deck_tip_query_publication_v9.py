"""Governed retained V8 -> V9 migration; all execution stays offline."""
import json
import os
from pathlib import Path
import shutil
import sqlite3
import subprocess
import sys

import pytest

from tests.test_deck_tip_query_publication import query_rig, query, named_move, reopen
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_scoped_authority import retained_rig


@pytest.mark.parametrize('source', ['prepared_v8', 'captured_31ac14c_v8'])
def test_retained_v8_migration_preserves_rows_and_exact_constraints(tmp_path, source):
    from bioxp.oem_runtime_store import OEMRuntimeStore, verify_canonical_runtime_database
    from bioxp import oem_deck_schema_v9 as v9
    root = tmp_path / 'migration'
    shutil.copytree(os.environ['DECK_RETAINED_BASELINE'], root)
    # The exact frozen pre-repair production owner, not a synthetic CREATE TABLE
    # fixture or a modified candidate registry, prepares the real V8 prefix.
    script = '''import importlib.util,sys,json
import bioxp
spec=importlib.util.spec_from_file_location('bioxp.oem_runtime_store',sys.argv[1])
module=importlib.util.module_from_spec(spec)
sys.modules[spec.name]=module
spec.loader.exec_module(module)
bioxp.oem_runtime_store=module
store=module.OEMRuntimeStore(sys.argv[2])
print(json.dumps({'version':store._db.execute('PRAGMA user_version').fetchone()[0]}))
store.close()
'''
    if source == 'prepared_v8':
        result = json.loads(subprocess.check_output([sys.executable, '-c', script,
            os.environ['DECK_V8_STORE_SOURCE'], str(root)], text=True))
        assert result['version'] == 8
    else:
        import hashlib
        captured = Path(os.environ['DECK_CAPTURED_V8_SOURCE'])
        assert hashlib.sha256(captured.read_bytes()).hexdigest() == '36b3be441abf2e22a17ded583c70e357bc3182e081d0f546f25daac44fb864b0'
        # Copy captured bytes and sidecars to disposable storage before opening;
        # never let SQLite alter the parent's stopped evidence.
        for suffix in ('', '-wal', '-shm'):
            target = root / ('bioxp_runtime.db' + suffix)
            target.unlink(missing_ok=True)
            original = Path(str(captured) + suffix)
            if original.exists():
                shutil.copyfile(original, target)
    db = sqlite3.connect(root / 'bioxp_runtime.db')
    assert db.execute('PRAGMA user_version').fetchone()[0] == 8
    tables = [row[0] for row in db.execute("SELECT name FROM sqlite_master WHERE type='table' AND name NOT LIKE 'sqlite_%'")]
    excluded = {'runtime_schema_migrations', 'runtime_store_identity'}
    before = {table: db.execute('SELECT * FROM "' + table + '"').fetchall()
              for table in tables if table not in excluded}
    ledger = db.execute('SELECT * FROM runtime_schema_migrations ORDER BY version').fetchall()
    schema = dict(db.execute("SELECT name,sql FROM sqlite_master WHERE sql IS NOT NULL"))
    db.close()
    runtime = OEMRuntimeStore(root)
    db = runtime._db
    assert db.execute('PRAGMA user_version').fetchone()[0] == 9
    verify_canonical_runtime_database(db, full_data_check=True)
    assert {table: [tuple(row) for row in db.execute('SELECT * FROM "' + table + '"').fetchall()]
            for table in before} == before
    assert [tuple(row) for row in db.execute('SELECT * FROM runtime_schema_migrations WHERE version<=8 ORDER BY version')] == ledger
    after_schema = dict(db.execute("SELECT name,sql FROM sqlite_master WHERE sql IS NOT NULL"))
    assert set(schema) == set(after_schema)
    assert [key for key in schema if schema[key] != after_schema[key]] == [v9.TRIGGER]
    assert v9.NEW in after_schema[v9.TRIGGER]
    with pytest.raises(sqlite3.DatabaseError):
        db.execute('UPDATE operator_plane_deck_semantic_state SET tip_loaded=1,tip_location=NULL')
    runtime.close()
    # Genuine startup uses the prepared V9 metadata-only route repeatedly.
    runtime = OEMRuntimeStore(root)
    verify_canonical_runtime_database(runtime._db, full_data_check=True)
    runtime.close()


def test_loaded_unknown_survives_fresh_process_and_never_falls_back_false(query_rig):
    app, provider, _, _, root, _, _, wire, _ = query_rig
    named_move(query_rig)
    query(query_rig)
    wire['data'][2] = [32, 96, 49]
    result = query(query_rig, 'loaded-after-absent-v9')
    assert result['deck_state_publication']['status'] == 'published'
    after = app.state.operator_command_plane.store.deck_semantic_state()
    assert (after['tip_loaded'], after['tip_dirty'], after['tip_location']) == (True, None, None)
    offset = provider.deck_authority_snapshot(expected_generation=int(provider.generation_provider()), target='LOC_OC')
    assert offset['tip_loaded'] is True
    script = ('import json,sys; from tests.test_deck_tip_query_publication import reopen; '
              'print(json.dumps(reopen(sys.argv[1],sys.argv[2],int(sys.argv[3]))))')
    fresh = json.loads(subprocess.check_output([sys.executable, '-c', script,
        str(root), result['receipt_id'], str(provider.generation_provider())], text=True))
    assert fresh['semantic'] == after
    assert 'tip_dirty' in fresh['park_blocker']
    assert fresh['receipt']['result']['channels'][2]['tip_loaded'] is True
