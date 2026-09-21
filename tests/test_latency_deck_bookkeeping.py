"""Pure planning and host-only stage publication; no OEM fixture substitutes."""
import hashlib
import json
import sqlite3
import threading
from contextlib import nullcontext

import pytest
from bioxp.oem_deck_catalog import DeckCatalog, configured_location_names, _catalog_revision
from bioxp.oem_compat.position_table import PositionTable
from bioxp.oem_deck_movement import DeckPlanStep
from bioxp.operator_command_plane import OperatorCommandStore


def table():
    return PositionTable.from_rows([{'locationID': n, 'x': 10, 'y': 20, 'zHigh': 30}
                                   for n in configured_location_names()])


def test_cached_catalog_has_exact_original_digest_and_detached_outputs():
    _catalog_revision.cache_clear()
    t = table()
    first = DeckCatalog.from_position_table(t)
    canonical = {'schema_version': 'bioxp.oem_deck_catalog.v1', 'position_table_sha256': t.digest, 'rows': first.rows()}
    assert first.revision == hashlib.sha256(json.dumps(canonical, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    second = DeckCatalog.from_position_table(t)
    assert second is not first and second.revision == first.revision
    assert _catalog_revision.cache_info().hits == 1
    rows = first.rows()
    rows[0]['aliases'].append('foreign')
    assert 'foreign' not in second.rows()[0]['aliases']
    t.digest = 'changed-revision'
    assert DeckCatalog.from_position_table(t).revision != first.revision


def test_cached_catalog_still_validates_required_targets():
    DeckCatalog.from_position_table(table())
    incomplete = PositionTable.from_rows([{'locationID': 'LOC_MS'}])
    with pytest.raises(ValueError, match='destination absent'):
        DeckCatalog.from_position_table(incomplete)


@pytest.mark.parametrize('fail', [False, True])
def test_predicate_records_share_one_commit_and_rollback_together(fail):
    store = object.__new__(OperatorCommandStore)
    store._lock = threading.RLock()
    store._authority_write = nullcontext
    store._deck_owner_authority_scope = nullcontext
    db = store.connection = sqlite3.connect(':memory:', isolation_level=None)
    db.executescript('''CREATE TABLE operator_plane_deck_stages(command_id TEXT,stage_order INT,
      terminal_state TEXT,terminal_evidence_json TEXT);
      INSERT INTO operator_plane_deck_stages VALUES('c',1,'planned',NULL),('c',2,'planned',NULL);''')
    sql = []
    db.set_trace_callback(sql.append)
    try:
        try:
            with store.deck_predicate_publication_scope():
                for order in (1, 2):
                    if fail and order == 2: raise RuntimeError('injected second-record failure')
                    step = DeckPlanStep(order, 'check_latch_status', 'test-source')
                    store.terminalize_deck_stage('c', step, state='completed', result={'value': True})
        except RuntimeError:
            assert fail
        rows = db.execute('SELECT terminal_state,terminal_evidence_json FROM operator_plane_deck_stages ORDER BY stage_order').fetchall()
        assert all(row[0] == ('planned' if fail else 'completed') for row in rows)
        assert sum(s == 'BEGIN IMMEDIATE' for s in sql) == 1
        assert sum(s == 'COMMIT' for s in sql) == (0 if fail else 1)
        if fail: assert all(row[1] is None for row in rows)
    finally:
        db.close()
