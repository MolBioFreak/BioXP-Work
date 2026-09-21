"""Compact polling must preserve V2 fields without expanding full evidence."""
import json
import sqlite3
import threading

import pytest
from bioxp.operator_command_plane import OperatorCommandStore


@pytest.fixture
def store():
    s = object.__new__(OperatorCommandStore)
    s._lock = threading.RLock()
    s.connection = sqlite3.connect(':memory:')
    s.connection.row_factory = sqlite3.Row
    s.connection.executescript('''
    CREATE TABLE operator_plane_commands(command_id TEXT PRIMARY KEY,method_id TEXT,method_sequence INT,
      stream_sequence INT, action_id TEXT,status TEXT,ownership_generation INT, queued_at REAL,
      dispatched_at REAL, finished_at REAL,source_noop INT,source_noop_reason TEXT, remote_acknowledged INT,
      controller_acknowledged INT,physical_effect_verified INT,version INT,requested_json TEXT,effective_json TEXT,terminal_json TEXT);
    CREATE TABLE serial206_movement_commands(command_id TEXT,sequence INT,state TEXT,state_version INT,
      expected_board_epochs_json TEXT,terminal_receipt_id TEXT);
    CREATE TABLE operator_plane_transitions(command_id TEXT,transition_sequence INT);
    CREATE TABLE operator_plane_deck_commands(command_id TEXT,ambiguity_state TEXT);
    ''')
    s._deck_command_detail = lambda cid: {'ambiguity_state': s.connection.execute(
        'SELECT ambiguity_state FROM operator_plane_deck_commands WHERE command_id=?',(cid,)).fetchone()[0]}
    yield s
    s.connection.close()


@pytest.mark.parametrize('status,canonical,explicit,ambiguity', [
    ('queued', 'queued', None, 'none'), ('dispatched', 'dispatched', None, 'none'),
    ('completed', 'completed', 'completed', 'none'), ('failed', 'failed', None, 'none'),
    ('ambiguous', 'ambiguous', None, 'recovery_required'),
    ('ambiguous', 'ambiguous', 'operator_reconciled', 'recovery_required'),
    ('dispatched', 'ambiguous', None, 'recovery_required'),
    ('cleared', 'cleared', None, 'none')])
def test_summary_matches_every_compact_field_without_loading_payloads(store, status, canonical, explicit, ambiguity):
    finished = None if status in {'queued', 'dispatched'} else 3.
    large = json.dumps({'payload': 'x' * 100_000})
    terminal = json.dumps({'completion_class': explicit, 'payload': 'x' * 100_000})
    store.connection.execute('INSERT INTO operator_plane_commands VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)',
        ('c', None, None, 7, 'oem.deck.move_to_location', status, 1, 1., 2., finished, 0, None, 0, 1, 0, 3, large, large, terminal))
    store.connection.execute('INSERT INTO serial206_movement_commands VALUES(?,?,?,?,?,?)',
        ('c', 8, canonical, 4, '{"4":76,"5":2}', 'receipt'))
    store.connection.execute('INSERT INTO operator_plane_deck_commands VALUES(?,?)', ('c', ambiguity))
    full = store.get_command('c')
    def forbidden(*args): raise AssertionError('compact poll expanded evidence')
    store._deck_command_detail = forbidden
    summary = store.get_command_summary('c')
    keys = ('command_id','action_id','status','sequence','method_id','ownership_generation',
            'expected_board_epoch_by_board','state_version','accepted_at','queued_at','dispatched_at',
            'finished_at','terminal_receipt_id','completion_class','physical_effect_verified')
    assert {k: summary[k] for k in keys} == {k: full[k] for k in keys}
    assert len(json.dumps(summary)) < 2000
    assert store.get_command_summary('absent') is None
