"""Populate real SQLite history tables; never mock the reader/paginator."""
import json
import sqlite3
from contextlib import closing


def seed_history(app, direct, retained):
    store = app.state.operator_receipt_store
    for raw in sorted(direct, key=lambda row: row.get('sequence', 0)):
        row = {k: v for k, v in raw.items() if k != '__projection_source'}
        store.put({
            'idempotency_key': 'fixture-' + row['command_id'],
            'action_id': 'historical.readback', 'status': 'failed',
            'ownership_generation': 0, 'started_at': 0.0,
            **row,
        })
    with closing(sqlite3.connect(store.path)) as db:
        db.create_function('authority_write_allowed', 0, lambda: 1)
        for i, row in enumerate(retained, 1):
            db.execute('''INSERT INTO operator_plane_commands (
                command_id,stream_sequence,method_id,method_sequence,action_id,
                requested_json,effective_json,status,version,ownership_generation,
                queued_at,dispatched_at,finished_at,terminal_json,updated_at,
                remote_acknowledged,controller_acknowledged,physical_effect_verified
            ) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)''', (
                row['command_id'], row.get('stream_sequence', row.get('sequence', i)),
                row.get('method_id'), row.get('method_sequence'), row.get('action_id', 'historical.readback'),
                json.dumps(row.get('requested_inputs', {})), json.dumps(row.get('effective_inputs', {})),
                row.get('stored_status', row.get('status', 'failed')), row.get('state_version', 1), row.get('ownership_generation', 0),
                row.get('accepted_at', row.get('queued_at', 0.0)), row.get('dispatched_at'), row.get('finished_at'),
                json.dumps(row.get('terminal_evidence')) if row.get('terminal_evidence') is not None else None, 0.0,
                int(row.get('remote_acknowledged', False)), int(row.get('controller_acknowledged', False)), int(row.get('physical_effect_verified', False)),
            ))
        db.commit()
