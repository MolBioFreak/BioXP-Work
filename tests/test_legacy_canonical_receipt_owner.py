"""Connected canonical named-move receipt producer/readers; inert USB leaf only."""
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys

import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_complete_noop import setup_native, submit, terminal
from bioxp.operator_receipt_store import OperatorReceiptStore
from bioxp.operator_history import read_history_page
from bioxp.operator_reports import _command_source, _source_high_waters


@pytest.mark.parametrize('fault,before', [(None, (0, 0)), (None, (42518, 8779)),
                                        ('missing_ack', (0, 0)), ('ack_only', (0, 0)), ('source_exception', (0, 0)), ('latch_open', (0, 0))],
                         ids=['delivered', 'source-noop', 'unknown', 'partial', 'exception', 'failed'])
def test_named_receipt_readers_without_duplicate_writes(installed_retained, retained_rig, monkeypatch, fault, before):
    app, provider, observations, refs, root = installed_retained
    leaf, waits, raw = setup_native(installed_retained, retained_rig, monkeypatch, before=before, fault=fault)
    if fault == 'source_exception':
        def fail(*args, **kwargs):
            raise RuntimeError('inert controller source exception')
        monkeypatch.setattr(leaf, 'motor_oem_move_absolute', fail)
    if fault == 'latch_open':
        monkeypatch.setattr(observations, 'read_oem_latch_status', lambda: {'ok': True, 'value': False, 'observation_id': 'offline-open'})
        monkeypatch.setattr(observations, 'query_latch', lambda: {'ok': True, 'value': 0})
    plane = app.state.operator_command_plane
    legacy = OperatorReceiptStore(root)
    traced = []
    plane.store.connection.set_trace_callback(traced.append)
    client = TestClient(app)
    plane.start()
    cid, request = submit(app, client, 'LOC_BSC', provider)
    detail = terminal(client, cid)
    assert plane.store.wait_for_command_workers([cid], timeout=2)
    plane.stop()
    compact = client.get('/operator/v2/actions/receipts/' + cid).json()
    saved = legacy.connection.execute('SELECT * FROM operator_commands WHERE command_id=?', (cid,)).fetchone()
    assert saved['receipt_json'] == '{}'
    assert saved['status'] == plane.store.get_command(cid)['status']
    assert float(saved['finished_at']) == plane.store.get_command(cid)['finished_at']
    assert not any(sql.lstrip().upper().startswith('UPDATE OPERATOR_COMMANDS SET') and
                   'RECEIPT_JSON=' in sql.upper().replace(' ', '') for sql in traced)
    claims = legacy.by_command(cid)
    replay_claim = legacy.by_idempotency(request['idempotency_key'])
    assert replay_claim == claims
    listed = next(row for row in legacy.list(200) if row['command_id'] == cid)
    assert listed == claims
    for k, v in plane.store.get_command(cid).items():
        expected = (saved['sequence'] if k == 'sequence' else
                    bool(saved[k]) if k in {'controller_acknowledged', 'physical_effect_verified'} else v)
        assert claims[k] == expected
    assert client.get('/operator/actions/receipts/' + cid + '?detail=true').json() == claims
    replay = client.post('/operator/v2/actions/oem.deck.move_to_location', json=request)
    assert replay.status_code == 200 and replay.json()['command_id'] == cid
    assert len(raw) == (0 if fault in {'source_exception', 'latch_open'} else 1)
    attempts = legacy.connection.execute('SELECT COUNT(*) FROM operator_plane_delivery_attempts WHERE command_id=?', (cid,)).fetchone()[0]
    if fault is None:
        assert detail['status'] == 'completed'
        if before == (42518, 8779):
            assert detail['completion_class'] == 'source_noop'
            assert not leaf.moves and not waits
            assert not claims['controller_acknowledged']
            assert not detail['deck_movement']['delivery_attempted']
        else:
            assert leaf.moves and waits and attempts
            assert detail['deck_movement']['controller_command_acknowledged']
            assert claims['controller_acknowledged'] == bool(saved['controller_acknowledged'])
    else:
        assert detail['status'] in {'failed', 'ambiguous'}
        assert not detail['deck_movement']['semantic_state_committed']
        if fault not in {'source_exception', 'latch_open'}:
            assert attempts
        if fault == 'latch_open':
            assert detail['status'] == 'failed'
            assert not leaf.moves
    assert claims['physical_effect_verified'] is False
    assessment = legacy.assess(cid, expected_generation=request['expected_ownership_generation'],
                               verdict='fail', note='human assessment, not outcome',
                               idempotency_key='legacy-assessment-' + str(fault or before))
    assert assessment['operator_assessment'] == 'fail'
    assert assessment['status'] == claims['status']
    assert legacy.assess(cid, expected_generation=request['expected_ownership_generation'],
                         verdict='fail', note='human assessment, not outcome',
                         idempotency_key='legacy-assessment-' + str(fault or before)) == assessment
    history, _ = read_history_page(root, 200)
    item = next(row for row in history if row['command_id'] == cid)
    assert item['status'] == claims['status']
    assert item['accepted_at'] == claims['accepted_at']
    assert item['sequence'] == saved['sequence']
    assert item['history']['operator_assessment'] == 'fail'
    assert item['history']['controller_acknowledged'] == claims['controller_acknowledged']
    # A receipt-only clear must not alter any source field used by historical
    # report exports at their retained version watermark (actual SQL reader).
    watermarks = _source_high_waters(legacy.connection)
    query = 'SELECT * FROM ' + _command_source(watermarks) + ' WHERE command_id=?'
    frozen = dict(legacy.connection.execute(query, (cid,)).fetchone())
    legacy.connection.execute("UPDATE operator_commands SET receipt_json='{}' WHERE command_id=?", (cid,))
    assert dict(legacy.connection.execute(query, (cid,)).fetchone()) == frozen
    assert dict(legacy.connection.execute('SELECT * FROM ' + _command_source(_source_high_waters(legacy.connection)) +
                                         ' WHERE command_id=?', (cid,)).fetchone()) == frozen
    # Verify the same transaction sees its uncommitted owner state through the
    # borrowed reader. A second connection or store initializer cannot do this.
    with legacy.lock:
        legacy.connection.execute('BEGIN IMMEDIATE')
        try:
            legacy.connection.execute('INSERT INTO operator_transitions(command_id,state,observed_at,detail_json) VALUES(?,?,?,?)',
                                      (cid, claims['status'], 1.0, json.dumps({'operator_assessment': 'pass', 'operator_note': 'uncommitted'})))
            assert legacy.by_command(cid)['operator_note'] == 'uncommitted'
        finally:
            legacy.connection.execute('ROLLBACK')
    script = ('import json,sys; from tests.test_deck_scoped_integration import fresh_process_receipts; '
              'print(json.dumps(fresh_process_receipts(sys.argv[1],sys.argv[2])))')
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', script, str(root), cid], text=True, timeout=15))
    assert reopened['compact'] == compact
    # Assessment is separate from terminal receipt truth and compact contract.
    assert reopened['detail']['status'] == detail['status']
    assert reopened['detail']['deck_movement'] == detail['deck_movement']
    assert reopened['detail']['source_receipt']['operator_assessment'] == 'fail'
    history_response = client.get('/operator/actions/history?limit=200')
    assert history_response.status_code == 200, history_response.text
    out = os.environ.get('LEGACY_RECEIPT_EVIDENCE')
    if out:
        Path(out).mkdir(parents=True, exist_ok=True)
        Path(out, ('noop' if before != (0, 0) else fault or 'delivered') + '.json').write_text(json.dumps({
            **reopened, 'legacy': legacy.by_command(cid), 'history': item,
            'history_response': history_response.json(),
            'receipt_json_bytes': len(saved['receipt_json']), 'delivery_attempts': attempts,
            'native_returns': raw, 'moves': leaf.moves,
        }, indent=2))
    from bioxp.runtime_retention import retain_runtime_rows
    # Emulate the old producer only with this actually rendered connected
    # receipt, never an invented terminal outcome. Existing triggers stay on.
    legacy.connection.execute('UPDATE operator_commands SET receipt_json=? WHERE command_id=?',
                              (json.dumps(claims), cid))
    aged_clock = max(legacy.connection.execute('SELECT updated_at FROM operator_commands WHERE command_id=?',
                                               (cid,)).fetchone()[0], plane.store.connection.execute(
        'SELECT updated_at FROM operator_plane_commands WHERE command_id=?', (cid,)).fetchone()[0]) + 14 * 86400 + 1
    current_plan = retain_runtime_rows(root, as_of=aged_clock - 2)
    assert cid not in current_plan['tables']['operator_commands']['eligible_command_ids']
    # A real assessment present only in an old envelope must not be discarded.
    legacy.connection.execute('UPDATE operator_commands SET receipt_json=? WHERE command_id=?',
                              (json.dumps(assessment), cid))
    assert cid not in retain_runtime_rows(root, as_of=aged_clock)['tables']['operator_commands']['eligible_command_ids']
    legacy.connection.execute('UPDATE operator_commands SET receipt_json=? WHERE command_id=?',
                              (json.dumps(claims), cid))
    aged_plan = retain_runtime_rows(root, as_of=aged_clock)
    assert (cid in aged_plan['tables']['operator_commands']['eligible_command_ids']) is (claims['status'] == 'completed')
    applied = retain_runtime_rows(root, as_of=aged_clock, apply=True)
    cleared_blob = legacy.connection.execute('SELECT receipt_json FROM operator_commands WHERE command_id=?', (cid,)).fetchone()[0]
    assert (cleared_blob == '{}') is (claims['status'] == 'completed')
    assert legacy.by_command(cid)['status'] == claims['status']
    assert legacy.by_command(cid)['operator_assessment'] == 'fail'
    assert retain_runtime_rows(root, as_of=aged_clock, apply=True)['cleared_receipts'] == 0
    legacy.connection.close()
