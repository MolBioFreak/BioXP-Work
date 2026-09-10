"""One public history path, bounded SQLite projection and on-demand evidence."""
import base64
import json
import sqlite3
from concurrent.futures import ThreadPoolExecutor
from contextlib import closing

import pytest
from fastapi.testclient import TestClient
from bioxp import operator_controls, operator_history
from history_fixture_support import seed_history
from test_operator_controls import make_app


def test_retained_json_without_identity_uses_sql_columns_across_all_pages(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    store = app.state.operator_receipt_store
    seed_history(app, [{'command_id': f'historical-{i}', 'sequence': i + 1, 'accepted_at': float(i + 1)} for i in range(201)], [])
    with store.lock:
        store.connection.execute("UPDATE operator_commands SET receipt_json='{}'")
        before = [tuple(r) for r in store.connection.execute('SELECT sequence,command_id,receipt_json FROM operator_commands')]
    client = TestClient(app)
    first = client.get('/operator/actions/history', params={'limit': 200}).json()
    second = client.get('/operator/actions/history', params={'limit': 200, 'cursor': first['next_cursor']}).json()
    rows = first['items'] + second['items']
    assert len(first['items']) == 200 and len(second['items']) == 1
    assert second['next_cursor'] is None
    assert len({r['command_id'] for r in rows}) == 201
    assert all(r['sequence'] > 0 and r['accepted_at'] == 0.0 for r in rows)
    assert [tuple(r) for r in store.connection.execute('SELECT sequence,command_id,receipt_json FROM operator_commands')] == before
    paths = client.get('/openapi.json').json()['paths']
    assert '/operator/actions/history' in paths
    assert '/operator/v2/actions/history' not in paths
    assert 'schema_version' not in {p['name'] for p in paths['/operator/actions/history']['get']['parameters']}
    assert client.get('/operator/v2/actions/history').status_code in (404, 405)
    assert calls == []


@pytest.mark.parametrize('count,limit', [(0, 8), (8, 8), (9, 8), (200, 200), (201, 200)])
def test_cursor_exists_only_when_another_record_exists(tmp_path, monkeypatch, count, limit):
    app, _ = make_app(tmp_path, monkeypatch)
    seed_history(app, [{'command_id': str(i), 'accepted_at': i, 'sequence': i + 1} for i in range(count)], [])
    page = TestClient(app).get('/operator/actions/history', params={'limit': limit}).json()
    assert len(page['items']) == min(count, limit)
    assert (page['next_cursor'] is not None) is (count > limit)


@pytest.mark.parametrize('cursor', ['', '1', 'not-a-cursor', 'x' * 1025, *[
    base64.urlsafe_b64encode(json.dumps(value).encode()).decode().rstrip('=')
    for value in ({}, {'v': 1, 'key': [float('nan'), 1, 1, 'a']},
                  {'v': 1, 'key': [1, 1, -1, 'a']}, {'v': 1, 'key': [1, 1, 2**64, 'a']},
                  {'v': 1, 'key': [1, True, 1, 'a']}, {'v': 1, 'key': [1, 1, 1, '']})
]])
def test_malformed_cursors_are_422(tmp_path, monkeypatch, cursor):
    app, _ = make_app(tmp_path, monkeypatch)
    assert TestClient(app).get('/operator/actions/history', params={'cursor': cursor}).status_code == 422


def test_poll_does_not_wait_for_writer_lock_or_read_evidence(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    store = app.state.operator_receipt_store
    seed_history(app, [{'command_id': 'one', 'accepted_at': 1, 'status': 'failed'}], [])
    def forbidden(*args, **kwargs):
        raise AssertionError('history must not hydrate receipts or read evidence')
    monkeypatch.setattr(store, 'by_command', forbidden)
    monkeypatch.setattr(operator_controls.OperatorHistoryReader, 'get_command', forbidden)
    with ThreadPoolExecutor(max_workers=1) as pool:
        with store.lock:
            response = pool.submit(TestClient(app).get, '/operator/actions/history').result(timeout=2)
    assert response.status_code == 200
    assert response.json()['items'][0]['command_id'] == 'one'
    assert calls == []


def test_single_snapshot_and_batched_selection(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    store = app.state.operator_receipt_store
    seed_history(app, [{'command_id': str(i), 'accepted_at': i, 'sequence': i + 1, 'status': 'failed'} for i in range(100)], [])
    statements = []
    original_connect = sqlite3.connect
    updated = False
    def trace(statement):
        nonlocal updated
        statements.append(statement)
        if 'WITH history_keys' in statement and not updated:
            updated = True
            with closing(original_connect(store.path)) as writer:
                writer.create_function('authority_write_allowed', 0, lambda: 1)
                writer.execute("UPDATE operator_commands SET status='completed' WHERE command_id='99'")
                writer.commit()
    def connect(*args, **kwargs):
        db = original_connect(*args, **kwargs)
        db.set_trace_callback(trace)
        return db
    monkeypatch.setattr(operator_history.sqlite3, 'connect', connect)
    rows, cursor = operator_history.read_history_page(store.root, 100)
    assert updated and rows[0]['command_id'] == '99' and rows[0]['status'] == 'failed'
    assert cursor is None
    assert len(statements) <= 7  # constant number of statements, not 100 receipt lookups
    assert not any(s.lstrip().upper().startswith(('INSERT', 'UPDATE', 'DELETE')) for s in statements)


def test_retained_nonterminal_stays_ambiguous_and_direct_detail_keeps_evidence(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    native = {'command_id': 'direct', 'accepted_at': 11, 'status': 'failed',
              'response': {'http_status': 409, 'body': {'failure': 'retained-failure', 'native_layers': [1, 2, 3]}},
              'stage_receipts': [{'stage_id': 'readback', 'raw': 'unchanged evidence'}]}
    seed_history(app, [native], [{'command_id': 'retained', 'accepted_at': 10, 'status': 'queued'}])
    client = TestClient(app)
    page = client.get('/operator/actions/history').json()
    row = next(r for r in page['items'] if r['command_id'] == 'retained')
    assert row['status'] == 'ambiguous' and row['terminal'] is True
    assert row['history']['recorded_status'] == 'queued'
    assert row['physical_effect_verified'] is False and row['error']['retryable'] is False
    assert 'response' not in page['items'][0] and 'source_receipt' not in page['items'][0]
    detail = client.get('/operator/v2/actions/receipts/direct?detail=true').json()
    assert detail['source_receipt']['response'] == native['response']
    assert detail['source_receipt']['stage_receipts'] == native['stage_receipts']
    assert detail['accepted_at'] == page['items'][0]['accepted_at']
    assert calls == []

def test_history_preserves_iso_instants_and_embedded_operator_assessment(tmp_path, monkeypatch):
    from datetime import datetime
    app, _ = make_app(tmp_path, monkeypatch)
    instant = "2026-08-01T12:00:00+00:00"
    record = {"command_id": "iso-and-note", "accepted_at": instant, "sequence": 1}
    record.update(finished_at=instant, operator_assessment="fail", operator_note="retained operator observation")
    seed_history(app, [record], [])
    item = TestClient(app).get("/operator/actions/history").json()["items"][0]
    assert item["accepted_at"] == datetime.fromisoformat(instant).timestamp()
    assert item["finished_at"] == item["accepted_at"]
    assert item["history"]["operator_assessment"] == "fail"
    assert item["history"]["operator_note"] == "retained operator observation"
