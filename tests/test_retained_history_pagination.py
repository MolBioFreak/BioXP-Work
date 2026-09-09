import json
from fastapi.testclient import TestClient
from bioxp import operator_controls
from test_operator_controls import make_app


def test_retained_json_without_identity_uses_sql_columns_across_pages(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    store = app.state.operator_receipt_store
    for i in range(201):
        store.claim({'command_id': f'historical-{i}', 'idempotency_key': f'historical-{i}',
                     'action_id': 'historical.readback', 'started_at': float(i + 1)})
    with store.lock:
        store.connection.execute("UPDATE operator_commands SET receipt_json='{}'")
        before = list(store.connection.execute('SELECT sequence,command_id,receipt_json FROM operator_commands'))
    client = TestClient(app)
    rows = client.get('/operator/actions/history', params={'limit': 200}).json()['receipts']
    assert len(rows) == 200
    assert len({r['command_id'] for r in rows}) == 200
    assert all(r['sequence'] > 0 for r in rows)
    for path in ['/operator/v2/control-catalog', '/operator/control-catalog', '/operator/v2/actions/history']:
        assert client.get(path).status_code == 200
    assert [tuple(r) for r in store.connection.execute('SELECT sequence,command_id,receipt_json FROM operator_commands')] == [tuple(r) for r in before]
    assert calls == []


def test_durable_cursor_uses_stream_not_canonical_receipt_sequence(tmp_path, monkeypatch):
    app, calls = make_app(tmp_path, monkeypatch)
    rows = [{'command_id': f'durable-{i}', 'sequence': i + 10000,
             'stream_sequence': i, 'status': 'failed', 'accepted_at': float(i)} for i in range(1, 202)]
    boundaries = []
    def list_commands(self, limit=200, before_sequence=None, **kwargs):
        boundaries.append(before_sequence)
        return sorted((r.copy() for r in rows if before_sequence is None or r['stream_sequence'] < before_sequence),
                      key=lambda r: r['stream_sequence'], reverse=True)[:limit]
    monkeypatch.setattr(operator_controls.OperatorHistoryReader, 'list_commands', list_commands)
    response = TestClient(app).get('/operator/actions/history', params={'limit': 200})
    assert response.status_code == 200
    assert len(response.json()['receipts']) == 200
    assert boundaries == [None, 2]
    assert calls == []
