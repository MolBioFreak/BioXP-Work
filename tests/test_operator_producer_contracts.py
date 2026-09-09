"""Offline producer regressions; optional retained corpus uses real BMS models.

Run corpus checks with BIOXP_AUDIT_CORPUS and BIOXP_BMS_OPERATOR_MODELS
pointing at the retained audit directory and consumer operator_models.py.
"""
import asyncio
import copy
import importlib.util
import json
import os
from pathlib import Path
import sys

from fastapi.testclient import TestClient
import pytest
from pydantic import ValidationError

from bioxp import operator_controls
from test_operator_controls import make_app
from test_oem_deck_install_binding import _install_reconciliation_app


def test_generated_catalog_routes_keep_v2_only_actions_out_of_v1(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    client = TestClient(app)
    v1 = client.get('/operator/control-catalog').json()
    v2 = client.get('/operator/v2/control-catalog').json()
    assert 'oem.deck.move_to_location' not in {r['action_id'] for r in v1['actions']}
    assert 'oem.deck.move_to_location' in {r['action_id'] for r in v2['actions']}


def invoke(client):
    return client.post('/operator/v2/actions/oem.z.manual_home', json={
        'schema_version': 'bioxp.operator_action_request.v2',
        'expected_ownership_generation': 7, 'expected_board_epoch_by_board': {},
        'idempotency_key': 'producer-failure-regression', 'inputs': {},
    })


@pytest.mark.parametrize('http,body,code,message', [
    (200, {'ok': False, 'failure': 'RuntimeError: Reach GZ position time out! board=4; axis=0; position=10000'},
     'controller_position_wait_timeout', 'Controller position wait timed out; inspect retained board/axis/position evidence.'),
    (200, {'ok': False, 'failure': 'RuntimeError: timeout position=10000'},
     'route_application_failed', 'Robot route reported an application failure.'),
    (200, {'ok': False, 'result': {'error': 'missing_reference', 'message': 'Y reference absent'}},
     'missing_reference', 'Required axis reference is unavailable.'),
    (200, {'result': {'ok': False, 'error': 'reference_missing'}},
     'reference_missing', 'Required axis reference is unavailable.'),
    (409, {'detail': {'error': 'missing_reference', 'message': 'password=hunter2 /private/key'}},
     'missing_reference', 'Required axis reference is unavailable.'),
    (409, {'detail': 'postgres://user:secret@host/db\nTraceback /private/config'},
     'route_http_conflict', 'Robot route reported an HTTP conflict.'),
    (200, {'ok': False, 'failure': 'x' * 2000},
     'route_application_failed', 'Robot route reported an application failure.'),
])
def test_failure_classification_and_evidence(tmp_path, monkeypatch, http, body, code, message):
    app, _ = make_app(tmp_path, monkeypatch)
    async def failed(*args, **kwargs):
        return http, copy.deepcopy(body)
    monkeypatch.setattr(operator_controls, '_dispatch_asgi', failed)
    client = TestClient(app)
    receipt = invoke(client).json()
    assert receipt['status'] == 'failed'
    assert receipt['error']['code'] == code
    assert receipt['error']['message'] == message
    assert receipt['error']['retryable'] is False
    assert receipt['physical_effect_verified'] is False
    row = client.get('/operator/actions/history').json()['receipts'][0]
    assert row['command_id'] == receipt['command_id']
    detail = client.get(f"/operator/actions/receipts/{receipt['command_id']}", params={'detail': True}).json()
    assert detail['response'] == {'http_status': http, 'body': body}
    assert row['error'] == message


def test_timeout_stays_ambiguous_and_retry_forbidden(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    async def timed_out(*args, **kwargs):
        raise asyncio.TimeoutError
    monkeypatch.setattr(operator_controls, '_dispatch_asgi', timed_out)
    client = TestClient(app)
    receipt = invoke(client).json()
    assert receipt['status'] == 'ambiguous'
    assert receipt['error']['code'] == 'action_outcome_unknown'
    assert receipt['error']['retryable'] is False
    row = client.get('/operator/actions/history').json()['receipts'][0]
    assert row['retry_forbidden'] is True
    assert row['reconciliation_required'] is True
    assert row['physical_effect_verified'] is False


@pytest.mark.parametrize('diagnostic,expected', [
    ('deck_authority_cache_unavailable', 'canonical_deck_authority_unavailable:deck_authority_cache_unavailable'),
    ('deck_authority_cache_stale', 'canonical_deck_authority_unavailable:deck_authority_cache_stale'),
    ('source_authority_missing:deck_authority_cached_snapshot',
     'canonical_deck_authority_unavailable:source_authority_missing:deck_authority_cached_snapshot'),
    ('deck_semantic_state_not_authoritative:location_revision',
     'canonical_deck_authority_unavailable:deck_semantic_state_not_authoritative:location_revision'),
    ('deck_semantic_state_not_authoritative:location_revision password=secret',
     'canonical_deck_authority_unavailable'),
    ('postgres://user:secret@host/db\nTraceback /private/key',
     'canonical_deck_authority_unavailable'),
])
def test_deck_diagnostics_are_exact_allowlisted_codes(tmp_path, monkeypatch, diagnostic, expected):
    from bioxp.oem_runtime_store import OEMRuntimeStore
    OEMRuntimeStore(tmp_path / 'canonical-oem-runtime').close()
    app, provider, _ = _install_reconciliation_app(tmp_path, monkeypatch)
    def unavailable(**kwargs):
        raise RuntimeError(diagnostic)
    active_calls = []
    def forbidden_active(**kwargs):
        active_calls.append(kwargs)
        raise AssertionError('metadata GET must not collect XYZ/latch hardware')
    monkeypatch.setattr(provider, 'deck_authority_snapshot', forbidden_active)
    # Polling now requires the passive reader; never route catalog diagnostics
    # through the active XYZ/latch hardware collection method.
    monkeypatch.setattr(provider, 'deck_authority_cached_snapshot', unavailable, raising=False)
    try:
        catalog = TestClient(app).get('/operator/v2/control-catalog').json()
        action = next(a for a in catalog['actions'] if a['action_id'] == 'oem.deck.move_to_location')
        assert action['enabled'] is False
        assert active_calls == []
        assert action['disabled_reason'] == expected
        assert action['expected_board_epoch_by_board'] == {}
        assert all(not option['enabled'] for option in action['destination_options'])
    finally:
        app.state.operator_command_plane.stop()


def fake_history(monkeypatch, direct, durable):
    def select(rows, limit, before):
        return sorted((copy.deepcopy(r) for r in rows if before is None or (r.get('sequence') or r.get('stream_sequence') or 0) < before),
                      key=lambda r: r.get('sequence') or r.get('stream_sequence') or 0, reverse=True)[:limit]
    monkeypatch.setattr(operator_controls.OperatorReceiptStore, 'list',
        lambda self, limit, before_sequence=None: select(direct, limit, before_sequence))
    monkeypatch.setattr(operator_controls.OperatorHistoryReader, 'list_commands',
        lambda self, limit, before_sequence=None, **kw: select(durable, limit, before_sequence))


@pytest.mark.parametrize('durable_time', [10, 9, 11])
def test_mixed_history_deduplicates_before_cursor_with_conflicting_projections(tmp_path, monkeypatch, durable_time):
    app, _ = make_app(tmp_path, monkeypatch)
    direct = [{'command_id': 'same', 'sequence': 3, 'accepted_at': 10, 'status': 'failed'},
              {'command_id': 'tie-b', 'sequence': 2, 'accepted_at': 10, 'status': 'completed'}]
    durable = [{'command_id': 'same', 'sequence': 3, 'accepted_at': durable_time, 'status': 'completed'},
               {'command_id': 'tie-a', 'sequence': 2, 'accepted_at': 10, 'status': 'failed'}]
    fake_history(monkeypatch, direct, durable)
    client = TestClient(app)
    rows = client.get('/operator/actions/history').json()['receipts']
    assert len(rows) == 3
    assert all('__projection_source' not in r for r in rows)
    # Direct receipt is the documented identity winner, even if durable time is newer.
    assert next(r for r in rows if r['command_id'] == 'same')['status'] == 'failed'
    ids, cursor = [], None
    for _ in range(5):
        params = {'schema_version': 'bioxp.operator_action_history.v2', 'limit': 1}
        if cursor:
            params['cursor'] = cursor
        page = client.get('/operator/actions/history', params=params).json()
        ids.extend(r['command_id'] for r in page['items'])
        cursor = page['next_cursor']
        if not cursor:
            break
    assert ids == [r['command_id'] for r in rows] == ['same', 'tie-b', 'tie-a']


def test_history_sorts_across_store_batch_boundaries(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    direct = [{'command_id': str(i), 'sequence': i, 'accepted_at': 1, 'status': 'failed'} for i in range(1, 202)]
    direct[0]['accepted_at'] = 100  # Older sequence, newest timestamp.
    fake_history(monkeypatch, direct, [])
    rows = TestClient(app).get('/operator/actions/history', params={'limit': 1}).json()['receipts']
    assert rows[0]['command_id'] == '1'


def test_history_does_not_silently_drop_missing_identity(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    fake_history(monkeypatch, [{'sequence': 1, 'accepted_at': 10}], [])
    with pytest.raises(RuntimeError, match='history_receipt_missing_command_id'):
        TestClient(app).get('/operator/actions/history')


def test_history_does_not_return_partial_page_when_reader_stalls(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    rows = [{'command_id': str(i), 'sequence': i, 'accepted_at': 1} for i in range(1, 201)]
    monkeypatch.setattr(operator_controls.OperatorReceiptStore, 'list', lambda *args, **kwargs: rows)
    with pytest.raises(RuntimeError, match='history_store_pagination_not_advancing'):
        TestClient(app).get('/operator/actions/history')


@pytest.fixture
def retained_contract():
    corpus = os.environ.get('BIOXP_AUDIT_CORPUS')
    models_path = os.environ.get('BIOXP_BMS_OPERATOR_MODELS')
    if not corpus or not models_path:
        pytest.skip('Set BIOXP_AUDIT_CORPUS and BIOXP_BMS_OPERATOR_MODELS for retained consumer checks')
    spec = importlib.util.spec_from_file_location('retained_bms_operator_models', models_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return Path(corpus), module


def test_retained_catalog_validates_entire_real_consumer_model(retained_contract):
    corpus, models = retained_contract
    raw = json.loads((corpus / 'robot-catalog-v1.json').read_text())
    with pytest.raises(ValidationError):
        models.OperatorControlCatalog.model_validate(raw)
    patched = {**raw, 'actions': operator_controls._v1_catalog_actions(raw['actions'])}
    validated = models.OperatorControlCatalog.model_validate(patched)
    assert len(validated.actions) == 237


def test_retained_history_real_endpoint_preserves_durable_contract(tmp_path, monkeypatch, retained_contract):
    corpus, models = retained_contract
    raw = json.loads((corpus / 'robot-history-v1.json').read_text())
    app, _ = make_app(tmp_path, monkeypatch)
    direct = [r for r in raw['receipts'] if r['__projection_source'] == 'direct']
    durable = [r for r in raw['receipts'] if r['__projection_source'] != 'direct']
    fake_history(monkeypatch, direct, durable)
    result = TestClient(app).get('/operator/actions/history').json()
    assert len(result['receipts']) == len(raw['receipts']) == 25
    by_id = {r['command_id']: r for r in result['receipts']}
    for row in raw['receipts']:
        assert by_id[row['command_id']] == {k: v for k, v in row.items() if k != '__projection_source'}
    # The original public leak must remain invalid even after BMS gains the
    # explicit durable variant: closed models, not a global extras escape hatch.
    with pytest.raises(ValidationError):
        models.OperatorActionHistory.model_validate(raw)
    # Requires the separate BMS lane's explicit durable receipt variant; no coercion.
    models.OperatorActionHistory.model_validate(result)
    v2 = TestClient(app).get('/operator/actions/history', params={
        'schema_version': 'bioxp.operator_action_history.v2',
    }).json()
    models.OperatorActionHistoryV2.model_validate(v2)
    assert {r['command_id'] for r in v2['items']} == set(by_id)
    assert all(r['error'] is None or r['error']['code'] in operator_controls._ROUTE_FAILURE_MESSAGES for r in v2['items'])
