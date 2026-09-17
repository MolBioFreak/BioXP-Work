"""Replay actual captured executor return; fault injection only at return seam.

The canonical executor still runs first and commits its own actual native no-op.
This supplemental transport/classifier test does not replace the unmodified
API/native/SQLite sequence in test_deck_complete_noop.py.
"""
import copy
import hashlib
import json
from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_complete_noop import setup_native, submit, terminal

CAPTURE = Path('/home/dalab/.hermes/profiles/fresh/robot-audit/deck-tip-failure/sweep-finalization-fixed/02-LOC_BSC-native-receipt.json')


@pytest.mark.parametrize('mutation', ['none', 'semantic_only', 'no_rows', 'missing_noop',
    'missing_terminal', 'ack_contradiction', 'delivered_child', 'failed_child', 'failed_outer', 'no_commit',
    'extra_incomplete_child', 'extra_noop_child', 'unknown_branch'])
def test_captured_return_through_queued_finalizer(installed_retained, retained_rig, monkeypatch, mutation):
    app, provider, observations, references, root = installed_retained
    leaf, waits, raw = setup_native(installed_retained, retained_rig, monkeypatch, before=(42518, 8779))
    capture_bytes = CAPTURE.read_bytes()
    assert hashlib.sha256(capture_bytes).hexdigest() == 'becfbbbbfdbc6abce74d18f453e6dd7d8058725d82aeec529111746a13f53381'
    captured = json.loads(capture_bytes)
    assert captured['command_id'] == '93630cfc-a662-4c57-8cd2-1be222ba6770'
    assert captured['status'] == 'failed' and captured['completion_class'] == 'deck_incomplete'
    response = copy.deepcopy(captured['source_receipt']['terminal_evidence']['response'])
    row = response['provider_results'][0]
    if mutation == 'semantic_only':
        response = {'ok': True, 'semantic_state_committed': True, 'delivery_attempted': False}
    elif mutation == 'no_rows':
        response['provider_results'] = []
    elif mutation == 'missing_noop':
        row.pop('source_noop')
    elif mutation == 'missing_terminal':
        row.pop('controller_completion_verified')
    elif mutation == 'ack_contradiction':
        row['controller_command_acknowledged'] = True
    elif mutation == 'delivered_child':
        row['delivery_attempted'] = True
    elif mutation == 'failed_child':
        row['ok'] = False
    elif mutation == 'failed_outer':
        response['ok'] = False
    elif mutation == 'no_commit':
        response['semantic_state_committed'] = False
    elif mutation == 'extra_incomplete_child':
        response['provider_results'].append({'ok': True, 'delivery_attempted': True,
            'controller_command_acknowledged': True, 'controller_completion_verified': False})
    elif mutation == 'extra_noop_child':
        response['provider_results'].append(copy.deepcopy(row))
    elif mutation == 'unknown_branch':
        response['source_branch'] = 'not_an_ordinary_move'
    executor = app.state.oem_deck_command_executor
    calls = []
    def replay(**kwargs):
        actual = executor(**kwargs)
        assert actual['ok'] is True and actual['semantic_state_committed'] is True
        assert actual['provider_results'][0]['source_noop'] is True
        assert actual['delivery_attempted'] is False
        calls.append(kwargs['command_id'])
        return response
    monkeypatch.setattr(app.state, 'oem_deck_command_executor', replay)
    app.state.operator_command_plane.start()
    client = TestClient(app)
    cid, _ = submit(app, client, 'LOC_BSC', provider)
    detail = terminal(client, cid)
    assert calls == [cid] and len(raw) == 1 and not leaf.moves and not waits
    assert detail['status'] == ('completed' if mutation == 'none' else 'failed'), detail
    assert (detail['completion_class'] == 'source_noop') is (mutation == 'none')
    assert detail['source_receipt']['source_noop'] is (mutation == 'none')
    assert detail['physical_effect_verified'] is False
