"""Post-delivery ambiguity remains visible through strict receipt readers."""
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys
import time
import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig, qualify_test_references
from tests.test_deck_scoped_integration import installed_retained, catalog_action
from bioxp import api


def test_post_delivery_failure_keeps_truth_and_recovery_class(installed_retained, monkeypatch):
    app, provider, primitive, references, root = installed_retained
    catalog_action(app)
    qualify_test_references(references)
    api._collect_and_publish_hardware_snapshot(['axes', 'latch'], reason='isolated-test')
    action = catalog_action(app)
    assert action['enabled'] is True
    returned = False
    actual_move = primitive.oem_move_to
    observer = provider.assert_deck_observation_current
    def move(*args, **kwargs):
        nonlocal returned
        result = actual_move(*args, **kwargs)
        returned = True
        return result
    def observe(*args, **kwargs):
        if returned:
            raise RuntimeError('deck_reference_versions_changed_after_source_return')
        return observer(*args, **kwargs)
    monkeypatch.setattr(primitive, 'oem_move_to', move)
    monkeypatch.setattr(provider, 'assert_deck_observation_current', observe)
    store = app.state.operator_command_plane.store
    body = {'schema_version': 'bioxp.operator_action_request.v2',
            'expected_ownership_generation': int(provider.generation_provider()),
            'expected_board_epoch_by_board': action['expected_board_epoch_by_board'],
            'idempotency_key': 'post-return-fence',
            'inputs': {'target': 'LOC_OC', 'camera_offset': False}}
    with TestClient(app, raise_server_exceptions=False) as client:
        admitted = client.post('/operator/v2/actions/oem.deck.move_to_location', json=body)
        assert admitted.status_code == 200, admitted.text
        cid = admitted.json()['command_id']
        app.state.operator_command_plane.start()
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            pending = client.get('/operator/v2/actions/receipts/' + cid).json()
            if pending['status'] not in {'queued', 'dispatched', 'issued_pending'}:
                break
            time.sleep(.01)
        before = dict(store.connection.execute('SELECT * FROM operator_plane_commands WHERE command_id=?', (cid,)).fetchone())
        assert before['status'] == 'ambiguous' and before['finished_at'] is not None
        assert json.loads(before['terminal_json']).get('completion_class') is None
        compact = client.get('/operator/v2/actions/receipts/' + cid).json()
        detail = client.get('/operator/v2/actions/receipts/' + cid + '?detail=true').json()
        assert compact['completion_class'] == detail['completion_class'] == 'recovery_required'
        assert detail['status'] == 'ambiguous' and detail['physical_effect_verified'] is False
        deck = detail['deck_movement']
        assert deck['controller_completion_verified'] is True
        assert deck['semantic_state_committed'] is False and deck['ambiguity_state'] == 'recovery_required'
        assert store.deck_recovery_blocker() == 'deck_recovery_hold'
        assert before == dict(store.connection.execute('SELECT * FROM operator_plane_commands WHERE command_id=?', (cid,)).fetchone())
        assert sum(row[0] == 'move' for row in primitive.calls) == 1
    # A new interpreter reads the same durable state and disposition, no replay.
    code = "from pathlib import Path; import json; from bioxp.operator_command_plane import OperatorCommandStore; s=OperatorCommandStore(Path(%r)); print(json.dumps(s.get_command(%r)))" % (str(root), cid)
    fresh = json.loads(subprocess.check_output([sys.executable, '-c', code], text=True))
    assert fresh['status'] == 'ambiguous' and fresh['completion_class'] == 'recovery_required'
    output = os.environ.get('POSTMOVE_RECEIPT_EXPORT')
    if output:
        Path(output).write_text(json.dumps({'compact': compact, 'detail': detail}, indent=2))


@pytest.mark.parametrize('stored_status,finished,ambiguity,explicit,expected', [
    ('dispatched', None, 'recovery_required', None, None),
    ('completed', 5.0, 'none', None, None),
    ('ambiguous', 5.0, 'none', None, None),
    ('ambiguous', 5.0, 'recovery_required', 'operator_reconciled', 'operator_reconciled'),
])
def test_recovery_class_does_not_invent_terminal_or_override_class(stored_status, finished, ambiguity, explicit, expected):
    # Exact projection method, with SQLite containing only the independent rows
    # it reads. This fault/absence control grants no operational authority.
    from bioxp.operator_command_plane import OperatorCommandStore
    store = object.__new__(OperatorCommandStore)
    store.connection = sqlite3.connect(':memory:')
    store.connection.row_factory = sqlite3.Row
    store.connection.executescript('CREATE TABLE serial206_movement_commands(command_id TEXT,sequence INTEGER,state TEXT,state_version INTEGER,expected_board_epochs_json TEXT,terminal_receipt_id TEXT); CREATE TABLE operator_plane_transitions(command_id TEXT, transition_sequence INTEGER);')
    store._deck_command_detail = lambda cid: {'ambiguity_state': ambiguity}
    row = dict(command_id='probe', method_id=None, method_sequence=None, stream_sequence=1, action_id='oem.deck.move_to_location', status=stored_status, ownership_generation=1, requested_json='{}', effective_json='{}', queued_at=1.0, dispatched_at=2.0, finished_at=finished, source_noop=0, source_noop_reason=None, remote_acknowledged=0, controller_acknowledged=1, physical_effect_verified=0, terminal_json=json.dumps({'completion_class': explicit}), version=3)
    assert store._command_response(row)['completion_class'] == expected
