"""Connected canonical constructor detail; synthetic exchanges, no live transport."""
import asyncio
import copy
import json
import sqlite3
import subprocess
import sys

import pytest
from fastapi.testclient import TestClient
from bioxp import api
from bioxp.lifecycle_state import CanonicalLifecycleOwner
from bioxp.hardware_status import HardwareStateOwner
from bioxp.pipette import transport as tm
from tests.test_deck_tip_query_publication import query_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_scoped_authority import retained_rig
from tests.test_w1_diagnostics_connected import transport_rig, bind_full_infrastructure_identity, artifact


def stage(payload):
    return payload['lifecycle']['startup']['stages']['constructor_pipette_stage']


@pytest.fixture
def detail_rig(query_rig, monkeypatch):
    rig = query_rig
    bind_full_infrastructure_identity(monkeypatch)
    owner = CanonicalLifecycleOwner()
    owner.transport_changed(True, reason='controlled offline CAN exchange owner')
    monkeypatch.setattr(api, 'lifecycle_state', owner)
    # Infrastructure predecessor, not a constructor/receipt success double.
    hardware = HardwareStateOwner()
    hardware.change_ownership(reason='controlled exchange endpoints', transport='owned',
                              usb='service', router='running')
    ready = hardware.publish_can_ready_from_preparation(
        expected_ownership_epoch=hardware._epoch, reason='offline prepared CAN endpoints')
    assert ready['published']
    monkeypatch.setattr(api, 'hardware_state', hardware)
    app = rig[0]
    app.add_api_route('/liquid/init', api.liquid_init, methods=['POST'])
    app.add_api_route('/oem/startup/constructor_pipettes', api.oem_startup_constructor_pipettes, methods=['POST'])
    app.add_api_route('/oem/startup/status/latest', api.oem_startup_status_latest, methods=['GET'])
    app.add_api_route('/oem/startup/status/{session_id}', api.oem_startup_status, methods=['GET'])
    return rig, owner, TestClient(app)


@pytest.mark.parametrize('entrypoint', ['/liquid/init', '/oem/startup/constructor_pipettes'])
@pytest.mark.parametrize('mode', ['initial', 'unknown_offsets', 'partial_offsets',
    'conditional_retry', 'delayed_completion', 'completion_failure', 'partial_channel'])
def test_explicit_detail_real_api_service_typed_sqlite(detail_rig, monkeypatch, entrypoint, mode):
    rig, owner, client = detail_rig
    group, wire, timers = transport_rig(tm, mode)
    monkeypatch.setattr(api, '_pipette_transport', group)
    monkeypatch.setattr(api, '_get_pipette_transport', lambda: group)
    response = client.post(entrypoint, json={}, headers={'Idempotency-Key': 'detail-' + mode})
    for timer in timers: timer.join()
    expected_ok = mode not in ('completion_failure', 'partial_channel')
    assert response.status_code == (200 if expected_ok else 409), response.text
    payload = response.json() if expected_ok else {'lifecycle': response.json()['detail']}
    explicit = stage(payload)
    compact = owner.projection()['startup']['stages']['constructor_pipette_stage']
    assert compact['evidence'] is None and compact['history'] == []
    assert compact['state'] == ('passed' if expected_ok else 'failed')
    assert explicit['attempt_id'] == compact['attempt_id']
    assert explicit['state'] == compact['state']
    assert explicit['completed_at'] == compact['completed_at']
    assert explicit['error'] == compact['error']
    row = dict(rig[5].connection.execute(
        'SELECT * FROM pipette_operations WHERE lifecycle_stage_id=? AND lifecycle_attempt_id=?',
        ('constructor_pipette_stage', compact['attempt_id'])).fetchone())
    receipt = json.loads(row['receipt_json'])
    assert receipt['runtime_binding']['lifecycle_attempt_id'] == compact['attempt_id']
    assert receipt['runtime_binding']['caller_class'] == 'lifecycle'
    assert row['command_id'] == explicit['evidence']['command_id']
    assert row['status'] == ('completed' if expected_ok else 'failed')
    command = rig[5].connection.execute('SELECT * FROM operator_commands WHERE command_id=?',
        (row['command_id'],)).fetchone()
    assert command['status'] == row['status'] and command['outcome'] == row['outcome']
    expected_detail = {**receipt['result'], 'command_id': row['command_id'],
        'receipt_id': receipt['receipt_id'], 'receipt_truth': receipt['truth'],
        'source_identity': receipt['source_identity']}
    assert explicit['evidence'] == expected_detail
    assert receipt['truth']['physical_effect_verified'] is False
    assert receipt['result']['ok'] is expected_ok
    assert len(receipt['result']['collection_source']['channels']) == 4
    if mode in ('initial', 'unknown_offsets', 'partial_offsets'):
        offsets = expected_detail['pressure_offset_evidence']
        valid = [int(ch) for ch, value in offsets.items() if value['valid']]
        assert valid == ([0, 1, 2, 3] if mode == 'initial' else [0, 1] if mode == 'partial_offsets' else [])
        for ch in valid:
            assert offsets[str(ch)]['offset'] == 100 + ch
    if mode == 'conditional_retry':
        assert expected_detail['single_conditional_retry_performed'] is True
        assert expected_detail['pressure_epoch']['epoch'] == 2
    if mode == 'delayed_completion':
        assert all(c['result']['ok'] for c in expected_detail['initial_group']['delayed_completions'])
    if mode == 'completion_failure':
        assert expected_detail['initial_group']['delayed_completions'][3]['result']['event_error_code'] == 0x21
    before = list(wire)
    latest = client.get('/oem/startup/status/latest')
    assert latest.status_code == 200 and stage(latest.json()) == compact
    canonical = client.get('/oem/startup/status/canonical')
    assert canonical.status_code == 200 and stage(canonical.json()) == explicit
    result = api._oem_non_motion_startup_result(owner.projection())
    assert stage(result) == explicit
    assert owner.projection()['startup']['stages']['constructor_pipette_stage'] == compact
    assert wire == before  # Explicit detail is a lookup, never another execution.
    # Independent interpreter sees the exact typed row, identity and truth.
    script = ('import json,sqlite3,sys;c=sqlite3.connect(sys.argv[1]);c.row_factory=sqlite3.Row;'
              'print(json.dumps(dict(c.execute("SELECT * FROM pipette_operations WHERE command_id=?",'
              '(sys.argv[2],)).fetchone())))')
    db = rig[5].connection.execute('PRAGMA database_list').fetchone()[2]
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', script, db, row['command_id']], text=True))
    assert reopened == row
    artifact('detail-' + entrypoint.split('/')[1] + '-' + mode + '.json', {
        'entrypoint': entrypoint, 'response': payload, 'row': row, 'wire': wire,
        'provenance': 'controlled exchange; actual driver/router/API/service/typed SQLite, not live hardware'})


@pytest.mark.parametrize('missing', ['absent_store', 'no_matching_attempt', 'sqlite_read_denied'])
def test_optional_detail_never_changes_passed_stage_or_constructor_admission(detail_rig, monkeypatch, missing):
    rig, owner, client = detail_rig
    group, wire, timers = transport_rig(tm, 'initial')
    monkeypatch.setattr(api, '_pipette_transport', group)
    monkeypatch.setattr(api, '_get_pipette_transport', lambda: group)
    response = client.post('/liquid/init', json={}, headers={'Idempotency-Key': 'optional-detail-' + missing})
    assert response.status_code == 200, response.text
    compact = copy.deepcopy(owner.projection())
    before = list(wire)
    prior_admission = client.post('/oem/startup/constructor_pipettes')
    assert prior_admission.status_code == 409
    assert prior_admission.json()['detail'] == 'constructor_pipette_stage already passed in this ownership epoch'
    store = rig[5]
    denied = []
    if missing == 'absent_store':
        monkeypatch.setattr(api, '_pipette_receipts', None)
    else:
        def authorizer(action, table, column, database, trigger):
            if action == sqlite3.SQLITE_READ and table == 'pipette_operations':
                if missing == 'no_matching_attempt':
                    # SQLite's actual NULL projection yields no matching detail;
                    # no row/attempt is rewritten and the real stage stays passed.
                    if column == 'lifecycle_attempt_id':
                        denied.append(column)
                        return sqlite3.SQLITE_IGNORE
                else:
                    denied.append(column)
                    return sqlite3.SQLITE_DENY
            return sqlite3.SQLITE_OK
        store.connection.set_authorizer(authorizer)
    try:
        canonical = client.get('/oem/startup/status/canonical')
        assert canonical.status_code == 200
        assert canonical.json()['lifecycle'] == compact
        assert api._oem_non_motion_startup_result(copy.deepcopy(compact))['lifecycle'] == compact
        # Preserve the existing already-passed refusal exactly, rather than
        # inventing a receipt-dependent refusal or a repeat initialization.
        repeated = client.post('/oem/startup/constructor_pipettes')
        assert repeated.status_code == prior_admission.status_code
        assert repeated.json() == prior_admission.json()
        assert stage(canonical.json())['state'] == 'passed'
        assert stage(canonical.json())['evidence'] is None
    finally:
        store.connection.set_authorizer(None)
    if missing == 'sqlite_read_denied': assert denied
    assert wire == before
    assert api.lifecycle_state.projection() == compact


def test_pipettes_init_is_not_a_registered_api_on_merged_base():
    # Preserve this acceptance limitation rather than inventing a compatibility
    # route in a test-only lane. /liquid/init is the existing constructor endpoint.
    assert '/pipettes/init' not in api.app.openapi()['paths']
