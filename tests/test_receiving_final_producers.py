"""Fresh merged receiving exports; all hardware remains at existing offline seams."""
import asyncio
import json
import os
import socket
from pathlib import Path
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from tests.test_w1_diagnostics_connected import query_rig, installed_retained, retained_rig, bind_full_infrastructure_identity, transport_rig, inline, tm, rm
from tests.test_e113_inspection_gz_connected import native_inspection, connected, rig, no_hardware
from tests.test_native_completion_handoff import handoff

SOCKET = socket.socket
OUT = Path(os.environ['RECEIVING_FINAL_ROOT'])

def save(name, value):
    (OUT / name).write_text(json.dumps(value, indent=2, sort_keys=True))

def public_native(root, ids, monkeypatch):
    from bioxp import operator_controls
    monkeypatch.setattr(socket, 'socket', SOCKET)
    from bioxp.runtime_audit_store import RUNTIME_ROOT_ENV_NAMES
    for name in RUNTIME_ROOT_ENV_NAMES:
        monkeypatch.setenv(name, str(root))
    app = FastAPI()
    operator_controls.install_operator_control_plane(app)
    client = TestClient(app)
    exported = {}
    for cid in ids:
        response = client.get('/operator/v2/actions/receipts/' + cid + '?detail=true')
        assert response.status_code == 200, response.text
        exported[cid] = response.json()
    history = client.get('/operator/actions/history?limit=100')
    assert history.status_code == 200, history.text
    exported['history'] = history.json()
    app.state.operator_poll_cache.close()
    return exported

@pytest.mark.parametrize('variant', ['before_ack', 'after_ack', 'ack_without_target', 'unequal', 'late', 'wrong_motor', 'old_generation', 'fault', 'stop', 'same_position', 'partial_failure'])
def test_fresh_inspection_public(native_inspection, monkeypatch, variant):
    from tests import test_e113_inspection_gz_connected as original
    r = native_inspection
    original.test_connected_inspection_gz_variants(r, variant)
    save('inspection-public-' + variant + '.json', public_native(r.root, [r.cid, 'controlled-inspection-parent'], monkeypatch))

@pytest.mark.parametrize('outcome', ['success', 'child_failure', 'stop'])
def test_fresh_document_public(handoff, monkeypatch, outcome):
    from tests import test_e113_lineage_custody_document as original
    original.test_sequential_changing_custody_document(handoff, monkeypatch, outcome)
    payload = json.loads((OUT / 'documents' / (outcome + '.json')).read_text())
    ids = [payload['terminal']['command']['command_id']] + [c['command_id'] for c in payload['commands']]
    save('document-public-' + outcome + '.json', public_native(handoff.root, ids, monkeypatch))

def test_explicit_constructor_canonical_detail(query_rig, monkeypatch):
    from bioxp import api
    from bioxp.lifecycle_state import CanonicalLifecycleOwner
    from bioxp.pipette.models import PipetteInitCommand
    from bioxp.services.pipette_service import run_pipette_init_command
    from bioxp.operator_reports import create_operator_reports_router
    bind_full_infrastructure_identity(monkeypatch)
    group, wire, timers = transport_rig(tm, 'initial')
    owner = CanonicalLifecycleOwner()
    monkeypatch.setattr(api, 'lifecycle_state', owner)
    monkeypatch.setattr(api, '_pipette_receipts', query_rig[5])
    owner.transport_changed(True, reason='receiving offline merged constructor')
    def action():
        attempt = owner.projection()['startup']['stages']['constructor_pipette_stage']['attempt_id']
        return asyncio.run(run_pipette_init_command(PipetteInitCommand(), get_transport=lambda: group,
            run_blocking=inline, receipt_store=query_rig[5], runtime_binding={
                'entrypoint_id': 'lifecycle.constructor_pipette_stage', 'caller_class': 'lifecycle',
                'lifecycle_stage_id': 'constructor_pipette_stage', 'lifecycle_attempt_id': attempt,
                'idempotency_key': 'constructor:' + attempt}))
    owner.run_stage('constructor_pipette_stage', action)
    app = query_rig[0]
    app.add_api_route('/receiving/startup/{session_id}', api.oem_startup_status, methods=['GET'])
    app.include_router(create_operator_reports_router(app.state.operator_receipt_store))
    before = list(wire)
    response = TestClient(app).get('/receiving/startup/canonical')
    assert response.status_code == 200, response.text
    receipt = query_rig[5].latest()
    evidence = response.json()['lifecycle']['startup']['stages']['constructor_pipette_stage']['evidence']
    assert evidence['receipt_id'] == receipt['receipt_id']
    assert evidence['receipt_truth'] == receipt['truth']
    assert evidence['source_identity'] == receipt['source_identity']
    assert wire == before
    assert owner.projection()['startup']['stages']['constructor_pipette_stage']['evidence'] is None
    from tests.test_w1_diagnostics_connected import database_artifact, subprocess, sys
    row = query_rig[5].connection.execute('SELECT pipette_operation_id FROM pipette_operations ORDER BY rowid DESC LIMIT 1').fetchone()
    db = query_rig[5].connection.execute('PRAGMA database_list').fetchone()[2]
    code = 'import sqlite3,sys; c=sqlite3.connect(sys.argv[1]); print(c.execute("SELECT receipt_json FROM pipette_operations WHERE pipette_operation_id=?",(sys.argv[2],)).fetchone()[0])'
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', code, db, row['pipette_operation_id']], text=True))
    assert reopened == receipt
    database_artifact('explicit-constructor.db', db)
    save('explicit-constructor.json', {'public': response.json(), 'receipt': receipt, 'fresh_process_receipt': reopened})
