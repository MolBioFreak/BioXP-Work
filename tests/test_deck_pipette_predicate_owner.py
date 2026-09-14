"""OEM TipExist must not be replaced by stale MachineStatus.TipLoaded.

Real query/SQLite/catalog/queued Park owners; only the existing hardware leaves
are doubles. Denying a deck state UPDATE must neither hide a recorded positive
nor introduce a blanket motor reset/storage-error lockout for verified absence.
"""
import json
import os
import sqlite3
import time
from pathlib import Path
import pytest
from fastapi.testclient import TestClient
from tests.test_deck_tip_query_publication import query_rig
from tests.test_deck_scoped_integration import installed_retained, catalog_action
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_tip_query_publication_contradiction import warm_no_tip, invoke


@pytest.mark.parametrize('tip_present', [False, True])
def test_park_uses_pipette_owner_after_deck_publication_refusal(query_rig, monkeypatch, tip_present):
    from bioxp import api, oem_machine_bundle
    from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
    from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot
    snapshot = bind_serial206_oem_snapshot(monkeypatch)
    snapshot = oem_machine_bundle.load_oem_machine_snapshot(
        snapshot.bundle_root / 'OEM_EVIDENCE_LOCK.json', operator_label_serial=206,
        require_operator_label=True)
    monkeypatch.setattr(oem_machine_bundle, '_active_snapshot', snapshot)
    rig = query_rig
    rig[2].oem_initialize_motion_scriptmove_to_waste = (
        Serial206ProductionPrimitiveAdapter.oem_initialize_motion_scriptmove_to_waste.__get__(rig[2]))
    before = warm_no_tip(rig)
    app, provider, primitive, references, root, receipts, calls, wire, transport = rig
    refs_before = references.snapshot(('x','y','z','g'))['rows']
    denied = []
    def authorizer(action, table, column, database, trigger):
        if action == sqlite3.SQLITE_UPDATE and table == 'operator_plane_deck_semantic_state':
            denied.append((table, column))
            return sqlite3.SQLITE_DENY
        return sqlite3.SQLITE_OK
    connection = app.state.operator_command_plane.store.connection
    connection.set_authorizer(authorizer)
    wire['data'][0] = [32,96,49 if tip_present else 48]
    try:
        response = invoke(rig, 'predicate-owner-publication-refusal')
    finally:
        connection.set_authorizer(None)
    assert denied
    assert response.status_code == 200, response.text
    observed = response.json()
    assert observed['deck_state_publication']['status'] == 'blocked'
    assert observed['semantic_query_response_verified'] is True
    assert observed['source_tip_exists'] is tip_present
    assert app.state.operator_command_plane.store.deck_semantic_state() == before
    assert references.snapshot(('x','y','z','g'))['rows'] == refs_before
    source_before = [t._get_driver()._pipette_message_state['tip_loaded'] for t in transport._transports]
    assert source_before == [tip_present,False,False,False]
    api._collect_and_publish_hardware_snapshot(['axes','latch'], reason='isolated-predicate-owner-regression')
    action = catalog_action(app)
    before_calls = len(primitive.calls)
    client = TestClient(app)
    submitted = client.post('/operator/v2/actions/oem.deck.move_to_location', json={
        'schema_version': 'bioxp.operator_action_request.v2',
        'idempotency_key': 'park-after-refused-publication',
        'expected_ownership_generation': provider.generation_provider(),
        'expected_board_epoch_by_board': action['expected_board_epoch_by_board'],
        'inputs': {'target': 'LOC_PARK', 'camera_offset': False}})
    assert submitted.status_code in (200,409), submitted.text
    receipt = None
    if submitted.status_code == 200:
        route = '/operator/v2/actions/receipts/' + submitted.json()['command_id']
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            receipt = client.get(route).json()
            if receipt.get('terminal') is True:
                break
            time.sleep(.01)
        assert receipt and receipt.get('terminal') is True, receipt
        receipt = client.get(route+'?detail=true').json()
    leaf_calls = primitive.calls[before_calls:]
    no_tip_moves = [row for row in leaf_calls if row[0] == 'move' and row[2].get('tip_loaded') is False]
    if os.environ.get('DECK_TEST_OUTPUT'):
        Path(os.environ['DECK_TEST_OUTPUT'] + f'.predicate-owner-{tip_present}.json').write_text(json.dumps({
            'query': observed, 'source_before': source_before, 'retained_before': before,
            'admission_code': submitted.status_code, 'admission': submitted.json(),
            'receipt': receipt, 'leaf_double_calls': leaf_calls,
            'canonical_after': app.state.operator_command_plane.store.deck_semantic_state(),
            'actual_hardware_access': False}, indent=2))
    if tip_present:
        assert not no_tip_moves, 'Park used stale no-tip MachineStatus despite current positive pipette owner'
    else:
        assert submitted.status_code == 200, submitted.text
        assert receipt['status'] == 'completed', receipt
        assert no_tip_moves, 'Verified absence must retain the normal OEM no-tip branch'
