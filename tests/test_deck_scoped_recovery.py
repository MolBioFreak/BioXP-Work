"""Readback-only first-move recovery through the existing governed route."""
import json
from pathlib import Path
import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_recovery_receipt_projection import test_post_delivery_failure_keeps_truth_and_recovery_class as _make_failed_command
from tests.test_deck_postmove_reference import USBLeaf
from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from bioxp.services.reference_service import MarkAxisDesyncedCommand


@pytest.fixture
def stopped_failure(installed_retained, monkeypatch, tmp_path):
    export = tmp_path / 'failed.json'
    monkeypatch.setenv('POSTMOVE_RECEIPT_EXPORT', str(export))
    _make_failed_command(installed_retained, monkeypatch)
    data = json.loads(export.read_text())
    data['command_id'] = data['detail']['command_id']
    app, provider, primitive, refs, root = installed_retained
    leaf = USBLeaf()
    # The controlled controller leaf reports the independently observed target,
    # not a fabricated semantic location. Exact matching is production code.
    leaf.positions.update({(5,0):26213, (4,0):42413, (4,1):0})
    adapter = Serial206ProductionPrimitiveAdapter(leaf, None,
        authority_provider=lambda:{}, generation_provider=provider.generation_provider, reference_store=refs)
    monkeypatch.setattr(primitive, '_read_axis_position', adapter._read_axis_position)
    monkeypatch.setattr(primitive, 'read_deck_semantic_observation', adapter.read_deck_semantic_observation, raising=False)
    return app, provider, primitive, refs, root, leaf, data


def body(provider):
    return {'schema_version':'bioxp.operator_deck_reconciliation_request.v1', 'current_location':'LOC_OC', 'current_well':0,
        'decision_id':'isolated-exact-readback', 'approved_by':'isolated-operator',
        'reason':'Exact stopped controller readbacks match calibrated LOC_OC; no motion replay.',
        'operator_ack':'RECONCILE_DECK'}


def test_exact_first_move_recovery_does_not_replay(stopped_failure, monkeypatch):
    app, provider, primitive, refs, root, leaf, data = stopped_failure
    store = app.state.operator_command_plane.store
    before = store.get_command(data['command_id'])
    stored_before = dict(store.connection.execute('SELECT * FROM operator_plane_commands WHERE command_id=?',(data['command_id'],)).fetchone())
    assert store.deck_semantic_state()['current_location'] is None
    count = sum(row[0]=='move' for row in primitive.calls)
    with TestClient(app) as client:
        result = client.post('/operator/recovery/deck/'+data['command_id']+'/reconcile', json=body(provider))
        assert result.status_code == 200, result.text
    semantic = store.deck_semantic_state()
    assert semantic['current_location']=='LOC_OC' and semantic['current_well']==0
    assert semantic['tip_dirty'] is None and semantic['plate_on_gantry'] is None
    assert semantic['ambiguity_state']=='none'
    after = store.get_command(data['command_id'])
    assert dict(store.connection.execute('SELECT * FROM operator_plane_commands WHERE command_id=?',(data['command_id'],)).fetchone()) == stored_before
    # The separate reconciliation advances the disclosure sequence, not the
    # original command outcome or its stored stages/terminal payload.
    assert {k:v for k,v in after.items() if k!='transition_sequence'} == {k:v for k,v in before.items() if k!='transition_sequence'}
    assert sum(row[0]=='move' for row in primitive.calls)==count and leaf.moves==[]
    assert store._deck_recovery_blocker(store.connection) is None


@pytest.mark.parametrize('fault', ['wrong_expected_location','no_match','reference_lost','wrong_generation'])
def test_scoped_recovery_retains_fences(stopped_failure, fault, monkeypatch):
    app, provider, primitive, refs, root, leaf, data = stopped_failure
    store = app.state.operator_command_plane.store
    before = store.deck_semantic_state()
    request = body(provider)
    if fault == 'wrong_expected_location': request['current_location']='LOC_MS'
    elif fault == 'no_match': leaf.positions[5,0] += 1
    elif fault == 'reference_lost': refs.mark_desynced(MarkAxisDesyncedCommand('x',reason='independent invalidation'))
    elif fault == 'wrong_generation': monkeypatch.setattr(provider, 'generation_provider', lambda: 999)
    with TestClient(app) as client:
        result=client.post('/operator/recovery/deck/'+data['command_id']+'/reconcile',json=request)
        assert result.status_code in {409,503},result.text
    assert store.deck_semantic_state()==before
    assert store._deck_recovery_blocker(store.connection)=='deck_recovery_hold'
    assert leaf.moves==[]
