"""Bound BMS full-method documents through real native action owners."""
import pytest
from bioxp import api
from bioxp.pipette.receipts import PipetteReceiptStore
from tests.test_cavro_application import rig as wire_rig
from tests.test_cover_carry_release_connected import connected
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_method_source_mechanisms import local_asyncio
from tests.test_method_runtime_connected import mount, thermal_leaf, control
from tests.test_protocol_workflow_connected import await_job
from tests.test_bms_method_documents_connected import DOCUMENTS, submit_document, export


@pytest.mark.parametrize('index', [11, 12, 13, 14, 15, 16, 17, 18, 54, 64, 58, 65, 39, 44])
def test_bound_bms_examples_actual_dispatch(connected, wire_rig, installed_retained, monkeypatch, tmp_path, thermal_leaf, index):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    connected.provider.primitives.pipette_transport = wire_rig.group
    # Fixture operator setup at the authored setup checkpoint; not a product
    # pickup, runtime assumption, or hardware qualification claim.
    for leaf in wire_rig.group._transports:
        leaf._tip_loaded = True
    monkeypatch.setattr(api, '_get_pipette_transport', lambda: wire_rig.group)
    monkeypatch.setattr(api, '_pipette_receipts', PipetteReceiptStore(installed_retained[4]))
    monkeypatch.setattr(api, '_reference_state_store', installed_retained[3])
    monkeypatch.setattr(api, '_require_motion_route_ready', lambda: None)
    # Existing board-stop physical leaf used by pLLD, with native adapter intact.
    from tests.test_oem_pipette_calibration import MotionNative
    connected.native.motor_oem_board_stop = MotionNative.motor_oem_board_stop.__get__(connected.native)
    job = submit_document(app, client, index)
    reviewed = set()
    snapshots = []
    for _ in range(30):
        state = await_job(client, job, lambda r: r['command']['terminal'] or (
            r['execution']['runtime_state']['workflow']['gate'] in {'review', 'error_hold'} and
            r['execution']['runtime_state']['workflow']['gate_id'] not in reviewed), timeout=30)
        if state['command']['terminal']:
            break
        workflow = state['execution']['runtime_state']['workflow']
        if workflow['gate'] == 'error_hold':
            export(client, job, index, 'unexpected-hold', wire_rig.wire, state)
            control(client, job, state['command']['ownership_generation'], action='abort')
            await_job(client, job, lambda r: r['command']['terminal'])
            pytest.fail(str([(r.get('action_id'), r.get('detail'), r.get('error')) for r in state['execution']['runtime_state']['action_results']]))
        snapshots.append(state)
        reviewed.add(workflow['gate_id'])
        runtime = state['execution']['runtime_state']
        action = next(a for s in DOCUMENTS[index]['stages'] for a in s['actions'] if a.get('source_occurrence_id') == workflow['gate_id'] or a['action_id'] == workflow['gate_id'])
        reply = client.post('/protocol/jobs/' + job + '/review', json={
            'command_id': job, 'idempotency_key': f'review-{index}-{len(reviewed)}',
            'expected_ownership_generation': state['command']['ownership_generation'],
            'stage_id': action['stage_id'], 'action_id': action['action_id'], 'reviewer': 'offline-fixture-operator'})
        assert reply.status_code == 200, reply.text
    else:
        pytest.fail('too many review boundaries')
    actual = export(client, job, index, 'success', {'pipette': wire_rig.wire, 'motion': connected.native.moves}, snapshots)
    assert actual['command']['status'] == 'completed', actual['execution']['runtime_state']['action_results']
    expected = [a for s in DOCUMENTS[index]['stages'] for a in s['actions']]
    rows = actual['execution']['runtime_state']['action_results']
    assert [a['action_id'] for a in rows] == [a['action_id'] for a in expected]
    assert [a['metadata'] for a in rows] == [a['metadata'] for a in expected]
