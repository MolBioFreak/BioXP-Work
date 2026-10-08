"""Actual native factory -> finite dispatcher -> Cavro CAN/router -> SQLite -> API.

Only physical transport and installation identity are fixture-owned. No fake
handler outcomes, no injected application result and no physical qualification.
"""
import json
import os
from pathlib import Path
import pytest
from bioxp import api
from bioxp.pipette.receipts import PipetteReceiptStore
from tests.test_cavro_application import rig as wire_rig, request, stroke
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_method_runtime_connected import mount, submit, action, control
from tests.test_protocol_workflow_connected import await_job


@pytest.mark.parametrize('failure,policy,on_error', [(None, 'stop', 'stop'), ('device', 'stop', 'stop'), ('missing', 'pause_for_operator', 'stop'), ('device', 'stop', 'pause_for_operator')])
def test_cavro_native_factory_store_api(wire_rig, installed_retained, monkeypatch, tmp_path, failure, policy, on_error):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    provider = installed_retained[1]
    provider.primitives.pipette_transport = wire_rig.group
    receipts = PipetteReceiptStore(installed_retained[4])
    monkeypatch.setattr(api, '_pipette_receipts', receipts)
    # Motion-ready is an installation seam; execution/claim/epoch fences remain.
    monkeypatch.setattr(api, '_require_motion_route_ready', lambda: None)
    if failure:
        wire_rig.fault[failure] = True
    application = request({'operation': 'settings', 'channels': [0, 1], 'timeout_ms': 30,
        'values': {'slope': [1, 2], 'pressure_streaming': True}}, stroke(),
        stroke('dispense', 12.5, 375), stroke('dispense', 10, 875))
    application['event_policy'] = policy
    actions = [action('pipette_manual_physical', {'operation': 'cavro_application',
        'application': application}, on_error=on_error, metadata={'step_id': 'transfer', 'child_index': 2}), action('note')]
    job = submit(app, client, actions, key='cavro-native-' + str(failure))
    held = None
    if policy == 'pause_for_operator' or on_error == 'pause_for_operator':
        held = await_job(client, job, lambda r: r['execution']['runtime_state']['workflow']['gate'] == 'error_hold')
        state = held['execution']['runtime_state']
        assert state['workflow']['gate_id'] == 'pipette_manual_physical'
        assert len(state['action_results']) == 1
        generation = int(provider.generation_provider())
        rejected = control(client, job, generation, action='continue', gate='ordinary_pause', gate_id='pipette_manual_physical')
        assert rejected.status_code >= 400
        before = list(wire_rig.wire)
        assert control(client, job, generation, action='abort').status_code in (200, 202)
    done = await_job(client, job, lambda r: r['command']['terminal'])
    rows = done['execution']['runtime_state']['action_results']
    source = rows[0]['pipette_result']
    assert source['kind'] == 'cavro_application'
    assert source['requested'] == application
    assert rows[0]['metadata'] == actions[0]['metadata']
    assert source['events'][0]['origin'] == 'native_device'
    events = done['execution']['runtime_state']['events']
    assert [e['detail'] for e in events if e['event'] == 'cavro_application_event'] == source['events']
    assert {e['inputs'].get('field') for e in source['events'][:3]} == {'slope', 'pressure_streaming'}
    assert source['events'][0]['result']['channels']
    assert source['events'][0]['reported_applied']['readback'] is None
    if failure:
        assert len(rows) == 1 and not source['ok']
        assert source['requested_control'] == policy
        assert source['partial_effects'] is True
        assert source['events'][-1]['status'] == 'failed'
        assert not any(command.startswith('D') for _, command in wire_rig.wire)
        if held:
            assert wire_rig.wire == before
    else:
        assert done['command']['status'] == 'completed', done
        assert source['ok'] and len(rows) == 2
        assert [command for channel, command in wire_rig.wire if channel == 0] == [
            'b15R', 'o0,1R', 'L1,2R', 'V50,1R', 'P22.5,1R', 'V375,1R', 'D12.5,1R', 'V875,1R', 'D10,1R']
    store = app.state.operator_command_plane.store
    child_ids = done['execution']['runtime_state']['workflow']['child_command_ids']
    finite_ids = [cid for cid in child_ids if not cid.startswith('pipette_')]
    assert len(finite_ids) == 1
    finite = store.wp8_operation_evidence(finite_ids[0])
    assert finite['children'][0]['operation'] == 'sourceCavroApplication'
    durable = json.loads(finite['children'][0]['terminal_evidence_json'])
    from bioxp.protocols.operational_results import compact_application
    assert compact_application(durable['result'])['events'] == source['events']
    assert receipts.read(limit=100)
    reread = client.get('/protocol/jobs/' + job).json()
    assert reread['execution']['runtime_state'] == done['execution']['runtime_state']
    if root := os.environ.get('CAVRO_EVIDENCE_ROOT'):
        Path(root, 'native-exports').mkdir(exist_ok=True)
        Path(root, 'native-exports', 'cavro-' + str(failure) + '-' + on_error + '.json').write_text(json.dumps({
            'document': {'protocol_id': 'offline-host-integration', 'stages': [{'stage_id': 'one', 'actions': actions}]},
            'result': reread, 'held': held, 'wire': wire_rig.wire,
            'qualification': 'offline native factory/finite provider/CAN/router/SQLite/API; physical leaves replaced'}, indent=2))
