"""Immutable final BMS recovery/control producers, no rewritten job state."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import pytest
from bioxp import api
from bioxp.pipette.receipts import PipetteReceiptStore
from tests.test_cavro_application import rig as wire_rig
from tests.test_cover_carry_release_connected import connected
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_method_source_mechanisms import local_asyncio
from tests.test_method_runtime_connected import mount, control
from tests.test_protocol_workflow_connected import await_job, request_payload

SOURCE = Path(__file__).parents[1] / 'testdata/final_recovery_documents.json.gz'
DOCUMENTS = json.loads(gzip.decompress(SOURCE.read_bytes()))


def capture(client, job, index, name, wire):
    result = client.get('/protocol/jobs/' + job).json()
    assert result['protocol']['document'] == DOCUMENTS[index]
    root = Path(os.environ['CAVRO_EVIDENCE_ROOT'], 'exports'); root.mkdir(exist_ok=True)
    (root / ('recovery-' + name + '.json.gz')).write_bytes(gzip.compress(json.dumps({
        'document': DOCUMENTS[index], 'result': result, 'wire': wire,
        'producer_input_sha256': hashlib.sha256(gzip.decompress(SOURCE.read_bytes())).hexdigest(),
        'qualification': 'Actual BMS run snapshot/native finite owner/SQLite/API; physical leaves inert'}, indent=2).encode(), mtime=0))
    return result


@pytest.mark.parametrize('phase', ['post-pickup', 'mid-aspirate', 'mid-dispense', 'held'])
def test_recovery_producer(connected, wire_rig, installed_retained, monkeypatch, tmp_path, phase):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    provider = connected.provider
    state = provider._load_state()
    state['machine_status']['constructed_tip_trays'] = provider._new_state()['machine_status']['constructed_tip_trays']
    provider._save_state(state)
    provider.primitives.pipette_transport = wire_rig.group
    from tests.test_oem_pipette_calibration import MotionNative
    connected.native.motor_oem_move_z_home = MotionNative.motor_oem_move_z_home.__get__(connected.native)
    monkeypatch.setattr(api, '_get_pipette_transport', lambda: wire_rig.group)
    monkeypatch.setattr(api, '_pipette_receipts', PipetteReceiptStore(installed_retained[4]))
    monkeypatch.setattr(api, '_reference_state_store', installed_retained[3])
    monkeypatch.setattr(api, '_require_motion_route_ready', lambda: None)
    index = 1 if phase == 'held' else 0
    recipe = DOCUMENTS[index]['stages'][0]['actions'][1]['params']['recipe']
    fail = ('P' + str(recipe['leading_air']['volume_ul']).removesuffix('.0') + ',1R' if phase in ('post-pickup', 'held') else
            'P' + str(recipe['commanded_aspiration_ul']).removesuffix('.0') + ',1R' if phase == 'mid-aspirate' else
            'D' + str(recipe['dispense_segments'][0]['volume_ul']).removesuffix('.0') + ',1R')
    for leaf_driver in wire_rig.drivers:
        original_send = leaf_driver._send_pipette_command
        def query_exchange(wire, _send=original_send, _driver=leaf_driver, **kw):
            if wire == '?31':
                wire_rig.wire.append((_driver.pipette_id, wire))
                return {'ok': True, 'tx_ok': True, 'ack': {'received': True, 'data': [32, 96, 49]}}
            return _send(wire, **kw)
        monkeypatch.setattr(leaf_driver, '_send_pipette_command', query_exchange)
    driver = wire_rig.drivers[0 if phase in ('post-pickup', 'held') else 1]
    send = driver._send_pipette_command
    def exchange(wire, **kw):
        if wire == fail:
            raise RuntimeError('offline recovery ' + phase)
        return send(wire, **kw)
    monkeypatch.setattr(driver, '_send_pipette_command', exchange)
    payload = request_payload('final-recovery-' + phase); payload['document'] = DOCUMENTS[index]
    reply = client.post('/protocol/execute', json=payload)
    assert reply.status_code == 202, reply.text
    job = reply.json()['job_id']; app.state.operator_command_plane.start()
    done = await_job(client, job, lambda r: r['command']['terminal'] or r['execution']['runtime_state']['workflow']['gate'] == 'error_hold', timeout=40)
    result = capture(client, job, index, phase, wire_rig.wire)
    rows = result['execution']['runtime_state']['action_results']
    assert rows[0]['ok'], rows
    assert rows[0]['kind'] == 'pipette_manual_physical'
    assert len(rows) == 2 and not rows[1]['ok'], rows
    assert 'offline recovery' in json.dumps(rows[1])
    if phase == 'held':
        assert result['execution']['runtime_state']['workflow']['gate'] == 'error_hold'
        control(client, job, result['command']['ownership_generation'], action='abort')
        await_job(client, job, lambda r: r['command']['terminal'])
        capture(client, job, index, 'aborted', wire_rig.wire)


@pytest.mark.parametrize('mode', ['abort', 'ordinary', 'deferred', 'safe_stop'])
def test_control_producer(installed_retained, monkeypatch, tmp_path, mode):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    index = 2 if mode == 'abort' else 3
    payload = request_payload('final-controls-' + mode); payload['document'] = DOCUMENTS[index]
    reply = client.post('/protocol/execute', json=payload)
    assert reply.status_code == 202, reply.text
    job = reply.json()['job_id']; app.state.operator_command_plane.start()
    running = await_job(client, job, lambda r: bool(r['execution']['runtime_state']['action_results']))
    generation = running['command']['ownership_generation']
    if mode in ('abort', 'safe_stop'):
        reply = control(client, job, generation, action=mode)
        assert reply.status_code in (200, 202), reply.text
    else:
        reply = control(client, job, generation, action='pause', mode=mode)
        if mode == 'deferred':
            assert reply.status_code == 409 and reply.json()['detail']['reason'] == 'Deferred pause requires OEM lifecycle'
            refused = [{'action': 'pause', 'mode': mode, 'status': reply.status_code, 'response': reply.json()}]
            wake = control(client, job, generation, action='wake', gate_id='not-reached')
            assert wake.status_code == 409
            refused.append({'action': 'wake', 'status': wake.status_code, 'response': wake.json()})
            await_job(client, job, lambda r: r['command']['terminal'])
            capture(client, job, index, 'control-deferred-wake-refused', refused)
            return
        assert reply.status_code in (200, 202), reply.text
        held = await_job(client, job, lambda r: r['execution']['runtime_state']['workflow']['gate'] == mode + '_pause')
        capture(client, job, index, 'control-' + mode + '-held', [])
        gate = held['execution']['runtime_state']['workflow']
        reply = control(client, job, generation, action='continue', gate=gate['gate'], gate_id=gate['gate_id'])
        assert reply.status_code in (200, 202), reply.text
    await_job(client, job, lambda r: r['command']['terminal'])
    capture(client, job, index, 'control-' + mode, [])
