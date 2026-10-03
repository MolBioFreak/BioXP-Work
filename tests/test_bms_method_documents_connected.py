"""BMS 9da7c70 emitted bytes -> native parser/finite/store/API.

The captured compiler documents are never rewritten, including embedded BMS
snapshots, occurrence IDs, raw numeric spelling and nulls. Physical leaves and
installation identity are fixture-owned. Exports are actual API readbacks.
"""
import hashlib
import json
import os
from pathlib import Path
import pytest
from bioxp import api
from bioxp.protocols import compile_native_protocol
from bioxp.pipette.receipts import PipetteReceiptStore
from tests.test_cavro_application import rig as wire_rig
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_method_runtime_connected import mount, control, thermal_leaf
from tests.test_protocol_workflow_connected import request_payload, await_job
from tests.test_camera_oem_led_binding import led_rig

SOURCE = Path(__file__).parents[1] / 'testdata/bms_method_finish_documents.json'
DOCUMENTS = json.loads(SOURCE.read_text())
RECIPE_INDICES = [i for i, doc in enumerate(DOCUMENTS) if len(doc['stages']) == 1
    and len(doc['stages'][0]['actions']) == 1
    and doc['stages'][0]['actions'][0]['params'].get('operation') == 'cavro_liquid_recipe']


@pytest.mark.parametrize('index', range(len(DOCUMENTS)))
def test_all_bms_emitted_documents_actual_native_parser(index):
    parsed = compile_native_protocol(DOCUMENTS[index])
    assert parsed.metadata['bms_method']['digest'] == DOCUMENTS[index]['metadata']['bms_method']['digest']


def submit_document(app, client, index, suffix='success'):
    payload = request_payload('bms-' + str(index) + '-' + suffix)
    payload['document'] = DOCUMENTS[index]
    reply = client.post('/protocol/execute', json=payload)
    assert reply.status_code == 202, reply.text
    app.state.operator_command_plane.start()
    return reply.json()['job_id']


def export(client, job, index, suffix, wire, held=None):
    reread = client.get('/protocol/jobs/' + job).json()
    assert reread['protocol']['document'] == DOCUMENTS[index]
    if root := os.environ.get('CAVRO_EVIDENCE_ROOT'):
        target = Path(root, 'native-close-bms-exports'); target.mkdir(exist_ok=True)
        (target / f'bms-{index}-{suffix}.json').write_text(json.dumps({
            'document': DOCUMENTS[index], 'result': reread, 'held': held, 'wire': wire,
            'producer_input_sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
            'producer_compiler_commit': '9da7c709d71d5ee86def2d367ecb101a062c267f',
            'qualification': 'actual BMS document/native dispatcher/store/API; replaced physical leaves'}, indent=2))
    return reread


@pytest.mark.parametrize('index', RECIPE_INDICES)
def test_bms_recipe_finite_dispatch(wire_rig, installed_retained, monkeypatch, tmp_path, index):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    installed_retained[1].primitives.pipette_transport = wire_rig.group
    monkeypatch.setattr(api, '_pipette_receipts', PipetteReceiptStore(installed_retained[4]))
    monkeypatch.setattr(api, '_require_motion_route_ready', lambda: None)
    job = submit_document(app, client, index)
    done = await_job(client, job, lambda r: r['command']['terminal'])
    actual = export(client, job, index, 'success', wire_rig.wire)
    assert actual['execution']['runtime_state'] == done['execution']['runtime_state']
    assert actual['command']['status'] == 'completed', actual['execution']['runtime_state']['action_results']
    rows = actual['execution']['runtime_state']['action_results']
    authored = DOCUMENTS[index]['stages'][0]['actions'][0]
    assert rows[0]['action_id'] == authored['action_id']
    assert rows[0]['metadata'] == authored['metadata']
    assert rows[0]['pipette_result']['liquid_settings']['resolved_recipe'] == authored['params']['recipe']


@pytest.mark.parametrize('phase', ['aspirate', 'dispense'])
def test_bms_recipe_actual_partial(wire_rig, installed_retained, monkeypatch, tmp_path, phase):
    index = 46  # Actual two-segment glycerol recipe and original class provenance.
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    installed_retained[1].primitives.pipette_transport = wire_rig.group
    monkeypatch.setattr(api, '_pipette_receipts', PipetteReceiptStore(installed_retained[4]))
    monkeypatch.setattr(api, '_require_motion_route_ready', lambda: None)
    driver = wire_rig.drivers[1]
    original = driver._send_pipette_command
    failing_wire = 'P22.75,1R' if phase == 'aspirate' else 'D12.75,1R'
    def send(wire, **kw):
        if wire == failing_wire:
            raise RuntimeError('offline mid-' + phase + ' channel-1 failure')
        return original(wire, **kw)
    monkeypatch.setattr(driver, '_send_pipette_command', send)
    job = submit_document(app, client, index, phase)
    done = await_job(client, job, lambda r: r['command']['terminal'])
    actual = export(client, job, index, phase, wire_rig.wire)
    source = actual['execution']['runtime_state']['action_results'][0]['pipette_result']
    assert source['ok'] is False and source['partial_effects'] is True
    assert failing_wire in [w for ch, w in wire_rig.wire if ch == 0]
    assert failing_wire not in [w for ch, w in wire_rig.wire if ch == 1]
    assert 'D10,1R' not in [w for _, w in wire_rig.wire]


@pytest.mark.parametrize('index', [2, 4, 19, 20, 22, 23, 24, 25, 53, 57, 61, 63, 67])
def test_bms_thermal_time_actual_dispatch(installed_retained, monkeypatch, tmp_path, thermal_leaf, index):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    job = submit_document(app, client, index)
    done = await_job(client, job, lambda r: r['command']['terminal'], timeout=20)
    actual = export(client, job, index, 'success', thermal_leaf[1])
    assert actual['command']['status'] == 'completed', done


@pytest.mark.parametrize('index', [47, 52])
def test_bms_manual_diagnostic_and_settings_dispatch(wire_rig, installed_retained, monkeypatch, tmp_path, index):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    installed_retained[1].primitives.pipette_transport = wire_rig.group
    monkeypatch.setattr(api, '_pipette_receipts', PipetteReceiptStore(installed_retained[4]))
    monkeypatch.setattr(api, '_require_motion_route_ready', lambda: None)
    job = submit_document(app, client, index)
    await_job(client, job, lambda r: r['command']['terminal'])
    actual = export(client, job, index, 'success', wire_rig.wire)
    assert actual['command']['status'] == 'completed', actual['execution']['runtime_state']['action_results']


@pytest.mark.parametrize('index', [5, 10, 26, 28, 29, 56])
def test_bms_camera_document_real_owner(installed_retained, led_rig, monkeypatch, tmp_path, index):
    from io import BytesIO
    from types import SimpleNamespace
    from PIL import Image
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    camera, calls, _ = led_rig
    pixels = BytesIO(); Image.new('RGB', (640, 480), 'white').save(pixels, format='JPEG')
    camera._runner = lambda *a, **kw: SimpleNamespace(returncode=0, stdout=pixels.getvalue(), stderr=b'')
    monkeypatch.setattr(api, '_camera_provider', camera)
    job = submit_document(app, client, index)
    done = await_job(client, job, lambda r: r['command']['terminal'] or r['execution']['runtime_state']['workflow']['gate'] == 'error_hold')
    if not done['command']['terminal']:
        control(client, job, done['command']['ownership_generation'], action='abort')
        await_job(client, job, lambda r: r['command']['terminal'])
    actual = export(client, job, index, 'success', [])
    assert actual['command']['status'] == 'completed', actual['execution']['runtime_state']['action_results']
    if DOCUMENTS[index]['stages'][0]['actions'][0]['kind'] == 'snapshot':
        artifact = actual['execution']['runtime_state']['action_results'][0]['artifact']
        assert artifact['artifact_saved'] and Path(artifact['path']).read_bytes() == pixels.getvalue()


@pytest.mark.parametrize('index', [0, 21])
def test_bms_unstarted_timer_actual_error_hold(installed_retained, monkeypatch, tmp_path, index):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    job = submit_document(app, client, index, 'error-hold')
    if index == 21:  # Same missing authored predecessor, with stop policy.
        done = await_job(client, job, lambda r: r['command']['terminal'])
        assert done['command']['status'] == 'failed'
        export(client, job, index, 'missing-timer', [])
        return
    held = await_job(client, job, lambda r: r['execution']['runtime_state']['workflow']['gate'] == 'error_hold')
    generation = int(installed_retained[1].generation_provider())
    assert control(client, job, generation, action='continue', gate='ordinary_pause', gate_id='method-action-0').status_code >= 400
    assert control(client, job, generation, action='abort').status_code in (200, 202)
    await_job(client, job, lambda r: r['command']['terminal'])
    export(client, job, index, 'error-hold', [], held)
