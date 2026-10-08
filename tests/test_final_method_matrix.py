"""Every final BMS snapshot: real parser/dispatcher/owners/SQLite/API.

Physical hardware leaves only are inert. Standalone missing predecessor cases
are retained as expected failures, never rewritten into successful documents.
"""
import gzip
import hashlib
import json
import os
from pathlib import Path
import pytest
from bioxp import api
from bioxp.protocols import compile_native_protocol
from bioxp.pipette.receipts import PipetteReceiptStore
from tests.test_cavro_application import rig as wire_rig
from tests.test_cover_carry_release_connected import connected
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_method_source_mechanisms import local_asyncio
from tests.test_method_runtime_connected import mount, thermal_leaf, control
from tests.test_protocol_workflow_connected import await_job, request_payload
from tests.test_camera_oem_led_binding import led_rig

SOURCE = Path(__file__).parents[1] / 'testdata/final_method_documents.json.gz'
DOCUMENTS = json.loads(gzip.decompress(SOURCE.read_bytes()))


def capture(client, job, index, suffix, wire, held=None):
    result = client.get('/protocol/jobs/' + job).json()
    assert result['protocol']['document'] == DOCUMENTS[index]
    root = Path(os.environ['CAVRO_EVIDENCE_ROOT'], 'exports')
    root.mkdir(exist_ok=True)
    content = {'document': DOCUMENTS[index], 'result': result, 'held': held,
               'wire': wire, 'producer_input_sha256': hashlib.sha256(gzip.decompress(SOURCE.read_bytes())).hexdigest(),
               'qualification': 'Actual immutable BMS snapshot, native owners, SQLite and API; inert physical leaves'}
    (root / f'final-{index:02d}-{suffix}.json.gz').write_bytes(gzip.compress(json.dumps(content, indent=2).encode(), mtime=0))
    return result


@pytest.mark.parametrize('index', [int(os.environ['FINAL_DOCUMENT_INDEX'])] if 'FINAL_DOCUMENT_INDEX' in os.environ else range(len(DOCUMENTS)))
def test_final_document_execution(connected, wire_rig, installed_retained, monkeypatch, tmp_path, thermal_leaf, led_rig, index):
    from io import BytesIO
    from types import SimpleNamespace
    from PIL import Image
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    connected.provider.primitives.pipette_transport = wire_rig.group
    setup = connected.provider._load_state()
    setup['machine_status']['constructed_tip_trays'] = connected.provider._new_state()['machine_status']['constructed_tip_trays']
    connected.provider._save_state(setup)
    monkeypatch.setattr(api, '_can_ready_observation', lambda: True)
    monkeypatch.setattr(thermal_leaf[0], 'send_tmcl_retry', lambda *a, **kw: {'status': 100})
    connected.native.motor_wait_stopped = lambda *a, **kw: {'ok': True, 'target_reached': True}
    def home_axis(axis, **kw):
        connected.native.events.append(('home', axis))
        connected.native.positions[connected.native.addresses[axis]] = 0
        return {'ok': True, 'source_return_code': 0, 'home_verified': True,
                'controller_command_acknowledged': True, 'controller_terminal_state_verified': True,
                'controller_home_proof_verified': True,
                'position_after_sethome': connected.native.motor_get_position(*connected.native.addresses[axis][:1], motor=connected.native.addresses[axis][1])}
    connected.native.motor_oem_go_home = home_axis
    for driver in wire_rig.drivers:
        transact = driver.bus.transact_can
        def synchronous_exchange(msg, _transact=transact, _driver=driver, **kw):
            reply = _transact(msg, **kw)
            if kw.get('wait_for_completion'):
                completion = _driver.bus.wait_pipette_completion(kw['channel'], 1, owner_token=reply['completion_owner_token'])
                reply.update(completion_received=completion['ok'], completion_deferred=False)
                reply['frames'].append({'data': [32, 96], 'dlc': 2, 'arbitration_id': 0x501 + 8 * kw['channel']})
            return reply
        monkeypatch.setattr(driver.bus, 'transact_can', synchronous_exchange)
        original_send = driver._send_pipette_command
        driver.fixture_tip_loaded = True
        def query_exchange(wire, _send=original_send, _driver=driver, **kw):
            if wire in ('E1R', 'E0R'):
                _driver.fixture_tip_loaded = False
            if wire in ('?31', 'Q1', '&1'):
                wire_rig.wire.append((_driver.pipette_id, wire))
                value = (49 if _driver.fixture_tip_loaded else 48) if wire == '?31' else 32 if wire == 'Q1' else 49
                return {'ok': True, 'tx_ok': True, 'ack': {'received': True, 'data': [32, 96, value]}}
            return _send(wire, **kw)
        monkeypatch.setattr(driver, '_send_pipette_command', query_exchange)
    from tests.test_pipette_constructor_collection import constructor
    constructor_rig = [None] * 9
    constructor_rig[4] = installed_retained[4]
    constructor_rig[5] = PipetteReceiptStore(installed_retained[4])
    constructor_rig[6] = []
    constructor_rig[8] = wire_rig.group
    with monkeypatch.context() as constructor_leaves:
        constructor(constructor_rig, constructor_leaves)
    for leaf in wire_rig.group._transports:
        leaf._tip_loaded = True
    monkeypatch.setattr(api, '_get_pipette_transport', lambda: wire_rig.group)
    monkeypatch.setattr(api, '_pipette_receipts', PipetteReceiptStore(installed_retained[4]))
    monkeypatch.setattr(api, '_reference_state_store', installed_retained[3])
    monkeypatch.setattr(api, '_require_motion_route_ready', lambda: None)
    from tests.test_oem_pipette_calibration import MotionNative
    connected.native.motor_oem_board_stop = MotionNative.motor_oem_board_stop.__get__(connected.native)
    connected.native.motor_oem_move_z_home = MotionNative.motor_oem_move_z_home.__get__(connected.native)
    camera, calls, _ = led_rig
    pixels = BytesIO(); Image.new('RGB', (640, 480), 'white').save(pixels, format='JPEG')
    from tests.test_camera_oem_inspection import CONTROLS, DISCOVERY
    import re
    camera_controls = CONTROLS
    inspection_capture = 0
    def camera_exchange(argv, **kw):
        nonlocal camera_controls, inspection_capture
        if argv[0] == 'ffmpeg':
            count = int(argv[argv.index('-frames:v') + 1]) if '-frames:v' in argv else 1
            frame = pixels.getvalue()
            if any(a['kind'] == 'inspect' for s in DOCUMENTS[index]['stages'] for a in s['actions']):
                from tests.test_cover_inspection_flow import _jpeg, _GRAY_WITH_COVER
                import numpy as np
                frame = _jpeg(_GRAY_WITH_COVER if inspection_capture < 2 else np.zeros_like(_GRAY_WITH_COVER))
                inspection_capture += 1
            return SimpleNamespace(returncode=0, stdout=frame * count, stderr=b'')
        assert argv[0] == 'v4l2-ctl'
        if '--set-ctrl' in argv:
            for setting in argv[-1].split(','):
                name, value = setting.split('=')
                camera_controls = re.sub(r'(' + name + r' .*?value=)\d+', lambda match: match[1] + value, camera_controls)
            return SimpleNamespace(returncode=0, stdout=b'', stderr=b'')
        return SimpleNamespace(returncode=0, stdout=(DISCOVERY if argv[-1] == '--all' else camera_controls).encode(), stderr=b'')
    camera._runner = camera_exchange
    monkeypatch.setattr(api, '_camera_provider', camera)
    doc = DOCUMENTS[index]
    compile_native_protocol(doc)
    payload = request_payload(f'final-{index:02d}')
    payload['document'] = doc
    response = client.post('/protocol/execute', json=payload)
    assert response.status_code == 202, response.text
    job = response.json()['job_id']
    app.state.operator_command_plane.start()
    reviews = set()
    held = []
    for _ in range(40):
        state = await_job(client, job, lambda r: r['command']['terminal'] or (
            r['execution']['runtime_state']['workflow']['gate'] in {'review', 'error_hold'} and
            r['execution']['runtime_state']['workflow']['gate_id'] not in reviews), timeout=40)
        if state['command']['terminal']:
            break
        workflow = state['execution']['runtime_state']['workflow']
        held.append(state)
        if workflow['gate'] == 'error_hold':
            capture(client, job, index, 'held', wire_rig.wire, state)
            control(client, job, state['command']['ownership_generation'], action='abort')
            state = await_job(client, job, lambda r: r['command']['terminal'])
            break
        reviews.add(workflow['gate_id'])
        action = next(a for s in doc['stages'] for a in s['actions'] if a.get('source_occurrence_id') == workflow['gate_id'] or a['action_id'] == workflow['gate_id'])
        reply = client.post('/protocol/jobs/' + job + '/review', json={
            'command_id': job, 'idempotency_key': f'review-{index}-{len(reviews)}',
            'expected_ownership_generation': state['command']['ownership_generation'],
            'stage_id': action['stage_id'], 'action_id': action['action_id'], 'reviewer': 'offline-fixture-operator'})
        assert reply.status_code == 200, reply.text
    else:
        pytest.fail('review boundary limit')
    actual = capture(client, job, index, 'execution', {'pipette': wire_rig.wire, 'motion': connected.native.moves}, held)
    rows = actual['execution']['runtime_state']['action_results']
    expected_context_failures = {
        22: 'machine_target_absent_from_serial206_position_table:32',
        29: 'timer_not_started', 40: 'timer_not_started',
        50: 'source_authority_missing:updatePlateLocation', 61: 'source_authority_missing:updatePlateLocation',
        53: 'thermal_door_must_be_open', 54: 'thermal_door_must_be_open',
        66: 'thermal_door_must_be_open', 76: 'thermal_door_must_be_open',
    }
    if index in expected_context_failures:
        assert actual['command']['status'] in {'failed', 'ambiguous', 'aborted'}
        assert expected_context_failures[index] in json.dumps(actual)
        return
    assert actual['command']['status'] == 'completed', [(r.get('kind'), r.get('detail'), r.get('error')) for r in rows]
    expected = [a for s in doc['stages'] for a in s['actions']]
    assert [r['action_id'] for r in rows] == [a['action_id'] for a in expected]
    assert [r['metadata'] for r in rows] == [a['metadata'] for a in expected]
