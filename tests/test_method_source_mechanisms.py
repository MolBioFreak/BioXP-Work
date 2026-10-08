"""Real multi-pose/positioning owners and publications, physical leaves replaced."""
from io import BytesIO
import socket
_SOCKET = socket.socket
from types import SimpleNamespace
import pytest
from PIL import Image
from bioxp import api
from tests.test_deck_scoped_authority import retained_rig
from tests.test_cover_carry_release_connected import connected, _SETTINGS
from tests.test_deck_scoped_integration import installed_retained
from tests.test_method_runtime_connected import mount, submit, action
from tests.test_protocol_workflow_connected import await_job
from tests.test_deck_tip_query_publication import query_rig


@pytest.fixture(autouse=True)
def local_asyncio(connected, monkeypatch):
    def local_socket(family=socket.AF_INET, *args, **kwargs):
        if family != socket.AF_UNIX:
            pytest.fail("offline network socket")
        return _SOCKET(family, *args, **kwargs)
    monkeypatch.setattr(socket, "socket", local_socket)
    connected.provider.primitives.pipette_transport = None
    native = connected.native
    native._oem_active_board_lifecycle_generation = connected.provider._load_state()['z_lifecycle']['board_lifecycle_generation']
    from bioxp.usb_driver import BioXpTester
    profile = native._motion_oem_axis_profile
    native._motion_oem_axis_profile = lambda axis, startup=False: profile(axis, startup=startup)
    connected.provider.primitives._z_profile_overrides = {}
    native._tmcl_success = BioXpTester._tmcl_success
    native.oem_current_board_lifecycle_generation = lambda: 3
    native.motor_oem_require_no_motion_profile = BioXpTester.motor_oem_require_no_motion_profile.__get__(native)
    for axis in ("x", "z"):
        preset = native._motion_oem_axis_profile(axis)
        for param, value in ((4, preset["speed"]), (5, preset["acc"]), (6, preset["run_current"]), (205, preset["stall_guard"]), (12, 1)):
            native.parameters[preset["board"], preset["motor"], param] = value


@pytest.mark.parametrize('mode,count' , [('job_id', 3), ('reagent_id', 2)])
@pytest.mark.parametrize('barcode', ['', 'Sample-42', 'BLACK'])
def test_source_barcode_real_decoder_finite_api(connected, installed_retained, monkeypatch, tmp_path, mode, count, barcode):
    from bioxp.camera_provider import CameraProvider, CameraIdentity
    from bioxp.vision.oem_inspection import scan_barcode
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    provider = connected.provider
    image = BytesIO()
    pixels = Image.new('RGB', (640, 480), 'white')
    if barcode:
        import cv2
        code = cv2.QRCodeEncoder_create().encode(barcode)
        qr = Image.fromarray(code).resize((280, 280), Image.Resampling.NEAREST).convert('RGB')
        pixels.paste(qr, (160, 100))
    pixels.save(image, format='JPEG')
    assert scan_barcode(image.getvalue()) == barcode
    captures, leds, rgb = [], [], []
    def capture(argv, **kwargs):
        captures.append(argv)
        return SimpleNamespace(returncode=0, stdout=image.getvalue(), stderr=b'')
    camera = CameraProvider(runner=capture)
    monkeypatch.setattr(camera, 'discover', lambda: CameraIdentity('/fixture/video0', 'camera', '2084', 'f37d'))
    monkeypatch.setattr(api, '_camera_provider', camera)
    monkeypatch.setattr(api, '_require_motion_route_ready', lambda: None)
    def frame(**kwargs):
        value = camera.capture()
        return {'frame': value.content, 'capture_evidence': {'sequence': value.sequence, 'generation': value.provider_generation}}
    provider.bind_oem_cover_inspection_callbacks(settings=lambda: _SETTINGS,
        capture=frame, save=lambda **kw: {'ok': True},
        led=lambda **kw: leds.append(kw), rgb=lambda *v: rgb.append(v),
        rgb_state=lambda: [11, 22, 33], barcode=scan_barcode)
    job = submit(app, client, [action('barcode_read', {'mode': mode})], key='barcode-' + mode)
    done = await_job(client, job, lambda r: r['command']['terminal'], timeout=20)
    row = done['execution']['runtime_state']['action_results'][0]
    assert done['command']['status'] == 'completed', row
    source = row['barcode_result']
    positive = bool(barcode and barcode != 'BLACK')
    expected = barcode.lower() if positive or (barcode == 'BLACK' and mode == 'job_id') else ''
    assert source['value'] == expected
    assert source['source_return'] is bool(expected)
    assert source['decoded'] is positive and source['ok'] is True
    if positive:
        count = 1
    assert len(source['attempts']) == len(captures) == count
    assert len({a['source_identity'] for a in source['attempts']}) == count
    assert all(a['status'] == 'completed' and a['move']['target'] for a in source['attempts'])
    first = source['attempts'][0]
    if count > 1:
        second = source['attempts'][1]
        assert second['offset_x'] == first['offset_x'] + 2000
        assert second['offset_y'] == first['offset_y'] - 4000
    if count == 3:
        assert source['attempts'][2]['offset_x'] == first['offset_x'] - 2000
        assert source['attempts'][2]['offset_y'] == first['offset_y']
    assert rgb == [(255, 255, 255), (11, 22, 33)]
    assert leds[-3:] == [{'channel': i, 'on': False} for i in (1, 2, 3)]
    assert connected.store.deck_semantic_state()['current_location'] == ('LOC_TC' if mode == 'job_id' else 'LOC_RC')
    assert connected.native.moves


@pytest.mark.parametrize('already_parked', [False, True])
def test_ordinary_park_runs_source_finite_child(connected, query_rig, monkeypatch, tmp_path, already_parked):
    from tests.test_pipette_constructor_collection import constructor
    app, client = mount(query_rig, monkeypatch, tmp_path)
    monkeypatch.setattr(api, '_require_motion_route_ready', lambda: None)
    constructor(query_rig, monkeypatch)
    connected.provider.primitives.pipette_transport = query_rig[8]
    if already_parked:
        connected.provider.wp8_update_location('updateLocation', {'destination': 28, 'well': 0},
            command_id='fixture-park', child_order=0, plan_digest='fixture')
    before = list(connected.native.moves)
    job = submit(app, client, [action('park', {'rehome': False})], key='source-park')
    done = await_job(client, job, lambda r: r['command']['terminal'])
    assert done['command']['status'] == 'completed', done['execution']['runtime_state']['action_results']
    assert connected.store.deck_semantic_state()['current_location'] == 'LOC_PARK'
    ids = done['execution']['runtime_state']['workflow']['child_command_ids']
    finite = app.state.operator_command_plane.store.wp8_operation_evidence(ids[0])
    assert finite['children'][0]['operation'] == 'parkGantry'
    assert (connected.native.moves == before) is already_parked


@pytest.mark.parametrize('pattern,failure', [('d', False), ('r', False), ('h', False), ('t', False), ('h', True)])
def test_pipette_pierce_existing_mov_owner(connected, installed_retained, monkeypatch, tmp_path, pattern, failure):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    monkeypatch.setattr(api, '_require_motion_route_ready', lambda: None)
    store = app.state.operator_command_plane.store
    # Source logical plate locations, authored fixture predecessor only.
    plate = 0 if pattern == 't' else 2
    connected.provider.wp8_update_plate_location('updatePlateLocation', {'plate': plate, 'location': 3 if plate == 2 else 2},
        command_id='pierce-fixture', child_order=0, plan_digest='fixture')
    if failure:
        from bioxp.oem_compat.position_table import load_bound_oem_position_table
        connected.native.fail_at = ('x', load_bound_oem_position_table().resolve(location_id='LOC_RC').oem_move_to_coordinates(column=0, row=0, high_pos=False)['x'] + 350)
    job = submit(app, client, [action('pipette_pierce', {'plate': plate, 'well': 'A1', 'pattern': pattern})], key='pierce-' + pattern)
    done = await_job(client, job, lambda r: r['command']['terminal'], timeout=20)
    import os, json
    from pathlib import Path
    if root := os.environ.get('CAVRO_EVIDENCE_ROOT'):
        Path(root, 'pierce-' + pattern + '.json').write_text(json.dumps(done, indent=2))
    rows = done['execution']['runtime_state']['action_results']
    if failure:
        assert not rows[0]['ok'] and done['command']['status'] == 'ambiguous'
        failed = [c for c in rows[0]['child_outcomes'] if c['status'] == 'ambiguous']
        assert failed and failed[0]['provider_result']['provider_results']
        assert 'injected_native_transfer_failure' in repr(failed)
        assert 'well_pierced' not in store.deck_semantic_state() or not store.deck_semantic_state()['well_pierced'].get(f'{plate}:0:0')
        return
    assert done['command']['status'] == 'completed', rows
    state = store.deck_semantic_state()
    assert state['well_pierced'][f'{plate}:0:0'] is True
    assert connected.native.moves
    rows = done['execution']['runtime_state']['action_results']
    assert rows[0]['kind'] == 'pipette_pierce'
    assert rows[0]['command']['status'] == 'completed'
