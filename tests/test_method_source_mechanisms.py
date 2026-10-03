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


@pytest.fixture(autouse=True)
def local_asyncio(connected, monkeypatch):
    def local_socket(family=socket.AF_INET, *args, **kwargs):
        if family != socket.AF_UNIX:
            pytest.fail("offline network socket")
        return _SOCKET(family, *args, **kwargs)
    monkeypatch.setattr(socket, "socket", local_socket)
    connected.provider.primitives.pipette_transport = None
    native = connected.native
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
def test_source_barcode_real_decoder_finite_api(connected, installed_retained, monkeypatch, tmp_path, mode, count):
    from bioxp.camera_provider import CameraProvider, CameraIdentity
    from bioxp.vision.oem_inspection import scan_barcode
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    provider = connected.provider
    image = BytesIO()
    Image.new('RGB', (640, 480), 'white').save(image, format='JPEG')
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
    assert source['value'] == '' and source['source_return'] is False
    assert source['decoded'] is False and source['ok'] is True
    assert len(source['attempts']) == len(captures) == count
    assert len({a['source_identity'] for a in source['attempts']}) == count
    assert all(a['status'] == 'completed' and a['move']['target'] for a in source['attempts'])
    first, second = source['attempts'][:2]
    assert second['offset_x'] == first['offset_x'] + 2000
    assert second['offset_y'] == first['offset_y'] - 4000
    if count == 3:
        assert source['attempts'][2]['offset_x'] == first['offset_x'] - 2000
        assert source['attempts'][2]['offset_y'] == first['offset_y']
    assert rgb == [(255, 255, 255), (11, 22, 33)]
    assert leds[-3:] == [{'channel': i, 'on': False} for i in (1, 2, 3)]
    assert connected.store.deck_semantic_state()['current_location'] == ('LOC_TC' if mode == 'job_id' else 'LOC_RC')
    assert connected.native.moves


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
