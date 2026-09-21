"""Offline shared-owner binding; real leaf/ABI, mocked device syscalls only."""
import ctypes
import os
import stat
import struct
from types import SimpleNamespace

import pytest
from bioxp import camera_provider as camera
from bioxp.vision import oem_camera_led as smi
from bioxp import oem_preparation_runtime as runtime

XU = bytes.fromhex('1a24060292423946d10ce34a87833133f9eaaa3b08010301ff00')
DEVICE = bytes.fromhex('120100020000004084207df3000101020301')
INTERFACE = bytes.fromhex('09040000010e010000')

@pytest.fixture
def led_rig(tmp_path, monkeypatch):
    usb = tmp_path / 'usb'
    node = usb / 'video0'
    node.mkdir(parents=True)
    (usb / 'idVendor').write_text('2084')
    (usb / 'idProduct').write_text('f37d')
    (usb / 'descriptors').write_bytes(DEVICE + INTERFACE + XU)
    (node / 'dev').write_text('81:0')
    identity = camera.CameraIdentity('/fixture/video0', 'IZONE UVC 5M CAMERA', '2084', 'f37d')
    p = camera.CameraProvider(sysfs_root=usb, generation=10)
    monkeypatch.setattr(p, 'discover', lambda: identity)
    device_stat = SimpleNamespace(st_mode=stat.S_IFCHR, st_dev=1, st_ino=2, st_rdev=os.makedev(81, 0))
    state = SimpleNamespace(opened=[], closed=[], seen=[], fail=None,
        registers={0x0213: 0x2018, 0xD160: 0, 0xD164: 1,
                   smi.GPIO0_REGISTER: 0x80, smi.LED3_REGISTER: 0},
        selected=None, device_stat=device_stat)
    def opened(path, flags):
        state.opened.append((path, flags))
        return 71
    monkeypatch.setattr(camera, 'os', SimpleNamespace(open=opened, close=state.closed.append,
        stat=lambda path: state.device_stat, fstat=lambda fd: device_stat,
        major=os.major, minor=os.minor, O_RDWR=os.O_RDWR, O_NONBLOCK=os.O_NONBLOCK, O_CLOEXEC=os.O_CLOEXEC))
    def ioctl(fd, operation, header, mutate):
        assert p._lock._is_owned()
        assert fd == 71 and mutate is True
        unit, selector, query, size, pointer = struct.unpack('@BBBxHP', header)
        assert (unit, selector) == (2, 1)
        data = ctypes.string_at(pointer, size)
        state.seen.append((query, data))
        if state.fail == len(state.seen):
            raise OSError('uncertain ioctl')
        assert operation == smi.UVCIOC_CTRL_QUERY
        assert query in (smi.UVC_GET_LEN, smi.UVC_GET_INFO,
                         smi.UVC_SET_CUR, smi.UVC_GET_CUR)
        reply = {0x85: b'\x02\x00', 0x86: b'\x03'}.get(query)
        # Source type0 protocol: SET address then GET reads; SET address
        # then SET value writes. Only explicitly modeled registers exist.
        if query == smi.UVC_SET_CUR:
            assert size == 2
            value, = struct.unpack('<H', data)
            if state.selected is None:
                assert value in state.registers, hex(value)
                state.selected = value
            else:
                assert state.selected != 0x0213, 'chip ID is read-only'
                state.registers[state.selected] = value
                state.selected = None
        elif query == smi.UVC_GET_CUR:
            assert size == 2 and state.selected is not None
            reply = struct.pack('<H', state.registers[state.selected])
            state.selected = None
        if reply is not None:
            assert len(reply) == size
            ctypes.memmove(pointer, reply, size)
    monkeypatch.setattr(smi.fcntl, 'ioctl', ioctl)
    return p, state, usb


def test_preflight_only_gets_no_capture_owner(led_rig):
    p, s, _ = led_rig
    assert p.probe_illumination()['control_length'] == 2
    assert s.seen == [(0x85, b'\0\0'), (0x86, b'\0')]
    assert s.opened == [('/fixture/video0', os.O_RDWR | os.O_NONBLOCK | os.O_CLOEXEC)]
    assert p._stream_owner is None
    p.close()
    p.close()
    assert s.closed == [71]


@pytest.mark.parametrize('channel,on', [(1, True), (2, False), (3, True)])
def test_initialize_and_led_same_fd_preserve_preview(led_rig, channel, on):
    p, s, _ = led_rig
    p.begin_stream('preview')
    result = p.initialize_illumination()
    assert result['dsp_type'] == 0 and not result['physical_effect_verified']
    assert s.registers[smi.GPIO0_REGISTER] == 0x80
    assert p.set_illumination(channel=channel, on=on)['ok']
    assert len(s.opened) == 1 and not s.closed
    assert p._stream_owner == 'preview' and p._stream_accepting
    assert all(len(data) == 2 for query, data in s.seen if query in (1, 0x81))
    p.end_stream('preview')
    assert s.closed == [71]


@pytest.mark.parametrize('action', ['invalidate', 'generation', 'descriptor', 'node', 'close'])
def test_owner_change_retires_no_implicit_discovery(led_rig, action):
    p, s, usb = led_rig
    p.begin_stream('preview')
    p.initialize_illumination()
    before = len(s.seen)
    if action == 'invalidate': p.invalidate_stream('preview')
    elif action == 'generation': p._generation += 1
    elif action == 'descriptor': (usb / 'descriptors').write_bytes(DEVICE + INTERFACE + XU + b'\x02\x00')
    elif action == 'node': s.device_stat = SimpleNamespace(st_mode=stat.S_IFCHR, st_dev=1, st_ino=3, st_rdev=os.makedev(81, 0))
    else: p.close()
    with pytest.raises(camera.CameraUnavailable): p.set_illumination(channel=3, on=False)
    assert s.closed == [71] and len(s.seen) == before
    assert len(s.opened) == 1


@pytest.mark.parametrize('fail_at', range(1, 7))
def test_every_initialization_ioctl_failure_closes_without_retry(led_rig, fail_at):
    p, s, _ = led_rig
    s.fail = fail_at
    with pytest.raises(OSError): p.initialize_illumination()
    assert len(s.seen) == fail_at and s.closed == [71]
    with pytest.raises(camera.CameraUnavailable): p.set_illumination(channel=1, on=True)
    assert len(s.seen) == fail_at and len(s.opened) == 1


@pytest.mark.parametrize('raw', [b'', DEVICE + INTERFACE, DEVICE + INTERFACE + XU * 2,
                                  DEVICE + INTERFACE + XU[:-1], DEVICE[:8] + b'\0' * 4 + DEVICE[12:] + INTERFACE + XU])
def test_descriptor_mismatch_never_opens(led_rig, raw):
    p, s, usb = led_rig
    (usb / 'descriptors').write_bytes(raw)
    with pytest.raises(camera.CameraUnavailable): p.probe_illumination()
    assert not s.opened and not s.seen


@pytest.mark.parametrize('chip', [0x3469, 0x347f])
def test_special_dsp_refuses_and_closes(led_rig, chip):
    p, s, _ = led_rig
    s.registers[0x0213] = chip
    with pytest.raises(RuntimeError, match='A390'): p.initialize_illumination()
    assert s.closed == [71]


@pytest.mark.parametrize('offset', [1, 2])
def test_led_uncertain_failure_retires_fd_and_never_replays(led_rig, offset):
    p, s, _ = led_rig
    p.initialize_illumination()
    before = len(s.seen)
    s.fail = before + offset
    with pytest.raises(OSError): p.set_illumination(channel=3, on=True)
    assert s.closed == [71] and len(s.seen) == before + offset
    with pytest.raises(camera.CameraUnavailable): p.set_illumination(channel=3, on=True)
    assert len(s.seen) == before + offset and len(s.opened) == 1


def test_stateful_register_device_preserves_gpio_across_led_sequence(led_rig):
    p, s, _ = led_rig
    s.registers[smi.GPIO0_REGISTER] = 0xA4
    p.initialize_illumination()
    for channel, on, expected in ((1, True, 0xA5), (2, False, 0x25),
                                  (1, False, 0x24), (2, True, 0xA4)):
        assert p.set_illumination(channel=channel, on=on)['ok']
        assert s.registers[smi.GPIO0_REGISTER] == expected
        assert s.selected is None
    for on in (True, False, True, False):
        assert p.set_illumination(channel=3, on=on)['ok']
        assert s.registers[smi.LED3_REGISTER] == int(on)
        assert s.registers[smi.GPIO0_REGISTER] == 0xA4
    assert s.registers[0x0213] == 0x2018
    p.close()


def test_runtime_preflight_and_delivery_do_not_reprobe(led_rig, tmp_path):
    p, s, _ = led_rig
    r = runtime.PreparationCameraRuntime(p, artifact_root=tmp_path)
    r.require_illumination()
    assert all(q in (0x85, 0x86) for q, _ in s.seen)
    r.initialize_illumination()
    before = len(s.seen)
    r.led(channel=3, on=True)
    assert s.seen[before:] == [(1, b'\x7d\xd8'), (1, b'\x01\0')]


@pytest.mark.parametrize('ok', [True, False])
def test_runtime_initialize_child_precedes_native(monkeypatch, tmp_path, ok):
    events = []
    captured = {'source_settings': {'DeckInspection': True, 'InspectionSettings': {}}, 'source_model': {}}
    monkeypatch.setattr(runtime, 'capture_selected_preparation', lambda **kw: captured)
    monkeypatch.setattr(runtime, 'build_preparation_inspection_reader', lambda **kw: None)
    def native(state):
        events.append('native')
        return {'ok': True}
    monkeypatch.setattr(runtime, 'build_prepare_handler', lambda **kw: native)
    def control(operation, arguments, state):
        assert operation == 'preparation_camera_initialize'
        assert arguments == {}
        events.append('child')
        return {'ok': ok}
    r = SimpleNamespace(require_illumination=lambda: events.append('readonly'), capture_image=None)
    run = runtime.bind_selected_preparation(metadata={}, snapshot=None, camera_runtime=r,
        execute_native=None, execute_control=control, sleep=None)
    assert events == ['readonly']
    result = run(SimpleNamespace(events=[]))
    assert result['ok'] is ok
    assert events == (['readonly', 'child', 'native'] if ok else ['readonly', 'child'])


@pytest.mark.parametrize('value,expected', [(None, 0), (1000, 1000)])
def test_exposure_calls_shared_owner_not_cache(tmp_path, value, expected):
    calls = []
    def capture(settings):
        calls.append(settings)
        return SimpleNamespace(frame=SimpleNamespace(content_sha256='fixture'), source_frames_discarded=1)
    r = runtime.PreparationCameraRuntime(SimpleNamespace(capture_inspection=capture), artifact_root=tmp_path)
    assert r.exposure(value=value)['ok']
    assert len(calls) == 1 and calls[0].exposure == expected
