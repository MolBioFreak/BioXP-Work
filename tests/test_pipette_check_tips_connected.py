"""Offline checkTips: real callback/compiler/provider/CV/SQLite publication.

Only motion/camera/illumination boundaries are replaced; images are explicitly
synthetic fixtures, not instrument observations.
"""
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from bioxp.oem_deck_movement import execute_finite_plate_operation
from bioxp.vision.oem_tips import clung_tips, clung_tip_wells
from tests.test_deck_scoped_authority import retained_rig


def image(bits=0, *, reduce=False):
    gray = np.full((480, 640), 255, np.uint8)
    for bit in range(4 if reduce else 8):
        if bits & (1 << bit):
            x = 20 + (111, 237, 363, 489)[bit % 4]
            y = 120 + (60 if reduce else (57, 183)[bit // 4])
            gray[y-30:y+31, x-30:x+31] = 0
    ok, encoded = cv2.imencode('.png', gray)
    assert ok
    return encoded.tobytes()


@pytest.mark.parametrize('reduce,bits', [(False, 0), (False, 1), (False, 255), (True, 15)])
def test_literal_clung_encoding(reduce, bits):
    assert clung_tips(image(bits, reduce=reduce), 100, reduce) == bits * 256 + bits.bit_count() + 1


@pytest.mark.parametrize('well,half,expected', [
    ('A1', 0, (36, 12, 37, 25, 13, 1)),
    ('B1', 0, (37, 25, 13, 1)),
    ('A1', 1, (84, 60, 85, 73, 61, 49)),
    ('B1', 1, (85, 73, 61, 49)),
    ('A12', 0, (47, 23)),
])
def test_source_bit_mapping(well, half, expected):
    assert clung_tip_wells(255 << 8, well, half) == expected
    # Bit3 in 0x28 is masked out by the caller before interpretation.
    assert clung_tip_wells(8 << 8, well, half) == ()


@pytest.fixture
def connected(retained_rig):
    p, primitive, runtime, references, store, root = retained_rig
    trace, receipts, saved = [], [], []
    frames = [image(), image()]
    settings = dict(CheckForStaticTipLoss=True, CameraXOffset=10, CameraYOffset=20,
        InspectionSettings={'ClungTips': dict(Exposure=1000, Gain=1000,
            LED1=False, LED2=True, LED3=False, Parameters={'threshold': 100})})
    primitive.pipette_transport = SimpleNamespace()
    def move(axis, steps):
        trace.append(('move', axis, steps))
        return dict(ok=True, controller_command_acknowledged=True, controller_completion_verified=True)
    primitive.motor_x_move_relative_strict = lambda steps: move('x', steps)
    primitive.motor_y_move_relative_strict = lambda steps: move('y', steps)
    p.sleep = lambda seconds: trace.append(('sleep', seconds))
    def capture(**kwargs):
        trace.append(('capture', kwargs))
        frame = frames.pop(0)
        if isinstance(frame, Exception):
            raise frame
        return {'frame': frame, 'capture_evidence': {'synthetic_fixture': True}}
    def save(**kwargs):
        saved.append(kwargs)
        path = root / (str(len(saved)) + '.png')
        path.write_bytes(kwargs['frame'])
        return {'artifact_saved': True, 'path': str(path)}
    p.bind_oem_cover_inspection_callbacks(settings=lambda: settings, capture=capture,
        save=save, led=lambda **kw: trace.append(('led', kw['channel'], kw['on'])),
        rgb=lambda *args: pytest.fail('no RGB in checkTips'), barcode=lambda frame: '')
    p.bind_tip_tray_state_reader(store.tip_tray_state)
    p.bind_tip_tray_state_publisher(store.publish_tip_tray_transition)
    stamps = p.deck_owner_authority_stamps()
    store.publish_tip_tray_transition(tray_id=0, transition='construct', operation_id='test-constructor',
        command_id='test-constructor', provenance={'synthetic_fixture': True}, **stamps)
    plans = []
    def execute(plan, action, state):
        plans.append(plan)
        def invoke(child):
            result = p.execute_wp8_child(child, command_id=f'test-check-{len(plans)}',
                child_order=child['order'], plan_digest=plan['plan_digest'])
            receipts.append(result)
            return result
        return execute_finite_plate_operation(plan, invoke)
    binding = p.build_oem_pipette_source_callbacks(execute_plan=execute,
        start_child=lambda *args: pytest.fail('no second scheduler'), stopped=lambda: False)['source_bindings']
    from bioxp.protocols.runtime_state import ProtocolSourceModel, SourceTray, SourceWell
    state = SimpleNamespace(source_model=ProtocolSourceModel(tip_trays=[SourceTray(
        '0', 7, [SourceWell('fixture', 0, 50, empty=False) for _ in range(96)], tip_type=50)]))
    return SimpleNamespace(p=p, binding=binding, state=state, settings=settings, trace=trace,
        frames=frames, receipts=receipts, store=store, saved=saved, plans=plans)


def test_connected_loss_publication_and_repeated_already_missing(connected):
    r = connected
    r.frames[:] = [image(1), image(4)]
    assert r.binding.check_tips(0, 'A1', None, r.state)['source_return'] is False
    result = r.receipts[-1]
    assert result['inspection_completed'] is True
    assert result['removed_wells'] == [36, 60]
    assert result['captures'][0]['tipstatus'] == 258
    assert r.store.tip_tray_state(0)['occupancy'][36] is False
    assert r.store.tip_tray_state(0)['occupancy'][60] is False
    assert r.state.source_model.tip_trays[0].wells[36].empty is True
    assert r.state.source_model.tip_trays[0].wells[60].content is None
    assert len(r.saved) == 2  # source flag remains false for the second image
    assert [x for x in r.trace if x[0] == 'move'] == [
        ('move', 'y', 3218), ('move', 'x', -1056), ('move', 'y', 8528)]
    assert r.trace[-3:] == [('led', 1, False), ('led', 2, False), ('led', 3, False)]
    r.frames[:] = [image(1), image(4)]
    assert r.binding.check_tips(0, 'A1', None, r.state)['source_return'] is True
    assert len(r.saved) == 2
    assert r.plans[-1]['operation'] == 'pipette_check_tips'


@pytest.mark.parametrize('well,enabled,tiptype,reason', [
    ('A1', False, 50, 'static_tip_loss_disabled'),
    ('A1', True, 200, '200ul_tip'), ('B12', True, 50, 'tip_location_ge_23')])
def test_source_early_returns(connected, well, enabled, tiptype, reason):
    r = connected
    r.settings['CheckForStaticTipLoss'] = enabled
    r.state.source_model.tip_trays[0].tip_type = tiptype
    assert r.binding.check_tips(0, well, None, r.state)['source_return'] is True
    assert r.receipts[-1]['skip_reason'] == reason
    assert r.receipts[-1]['inspection_completed'] is False
    assert r.trace == [('led', 2, False), ('led', 1, False), ('led', 2, False), ('led', 3, False)]


@pytest.mark.parametrize('well,y', [('B1',1086), ('A12',3218)])
def test_connected_other_pickup_rows(connected, well, y):
    r = connected
    assert r.binding.check_tips(0, well, None, r.state)['source_return'] is True
    assert r.receipts[-1]['inspection_completed'] is True
    assert [x for x in r.trace if x[0] == 'move'][0] == ('move', 'y', y)
    assert not r.saved


@pytest.mark.parametrize('first_loss', [False, True])
def test_camera_exception_retains_source_flag_without_claiming_inspection(connected, first_loss):
    r = connected
    r.frames[:] = [image(1) if first_loss else image(), RuntimeError('offline camera fault')]
    result = r.binding.check_tips(0, 'A1', None, r.state)
    assert result['source_return'] is (not first_loss)
    evidence = r.receipts[-1]
    assert evidence['ok'] is True  # OEM swallowed exception, not optical success
    assert evidence['inspection_completed'] is False
    assert evidence['source_exception'] == 'offline camera fault'
    assert r.trace[-3:] == [('led',1,False),('led',2,False),('led',3,False)]
    assert len(r.frames) == 0  # no invented retry


def test_motion_exception_no_retry_and_final_led_cleanup(connected):
    r = connected
    def fault(steps):
        raise RuntimeError('offline motion fault')
    r.p.primitives.tester.motor_x_move_relative_strict = fault
    result = r.binding.check_tips(0, 'A1', None, r.state)
    assert result['source_return'] is True
    assert result['inspection_completed'] is False
    assert result['source_exception'] == 'offline motion fault'
    assert len(r.frames) == 2
    assert r.trace[-3:] == [('led', 1, False), ('led', 2, False), ('led', 3, False)]


def test_source_ignored_motion_return_is_evidence_not_new_gate(connected):
    r = connected
    r.p.primitives.tester.motor_x_move_relative_strict = lambda steps: {'ok': False, 'source_return_code': 1}
    result = r.binding.check_tips(0, 'A1', None, r.state)
    assert result['source_return'] is True
    assert result['inspection_completed'] is True
    assert r.receipts[-1]['moves'][1]['result']['ok'] is False


def test_final_led_exception_is_not_swallowed(connected):
    r = connected
    def led(*, channel, on):
        if channel == 2 and not on:
            raise RuntimeError('offline cleanup LED fault')
    r.p._oem_cover_inspection_callbacks['led'] = led
    # execute_finite_plate_operation propagates this operation's exception.
    with pytest.raises(RuntimeError, match='offline cleanup LED fault'):
        r.binding.check_tips(0, 'A1', None, r.state)
