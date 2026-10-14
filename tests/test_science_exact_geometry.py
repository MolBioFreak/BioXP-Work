"""Exact authored numbers through finite native dispatch and scratch stores."""
from decimal import Decimal
import pytest
from bioxp import api
from bioxp.pipette.cavro_application import ApplicationRequest, compile_application
from bioxp.pipette.cavro_geometry import SurfaceMove
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from tests.test_cavro_application import rig as wire_rig, request
from tests.test_cavro_surface_coordination import surface, export
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_method_runtime_connected import mount, submit, action
from tests.test_protocol_workflow_connected import await_job
from tests.test_oem_pipette_calibration import MotionNative


@pytest.mark.parametrize('depth,tip,scale,speed,speed_scale,expected', [
    (1.25, .25, 100, 2, 30, (40150, 60)),
    ('0.10000000000000001', '0.89999999999999999', '100.00', '0.10000000000000001', '100000000000000000', (40100, 10000000000000001)),
    (Decimal('0.10000000000000001'), Decimal('0.89999999999999999'), Decimal('100.00'), Decimal('2.00'), Decimal('30.00'), (40100, 60)),
    ('9007199254740993', 0, 1, 1, 1, (9007199254780993, 1)),
])
def test_exact_metric_emitted_native_and_durable(wire_rig, installed_retained, monkeypatch, tmp_path, depth, tip, scale, speed, speed_scale, expected):
    op = surface(detected=False)
    op['displacement'] = {'unit': 'mm', 'depth_mm': depth, 'tip_offset_mm': tip,
        'speed_mm_s': speed, 'calibration': {'calibration_id': 'synthetic-representation-only',
        'steps_per_mm': scale, 'native_speed_per_mm_s': speed_scale, 'positive_steps_direction': 'down'}}
    model = SurfaceMove.model_validate(op)
    normalized = model.model_dump(mode='json')
    for field, original in [('depth_mm', depth), ('tip_offset_mm', tip), ('speed_mm_s', speed)]:
        assert normalized['displacement'][field] == (format(original, 'f') if isinstance(original, Decimal) else original)
    for field, original in [('steps_per_mm', scale), ('native_speed_per_mm_s', speed_scale)]:
        assert normalized['displacement']['calibration'][field] == (format(original, 'f') if isinstance(original, Decimal) else original)
    application = request(normalized)
    plan = compile_application(application)
    assert not plan['issues'], plan
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    provider = installed_retained[1]
    native = MotionNative()
    provider.primitives = Serial206ProductionPrimitiveAdapter(native, wire_rig.group,
        authority_provider=lambda: None, generation_provider=provider.generation_provider)
    receipts = PipetteReceiptStore(installed_retained[4])
    monkeypatch.setattr(api, '_pipette_receipts', receipts)
    monkeypatch.setattr(api, '_require_motion_route_ready', lambda: None)
    job = submit(app, client, [action('pipette_manual_physical', {'operation': 'cavro_application', 'application': application})], key='exact-geometry')
    done = await_job(client, job, lambda r: r['command']['terminal'])
    assert done['command']['status'] == 'completed', done
    result = done['execution']['runtime_state']['action_results'][0]['pipette_result']
    assert result['ok'], result
    assert ('z', expected[0], True) in native.moves
    event = next(e for e in result['events'] if e['operation'] == 'surface_reference')
    assert event['inputs']['target_steps'] == expected[0]
    assert event['inputs']['speed_native'] == expected[1]
    assert event['inputs']['authored']['displacement'] == normalized['displacement']
    assert event['inputs']['authored']['reference'] == normalized['reference']
    from bioxp.operator_command_plane import OperatorCommandStore
    store = app.state.operator_command_plane.store
    ids = done['execution']['runtime_state']['workflow']['child_command_ids']
    finite = [cid for cid in ids if not cid.startswith('pipette_')]
    assert len(finite) == 1
    assert store.wp8_operation_evidence(finite[0])['children'][0]['operation'] == 'sourceCavroApplication'
    reopened = OperatorCommandStore(installed_retained[4])
    try:
        assert reopened.get_workflow(job)['execution']['runtime_state'] == done['execution']['runtime_state']
    finally:
        reopened.stop()
    assert client.get('/protocol/jobs/' + job).json()['execution']['runtime_state'] == done['execution']['runtime_state']
    export('exact-geometry-' + str(depth) + '-' + type(depth).__name__, {'application': application, 'plan': plan, 'job': done, 'moves': native.moves, 'physical_acceptance': False})
    export('exact-geometry-application-schema', ApplicationRequest.model_json_schema())


def test_integer_boundary_native_owner_without_json_float_admission(wire_rig):
    # The workflow document's existing safe-JSON-int gate is not changed.
    # Native Number accepts this integer; exercise its actual emitting owner.
    from tests.test_cavro_application import run
    op = surface(detected=False)
    op['displacement'] = {'unit': 'mm', 'depth_mm': 9007199254740993,
        'tip_offset_mm': 0, 'speed_mm_s': 1, 'calibration': {
        'calibration_id': 'synthetic-integer-boundary', 'steps_per_mm': 1,
        'native_speed_per_mm_s': 1, 'positive_steps_direction': 'down'}}
    plan = compile_application(request(op))
    assert not plan['issues'], plan
    assert plan['operations'][0]['displacement']['depth_mm'] == 9007199254740993
    assert plan['operations'][0]['resolved_geometry']['target_steps'] == 9007199254780993
    result = run(wire_rig, request(op))
    assert result['ok'], result
    assert ('z_move', 9007199254780993) in wire_rig.native
    export('exact-geometry-integer-native', {'request': request(op), 'plan': plan,
        'result': result, 'emitted': wire_rig.native, 'physical_acceptance': False})
