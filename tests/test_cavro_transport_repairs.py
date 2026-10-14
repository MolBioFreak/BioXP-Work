"""Real driver/USB framing/router/collection + receipt SQLite, inert endpoints.

All frames below are controlled test stimuli, NOT observed hardware responses.
Run with tests.z_stop_offline_guard. No physical driver constructors are called.
"""
import asyncio
import json
import os
import time
from pathlib import Path
from types import SimpleNamespace
import pytest
from bioxp.can_driver import BioXpCanDriver
from bioxp.novo_router import NovoRouter, NovoFrame, PRESSURE_STREAM_IDS
from bioxp.novo_usb_can import NovoUsbCanBus, novo_decode
from bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.protocols.operational_results import application_outcome
from tests.test_cavro_application import rig, run, request, stroke
from tests.test_deck_tip_query_publication import bind_collection_test_identity


@pytest.fixture
def receiver(monkeypatch):
    bind_collection_test_identity(monkeypatch)
    wire, fault, pending = [], {}, {}
    router = NovoRouter(ep_in=object(), ep_out=object(), decode=novo_decode)
    router._reader = SimpleNamespace(is_alive=lambda: True)
    def write(raw, timeout):
        data = novo_decode(raw)
        txid, payload = int.from_bytes(data[:4], 'big'), data[5:]
        wire.append((txid, payload.decode('ascii')))
        ch, function = (txid & 0x78) >> 3, txid & 7
        if function in (3, 4):
            pending[ch] = pending.get(ch, b'') + payload
            return len(raw)
        command = pending.pop(ch, b'') + payload
        def rx(rxid, data):
            router._dispatch(NovoFrame(rxid, len(data), bytes(data), b'', time.monotonic(), 'pipette'))
        if fault.get('missing') == ch:
            return len(raw)
        if function == 6:
            # Wrong-channel input must not satisfy this pending report.
            rx(0x506 | (((ch + 1) % 4) << 3), [32, 96, 57])
            reply = (b'FFFE' if command == b'#' else b'\x20\x601.25')
            if fault.get('malformed'):
                reply = b'\x20\x60NaN'
            rx(txid | 0x400, reply)
        else:
            rx(txid | 0x400, [])
            # Append/control deliberately receive ACK only, no invented completion.
            if command.endswith(b'R') and command not in (b'!R', b'R') and not fault.get('hold'):
                reply = ([32, 96] + list(b'123')) if command == b'd0R' else [33 if fault.get('device') == ch else 32, 96]
                rx(txid | 0x400, reply)
        return len(raw)
    router.ep_out = SimpleNamespace(write=write)
    shared = SimpleNamespace(novo_router=router, ep_in=object(), ep_out=router.ep_out)
    bus = NovoUsbCanBus(shared_usb=shared)
    leaves = []
    for channel in range(4):
        driver = BioXpCanDriver.__new__(BioXpCanDriver)
        driver.bus, driver.pipette_id, driver.response_timeout_s = bus, channel, .02
        driver._pipette_completion_owner_token = None
        driver._pipette_last_command = None
        leaf = CanPipetteTransport(driver_factory=lambda d=driver: d, pipette_id=channel)
        leaves.append(leaf)
    group = FourPipetteTransport(leaves, sleep=lambda _: None)
    return SimpleNamespace(router=router, group=group, wire=wire, fault=fault, bus=bus)


def compile_command(value):
    from bioxp.pipette.cavro_commands import compile_device_command
    return compile_device_command(value)


def test_real_codec_reports_completion_ack_control(receiver):
    from bioxp.pipette.cavro_commands import compile_device_program
    group = receiver.group
    for selector, units in [(57, 'psi'), (56, 'adc_counts')]:
        out = group.run_cavro_command(compile_command({'command': 'report', 'selector': selector}), channels=[0, 2], timeout_ms=20)
        assert out['ok'] and not out['completion_verified']
        for row in out['channels']:
            assert row['result']['report']['value'] == 1.25
            assert row['result']['report']['units'] == units
            assert row['result']['semantic_query_response_verified']
    raw = group.run_cavro_command(compile_command({'command': 'raw_pressure'}), channels=[1], timeout_ms=20)
    assert raw['channels'][0]['result']['report']['value'] == -2
    command = {'command': 'plunger_aspirate', 'value': 22.5, 'unit': 'ul'}
    move = group.run_cavro_command(compile_command(command), channels=[0, 1], timeout_ms=20)
    assert move['ok'] and move['completion_verified']
    account = move['channels'][0]['result']['accounting']
    assert account['pneumatic_displacement_delta_ul'] == 22.5
    assert account['intended_liquid_delta_ul'] is None and account['measured_liquid_ul'] is None
    program = compile_device_program({'mode': 'append', 'commands': [command, command]})
    appended = group.run_cavro_command(program, channels=[0], timeout_ms=20)
    assert appended['ok'] and not appended['completion_verified']
    assert appended['command_effects'] == program['command_effects']
    assert receiver.router._pipette_completions.get(0) is None
    control = group.run_cavro_command(compile_command({'command': 'buffer_clear'}), channels=[0], timeout_ms=20)
    assert control['ok'] and not control['completion_verified']
    assert any(tx & 7 == 3 for tx, _ in receiver.wire)
    export('typed-receiving.json', {'report': raw, 'move': move, 'append': appended, 'control': control, 'wire': receiver.wire})


def test_active_diagnostic_answer_and_explicit_heartbeat(receiver):
    result = receiver.group.run_cavro_command(compile_command({'command':'diagnostic','test':'minimum_moving_current'}), channels=[0], timeout_ms=20)
    assert result['ok'] and result['completion_verified']
    assert result['channels'][0]['result']['report']['value'] == 123
    assert result['channels'][0]['result']['report']['units'] == 'ma'
    result = receiver.group.run_cavro_command(compile_command({'command':'heartbeat_period','half_seconds':15}), channels=[0], timeout_ms=20)
    assert result['ok'] and result['persistent']
    assert receiver.wire[-1][1] == 'b15R'


def test_typed_terminate_detaches_exact_old_completion(receiver):
    receiver.fault['hold'] = True
    driver = receiver.group._transports[0]._get_driver()
    sent = driver.send_cavro_command(compile_command({'command':'plunger_aspirate','value':1,'unit':'ul'}), timeout_ms=20)
    owner = sent['provenance']['completion_owner_token']
    result = receiver.group.run_cavro_command(compile_command({'command':'terminate'}), channels=[0], timeout_ms=20)
    assert result['ok'] and not result['completion_verified']
    previous = receiver.router.wait_pipette_completion(0, 0, owner_token=owner)
    assert not previous['ok'] and previous['outcome'] == 'interrupted_by_terminate'
    assert receiver.wire[-1][1] == 'T'


def test_real_partial_completion_and_missing_report(receiver):
    receiver.fault['device'] = 1
    result = receiver.group.run_cavro_command(compile_command({'command':'plunger_dispense','value':2,'unit':'ul'}), channels=[0,1], timeout_ms=20)
    assert not result['ok']
    assert result['channels'][0]['completion']['ok']
    assert not result['channels'][1]['completion']['ok']
    assert result['channels'][0]['result']['accounting']['pneumatic_displacement_delta_ul'] == -2
    assert 'accounting' not in result['channels'][1]['result']
    receiver.fault['missing'] = 2
    result = receiver.group.run_cavro_command(compile_command({'command':'report','selector':57}), channels=[0,2], timeout_ms=5)
    assert not result['ok'] and result['channels'][0]['result']['report']['value'] == 1.25
    assert result['channels'][1]['result']['report']['value'] is None


def test_report_serialization_not_actuator_completion(receiver, rig):
    compiled = compile_command({'command':'report','selector':57})
    result = rig.provider._manual_pipette_receipt_runner('cavro_device_command',
        lambda _: receiver.group.run_cavro_command(compiled, channels=[0, 1], timeout_ms=20),
        'offline-finite-owner', {'source_identity':'typed-report-test'}, compiled['requested'])
    assert result['receipt_truth']['semantic_query_response_verified']
    assert not result['receipt_truth']['completion_verified']
    row = rig.store.connection.execute('SELECT receipt_json FROM pipette_operations WHERE command_id=?', (result['command_id'],)).fetchone()
    receipt = json.loads(row[0])
    assert receipt['result']['channels'][1]['result']['report']['units'] == 'psi'
    assert receipt['result']['channels'][1]['result']['report']['value'] == 1.25
    export('typed-report-finite-receipt.json', {'result':result, 'receipt':receipt})


def test_malformed_numeric_is_observation_not_new_gate(receiver):
    receiver.fault['malformed'] = True
    result = receiver.group.run_cavro_command(compile_command({'command':'report','selector':57}), channels=[0], timeout_ms=20)
    assert result['ok']  # Existing correlated transport outcome is not rewritten.
    assert not result['semantic_query_response_verified']
    assert result['channels'][0]['result']['report']['raw_ascii'] == 'NaN'
    assert result['channels'][0]['result']['report']['value'] is None


def test_send_exception_preserves_failed_channel(receiver):
    receiver.fault['missing'] = 1
    with pytest.raises(Exception) as raised:
        receiver.group.run_cavro_command(compile_command({'command':'plunger_aspirate','value':2,'unit':'ul'}), channels=[0,1], timeout_ms=5)
    partial = raised.value.pipette_partial_result
    assert partial['failed_channel'] == 1
    assert partial['channels'][0]['channel'] == 0
    assert partial['failure_details']['driver_result']['ok'] is False
    projected = application_outcome(partial)
    assert projected['failed_channel'] == 1
    assert projected['failure_details'] == partial['failure_details']


def test_pressure_receive_does_not_block_or_extend_deadline(receiver):
    router = receiver.router
    capture = router.begin_scientific_pressure_capture({'channels':[0], 'max_frames':1, 'duration_ms':20})
    data = b'1\x00\x01'
    capture.lock.acquire()
    try:
        router._dispatch(NovoFrame(min(PRESSURE_STREAM_IDS),3,data,b'',time.monotonic(),'pressure'))
    finally:
        capture.lock.release()
    router._dispatch(NovoFrame(min(PRESSURE_STREAM_IDS),3,data,b'',capture.deadline+1,'pressure'))
    result = router.finish_scientific_pressure_capture(capture)
    assert result['lost_frame_count'] == 1
    assert result['retained_frame_count'] == 0
    assert result['channels'][0]['baseline_statistics']['mean'] is None


def test_real_reaspiration_and_unknown_accounting_receipts(rig):
    result = run(rig, request(stroke('trailing_air', 1), stroke('reaspirate', .5), stroke('aspirate', 22.5)))
    assert result['ok'], result
    assert (0, 'P0.5,1R') in rig.wire
    assert rig.group._transports[0]._rear_air_level_ul == 1.5
    row = result['events'][-1]['result']['channels'][0]['result']
    assert row['accounting']['intended_liquid_delta_ul'] is None
    assert not row['accounting']['initial_fill_known']
    reopened = PipetteReceiptStore(rig.root)
    rows = [json.loads(row[0]) for row in reopened.connection.execute('SELECT receipt_json FROM pipette_operations')]
    assert rows
    export('effect-receipts.json', {'application': result, 'receipts': rows})


def test_intent_and_pneumatics_remain_distinct(rig):
    # Actual compiler/emitter, with the cross-owner forwarding seam tested explicitly.
    from bioxp.pipette.cavro_application import compile_application
    payload = request({**stroke('aspirate', 22.5), 'target_liquid_ul': 20})
    compiled = compile_application(payload)
    assert not compiled['issues']
    operation = compiled['operations'][0]
    from bioxp.pipette.cavro_application import _send_wire
    out = _send_wire(rig.group, [0], operation['wire_commands'][-1]['ascii'], 20,
                     effect='aspirate', inputs={'volume_ul': operation['volume_ul'],
                                               'target_liquid_ul': operation['target_liquid_ul']})
    assert out['ok']
    account = out['channels'][0]['result']['accounting']
    assert account['pneumatic_displacement_delta_ul'] == 22.5
    assert account['intended_liquid_delta_ul'] == 20
    assert account['measured_liquid_ul'] is None


def test_compaction_failure_detail_survives():
    value = {'failed_channel': 2, 'failure_details': {'specific_fault': 'fixture'},
             'completion_wait_started': False, 'source_timeout_termination': {'allow_to_stop': True, 'channels': []}}
    assert application_outcome(value) == value
    from bioxp.protocols.operational_results import compact_operational
    assert compact_operational(value) == value


def test_stream_no_implicit_nvram_and_psi(receiver):
    driver = receiver.group._transports[0]._get_driver()
    driver.enable_pressure_stream(True)
    assert all('b15' not in wire for _, wire in receiver.wire)
    pressure = driver.query_pressure()
    assert pressure['pressure'] == 1.25 and pressure['pressure_units'] == 'psi'


def test_bounded_pressure_real_receiver_sqlite(rig):
    run(rig, request(stroke('aspirate', 1)))
    operation = rig.store.connection.execute('SELECT command_id,pipette_operation_id FROM pipette_operations LIMIT 1').fetchone()
    capture = rig.router.begin_scientific_pressure_capture({'channels':[0], 'max_frames':2, 'duration_ms':1000, 'sample_period_ms':2.0})
    for values in ([1,2,3], [-2,4], [999]):
        data = bytes([48+len(values)]) + b''.join(v.to_bytes(2,'big',signed=True) for v in values)
        rig.router._dispatch(NovoFrame(min(PRESSURE_STREAM_IDS),len(data),data,b'',time.monotonic(),'pressure'))
    result = rig.router.finish_scientific_pressure_capture(capture)
    assert result['lost_frame_count'] == 1 and result['lost_sample_count'] is None
    assert result['channels'][0]['baseline_statistics']['count'] == 5
    assert result['anomaly_model'] is None and result['classification'] is None
    identity = rig.store.retain_scientific_pressure_capture(result, command_id=operation['command_id'], pipette_operation_id=operation['pipette_operation_id'])
    reopened = PipetteReceiptStore(rig.root)
    readback = reopened.scientific_pressure_capture(identity['stream_session_id'])
    assert readback['stream']['terminal_state'] == 'completed'
    retained = readback['chunks'][0]['scientific_result']
    assert [r['raw_pressure'] for r in retained['samples']] == [1,2,3,-2,4]
    assert retained['scientific_result']['anomaly_model'] is None
    export('pressure-sqlite-api-ready.json', readback)


def test_capture_finite_receipt_serialization(rig):
    from bioxp.pipette.pressure_capture import PressureCaptureRequest
    export('pressure-capture-request.schema.json', PressureCaptureRequest.model_json_schema())
    capture_request = {'channels': [0], 'max_frames': 2, 'duration_ms': 20}
    def inject(_):
        data = b'1\x00\x08'
        rig.router._dispatch(NovoFrame(min(PRESSURE_STREAM_IDS), len(data), data, b'', time.monotonic(), 'pressure'))
    rig.group._sleep = inject
    result = rig.provider._manual_pipette_receipt_runner('pressure_capture',
        lambda group: group.capture_pressure(capture_request), 'offline-finite-owner',
        {'source_identity':'scientific-capture-test'}, capture_request)
    assert result['ok'] and not result['receipt_truth']['completion_verified']
    identity = result['scientific_pressure']['stream_session_id']
    row = rig.store.connection.execute('SELECT receipt_json FROM pipette_operations WHERE command_id=?', (result['command_id'],)).fetchone()
    receipt = json.loads(row[0])
    assert receipt['result']['scientific_pressure']['stream_session_id'] == identity
    assert 'samples' not in receipt['result']['channels'][0]
    reread = PipetteReceiptStore(rig.root).scientific_pressure_capture(identity)
    assert reread['chunks'][0]['scientific_result']['samples'][0]['raw_pressure'] == 8
    export('pressure-finite-receipt.json', {'result':result, 'receipt':receipt, 'readback':reread})


def export(name, value):
    if root := os.environ.get('NON_OEM_NATIVE_EXPORT_DIR'):
        Path(root).mkdir(parents=True, exist_ok=True)
        Path(root, name).write_text(json.dumps(value, indent=2))
