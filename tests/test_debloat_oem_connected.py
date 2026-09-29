"""XML -> native factory -> canonical worker/SQLite; transport replaced only.

The inherited fixture retains real lifecycle/provider/receipt implementations.
No test calls a robot or claims mechanical/thermal/optical acceptance.
"""
import pytest
from tests.protocol_v1_integration_fixture import integrated_rig
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig
from tests.test_debloat_oem_xml import ROOT, SETTINGS, actions, generated
from bioxp.protocols.oem_xml_expand import expand_oem_xml_protocol


@pytest.fixture(autouse=True)
def xml_native_wiring(integrated_rig, monkeypatch):
    from bioxp import api
    from bioxp.protocols.executor import oem_source_default_handlers
    original = api._protocol_bindings
    def bindings(*args, **kwargs):
        result = original(*args, **kwargs)
        result[1].update(oem_source_default_handlers())
        return result
    # Exact parent-owned API integration hook, no hardware body replacement.
    monkeypatch.setattr(api, '_protocol_bindings', bindings)
    for leaf in integrated_rig.transport._transports:
        driver = leaf._get_driver()
        driver.response_timeout_s = 60.0
        driver._pipette_completion_owner_token = None
        driver._sleep = lambda _: None
        original_exchange = driver._send_pipette_command
        def exchange(command, *, command_name, driver=driver, previous=original_exchange, **kwargs):
            if command == '?31':
                return previous(command, command_name=command_name, **kwargs)
            assert command in ('WR', 'b15R', 'o0,1R', 'o0,0R', '&1', 'Q1'), command
            driver._pipette_last_command = command_name
            data = [32,96,32] if command == 'Q1' else [32,96,49] if command == '&1' else []
            driver.process_pipette_message(len(data), data, command_name=command_name)
            return {'ok': True, 'tx_ok': True, 'delivery_verified': True,
                    'controller_acknowledged': True, 'ack': {'received': True, 'outcome': 'ack', 'data': data}}
        def packet(board, data, *, command_name, channel=driver.pipette_id):
            assert board == 128 and data == [32 | channel] * 2
            return {'ok': True, 'tx_ok': True, 'ack': {'received': True}}
        def completion(channel, timeout, **kwargs):
            return {'ok': True, 'outcome': 'completion', 'data': [32,96],
                    'observed_rx_dlc': 2, 'observed_rx_id': 0x501 + 8 * channel,
                    'command_name': 'pipette_initialize'}
        monkeypatch.setattr(driver, '_send_pipette_command', exchange)
        monkeypatch.setattr(driver, '_send_packet', packet)
        driver.bus.wait_pipette_completion = completion


def payload(rig, doc, key):
    request = rig.payload(key)
    # Keep the fixture's explicitly synthetic prepared model, not live inventory.
    request['document'] = doc.to_payload()
    request['document']['metadata']['source_model'] = rig.payload(key)['document']['metadata']['source_model']
    return request


def test_retained_system_check_preserves_failed_native_child(integrated_rig):
    rig = integrated_rig
    # The source chiller setter throws on transport error. Earlier LEDs/waits
    # remain recorded; the first plate press must not execute after that throw.
    rig.native.replies[(7, 140, 0, 0)] = {'status': 2, 'value': 0}
    doc = expand_oem_xml_protocol(ROOT / 'scripts/Inital Test Script.xml', source_settings=SETTINGS)
    done = rig.terminal(rig.start(payload(rig, doc, 'xml-native-failure')))
    assert done['command']['status'] != 'completed', done
    runtime = done['execution']['runtime_state']
    source_rows = [r for r in runtime['action_results'] if r.get('source_occurrence_id')]
    assert any(r.get('ok') is False for r in source_rows)
    assert not rig.native_results(done, 'press_plates')
    assert rig.child_rows(done)


def test_source_defaults_have_real_parent_and_no_native_children(integrated_rig, tmp_path):
    rig = integrated_rig
    doc = generated(tmp_path, ['SS','RT ignored','ST ignored','SW ignored','TT ignored','ZW ignored'])
    done = rig.terminal(rig.start(payload(rig, doc, 'xml-defaults')))
    assert done['command']['status'] == 'completed', done
    rows = [r for r in done['execution']['runtime_state']['action_results'] if r.get('source_noop') == 'ControlLib.scriptInterpretor.default']
    assert len(rows) == 6
    assert [r['source_return'] for r in rows] == [[20],[30],[40],[50],[60],[70]]
    children = rig.child_rows(done)
    assert not any('cutseal' in str(row) for row in children)


def test_dwell_source_sequence_through_real_thermal_waiters(integrated_rig, tmp_path):
    rig = integrated_rig
    # Native physical recorder reports 25C; real timers consume these targets.
    doc = generated(tmp_path, ['LOOP 2','DWELL 1','SP T25 DUR0 R2','SP T25 DUR0 R2','LOOP'])
    from tests.test_protocol_workflow_connected import await_job
    job = rig.start(payload(rig, doc, 'xml-dwell'))
    done = await_job(rig.client, job['job_id'], lambda row: row['command']['terminal'], timeout=60)
    assert done['command']['status'] == 'completed', done
    receipts = rig.native_results(done, 'set_tc_temperature')
    assert len(receipts) == 4
    assert all(r['response']['source_wait_satisfied'] for r in receipts)
    assert rig.native.thermal_wait.is_set()
