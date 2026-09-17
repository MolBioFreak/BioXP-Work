"""Whole-factory target completion, plus explicitly separate recorder checks."""
import pytest
from tests.protocol_v1_integration_fixture import integrated_rig, NativePhysicalRecorder
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig
from tests.test_protocol_v1_integrated_lifecycle import assert_reopened


@pytest.mark.parametrize('integrated_rig', [{'cold_transport': True}], indirect=True)
def test_cold_first_job_query_retains_reader_authority(integrated_rig):
    rig = integrated_rig
    assert all(t._driver is None for t in rig.transport._transports)
    job = rig.start(rig.payload('cold-first-query'))
    done = rig.terminal(job)
    prefix = next(r for r in done['execution']['runtime_state']['action_results']
                  if r.get('hook') == 'run_job')
    assert prefix['ok'] is True, prefix
    assert done['command']['status'] == 'completed'
    assert_reopened(rig, done)


@pytest.mark.parametrize('opcode,arguments,operation', [
    ('sp', ['25', '1', '2.5'], 'set_tc_temperature'),
    ('splid', ['25', '1', '-20'], 'set_lid_temperature'),
    ('cc', ['OC', '25'], 'set_chiller_temperature'),
])
def test_target_complete_through_whole_factory(integrated_rig, opcode, arguments, operation):
    rig = integrated_rig
    job = rig.start(rig.payload('native-complete-' + opcode, opcode=opcode, arguments=arguments))
    done = rig.terminal(job)
    assert done['command']['status'] == 'completed'
    receipts = rig.native_results(done, operation)
    assert len(receipts) == 1 and receipts[0]['status'] == 'completed', receipts
    raw = receipts[0]['response']
    assert raw['ok'] is True and raw['source_body_returned'] is True, raw
    if opcode != 'cc':
        assert rig.native.thermal_wait.is_set(), 'native timer must actually run'
        assert raw['source_wait_satisfied'] is True, raw
        assert rig.native.tester._oem_thermal_board_timer_enabled is False
    else:
        # Native chiller target has immediate source return, not a wait ticket.
        assert (7, 140, 0, 1, 25000) in rig.native.trace
    assert_reopened(rig, done)


@pytest.mark.parametrize('method,arguments', [
    ('oem_set_tc_temperature', (25, 1, 2.5)),
    ('oem_set_lid_temperature', (25, 1, -20, True)),
    ('oem_chiller_set_temperature', (1, 25)),
])
def test_physical_recorder_native_source_contract(monkeypatch, method, arguments):
    """Fixture-only qualification; these are NOT integrated acceptance counts."""
    recorder = NativePhysicalRecorder(monkeypatch)
    try:
        result = getattr(recorder.tester, method)(*arguments)
        assert result['ok'] is True and result['source_body_returned'] is True, result
        assert recorder.trace
        if method != 'oem_chiller_set_temperature':
            assert recorder.thermal_wait.is_set()
            assert result['source_wait_satisfied'] is True
            assert recorder.tester._oem_thermal_board_timer_enabled is False
        with pytest.raises(AssertionError, match='unrecorded native physical command'):
            recorder.exchange(99, 999, 999, 999, 0)
    finally:
        recorder.close()
