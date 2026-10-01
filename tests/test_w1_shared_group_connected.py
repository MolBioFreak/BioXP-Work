"""W1 shared nonconstructor group owners preserve full baseline detail."""
import copy

import pytest
from bioxp import api
from bioxp.pipette import transport as tm
from tests.test_deck_tip_query_publication import query_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_scoped_authority import retained_rig
from tests.test_w1_diagnostics_connected import baseline_module, transport_rig, stable, artifact


@pytest.mark.parametrize('caller', ['initializeMotion.initial', 'initializeMotion.retry', 'detectFluid.initiateGroup'])
@pytest.mark.parametrize('mode', ['initial', 'completion_failure'])
def test_shared_group_source_detail_is_not_constructor_pruned(query_rig, monkeypatch, caller, mode):
    baseline = baseline_module('src/bioxp/pipette/transport.py', tm)
    runs = []
    for label, module in [('baseline', baseline), ('candidate', tm)]:
        group, wire, timers = transport_rig(module, mode)
        monkeypatch.setattr(api, '_pipette_transport', group)
        monkeypatch.setattr(api, '_get_pipette_transport', lambda: group)
        # Real shared source method, real CAN driver and NovoRouter event waits.
        # Only the controller exchanges in transport_rig are synthetic.
        result = (group.initiate_group_once_for_oem_detect_fluid()
            if caller == 'detectFluid.initiateGroup' else
            group.initiate_group_once_for_oem_initialize_motion(cycle=caller))
        for timer in timers: timer.join()
        status = group.get_status()
        assert result['cycle'] == caller
        assert result['ok'] is (mode == 'initial')
        assert result['outcome'] == ('completion' if mode == 'initial' else 'group_completion_timeout_or_error')
        assert result['single_group_cycle'] is True
        if caller != 'detectFluid.initiateGroup':
            assert result['retry_selected_by_transport'] is False
        assert [r['channel'] for r in result['sends']] == [0, 1, 2, 3]
        assert [r['channel'] for r in result['delayed_completions']] == [0, 1, 2, 3]
        assert all(r['result']['driver_result']['immediate_ack_received'] is True for r in result['sends'])
        assert all('last_transaction' in r['result'] for r in result['sends'])
        terminal = [r['result']['ok'] for r in result['delayed_completions']]
        assert terminal == ([True] * 4 if mode == 'initial' else [True, True, True, False])
        if mode == 'completion_failure':
            assert result['delayed_completions'][3]['result']['event_error_code'] == 0x21
        pressure_expected = mode == 'initial' or caller == 'detectFluid.initiateGroup'
        if pressure_expected:
            assert result['pressure_stream']['selected_channels'] == [0, 1, 2, 3]
            assert result['pressure_stream']['wait_ms'] == 1000
            assert result['pressure_stream']['terminal_state'] == 'stopped'
            assert result['pressure_offsets'] == {ch: 100 + ch for ch in range(4)}
            assert result['pressure_offsets_valid'] is True
            assert result['completion_verified'] is (mode == 'initial')
            assert all(result['pressure_offset_evidence'][ch]['valid'] is True for ch in range(4))
        else:
            # initializeMotion's source failure returns before pressure; diagnostic
            # detectFluid's void initiateGroup continues even after wait false.
            assert 'pressure_stream' not in result and 'pressure_offset_evidence' not in result
            assert not any(command == 'o0,1R' for _, command in wire)
        artifact('shared-' + caller + '-' + mode + '-' + label + '.json',
            {'result': result, 'status': status, 'wire': wire,
             'provenance': 'controlled controller exchange; actual nonconstructor group/driver/router'})
        runs.append((stable(copy.deepcopy(result)), stable(copy.deepcopy(status)), list(wire)))
    # Whole detail-tree equality includes channel transactions, ACK/completion,
    # pressure evidence/attachments and last_transaction shape, not just ok.
    assert runs[0] == runs[1]
    assert query_rig[6] == []  # No hidden TipExist query or constructor traffic.
