"""Host-only preparation boundary tests; native work is explicitly doubled."""
import copy
import pytest
from bioxp.protocols.runtime_state import SourceTray, SourceWell
from tests.test_protocol_oem_lifecycle import document, engine, start, finish


def fixture_trays(count=4, wells=96):
    return [SourceTray(tray_id=i, location=7+i,
                       wells=[SourceWell(content='emptytip', volume=0.0, capacity=50.0,
                                         empty=(j % 5 == 0)) for j in range(wells)])
            for i in range(count)]


def test_prepare_produces_model_before_runjob_without_rewriting_input():
    doc = document('step', oem_prepare=True)
    original = copy.deepcopy(doc.to_payload())
    produced = fixture_trays()
    observed = []
    def prepare(state):
        assert state.source_model.tip_trays == []
        state.source_model.tip_trays = produced
        observed.append('prepare')
        return {'ok': True, 'offline_native_double': 'prepare'}
    def runjob(state):
        assert state.source_model.tip_trays is produced
        observed.append('run_job')
        return {'ok': True, 'offline_native_double': 'run_job'}
    executor, trace = engine(doc, overrides={'prepare': prepare, 'run_job': runjob})
    state = finish(start(executor, doc))
    assert state.completed and executor.outcome == 'completed'
    assert observed == ['prepare', 'run_job']
    assert doc.to_payload() == original
    assert [well.empty for well in state.source_model.tip_trays[0].wells] == [j % 5 == 0 for j in range(96)]


@pytest.mark.parametrize('count,wells', [(0, 96), (3, 96), (4, 95)])
def test_prepare_success_without_required_output_cannot_enter_runjob(count, wells):
    doc = document('step', oem_prepare=True)
    def prepare(state):
        state.source_model.tip_trays = fixture_trays(count, wells)
        return {'ok': True, 'offline_native_double': 'incomplete_prepare'}
    executor, trace = engine(doc, overrides={'prepare': prepare})
    state = finish(start(executor, doc))
    assert executor.outcome == 'failed' and not state.completed
    assert 'run_job' not in trace and 'script_prologue' not in trace
    assert 'epilogue_sweep' not in trace and 'cleanup' not in trace
    assert any(event.event == 'preparation_inputs_missing' for event in state.events)


def test_failed_prepare_never_enters_body_even_with_complete_inventory():
    doc = document('step', oem_prepare=True)
    def prepare(state):
        state.source_model.tip_trays = fixture_trays()
        return {'ok': False, 'error': 'unit_inspection_failure'}
    executor, trace = engine(doc, overrides={'prepare': prepare})
    state = finish(start(executor, doc))
    assert not state.completed
    assert 'run_job' not in trace and 'script_prologue' not in trace
