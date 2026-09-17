"""API wake adapter -> real lifecycle body and native generation owner.

Board exchanges are leaf doubles; this is not complete native wake acceptance.
"""
from types import SimpleNamespace
import pytest
from bioxp import api
from bioxp.usb_driver import BioXpTester
from tests.test_workflow_wake_owner import owner


@pytest.mark.parametrize('bad_board', [None, 4, 5, 6, 7])
def test_wake_api_passes_raw_cycle_acks_to_native_generation(monkeypatch, bad_board):
    tester = object.__new__(BioXpTester)
    tester._oem_transport_generation = 3
    tester._oem_board_lifecycle_generation = 9
    tester._oem_active_board_lifecycle_generation = None
    calls = []
    tester.strip_set_rgb = lambda *a, **k: {'ok': True}
    tester.query_only_tmcl = lambda board, command, typ, bank, value: {
        'status': 100, 'value': 0 if typ == 0 else 1}
    tester.deck_io_set_type = lambda *a: {'ok': True}
    def cycle(active, **kwargs):
        calls.append(active)
        return {b: {'status': 1 if active and b == bad_board else 100}
                for b in tester.BOARDS}
    tester.deactivate_boards = lambda **k: cycle(False, **k)
    tester.activate_boards = lambda **k: cycle(True, **k)
    monkeypatch.setattr(api, '_get_tester', lambda: tester)
    monkeypatch.setattr(api, '_serial206_oem_initialization_provider',
                        SimpleNamespace(sleep=lambda seconds: None))
    monkeypatch.setattr(api, '_can_ready_observation', lambda: True)
    monkeypatch.setattr(api, 'lifecycle_state', owner())
    phases = []
    result = api._protocol_workflow_initial_check(None,
        validate_current=lambda phase: phases.append(phase) or True)
    assert calls == [False, True]
    if bad_board is None:
        assert result['ok'] is True and result['source_return'] is True
        assert tester._oem_active_board_lifecycle_generation == 10
        assert phases[-2:] == ['oem_begin_board_lifecycle_generation', 'completion']
    else:
        assert result['ok'] is False
        assert tester._oem_active_board_lifecycle_generation is None
        assert tester._oem_board_lifecycle_generation == 9
