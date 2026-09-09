"""CI.HomeAxis(z): preserve axisSearchHome scalar, not reference proof.
Source: BioXPControlLib IL HomeAxis; Z branch sets current then search597.
The provider owns the board-test wrapper, not a new Home transaction.
"""
from types import SimpleNamespace
import pytest
from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter


@pytest.mark.parametrize('scalar', [-123, 0, 10000])
def test_diagnostic_home_axis_preserves_child_scalar_without_fabricated_evidence(scalar):
    adapter = object.__new__(Serial206ProductionPrimitiveAdapter)
    calls = []
    def home(axis, **kwargs):
        calls.append((axis, kwargs))
        return {'ok':True, 'home': {'ok':True, 'source_return_code':scalar,
                'go_home': {'ok':True, 'source_return_code':scalar,
                            'controller_command_acknowledged':False,
                            'controller_terminal_state_verified':False}}}
    adapter.tester = SimpleNamespace(motor_oem_home_axis_board_test=home)
    row = adapter.z_diagnostic_home_axis(timeout_s=30)
    assert calls == [('z', {'timeout_s':30.0})]
    assert row['ok'] is True
    assert row['intent'] == 'diagnostic_home_axis_597'
    assert row['home']['home']['source_return_code'] == scalar
    assert row['controller_command_acknowledged'] is False
    assert row['controller_terminal_state_verified'] is False
    assert row['physical_effect_verified'] is False


def test_diagnostic_home_axis_propagates_source_exception():
    adapter = object.__new__(Serial206ProductionPrimitiveAdapter)
    def home(*a, **k):
        raise RuntimeError('source search failed')
    adapter.tester = SimpleNamespace(motor_oem_home_axis_board_test=home)
    with pytest.raises(RuntimeError, match='source search failed'):
        adapter.z_diagnostic_home_axis()
