"""Single real hardware projection feeds the operator machine state unchanged."""
from types import SimpleNamespace
import pytest
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained


@pytest.mark.parametrize('scenario', ['missing', 'fresh', 'stale', 'owner_changed'])
def test_machine_state_preserves_single_projection(installed_retained, monkeypatch, scenario):
    from bioxp import hardware_status, operator_controls
    app, provider, primitive, references, root = installed_retained
    clock = [2_000_000_000.0]
    monkeypatch.setattr(hardware_status, 'time', SimpleNamespace(time=lambda: clock[0]))
    owner = hardware_status.HardwareStateOwner()
    monkeypatch.setattr(operator_controls, 'hardware_state', owner)
    if scenario != 'missing':
        # Physical-reader doubles feed the real collector; freshness is about
        # observed domains, not invented motor/reference readiness.
        collectors = {name: (lambda context, name=name: {'fixture_domain': name})
                      for name in hardware_status.CANONICAL_DOMAINS}
        owner.collect(hardware_status.CANONICAL_DOMAINS, collectors)
    if scenario == 'stale':
        clock[0] += 60
    if scenario == 'owner_changed':
        owner.change_ownership(reason='isolated-owner-replacement', transport='replacement')
    seen = []
    project = owner.project
    def observe(*args, **kwargs):
        result = project(*args, **kwargs)
        seen.append(result)
        return result
    monkeypatch.setattr(owner, 'project', observe)
    calls = list(primitive.calls)
    state = app.state.operator_command_plane.machine_state_provider()
    assert len(seen) == 1
    expected = seen[0]
    assert state['snapshot_id'] == expected['snapshot_id']
    assert state['freshness'] == expected['freshness']
    assert state['freshness']['state'] == {
        'missing': 'missing', 'fresh': 'fresh', 'stale': 'stale', 'owner_changed': 'missing'
    }[scenario]
    current = owner.ownership_projection()
    assert state['ownership_generation'] == current['ownership_epoch']
    assert state['ownership'] == current['ownership']
    for name, row in expected['domains'].items():
        if name != 'pipette':  # This independent passive status has its own owner.
            assert state['domains'][name] == row
    assert primitive.calls == calls
