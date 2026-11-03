"""Offline pickup effects -> real provider -> canonical SQLite boundary.

Only physical/nested execution leaves are replaced; inventory publication uses
its production provider and store (including supported transition shapes).
"""
from types import SimpleNamespace
import pytest
from tests.test_debloat_core import named  # noqa: F401
from tests.test_wake_setup_debloat import rig  # noqa: F401
from bioxp.pipette.models import PipetteCommandError


def observed(ch, valid=True):
    return {"channel": ch, "tip_loaded": True, "result": {
        "ok": valid, "semantic_ok": valid, "hardware_truth_level": "hardware_query"}}


def sent(ch, complete=True):
    return {"channel": ch, "result": {"ok": True, "completion_verified": complete,
        "delivery_verified": True, "controller_acknowledged": True}}


@pytest.mark.parametrize('failure,remaining,released', [
    ('none', [2], [0, 1, 3]),
    ('home_false', [2], [0, 1, 3]),
    ('home_throw', [2], [0, 1, 3]),
    ('dirty', [2], [0, 1, 3]),
    ('release_receipt', [2], [0, 1, 3]),
    ('query_receipt', [0, 1, 2, 3], []),
    ('release_partial', [1, 2, 3], [0]),
    ('query_partial', [0, 2], []),
    ('lower', [], []),
    ('metadata', [2], [0, 1, 3]),
    ('accounting', [0, 1, 2, 3], [0, 1, 3]),
])
def test_provider_partial_effects_survive_tail_failures(named, monkeypatch, failure, remaining, released):
    _, provider, store, _, frames, _ = named
    provider.publish_tip_tray_transition(tray_id=0, transition='load_all', operation_id='seed',
        command_id='seed', provenance={})
    calls = []
    channels = [SimpleNamespace(_tip_loaded=True) for _ in range(4)]

    def query():
        calls.append('query')
        if failure == 'query_partial':
            raise PipetteCommandError('query interrupted', details={
                'observed_channels': [observed(0), observed(1, False), observed(2)]})
        return {'ok': True, 'channels': [observed(c) for c in range(4)]}

    def eject(drop, *, wait_for_completion):
        assert drop == [0, 1, 3] and wait_for_completion
        calls.append('eject')
        if failure == 'release_partial':
            raise PipetteCommandError('second release unknown', details={
                'sends': [sent(0), sent(1, False)], 'channel': 1})
        return [sent(c) for c in drop]

    transport = SimpleNamespace(_transports=channels, query_tip_status_all=query,
        _eject_tip_channels_once=eject, loadTip=lambda value, loc: {'ok': True})

    def runner(name, call, *args):
        calls.append(name)
        if name == 'load_tip_metadata' and failure == 'metadata':
            raise RuntimeError('metadata publication failed')
        value = call(transport)
        if (name == 'release_unneeded_tips' and failure == 'release_receipt') or (
                name == 'query_tip_status_all' and failure == 'query_receipt'):
            raise RuntimeError('receipt publication failed after physical return')
        return value

    def nested(*, plan, **kwargs):
        # Keep the real compiler in wp8_manual_pipette_physical.
        name = plan['operation']
        calls.append(name)
        if name == 'pipette_home' and failure.startswith('home'):
            if failure == 'home_throw':
                raise RuntimeError('home interrupted')
            return {'ok': False}
        if name == 'pipette_tip_state' and failure == 'dirty':
            raise RuntimeError('semantic publication failed')
        return {'ok': True}

    def lower(**kwargs):
        calls.append('lower')
        if failure == 'lower':
            return {'ok': False}
        return {'ok': True, 'target_position_steps': 141531}

    monkeypatch.setattr(provider, '_manual_pipette_receipt_runner', runner, raising=False)
    monkeypatch.setattr(provider, '_wp8_execution_fence_checker', lambda *a, **k: None, raising=False)
    monkeypatch.setattr(provider, '_wp8_execute_nested_plan', nested)
    monkeypatch.setattr(provider.primitives, 'z_pipette_position', lower)
    if failure == 'accounting':
        publisher = provider._tip_tray_state_publisher
        def fail_restore(**kwargs):
            if kwargs['transition'] == 'retip':
                raise RuntimeError('inventory publication unavailable')
            return publisher(**kwargs)
        monkeypatch.setattr(provider, '_tip_tray_state_publisher', fail_restore)
    frames.clear()
    result = provider.wp8_manual_pipette_physical('sourceManualLoadTip',
        {'tray': 1, 'well': 'A10', 'keep_channels': [2], 'lift_z': True},
        command_id='partial-pickup', child_order=0, plan_digest='test',
        owner_identity={'source_identity': 'partial-pickup:0'})
    assert result['ok'] is (failure in ('none', 'accounting')), result
    wells = [9, 33, 57, 81]
    state = store.tip_tray_state(0)
    assert [ch for ch, well in enumerate(wells) if not state['occupancy'][well]] == remaining, result
    assert sum(state['occupancy']) == 96 - len(remaining)
    if failure != 'lower':
        assert result['released_channels'] == released
        errors = [r for r in result['pickup_accounting'] if 'error' in r['result']]
        assert bool(errors) is (failure == 'accounting'), result
        if failure == 'release_partial':
            assert result['release_unverified_channels'] == [1, 3]
        elif released:
            assert result['release_unverified_channels'] == []
    if 'eject' in calls:
        assert calls.index('lower') < calls.index('query') < calls.index('pipette_move_z') < calls.index('eject')
    assert calls.count('eject') <= 1 and calls.count('lower') == 1
    if failure in ('release_partial', 'release_receipt', 'query_receipt', 'query_partial', 'lower'):
        assert 'pipette_home' not in calls
    assert frames == []
    # Independent canonical read, not just the returned effect dictionary.
    from bioxp.operator_command_plane import OperatorCommandStore
    reopened = OperatorCommandStore(store.root)
    try:
        assert reopened.tip_tray_state(0)['occupancy'] == state['occupancy']
    finally:
        reopened.stop()


@pytest.mark.parametrize('channels', [[0, 2], [0, 1, 3]])
def test_two_and_three_channel_bookkeeping_uses_supported_store_transitions(named, channels):
    _, provider, store, _, frames, _ = named
    provider.publish_tip_tray_transition(tray_id=0, transition='load_all', operation_id='seed',
        command_id='seed', provenance={})
    calls = []
    def finite(name, inputs):
        calls.append(inputs)
        return provider.publish_tip_tray_transition(**inputs, operation_id=f'partial:{len(calls)}',
            command_id='partial', provenance={})
    frames.clear()
    result = provider._manual_pickup_bookkeeping({'tray': 1, 'well': 'A10'}, finite,
        None, channels=channels)
    assert result['rack_tips_removed'] == [9 + 24 * ch for ch in channels]
    assert len(calls) == len(channels)
    assert all(len(call['well_ids']) == 1 for call in calls)
    assert sum(store.tip_tray_state(0)['occupancy']) == 96 - len(channels)
    assert frames == []
