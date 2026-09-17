"""Offline owner-level readiness acquisition and publication regression tests."""
import copy
import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig, rig, qualify_test_references
from tests.test_deck_scoped_integration import installed_retained, qualify_full_predecessor


def test_subset_preserves_original_rows_and_expiry(monkeypatch):
    from bioxp import hardware_status as module
    clock = [100.0]
    monkeypatch.setattr(module.time, 'time', lambda: clock[0])
    owner = module.HardwareStateOwner(fresh_for_s=30)
    domains = ['transport', 'boards', 'axes', 'power', 'interlock', 'latch', 'gripper', 'thermal', 'chiller']
    full = owner.collect(domains, {d: lambda c: {'source': 'full'} for d in domains})['snapshot']
    clock[0] = 120.0
    subset = owner.collect(['axes', 'latch'], {d: lambda c: {'source': 'subset'} for d in ['axes', 'latch']})['snapshot']
    assert subset['requested_domains'] == ['axes', 'latch']
    for domain in set(domains) - {'axes', 'latch'}:
        assert subset['domains'][domain] == full['domains'][domain]
    assert owner.project(*domains)['cache_state'] == 'fresh'
    clock[0] = 131.0
    assert owner.project('power')['cache_state'] == 'stale'
    assert owner.project('axes')['cache_state'] == 'fresh'


def test_failed_subset_replaces_success_without_dropping_other_domains():
    from bioxp.hardware_status import HardwareStateOwner
    owner = HardwareStateOwner()
    owner.collect(['power', 'latch'], {'power': lambda c: 0, 'latch': lambda c: True})
    def failed(c):
        raise RuntimeError('query failed')
    result = owner.collect(['latch'], {'latch': failed})['snapshot']['domains']
    assert result['power']['observation'] == 0
    assert result['latch']['status'] == 'error' and result['latch']['observation'] is None


@pytest.mark.parametrize('changed', ['power', 'axes', 'ownership', 'all'])
def test_subset_does_not_resurrect_invalidated_domains(changed):
    from bioxp.hardware_status import HardwareStateOwner
    owner = HardwareStateOwner()
    owner.collect(['power', 'axes'], {'power': lambda c: 0, 'axes': lambda c: 1})
    def collect(c):
        if changed == 'ownership':
            owner.change_ownership(reason='offline generation')
        elif changed == 'all':
            owner.invalidate(reason='offline invalidate')
        else:
            owner.invalidate_domains(changed, reason='offline revision')
        return 2
    result = owner.collect(['axes'], {'axes': collect})
    if changed == 'ownership':
        assert result['published'] is False and owner.completed_snapshot() is None
    else:
        rows = result['snapshot']['domains']
        assert ('power' in rows) == (changed == 'axes')
        assert ('axes' in rows) == (changed != 'axes')


@pytest.mark.parametrize('domain,stop_after', [('axes', n) for n in (1, 7, 8, 9)] + [('latch', n) for n in (1, 2, 3)])
def test_yield_between_individual_axis_and_io_queries(monkeypatch, domain, stop_after):
    from bioxp import api
    from bioxp.hardware_status import HardwareStateOwner, HardwareCollectionPreempted
    calls = []
    tester = SimpleNamespace(BOARD_DECK=4, MOTOR_SWITCH_ACTIVE_VALUE=1,
        query_only_tmcl=lambda *args: calls.append(args) or {'value': 0, 'status': 100})
    monkeypatch.setattr(api, '_axis_preset', lambda tester, axis: {'board': 4, 'motor': 0})
    owner = HardwareStateOwner()
    before = owner.collect([domain], {domain: lambda c: 'original'})['snapshot']
    def check():
        if len(calls) >= stop_after:
            raise HardwareCollectionPreempted('operator_action_pending')
    result = owner.collect([domain], api._hardware_collectors(tester, before_query=check))
    assert result == {'ok': False, 'published': False, 'reason': 'operator_action_pending'}
    assert len(calls) == stop_after
    assert owner.completed_snapshot() == before


@pytest.mark.parametrize('automatic', [False, True])
@pytest.mark.parametrize('freshness', ['fresh', 'stale', 'missing'])
def test_default_readiness_omits_only_auxiliary_diagnostics(installed_retained, monkeypatch, automatic, freshness):
    from bioxp import api
    app, _, _, _, _ = installed_retained
    calls = []
    monkeypatch.setattr(api.hardware_state, 'project', lambda *domains, **kwargs: {'cache_state': freshness})
    monkeypatch.setattr(api, '_collect_and_publish_hardware_snapshot', lambda requested, **kw: calls.append(requested) or {'ok': True})
    result = TestClient(app).post('/hardware/snapshot/collect', json={'automatic': automatic})
    assert result.status_code == 200
    expected = set(api.DEFAULT_HARDWARE_SNAPSHOT_DOMAINS) - ({'thermal', 'chiller'} if automatic and freshness == 'fresh' else set())
    assert set(calls[0]) == expected
    assert {'transport', 'boards', 'power', 'interlock', 'axes', 'latch', 'gripper', 'pipette'} <= expected


def test_batch_shares_only_current_physical_observations(retained_rig, monkeypatch):
    provider, primitive, _, _, _, _ = retained_rig
    qualify_full_predecessor(retained_rig)
    primitive.calls.clear()
    gripper_calls = []
    original = provider._deck_gripper_confirmed
    monkeypatch.setattr(provider, '_deck_gripper_confirmed', lambda: gripper_calls.append(True) or original())
    with provider.movement_lease():
        snapshots = provider.deck_authority_snapshots(expected_generation=3)
    ordinary, park = snapshots['LOC_MS'], snapshots['LOC_PARK']
    assert isinstance(ordinary, dict) and isinstance(park, dict), snapshots
    assert ordinary['dependency_scope'] == 'offset.v1' and park['dependency_scope'] == 'full'
    assert ordinary['captured_at'] == park['captured_at']
    assert provider._deck_authority_scoped_cache['offset.v1'][:2] == provider._deck_authority_scoped_cache['park.full'][:2]
    assert len(gripper_calls) == 1
    assert sum(c[0] == 'latch' for c in primitive.calls) == 1
    assert sum(c[0] == 'xyz' for c in primitive.calls) == 3, primitive.calls
    for axis in 'xyz':
        assert ordinary['current_' + axis] == park['current_' + axis]
    if os.environ.get('DECK_TEST_OUTPUT'):
        Path(os.environ['DECK_TEST_OUTPUT'] + '.plural.json').write_text(json.dumps({
            'snapshots': snapshots, 'physical_leaf_calls': primitive.calls,
            'gripper_acquisitions': len(gripper_calls)}, indent=2))


def test_batch_park_failure_does_not_remove_ordinary(rig):
    provider, primitive, _, _ = rig
    with provider.movement_lease():
        snapshots = provider.deck_authority_snapshots(expected_generation=3)
    assert isinstance(snapshots['LOC_MS'], dict)
    assert isinstance(snapshots['LOC_PARK'], Exception)
    assert provider.deck_authority_cached_snapshot(expected_generation=3, target='LOC_MS') == snapshots['LOC_MS']


@pytest.mark.parametrize('change', ['cache_owner', 'generation'])
def test_batch_does_not_reuse_after_owner_change(retained_rig, monkeypatch, change):
    provider, primitive, _, _, _, _ = retained_rig
    qualify_full_predecessor(retained_rig)
    primitive.calls.clear()
    original = provider.deck_authority_snapshot
    def changed(**kwargs):
        result = original(**kwargs)
        if kwargs['target'] == 'LOC_PARK':
            if change == 'cache_owner':
                provider.invalidate_deck_authority_cache(reason='offline owner publication')
            else:
                monkeypatch.setattr(provider, 'generation_provider', lambda: 4)
        return result
    monkeypatch.setattr(provider, 'deck_authority_snapshot', changed)
    snapshots = provider.deck_authority_snapshots(expected_generation=3)
    assert isinstance(snapshots['LOC_PARK'], dict)
    if change == 'cache_owner':
        assert isinstance(snapshots['LOC_MS'], dict)
        assert sum(c[0] == 'xyz' for c in primitive.calls) == 6
        assert snapshots['LOC_MS']['captured_at'] > snapshots['LOC_PARK']['captured_at']
    else:
        assert isinstance(snapshots['LOC_MS'], RuntimeError)
        assert str(snapshots['LOC_MS']) == 'ownership_generation_changed'


@pytest.mark.parametrize('stop_after', [1, 2, 3])
def test_provider_yields_between_coordinates_without_negative_cache(rig, stop_after):
    from bioxp.hardware_status import HardwareCollectionPreempted
    provider, primitive, _, _ = rig
    before = provider.deck_authority_snapshot(expected_generation=3, target='LOC_MS')
    primitive.calls.clear()
    def check():
        if sum(c[0] == 'xyz' for c in primitive.calls) >= stop_after:
            raise HardwareCollectionPreempted('operator_action_pending')
    with pytest.raises(HardwareCollectionPreempted):
        provider.deck_authority_snapshots(expected_generation=3, before_query=check)
    assert provider.deck_authority_cached_snapshot(expected_generation=3, target='LOC_MS') == before


def test_batch_scope_reuse_rejects_reference_drift(retained_rig, monkeypatch):
    provider, primitive, _, references, _, _ = retained_rig
    qualify_full_predecessor(retained_rig)
    original = provider._offset_deck_semantic_state
    def mutate(**kwargs):
        result = original(**kwargs)
        from bioxp.services.reference_service import MarkAxisDesyncedCommand
        references.mark_desynced(MarkAxisDesyncedCommand('x', source='offline reference changed'))
        return result
    monkeypatch.setattr(provider, '_offset_deck_semantic_state', mutate)
    snapshots = provider.deck_authority_snapshots(expected_generation=3)
    assert isinstance(snapshots['LOC_PARK'], dict)
    assert isinstance(snapshots['LOC_MS'], Exception)


def test_preserved_board_observations_cannot_promote_new_can_readiness():
    from bioxp import api
    from bioxp.hardware_status import HardwareStateOwner
    owner = HardwareStateOwner()
    collectors = {
        'transport': lambda c: {'transport_internal_observation': {
            'CAN_READY': True, 'usb_bound': True, 'router_running': True}},
        'boards': lambda c: {4: {'ack': {'status': 100}}},
        'axes': lambda c: {},
    }
    full = owner.collect(['transport', 'boards'], collectors)['snapshot']
    assert api._snapshot_proves_can_ready(full)
    subset = owner.collect(['axes'], collectors)['snapshot']
    assert 'boards' in subset['domains']
    assert not api._snapshot_proves_can_ready(subset)


def test_api_preserves_base_publication_when_deck_yields_inside_lease(installed_retained, monkeypatch):
    from bioxp import api
    app, _, _, _, _ = installed_retained
    monkeypatch.setattr(api, '_hardware_collectors', lambda tester, **kwargs: {})
    monkeypatch.setattr(api.hardware_state, 'collect', lambda *args, **kwargs: {
        'ok': True, 'published': True, 'snapshot': {'snapshot_id': 'already-published'}})
    calls = []
    def deferred(*, yield_requested):
        calls.append(yield_requested())
        return {'enabled': False, 'disabled_reason': 'operator_action_pending'}
    monkeypatch.setattr(app.state, 'oem_deck_authority_collector', deferred)
    result = api._collect_and_publish_hardware_snapshot(['axes', 'latch'], reason='offline-lease-wait', automatic=True)
    assert calls == [False]
    assert result['ok'] and result['published']
    assert result['deck_authority'] == {'available': False, 'reason': 'operator_action_pending'}


def test_shared_scopes_keep_original_expiry(retained_rig, monkeypatch):
    from bioxp import oem_serial206_initialization as module
    provider, primitive, _, _, _, _ = retained_rig
    qualify_full_predecessor(retained_rig)
    clock = [100.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    original = provider._offset_deck_semantic_state
    def later(**kwargs):
        clock[0] = 102.0
        return original(**kwargs)
    monkeypatch.setattr(provider, '_offset_deck_semantic_state', later)
    snapshots = provider.deck_authority_snapshots(expected_generation=3)
    assert all(isinstance(row, dict) for row in snapshots.values())
    assert provider._deck_authority_scoped_cache['offset.v1'][0] == 100.0
    clock[0] = 115.0
    for target in snapshots:
        with pytest.raises(RuntimeError, match='deck_authority_cache_stale'):
            provider.deck_authority_cached_snapshot(expected_generation=3, target=target)
