"""Frozen/offline replacement and status scheduling contracts."""
import copy
import json
import os
from pathlib import Path
import threading
import time

import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import rig, retained_rig, qualify_test_references
from tests.test_deck_scoped_integration import installed_retained, catalog_action
from tests.test_deck_cache_owner_fences import wait_enabled


def export_status(name, payload):
    """Optional cross-consumer evidence: actual producer bytes, never a fixture."""
    destination = os.environ.get('DECK_REFRESH_STATUS_EXPORT')
    if destination:
        path = Path(destination)
        samples = json.loads(path.read_text()) if path.exists() else {}
        samples[name] = payload
        path.write_text(json.dumps(samples, indent=2))


def test_failed_replacement_withdraws_only_affected_scope(rig, monkeypatch):
    provider, primitive, _, _ = rig
    provider.deck_authority_snapshot(expected_generation=3, target='LOC_OC')
    def failed():
        raise RuntimeError('deck_latch_observation_failed')
    monkeypatch.setattr(provider, '_fresh_deck_latch_observation', failed)
    with pytest.raises(RuntimeError, match='deck_latch_observation_failed'):
        provider.deck_authority_snapshot(expected_generation=3, target='LOC_OC')
    calls = list(primitive.calls)
    with pytest.raises(RuntimeError, match='deck_latch_observation_failed'):
        provider.deck_authority_cached_snapshot(expected_generation=3, target='LOC_OC')
    observed = provider.deck_observation_freshness(expected_generation=3)
    assert observed['available'] is False and observed['outcome'] == 'failed'
    assert observed['freshness']['state'] == 'fresh'
    assert observed['freshness']['fresh_for_s'] == 15
    assert primitive.calls == calls


def test_return_boundary_invalidation_cannot_restamp_sample(rig, monkeypatch):
    provider, _, _, _ = rig
    original = provider._collect_deck_authority
    def raced(**kwargs):
        result = original(**kwargs)
        provider.invalidate_deck_authority_cache(reason='offline real owner changed after final read')
        return result
    monkeypatch.setattr(provider, '_collect_deck_authority', raced)
    provider.deck_authority_snapshot(expected_generation=3, target='LOC_OC')
    with pytest.raises(RuntimeError, match='deck_authority_cache_unavailable'):
        provider.deck_authority_cached_snapshot(expected_generation=3, target='LOC_OC')


@pytest.mark.parametrize('boundary', [14.999, 15.0])
def test_replacement_does_not_extend_sample_deadline(rig, monkeypatch, boundary):
    from bioxp import oem_serial206_initialization as module
    provider, _, _, _ = rig
    clock = [100.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    snapshot = provider.deck_authority_snapshot(expected_generation=3, target='LOC_OC')
    original = provider._fresh_deck_latch_observation
    def replacement():
        clock[0] = 100.0 + boundary
        if boundary < 15:
            assert provider.deck_authority_cached_snapshot(expected_generation=3, target='LOC_OC') == snapshot
        else:
            with pytest.raises(RuntimeError, match='deck_authority_cache_stale'):
                provider.deck_authority_cached_snapshot(expected_generation=3, target='LOC_OC')
        return original()
    monkeypatch.setattr(provider, '_fresh_deck_latch_observation', replacement)
    clock[0] = 101.0
    provider.deck_authority_snapshot(expected_generation=3, target='LOC_OC')
    assert provider._deck_authority_scoped_cache['offset.v1'][0] == 101.0


@pytest.mark.parametrize('end', ['success', 'yield', 'failure', 'owner_change'])
def test_warm_catalog_survives_replacement_until_real_outcome(installed_retained, monkeypatch, end):
    from bioxp import api
    app, provider, primitive, references, _ = installed_retained
    catalog_action(app)
    qualify_test_references(references)
    assert api._collect_and_publish_hardware_snapshot(['axes', 'latch'], reason='warm')['deck_authority']['enabled']
    wait_enabled(app, True)
    epoch = provider._deck_authority_cache_epoch
    entered, release = threading.Event(), threading.Event()
    active = [False]
    monkeypatch.setattr(app.state, 'operator_normal_action_active', lambda: active[0])
    monkeypatch.setattr(api, '_hardware_collectors', lambda tester, **kwargs: {})
    def collect(*args, **kwargs):
        entered.set()
        assert release.wait(5)
        if end == 'yield':
            active[0] = True
        elif end == 'owner_change':
            provider.invalidate_deck_authority_cache(reason='offline owner mutation')
            active[0] = True
        elif end == 'failure':
            monkeypatch.setattr(provider, '_fresh_deck_latch_observation', lambda: (_ for _ in ()).throw(RuntimeError('deck_latch_observation_failed')))
        return {'ok': True, 'snapshot': {}}
    monkeypatch.setattr(api.hardware_state, 'collect', collect)
    results = []
    worker = threading.Thread(target=lambda: results.append(api._collect_and_publish_hardware_snapshot(
        ['axes', 'latch'], reason='replacement', automatic=True)))
    worker.start()
    try:
        assert entered.wait(3)
        calls = list(primitive.calls)
        for _ in range(3):
            assert catalog_action(app)['enabled'] is True
        assert provider._deck_authority_cache_epoch is epoch
        assert primitive.calls == calls
    finally:
        release.set()
        worker.join(5)
    assert not worker.is_alive() and results
    active[0] = False
    wait_enabled(app, end in {'success', 'yield'})


def test_manual_refresh_alias_uses_normal_full_domains(installed_retained, monkeypatch):
    from bioxp import api, operator_controls
    app, _, _, _, _ = installed_retained
    _, dispatch = operator_controls._build_catalog(app)
    target = dispatch['oem.deck.collect_authority']
    assert target['fixed_inputs'] == {'body': {}}
    called = []
    monkeypatch.setattr(api, '_collect_and_publish_hardware_snapshot', lambda requested, **kw: called.append(requested) or {'ok': True})
    response = TestClient(app).post(target['path'], json=target['fixed_inputs']['body'])
    assert response.status_code == 200
    assert called == [list(api.DEFAULT_HARDWARE_SNAPSHOT_DOMAINS)]
    assert {'transport', 'boards', 'power', 'interlock', 'latch', 'axes', 'gripper'} <= set(called[0])


def test_status_deck_freshness_is_passive_scope_specific_and_negative(installed_retained, monkeypatch):
    from bioxp import api
    from bioxp.hardware_status import HardwareStateOwner
    app, provider, primitive, references, _ = installed_retained
    catalog_action(app)
    # Real domain projection over offline observations, rather than the shared
    # installed fixture's deliberately missing generic hardware projection.
    owner = HardwareStateOwner()
    domains = ['transport', 'boards', 'power', 'interlock', 'latch', 'axes', 'gripper', 'chiller']
    owner.collect(domains, {name: lambda context: {} for name in domains})
    monkeypatch.setattr(api.hardware_state, 'project', owner.project)
    # Failed references are a current negative observation, not missing evidence.
    api._collect_and_publish_hardware_snapshot(['axes', 'latch'], reason='negative')
    status = api._status_payload()
    export_status('negative_ordinary', status)
    negative = status['deck_authority']
    assert negative['outcome'] == 'failed' and not negative['available']
    assert negative['freshness']['state'] == 'fresh'
    qualify_test_references(references)
    api._collect_and_publish_hardware_snapshot(['axes', 'latch'], reason='positive')
    before = list(primitive.calls)
    status = api._status_payload()
    export_status('positive_ordinary_failed_park', status)
    positive = status['deck_authority']
    assert positive['available'] and positive['freshness']['state'] == 'fresh'
    assert positive['freshness']['fresh_for_s'] == 15
    assert isinstance(provider._deck_authority_scoped_cache['park.full'][2], Exception)
    assert api._status_payload()['deck_authority']['freshness']['age_s'] >= positive['freshness']['age_s']
    assert primitive.calls == before
    provider.invalidate_deck_authority_cache(reason='offline command owner change')
    status = api._status_payload()
    export_status('deck_invalidated', status)
    assert status['deck_authority']['freshness']['state'] == 'missing'


@pytest.mark.parametrize('mutation', ['reference', 'board', 'semantic'])
def test_status_rejects_independent_owner_drift(installed_retained, monkeypatch, mutation):
    from bioxp import api
    app, provider, _, references, _ = installed_retained
    catalog_action(app)
    qualify_test_references(references)
    api._collect_and_publish_hardware_snapshot(['axes', 'latch'], reason='warm')
    assert provider.deck_observation_freshness(expected_generation=provider.generation_provider())['available']
    if mutation == 'reference':
        original = references.snapshot
        def changed(axes):
            result = copy.deepcopy(original(axes))
            result['rows']['x']['state_version'] += 1
            return result
        monkeypatch.setattr(references, 'snapshot', changed)
    elif mutation == 'board':
        stamps = provider.deck_owner_authority_stamps()
        monkeypatch.setattr(provider, 'deck_owner_authority_stamps', lambda: {**stamps, 'board_epoch_4': stamps['board_epoch_4'] + 1})
    else:
        original = provider._deck_semantic_state_reader
        monkeypatch.setattr(provider, '_deck_semantic_state_reader', lambda: {**original(), 'semantic_state_revision': -1})
    assert provider.deck_observation_freshness(expected_generation=provider.generation_provider())['freshness']['state'] == 'missing'


def test_status_admission_projection_requires_all_domains(installed_retained, monkeypatch):
    from bioxp import api
    from bioxp.hardware_status import HardwareStateOwner
    owner = HardwareStateOwner()
    monkeypatch.setattr(api, 'hardware_state', owner)
    domains = ['transport', 'boards', 'power', 'interlock', 'latch', 'axes', 'gripper', 'chiller']
    owner.collect(domains, {name: lambda context: {} for name in domains})
    status = api._status_payload()
    export_status('admission_all_domains', status)
    assert status['admission_observation']['available'] is True
    owner.invalidate_domains('axes', 'gripper', reason='offline command completion')
    status = api._status_payload()
    assert status['cache_state'] == 'fresh'
    export_status('admission_axes_gripper_invalidated', status)
    assert status['admission_observation']['available'] is False


@pytest.mark.parametrize('result', [
    {'ok': False, 'error': 'ownership_epoch_changed_during_collection'},
    {'ok': True, 'snapshot': {'domains': {'axes': {'status': 'error'}}}},
])
def test_failed_base_observation_withdraws_cache_before_yield(installed_retained, monkeypatch, result):
    from bioxp import api
    app, provider, _, references, _ = installed_retained
    catalog_action(app)
    qualify_test_references(references)
    api._collect_and_publish_hardware_snapshot(['axes', 'latch'], reason='warm')
    active = [False]
    monkeypatch.setattr(app.state, 'operator_normal_action_active', lambda: active[0])
    monkeypatch.setattr(api, '_hardware_collectors', lambda tester, **kwargs: {})
    def collect(*args, **kwargs):
        active[0] = True
        return copy.deepcopy(result)
    monkeypatch.setattr(api.hardware_state, 'collect', collect)
    api._collect_and_publish_hardware_snapshot(['axes', 'latch'], reason='failed', automatic=True)
    with pytest.raises(RuntimeError, match='deck_authority_cache_unavailable'):
        provider.deck_authority_cached_snapshot(expected_generation=provider.generation_provider(), target='LOC_OC')


def test_bootstrap_publishes_both_scopes_under_actual_new_epoch(retained_rig):
    from tests.test_deck_scoped_integration import qualify_full_predecessor
    provider, primitive, _, _, _, _ = retained_rig
    before = provider._deck_authority_cache_epoch
    qualify_full_predecessor(retained_rig)
    assert before is not provider._deck_authority_cache_epoch
    full = provider.deck_authority_cached_snapshot(expected_generation=3, target='LOC_PARK')
    assert full['current_location_id'] == 'LOC_PARK'
    provider.deck_authority_snapshot(expected_generation=3, target='LOC_OC')
    calls = list(primitive.calls)
    assert provider.deck_authority_cached_snapshot(expected_generation=3, target='LOC_PARK') == full
    provider.deck_authority_cached_snapshot(expected_generation=3, target='LOC_OC')
    assert primitive.calls == calls


def test_idle_refresh_after_real_offline_command_recovers_warm_catalog(installed_retained, monkeypatch):
    from bioxp import api
    from tests.test_deck_automatic_refresh_owner import request, finish
    app, provider, _, references, _ = installed_retained
    catalog_action(app)
    qualify_test_references(references)
    api._collect_and_publish_hardware_snapshot(list(api.DEFAULT_HARDWARE_SNAPSHOT_DOMAINS), reason='warm')
    wait_enabled(app, True)
    client = TestClient(app)
    submitted = client.post('/operator/v2/actions/oem.deck.move_to_location', json=request(provider, 'offline-post-command-refresh'))
    assert submitted.status_code == 200, submitted.text
    app.state.operator_command_plane.start()
    command_id = submitted.json()['command_id']
    assert finish(client, command_id)['status'] == 'completed'
    assert app.state.operator_command_plane.store.wait_for_command_workers([command_id], timeout=2)
    wait_enabled(app, False)
    monkeypatch.setattr(api, '_hardware_collectors', lambda tester, **kwargs: {})
    refreshed = api._collect_and_publish_hardware_snapshot(list(api.DEFAULT_HARDWARE_SNAPSHOT_DOMAINS), reason='idle', automatic=True)
    assert refreshed['deck_authority']['enabled']
    wait_enabled(app, True)
