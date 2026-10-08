"""Read-only capture replay and native HTTP update negotiation, no hardware IO."""
import asyncio
import copy
import json
import os
import time
from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from bioxp import operator_controls as controls
from tests.z_stop_fixtures import make_app


def apply(base, changes):
    body = copy.deepcopy(base)
    for change in changes:
        path = change[0]
        target = body
        for key in path[:-1]:
            target = target[key]
        if len(change) == 1:
            del target[path[-1]]
        else:
            value = change[1]
            if len(change) == 3:
                value = base
                for key in change[2]:
                    value = value[key]
            if isinstance(target, list) and path[-1] == len(target):
                target.append(copy.deepcopy(value))
            else:
                target[path[-1]] = copy.deepcopy(value)
    return body


def size(body):
    return len(json.dumps(body, separators=(',', ':'), ensure_ascii=False).encode())


def test_lossless_types_omission_arrays_and_shifted_receipts():
    before = {'a': True, 'b': None, 'c': [], 'receipts': [{'command_id': 'a', 'ok': False}, {'command_id': 'b', 'ok': None}]}
    for after in [
        {'a': 1, 'c': [None, False], 'receipts': [{'command_id': 'new', 'ok': False}, *before['receipts']]},
        {'a': None, 'b': False, 'c': {}, 'receipts': [before['receipts'][1]]},
        {'a': False, 'receipts': []},
    ]:
        assert apply(before, controls._catalog_changes(before, after)) == after
        assert type(apply(before, controls._catalog_changes(before, after))['a']) is type(after['a'])


def test_shifted_nested_receipts_read_original_baseline_paths():
    before = {'rows': [{'command_id': 'a', 'children': [{'command_id': 'aa'}, {'command_id': 'ab'}]},
                       {'command_id': 'b', 'children': [{'command_id': 'ba'}, {'command_id': 'bb'}]}]}
    after = {'rows': [copy.deepcopy(before['rows'][1]), copy.deepcopy(before['rows'][0])]}
    after['rows'][0]['children'].reverse()
    after['rows'][1]['children'].reverse()
    assert apply(before, controls._catalog_changes(before, after)) == after


def receive(part, previous=None):
    source = part.get('assessment_base', previous[1] if previous else None)
    assert source is not None
    if 'assessment_base' not in part:
        assert previous[0] == part['assessment_source_revision']
    snapshot = apply(source, part['assessment_changes'])
    return (part['assessment_revision'], snapshot), apply(snapshot, part['assessment_overlay'])


def test_capture_minutes_and_export():
    root = os.environ.get('BIOXP_CATALOG_CAPTURE_ROOT')
    if not root:
        pytest.skip('set BIOXP_CATALOG_CAPTURE_ROOT to actual captures')
    root = Path(root)
    captures = [json.loads((root / f'catalog-assessment-{i}.json').read_text()) for i in range(3)]
    baseline = captures[0]
    export = {'metadata': json.loads((root / 'catalog-metadata.json').read_text()), 'windows': {}}
    for mode in ('idle', 'changing', 'arrivals25_idle', 'multiple_clients', 'multiple_drafts'):
        cache = controls._OperatorPollCache()
        def wire(body, revisions):
            return {**cache.assessment_wire({k:v for k,v in body.items() if k != 'canonical'}, revisions[0]),
                    'canonical': cache.assessment_wire(body['canonical'], revisions[1])}
        clients = [None] * (2 if mode.startswith('multiple_') else 1)
        events, colds, cold_expected = [], [], []
        current = copy.deepcopy(baseline)
        def request(body, client, cold=False, draft=None):
            previous = clients[client]
            revisions = [item[0] for item in previous] if previous else ['', '']
            result = wire(body, revisions)
            legacy, full = receive(result, previous[0] if previous else None)
            canonical, canonical_full = receive(result['canonical'], previous[1] if previous else None)
            assert {**full, 'canonical': canonical_full} == body
            clients[client] = (legacy, canonical)
            if cold:
                colds.append(result)
                cold_expected.append(body)
            else:
                events.append({'client': client, 'draft': draft, 'request': revisions, 'response': result, 'expected': body})
            return result
        try:
            for client in range(len(clients)):
                body = copy.deepcopy(current)
                if mode.startswith('multiple_'):
                    body['dashboard']['z_axis']['provider']['target_preview'] = {
                        'requested_position_steps': client * 500, 'effective_position_steps': None}
                request(body, client, cold=True)
            ticks = 37 if mode == 'arrivals25_idle' else 12
            for tick in range(ticks):
                dashboard = current['canonical']['dashboard']
                dashboard['generated_at'] += 5
                dashboard['command_queue']['generated_at'] += 5
                if mode in ('arrivals25_idle', 'multiple_clients', 'multiple_drafts') and tick < 25:
                    # Ordinary same-outcome arrivals change native identity/path/
                    # sequence/times, NOT status, terminality or historical outcomes.
                    row = copy.deepcopy(dashboard['latest_receipts'][0])
                    row.update(command_id=f'offline-arrival-{tick}', sequence=1000+tick,
                               accepted_at=1800000000+tick*5, queued_at=1800000000+tick*5,
                               dispatched_at=1800000001+tick*5, finished_at=1800000002+tick*5,
                               status_path=f'/operator/v2/actions/receipts/offline-arrival-{tick}')
                    dashboard['latest_receipts'] = [row, *dashboard['latest_receipts'][:-1]]
                if mode == 'changing':
                    state = current['action_states'][0]
                    state.update(enabled=tick % 2 == 0, available=tick % 2 == 0, provider_available=tick % 2 == 0,
                                 disabled_reason=None if tick % 2 == 0 else 'offline fixture provider',
                                 provider_unavailable_reason=None if tick % 2 == 0 else 'offline fixture provider')
                    state['dependencies'][0]['met'] = tick % 2 == 0
                    current['dashboard']['pipettes']['channels'][0]['hardware_tip_status'] = {'ok': True, 'tip_loaded': tick % 2 == 0, 'hardware_truth_level': 'offline_fixture'}
                    dashboard['deck']['current_well'] = tick
                    dashboard['deck']['semantic_state_revision'] += 1
                    dashboard['latest_receipts'][0]['state_version'] += 1
                    dashboard['latest_receipts'][0]['error'] = None if tick % 2 == 0 else {'message': 'offline fixture failure'}
                    dashboard['latest_receipts'][0]['recovery'] = {'decision': None if tick % 2 == 0 else False, 'at': tick}
                for client in range(len(clients)):
                    body = copy.deepcopy(current)
                    draft = ((tick + 1) * 500 + client if mode == 'multiple_drafts' else client * 500) if mode.startswith('multiple_') else None
                    if draft is not None:
                        # Source changes BETWEEN the two clients too: resolve
                        # each requested old snapshot before advancing slots.
                        body['canonical']['dashboard']['deck']['current_well'] = client
                        body['dashboard']['z_axis']['provider']['target_preview'] = {
                            'requested_position_steps': draft, 'effective_position_steps': None}
                    result = request(body, client, draft=draft)
                    # A second ordinary client and arbitrary drafts do not evict
                    # the first one's source revision or trigger full resync.
                    assert 'assessment_base' not in result
                    assert 'assessment_base' not in result['canonical']
                assert len(cache._assessment_bases) == 2
                assert all(len(slot) == 3 for slot in cache._assessment_bases.values())
            old_bytes = sum(size(e['expected']) for e in events)
            new_bytes = sum(size(e['response']) for e in events)
            result = {'colds': colds, 'cold_expected': cold_expected, 'events': events,
                      'old_bytes': old_bytes, 'new_bytes': new_bytes,
                      'reduction_percent': 100 * (1-new_bytes/old_bytes),
                      'resyncs': sum('assessment_base' in e['response'] or 'assessment_base' in e['response']['canonical'] for e in events)}
            if mode == 'arrivals25_idle':
                result['phases'] = {}
                for name, subset in [('arrivals', events[:25]), ('idle_after25', events[25:])]:
                    old, new = sum(size(e['expected']) for e in subset), sum(size(e['response']) for e in subset)
                    result['phases'][name] = {'old_bytes': old, 'new_bytes': new, 'reduction_percent':100*(1-new/old)}
                    assert new <= old * .05
            export['windows'][mode] = result
            assert new_bytes <= old_bytes * .05, (mode, new_bytes, old_bytes)
            # Eviction is bounded and costs a current full resync, counted here
            # explicitly rather than misclassifying it as a new cold fetch.
            stale = wire(current, [colds[0]['assessment_revision'], colds[0]['canonical']['assessment_revision']])
            if mode != 'idle':
                assert 'assessment_base' in stale['canonical']
                assert stale['canonical']['assessment_base']['dashboard']['latest_receipts'] == current['canonical']['dashboard']['latest_receipts']
            for part, expected in [(stale, {k:v for k,v in current.items() if k != 'canonical'}), (stale['canonical'], current['canonical'])]:
                previous = receive(colds[0] if part is stale else colds[0]['canonical'])[0]
                assert receive(part, previous)[1] == expected
            result['explicit_evicted_resync_bytes'] = size(stale)
            assert cache.assessment_wire(current, None) is current
        finally:
            cache.close()
    export['cold'] = export['windows']['idle']['colds'][0]
    export['cold_bytes'] = size(export['cold'])
    export['metadata_bytes'] = size(export['metadata'])
    output = os.environ.get('BIOXP_CATALOG_UPDATE_EXPORT')
    if output:
        Path(output).write_text(json.dumps(export, separators=(',', ':')))
    print(json.dumps({k: {f:v for f,v in w.items() if f not in ('colds', 'cold_expected', 'events')} for k,w in export['windows'].items()}))


def test_snapshot_advancement_resync_and_display_overlay_are_lossless():
    cache = controls._OperatorPollCache()
    body = {'catalog_view': 'assessment', 'schema_version': 'bioxp.operator_control_catalog.v1',
            'ownership_generation': 1, 'metadata_revision': 'definitions',
            'dashboard': {'generated_at': 1, 'command_queue': {'generated_at': 1},
                          'z_axis': {'provider': {'target_preview': None}}},
            'evidence': {'nullable': None, 'truth': False}}
    try:
        cold = cache.assessment_wire(body, '')
        original, _ = receive(cold)
        retained = original
        for tick in range(1, 5):
            current = copy.deepcopy(body)
            current['dashboard']['generated_at'] = tick
            current['dashboard']['command_queue']['generated_at'] = tick
            current['dashboard']['z_axis']['provider']['target_preview'] = {'requested_position_steps': tick}
            update = cache.assessment_wire(current, retained[0])
            retained, reconstructed = receive(update, retained)
            assert retained == original  # no draft/time-only eviction
            assert reconstructed == current
        for tick in range(3):
            body['evidence'] = {'recovery': {'decision': None if tick == 0 else False, 'sequence': tick}}
            update = cache.assessment_wire(body, retained[0])
            assert 'assessment_base' not in update
            retained, reconstructed = receive(update, retained)
            assert reconstructed == body
            assert 'nullable' not in reconstructed['evidence']
        stale = cache.assessment_wire(body, original[0])
        assert stale['assessment_base'] == body
        assert stale['assessment_changes'] == []
        recovered, reconstructed = receive(stale)
        assert reconstructed == body
        assert 'assessment_base' not in cache.assessment_wire(body, recovered[0])
        for field, value in [('ownership_generation', 2), ('metadata_revision', 'new-definitions')]:
            body[field] = value
            reset = cache.assessment_wire(body, retained[0])
            assert reset['assessment_base'] == body
            assert cache._assessment_bases[body['schema_version']][2] is None
            retained, reconstructed = receive(reset)
            assert reconstructed == body
        assert len(cache._assessment_bases) == 1
    finally:
        cache.close()


def test_native_http_updates_preserve_draft_z_and_final_deck_display(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    project = controls.hardware_state.project
    monkeypatch.setattr(controls.hardware_state, 'project', lambda *args, include_lifecycle=False, **kwargs: project(*args, **kwargs))
    def get(client, route, params):
        for _ in range(100):
            response = client.get(route, params=params)
            if response.status_code != 503:
                assert response.status_code == 200, response.text
                return response.json()
            time.sleep(.01)
        pytest.fail('bounded test warmup exhausted')
    with TestClient(app) as client:
        for route in ('/operator/control-catalog', '/operator/v2/control-catalog'):
            cold = get(client, route, {'view': 'assessment', 'assessment_base': ''})
            assert 'assessment_base' in cold
            previous, _ = receive(cold)
            for z in (-2147483648, 0, 65000, 2147483647):
                update = get(client, route, {'view': 'assessment', 'assessment_base': previous[0], 'z_target_steps': z})
                assert 'assessment_base' not in update
                previous, current = receive(update, previous)
                assert 'dashboard' in current
                if route == '/operator/control-catalog':
                    assert current['dashboard']['z_axis']['provider']['target_preview']['requested_position_steps'] == z
            full = get(client, route, {})
            assert 'actions' in full and 'assessment_changes' not in full
        assert len(app.state.operator_poll_cache._assessment_bases) == 2
