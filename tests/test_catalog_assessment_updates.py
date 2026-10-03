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


def test_capture_minutes_and_export():
    root = os.environ.get('BIOXP_CATALOG_CAPTURE_ROOT')
    if not root:
        pytest.skip('set BIOXP_CATALOG_CAPTURE_ROOT to actual captures')
    root = Path(root)
    captures = [json.loads((root / f'catalog-assessment-{i}.json').read_text()) for i in range(3)]
    baseline = captures[0]
    metadata = json.loads((root / 'catalog-metadata.json').read_text())
    cache = controls._OperatorPollCache()
    def wire(body, legacy='', canonical=''):
        return {**cache.assessment_wire({k:v for k,v in body.items() if k != 'canonical'}, legacy),
                'canonical': cache.assessment_wire(body['canonical'], canonical)}
    cold = wire(baseline)
    revisions = (cold['assessment_revision'], cold['canonical']['assessment_revision'])
    export = {'metadata': metadata, 'cold': cold, 'windows': {}}
    try:
        for mode in ('idle', 'changing'):
            responses, expected = [], []
            for tick in range(12):
                current = copy.deepcopy(captures[1 + tick % 2])
                # Timestamp extrapolation is an offline minute, not live sampling.
                dashboard = current['canonical']['dashboard']
                dashboard['generated_at'] += tick * 5
                dashboard['command_queue']['generated_at'] += tick * 5
                if mode == 'changing':
                    state = current['action_states'][0]
                    state.update(enabled=tick % 2 == 0, available=tick % 2 == 0, provider_available=tick % 2 == 0,
                                 disabled_reason=None if tick % 2 == 0 else 'offline fixture provider',
                                 provider_unavailable_reason=None if tick % 2 == 0 else 'offline fixture provider')
                    state['dependencies'][0]['met'] = tick % 2 == 0
                    current['dashboard']['pipettes']['channels'][0]['hardware_tip_status'] = {'ok': True, 'tip_loaded': tick % 2 == 0, 'hardware_truth_level': 'offline_fixture'}
                    dashboard['deck']['current_well'] = tick
                    dashboard['deck']['semantic_state_revision'] += tick
                    dashboard['latest_receipts'][0]['state_version'] += tick
                    dashboard['latest_receipts'][0]['error'] = None if tick % 2 == 0 else {'message': 'offline fixture failure'}
                    # Meaningful cross-client arrival: preserve full queue/receipt identities.
                    if tick >= 6:
                        new = copy.deepcopy(dashboard['latest_receipts'][0])
                        new.update(command_id='offline-new-command', state_version=1, status='accepted', terminal=False)
                        dashboard['latest_receipts'] = [new, *dashboard['latest_receipts'][:-1]]
                result = wire(current, *revisions)
                for name, full in [(None, {k:v for k,v in current.items() if k != 'canonical'}), ('canonical', current['canonical'])]:
                    part = result if name is None else result[name]
                    base = cold if name is None else cold[name]
                    assert 'assessment_base' not in part
                    assert apply(base['assessment_base'], part['assessment_changes']) == full
                responses.append(result)
                expected.append(current)
            old_bytes = sum(map(size, expected))
            new_bytes = sum(map(size, responses))
            export['windows'][mode] = {'responses': responses, 'expected': expected, 'old_bytes': old_bytes, 'new_bytes': new_bytes,
                                      'reduction_percent': 100 * (1-new_bytes/old_bytes)}
            assert new_bytes <= old_bytes * .05, (mode, new_bytes, old_bytes)
        export['cold_bytes'] = size(cold)
        export['metadata_bytes'] = size(metadata)
        # Concrete semantic boundary, not a claim of universal 95% compression:
        # accumulated changes to every retained receipt must still be delivered.
        broad = copy.deepcopy(baseline)
        for index, row in enumerate(broad['canonical']['dashboard']['latest_receipts']):
            row.update(command_id=f'offline-replacement-{index}', status='ambiguous', terminal=True,
                       state_version=101, error={'message': 'offline simultaneous outcome change'})
        broad_wire = wire(broad, *revisions)
        assert apply(cold['canonical']['assessment_base'], broad_wire['canonical']['assessment_changes']) == broad['canonical']
        export['wholesale_boundary'] = {'old_bytes': size(broad), 'new_bytes': size(broad_wire),
                                       'reduction_percent': 100 * (1-size(broad_wire)/size(broad))}
        # Explicit full and old assessment callers are not changed.
        assert cache.assessment_wire(baseline, None) is baseline
        # Unknown baselines resync in one read; metadata/generation changes replace only finite slots.
        assert wire(baseline, 'not-known', 'not-known')['assessment_base'] == cold['assessment_base']
        changed = copy.deepcopy(baseline)
        changed['ownership_generation'] += 1
        replacement = wire(changed, *revisions)
        assert replacement['assessment_revision'] != revisions[0]
        assert 'assessment_base' in replacement
        assert len(cache._assessment_bases) == 2
        output = os.environ.get('BIOXP_CATALOG_UPDATE_EXPORT')
        if output:
            Path(output).write_text(json.dumps(export, separators=(',', ':')))
        print(json.dumps({k: {f:v for f,v in w.items() if f not in ('responses', 'expected')} for k,w in export['windows'].items()}))
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
            for z in (-2147483648, 0, 65000, 2147483647):
                update = get(client, route, {'view': 'assessment', 'assessment_base': cold['assessment_revision'], 'z_target_steps': z})
                assert 'assessment_base' not in update
                current = apply(cold['assessment_base'], update['assessment_changes'])
                assert 'dashboard' in current
                if route == '/operator/control-catalog':
                    assert current['dashboard']['z_axis']['provider']['target_preview']['requested_position_steps'] == z
            full = get(client, route, {})
            assert 'actions' in full and 'assessment_changes' not in full
        assert len(app.state.operator_poll_cache._assessment_bases) == 2
