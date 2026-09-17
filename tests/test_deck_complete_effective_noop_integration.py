"""Run with follow-up N's exact command-plane integration overlay."""
import json
import subprocess
import sys
import time
import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained, catalog_action
from tests.test_deck_complete_effective import adapter_rig
from tests.test_deck_complete_oem import export


def test_canonical_camera_all_board_noop_with_n(installed_retained, retained_rig, monkeypatch):
    from bioxp import api, operator_command_plane
    import hashlib, os
    from pathlib import Path
    active_sha = hashlib.sha256(Path(operator_command_plane.__file__).read_bytes()).hexdigest()
    if os.environ.get('E_N_OVERLAY_SHA256'):
        assert active_sha == os.environ['E_N_OVERLAY_SHA256']
    export('N-effective-source', {'path': operator_command_plane.__file__, 'sha256': active_sha})
    app, provider, observations, refs, root = installed_retained
    plane = app.state.operator_command_plane
    generation = int(provider.generation_provider())
    api.serial206_oem_initialization_provider_status()
    leaf, adapter, raw = adapter_rig((provider, observations, retained_rig[2], refs, plane.store, root),
        monkeypatch, (87751, 0))
    monkeypatch.setattr(adapter, 'generation_provider', lambda: generation)
    monkeypatch.setattr(adapter.y_provider, 'generation_provider', lambda: generation)
    assert api._collect_and_publish_hardware_snapshot(['axes', 'latch'],
        reason='isolated-clamp-noop')['deck_authority']['enabled']
    action = catalog_action(app)
    client = TestClient(app)
    body = {'schema_version': 'bioxp.operator_action_request.v2',
        'idempotency_key': 'canonical-clamp-noop',
        'expected_ownership_generation': generation,
        'expected_board_epoch_by_board': action['expected_board_epoch_by_board'],
        'inputs': {'target': 'LOC_OC_COVER_STORAGE', 'camera_offset': True}}
    response = client.post('/operator/v2/actions/oem.deck.move_to_location', json=body)
    assert response.status_code == 200, response.text
    cid = response.json()['command_id']
    plane.start()
    compact = {}
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        compact = client.get('/operator/v2/actions/receipts/' + cid).json()
        if compact['status'] not in {'queued', 'dispatched', 'issued_pending'}:
            break
        time.sleep(.01)
    detail = client.get('/operator/v2/actions/receipts/' + cid + '?detail=true').json()
    export('canonical-all-board-noop-N', {'detail': detail, 'compact': compact, 'native': raw,
        'xy': adapter.xy_rows, 'moves': leaf.moves})
    assert detail['status'] == 'completed', detail
    deck = detail['deck_movement']
    assert deck['semantic_state_committed']
    assert not deck['controller_completion_verified']
    assert not deck['controller_command_acknowledged']
    assert not deck['delivery_attempted']
    assert detail['completion_class'] == 'source_noop'
    assert not detail['physical_effect_verified']
    proof = deck['stages'][3]['terminal_evidence']['provider_evidence']
    assert proof['source_noop'] and not proof['delivery_attempted']
    assert proof['controller_completion_verified']  # terminal source-only proof, NOT motor event
    assert not proof['controller_command_acknowledged']
    assert proof['raw_requested_y_steps'] == -1687 and proof['oem_effective_y_steps'] == 0
    assert not leaf.moves and not leaf.wait_calls
    xy = adapter.xy_rows[-1]
    assert xy['branch'] == 'near_axis_sequential' and xy['launch_order'] == ['y']
    assert xy['source_noop_verified']
    for _ in range(3):
        assert client.get('/operator/v2/actions/receipts/' + cid + '?detail=true').json() == detail
        catalog_action(app)
    replay = client.post('/operator/v2/actions/oem.deck.move_to_location', json=body)
    assert replay.status_code == 200 and replay.json()['command_id'] == cid
    assert not leaf.moves
    plane.stop()
    script = ('import json,sys; from tests.test_deck_scoped_integration import fresh_process_receipts; '
        'print(json.dumps(fresh_process_receipts(sys.argv[1],sys.argv[2])))')
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', script, str(root), cid], text=True, timeout=12))
    assert reopened == {'compact': compact, 'detail': detail}
