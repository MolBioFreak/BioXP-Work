"""Connected offline native document, real custody publication and addressed Stop."""
import json
import os
import subprocess
import sys
import threading
from pathlib import Path

import pytest
from tests.test_deck_scoped_authority import retained_rig
from tests.test_cover_carry_release_connected import connected
from tests.test_native_completion_handoff import handoff


@pytest.mark.parametrize('outcome', ['success', 'child_failure', 'stop'])
def test_sequential_changing_custody_document(handoff, monkeypatch, outcome):
    from bioxp import api
    from bioxp.services.protocol_service import (
        bind_protocol_dispatcher, create_protocol_job, ProtocolOperatorBundleStore)
    rig = handoff
    store, provider = rig.store, rig.provider
    payload = {'source_type': 'native', 'idempotency_key': 'e113-custody-' + outcome,
        'live_execution': {'live_execution_ack': True}, 'document': {
        'protocol_id': 'e113-changing-custody', 'stages': [{'stage_id': 'moves', 'actions': [
            {'action_id': 'output-out', 'kind': 'move_cover',
             'params': {'cover_id': 'CV_OUTPUT', 'target_location': 'LOC_OCS'}},
            {'action_id': 'reagent-out', 'kind': 'move_cover',
             'params': {'cover_id': 'CV_REAGENT', 'target_location': 'LOC_RCS'}},
            {'action_id': 'output-back', 'kind': 'move_cover',
             'params': {'cover_id': 'CV_OUTPUT', 'target_location': 'LOC_OC'}},
        ]}]}}
    # Observe genuine publications, returning the unmodified owner result.
    publications = []
    real_publish = store.publish_deck_owner_state
    def publish(**kw):
        result = real_publish(**kw)
        publications.append(store.deck_semantic_state())
        return result
    provider.bind_deck_semantic_state_publisher(publish)
    done = threading.Event()
    real_finish = store.finish_workflow
    def finish(*args, **kwargs):
        result = real_finish(*args, **kwargs)
        done.set()
        return result
    monkeypatch.setattr(store, 'finish_workflow', finish)
    original_move = rig.native.motor_oem_move_absolute
    injected = []
    def move(board, target, **kwargs):
        # Fail/Stop action two's carry AFTER real pickup publication.
        semantic = store.deck_semantic_state()
        if (outcome != 'success' and not injected and len(rig.admitted) == 2
                and semantic['plate_on_gantry'] == 5
                and semantic['movable_plate_locations']['REAGENT_COVER'] == 'LOC_GANTRY'):
            injected.append({'custody': semantic, 'command_id': rig.admitted[-1][2]})
            if outcome == 'child_failure':
                raise RuntimeError('e113_second_child_transport_failure')
            receipt = store.begin_interrupt('oem.x.stop', state=rig.plane._state(),
                request={'idempotency_key': 'e113-document-stop'})
            store.mark_interrupt_attempted(idempotency_key='e113-document-stop')
            store.finalize_interrupt(idempotency_key='e113-document-stop', receipt=receipt,
                attempted=True, acknowledged=True, response={'ok': True, 'offline_transport': True})
            injected[-1]['interrupt'] = receipt
        return original_move(board, target, **kwargs)
    monkeypatch.setattr(rig.native, 'motor_oem_move_absolute', move)
    artifacts = ProtocolOperatorBundleStore(rig.root / 'e113-artifacts')
    handlers = {'move_cover': api._protocol_live_plate_move_handler}
    bind_protocol_dispatcher(store, binding_factory=lambda *a, **k: (handlers, {}, {}),
        artifact_store=artifacts)
    stamps = provider.deck_owner_authority_stamps()
    job = create_protocol_job(payload, dry_run=False, command_store=store, store=artifacts,
        handlers=handlers, ownership_generation=stamps['ownership_generation'], board_epochs={})
    store.start(rig.plane._dispatch_one)
    assert done.wait(20), store.get_workflow(job['job_id'])
    terminal = store.get_workflow(job['job_id'])
    semantic = store.deck_semantic_state()
    commands = [store.get_command(cid) for _, _, cid in rig.admitted]
    if os.environ.get('E113_LINEAGE_OUTPUT'):
        Path(os.environ['E113_LINEAGE_OUTPUT'], outcome + '-probe.json').write_text(json.dumps({'commands': commands, 'publications': publications, 'injected': injected}, indent=2))
    assert commands[0]['status'] == 'completed'
    assert all(c['physical_effect_verified'] is False for c in commands)
    assert any(s['plate_on_gantry'] == 4 and s['movable_plate_locations']['OUTPUT_COVER'] == 'LOC_GANTRY' for s in publications)
    assert any(s['plate_on_gantry'] is None and s['movable_plate_locations']['OUTPUT_COVER'] == 'LOC_OC_COVER_STORAGE' for s in publications)
    assert any(s['plate_on_gantry'] == 5 and s['movable_plate_locations']['REAGENT_COVER'] == 'LOC_GANTRY' for s in publications)
    if outcome == 'success':
        assert terminal['command']['status'] == 'completed', terminal
        assert len(commands) == 3 and all(c['status'] == 'completed' for c in commands)
        assert semantic['movable_plate_locations'] == {'OUTPUT_COVER': 'LOC_OC_COVER', 'REAGENT_COVER': 'LOC_RC_COVER_STORAGE'}
        assert semantic['plate_on_gantry'] is None
    else:
        assert terminal['command']['status'] != 'completed', terminal
        assert len(commands) == 2
        assert semantic['plate_on_gantry'] == 5
        assert semantic['movable_plate_locations'] == {'OUTPUT_COVER': 'LOC_OC_COVER_STORAGE', 'REAGENT_COVER': 'LOC_GANTRY'}
        assert len(injected) == 1
        if outcome == 'stop':
            assert commands[1]['status'] == 'interrupted', commands[1]
            assert commands[1]['command_id'] in injected[0]['interrupt']['active_command_ids']
        else:
            assert commands[1]['status'] == 'ambiguous', commands[1]
            assert 'e113_second_child_transport_failure' in repr(commands[1])
    children = [store.wp8_operation_evidence(c['command_id']) for c in commands]
    parent_rows = store.connection.execute("SELECT command_id,parent_command_id FROM operator_commands WHERE action_id='oem.deck._finite_operation'").fetchall()
    assert len(parent_rows) == len(commands)
    assert all(r['parent_command_id'] == job['job_id'] for r in parent_rows)
    store.stop()
    code = ('import json,sys;from bioxp.operator_command_plane import OperatorCommandStore;'
            's=OperatorCommandStore(sys.argv[1]);print(json.dumps({"workflow":s.get_workflow(sys.argv[2]),"semantic":s.deck_semantic_state()}));s.stop()')
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', code, str(rig.root), job['job_id']], text=True, timeout=25))
    assert reopened['workflow']['command'] == terminal['command']
    assert reopened['semantic'] == semantic
    if os.environ.get('E113_LINEAGE_OUTPUT'):
        Path(os.environ['E113_LINEAGE_OUTPUT'], outcome + '.json').write_text(json.dumps({
            'qualification': 'offline native transport; not physical placement proof',
            'document': payload, 'terminal': terminal, 'commands': commands,
            'children': children, 'publications': publications, 'injected': injected,
            'fresh_process': reopened}, indent=2))
