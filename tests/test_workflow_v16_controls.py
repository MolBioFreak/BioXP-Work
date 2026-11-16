"""V16 control/recovery acceptance on real private stores, no hardware."""
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys
import threading
import time

import pytest
from fastapi.testclient import TestClient
from tests.deck_scoped_rig import retained_rig
from tests.deck_scoped_installed import installed_retained
from tests.test_protocol_workflow_connected import request_payload, mount_protocol_routes, await_job


def history_count(store, job):
    return store.connection.execute('SELECT count(*) FROM operator_plane_command_versions WHERE command_id=?', (job,)).fetchone()[0]


def control_body(job, generation, action, **kwargs):
    return dict(command_id=job, expected_ownership_generation=generation,
        idempotency_key='control-' + action, action=action, **kwargs)


@pytest.mark.parametrize('finish', ['continue', 'abort'])
def test_v16_explicit_pause_then_control_and_cold_read(installed_retained, monkeypatch, tmp_path, finish):
    from bioxp import api
    from bioxp.protocols.models import ProtocolActionKind
    app, _, _, _, root = installed_retained
    store = app.state.operator_command_plane.store
    assert store.connection.execute('PRAGMA user_version').fetchone()[0] == 16
    entered, release = threading.Event(), threading.Event()
    calls = []
    def leaf(action, state):
        calls.append(action.action_id)
        if action.action_id == 'first':
            entered.set()
            assert release.wait(15)
        return {'ok':True, 'fixture_only':True}
    monkeypatch.setenv('BIOXP_PROTOCOL_JOBS_ROOT', str(tmp_path / 'artifacts'))
    monkeypatch.setattr(api, '_protocol_live_handlers', lambda:{ProtocolActionKind.LED:leaf})
    monkeypatch.setattr(api, '_protocol_mechanism_handlers', lambda *a, **k:{ProtocolActionKind.LED:leaf})
    mount_protocol_routes(app)
    app.add_api_route('/protocol/jobs/{job_id}/control', api.protocol_job_control, methods=['POST'])
    client = TestClient(app)
    response = client.post('/protocol/execute', json=request_payload('v16-'+finish))
    assert response.status_code == 202, response.text
    job = response.json()['job_id']
    generation = response.json()['command']['ownership_generation']
    app.state.operator_command_plane.start()
    try:
        assert entered.wait(8)
        before = history_count(store, job)
        pause = client.post('/protocol/jobs/'+job+'/control', json=control_body(job, generation, 'pause', mode='ordinary'))
        assert pause.status_code == 200 and pause.json()['accepted'], pause.text
    finally:
        release.set()
    paused = await_job(client, job, lambda row:row['execution']['runtime_state']['workflow']['gate']=='ordinary_pause')
    assert calls == ['first']
    assert history_count(store, job) == before  # receipt-only boundaries are not full report snapshots
    wf = paused['execution']['runtime_state']['workflow']
    assert wf['reached_control_id'] == pause.json()['control_command_id']
    # Independent connection sees the current receipt, not the older report version.
    cold = sqlite3.connect(root / 'bioxp_runtime.db')
    try:
        receipt = json.loads(cold.execute('SELECT receipt_json FROM operator_commands WHERE command_id=?', (job,)).fetchone()[0])
        assert receipt['execution']['runtime_state'] == paused['execution']['runtime_state']
    finally:
        cold.close()
    extra = {'gate':'ordinary_pause','gate_id':wf['gate_id']} if finish == 'continue' else {}
    request = control_body(job, generation, finish, **extra)
    response = client.post('/protocol/jobs/'+job+'/control', json=request)
    assert response.status_code == 200 and response.json()['accepted'], response.text
    done = await_job(client, job, lambda row:row['command']['terminal'])
    assert done['command']['status'] == ('completed' if finish == 'continue' else 'interrupted'), done
    assert calls == (['first','second'] if finish == 'continue' else ['first'])
    assert history_count(store, job) > before
    replay = client.post('/protocol/jobs/'+job+'/control', json=request)
    assert replay.status_code == 200 and replay.json() == response.json()
    app.state.operator_command_plane.stop()
    code = "import sys,json;from bioxp.operator_command_plane import OperatorCommandStore;s=OperatorCommandStore(sys.argv[1]);print(json.dumps(s.get_workflow(sys.argv[2])));s.stop()"
    reopened = json.loads(subprocess.check_output([sys.executable,'-c',code,str(root),job], text=True, timeout=25))
    assert reopened['command'] == done['command']
    assert reopened['execution']['runtime_state'] == done['execution']['runtime_state']


def crash_worker(root, mode):
    """Separate process; parent SIGKILLs it, so stop/finally cannot settle work."""
    from types import SimpleNamespace
    from bioxp.operator_command_plane import OperatorCommandStore
    from bioxp.protocols.models import ProtocolDocument, ProtocolActionKind
    from bioxp.protocols.runtime_state import ProtocolRuntimeState
    from bioxp.services.protocol_service import bind_protocol_dispatcher, control_protocol_job
    root = Path(root)
    store = OperatorCommandStore(root)
    assert store.connection.execute('PRAGMA user_version').fetchone()[0] == 16
    entered, release = threading.Event(), threading.Event()
    def leaf(action, state):
        with (root/'native-entry.log').open('a') as stream:
            stream.write(action.action_id+'\n'); stream.flush(); os.fsync(stream.fileno())
        entered.set()
        release.wait(60)
        return {'ok':True}
    payload = request_payload('crash')['document']
    for stage in payload['stages']:
        for action in stage['actions']:
            action['stage_id'] = stage['stage_id']
    doc = ProtocolDocument.from_payload(payload)
    state = ProtocolRuntimeState.from_document(doc, dry_run=False, job_id='crash-parent')
    bundle = {'protocol':{'document':doc.to_payload()}, 'execution':{'runtime_state':state.to_payload()}, 'operator':{'reviews':[]}}
    bind_protocol_dispatcher(store, binding_factory=lambda *a, **k:({ProtocolActionKind.LED:leaf},{},{}),
        artifact_store=SimpleNamespace(save=lambda bundle:None))
    store.admit_workflow(command_id='crash-parent', idempotency_key='crash-parent', plan_fingerprint='crash',
        requested_inputs={'bundle':bundle}, ownership_generation=7, resources=(), board_epochs={})
    store.start(lambda c: (_ for _ in ()).throw(AssertionError('unexpected native dispatch')))
    assert entered.wait(10)
    if mode == 'paused':
        control_protocol_job('crash-parent', control_body('crash-parent',7,'pause',mode='ordinary'), command_store=store)
        release.set()
        deadline = time.monotonic()+10
        while store.get_workflow('crash-parent')['execution']['runtime_state']['workflow']['gate'] != 'ordinary_pause':
            assert time.monotonic() < deadline
            time.sleep(.01)
    (root/'kill-ready').write_text(mode)
    threading.Event().wait(60)
    raise AssertionError('parent failed to kill worker')


@pytest.mark.parametrize('mode', ['during_action', 'paused'])
def test_v16_sigkill_reopen_retains_unknown_and_refuses_replay(installed_retained, mode):
    from bioxp.operator_command_plane import OperatorCommandStore
    from bioxp.services.protocol_service import control_protocol_job
    app, _, _, _, root = installed_retained
    app.state.operator_command_plane.stop()
    code = ('import sys,os;import tests.z_stop_offline_guard;'
        'os.environ["BIOXP_OEM_RUNTIME_STATE_ROOT"]=sys.argv[1];'
        'from tests.test_workflow_v16_controls import crash_worker;crash_worker(sys.argv[1],sys.argv[2])')
    with (root/'crash-worker.log').open('w') as log:
        child = subprocess.Popen([sys.executable,'-c',code,str(root),mode], stdout=log, stderr=log)
        try:
            deadline = time.monotonic()+20
            while not (root/'kill-ready').exists():
                assert child.poll() is None, (root/'crash-worker.log').read_text()
                assert time.monotonic()<deadline, (root/'crash-worker.log').read_text()
                time.sleep(.02)
            conn = sqlite3.connect(root/'bioxp_runtime.db')
            before = json.loads(conn.execute("SELECT receipt_json FROM operator_commands WHERE command_id='crash-parent'").fetchone()[0])
            conn.close()
            child.kill()
            assert child.wait(timeout=5) == -9
        finally:
            if child.poll() is None:
                child.kill(); child.wait(timeout=5)
    # Let the actual dead owner's lease expire; never edit a lease/recovery row.
    conn = sqlite3.connect(root/'bioxp_runtime.db')
    lease = conn.execute('SELECT owner_lease_until FROM operator_plane_lane').fetchone()[0]
    conn.close()
    time.sleep(max(0, lease-time.time())+.05)
    store = OperatorCommandStore(root)
    try:
        assert store._owner_acquired
        recovered = store.get_workflow('crash-parent')
        assert recovered['command']['status'] == 'ambiguous', recovered
        wf = recovered['execution']['runtime_state']['workflow']
        assert wf['phase'] == 'reconciling' and wf['held_reason'] == 'workflow_owner_lost'
        assert recovered['execution']['runtime_state']['action_results'] == before['execution']['runtime_state']['action_results']
        assert recovered['protocol'] == before['protocol']
        for action, extra in [('continue',{'gate':'ordinary_pause','gate_id':'wrong'}), ('abort',{})]:
            with pytest.raises(ValueError, match='workflow_authority_lost'):
                control_protocol_job('crash-parent', control_body('crash-parent',7,action,**extra), command_store=store)
        if mode == 'paused':
            replay = control_protocol_job('crash-parent', control_body('crash-parent',7,'pause',mode='ordinary'), command_store=store)
            assert replay['accepted'] is True  # retained acceptance, not a new executor
        assert store.claim_next() is None
        assert (root/'native-entry.log').read_text().splitlines() == ['first']
    finally:
        store.stop()
    cold = OperatorCommandStore(root)
    try:
        assert cold.get_workflow('crash-parent')['command']['status'] == 'ambiguous'
        assert (root/'native-entry.log').read_text().splitlines() == ['first']
    finally:
        cold.stop()
