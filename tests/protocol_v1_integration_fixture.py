"""Real-factory lifecycle fixture; synthetic inputs, never live readiness.

No binding, provider handler, finite child, dispatcher or store is replaced.
The execute observer delegates unchanged and exposes the *active* executor for
source-event injection (the source board-error event has no public HTTP route).
Physical recorders inherited from query_rig/ready are explicit offline leaves.
"""
import json
import subprocess
import sys
import threading
from typing import Any
from dataclasses import dataclass, field


@dataclass
class PhysicalLeafGate:
    """Bounded leaf overlap; always released by the owning fixture teardown."""
    entered: Any = field(default_factory=threading.Event)
    release: Any = field(default_factory=threading.Event)

    def wait(self):
        self.entered.set()
        assert self.release.wait(12), 'test-owned physical leaf was not released'

import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained, qualify_full_predecessor
from tests.test_deck_tip_query_publication import query_rig
from tests.test_protocol_workflow_connected import mount_protocol_routes, await_job, request_payload


@dataclass
class IntegratedRig:
    app: Any
    provider: Any
    client: Any
    store: Any
    root: Any
    transport: Any
    wire: dict
    query_calls: list
    trace: list = field(default_factory=list)
    executors: dict = field(default_factory=dict)
    body_gate: Any = None

    def payload(self, key, *, delayed=False, opcode='led', arguments=None):
        actions = []
        if delayed:
            actions.append({'action_id': 'delay', 'kind': 'oem_operation',
                'oem_opcode': 'delaypoint', 'source_occurrence_id': 'source:delay',
                'params': {'arguments': []}})
        actions.append({'action_id': 'body', 'kind': 'oem_operation',
            'oem_opcode': opcode, 'source_occurrence_id': 'source:body',
            'params': {'arguments': ['0','0','0'] if arguments is None else arguments}})
        payload = request_payload(key, actions)
        payload['document']['metadata'] = {
            'input_mode': 'oem_prepared', 'delayed_start': delayed, 'oem_prepare': False,
            'source_settings': {'JobName': None, 'LogPressure': False,
                'CameraInstalled': False, 'CameraCalibrated': False,
                'CheckSnapTips': False, 'OverPressChecked': None, 'StartMode': 0},
            'source_model': {'logical_tip_present': False, 'carried_plate_present': False,
                'allow_to_stop': True, 'tip_trays': [
                    {'tray_id': str(i), 'location': 7+i, 'wells': [
                        {'content': None, 'volume': 0, 'capacity': 0, 'empty': True}
                        for _ in range(96)]} for i in range(4)]}}
        return payload

    def submit(self, payload):
        return self.client.post('/protocol/execute', json=payload)

    def start(self, payload):
        response = self.submit(payload)
        assert response.status_code == 202, response.text
        self.app.state.operator_command_plane.start()
        return response.json()

    def wait(self, job, predicate):
        return await_job(self.client, job['job_id'], predicate, timeout=12)

    def terminal(self, job):
        return self.wait(job, lambda row: row['command']['terminal'])

    def gate(self, job, name):
        return self.wait(job, lambda row: row['execution']['runtime_state']['workflow']['gate'] == name)

    def control(self, job, key, **fields):
        return self.client.post('/protocol/jobs/' + job['job_id'] + '/control', json={
            'command_id': job['job_id'], 'expected_ownership_generation': job['command']['ownership_generation'],
            'idempotency_key': key, **fields})

    def children(self, job):
        return [dict(row) for row in self.store.connection.execute(
            'SELECT command_id,status,action_id FROM operator_commands WHERE parent_command_id=? ORDER BY sequence',
            (job['job_id'],)).fetchall()]

    def reopen(self, job):
        code = ('import json,sys;from bioxp.operator_command_plane import OperatorCommandStore;'
                's=OperatorCommandStore(sys.argv[1]);print(json.dumps(s.get_workflow(sys.argv[2])));s.stop()')
        return json.loads(subprocess.check_output([sys.executable, '-c', code,
            str(self.root), job['job_id']], text=True, timeout=25))


@pytest.fixture
def integrated_rig(query_rig, retained_rig, monkeypatch, tmp_path):
    from bioxp import api
    from bioxp.protocols.executor import ProtocolExecutor
    from tests.test_deck_complete_admission import ready
    app, provider, primitive, references, root, _, calls, wire, transport = query_rig
    ready((app, provider, primitive, references, root), monkeypatch, retained_rig)
    store = app.state.operator_command_plane.store
    primitive.calls.clear()
    machine = provider._load_state()
    machine['machine_status']['GripperVersion'] = 1
    provider._save_state(machine)  # Synthetic configuration, not observed hardware.
    qualify_full_predecessor((provider, primitive, retained_rig[2], references, store, root))
    stamps = provider.deck_owner_authority_stamps()
    provider.refresh_deck_semantic_bootstrap(expected_generation=stamps['ownership_generation'])
    for tray in range(1, 4):
        store.publish_tip_tray_transition(tray_id=tray, transition='construct',
            operation_id=f'integration-fixture-{tray}', command_id=f'integration-fixture-{tray}',
            provenance={'synthetic_test_inventory': True}, **stamps)
    monkeypatch.setattr(provider.primitives, 'pipette_transport', transport, raising=False)
    monkeypatch.setattr(provider.primitives, 'pipette_audit_runner', api._run_serial206_pipette_audit, raising=False)
    monkeypatch.setenv('BIOXP_PROTOCOL_JOBS_ROOT', str(tmp_path / 'artifacts'))
    mount_protocol_routes(app)
    app.add_api_route('/protocol/jobs/{job_id}/control', api.protocol_job_control, methods=['POST'])
    rig = IntegratedRig(app, provider, TestClient(app), store, root, transport, wire, calls)
    original = ProtocolExecutor.execute
    def observe(executor, document, *args, **kwargs):
        rig.executors[executor.job_id] = executor
        return original(executor, document, *args, **kwargs)
    monkeypatch.setattr(ProtocolExecutor, 'execute', observe)
    def rgb(*args, **kwargs):
        rig.trace.append(('rgb', args))
        if args == (0, 0, 0) and rig.body_gate is not None:
            rig.body_gate.wait()
        return {'ok': True, 'controller_command_acknowledged': True}
    monkeypatch.setattr(primitive, 'strip_set_rgb', rgb, raising=False)
    # Only I/O leaves, never execute_wp8_child or lifecycle/native callbacks.
    monkeypatch.setattr(primitive, 'deck_io_query_type', lambda kind: {'value': 0}, raising=False)
    monkeypatch.setattr(primitive, 'deck_io_set_type', lambda *a: {
        'ok': True, 'controller_command_acknowledged': True}, raising=False)
    calls.clear()
    try:
        yield rig
    finally:
        if rig.body_gate is not None:
            rig.body_gate.release.set()
        # Source event releases test-owned gates without faking parent settlement.
        for job_id, executor in rig.executors.items():
            row = store.get_workflow(job_id)
            if not row['command']['terminal']:
                executor.source_error(false_abort=True)
        app.state.operator_command_plane.stop()
