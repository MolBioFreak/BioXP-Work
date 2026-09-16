"""Real-factory lifecycle fixture; synthetic inputs, never live readiness.

No binding, provider handler, finite child, dispatcher or store is replaced.
The execute observer delegates unchanged and exposes the *active* executor for
source-event injection (the source board-error event has no public HTTP route).
Physical recorders inherited from query_rig/ready are explicit offline leaves.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
from typing import Any


class NativePhysicalRecorder:
    """Native tester state and source methods; only board exchange is synthetic.

    Never replace forceAbortMotion, thermal wait/tick, initialCheck, generation
    minting, initialize_motors, the canonical adapter or lifecycle hooks.
    """
    door_open_position: int
    positions: dict[tuple[int, int], int]

    def __init__(self, monkeypatch):
        from bioxp.usb_driver import BioXpTester
        self.tester = object.__new__(BioXpTester)
        self.tester._oem_board_initialized = {b: True for b in (4, 5, 6, 7)}
        self.tester._oem_transport_generation = 3
        self.tester._oem_abort_generation = 0
        self.tester._oem_24v_dropped = False
        self.tester._oem_user_stopped = False
        self.trace, self.replies, self.timers = [], {}, []
        self.axis_parameters = {}
        self.thermal_wait = threading.Event()
        self.tester.send_tmcl_retry = self.exchange
        self.tester._send_motor = self.exchange
        # Keep real native timer processing and time. Timer objects are tracked
        # so teardown cannot leak native callbacks into the next SQLite owner.
        self.tester._oem_thermal_schedule = self.schedule

    def exchange(self, board, command, typ, bank, value, **kwargs):
        key = (board, command, typ, bank)
        self.trace.append((*key, value))
        if key in self.replies:
            result = self.replies[key]
            return result() if callable(result) else result
        # Closed native thermal wire contract (usb_driver._oem_thermal_tx),
        # not a blanket successful controller. Mechanical/wake additions must
        # be evidenced by a delivered source command before joining this set.
        thermal = (board == 6 and bank in (0, 1) and
            ((command == 10 and typ in (4, 7, 8, 23)) or
             (command == 9 and typ in (7, 8)) or
             (command in (140, 144) and typ == 0)))
        chiller = (board == 7 and
            ((command == 143 and typ == 0 and bank in (0, 3)) or
             (command == 10 and typ in (7, 8) and bank in (0, 1)) or
             (command in (140, 144) and typ == 0 and bank in (0, 1))))
        gripper_home_switch = key == (4, 6, 9, 2)
        motor_readback = command == 6 and typ in (1, 3) and (board, bank) in self.positions
        if motor_readback:
            return {'status': 100, 'value': self.positions[board, bank] if typ == 1 else 0}
        if key == (5, 14, 2, 0) and value in (0, 1):
            return {'status': 100, 'value': value}
        if key == (4, 3, 0, 2) and value == 0:
            return {'status': 100, 'value': 0}
        if key == (4, 2, 0, 2) and value == 200:
            # Synthetic instantaneous arrival at G's mechanical stop.
            self.positions[4, 2] = 0
            return {'status': 100, 'value': 0}
        axis_parameters = command in (5, 6) and (board, bank, typ) in {
            (6, 0, 6), (6, 0, 205), (4, 2, 1), (4, 2, 4), (4, 2, 6), (4, 2, 205), (4, 0, 6)}
        door_sensors = board == 6 and bank == 0 and command == 6 and typ in (9, 10, 12, 13)
        assert thermal or chiller or gripper_home_switch or axis_parameters or door_sensors, f'unrecorded native physical command: {(*key, value)}'
        if gripper_home_switch:
            return {'status': 100, 'value': int(self.tester.motor_get_position(4, motor=2)['position'] == 0)}
        if door_sensors:
            position = self.tester.motor_get_position(6, motor=0)['position']
            active = position == (0 if typ == 9 else self.door_open_position)
            return {'status': 100, 'value': int(active) if typ in (9, 10) else 1}
        # Synthetic controller register storage for actual SAP/GAP round trips.
        if axis_parameters:
            parameter = (board, typ, bank)
            if command == 5:
                self.axis_parameters[parameter] = value
                if typ == 1:
                    self.positions[board, bank] = value
            return {'status': 100, 'value': self.axis_parameters.get(parameter, 0)}
        # Synthetic measured temperature/PWM and rate replies, not live proof.
        scalar = 10 if typ == 23 else 25000 if command == 143 or (command == 10 and typ == 4) else 1000 if command == 10 else 0
        return {'status': 100, 'value': scalar}

    def schedule(self, callback, delay=1.0):
        self.thermal_wait.set()
        timer = threading.Timer(delay, callback)
        timer.daemon = True
        self.timers.append(timer)
        timer.start()
        return timer

    def close(self):
        for timer in self.timers:
            timer.cancel()
        for timer in self.timers:
            timer.join(timeout=2)


class PrimitiveComposition:
    """Retained motor/readback leaves plus the real production adapter."""
    def __init__(self, observations, adapter):
        self.observations, self.adapter = observations, adapter
        self.tester = adapter.tester

    def __getattr__(self, name):
        # Existing fixture's oem_move_to delegates to the actual XY adapter.
        if hasattr(self.observations, name):
            return getattr(self.observations, name)
        return getattr(self.adapter, name)
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
    native: Any = None
    references: Any = None
    raw_moves: Any = None

    def safety(self):
        return {
            'no24v': self.native.tester.oem_no24v_state(),
            'references': self.references.snapshot(('x', 'y', 'z', 'g')),
            'epochs': self.provider.deck_owner_authority_stamps(),
        }

    def child_rows(self, job):
        with self.store._lock:
            rows = self.store.connection.execute(
                "SELECT * FROM operator_commands WHERE parent_command_id=? "
                "AND command_kind<>'protocol_control' ORDER BY sequence", (job['job_id'],)).fetchall()
        return [dict(row) for row in rows]

    def native_results(self, job, operation):
        return [json.loads(row['receipt_json']) for row in self.child_rows(job)
                if row['action_id'] == 'protocol.oem_lifecycle.' + operation]

    def control_chain(self, job):
        return [row['action_id'].removeprefix('protocol.oem_lifecycle.')
                for row in self.child_rows(job)
                if row['action_id'].startswith('protocol.oem_lifecycle.')]

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
        code = ('import json,sys;from tests.protocol_v1_integration_fixture import persisted;'
                'print(json.dumps(persisted(sys.argv[1],sys.argv[2])))')
        return json.loads(subprocess.check_output([sys.executable, '-c', code,
            str(self.root), job['job_id']], text=True, timeout=25))


def persisted(root, job_id):
    from bioxp.operator_command_plane import OperatorCommandStore
    store = OperatorCommandStore(root)
    try:
        workflow = store.get_workflow(job_id)
        children = [dict(row) for row in store.connection.execute(
            "SELECT * FROM operator_commands WHERE parent_command_id=? "
            "AND command_kind<>'protocol_control' ORDER BY sequence", (job_id,))]
        return {'workflow': workflow, 'children': children}
    finally:
        store.stop()


@pytest.fixture
def integrated_rig(query_rig, retained_rig, monkeypatch, tmp_path, request):
    from bioxp import api
    from bioxp.protocols.executor import ProtocolExecutor
    from tests.test_deck_complete_admission import ready
    app, provider, primitive, references, root, _, calls, wire, transport = query_rig
    motor_leaf, raw_moves = ready((app, provider, primitive, references, root), monkeypatch, retained_rig)
    native = NativePhysicalRecorder(monkeypatch)
    native.door_open_position = provider._wp8_door_config()['open']
    # Mechanical motion/readback leaves only. Do not copy the recorder's fixed
    # oem_no24v_state or board-state methods over real native safety owners.
    for name in ('motor_oem_move_absolute', 'motor_wait_target_reached',
                 'motor_wait_target_reached_many', 'begin_bus_event_window',
                 'collect_bus_events'):
        setattr(native.tester, name, getattr(motor_leaf, name))
    motor_leaf.positions.update({(5, 1): 0, (6, 0): 0, (4, 2): 0})
    native.positions = motor_leaf.positions
    from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
    adapter = Serial206ProductionPrimitiveAdapter(native.tester, None,
        authority_provider=lambda: {}, generation_provider=provider.generation_provider,
        reference_store=references)
    adapter.y_provider = provider.y_provider
    monkeypatch.setattr(api, '_get_tester', lambda: native.tester)
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
    pipette_errors = []
    from bioxp.pipette.receipts import PipetteReceiptError
    real_error_init = PipetteReceiptError.__init__
    def observe_receipt_error(error, *args, **kwargs):
        import traceback
        pipette_errors.append(''.join(traceback.format_stack()) + repr(args))
        real_error_init(error, *args, **kwargs)
    monkeypatch.setattr(PipetteReceiptError, '__init__', observe_receipt_error)
    monkeypatch.setattr(provider.primitives, 'pipette_audit_runner', api._run_serial206_pipette_audit, raising=False)
    monkeypatch.setenv('BIOXP_PROTOCOL_JOBS_ROOT', str(tmp_path / 'artifacts'))
    mount_protocol_routes(app)
    app.add_api_route('/protocol/jobs/{job_id}/control', api.protocol_job_control, methods=['POST'])
    rig = IntegratedRig(app, provider, TestClient(app), store, root, transport, wire, calls)
    rig.native, rig.references, rig.raw_moves = native, references, raw_moves
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
    monkeypatch.setattr(native.tester, 'strip_set_rgb', rgb)
    # Only I/O leaves, never execute_wp8_child or lifecycle/native callbacks.
    monkeypatch.setattr(primitive, 'deck_io_query_type', lambda kind: {'value': 0}, raising=False)
    monkeypatch.setattr(primitive, 'deck_io_set_type', lambda *a: {
        'ok': True, 'controller_command_acknowledged': True}, raising=False)
    native_generation = provider._load_state()['x_lifecycle']['board_lifecycle_generation']
    native.tester._oem_board_lifecycle_generation = native_generation
    native.tester._oem_active_board_lifecycle_generation = native_generation
    native.tester.set_board_activation_observer(provider.notify_board_activation)
    app.state.oem_workflow_initial_check = api._protocol_workflow_initial_check
    monkeypatch.setattr(provider, 'primitives', PrimitiveComposition(primitive, adapter))
    calls.clear()
    try:
        yield rig
    finally:
        # Preserve pre-teardown evidence even for a failed assertion. Teardown's
        # release is explicitly not part of a successful scenario transcript.
        export = os.environ.get('BIOXP_WORKFLOW_EXPORT')
        if export:
            jobs = [store.get_workflow(job_id) for job_id in rig.executors]
            with store._lock:
                pipette_rows = [dict(row) for row in store.connection.execute(
                    "SELECT * FROM pipette_operations ORDER BY rowid")]
            Path(export + '.' + request.node.name + '.json').write_text(json.dumps({
                'nodeid': request.node.nodeid, 'phase': 'before_teardown',
                'jobs': jobs, 'children': [rig.child_rows(job) for job in jobs],
                'pipette_errors': pipette_errors, 'pipette_rows': pipette_rows,
                'pipette_identity': transport.collection_source_identity(),
                'thermal_wire': native.trace, 'source_rgb': rig.trace,
                'no24v': native.tester.oem_no24v_state(),
                'references': references.snapshot(('x', 'y', 'z', 'g')),
                'raw_moves': rig.raw_moves,
            }, indent=2))
        if rig.body_gate is not None:
            rig.body_gate.release.set()
        # Source event releases test-owned gates without faking parent settlement.
        for job_id, executor in rig.executors.items():
            row = store.get_workflow(job_id)
            if not row['command']['terminal']:
                executor.source_error(false_abort=True)
        app.state.operator_command_plane.stop()
        native.close()
