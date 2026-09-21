"""Terminal WP8 abandonment through real workflow/API owners, offline leaves only."""
import json
import time

import pytest

from tests.protocol_v1_integration_fixture import integrated_rig
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig
from tests.test_protocol_live_prepare_connected import connected_motion_wait


@pytest.fixture(autouse=True)
def recovery_routes(monkeypatch):
    from bioxp import api, operator_controls as controls
    install = controls.install_operator_control_plane
    def mount(app, **kwargs):
        app.add_api_route('/oem/startup/initialize_environment', api.oem_startup_initialize_environment, methods=['POST'])
        app.add_api_route('/motion/oem/home_xy', api.motion_oem_home_xy, methods=['POST'])
        app.add_api_route('/motion/oem/move_xy', api.motion_oem_move_xy, methods=['POST'])
        app.add_api_route('/motion/oem/initialization/initialize_motion', api.motion_oem_serial206_initialize_motion, methods=['POST'])
        return install(app, **kwargs)
    monkeypatch.setattr(controls, 'install_operator_control_plane', mount)


@pytest.fixture
def failed_workflow(integrated_rig, monkeypatch):
    rig = integrated_rig
    rig.store.publish_deck_owner_state(source_operation='updateLocation',
        source_command_id='offline-recovery-predecessor',
        updates={'current_location': 'LOC_STRIP1', 'current_well': 0},
        **rig.provider.deck_owner_authority_stamps())
    def uncertain_move(**kwargs):
        raise RuntimeError('isolated physical Park completion unavailable')
    monkeypatch.setattr(rig.provider.primitives,
        'oem_initialize_motion_scriptmove_to_waste', uncertain_move, raising=False)
    job = rig.start(rig.payload('finite-recovery-abandon'))
    done = rig.terminal(job)
    assert done['command']['status'] == 'ambiguous', done
    ids = [job['job_id'], *[row['command_id'] for row in rig.child_rows(job)]]
    assert rig.store.wait_for_command_workers(ids, timeout=5)
    with rig.store._lock:
        child = rig.store.connection.execute(
            "SELECT c.*,w.operation,w.plan_digest FROM operator_plane_commands c "
            "JOIN operator_plane_wp8_operations w USING(command_id) "
            "JOIN operator_commands a USING(command_id) "
            "WHERE a.parent_command_id=? AND c.status='ambiguous'", (job['job_id'],)).fetchone()
        assert child is not None and child['operation'] == 'thermal_door'
        child_id = child['command_id']
        assert rig.store.connection.execute('SELECT 1 FROM operator_plane_deck_commands WHERE command_id=?', (child_id,)).fetchone() is None
    return rig, job, done, child_id


def snapshot(rig, ids):
    with rig.store._lock:
        return {table: [dict(row) for row in rig.store.connection.execute(
            'SELECT * FROM '+table+' WHERE command_id IN ('+','.join('?' for _ in ids)+') ORDER BY command_id', ids)]
            for table in ('operator_plane_commands', 'operator_commands', 'serial206_movement_commands',
                          'serial206_command_resources', 'operator_plane_wp8_operations')}


def request(rig, key='finite-abandon-ack'):
    recovery = rig.client.get('/operator/recovery').json()
    return '/operator/recovery/'+str(recovery['recovery_epoch'])+'/resolve', {
        'schema_version': 'bioxp.operator_recovery_resolve.v1', 'idempotency_key': key,
        'expected_version': recovery['version'], 'expected_safety_epoch': recovery['global_safety_epoch'],
        'acknowledge_command_ids': recovery['outcome_unknown_command_ids'], 'operation': 'cancel_pending'}


def abandon(rig):
    url, body = request(rig)
    response = rig.client.post(url, json=body)
    assert response.status_code == 200, response.text
    return response.json(), url, body


def test_actual_finite_terminal_abandon_preserves_unknown_and_physical_hold(failed_workflow):
    rig, job, done, child_id = failed_workflow
    before = snapshot(rig, [job['job_id'], child_id])
    semantic, safety = rig.store.deck_semantic_state(), rig.safety()
    trace = list(rig.native.trace)
    response, url, body = abandon(rig)
    assert response['outcome_remains'] == 'unknown' and response['recovery_hold'] is True
    assert response['abandoned_workflow_command_id'] == job['job_id']
    assert rig.store.connection.execute('SELECT workflow_command_id FROM operator_plane_lane').fetchone()[0] is None
    assert snapshot(rig, [job['job_id'], child_id]) == before
    assert rig.store.deck_semantic_state() == semantic and rig.safety() == safety
    assert rig.native.trace == trace
    assert rig.store.deck_recovery_blocker() == 'deck_recovery_hold'
    replay = rig.client.post(url, json=body)
    assert replay.status_code == 200 and replay.json()['idempotent_replay'] is True
    assert replay.json()['transition_sequence'] == response['transition_sequence']
    assert rig.control(job, 'cannot-resume-abandoned', action='continue', gate='pause', gate_id='old').status_code >= 400
    reopened = rig.reopen(done)
    assert reopened['workflow']['command']['status'] == 'ambiguous'
    assert next(row for row in reopened['children'] if row['command_id'] == child_id)['status'] == 'ambiguous'
    # Actual ordinary deck request remains blocked, not only a store helper.
    denied = rig.client.post('/operator/v2/actions/oem.deck.move_to_location', json={
        'schema_version': 'bioxp.operator_action_request.v2', 'idempotency_key':'ordinary-after-abandon',
        'expected_ownership_generation':rig.provider.generation_provider(),
        'expected_board_epoch_by_board':{}, 'inputs':{'location':'LOC_STRIP1'}})
    assert denied.status_code == 409 and 'deck_recovery_hold' in denied.text, denied.text
    assert rig.native.trace == trace


@pytest.mark.parametrize('fault', ['missing_id', 'duplicate_id', 'version', 'safety', 'executor'])
def test_abandon_fences_and_active_executor_refuse_atomically(failed_workflow, fault):
    rig, job, _, child_id = failed_workflow
    before = snapshot(rig, [job['job_id'], child_id])
    url, body = request(rig)
    if fault == 'missing_id': body['acknowledge_command_ids'].remove(child_id)
    if fault == 'duplicate_id': body['acknowledge_command_ids'].append(child_id)
    if fault == 'version': body['expected_version'] += 1
    if fault == 'safety': body['expected_safety_epoch'] += 1
    if fault == 'executor': rig.store.bind_workflow_controls(job['job_id'], lambda *a: None)
    try:
        response = rig.client.post(url, json=body)
        assert response.status_code == 409, response.text
        assert snapshot(rig, [job['job_id'], child_id]) == before
        assert rig.store.connection.execute('SELECT workflow_command_id FROM operator_plane_lane').fetchone()[0] == job['job_id']
        assert rig.store.connection.execute('SELECT COUNT(*) FROM operator_plane_recovery_acknowledgements').fetchone()[0] == 0
    finally:
        if fault == 'executor': rig.store.unbind_workflow_controls(job['job_id'])


def test_existing_source_initializer_after_software_abandonment(failed_workflow, monkeypatch):
    from bioxp import api, operator_controls as controls
    from bioxp.hardware_status import HardwareStateOwner
    rig, job, _, child_id = failed_workflow
    # Restore the real snapshot producer omitted by the deck-only fixture;
    # only controller observations are synthetic, never admission decisions.
    owner = HardwareStateOwner()
    observations = {'axes': {}, 'power': {'safety_valid': True},
        'latch': {'door_sensor': 1, 'latch_sensor': 1},
        'interlock': {'motion_arm': {'armed': True}}}
    owner.collect(list(observations), {name: (lambda context, value=value: value)
        for name, value in observations.items()})
    monkeypatch.setattr(controls.hardware_state, 'project', owner.project)
    monkeypatch.setattr(api, '_maintenance_state', {'motion_blocked': False, 'recovery_required': False})
    from tests.protocol_v1_integration_fixture import NativePhysicalRecorder
    from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
    from bioxp.oem_runtime_store import OEMRuntimeStore
    from bioxp.serial206_y_provider import Serial206YProvider
    cold_native = NativePhysicalRecorder(monkeypatch)
    from bioxp.usb_driver import BioXpTester
    # NativePhysicalRecorder uses __new__. Match these *constructor defaults*
    # from usb_driver.py, not a physical latch observation or retained readiness.
    # The independent hardware latch read below still uses the real source reader.
    cold_native.tester._oem_latch_status = True
    cold_native.tester._oem_latch_status_generation = 0
    cold_native.positions = rig.native.positions  # the same physical fixture, not lifecycle generations
    cold_native.door_open_position = rig.native.door_open_position
    # All motion and wait methods belong to this cold tester. Only physical
    # coordinates survive the owner replacement; no old-driver closures do.
    for name in ('motor_oem_move_absolute', 'motor_wait_target_reached', 'motor_wait_target_reached_many'):
        assert getattr(cold_native.tester, name).__self__ is cold_native.tester
        assert getattr(cold_native.tester, name).__func__ is getattr(BioXpTester, name)
    exchange = cold_native.exchange
    startup_registers = {}
    def stopped_door(board, command, typ, bank, value, **kwargs):
        # Model ordinary SAP/GAP configuration on the same physical axes.
        # Native profile construction/readback and lifecycle publication remain real.
        if (board, bank) in cold_native.positions and command == 5 and typ not in (0, 1, 3, 9):
            cold_native.trace.append((board, command, typ, bank, value))
            startup_registers[board, typ, bank] = value
            return {'status':100, 'value':value, 'command':command, 'module':board}
        if command == 6 and typ not in (1, 3, 9) and (board, typ, bank) in startup_registers:
            cold_native.trace.append((board, command, typ, bank, value))
            return {'status':100, 'value':startup_registers[board, typ, bank], 'command':command, 'module':board}
        if board == 5 and command == 50 and typ == 0 and bank in (0, 1, 2) and value == 1024:
            cold_native.trace.append((board, command, typ, bank, value))
            return {'status':100, 'value':value, 'command':command, 'module':board}
        if board == 7 and command == 9 and typ == 8 and bank in (0, 1) and value == -25:
            cold_native.trace.append((board, command, typ, bank, value))
            return {'status':100, 'value':value, 'command':command, 'module':board}
        # Native absolute/relative moves and completion ingress remain owned
        # by the cold tester. Model the addressed physical controller only.
        if (board, bank) in cold_native.positions and command == 4 and typ in (0, 1):
            from tests.test_motor_receive_identity import receive
            cold_native.trace.append((board, command, typ, bank, value))
            cold_native.positions[board, bank] = value if typ == 0 else cold_native.positions[board, bank] + value
            receive(cold_native.tester, board=board, motor=bank)
            return {'status':100, 'value':value, 'command':command, 'module':board}
        if (board, bank) in ((5, 0), (4, 0)) and command == 2 and typ == 0 and value == 200:
            cold_native.trace.append((board, command, typ, bank, value))
            cold_native.positions[board, bank] = 0
            return {'status':100, 'value':0, 'command':command, 'module':board}
        if (board, bank) in cold_native.positions and (command, typ, value) == (138, 0, 0):
            cold_native.trace.append((board, command, typ, bank, value))
            return {'status': 100, 'value': 0}  # native queryMotorStop physical reply
        if (board, command, typ, bank, value) == (6, 4, 1, 0, 2000):
            from tests.test_motor_receive_identity import receive
            cold_native.trace.append((board, command, typ, bank, value))
            cold_native.positions[board, bank] += value
            receive(cold_native.tester, board=board, motor=bank)
            return {'status': 100, 'value': value}
        return exchange(board, command, typ, bank, value, **kwargs)
    cold_native.tester.send_tmcl = stopped_door
    cold_native.tester.send_tmcl_retry = stopped_door
    cold_native.tester._send_motor = stopped_door
    from bioxp.motion_safety import Serial206MotionAuthority
    adapter = Serial206ProductionPrimitiveAdapter(cold_native.tester, rig.transport,
        authority_provider=Serial206MotionAuthority.from_active_snapshot, generation_provider=rig.provider.generation_provider,
        reference_store=rig.references, pipette_audit_runner=api._run_serial206_pipette_audit)
    runtime = OEMRuntimeStore(rig.root)
    cold = type(rig.provider)(adapter,
        state_store=runtime, reference_store=rig.references,
        generation_provider=rig.provider.generation_provider, preparation_provider=adapter)
    cold.y_provider = Serial206YProvider(cold_native.tester, state_store=runtime,
        reference_store=rig.references, generation_provider=rig.provider.generation_provider)
    adapter.y_provider = cold.y_provider
    adapter.bind_x_lifecycle_executor(cold.execute_x_intent)
    assert adapter.capability_status()['initialize_motion_complete'], adapter.capability_status()
    cold_native.tester.set_board_activation_observer(cold.notify_board_activation)
    assert adapter.current_board_lifecycle_generation() is None
    cold.x_projection()
    cold.z_projection()
    assert rig.references.snapshot(('x',))['rows']['x']['state'] == 'desynced'
    before = snapshot(rig, [job['job_id'], child_id])
    abandon(rig)  # does not require the invalidated reference
    monkeypatch.setattr(api, '_serial206_oem_initialization_provider', cold)
    monkeypatch.setattr(api, '_serial206_y_provider', cold.y_provider)
    monkeypatch.setattr(api, '_get_tester', lambda: cold_native.tester)
    from fastapi import FastAPI
    from fastapi.testclient import TestClient
    from bioxp.oem_compat.position_table import load_bound_oem_position_table
    rig.app.state.operator_command_plane.stop()
    app = FastAPI()
    app.add_api_route('/liquid/tip-status', api.liquid_tip_status, methods=['POST'])
    app.middleware('http')(api.bind_direct_pipette_idempotency)
    controls.install_operator_control_plane(app,
        maintenance_state_provider=lambda: api._maintenance_state,
        reference_state_provider=lambda: rig.references.snapshot(('x', 'y', 'z', 'g')),
        lifecycle_state_provider=lambda: {'operation_state': 'stopped'},
        serial206_initialization_state_provider=api.serial206_oem_initialization_provider_status,
        oem_deck_provider=lambda: cold, oem_deck_position_table_provider=load_bound_oem_position_table)
    monkeypatch.setattr(api, 'app', app)
    action = next(row for row in app.state.operator_command_plane.actions
        if row['informational_path'] == '/motion/oem/initialization/initialize_motion')
    client = TestClient(app)
    client.__enter__()
    try:
        from tests.test_oem_cold_startup_contract import startup
        startup(monkeypatch, cold_native.tester)
        environment = client.post('/oem/startup/initialize_environment', json={
            'mode':'live', 'operator_ack':'INITIALIZE'})
        assert environment.status_code == 200, environment.text
        for name in ("query_only_tmcl", "deck_io_set_type"):
            monkeypatch.setattr(cold_native.tester, name, getattr(BioXpTester, name).__get__(cold_native.tester))
        assert adapter.current_board_lifecycle_generation() is not None
        response = client.post('/operator/actions/'+action['action_id'], json={
            'expected_generation':cold.generation_provider(), 'idempotency_key':'abandoned-initialize-motion',
            'inputs':{'idempotency_key':'abandoned-native-initialize-motion', 'timeout_s':30.0}})
        assert response.status_code == 200, response.text
        assert response.json()['status'] == 'completed', (response.text, runtime.read_oem_serial206_initialization_state()['initialize_motion_ledger'])
        import os
        from pathlib import Path
        Path(os.environ['BIOXP_WORKFLOW_EXPORT']+'.native-home-publication.json').write_text(json.dumps({
            'native_home_receipts': cold._native_initialized_home_receipts(cold._load_state()),
            'references': rig.references.snapshot(('x', 'y', 'z', 'g')),
            'source_state': runtime.read_oem_serial206_initialization_state(),
            'physical_fixture_positions': {str(k): v for k, v in cold_native.positions.items()},
            'native_wire': cold_native.trace,
            'history_unchanged': snapshot(rig, [job['job_id'], child_id]) == before}, indent=2))
        # A complete initialization supplies Home provenance, not a claim that
        # its subsequent source Park left all axes at zero. Ask the unchanged
        # physical consumer first; request explicit HomeXY only for nonzero XY.
        home = None
        home_receipt = None
        native_snapshot_blocker = None
        try:
            home_snapshot = cold.deck_home_reconciliation_snapshot(expected_generation=cold.generation_provider())
        except RuntimeError as exc:
            native_snapshot_blocker = str(exc)
            if native_snapshot_blocker not in {
                'deck_home_current_readback_unverified:x:position',
                'deck_home_current_readback_unverified:y:position'}:
                raise
            home = client.post('/operator/v2/actions/oem.xy.home', json={
                'schema_version':'bioxp.operator_action_request.v2', 'idempotency_key':'cold-after-initialize-home',
                'expected_ownership_generation':cold.generation_provider(),
                'expected_board_epoch_by_board':{}, 'inputs':{}})
            home_receipt = home.json()
            if home.status_code == 200 and home_receipt.get('command_id'):
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    home_receipt = client.get('/operator/v2/actions/receipts/'+home_receipt['command_id']+'?detail=true').json()
                    if home_receipt.get('status') not in {'queued', 'running', 'dispatched', 'issued_pending'}:
                        break
                    time.sleep(.01)
            assert home.status_code == 200 and home_receipt.get('status') == 'completed', home_receipt
        home_snapshot = cold.deck_home_reconciliation_snapshot(expected_generation=cold.generation_provider())
        assert set(home_snapshot['recovery_home_evidence']) == {'x', 'y', 'z'}
        assert runtime.board4_authority_projection()['axes']['y']['prepared_board_epoch'] is None
        current = runtime.read_oem_serial206_initialization_state()
        refs = rig.references.snapshot(('x', 'y', 'z'))
        import os
        from pathlib import Path
        Path(os.environ['BIOXP_WORKFLOW_EXPORT']+'.cold-source-recovery.json').write_text(json.dumps({
            'response': response.json(), 'home_http_status': home.status_code if home is not None else None, 'home_receipt':home_receipt,
            'native_snapshot_blocker_before_explicit_home': native_snapshot_blocker,
            'home_authority_snapshot': home_snapshot,
            'references': refs, 'native_wire': cold_native.trace,
            'source_state': current, 'history_unchanged': snapshot(rig, [job['job_id'], child_id]) == before,
            'hold': app.state.operator_command_plane.store.deck_recovery_blocker()}, indent=2))
        assert home is None or (home.status_code == 200 and home_receipt.get('status') == 'completed'), home_receipt
        assert all(row['state'] == 'referenced' for row in refs['rows'].values()), refs
        assert snapshot(rig, [job['job_id'], child_id]) == before
        assert rig.store.deck_recovery_blocker() == 'deck_recovery_hold'
        # Join the cold native producer to the finite consumer through the real
        # post-failure query/publication and public governed recovery route.
        from tests.test_deck_home_recovery import home_body
        query = client.post('/liquid/tip-status', headers={'Idempotency-Key':'cold-post-failure-no-tip'})
        assert query.status_code == 200, query.text
        assert query.json().get('deck_state_publication', {}).get('status') == 'published', query.text
        resolved = client.post('/operator/recovery/deck/'+child_id+'/reconcile', json=home_body(cold))
        assert resolved.status_code == 200, resolved.text
        assert app.state.operator_command_plane.store.deck_recovery_blocker() is None
        assert snapshot(rig, [job['job_id'], child_id]) == before
        Path(os.environ['BIOXP_WORKFLOW_EXPORT']+'.cold-finite-recovery.json').write_text(json.dumps({
            'query':query.json(), 'home':home_snapshot, 'decision':resolved.json(),
            'history_unchanged':snapshot(rig,[job['job_id'],child_id]) == before}, indent=2))
        # Constructor-default host latch is not an always-true override and
        # is distinct from the independently observed physical latch sensor.
        cold_native.tester._mark_oem_latch_unlocked()
        after_unlock = cold.deck_home_reconciliation_snapshot(expected_generation=cold.generation_provider())
        assert after_unlock['latch_status'] is False
        assert after_unlock['machine_latch_closed'] is True
        assert after_unlock['latch_observation_id'] != home_snapshot['latch_observation_id']
        del cold_native.tester._oem_latch_status
        with pytest.raises(RuntimeError, match='oem_host_latch_status_observation_failed'):
            cold.deck_home_reconciliation_snapshot(expected_generation=cold.generation_provider())
    finally:
        client.__exit__(None, None, None)
        app.state.operator_command_plane.stop()
        app.state.operator_poll_cache.close()
        app.state.operator_admission_state_reader.close()
        app.state.operator_preview_state_reader.close()
        cold_native.close()
        runtime.close()
