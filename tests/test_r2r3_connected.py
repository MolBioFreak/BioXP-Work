"""R2/R3 through actual native/provider/camera/store owners; device I/O doubled."""
import asyncio
import json
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from tests.test_cover_carry_release_connected import connected
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_current_owner_v14 import state, scripted
from tests.test_camera_oem_led_binding import led_rig
from tests.test_camera_oem_inspection import CONTROLS, jpeg

_REAL_SPAWN = asyncio.create_subprocess_exec


def export(name, value):
    directory = os.environ.get('R2R3_EXPORT_DIR')
    if directory:
        target = Path(directory)
        target.mkdir(parents=True, exist_ok=True)
        (target / (name + '.json')).write_text(json.dumps(value, indent=2, default=str) + '\n')


def test_protocol_xy_seals_mta_and_records_actual_owner_lineage(connected):
    from bioxp.oem_deck_movement import compile_finite_plate_operation, make_wp8_operation_executor
    r = connected
    plan = compile_finite_plate_operation('pipette_move_xy', source_leaf_available=True, x=34000, y=500)
    r.store.bind_workflow_dispatcher(lambda command: None)
    r.store.admit_workflow(command_id='r2-parent', idempotency_key='r2-parent',
        plan_fingerprint='r2-xy', requested_inputs={'bundle': {'execution': {'runtime_state': {}}}},
        ownership_generation=r.provider.deck_owner_authority_stamps()['ownership_generation'],
        resources=('axis:x', 'axis:y', 'axis:z', 'gripper'), board_epochs={})
    assert r.store.claim_next()['command_id'] == 'r2-parent'
    with r.store.workflow_context('r2-parent', source_occurrence_id='r2-xy'):
        admitted = r.store.admit_internal_wp8_operation('pipette_move_xy', inputs={}, state=state(r.provider),
            idempotency_key='r2-xy', prepared_plan=plan)
    claimed = r.store.claim_next()
    assert claimed['command_id'] == admitted['command_id']
    result = make_wp8_operation_executor(provider_getter=lambda: r.provider, command_store=r.store)(
        command_id=claimed['command_id'], plan=plan)
    assert result['ok'], result
    evidence = r.store.wp8_operation_evidence(claimed['command_id'])
    child = evidence['children'][0]
    native = json.loads(child['terminal_evidence_json'])['result']
    assert native['source_context_sealed'] is True
    assert native['source_context'] == 'ControlLib.MotionThread'
    assert native['wait_schedule'] == 'MTA_WaitAll'
    assert r.native.waits[0][1]['sta_sequential'] is False
    assert r.native.waits[0][1]['timeout_s'] == 5.0
    assert r.native.moves
    attempts = r.store.connection.execute('SELECT * FROM operator_plane_delivery_attempts WHERE command_id=?',
        (claimed['command_id'],)).fetchall()
    assert len(attempts) == 1
    owner = r.provider.deck_owner_authority_stamps()
    for attempt in attempts:
        assert attempt['owner_id'] == r.store.owner_id
        assert attempt['dispatch_attempt_id'] == claimed['dispatch_attempt_id']
        assert attempt['plan_digest'] == plan['plan_digest']
        assert all(attempt[key] == owner[key] for key in owner)
    r.store.finish(claimed['command_id'], status='completed', payload=result, claimed=claimed)
    assert r.store.get_command(claimed['command_id'])['status'] == 'completed'
    export('protocol-xy', dict(claimed=claimed, native=native,
        actual_owner=owner, delivery_attempts=[dict(row) for row in attempts]))


@pytest.mark.parametrize('delivered', [False, True])
@pytest.mark.parametrize('acquire', ['constructor', 'start'])
def test_new_store_recovers_immediately_at_acquire_without_dispatch(connected, delivered, acquire):
    from bioxp.operator_command_plane import OperatorCommandStore
    r = connected
    claimed, plan = scripted(r)
    cid = claimed['command_id']
    r.store.persist_mov_execution_plan(cid, plan)
    if delivered:
        step = plan.steps[0]
        r.store.record_delivery_attempt(cid, work_kind='wp7_stage',
            work_identity=f'stage:{step.order}:{step.operation}', plan_digest=plan.plan_digest)
    # Model abrupt process death by expiring only the lease. Constructing
    # before it expires also covers the start() takeover path.
    new = OperatorCommandStore(r.store.root) if acquire == 'start' else None
    if new is not None:
        assert not new._owner_acquired
        assert new.connection.execute('SELECT status FROM operator_plane_commands WHERE command_id=?', (cid,)).fetchone()[0] == 'dispatched'
    r.store.connection.execute('UPDATE operator_plane_lane SET owner_lease_until=0 WHERE singleton=1')
    original_owner = r.store.owner_id
    dispatched = []
    if new is None:
        new = OperatorCommandStore(r.store.root)
    else:
        new.start(dispatched.append)
    try:
        assert new._owner_acquired and new.owner_id != original_owner
        row = new.get_command(cid)
        # get_command exports uncertain deck results as ambiguous; preserve
        # that existing meaning as well as the internal interrupted state.
        assert row['status'] == ('ambiguous' if delivered else 'failed')
        assert new.connection.execute('SELECT status FROM operator_plane_commands WHERE command_id=?', (cid,)).fetchone()[0] == ('interrupted' if delivered else 'failed')
        terminal = json.loads(new.connection.execute('SELECT terminal_json FROM operator_plane_commands WHERE command_id=?', (cid,)).fetchone()[0])
        assert terminal['reason'] == ('process_owner_loss' if delivered else 'process_owner_loss_before_first_tx')
        assert terminal['outcome_unknown'] is delivered
        assert terminal['delivery_attempted'] is delivered
        assert new.connection.execute('SELECT active_command_id FROM operator_plane_lane').fetchone()[0] is None
        assert new.connection.execute('SELECT COUNT(*) FROM operator_plane_delivery_attempts WHERE command_id=?', (cid,)).fetchone()[0] == int(delivered)
        assert not r.native.moves
        assert not dispatched  # Lost work is never automatically rescheduled.
        if acquire == 'constructor':
            assert new._thread is None  # Closed even before starting dispatch.
        export(f'owner-loss-{acquire}-{delivered}', dict(command_id=cid,
            previous_owner=original_owner, new_owner=new.owner_id,
            terminal=terminal, receipt=row, native_moves=r.native.moves, dispatched=dispatched))
    finally:
        new.stop()
        new.connection.close()
        # The old test object represents the dead process, not a live owner
        # entitled to renew the replacement's lease during fixture teardown.
        r.store._owner_acquired = False


def test_unit_stops_entire_old_process_group():
    unit = (Path(__file__).parents[1] / 'systemd/bioxp-api.service').read_text()
    assert 'TimeoutStopSec=20s' in unit
    assert 'KillMode=control-group' in unit
    assert 'SendSIGKILL=yes' in unit


@pytest.mark.parametrize('entry', ['preparation', 'cover-profile'])
def test_inspection_reaps_real_preview_then_initializes_led_and_captures(led_rig, monkeypatch, tmp_path, entry):
    from bioxp import api
    from bioxp.oem_preparation_runtime import PreparationCameraRuntime
    p, device, _ = led_rig
    processes = []
    monkeypatch.setattr(api, '_camera_provider', p)
    monkeypatch.setattr(api, '_camera_session', None)
    monkeypatch.setattr(api, '_camera_stream_state', {})
    monkeypatch.setattr(api, '_camera_projection_epoch', 0)
    monkeypatch.setattr(api, '_camera_owner_lock', asyncio.Lock())
    monkeypatch.setattr(api.shutil, 'which', lambda _: '/offline/ffmpeg')
    monkeypatch.setattr(api.lifecycle_state, 'record_camera_evidence', lambda _: None)
    async def spawn(*args, **kwargs):
        # Replace only device input with a real idle subprocess and real pipes.
        process = await _REAL_SPAWN(sys.executable, '-c', 'import time; time.sleep(60)', **kwargs)
        processes.append(process)
        return process
    monkeypatch.setattr(api.asyncio, 'create_subprocess_exec', spawn)
    def runner(argv, **kwargs):
        assert processes[0].returncode is not None, 'preview not reaped before capture'
        assert p._stream_owner is None
        assert p._led_initialized
        if argv[0] == 'ffmpeg':
            return SimpleNamespace(returncode=0, stdout=jpeg() * 2, stderr=b'')
        return SimpleNamespace(returncode=0, stdout=CONTROLS.encode(), stderr=b'')
    p._runner = runner
    async def exercise():
        await api._start_owned_camera_session({})
        assert p._stream_owner is not None and processes[0].returncode is None
        runtime = PreparationCameraRuntime(p, artifact_root=tmp_path)
        try:
            if entry == 'preparation':
                await asyncio.to_thread(runtime.initialize_illumination)
                await asyncio.to_thread(runtime.led, channel=1, on=True)
            else:
                from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider
                inspector = object.__new__(Serial206OemInitializationProvider)
                monkeypatch.setattr(api, '_deck_inspection_settings', lambda: {
                    'InspectionSettings': {'r2': {'Exposure': 1000, 'LED1': True,
                        'LED2': False, 'LED3': False}}})
                api._bind_deck_cover_inspection(inspector)
                await asyncio.to_thread(inspector._cover_inspection_profile, 'r2')
            result = await asyncio.to_thread(runtime.snapshot_image, condition='r2', artifact_id='r2')
            assert result['ok'] and Path(result['path']).read_bytes() == jpeg()
            assert api._camera_session is None and processes[0].returncode is not None
            assert len(processes) == 1  # No preview restart/retry.
            assert p.illumination_state()['channels'][0]['on'] is True
            assert len(device.opened) == 1
            export('camera-' + entry, dict(process_pid=processes[0].pid,
                process_returncode=processes[0].returncode, process_count=len(processes),
                stream_owner=p._stream_owner, illumination=p.illumination_state(),
                artifact=result, illumination_ioctls=device.seen))
        finally:
            await api._stop_owned_camera_session(reason='offline cleanup')
            p.close()
    asyncio.run(exercise())
