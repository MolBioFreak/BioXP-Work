"""Qualify source native initialization without constructor setup.

Driver/adapter/provider/SQLite are real. Only TMCL wire replies are doubled.
No profile flags, preparation receipts, home results or lifecycle rows are seeded.
"""
import json
import os
from pathlib import Path
import pytest

from bioxp.usb_driver import BioXpTester
from bioxp.oem_serial206_initialization import (
    Serial206OemInitializationProvider, Serial206ProductionPrimitiveAdapter,
)
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.services.reference_service import ReferenceStateStore
from bioxp.motion_safety import Serial206MotionAuthority


@pytest.fixture
def rig(tmp_path, monkeypatch, request):
    from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot
    snapshot = bind_serial206_oem_snapshot(monkeypatch)
    from bioxp import oem_machine_bundle
    monkeypatch.setattr(oem_machine_bundle, '_active_snapshot',
        oem_machine_bundle.load_oem_machine_snapshot(
            snapshot.bundle_root / 'OEM_EVIDENCE_LOCK.json',
            operator_label_serial=206, require_operator_label=True))
    # Avoid transport construction, retaining real lazy native controller state.
    driver = BioXpTester.__new__(BioXpTester)
    from bioxp.novo_router import NovoRouter
    from bioxp.usb_driver import novo_decode
    driver.novo_router = NovoRouter(ep_in=object(), ep_out=object(), decode=novo_decode)
    driver._motor_noresp_streak = {}
    driver._chiller_last_tx_ts = 0.0
    driver._chiller_noresp_streak = 0
    frames, registers = [], {}
    switches = {}
    fault = {'reject': None, 'on_frame': None, 'home_missing': False}

    def wire(board, command, typ, motor, value, **kwargs):
        frames.append((board, command, typ, motor, value))
        if fault['on_frame'] is not None:
            fault['on_frame'](board, command, typ, motor, value)
        status = 1 if fault['reject'] == (board, command, typ, motor) else 100
        if status == 100 and command == 5:
            registers[board, motor, typ] = value
        if command == 4:
            from tests.test_motor_receive_identity import receive
            receive(driver, board=board, motor=motor)
        result = registers.get((board, motor, typ), 0) if command == 6 else value
        if command == 6 and typ == 9:
            key = (board, motor)
            result = 0 if switches.get(key, 0) == 0 or fault['home_missing'] else 1
            switches[key] = switches.get(key, 0) + 1
        return {'status': status, 'value': result,
                'command': command, 'module': board}

    monkeypatch.setattr(driver, 'send_tmcl', wire)
    monkeypatch.setattr(driver, 'send_tmcl_retry', wire)
    store = OEMRuntimeStore(tmp_path)
    refs = ReferenceStateStore(tmp_path / 'bioxp_runtime.db')
    adapter = Serial206ProductionPrimitiveAdapter(driver, None,
        authority_provider=Serial206MotionAuthority.from_active_snapshot,
        generation_provider=lambda: 7, reference_store=refs)
    provider = Serial206OemInitializationProvider(adapter,
        state_store=store, reference_store=refs,
        generation_provider=lambda: 7)
    driver._board_activation_observer = provider.notify_board_activation
    yield driver, provider, frames, fault, tmp_path
    output = os.environ.get('BIOXP_QUEUED_MOVE_EXPORT')
    if output:
        with Path(output).open('a') as stream:
            stream.write(json.dumps({'nodeid': request.node.nodeid, 'wire': frames,
                'native_profile_generations': getattr(driver, '_oem_no_motion_profile_generations', {}),
                'state': store.read_oem_serial206_initialization_state(),
                'references': refs.snapshot(('x','y','z','g'))}) + '\n')


def cycle(driver):
    off = driver.deactivate_boards()
    on = driver.activate_boards()
    result = driver.oem_begin_board_lifecycle_generation(deactivation=off, activation=on)
    assert result['ok'] is True
    return result


def test_cycle_then_native_homes_publish_without_setup(rig):
    driver, provider, frames, fault, root = rig
    cycle(driver)
    from bioxp.oem_serial206_initialization import load_oem_parity_config
    parity = load_oem_parity_config(None)
    assert not parity.blockers, (parity.blockers, provider.primitives.capability_status(), driver._machine_config_bundle())
    result = provider.initialize_motors(mode='live')
    assert result['ok'] is True
    assert all(v['published'] for v in result['reference_publications'].values()), json.dumps(result['reference_publications'])
    assert set(result['reference_publications']) == {'x', 'y', 'z', 'g'}
    assert all(row['state'] == 'referenced' for row in
        ReferenceStateStore(root / 'bioxp_runtime.db').snapshot(('x','y','z','g'))['rows'].values())
    state = OEMRuntimeStore(root).read_oem_serial206_initialization_state()
    assert state['x_lifecycle']['prepared_receipt'] is None
    assert state['z_lifecycle']['prepared_receipt'] is None
    assert state['preparation']['state'] == 'not_started'
    assert driver._oem_no_motion_profiles_ready == set()
    assert all(row['prepared_board_epoch'] is None for row in
        provider.state_store.board4_authority_projection()['axes'].values())


def test_current_reference_survives_fresh_process_without_setup(rig):
    import subprocess
    import sys
    driver, provider, frames, fault, root = rig
    cycle(driver)
    result = provider.initialize_motors(mode='live')
    assert all(v['published'] for v in result['reference_publications'].values())
    code = """import json,sys
from pathlib import Path
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.services.reference_service import ReferenceStateStore
root=Path(sys.argv[1])
s=OEMRuntimeStore(root).read_oem_serial206_initialization_state()
r=ReferenceStateStore(root/'bioxp_runtime.db').snapshot(('x','y','z','g'))
assert all(v['state']=='referenced' for v in r['rows'].values())
assert s['preparation']['state']=='not_started'
assert s['x_lifecycle']['prepared_receipt'] is None
assert s['x_lifecycle']['state']=='referenced_ready'
print(json.dumps({'references':r,'x':s['x_lifecycle'],'z':s['z_lifecycle']}))
"""
    completed = subprocess.run([sys.executable, '-c', code, str(root)], text=True, capture_output=True)
    assert completed.returncode == 0, completed.stderr
    assert json.loads(completed.stdout)['references']['durable_clean'] is True


def test_setup_then_cycle_does_not_clone_preparation(rig):
    driver, provider, frames, fault, root = rig
    cycle(driver)
    setup = driver.oem_initialize_without_motion_test_case()
    assert setup['ok'] is True
    cycle(driver)
    before = len(frames)
    result = provider.initialize_motors(mode='live')
    assert all(v['published'] for v in result['reference_publications'].values())
    assert driver._oem_no_motion_profiles_ready == set()
    # Constructor-only divider/standby/ramp writes cannot be inserted by wake.
    assert not any(row[1] == 5 and row[2] in (7, 12, 13, 140, 153, 154) for row in frames[before:])


def test_no24v_stops_before_first_home(rig):
    driver, provider, frames, fault, root = rig
    cycle(driver)
    driver.oem_latch_24v_dropped(reason='offline source latch')
    before = len(frames)
    with pytest.raises(RuntimeError, match='Lost 24V power axisSearchHome'):
        provider.initialize_motors(mode='live')
    assert frames[before:] == []
    assert driver.oem_no24v_state() is True
    assert all(row['state'] != 'referenced' for row in
        provider.reference_store.snapshot(('x','y','z','g'))['rows'].values())


@pytest.mark.parametrize('kind', ['transport', 'external_cycle', 'ownership', 'abort'])
def test_real_authority_drift_cannot_publish_remaining_homes(rig, kind):
    driver, provider, frames, fault, root = rig
    cycle(driver)
    fired = []
    def drift(board, command, typ, motor, value):
        if not fired and (board, command, typ, motor) == (4, 5, 6, 2):
            fired.append(True)
            fault['on_frame'] = None
            if kind == 'transport':
                driver._oem_transport_generation = getattr(driver, '_oem_transport_generation', 0) + 1
                driver._invalidate_oem_no_motion_profiles(reason='transport_reconnected')
            elif kind == 'external_cycle':
                cycle(driver)
            elif kind == 'ownership':
                provider.generation_provider = lambda: 8
            else:
                driver.motor_oem_force_abort_motion(reason='offline cancellation')
    fault['on_frame'] = drift
    try:
        result = provider.initialize_motors(mode='live')
    except RuntimeError:
        result = None
    assert fired
    if result is not None:
        assert not all(v['published'] for v in result['reference_publications'].values())
    refs = ReferenceStateStore(root / 'bioxp_runtime.db').snapshot(('x','y','z','g'))
    assert not all(row['state'] == 'referenced' for row in refs['rows'].values())


@pytest.mark.parametrize('command', [2, 5])
def test_rejected_home_or_zero_not_published(rig, command):
    driver, provider, frames, fault, root = rig
    cycle(driver)
    fault['reject'] = (4, command, 0 if command == 2 else 1, 1)
    try:
        result = provider.initialize_motors(mode='live')
    except RuntimeError:
        result = None
    if result is not None:
        assert result['reference_publications']['z']['published'] is False
    assert provider.reference_store.snapshot(('z',))['rows']['z']['state'] != 'referenced'


def test_reference_storage_failure_remains_unqualified(rig):
    import sqlite3
    driver, provider, frames, fault, root = rig
    cycle(driver)
    db = sqlite3.connect(root / 'bioxp_runtime.db')
    db.execute("CREATE TRIGGER offline_fail_reference BEFORE UPDATE ON reference_state_authority BEGIN SELECT RAISE(ABORT,'offline persistence fault'); END")
    db.commit()
    db.close()
    before = len(frames)
    result = provider.initialize_motors(mode='live')
    assert result['ok'] is False
    assert frames[before:] == []
    assert not all(row['state'] == 'referenced' for row in
        provider.reference_store.snapshot(('x','y','z','g'))['rows'].values())


def test_missing_home_switch_never_becomes_reference(rig):
    driver, provider, frames, fault, root = rig
    cycle(driver)
    fault['home_missing'] = True
    try:
        result = provider.initialize_motors(mode='live', timeout_s=2)
    except RuntimeError:
        result = None
    if result is not None:
        assert result['reference_publications']['z']['published'] is False
    assert provider.reference_store.snapshot(('z',))['rows']['z']['state'] != 'referenced'


def test_partial_home_publication_storage_failure(rig):
    import sqlite3
    driver, provider, frames, fault, root = rig
    cycle(driver)
    fired = []
    def fail_reference(board, command, typ, motor, value):
        if not fired and (board, command, typ, motor) == (4, 5, 6, 2):
            assert provider.reference_store.snapshot(('z',))['rows']['z']['state'] == 'referenced'
            fired.append(True)
            db = sqlite3.connect(root / 'bioxp_runtime.db')
            db.execute("CREATE TRIGGER offline_fail_reference BEFORE UPDATE ON reference_state_authority BEGIN SELECT RAISE(ABORT,'offline partial persistence fault'); END")
            db.commit()
            db.close()
    fault['on_frame'] = fail_reference
    result = provider.initialize_motors(mode='live')
    assert fired
    assert result['reference_publications']['z']['published'] is False
    assert result['reference_publications']['z']['blocker'] == 'aggregate_reference_no_longer_current'
    assert result['reference_publications']['g']['published'] is False
    assert not all(row['state'] == 'referenced' for row in
        ReferenceStateStore(root / 'bioxp_runtime.db').snapshot(('x','y','z','g'))['rows'].values())


def test_manual_publication_still_requires_preparation(rig):
    driver, provider, frames, fault, root = rig
    cycle(driver)
    result = provider.state_store.publish_axis_reference('y', position_steps=0, ownership_generation=7)
    assert result['ok'] is False
    assert result['failure'] == 'axis_board_epoch_not_current'
