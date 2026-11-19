"""Regressions from the 2026-11-18 failed launch: panel homes after a board
off/on, the profile check wiping the board session, refused launches stored
without a reason, and manual chiller off leaving the cool-down lock set.

Driver/adapter/provider/SQLite are real (existing wake rig); only wire replies
are doubled.
"""
import asyncio
import time

import pytest

from tests.test_wake_setup_debloat import rig, cycle  # noqa: F401
from tests.deck_scoped_rig import retained_rig  # noqa: F401


def _lifecycles(provider):
    state = provider.state_store.read_oem_serial206_initialization_state()
    return state['x_lifecycle'], state['z_lifecycle']


def _panel_homes(provider, tag):
    z = provider.execute_z_intent('manual_home', inputs={'timeout_s': 30.0},
                                  expected_generation=7, idempotency_key=f'z-home-{tag}')
    x = provider.execute_x_intent('manual_panel_home', {
        'timeout_s': 30.0, 'command_id': f'x-home-{tag}', 'idempotency_key': f'x-home-{tag}',
        'expected_generation': 7})
    assert z['ok'] is True and x['ok'] is True, (z, x)


def _references(provider):
    return {axis: row['state'] for axis, row in
            provider.reference_store.snapshot(('x', 'z'))['rows'].items()}


def test_panel_homes_after_board_cycle_are_kept(rig):
    driver, provider, frames, fault, root = rig
    cycle(driver)  # board off/on; nothing prepared afterwards
    live = driver.oem_current_board_lifecycle_generation()
    assert type(live) is int
    _panel_homes(provider, 'a')
    # Status reads used to discard both homes here.
    provider.z_projection()
    provider.x_projection()
    provider.z_projection()
    provider.x_projection()
    x, z = _lifecycles(provider)
    assert (x['state'], z['state']) == ('referenced_ready', 'referenced_ready'), (x.get('last_failure'), z.get('last_failure'))
    assert x['board_lifecycle_generation'] == live and z['board_lifecycle_generation'] == live
    assert _references(provider) == {'x': 'referenced', 'z': 'referenced'}


def test_real_board_cycle_after_homes_still_discards_them(rig):
    driver, provider, frames, fault, root = rig
    cycle(driver)
    _panel_homes(provider, 'b')
    provider.z_projection(); provider.x_projection()
    assert _references(provider) == {'x': 'referenced', 'z': 'referenced'}
    cycle(driver)  # a genuine new board session
    provider.z_projection(); provider.x_projection()
    x, z = _lifecycles(provider)
    assert (x['state'], z['state']) == ('unprepared', 'unprepared')
    assert _references(provider) == {'x': 'desynced', 'z': 'desynced'}


def test_failed_profile_check_keeps_board_session_and_other_axes(rig):
    driver, provider, frames, fault, root = rig
    cycle(driver)
    assert driver.oem_initialize_without_motion_test_case()['ok'] is True
    live = driver.oem_current_board_lifecycle_generation()
    assert {'x', 'y', 'z', 'g', 'door'} <= set(driver._oem_no_motion_profiles_ready)
    # Gripper stall guard changed on the controller (what the OEM gripper home leaves).
    driver.send_tmcl(4, 5, 205, 2, 5)
    with pytest.raises(RuntimeError, match='readback mismatch for g'):
        driver.motor_oem_require_no_motion_profile('g')
    assert driver.oem_current_board_lifecycle_generation() == live
    assert 'g' not in driver._oem_no_motion_profiles_ready
    assert {'x', 'y', 'z', 'door'} <= set(driver._oem_no_motion_profiles_ready)
    # The next call is refused for the same stated reason, not a lost session.
    with pytest.raises(RuntimeError, match='readback mismatch for g'):
        driver.motor_oem_require_no_motion_profile('g')
    assert driver.motor_oem_require_no_motion_profile('x')['ok'] is True


def _notes(count):
    from bioxp.protocols.models import ProtocolDocument
    return ProtocolDocument.from_payload({'protocol_id': 'refusal', 'stages': [{
        'stage_id': 'one', 'actions': [{'stage_id': 'one', 'action_id': f'a{i}', 'kind': 'note',
                                        'params': {}, 'description': 'n'} for i in range(count)]}]})


def test_launch_refused_before_first_step_fails_with_reason():
    from bioxp.protocols.executor import ProtocolExecutor

    def refuse(identity, state):
        raise ValueError('workflow_board_epoch_changed')
    executor = ProtocolExecutor(dry_run=False, job_id='parent', before_native_entry=refuse)
    state = executor.execute(_notes(2))
    assert executor.outcome == 'failed', state.to_payload()
    refused = [e for e in state.to_payload()['events'] if e['event'] == 'native_entry_refused']
    assert len(refused) == 1
    assert refused[0]['detail']['error'] == 'workflow_board_epoch_changed'
    assert refused[0]['detail']['native_entries_before'] == 0
    assert state.action_results == []


def test_refusal_after_a_step_ran_stays_unknown_and_keeps_reason():
    from bioxp.protocols.executor import ProtocolExecutor
    seen = []

    def refuse_second(identity, state):
        seen.append(identity)
        if len(seen) > 1:
            raise ValueError('workflow_authority_lost')
    executor = ProtocolExecutor(dry_run=False, job_id='parent', before_native_entry=refuse_second)
    state = executor.execute(_notes(2))
    assert executor.outcome == 'ambiguous', state.to_payload()
    refused = [e for e in state.to_payload()['events'] if e['event'] == 'native_entry_refused']
    assert refused[0]['detail']['error'] == 'workflow_authority_lost'
    assert refused[0]['detail']['native_entries_before'] == 1


@pytest.fixture
def chiller(monkeypatch):
    from bioxp import api
    from bioxp.usb_driver import BioXpTester
    driver = BioXpTester.__new__(BioXpTester)
    driver._chiller_last_tx_ts = 0.0
    driver._chiller_noresp_streak = 0
    frames = []
    temp = {'milli_c': 22000}
    pwm = {0: 60, 1: 60}

    def wire(board, command, typ, motor, value=0, **kwargs):
        frames.append((board, command, typ, motor, value))
        result = value
        if command == 143:
            result = temp['milli_c']
        elif command == 144:
            pwm[motor] = value
        elif command == 10 and typ == 23:
            result = pwm[motor]
        elif command == 10 and typ == 8:
            result = 125
        return {'status': 100, 'value': result, 'command': command, 'module': board}

    monkeypatch.setattr(driver, 'send_tmcl', wire)
    monkeypatch.setattr(driver, 'send_tmcl_retry', wire)
    monkeypatch.setattr(driver, '_send_chiller', lambda command, typ, motor, value=0, **kw: wire(7, command, typ, motor, value))
    monkeypatch.setattr(BioXpTester, 'CHILLER_WRITE_SETTLE_S', 0.0, raising=False)
    monkeypatch.setattr(BioXpTester, 'CHILLER_VERIFY_SETTLE_S', 0.0, raising=False)
    monkeypatch.setattr(api, '_get_tester', lambda: driver)

    async def inline(label, operation, *, timeout_s=None):
        return operation()
    monkeypatch.setattr(api, '_run_blocking', inline)
    return driver, frames, api


@pytest.mark.parametrize('bank,name', [('rc', 'Reagent Chiller'), ('oc', 'Output Nest')])
def test_manual_chiller_off_clears_cooldown_lock(chiller, bank, name):
    driver, frames, api = chiller
    index = 0 if bank == 'rc' else 1
    assert driver.oem_chiller_set_temperature(index, 10.0)['ok'] is True
    # The block never cooled and the OEM cool-down window has passed.
    driver._oem_thermal_state(driver.BOARD_CHILLER, index)['start'] = time.monotonic() - 3600
    locked = driver.oem_chiller_set_temperature(index, 10.0)
    assert locked['ok'] is False and locked['source_caught_exception'] == f'It took too long to cool the {name}'
    off = asyncio.run(api.set_chiller_pwm(api.ChillerPwmRequest(bank=bank, pwm=0)))
    assert off['ok'] is True and off['verified'] is True and off['pwm'] == 0
    assert (7, 144, 0, index, 0) in frames
    assert not any(row[1] == 64 for row in frames)  # OEM off sends no board activation
    service = driver.oem_fan_service_status()['services']
    assert [s['tc_on'] for s in service if s['axis'] == index] == [False]
    again = driver.oem_chiller_set_temperature(index, 10.0)
    assert again['ok'] is True and again.get('source_caught_exception') is None, again


def test_manual_chiller_pwm_nonzero_route_unchanged(chiller):
    driver, frames, api = chiller
    out = asyncio.run(api.set_chiller_pwm(api.ChillerPwmRequest(bank='rc', pwm=40)))
    assert out['ok'] is True and out['pwm'] == 40 and out['verified'] is True
    assert (7, 144, 0, 0, 40) in frames


def test_refused_launch_is_stored_failed_with_reason(retained_rig):
    """Real job store and dispatcher: the 05:29 launch, refused before step one."""
    import json
    from types import SimpleNamespace
    from bioxp.protocols.models import ProtocolDocument, ProtocolActionKind
    from bioxp.protocols.runtime_state import ProtocolRuntimeState
    from bioxp.services.protocol_service import bind_protocol_dispatcher
    from tests.test_protocol_workflow_connected import request_payload
    _, _, _, _, store, root = retained_rig
    payload = request_payload('refused')['document']
    for stage in payload['stages']:
        for action in stage['actions']:
            action['stage_id'] = stage['stage_id']
    doc = ProtocolDocument.from_payload(payload)
    state = ProtocolRuntimeState.from_document(doc, dry_run=False, job_id='refused-parent')
    bundle = {'protocol': {'document': doc.to_payload(), 'source_type': 'document'},
              'execution': {'dry_run': False, 'runtime_state': state.to_payload()},
              'operator': {'reviews': []}, 'created_at': 'fixture-created'}
    calls = []
    bind_protocol_dispatcher(store, binding_factory=lambda *a, **k: (
        {ProtocolActionKind.LED: lambda action, state: calls.append(action.action_id) or {'ok': True}}, {}, {}),
        artifact_store=SimpleNamespace(save=lambda bundle: None))
    # Board sessions recorded at launch that do not match the store's current ones.
    store.admit_workflow(command_id='refused-parent', idempotency_key='refused-parent', plan_fingerprint='refused',
        requested_inputs={'bundle': bundle}, ownership_generation=7, resources=(),
        board_epochs={'4': 999999, '5': 999999})
    claimed = store.claim_next()
    assert claimed['command_id'] == 'refused-parent'
    assert store._renew_owner(lease_seconds=120)
    store.assert_workflow_current(claimed["command_id"])  # Stale board numbers are not a refusal.
    store._priority_fence.set()  # An actual Stop still refuses entry.
    store._workflow_dispatcher(claimed)
    final = store.get_workflow('refused-parent')
    assert calls == []  # nothing was sent
    assert final['status'] == 'failed', final['status']
    runtime = final['execution']['runtime_state']
    refused = [e for e in runtime['events'] if e['event'] == 'native_entry_refused']
    assert refused and refused[0]['detail']['error'] == 'workflow_interrupted', json.dumps(runtime['events'])[:600]
    assert runtime['workflow']['held_reason'] == 'native_authority_unavailable'
