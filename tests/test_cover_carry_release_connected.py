"""Offline provider -> finite plans -> real adapters -> native seams + SQLite.

No planner, child handler, semantic reader, or publisher success doubles.
SSD ControlLib 22215-22312, 26286-26508; OEM crossed pairing is unsafe.
"""
from types import SimpleNamespace

import pytest

import bioxp.oem_serial206_initialization as mod
from bioxp.oem_deck_movement import compile_finite_plate_operation, DeckExecutionFailure
from tests.test_cover_inspection_movement import OfflineNative
from tests.test_cover_inspection_flow import _SETTINGS, _jpeg, _GRAY_WITH_COVER
from tests.test_deck_scoped_authority import retained_rig, qualify_test_references


class TransferNative(OfflineNative):
    addresses = {**OfflineNative.addresses, 'g': (4, 2), 'd': (6, 0)}

    def __init__(self):
        super().__init__()
        self.parameters = {}
        self.positions[6, 0] = 0
        self.events = []
        self.fail_at = None

    def _machine_config_axis_max(self, axis):
        return (90263 if axis == 'x' else 102956), 'offline'

    def _motion_oem_axis_profile(self, axis, *, startup):
        return {**super()._motion_oem_axis_profile(axis, startup=startup),
                'speed': 1800, 'acc': 400, 'run_current': 31,
                'stall_guard': 16, 'disable_right': True}

    def motor_set_axis_param(self, board, param, value, *, motor):
        self.parameters[board, motor, param] = value
        return super().motor_set_axis_param(board, param, value, motor=motor)

    def motor_get_axis_param(self, board, param, *, motor):
        value = self.parameters[board, motor, param]
        return {'ok': True, 'value': value, 'ack': {'status': 100, 'value': value}}

    def motor_oem_move_absolute(self, board, target, *, motor, wait_for_stop, max_position=None):
        axis = next(a for a, address in self.addresses.items() if address == (board, motor))
        self.events.append(('move', axis, target, self.positions[5, 0], self.positions[4, 0]))
        if self.fail_at == (axis, target):
            raise RuntimeError('injected_native_transfer_failure')
        result = super().motor_oem_move_absolute(board, target, motor=motor,
            wait_for_stop=wait_for_stop, max_position=max_position)
        result.update(completion_class='event_128', source_return_code=0,
            terminal_position=self.motor_get_position(board, motor=motor))
        if axis == 'g':
            self.home = False
        return result

    def motor_oem_home_axis(self, axis, **kwargs):
        assert axis == 'g'
        self.events.append(('home', axis))
        self.positions[4, 2] = 0
        self.home = True
        return {'ok': True, 'axis': 'g', 'startup': False, 'prepare': {},
                'restore_current': {}, 'home': {'ok': True, 'source_return_code': 0}}

    def motor_thermal_door_status(self):
        opened = self.positions[6, 0] != 0
        return {'ok': True, 'predicates_verified': True, 'opened': opened, 'closed': not opened}


@pytest.fixture
def connected(retained_rig, monkeypatch):
    import socket
    def closed(*args, **kwargs):
        raise AssertionError('offline test may not open hardware transports')
    monkeypatch.setattr(socket, 'socket', closed)
    try:
        import serial
    except ImportError:
        pass
    else:
        monkeypatch.setattr(serial, 'Serial', closed)
    provider, _, runtime, references, store, root = retained_rig
    from bioxp import oem_machine_bundle as bundle
    snapshot = bundle.get_active_oem_machine_snapshot()
    monkeypatch.setattr(bundle, '_active_snapshot', bundle.load_oem_machine_snapshot(
        snapshot.bundle_root / 'OEM_EVIDENCE_LOCK.json',
        operator_label_serial=206, require_operator_label=True))
    qualify_test_references(references)
    native = TransferNative()
    adapter = object.__new__(mod.Serial206ProductionPrimitiveAdapter)
    adapter.tester, adapter.y_provider, adapter.reference_store = native, None, references
    from bioxp.serial206_y_provider import Serial206YProvider
    adapter.y_provider = Serial206YProvider(native, state_store=runtime,
        generation_provider=lambda: 3, reference_store=references)
    provider.primitives = adapter
    provider.bind_pipette_collection_state_reader(lambda: {'tip_exists': False})
    monkeypatch.setattr(mod.time, 'sleep', lambda seconds: None)
    stamps = provider.deck_owner_authority_stamps()
    for op, updates in (
        ('updateLocation', {'current_location': 'LOC_OC_COVER', 'current_well': 0}),
        ('pipette_owner', {'tip_loaded': False, 'tip_dirty': False, 'tip_location': -1}),
        ('clean_path_calculation', {'clean_path': False}),
        ('sourceUnlatch', {'latch_closed': False}),
        ('sourceForceToHighHome', {'pseudo_z_home': 500}),
    ):
        store.publish_deck_owner_state(source_operation=op,
            source_command_id='offline-predecessor-' + op, updates=updates, **stamps)
    provider.wp8_update_thermal_door_open('updateThermalDoorOpen', {'value': False},
        command_id='offline-door', child_order=0, plan_digest='offline')
    frame = _jpeg(_GRAY_WITH_COVER)
    import numpy as np
    empty = _jpeg(np.zeros_like(_GRAY_WITH_COVER))
    provider.bind_oem_cover_inspection_callbacks(settings=lambda: _SETTINGS,
        capture=lambda **kw: {'frame': empty if kw['condition'] in
            ('check_chiller_cover_20', 'check_chiller_cover_18') else frame, 'capture_evidence': {}},
        save=lambda **kw: {'ok': True}, led=lambda **kw: None, rgb=lambda *args: None,
        barcode=lambda frame: '')
    fences = []
    provider.bind_wp8_execution_fence_checker(lambda command_id, **kw:
        fences.append((command_id, kw)))
    # Use real local background workers; only the native hardware seam is fake.
    provider.bind_wp8_background_worker_starter(lambda worker, name: worker.start())
    owner = {**stamps, 'dispatch_attempt_id': 'offline-attempt',
             'work_identity': 'child:8:coverInspectionRelocate', 'plan_digest': 'root-plan'}
    plan = compile_finite_plate_operation('cover_inspection', source_leaf_available=True,
        deck_inspection=True, screen_resolution_high=False, inspection_log_only=False)
    assert store.deck_semantic_state()['movable_plate_locations'] == {}
    # Real camera classifiers and location publications before relocation.
    for child in plan['children'][2:8]:
        provider.execute_wp8_child(child, command_id='offline-inspection',
            child_order=child['order'], plan_digest=plan['plan_digest'])
    assert store.deck_semantic_state()['movable_plate_locations'] == {
        'OUTPUT_COVER': 'LOC_OC_COVER', 'REAGENT_COVER': 'LOC_RC_COVER'}
    native.events.clear()
    native.moves.clear()
    return SimpleNamespace(provider=provider, native=native, store=store,
        root=root, owner=owner, fences=fences, plan=plan)


def relocate(rig):
    return rig.provider.execute_wp8_child(
        {**rig.plan['children'][8], '_delivery_identity': rig.owner},
        command_id='offline-inspection', child_order=8, plan_digest=rig.plan['plan_digest'])


def test_both_covers_reach_storage_before_release_and_finalize(connected):
    try:
        result = relocate(connected)
    except DeckExecutionFailure as exc:
        pytest.fail(repr([{'phase': r.get('phase'), 'failures': r.get('result', {}).get('failure_evidence')}
                          for r in exc.provider_results]))
    assert result['ok'] is True
    assert result['final_cover_locations'] == {'output': 18, 'reagent': 20}
    assert result['door_open_verified'] is True
    state = connected.store.deck_semantic_state()
    assert state['movable_plate_locations'] == {
        'OUTPUT_COVER': 'LOC_OC_COVER_STORAGE', 'REAGENT_COVER': 'LOC_RC_COVER_STORAGE'}
    assert state['plate_on_gantry'] is None
    events = connected.native.events
    # Actual native lowering and gripper opening must occur at both storage
    # coordinates, not merely publish destination labels after failed travel.
    _, positions = connected.provider._wp8_calibration()
    lowers = [(i, e) for i, e in enumerate(events)
              if e[:3] == ('move', 'z', 105981)]
    assert [e[3:] for _, e in lowers] == [(84252, 6057), (84252, 36267)]
    for i, e in lowers:
        assert any(x[:3] == ('move', 'g', positions['open']) and x[3:] == e[3:]
                   for x in events[i + 1:])
    park = next(i for i, e in enumerate(events) if e[:3] == ('move', 'z', 114092))
    assert park > lowers[-1][0]
    tasks = connected.provider._wp8_tasks
    z_tasks = [r for r in tasks.values() if r['kind'] == 'move_z_pseudo_home']
    assert len(z_tasks) == 2
    assert len({r['plan_digest'] for r in z_tasks}) == 2
    for r in tasks.values():
        r['thread'].join(timeout=2)
        assert r['state'] == 'completed'
    # A fresh process observes the same committed custody, not RAM success.
    import json, subprocess, sys
    from pathlib import Path
    code = ('import sqlite3,json,sys; c=sqlite3.connect(sys.argv[1]); '
            'r=c.execute("SELECT movable_plate_locations_json,plate_on_gantry '
            'FROM operator_plane_deck_semantic_state").fetchone(); '
            'print(json.dumps([json.loads(r[0]),r[1]]))')
    observed = json.loads(subprocess.check_output(
        [sys.executable, '-c', code, str(Path(connected.root) / 'bioxp_runtime.db')], text=True))
    assert observed == [state['movable_plate_locations'], None]


def test_occupied_target_never_commands_transfer_or_final_storage_write(connected):
    # The observation layer is the only injected input; execution and SQLite
    # publication remain real, with hardware replaced only at the native seam.
    connected.provider._oem_cover_inspection_findings['offline-inspection'] = {
        17: True, 19: False, 20: True, 18: False,
    }
    before = connected.store.deck_semantic_state()['movable_plate_locations']
    result = relocate(connected)
    assert result['error_status'] == 'UNSAFE_COVER_TOPOLOGY'
    assert result['relocations'] == []
    assert result['final_cover_locations'] is None
    assert not connected.native.events
    assert connected.store.deck_semantic_state()['movable_plate_locations'] == before


@pytest.mark.parametrize('phase', ['catch', 'release'])
def test_failed_transfer_never_finalizes_or_parks(connected, phase):
    _, positions = connected.provider._wp8_calibration()
    connected.native.fail_at = ('g', positions['close']) if phase == 'catch' else ('x', 84252)
    with pytest.raises(DeckExecutionFailure, match='cover_inspection_' + phase + '_failed') as caught:
        relocate(connected)
    assert 'injected_native_transfer_failure' in repr(caught.value.provider_results)
    events = connected.native.events
    assert not any(e[:2] == ('move', 'd') for e in events)
    assert not any(e[:3] == ('move', 'z', 114092) for e in events)
    assert not any(e[:3] == ('move', 'z', 105981) for e in events)
    state = connected.store.deck_semantic_state()
    assert state['movable_plate_locations']['REAGENT_COVER'] == 'LOC_RC_COVER'
    assert state['movable_plate_locations']['OUTPUT_COVER'] == (
        'LOC_OC_COVER' if phase == 'catch' else 'LOC_GANTRY')
    assert state['plate_on_gantry'] == (None if phase == 'catch' else 4)
    assert not connected.provider._wp8_tasks
