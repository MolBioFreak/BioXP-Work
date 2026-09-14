"""Source-owned deck regressions. All controller primitives are offline doubles."""
import copy
from types import SimpleNamespace

import pytest

from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider
from bioxp.oem_deck_movement import DeckAuthoritySnapshot
from bioxp.operator_controls import _deck_authority_diagnostic
from bioxp.operator_command_plane import OperatorCommandStore
from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot


class Primitive:
    def __init__(self, home=False, position=50):
        self.home, self.position, self.calls = home, position, []
        self.tester = self
    def motor_query_home_switch(self, board, motor):
        self.calls.append(('home', board, motor))
        return {'ok': True, 'reply_valid': True, 'home': self.home}
    def motor_get_position(self, board, motor):
        self.calls.append(('position', board, motor))
        return {'ok': True, 'position': self.position, 'ack': {'status': 100, 'value': self.position}}
    def _read_axis_position(self, axis):
        self.calls.append(('xyz', axis))
        return 0
    def query_latch(self):
        self.calls.append(('latch',))
        return {'ok': True, 'value': 1}
    def read_oem_latch_status(self):
        return {'ok': True, 'value': True, 'observation_id': 'host-1'}
    def oem_move_to(self, *args, **kwargs):
        self.calls.append(('move', args, kwargs))
        return {'ok': True, 'controller_command_acknowledged': True, 'controller_completion_verified': True}


@pytest.fixture
def rig(monkeypatch):
    bind_serial206_oem_snapshot(monkeypatch)
    primitive = Primitive(home=True)
    provider = Serial206OemInitializationProvider(primitive, generation_provider=lambda: 3)
    # Explicit retained null predecessor, not a call to the constructor.
    state = {'schema_version': provider.schema, 'machine_status': {'current_location': None,
             'current_well': None, 'tip_loaded': False, 'tip_dirty': None,
             'tip_location': None, 'psudo_z_home_steps': 65000},
             'x_lifecycle': {'board_lifecycle_generation': 9}}
    monkeypatch.setattr(provider, '_load_state', lambda: copy.deepcopy(state))
    monkeypatch.setattr(provider, '_save_state', lambda value: state.update(copy.deepcopy(value)) or value)
    provider.state_store = SimpleNamespace(board4_authority_projection=lambda: {'board': {'active_board_epoch': 7, 'state': 'active'}})
    provider.reference_store = SimpleNamespace(snapshot=lambda axes: {'rows': {a: {'state': 'referenced', 'state_version': 1} for a in axes}})
    semantic = {'semantic_state_revision': 0, 'ambiguity_state': 'none', 'transition_provenance': {},
                'current_location': None, 'current_well': None, 'tip_loaded': None,
                'tip_dirty': None, 'tip_location': None, 'clean_path': None,
                'plate_on_gantry': None, 'pseudo_z_home': 65000,
                'ownership_generation': None, 'board_epoch_4': None, 'board_epoch_5': None}
    provider.bind_deck_semantic_state_reader(lambda: copy.deepcopy(semantic))
    return provider, primitive, semantic, state


def test_offset_first_authority_preserves_unused_nulls_and_passive_read(rig):
    provider, primitive, semantic, state = rig
    snap = provider.deck_authority_snapshot(expected_generation=3, target='LOC_OC')
    typed = DeckAuthoritySnapshot(**snap)
    assert typed.machine_state_revision == 0
    assert typed.current_location_id is None and typed.current_well_id is None
    assert typed.tip_dirty is None and typed.tip_location is None and typed.clean_path is None
    assert typed.tip_loaded is False and typed.gripper_confirmed is True
    calls = list(primitive.calls)
    assert provider.deck_authority_cached_snapshot(expected_generation=3, target='LOC_OC') == snap
    assert primitive.calls == calls
    assert semantic['semantic_state_revision'] == 0
    assert 'construction_id' not in state['machine_status']


@pytest.mark.parametrize('home,position,expected', [(True,50,True),(False,49,True),(False,50,False)])
def test_bound_move_uses_actual_gripper_not_reference_constant(rig, home, position, expected):
    provider, primitive, semantic, state = rig
    primitive.home, primitive.position = home, position
    # Legacy full sealed input also must carry actual source predicate, not reference-derived true.
    authority = dict(tip_loaded=False, tip_dirty=False, tip_location=-1, clean_path=False,
        pseudo_z_home=500, ownership_generation=3, board_epoch_4=7, board_epoch_5=9,
        current_location_id='LOC_MS', current_well_id=0, machine_state_revision=1,
        semantic_state_provenance_digest='a'*64, plate_on_gantry=None,
        gripper_confirmed=expected)
    provider.moveTo(location_id=1, authority_snapshot=authority)
    assert primitive.calls[-1][2]['gripper_confirmed'] is expected


@pytest.mark.parametrize('reason', ['deck_bootstrap_semantic_location_unavailable',
    'deck_bootstrap_board_epochs_unavailable', 'deck_bootstrap_branch_state_unavailable',
    'deck_bootstrap_latch_or_tip_state_unavailable'])
def test_exact_bootstrap_diagnostic(reason):
    assert _deck_authority_diagnostic(RuntimeError(reason)) == 'canonical_deck_authority_unavailable:' + reason
    assert _deck_authority_diagnostic(RuntimeError(reason + ':private text')) == 'canonical_deck_authority_unavailable'


def test_ordered_tray_unknown_is_not_stale_verified_association():
    assert OperatorCommandStore._oem_update_location_current_tray('LOC_OC', {}, 'OUTPUT_PLATE') is None


@pytest.mark.parametrize('field,value', [('tip_loaded', None), ('tip_loaded', 0), ('ambiguity_state', 'ambiguous')])
def test_consumed_unknowns_and_ambiguity_remain_blocked(rig, field, value):
    provider, primitive, semantic, state = rig
    if field == 'tip_loaded':
        state['machine_status'][field] = value
    else:
        semantic[field] = value
    with pytest.raises(RuntimeError, match='deck_semantic_state_not_authoritative'):
        provider.deck_authority_snapshot(expected_generation=3, target='LOC_OC')
    assert not any(row[0] == 'move' for row in primitive.calls)


@pytest.mark.parametrize('home,position,expected', [(True,50,True),(False,49,True),(False,50,False)])
def test_actual_query_predicate_and_short_circuit(rig, home, position, expected):
    provider, primitive, _, state = rig
    primitive.home, primitive.position = home, position
    state['machine_status']['plate_on_gantry'] = None
    snap = provider.deck_authority_snapshot(expected_generation=3, target='LOC_OC')
    assert snap['gripper_confirmed'] is expected
    assert any(row[0] == 'position' for row in primitive.calls) is (not home)


def test_unknown_home_reply_never_becomes_true(rig):
    provider, primitive, _, _ = rig
    primitive.motor_query_home_switch = lambda *a, **k: {'ok': False, 'reply_valid': False, 'home': True}
    with pytest.raises(RuntimeError, match='deck_gripper_observation_not_authoritative'):
        provider.deck_authority_snapshot(expected_generation=3, target='LOC_OC')


def test_full_failure_does_not_replace_offset_cache(rig):
    provider, primitive, _, _ = rig
    snapshot = provider.deck_authority_snapshot(expected_generation=3, target='LOC_OC')
    with pytest.raises(RuntimeError):
        provider.deck_authority_snapshot(expected_generation=3, target='LOC_PARK')
    before = list(primitive.calls)
    assert provider.deck_authority_cached_snapshot(expected_generation=3, target='LOC_MS') == snapshot
    with pytest.raises(RuntimeError):
        provider.deck_authority_cached_snapshot(expected_generation=3, target='LOC_PARK')
    assert primitive.calls == before
    started, epoch, payload = provider._deck_authority_scoped_cache['offset.v1']
    provider._deck_authority_scoped_cache['offset.v1'] = (started - 16, epoch, payload)
    with pytest.raises(RuntimeError, match='deck_authority_cache_stale'):
        provider.deck_authority_cached_snapshot(expected_generation=3, target='LOC_OC')


@pytest.fixture
def retained_rig(monkeypatch, tmp_path):
    import os, shutil
    from pathlib import Path
    from bioxp.oem_runtime_store import OEMRuntimeStore
    from bioxp.services.reference_service import ReferenceStateStore
    bind_serial206_oem_snapshot(monkeypatch)
    baseline = os.environ.get('DECK_RETAINED_BASELINE')
    if not baseline:
        pytest.skip('optional captured retained-state qualification requires DECK_RETAINED_BASELINE')
    root = tmp_path / 'retained-copy'
    shutil.copytree(Path(baseline), root)
    runtime = OEMRuntimeStore(root)
    references = ReferenceStateStore(root / 'bioxp_runtime.db')
    primitive = Primitive(home=True)
    provider = Serial206OemInitializationProvider(primitive, state_store=runtime,
        reference_store=references, generation_provider=lambda: 3)
    store = OperatorCommandStore(root)
    provider.bind_deck_semantic_state_reader(store.deck_semantic_state)
    provider.bind_deck_semantic_state_publisher(store.publish_deck_owner_state)
    store.bind_deck_owner_authority_reader(provider.deck_owner_authority_stamps)
    yield provider, primitive, runtime, references, store, root
    store.stop()
    runtime.close()


def qualify_test_references(references):
    from bioxp.services.reference_service import MarkAxisReferencedCommand
    # Explicit isolated evidence owner event, never source-store or hardware mutation.
    result = references.mark_referenced_many([MarkAxisReferencedCommand(a, 0,
        source='isolated_controller_fixture', note='No physical acceptance') for a in ('x','y','z','g')])
    assert result['ok'] is True


def test_retained_invalid_references_still_block(retained_rig):
    provider, primitive, runtime, references, store, root = retained_rig
    with pytest.raises(RuntimeError, match='deck_reference_not_authoritative'):
        provider.deck_authority_snapshot(expected_generation=3, target='LOC_OC')
    assert store.deck_semantic_state()['semantic_state_revision'] == 0
    assert not any(row[0] == 'move' for row in primitive.calls)


@pytest.mark.parametrize('invalidation', [None, 'reference', 'board_epoch', 'semantic'])
def test_retained_real_owner_first_move_force_commit_and_fresh_process(retained_rig, monkeypatch, invalidation):
    import json, os, subprocess, sys
    from pathlib import Path
    from bioxp.oem_compat.position_table import load_bound_oem_position_table
    from bioxp.oem_deck_movement import make_deck_command_executor
    provider, primitive, runtime, references, store, root = retained_rig
    qualify_test_references(references)
    initial = provider.deck_authority_snapshot(expected_generation=3, target='LOC_OC')
    assert initial['machine_state_revision'] == 0 and initial['current_location_id'] is None
    assert initial['tip_loaded'] is False and initial['tip_dirty'] is None
    stamps = provider.deck_owner_authority_stamps()
    state = {'ownership_generation': 3, 'serial206_initialization_provider': {
        'x_authority': {'current_board_lifecycle_generation': stamps['board_epoch_5']},
        'board4_authority': {'active_board_epoch': stamps['board_epoch_4']}}}
    request = dict(schema_version='bioxp.operator_action_request.v2', action_id='oem.deck.move_to_location',
        expected_ownership_generation=3, expected_board_epoch_by_board={'4': stamps['board_epoch_4'], '5': stamps['board_epoch_5']},
        idempotency_key='retained-first', inputs={'target': 'LOC_OC', 'camera_offset': False})
    admitted = store.admit_command(request, state=state, assessment={'enabled': True})
    claimed = store.claim_next()
    assert claimed['command_id'] == admitted['command_id']
    assert store.connection.execute('PRAGMA user_version').fetchone()[0] == 8
    assert store.connection.execute('SELECT deck_owner_authority_current(?,?,?)', (3, stamps['board_epoch_4'], stamps['board_epoch_5'])).fetchone()[0] == 1
    execute = make_deck_command_executor(provider_getter=lambda: provider,
        position_table_provider=load_bound_oem_position_table, command_store=store)
    if invalidation is not None:
        original_move = primitive.oem_move_to
        def completed_then_invalidated(*args, **kwargs):
            result = original_move(*args, **kwargs)
            if invalidation == 'reference':
                from bioxp.services.reference_service import MarkAxisDesyncedCommand
                references.mark_desynced(MarkAxisDesyncedCommand('x', reason='isolated late reference change'))
            elif invalidation == 'board_epoch':
                original_stamps = provider.deck_owner_authority_stamps
                monkeypatch.setattr(provider, 'deck_owner_authority_stamps', lambda: {
                    **original_stamps(), 'board_epoch_5': stamps['board_epoch_5'] + 1})
            else:
                state = provider._load_state()
                state['machine_status']['tip_loaded'] = True
                provider._save_state(state)
            return result
        monkeypatch.setattr(primitive, 'oem_move_to', completed_then_invalidated)
        from bioxp.oem_deck_movement import DeckExecutionFailure
        with pytest.raises(DeckExecutionFailure) as failure:
            execute(command_id=admitted['command_id'], target='LOC_OC', camera_offset=False,
                    expected_ownership_generation=3, expected_board_epoch_by_board=request['expected_board_epoch_by_board'])
        assert failure.value.controller_completion_verified is True
        assert failure.value.provider_results[-1]['controller_completion_verified'] is True
        semantic = store.deck_semantic_state()
        assert semantic['current_location'] is None and semantic['pseudo_z_home'] == 500
        stages = store.connection.execute('SELECT * FROM operator_plane_deck_stages WHERE command_id=?', (admitted['command_id'],)).fetchall()
        assert any(dict(row).get('operation') == 'ForceToHighHome' and dict(row).get('terminal_state') == 'completed' for row in stages)
        return
    result = execute(command_id=admitted['command_id'], target='LOC_OC', camera_offset=False,
        expected_ownership_generation=3, expected_board_epoch_by_board=request['expected_board_epoch_by_board'])
    assert result['ok'] is True and result['semantic_state_committed'] is True
    final = store.finish(admitted['command_id'], status='completed', payload=result, claimed=claimed,
        controller_acknowledged=True, full_response=result)
    semantic = store.deck_semantic_state()
    assert semantic['current_location'] == 'LOC_OC' and semantic['current_well'] == 0
    assert semantic['semantic_state_revision'] == 2 and semantic['tip_dirty'] is None
    assert semantic['tip_loaded'] is None  # runtime observation did not manufacture a canonical owner write
    assert semantic['transition_provenance']['current_tray_association'] == 'unavailable'
    assert provider._load_state()['machine_status'].get('construction_id') is None
    assert provider._load_state()['machine_status']['psudo_z_home_steps'] == 500
    after = provider.deck_authority_snapshot(expected_generation=3, target='LOC_OC')
    assert after['machine_state_revision'] == 2 and after['tip_loaded'] is False
    detail = store.command_detail_v2(admitted['command_id'])
    script = 'import json,sys; from bioxp.operator_command_plane import OperatorCommandStore; s=OperatorCommandStore(sys.argv[1]); print(json.dumps({"semantic":s.deck_semantic_state(),"detail":s.command_detail_v2(sys.argv[2])})); s.stop()'
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', script, str(root), admitted['command_id']], text=True))
    assert reopened['semantic'] == semantic and reopened['detail']['command_id'] == admitted['command_id']
    if os.environ.get('DECK_TEST_OUTPUT'):
        Path(os.environ['DECK_TEST_OUTPUT']).write_text(json.dumps({'initial_authority': initial,
            'result': result, 'detail': detail, 'fresh_process': reopened, 'calls': primitive.calls}, indent=2))
