"""Connected canonical admission/publication, with no native hardware entry."""
import json
import sqlite3
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from threading import Event

import pytest
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.operator_command_plane import OperatorCommandStore
from bioxp.runtime_audit_store import RuntimeAuditDatabase


@pytest.fixture
def store(tmp_path):
    seed = OEMRuntimeStore(tmp_path)
    seed.close()
    owner = OperatorCommandStore(tmp_path)
    owner.bind_workflow_dispatcher(lambda claimed: None)
    yield owner
    owner.stop()


def start(store, board_epochs=None):
    store.admit_workflow(command_id='parent', idempotency_key='parent-key',
        plan_fingerprint='plan', requested_inputs={}, ownership_generation=1,
        resources=('axis:x', 'pipette', 'thermal'), board_epochs=board_epochs or {})
    assert store.claim_next()['command_id'] == 'parent'


def request(key):
    return dict(command_id=key, idempotency_key=key, action_id='pipette.mix', operation='mix',
        entrypoint_id='test', caller_class='protocol', control_class='physical_liquid_command',
        ownership_generation=1, requested_inputs={'volume': 1})


def children(bundle):
    return bundle['execution']['runtime_state']['workflow']['child_command_ids']


def test_reverse_native_completion_and_fresh_process(store):
    start(store)
    first_entered, second_completed, release_first = Event(), Event(), Event()
    completed = []

    def pipette_leaf():
        db = RuntimeAuditDatabase(store.root)
        try:
            with store.workflow_context('parent', source_occurrence_id='pipette'):
                child, created = db.claim(request('first'), pipette=True)
                assert created
                first_entered.set()
                assert release_first.wait(8)
                db.finalize_claim(command_id='first', pipette_operation_id=child['pipette_operation_id'],
                    expected_status='reserved', status='completed', outcome='success', failure_code=None,
                    result={'ok': True})
                completed.append('first')
        finally:
            db.close()

    def thermal_leaf():
        assert first_entered.wait(8)
        with store.workflow_context('parent', source_occurrence_id='thermal'):
            with store.normal_mutation_scope(resources=('thermal',)) as child:
                completed.append(child)
        second_completed.set()
        return child

    with ThreadPoolExecutor(max_workers=2) as pool:
        first = pool.submit(pipette_leaf)
        second = pool.submit(thermal_leaf)
        try:
            assert second_completed.wait(8)
            second_id = second.result(timeout=8)
            # Deliberately stale/fabricated executor view must not own the list.
            supplied = {'execution': {'runtime_state': {'workflow': {
                'command_id': 'parent', 'child_command_ids': [second_id, 'not-admitted']}}}}
            assert children(store.publish_workflow('parent', payload=supplied)) == ['first', second_id]
            assert supplied['execution']['runtime_state']['workflow']['child_command_ids'] == [second_id, 'not-admitted']
        finally:
            release_first.set()
        first.result(timeout=8)
    assert completed == [second_id, 'first']
    final = store.finish_workflow('parent', status='completed', payload=supplied, lifecycle_settled=True)
    assert final['command']['status'] == 'completed'
    assert children(final) == ['first', second_id]
    code = """import json,sys
from bioxp.operator_command_plane import OperatorCommandStore
s=OperatorCommandStore(sys.argv[1])
print(json.dumps(s.get_workflow('parent')))
s.stop()
"""
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', code, str(store.root)], text=True))
    assert children(reopened) == ['first', second_id]
    assert reopened['command']['status'] == 'completed'


def test_rolled_back_claim_and_host_control_never_become_children(store):
    start(store)
    db = RuntimeAuditDatabase(store.root)
    try:
        with store.workflow_context('parent', source_occurrence_id='claim'):
            # Duplicate resource membership fails AFTER canonical insertion.
            bad = {**request('rolled-back'), 'resources': ['pipette', 'pipette']}
            with pytest.raises(sqlite3.IntegrityError, match='UNIQUE constraint failed'):
                db.claim(bad, pipette=True)
            assert db.connection.execute('SELECT 1 FROM operator_commands WHERE command_id=?', ('rolled-back',)).fetchone() is None
            child, created = db.claim(request('real'), pipette=True)
            assert created
            assert db.claim(request('real'), pipette=True)[1] is False
        store.bind_workflow_controls('parent', lambda cid, req: {'phase': 'executing', 'gate': None})
        control = store.control_workflow('parent', request={'action': 'pause', 'mode': 'ordinary',
            'command_id': 'parent', 'expected_ownership_generation': 1, 'idempotency_key': 'pause'})
        assert control['control_command_id'] != 'real'
        assert children(store.publish_workflow('parent', payload={})) == ['real']
        assert children(store.finish_workflow('parent', status='completed', payload={}, lifecycle_settled=True)) == ['real']
    finally:
        db.close()


@pytest.mark.parametrize('next_generation', [12, None])
def test_workflow_x_generation_uses_persisted_source_not_legacy_table(store, next_generation):
    source = OEMRuntimeStore(store.root)
    state = {'movement_ledger': [], 'used_approvals': [], 'initialize_motion_ledger': [],
             'x_lifecycle': {'board_lifecycle_generation': 11}, 'label': 'é'}
    try:
        source.write_oem_serial206_initialization_state(state)
        # Legacy board5 projection is intentionally not activated.
        start(store, {'5': 11})
        with store.workflow_context('parent', source_occurrence_id='source-child'):
            with store.normal_mutation_scope(resources=('thermal',)):
                pass
        assert children(store.publish_workflow('parent', payload={}))
        state['x_lifecycle']['board_lifecycle_generation'] = next_generation
        source.write_oem_serial206_initialization_state(state)
        with pytest.raises(ValueError, match='workflow_board_epoch_changed'):
            store.assert_workflow_current('parent')
        with pytest.raises(ValueError, match='workflow_board_epoch_changed'):
            with store.workflow_context('parent', source_occurrence_id='refused'):
                pytest.fail('changed source authority must fence entry')
    finally:
        source.close()


def test_queued_claim_order_is_shared_with_direct_children(store):
    start(store)
    with store.workflow_context('parent', source_occurrence_id='queued'):
        queued = store.admit_command({'action_id': 'oem.x.prepare', 'inputs': {},
            'idempotency_key': 'queued', 'expected_ownership_generation': 1}, state={})
    with store.workflow_context('parent', source_occurrence_id='direct'):
        with store.normal_mutation_scope(resources=('thermal',)) as direct_id:
            pass
    assert children(store.publish_workflow('parent', payload={})) == [queued['command_id'], direct_id]
    assert store.connection.execute('SELECT parent_command_id FROM operator_commands WHERE command_id=?',
                                    (queued['command_id'],)).fetchone()[0] == 'parent'


@pytest.mark.parametrize('action,plane_status', [('abort', 'abort_requested'), ('safe_stop', 'stop_requested')])
def test_cooperative_control_uses_plane_status_and_restart_mapping(store, action, plane_status):
    start(store)
    store.bind_workflow_controls('parent', lambda cid, request: {'phase': 'cleanup', 'gate': None})
    response = store.control_workflow('parent', request={'action': action, 'command_id': 'parent',
        'expected_ownership_generation': 1, 'idempotency_key': action})
    assert response['accepted']
    assert store.get_workflow('parent')['command']['status'] == 'interrupting'
    assert store.connection.execute("SELECT status FROM operator_plane_commands WHERE command_id='parent'").fetchone()[0] == plane_status
    store.assert_workflow_current('parent')
    store.publish_workflow('parent', payload={})
    store._startup_recover()
    assert store.get_workflow('parent')['command']['status'] == 'ambiguous'
    assert store.connection.execute("SELECT status FROM operator_plane_commands WHERE command_id='parent'").fetchone()[0] == 'ambiguous'


@pytest.mark.parametrize('terminal', ['observed', 'completed', 'cancelled', 'ambiguous', 'outcome_unknown', 'reconciliation_required'])
def test_query_terminal_vocabulary_and_uncertain_custody(store, terminal):
    start(store)
    db = RuntimeAuditDatabase(store.root)
    try:
        with store.workflow_context('parent', source_occurrence_id='query'):
            payload = {**request('query'), 'control_class': 'hardware_query'}
            db.claim(payload)
            db.finalize_claim(command_id='query', pipette_operation_id=None, expected_status='reserved',
                status=terminal, outcome=None, failure_code=None, result={'source_return': 'query'})
        final = store.finish_workflow('parent', status='completed', payload={}, lifecycle_settled=True)
        expected = 'ambiguous' if terminal in {'ambiguous', 'outcome_unknown', 'reconciliation_required'} else 'completed'
        assert final['command']['status'] == expected
        assert children(final) == ['query']
    finally:
        db.close()


def test_finite_source_publications_and_actual_well_occupancy(store):
    from contextlib import nullcontext
    from bioxp.oem_deck_movement import OEM_MOVABLE_OBJECT_DEFAULT_LOCATIONS
    stamps = dict(ownership_generation=1, board_epoch_4=2, board_epoch_5=3)
    store.bind_deck_owner_authority_reader(lambda: stamps, scope=nullcontext)
    published = []
    def publish(operation, updates):
        result = store.publish_deck_owner_state(source_operation=operation,
            source_command_id='source-' + str(len(published)), updates=updates, **stamps)
        published.append(result)
        return result
    locations = dict(OEM_MOVABLE_OBJECT_DEFAULT_LOCATIONS)
    publish('updatePlateLocation', {'movable_plate_locations': locations})
    state = publish('updateLocation', {'current_location': 'LOC_RC', 'current_well': 17})
    assert (state['current_location'], state['current_well'], state['current_tray']) == ('LOC_RC', 17, 'REAGENT_PLATE')
    publish('pipette_owner', {'tip_loaded': True, 'tip_dirty': True, 'tip_location': 2})
    assert publish('clearTipLoaded', {'tip_loaded': False})['tip_loaded'] is False
    assert publish('sourceImageGantryLoad', {'pseudo_z_home': 500})['pseudo_z_home'] == 500
    assert publish('updateThermalDoorOpen', {'thermal_door_open': True})['thermal_door_open'] is True
    publish('sourceWellPierced', {'well_pierced': [1, 17, 0]})
    state = publish('sourceWellPierced', {'well_pierced': [7, 3, 'strip']})
    assert state['well_pierced'] == {'1:17:0': True, '7:3:strip': True}
    assert state['thermal_door_open'] is True
    state = publish('sourceUnlatch', {'latch_closed': False})
    assert state['machine_latch_closed'] is False and state['latch_status'] is False
    actual = store.connection.execute('SELECT current_location,current_well,current_tray,well_pierced_json FROM operator_plane_deck_semantic_state').fetchone()
    assert tuple(actual[:3]) == ('LOC_RC', 17, 'REAGENT_PLATE')
    assert json.loads(actual[3]) == state['well_pierced']
    # Existing selected-well occupancy owner remains the sole tip inventory.
    store.publish_tip_tray_transition(tray_id=0, transition='construct', operation_id='construct',
        command_id=state['producer_command_id'], provenance={'source': 'test'}, **stamps)
    store.publish_tip_tray_transition(tray_id=0, transition='remove_well', operation_id='pick17',
        command_id=state['producer_command_id'], well_ids=[17], provenance={'source': 'test'}, **stamps)
    occupancy = json.loads(store.connection.execute('SELECT occupancy_json FROM operator_plane_tip_tray_state WHERE tray_id=0').fetchone()[0])
    assert occupancy == [index != 17 for index in range(96)]
    before = store.deck_semantic_state()
    with pytest.raises(RuntimeError, match='deck_owner_authority_changed'):
        store.publish_deck_owner_state(source_operation='sourceWellPierced', source_command_id='stale',
            updates={'well_pierced': [1, 18, 0]}, **{**stamps, 'board_epoch_5': 4})
    assert store.deck_semantic_state() == before
    code = """import json,sys
from bioxp.operator_command_plane import OperatorCommandStore
s=OperatorCommandStore(sys.argv[1])
print(json.dumps({'state':s.deck_semantic_state(),'tips':s.tip_tray_state(0)}))
s.stop()
"""
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', code, str(store.root)], text=True))
    assert reopened['state']['well_pierced'] == state['well_pierced']
    assert reopened['state']['thermal_door_open'] is True
    assert reopened['state']['current_tray'] == 'REAGENT_PLATE'
    assert reopened['tips']['occupancy'] == occupancy
