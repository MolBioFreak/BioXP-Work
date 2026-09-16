"""Accepted-wake transaction qualification; source-witness fixture is unit-only.

The separate connected test uses real native/provider callbacks and physical
leaves only. Here fault injection targets the canonical publisher/SQLite owner.
"""
import json
import sqlite3
import subprocess
import sys

import pytest
from tests.test_workflow_wake_canonical_transition import wake, run


def state(conn):
    return dict(conn.execute('SELECT * FROM operator_plane_deck_semantic_state').fetchone())


def immutable(conn, command_id):
    return dict(conn.execute('SELECT requested_inputs_json,canonical_request_sha256 '
                            'FROM operator_commands WHERE command_id=?', (command_id,)).fetchone())


def fresh(root):
    code = '''import json,sys
from bioxp.operator_command_plane import OperatorCommandStore
store=OperatorCommandStore(sys.argv[1])
try:
 print(json.dumps(dict(store.connection.execute('SELECT * FROM operator_plane_deck_semantic_state').fetchone())))
finally:
 store.stop()
'''
    return json.loads(subprocess.check_output([sys.executable, '-c', code, str(root)], text=True, timeout=25))


def seed(wake, prior):
    # Deliberately nonempty custody through actual production publication, not
    # an initialization snapshot or direct SQL mutation.
    for operation, updates in (
        ('pipette_owner', dict(tip_loaded=True, tip_dirty=True, tip_location=2)),
        ('clean_path_calculation', dict(clean_path=False)),
        ('GantryLoad', dict(plate_on_gantry=1, pseudo_z_home=65000)),
        ('sourceWellPierced', dict(well_pierced=[1, 3, 0])),
        ('sourceUnlatch', dict(latch_closed=False)),
        ('updateThermalDoorOpen', dict(thermal_door_open=prior)),
    ):
        wake.store.publish_deck_owner_state(source_operation=operation,
            source_command_id='fixture:' + operation, updates=updates, **wake.stamps)
    wake.provider._load_state()['machine_status']['thermal_door_open'] = prior


METADATA = {'semantic_state_revision', 'producer_operation', 'producer_command_id',
            'ownership_generation', 'board_epoch_4', 'board_epoch_5',
            'transition_provenance_json', 'updated_at'}


@pytest.mark.parametrize('prior', [False, True])
def test_exact_custody_publication_duplicate_and_fresh_read(wake, prior):
    seed(wake, prior)
    conn = wake.store.connection
    before = state(conn)
    original = immutable(conn, 'parent')
    plane = tuple(conn.execute("SELECT requested_json,effective_json FROM operator_plane_commands WHERE command_id='parent'").fetchone())
    result = run(wake)
    assert result['ok'] is True, result
    after = state(conn)
    assert {k:v for k,v in after.items() if k not in METADATA} == {k:v for k,v in before.items() if k not in METADATA}
    assert (after['board_epoch_4'], after['board_epoch_5']) == (2, 11)
    assert after['semantic_state_revision'] == before['semantic_state_revision'] + 1
    proof = json.loads(after['transition_provenance_json'])
    assert proof['updates'] == {'thermal_door_open': prior}
    assert proof['upstream_source_command_id'] == result['command_id']
    assert proof['source_operation'] == 'updateThermalDoorOpen'
    for key in ('latch_status', 'machine_latch_closed', 'latch_observation_id'):
        assert proof[key] == json.loads(before['transition_provenance_json'])[key]
    assert immutable(conn, 'parent') == original
    assert tuple(conn.execute("SELECT requested_json,effective_json FROM operator_plane_commands WHERE command_id='parent'").fetchone()) == plane
    child_identity = immutable(conn, result['command_id'])
    assert fresh(wake.store.root) == after
    assert run(wake) == result
    assert wake.called == ['initial', 'initialize']
    assert state(conn) == after
    with pytest.raises(ValueError, match='workflow_wake_binding_changed|workflow_wake_witness_already_consumed'):
        with wake.store._transaction() as tx:
            wake.store._accept_workflow_wake_initialization(tx, child_id=result['command_id'])
    assert state(conn) == after
    assert immutable(conn, result['command_id']) == child_identity
    assert run(wake, 'resume_temperature')['ok'] is True


@pytest.mark.parametrize('fault', ['after_write', 'python_owner', 'dispatcher_owner'])
def test_failed_publication_rolls_back_child_parent_and_semantics(wake, monkeypatch, fault):
    seed(wake, False)
    conn = wake.store.connection
    before = state(conn)
    parent_before = tuple(conn.execute("SELECT requested_inputs_json,canonical_request_sha256,effective_inputs_json FROM operator_commands WHERE command_id='parent'").fetchone())
    transitions_before = [tuple(r) for r in conn.execute('SELECT * FROM operator_plane_deck_semantic_transitions')]
    publications_before = [tuple(r) for r in conn.execute("SELECT * FROM operator_plane_commands WHERE action_id='oem.deck.semantic_state_publication'")]
    reached = []
    if fault == 'after_write':
        original = wake.store._insert_transition
        def fail(tx, **kwargs):
            result = original(tx, **kwargs)
            if kwargs['event_kind'] == 'deck_owner_state_published':
                assert state(tx)['semantic_state_revision'] == before['semantic_state_revision'] + 1
                reached.append(True)
                raise sqlite3.OperationalError('injected failure after complete semantic publication')
            return result
        monkeypatch.setattr(wake.store, '_insert_transition', fail)
    elif fault == 'python_owner':
        original = wake.store._publish_deck_owner_state
        def fail(tx, **kwargs):
            reached.append(True)
            wake.stamps['ownership_generation'] = 2
            return original(tx, **kwargs)
        monkeypatch.setattr(wake.store, '_publish_deck_owner_state', fail)
    else:
        original = wake.store._accept_workflow_wake_initialization
        def fail(tx, **kwargs):
            reached.append(True)
            owner = wake.store.owner_id
            try:
                wake.store.owner_id = 'changed-dispatcher-owner'
                return original(tx, **kwargs)
            finally:
                wake.store.owner_id = owner
        monkeypatch.setattr(wake.store, '_accept_workflow_wake_initialization', fail)
    result = run(wake)
    assert reached == [True]
    assert result['status'] == 'failed' and result['ok'] is False, result
    assert result['failure'] == 'workflow_wake_authority_not_accepted'
    assert all(row['ok'] is True for row in result['source_children'])
    assert all(r['published'] for r in result['source_children'][1]['reference_publications'].values())
    assert state(conn) == before == fresh(wake.store.root)
    assert tuple(conn.execute("SELECT requested_inputs_json,canonical_request_sha256,effective_inputs_json FROM operator_commands WHERE command_id='parent'").fetchone()) == parent_before
    assert [tuple(r) for r in conn.execute('SELECT * FROM operator_plane_deck_semantic_transitions')] == transitions_before
    assert [tuple(r) for r in conn.execute("SELECT * FROM operator_plane_commands WHERE action_id='oem.deck.semantic_state_publication'")] == publications_before
    assert conn.execute('SELECT status FROM operator_commands WHERE command_id=?', (result['command_id'],)).fetchone()[0] == 'failed'
    assert conn.execute('SELECT authority_write_allowed()').fetchone()[0] == 0
    # The genuine cycle advanced hardware epochs, but failed publication did
    # not advance parent authority. Even entry to a replay is therefore fenced.
    with pytest.raises(ValueError):
        run(wake)
    assert wake.called == ['initial', 'initialize']
    with pytest.raises(ValueError):
        run(wake, 'resume_temperature')
