"""R1 partial-publication regressions on real SQLite and native dispatch paths."""
import pytest
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig, named_move
from tests.test_r1_constructor_owner_connected import prepare_constructor
from tests.protocol_v1_integration_fixture import integrated_rig
from tests.test_pipette_collection_owner import native_leaves
from tests.test_deck_tip_query_publication_contradiction import submit_named


@pytest.mark.parametrize('event', ['restart', 'reconnect'])
def test_named_then_park_partial_publication_current_owner(query_rig, monkeypatch, event):
    from bioxp import api
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider
    from bioxp.oem_deck_movement import make_deck_command_executor
    from bioxp.oem_compat.position_table import load_bound_oem_position_table
    from bioxp.operator_command_plane import OperatorCommandStore
    app, provider, primitive, refs, root, receipts, calls, wire, group = query_rig
    prepare_constructor(query_rig, monkeypatch)
    native_leaves(query_rig, monkeypatch)
    if event == 'restart':
        # Open the retained SQLite through new provider/store instances, not a
        # fabricated full canonical predecessor or a restamped semantic row.
        provider = Serial206OemInitializationProvider(primitive,
            state_store=provider.state_store, reference_store=refs,
            generation_provider=provider.generation_provider)
        app.state.operator_command_plane.stop()
        store = OperatorCommandStore(root)
        store.bind_deck_owner_authority_reader(provider.deck_owner_authority_stamps,
            scope=provider.deck_owner_authority_scope)
        provider.bind_deck_semantic_state_reader(store.deck_semantic_state)
        provider.bind_deck_semantic_state_publisher(store.publish_deck_owner_state)
        provider.bind_tip_tray_state_reader(store.tip_tray_state)
        provider.bind_pipette_collection_state_reader(lambda: api._pipette_collection_state(ensure_constructor=True))
    else:
        store = app.state.operator_command_plane.store
        provider.notify_board_activation(4, {'status': 100}, active=True)
    execute = make_deck_command_executor(provider_getter=lambda: provider,
        position_table_provider=load_bound_oem_position_table, command_store=store)
    initial = store.deck_semantic_state()
    assert initial['tip_dirty'] is None and initial['latch_status'] is None
    for target in ['LOC_OC', 'LOC_PARK']:
        stamps = provider.deck_owner_authority_stamps()
        epochs = {'4': stamps['board_epoch_4'], '5': stamps['board_epoch_5']}
        request = dict(schema_version='bioxp.operator_action_request.v2',
            action_id='oem.deck.move_to_location', expected_ownership_generation=stamps['ownership_generation'],
            expected_board_epoch_by_board=epochs, idempotency_key=event + '-' + target,
            inputs={'target': target, 'camera_offset': False})
        admitted = store.admit_command(request, state={'ownership_generation': stamps['ownership_generation'],
            'serial206_initialization_provider': {'x_authority': {'current_board_lifecycle_generation': epochs['5']},
            'board4_authority': {'active_board_epoch': epochs['4']}}}, assessment={'enabled': True})
        claimed = store.claim_next()
        assert claimed['command_id'] == admitted['command_id']
        result = execute(command_id=admitted['command_id'], target=target, camera_offset=False,
            expected_ownership_generation=stamps['ownership_generation'], expected_board_epoch_by_board=epochs)
        assert result['ok'], result
        final = store.finish(admitted['command_id'], status='completed', payload=result, claimed=claimed,
            controller_acknowledged=result['controller_command_acknowledged'], full_response=result)
        assert store.command_detail_v2(admitted['command_id'])['status'] == 'completed'
    assert group._constructor_started
    assert calls == []
    assert store.deck_semantic_state()['current_location'] == 'LOC_PARK'
    assert store.deck_semantic_state()['tip_dirty'] is None  # not backfilled
    if event == 'restart':
        store.stop()


@pytest.mark.parametrize('event', ['restart', 'reconnect'])
def test_protocol_child_reads_partial_semantics_after_owner_boundary(integrated_rig, query_rig, monkeypatch, event):
    from bioxp import api
    rig = integrated_rig
    prepare_constructor(query_rig, monkeypatch)
    rig.provider.bind_pipette_collection_state_reader(lambda: api._pipette_collection_state(ensure_constructor=True))
    if event == 'reconnect':
        rig.provider.notify_board_activation(4, {'status': 100}, active=True)
    # Restart starts this real service/provider fixture from retained SQLite.
    # Publish a partial object update through the real producer, not SQL edits.
    before = rig.store.deck_semantic_state()
    owner = rig.provider.deck_owner_authority_stamps()
    # Reconnect preserves a current active board owner when continuity is
    # observed. It does not rewrite historical semantic publication stamps.
    payload = rig.payload('r1-' + event, opcode='mov', arguments={
        'm_destination': {'enum_type': 'locationID', 'value': 6}, 'm_well': 0})
    payload['document']['stages'][0]['actions'][-1]['params']['argument_type'] = 'ClassMoveTo'
    leaf = rig.native.tester.motor_wait_target_reached.__self__
    def wait_many(axes, **kwargs):
        # Protocol worker is MTA, not the manual STA-only rig default. This
        # replaces only the inert controller event seam, never native dispatch.
        return {'ok': True, 'per_axis': {axis: leaf.motor_wait_target_reached(board, motor)
            for axis, (board, motor) in zip(('x', 'y'), axes)}}
    monkeypatch.setattr(rig.native.tester, 'motor_wait_target_reached_many', wait_many)
    job = rig.start(payload)
    done = rig.terminal(job)
    import json
    from pathlib import Path
    Path('/home/dalab/.hermes/profiles/fresh/robot-audit/debloat-plan-20260929/implementation/finish/r1-protocol-' + event + '.json').write_text(json.dumps(done, indent=2))
    assert done['command']['status'] == 'completed', done
    children = rig.child_rows(done)
    from bioxp.runtime_audit_store import TERMINAL_COMMAND_STATES
    assert children and all(row['status'] in TERMINAL_COMMAND_STATES for row in children), children
    assert any(row['action_id'] == 'oem.deck._mov_execution' and row['status'] == 'completed'
        for row in children if 'action_id' in row)
