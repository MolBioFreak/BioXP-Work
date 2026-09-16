"""Actual API cleanup binding, canonical FIFO and reopened SQLite; no devices."""
import threading
import pytest
from bioxp import api
from bioxp.protocols.runtime_state import ProtocolRuntimeState, ProtocolWorkflowState
from bioxp.operator_command_plane import OperatorCommandStore
from tests.test_protocol_oem_bindings import document
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig


@pytest.mark.parametrize('door_ok', [True, False])
def test_actual_api_cleanup_requires_this_source_return_and_persists(query_rig, retained_rig, monkeypatch, door_ok):
    from tests.test_deck_complete_admission import ready
    app, provider, primitive, references, root, _, calls, _, transport = query_rig
    ready((app, provider, primitive, references, root), monkeypatch, retained_rig)
    store = app.state.operator_command_plane.store
    stamps = provider.deck_owner_authority_stamps()
    from tests.test_deck_scoped_integration import qualify_full_predecessor
    primitive.calls.clear()  # helper checks queries for this fixture-preparation step
    machine_fixture = provider._load_state()
    machine_fixture['machine_status']['GripperVersion'] = 1
    provider._save_state(machine_fixture)  # explicit synthetic device configuration
    qualify_full_predecessor((provider, primitive, retained_rig[2], references, store, root))
    boot = provider.refresh_deck_semantic_bootstrap(expected_generation=stamps['ownership_generation'])
    assert boot == {'status': 'retained', 'semantic_state_revision': 1}
    assert store.deck_semantic_state()['current_location'] == 'LOC_PARK'
    store.publish_deck_owner_state(source_operation='pipette_owner', source_command_id='fixture',
        updates={'tip_loaded': True, 'tip_dirty': False, 'tip_location': -1}, **stamps)
    monkeypatch.setattr(provider.primitives, 'pipette_transport', transport, raising=False)
    monkeypatch.setattr(provider.primitives, 'pipette_audit_runner', api._run_serial206_pipette_audit, raising=False)
    monkeypatch.setattr(provider.primitives, 'deck_io_query_type',
        lambda kind: {'value': 1 if kind == 0 and not door_ok else 0}, raising=False)
    monkeypatch.setattr(provider.primitives.tester, 'deck_io_set_type',
        lambda *args: {'ok': True, 'controller_command_acknowledged': True}, raising=False)
    monkeypatch.setattr(api, '_protocol_command_store', lambda: store)
    entered, release = threading.Event(), threading.Event()
    def dispatch(command):
        entered.set()
        assert release.wait(30)
    store.bind_workflow_dispatcher(dispatch)
    store.admit_workflow(command_id='cleanup-parent', idempotency_key='cleanup-parent',
        plan_fingerprint='synthetic-cleanup-fixture', requested_inputs={'bundle': {}},
        ownership_generation=stamps['ownership_generation'],
        resources=['axis:x','axis:y','axis:z','gripper','pipette'], board_epochs={})
    state = ProtocolRuntimeState(protocol_id='cleanup-fixture', job_id='cleanup-parent', dry_run=False)
    state.workflow = ProtocolWorkflowState(command_id='cleanup-parent')
    original = provider.execute_wp8_child
    trace = []
    host = {'waitStop', 'checkDoorStatus', 'queryTipStatus', 'clearTipLoaded',
            'updateLocation', 'updatePlateLocation'}
    def execute(child, **kwargs):
        trace.append(child['operation'])
        if child['operation'] in host:
            return original(child, **kwargs)
        return {'ok': True, 'delivery_attempted': True,
                'controller_command_acknowledged': True, 'controller_completion_verified': True}
    monkeypatch.setattr(provider, 'execute_wp8_child', execute)
    bindings = api._protocol_bindings({'protocol': {'document': document('sweep').to_payload()}})
    cleanup = bindings[2]['cleanup']
    app.state.operator_command_plane.start()
    try:
        assert entered.wait(5)
        assert provider._wp8_stop_event.is_set()
        with pytest.raises(RuntimeError, match='cleanup_source_script_not_returned'):
            cleanup(state)
        assert not trace and not calls
        bindings.source_script_begin(state)
        assert not provider._wp8_stop_event.is_set()
        with pytest.raises(RuntimeError, match='cleanup_source_script_not_returned'):
            cleanup(state)
        bindings.source_script_returned(state)
        result = cleanup(state)
        import json
        if not result['ok']:
            pytest.fail(json.dumps({'trace': trace, 'result': result}, default=str))
        assert not provider._wp8_stop_event.is_set()
        assert trace[:2] == ['waitStop', 'checkDoorStatus']
        assert ('clearTipLoaded' in trace) is door_ok
        assert ('sendGripperHome' in trace) is door_ok
        assert calls == ([0,1,2,3] if door_ok else [])
        assert 'ejectAllTipsCleanup' not in trace  # latest query says no tips
        assert 'catchPlate' not in trace  # both covers already at their sources
        rows = store.connection.execute('SELECT command_id,status FROM operator_commands WHERE parent_command_id=? ORDER BY sequence', ('cleanup-parent',)).fetchall()
        assert rows and all(row['status'] in ('completed','observed') for row in rows)
        fresh = OperatorCommandStore(root)
        try:
            assert fresh.deck_semantic_state()['tip_loaded'] is (not door_ok)
            persisted = fresh.connection.execute('SELECT command_id,status FROM operator_commands WHERE parent_command_id=? ORDER BY sequence', ('cleanup-parent',)).fetchall()
            assert [dict(row) for row in rows] == [dict(row) for row in persisted]
        finally:
            fresh.stop()
        # A consumed source event is not authority for another cleanup. Guard
        # must refuse before even collection, not wait for an ignored timeout.
        monkeypatch.setattr(provider, 'wp8_operation_machine_state',
            lambda *args: pytest.fail('consumed event reached machine collection'))
        with pytest.raises(RuntimeError, match='cleanup_source_script_not_returned'):
            cleanup(state)
        bindings.source_script_returned(state)
        assert not provider._wp8_stop_event.is_set()
    finally:
        release.set()
        app.state.operator_command_plane.stop()
