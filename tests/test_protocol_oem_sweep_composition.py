"""Real API/provider/compiler sweep composition; synthetic inventory, native doubles.

The canonical case exercises real parent/child SQLite publication with physical
leaves doubled; lightweight recorder cases separately cover ordering and faults.
No lifecycle, sweep handler, model selector or provider callback is replaced.
The canonical case remains unskipped: unowned publication failures are blockers.
"""
from types import SimpleNamespace

import pytest

from bioxp import api
from bioxp.protocols.runtime_state import ProtocolSourceModel, SourceTray, SourceWell
from bioxp.services.pipette_service import build_oem_pipette_handlers, build_oem_pipette_lifecycle_helpers
from tests.test_protocol_oem_bindings import rig, bind, document
from tests.test_protocol_oem_pipette_composites import composite_bindings
from tests.test_protocol_oem_pipette_models_sweep import sweep_bindings
from tests.test_protocol_oem_pipette import action
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig


def test_api_sweep_canonical_children_and_sqlite_with_physical_leaves_only(query_rig, retained_rig, monkeypatch):
    import threading
    from bioxp.protocols.runtime_state import ProtocolRuntimeState, ProtocolWorkflowState
    from bioxp.operator_command_plane import OperatorCommandStore
    from tests.test_deck_complete_admission import ready
    app, provider, primitive, references, root, _, calls, _, transport = query_rig
    ready((app, provider, primitive, references, root), monkeypatch, retained_rig)
    store = app.state.operator_command_plane.store
    stamps = provider.deck_owner_authority_stamps()
    store.publish_deck_owner_state(source_operation='pipette_owner', source_command_id='sweep-fixture',
        updates={'tip_loaded': False, 'tip_dirty': False, 'tip_location': -1}, **stamps)
    for tray in range(4):
        store.publish_tip_tray_transition(tray_id=tray, transition='construct',
            operation_id=f'fixture-{tray}', command_id=f'fixture-{tray}',
            provenance={'synthetic_test_inventory': True}, **stamps)
    monkeypatch.setattr(provider.primitives, 'pipette_transport', transport, raising=False)
    monkeypatch.setattr(api, '_protocol_command_store', lambda: store)
    native = []
    original = provider.execute_wp8_child
    physical = {'scriptmoveTo', 'sourceHomeZ', 'sourceZCurrent', 'sourceMoveZ', 'sourceMoveX'}
    def execute(child, **kwargs):
        if child['operation'] in physical:
            native.append(child)
            return {'ok': True, 'delivery_attempted': True,
                    'controller_command_acknowledged': True, 'controller_completion_verified': True}
        return original(child, **kwargs)
    monkeypatch.setattr(provider, 'execute_wp8_child', execute)
    ejections = []
    monkeypatch.setattr(transport, 'eject_all_tips', lambda **kw: ejections.append(kw) or {
        'ok': True, 'delivery_attempted': True, 'controller_command_acknowledged': True})
    entered, release = threading.Event(), threading.Event()
    def dispatch(command):
        entered.set()
        assert release.wait(30)
    store.bind_workflow_dispatcher(dispatch)
    store.admit_workflow(command_id='sweep-parent', idempotency_key='sweep-parent',
        plan_fingerprint='synthetic-sweep-fixture', requested_inputs={'bundle': {}},
        ownership_generation=stamps['ownership_generation'], resources=['pipette'], board_epochs={})
    state = ProtocolRuntimeState(protocol_id='sweep-fixture', job_id='sweep-parent', dry_run=False)
    state.workflow = ProtocolWorkflowState(command_id='sweep-parent')
    state.source_model = inventory([])
    state.source_model.tip_trays[0].wells[0].content = 'rm'
    _, handlers, lifecycle = bind(document('sweep'))
    app.state.operator_command_plane.start()
    try:
        assert entered.wait(5)
        assert handlers['sweep'](document('sweep').stages[0].actions[0], state)['ok']
        assert not native and not calls
        try:
            result = lifecycle['epilogue_sweep'](state)
        except Exception as exc:
            import json
            pytest.fail(json.dumps(getattr(exc, 'provider_results', {'error': str(exc)}), default=str))
        assert result['ok'], result
        assert len(ejections) == 1 and calls == [0, 1, 2, 3] * 2
        rows = store.connection.execute('SELECT command_id,status FROM operator_commands WHERE parent_command_id=? ORDER BY sequence', ('sweep-parent',)).fetchall()
        assert len(rows) == 11
        assert all(row['status'] in ('completed', 'observed') for row in rows)
        assert len({row['command_id'] for row in rows}) == 11
        fresh = OperatorCommandStore(root)
        try:
            tip = fresh.tip_tray_state(0)
            assert all(tip['occupancy'][i] is False for i in (0, 24, 48, 72))
            assert tip['command_id'] in {row['command_id'] for row in rows}
            assert fresh.get_command('sweep-parent')['status'] == 'dispatched'
            assert fresh.deck_semantic_state()['current_well'] == 96
            assert all(fresh.get_command(row['command_id'])['status'] == row['status'] for row in rows)
        finally:
            fresh.stop()
    finally:
        release.set()
        app.state.operator_command_plane.stop()


def inventory(marked):
    # Deliberately synthetic; includes partial A1, full B1, partial A2 per tray.
    trays = [SourceTray(str(i), 7 + i, [SourceWell(None, 0, 0, False) for _ in range(96)]) for i in range(4)]
    for tray in trays:
        for index in marked:
            tray.wells[index].content = "rm"
    return ProtocolSourceModel(tip_trays=trays)


@pytest.mark.parametrize("marked,ordinary,epilogue", [
    ([0], [], ["A1"]),
    ([0, 24, 48, 72], ["A1"], ["A1"]),
    ([], [], []),
    ([0, 12, 36, 60, 84, 1], ["B1"], ["A1", "B1", "A2"]),
])
@pytest.mark.parametrize("lifecycle", [False, True])
def test_real_selection_and_tray_order(marked, ordinary, epilogue, lifecycle, monkeypatch):
    monkeypatch.setattr("bioxp.services.pipette_service.time.sleep", lambda _: None)
    args, state, native, entries, effects = sweep_bindings()
    state.source_model = inventory(marked)
    handler = build_oem_pipette_handlers(**args)["sweep"]
    if lifecycle:
        extra, *_ = composite_bindings()
        helpers = build_oem_pipette_lifecycle_helpers(
            before_native_entry=args['before_native_entry'], pipette_call=args['pipette_call'],
            source_bindings=extra['source_bindings'], settings=extra['settings'],
            move_to_waste=args['move_to_waste'], sweep_handler=handler)
        result = helpers['epilogue_sweep'](state, source_occurrence_id='lifecycle:sweep:9')
        prefix = 'lifecycle:sweep:9:'
    else:
        result = handler(action('sweep'), state)
        prefix = 'oem:7:'
    selected = epilogue if lifecycle else ordinary
    assert [r for r in effects if r[0] == 'remove'] == [('remove', tray, well) for tray in range(4) for well in selected]
    assert result['ok']
    assert all(identity.startswith(prefix) for identity in entries)
    assert len(entries) == len(set(entries)) == 11 * 4 * len(selected)
    if selected:
        assert state.source_model.logical_tip_present is True
        assert state.source_model.allow_to_stop is True
    else:
        assert native.calls == effects == entries == []


@pytest.mark.parametrize('remain', [False, True])
def test_actual_api_epilogue_partial_group_and_error_hold(rig, monkeypatch, remain):
    store, provider, state, trace = rig
    state.source_model = inventory([0])
    monkeypatch.setattr('bioxp.services.pipette_service.time.sleep', lambda _: None)
    def pipette(name, call, action, runtime, identity):
        trace.append(('pipette', identity, name))
        transport = SimpleNamespace(
            query_tip_status_all=lambda: {'ok': True, 'source_tip_exists': remain},
            eject_all_tips=lambda **kw: trace.append(('eject', kw)) or {'ok': True})
        return call(transport)
    monkeypatch.setattr(api, '_protocol_source_pipette_call', pipette)
    doc = document('sweep')
    _, handlers, lifecycle = bind(doc)
    result = handlers['sweep'](doc.stages[0].actions[0], state)
    assert result['ok'] and trace == []  # Ordinary all-four selection unchanged.
    result = lifecycle['epilogue_sweep'](state)
    assert result['ok']
    plans = [r for r in trace if r[0] == 'plan']
    assert len(plans) == 8 * 4
    assert len({r[1] for r in plans}) == len(plans)
    assert all(r[1].startswith('lifecycle:epilogue_sweep:0:') for r in plans)
    transitions = [r[1] for r in trace if r[0] == 'native' and r[1]['operation'] == 'sourceTipTransition']
    assert [r['arguments']['tray_id'] for r in transitions] == [0, 1, 2, 3]
    assert all(r['arguments']['well_ids'] == [0, 24, 48, 72] for r in transitions)
    assert all(r['arguments']['transition'] == 'remove' for r in transitions)
    assert [r[1] for r in trace if r[0] == 'eject'] == [{'check_missing_tip': True, 'wait': True}] * 4
    assert state.source_model.logical_tip_present is True
    assert state.source_model.allow_to_stop is True
    assert bool(result.get('source_pause_scripts')) is remain
    if remain:
        assert result['source_error_event'] == 'Eject tip failed'
    # A second invocation has a distinct source occurrence; no hidden counter reset.
    trace.clear()
    lifecycle['epilogue_sweep'](state)
    assert all(r[1].startswith('lifecycle:epilogue_sweep:1:') for r in trace if r[0] in ('plan', 'pipette'))


def test_actual_api_epilogue_ejection_exception_retains_partial_custody(rig, monkeypatch):
    _, _, state, trace = rig
    state.source_model = inventory([0])
    def pipette(name, call, action, runtime, identity):
        trace.append(('pipette', identity, name))
        if name == 'eject_all_tips':
            raise RuntimeError('uncertain ejection')
        return call(SimpleNamespace(query_tip_status_all=lambda: {'ok': True, 'source_tip_exists': False}))
    monkeypatch.setattr(api, '_protocol_source_pipette_call', pipette)
    _, _, lifecycle = bind(document('sweep'))
    with pytest.raises(RuntimeError, match='uncertain ejection') as caught:
        lifecycle['epilogue_sweep'](state)
    assert len(caught.value.oem_partial_results) == 7
    assert state.source_model.allow_to_stop is False
    assert state.source_model.logical_tip_present is True
    assert [r[2] for r in trace if r[0] == 'pipette'] == ['query_tip_status_all', 'eject_all_tips']
    assert not [r for r in trace if r[0] == 'plan' and r[2] in ('pipette_move_z', 'pipette_move_x')]
