"""Source-only preparation publication; no hardware acceptance claim."""
from contextlib import nullcontext
from types import SimpleNamespace
from tests.test_protocol_workflow_child_admission import store
from bioxp.oem_deck_movement import compile_finite_plate_operation, make_wp8_operation_executor


def test_force_high_changes_only_source_pseudo_home_and_preserves_unknowns(store):
    stamps = dict(ownership_generation=1, board_epoch_4=2, board_epoch_5=3)
    store.bind_deck_owner_authority_reader(lambda: stamps, scope=nullcontext)
    store.publish_deck_owner_state(source_operation='pipette_owner', source_command_id='fixture-custody',
        updates={'tip_loaded': True, 'tip_dirty': True, 'tip_location': 2}, **stamps)
    before = dict(store.connection.execute('SELECT * FROM operator_plane_deck_semantic_state').fetchone())
    result = store.publish_deck_owner_state(source_operation='sourceForceToHighHome',
        source_command_id='source-high-home', updates={'pseudo_z_home': 500}, **stamps)
    after = dict(store.connection.execute('SELECT * FROM operator_plane_deck_semantic_state').fetchone())
    assert result['pseudo_z_home'] == after['pseudo_z_home'] == 500
    for field in ('tip_loaded','tip_dirty','tip_location','clean_path','plate_on_gantry','current_location','current_well'):
        assert after[field] == before[field]
    assert before['clean_path'] is None and after['clean_path'] is None


def test_force_high_plan_keeps_fences_without_controller_delivery_identity():
    plan = compile_finite_plate_operation('preparation_force_high_home', source_leaf_available=True)
    assert [x['operation'] for x in plan['children']] == ['sourceForceToHighHome']
    assert plan['children'][0]['state_mutation'] == {'pseudo_z_home': 500}
    calls, completions = [], []
    def forbidden(*args, **kwargs):
        raise AssertionError('software publication must not record a hardware delivery')
    ledger = SimpleNamespace(
        assert_deck_execution_current=lambda *a, **kw: calls.append(kw['boundary']),
        persist_wp8_plan=lambda *a, **kw: None,
        terminalize_wp8_child=lambda *a, **kw: completions.append(kw),
        persist_wp8_state_mutation=lambda *a, **kw: None,
        record_delivery_attempt=forbidden,
        assert_wp8_background_tasks_settled=lambda *a: calls.append('background_settled'),
    )
    def publish(child, **identity):
        assert '_delivery_identity' not in child
        assert identity['command_id'] == 'source-only'
        return {'ok': True, 'delivery_attempted': False, 'semantic_state_committed': True}
    provider = SimpleNamespace(execute_wp8_child=publish, movement_lease=nullcontext)
    result = make_wp8_operation_executor(provider_getter=lambda: provider, command_store=ledger)(
        command_id='source-only', plan=plan)
    assert result['ok'] is True and result['delivery_attempted'] is False
    assert completions[0]['state'] == 'completed'
    assert completions[0]['dispatch_attempt_id'] is None
    assert 'before_plan_write' in calls and 'after_provider_child_0' in calls
