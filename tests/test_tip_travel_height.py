"""OEM TipLoaded setter and observed workflow pickup travel regression (offline)."""
from types import SimpleNamespace
import pytest
from tests.test_debloat_core import named  # noqa: F401
from tests.test_wake_setup_debloat import rig  # noqa: F401


def pickup(named, monkeypatch):
    """Eject, then an observed workflow tip pickup with lift_z=false (no Home)."""
    run, p, store, _, frames, _ = named
    p.publish_pipette_owner_state(tip_loaded=True, tip_dirty=False, tip_location=-1, source_command_id='old-tips')
    p.wp8_clear_tip_loaded('clearTipLoaded', {}, command_id='eject', child_order=0, plan_digest='test')
    assert store.deck_semantic_state()['pseudo_z_home'] == 65000
    assert p._load_state()['machine_status']['psudo_z_home_steps'] == 65000
    p.publish_tip_tray_transition(tray_id=0, transition='load_all', operation_id='seed', command_id='seed', provenance={})
    channels = [SimpleNamespace(_tip_loaded=False) for _ in range(4)]
    def query():
        for ch in channels:
            ch._tip_loaded = True
        return {'ok': True, 'channels': [{'channel': i, 'tip_loaded': True, 'result': {
            'ok': True, 'semantic_ok': True, 'hardware_truth_level': 'hardware_query'}} for i in range(4)]}
    t = SimpleNamespace(_transports=channels, query_tip_status_all=query,
        loadTip=lambda *a: {'ok':True}, _tip_type=50)
    monkeypatch.setattr(p.primitives, 'pipette_transport', t, raising=False)
    p.bind_pipette_collection_state_reader(lambda: {'tip_exists': True})
    monkeypatch.setattr(p, 'tip_tray_type', lambda _: 'T50')
    monkeypatch.setattr(p, '_manual_pipette_receipt_runner', lambda name, call, *a: call(t), raising=False)
    monkeypatch.setattr(p, '_wp8_execution_fence_checker', lambda *a, **k: None, raising=False)
    nested = []
    def finite(*, plan, **kw):
        nested.append(plan['operation'])
        return {'ok': True}
    # Only pickup positioning leaves are replaced. The subsequent travel uses
    # the real queued executor/provider/adapter/native driver and inert wire.
    with monkeypatch.context() as leaves:
        leaves.setattr(p, '_wp8_execute_nested_plan', finite)
        leaves.setattr(p.primitives, 'z_pipette_position', lambda **kw: {'ok': True, 'target_position_steps':141531})
        result = p.wp8_manual_pipette_physical('sourceManualLoadTip',
            {'tray':1, 'well':'A10', 'lift_z':False}, command_id='pickup', child_order=0,
            plan_digest='test', owner_identity={'source_identity':'pickup:0'})
    assert result['ok'], result
    assert 'pipette_home' not in nested
    assert store.deck_semantic_state()['tip_loaded'] is True
    assert store.deck_semantic_state()['pseudo_z_home'] == 500
    assert p._load_state()['machine_status']['tip_loaded'] is True
    assert p._load_state()['machine_status']['psudo_z_home_steps'] == 500
    # lift_z=false leaves the head down in the rack.
    p.primitives.tester.motor_move_absolute(4, 141531, motor=1)


def test_eject_pickup_move_without_home(named, monkeypatch):
    run, p, store, _, frames, _ = named
    pickup(named, monkeypatch)
    # A named deck move runs ForceToHighHome itself, so this ordering does not
    # depend on the pickup's travel height; the workflow well move below does.
    result, _ = run('LOC_OC', key='after-pickup')
    assert result['ok'], result
    targets = [(board,motor,value) for board,cmd,typ,motor,value in frames if cmd == 4]
    assert targets[0] == (4, 1, 500), targets
    assert any((board,motor) != (4,1) for board,motor,value in targets[1:]), targets


def test_workflow_well_move_after_pickup_raises_z_before_xy(named, monkeypatch):
    """The move that struck hardware: pickup (lift_z=false) then a workflow well move.

    manual_pipette_move compiles scriptmoveTo + updateLocation only. Nothing in
    it forces the travel height; it is whatever the pickup published.
    """
    from bioxp.manual_pipetting import manual_position_plan
    from bioxp.oem_deck_movement import make_wp8_operation_executor
    _, p, store, _, frames, _ = named
    pickup(named, monkeypatch)
    stamps = p.deck_owner_authority_stamps()
    # The replaced pickup positioning leaf would have published the rack station.
    store.publish_deck_owner_state(source_operation='updateLocation', source_command_id='pickup-station',
        updates={'current_location': 'TECANRACK2', 'current_well': 9}, **stamps)
    plan = manual_position_plan({'operation': 'move', 'location_id': 1, 'well': 'A1', 'position_flag': 1})
    assert [c['operation'] for c in plan['children']] == ['scriptmoveTo', 'updateLocation']
    store.bind_workflow_dispatcher(lambda command: None)
    store.admit_workflow(command_id='parent', idempotency_key='parent', plan_fingerprint='fixture',
        requested_inputs={'bundle': {}}, ownership_generation=7,
        resources=('axis:x', 'axis:y', 'axis:z'), board_epochs={})
    assert store.claim_next()['command_id'] == 'parent'
    assert store._renew_owner(lease_seconds=600.0)
    admission = {'ownership_generation': 7, 'serial206_initialization_provider': {
        'x_authority': {'current_board_lifecycle_generation': stamps['board_epoch_5']},
        'board4_authority': {'active_board_epoch': stamps['board_epoch_4']}}}
    execute = make_wp8_operation_executor(provider_getter=lambda: p, command_store=store)
    with store.workflow_context('parent', source_occurrence_id='manual:move-after-pickup'):
        child = store.admit_internal_wp8_operation(plan['operation'], inputs={}, state=admission,
            idempotency_key='move-after-pickup', prepared_plan=plan)
        claimed = store.claim_next()
        assert claimed['command_id'] == child['command_id']
        frames.clear()
        result = execute(command_id=child['command_id'], plan=plan)
    assert result['ok'], result
    targets = [(board, motor, value) for board, cmd, typ, motor, value in frames if cmd == 4]
    assert targets[0] == (4, 1, 500), targets
    assert any((board, motor) != (4, 1) for board, motor, value in targets[1:]), targets
    assert store.deck_semantic_state()['current_location'] == 'LOC_OC'


@pytest.mark.parametrize('plate,held', [(None,65000),(3,65000),(4,500)])
def test_no_tips_never_lowers_height_while_plate_held(named, plate, held):
    _, p, store, _, _, _ = named
    def heights():
        return (store.deck_semantic_state()['pseudo_z_home'],
                p._load_state()['machine_status']['psudo_z_home_steps'])
    # DefaultParameters.GantryLoad(null, plate): explicit plate rule.
    p.gantry_load(tip_loaded=False, plate_on_gantry=plate)
    assert heights() == (held, held)
    # Routine tip-status query during the carry: nothing changed, height untouched.
    p.publish_pipette_owner_state(tip_loaded=False, tip_dirty=False, tip_location=-1, source_command_id='status-query')
    assert heights() == (held, held)
    p.publish_pipette_owner_state(tip_loaded=True, tip_dirty=False, tip_location=-1, source_command_id='pickup')
    assert heights() == (500, 500)
    # Tips reported gone: 65000 only with an empty gripper. With a plate
    # recorded on the gantry the height does not change.
    p.publish_pipette_owner_state(tip_loaded=False, tip_dirty=False, tip_location=-1, source_command_id='eject')
    expected = 65000 if plate is None else 500
    assert heights() == (expected, expected)
    assert store.deck_semantic_state()['plate_on_gantry'] == plate
    # The OEM clear site is the TipLoaded setter verbatim: GantryLoad(null,null).
    p.wp8_clear_tip_loaded('clearTipLoaded', {}, command_id='cleanup', child_order=0, plan_digest='test')
    assert heights() == (65000, 65000)
    # Release keeps its own rule: LoadGantry(null) -> GantryLoad(null,null).
    p.gantry_load(tip_loaded=False, plate_on_gantry=None, source_operation='LoadGantry')
    assert heights() == (65000, 65000)


def test_repeated_no_tips_observation_does_not_touch_height(named):
    _, p, store, _, _, _ = named
    # Fixture: no tips, then ForceToHighHome (500).
    assert store.deck_semantic_state()['tip_loaded'] is False
    assert store.deck_semantic_state()['pseudo_z_home'] == 500
    before = p._load_state()['machine_status'].get('psudo_z_home_steps')
    p.publish_pipette_owner_state(tip_loaded=False, tip_dirty=False, tip_location=-1, source_command_id='status-query')
    assert store.deck_semantic_state()['pseudo_z_home'] == 500
    assert p._load_state()['machine_status'].get('psudo_z_home_steps') == before


def test_dirty_only_publication_preserves_force_high_home(named):
    _, p, store, _, _, _ = named
    p.publish_pipette_owner_state(tip_loaded=False, tip_dirty=False, tip_location=-1, source_command_id='empty')
    p._publish_deck_owner_state(source_operation='sourceForceToHighHome', source_command_id='force', updates={'pseudo_z_home':500})
    p.wp8_pipette_source_leaf('sourceTipState', {'changes': {'tip_dirty':False}},
        command_id='dirty', child_order=0, plan_digest='test')
    assert store.deck_semantic_state()['pseudo_z_home'] == 500
    assert store.deck_semantic_state()['tip_loaded'] is False
