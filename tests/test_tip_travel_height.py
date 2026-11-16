"""OEM TipLoaded setter and observed workflow pickup travel regression (offline)."""
from types import SimpleNamespace
import pytest
from tests.test_debloat_core import named  # noqa: F401
from tests.test_wake_setup_debloat import rig  # noqa: F401


def test_eject_pickup_move_without_home(named, monkeypatch):
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
    monkeypatch.setattr(p, '_wp8_execute_nested_plan', finite)
    # Only pickup positioning leaves are replaced. The subsequent named travel
    # uses the real queued executor/provider/adapter/native driver and inert wire.
    monkeypatch.setattr(p.primitives, 'z_pipette_position', lambda **kw: {'ok': True, 'target_position_steps':141531})
    result = p.wp8_manual_pipette_physical('sourceManualLoadTip',
        {'tray':1, 'well':'A10', 'lift_z':False}, command_id='pickup', child_order=0,
        plan_digest='test', owner_identity={'source_identity':'pickup:0'})
    assert result['ok'], result
    assert 'pipette_home' not in nested
    assert store.deck_semantic_state()['tip_loaded'] is True
    assert store.deck_semantic_state()['pseudo_z_home'] == 500
    assert p._load_state()['machine_status']['tip_loaded'] is True
    p.primitives.tester.motor_move_absolute(4, 141531, motor=1)
    result, _ = run('LOC_OC', key='after-pickup')
    assert result['ok'], result
    targets = [(board,motor,value) for board,cmd,typ,motor,value in frames if cmd == 4]
    assert targets[0] == (4, 1, 500), targets
    assert any((board,motor) != (4,1) for board,motor,value in targets[1:]), targets


@pytest.mark.parametrize('plate,height', [(None,65000),(3,65000),(4,500)])
def test_tip_setter_does_not_replace_explicit_plate_rule(named, plate, height):
    _, p, store, _, _, _ = named
    p.gantry_load(tip_loaded=False, plate_on_gantry=plate)
    assert store.deck_semantic_state()['pseudo_z_home'] == height
    p.publish_pipette_owner_state(tip_loaded=True, tip_dirty=False, tip_location=-1, source_command_id='pickup')
    assert store.deck_semantic_state()['pseudo_z_home'] == 500
    p.publish_pipette_owner_state(tip_loaded=False, tip_dirty=False, tip_location=-1, source_command_id='eject')
    assert store.deck_semantic_state()['pseudo_z_home'] == 65000
    assert store.deck_semantic_state()['plate_on_gantry'] == plate


def test_dirty_only_publication_preserves_force_high_home(named):
    _, p, store, _, _, _ = named
    p.publish_pipette_owner_state(tip_loaded=False, tip_dirty=False, tip_location=-1, source_command_id='empty')
    p._publish_deck_owner_state(source_operation='sourceForceToHighHome', source_command_id='force', updates={'pseudo_z_home':500})
    p.wp8_pipette_source_leaf('sourceTipState', {'changes': {'tip_dirty':False}},
        command_id='dirty', child_order=0, plan_digest='test')
    assert store.deck_semantic_state()['pseudo_z_home'] == 500
    assert store.deck_semantic_state()['tip_loaded'] is False
