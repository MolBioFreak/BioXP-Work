"""Provider witnesses: CI.stopMotor/Head.stopMotor, CI.moveXY and HomeXY IL.
No hardware constructors; real board Stop wrapper with an observed fake leaf.
"""
from types import SimpleNamespace
import threading

import pytest
from bioxp.usb_driver import BioXpTester
from bioxp.serial206_y_provider import Serial206YProvider
from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from test_serial206_xy_kernel import adapter as xy_adapter


@pytest.mark.parametrize('axis', ['y', 'z'])
@pytest.mark.parametrize('present,initialized,no24v', [
    (False, False, False), (False, False, True), (False, True, False), (False, True, True),
    (True, False, False), (True, False, True), (True, True, True),
])
def test_stop_presence_precedes_board_no24v_then_initialization(axis, present, initialized, no24v):
    driver = object.__new__(BioXpTester)
    calls = []
    driver._oem_board_present = lambda b: calls.append(('present', b)) or present
    driver._oem_board_state = lambda: calls.append(('initialized',)) or {4: initialized}
    driver.oem_no24v_state = lambda: calls.append(('no24v',)) or no24v
    driver.motor_oem_stop_exact = lambda *a, **k: pytest.fail('unexpected leaf Stop')
    provider = stop_provider(axis, driver)
    if present and no24v:
        with pytest.raises(RuntimeError, match='Lost 24V power'):
            provider()
        assert calls == [('present', 4), ('no24v',)]
    else:
        row = provider()
        assert row['ok'] is True
        assert row['source_call_completed'] is True
        assert row['controller_command_acknowledged'] is False
        assert row['physical_effect_verified'] is False
        assert calls == ([('present', 4), ('no24v',), ('initialized',)] if present else [('present', 4)])


def stop_provider(axis, driver):
    if axis == 'y':
        return Serial206YProvider(driver, state_store=None, generation_provider=lambda: 1).stop
    obj = object.__new__(Serial206ProductionPrimitiveAdapter)
    obj.tester = driver
    return obj.z_stop


@pytest.mark.parametrize('axis', ['y', 'z'])
@pytest.mark.parametrize('second,scalar', [({'status': 100}, 0), ({'status': 2}, 1), (None, 0)])
def test_stop_normal_void_return_is_not_leaf_ack(axis, second, scalar):
    driver = object.__new__(BioXpTester)
    calls = []
    driver._oem_board_present = lambda b: True
    driver._oem_board_state = lambda: {4: True}
    driver.oem_no24v_state = lambda: False
    def leaf(board, motor=0):
        calls.append((board, motor))
        return {'ok': scalar == 0, 'source_return_code': scalar,
                'source_call_completed': True, 'first_delivery': {'status': 100}, 'second_delivery': second}
    driver.motor_oem_stop_exact = leaf
    row = stop_provider(axis, driver)()
    assert calls == [(4, 0 if axis == 'y' else 1)]
    assert row['ok'] is True
    assert row['source_board_return'] is None
    assert row['source_return_code'] == scalar
    assert row['controller_command_acknowledged'] is (second == {'status': 100})
    assert row['controller_terminal_state_verified'] is False
    assert row['physical_effect_verified'] is False


@pytest.mark.parametrize('axis', ['y', 'z'])
def test_stop_post_delivery_power_exception_is_not_success(axis):
    driver = object.__new__(BioXpTester)
    delivered = []
    driver._oem_board_present = lambda b: True
    driver._oem_board_state = lambda: {4: True}
    driver.oem_no24v_state = lambda: bool(delivered)
    def leaf(board, motor=0):
        delivered.append((board, motor))
        return {'ok': True, 'source_call_completed': True, 'source_return_code': 0}
    driver.motor_oem_stop_exact = leaf
    with pytest.raises(RuntimeError, match='stopMotor2'):
        stop_provider(axis, driver)()
    assert delivered == [(4, 0 if axis == 'y' else 1)]


@pytest.mark.parametrize('axis', ['y', 'z'])
@pytest.mark.parametrize('second', [{'status':100}, {'status':2}, None])
def test_provider_to_real_stop_leaf_exact_two_deliveries_without_proof_queries(axis, second):
    driver = object.__new__(BioXpTester)
    calls = []
    driver._oem_board_present = lambda b: True
    driver._oem_board_state = lambda: {4:True}
    driver.oem_no24v_state = lambda: False
    def wire(*args, **kwargs):
        calls.append((args, kwargs))
        return {'status':100} if len(calls) == 1 else second
    driver._send_motor = wire
    row = stop_provider(axis, driver)()
    motor = 0 if axis == 'y' else 1
    assert [a for a,k in calls] == [(4,3,0,motor,0), (4,3,0,motor,0)]
    assert all(k == {'attempts':1,'wait_reply':True,'write_timeout_ms':55,
                     'read_timeout_ms':60000,'max_reads':1,'strict_match':True,
                     'allow_recover':False,'ordinary_motor_retry':True} for a,k in calls)
    assert row['ok'] is True
    assert row['source_return_code'] == (1 if second == {'status':2} else 0)
    assert row['controller_command_acknowledged'] is (second == {'status':100})
    assert row['controller_terminal_state_verified'] is False


def test_xy_source_reads_y_before_x_and_ack_only_parameter_evidence(monkeypatch):
    obj = xy_adapter()
    obj.tester._oem_board_present = lambda board: True
    obj._x_profile = lambda: {"axis_max_steps": 90263, "board": 5, "motor": 0}
    calls = []
    original = obj.tester.motor_get_position
    obj.tester.motor_get_position = lambda b, **k: calls.append(('position', b)) or original(b, **k)
    obj.tester.motor_set_axis_param = lambda b, p, v, **k: calls.append(('sap', b, p, v)) or {'ok': True, 'ack': {'status': 100}, 'readback': None}
    monkeypatch.setattr('bioxp.oem_serial206_initialization.time.sleep', lambda s: calls.append(('sleep', s)))
    row = obj.move_xy(15000, 6000, wait_timeout_s=5)
    assert calls[:4] == [('position', 4), ('position', 5), ('sap', 5, 5, 400), ('sap', 4, 5, 400)]
    assert [r for r in calls if r[0] == 'sleep'] == [('sleep', .125), ('sleep', .005)]
    assert row['acceleration_setup_verified'] is True
    assert row['acceleration_restore_verified'] is True
    assert row['acceleration_evidence_kind'] == 'controller_ack_not_parameter_readback'


@pytest.mark.parametrize('context,sequential', [(None, False), ('ClassControlInterface.btnLOC1_Click', True)])
@pytest.mark.parametrize('wait_error', [False, True])
def test_xy_sealed_location_context_and_source_exception_restore_order(monkeypatch, context, sequential, wait_error):
    obj = xy_adapter(reference=False)
    calls = []
    obj._x_issue_absolute = lambda value, **kw: calls.append(('move', 'x', value)) or {'ok': True, 'command_issued': True}
    obj._move_xy_y_issue_absolute = lambda value, **kw: calls.append(('move', 'y', value)) or {'ok': True, 'command_issued': True}
    obj.tester.motor_set_axis_param = lambda b,p,v,**kw: calls.append(('sap', b,p,v)) or {'ack': {'status':100}}
    def pair(targets, **kw):
        calls.append(('wait', tuple(targets), kw['timeout_s'], kw['sta_sequential']))
        if wait_error:
            raise RuntimeError('wait exception')
        return {'ok':False, 'per_axis': {'x':{'ok':False}, 'y':{'ok':False}}}
    obj.tester.motor_wait_target_reached_many = pair
    monkeypatch.setattr('bioxp.oem_serial206_initialization.time.sleep', lambda s: calls.append(('sleep',s)))
    if wait_error:
        with pytest.raises(RuntimeError, match='wait exception'):
            obj.move_xy(6000, 15000, wait_timeout_s=999, source_context=context)
    else:
        row = obj.move_xy(6000, 15000, wait_timeout_s=999, source_context=context)
        assert row['ok'] is True  # Source logs timeout and returns normally.
        assert row['controller_terminal_state_verified'] is False
        assert row['source_context_sealed'] is (context is not None)
    assert calls == [
        ('sap',5,5,350),('sap',4,5,750),('move','y',15000),
        ('sleep',.125),('move','x',6000),('sleep',.005),
        ('wait',((5,0),(4,0)),5.0,sequential),
    ] + ([] if wait_error else [('sap',5,5,350),('sap',4,5,400)])


def test_actual_named_location_provider_seals_sta_through_move_to(monkeypatch):
    from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider
    from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot
    bind_serial206_oem_snapshot(monkeypatch)
    obj = xy_adapter(reference=False)
    obj.y_provider = None
    obj._read_axis_position = lambda axis: 0
    obj._x_issue_absolute = lambda *a, **k: {'ok':True, 'command_issued':True}
    obj._move_xy_y_issue_absolute = lambda *a, **k: {'ok':True, 'command_issued':True}
    waits = []
    def pair(targets, **kw):
        waits.append((tuple(targets), kw['sta_sequential'], kw['timeout_s']))
        return {'ok':False}
    obj.tester.motor_wait_target_reached_many = pair
    monkeypatch.setattr('bioxp.oem_serial206_initialization.time.sleep', lambda s: None)
    provider = object.__new__(Serial206OemInitializationProvider)
    provider.primitives = obj
    provider._deck_execution_semantics = lambda snapshot: {'pseudo_z_home':65000, 'tip_loaded':False, 'plate_on_gantry':None}
    row = provider.moveTo(location_id=2, authority_snapshot={})
    assert row['ok'] is True
    assert waits == [(((5,0),(4,0)), True, 5.0)]
    xy = row['primitive_result']['operations'][0]
    assert xy['source_context'] == 'ClassControlInterface.btnLOC1_Click'
    assert xy['source_context_sealed'] is True
    assert xy['controller_terminal_state_verified'] is False


@pytest.mark.parametrize('missing', ['switch', 'speed', 'set_home', 'position', 'cached_noop', None])
def test_y_home_evidence_does_not_promote_source_scalars_or_null_replies(missing):
    provider = Serial206YProvider(object(), state_store=None, generation_provider=lambda: 1)
    home = {
        'home_hit': {'home':True, 'reply_valid':True, 'ack':{'status':100}},
        'stop': {'ok':True, 'first_delivery':{'status':100}, 'second_delivery':{'status':100}},
        'wait': {'stopped':True, 'last_speed':0, 'last_ack':{'status':100},
                 'speed_reply_valid':True, 'controller_terminal_state_verified':True},
        'set_home': {'controller_command_acknowledged':True, 'ack':{'status':100}},
        'position_after_sethome': {'ok':True, 'position':0, 'ack':{'status':100}},
    }
    home['home_after'] = home['home_hit']
    if missing == 'switch': home['home_hit']['ack'] = None
    if missing == 'speed': home['wait']['last_ack'] = None
    if missing == 'set_home': home['set_home']['ack'] = None
    if missing == 'position': home['position_after_sethome']['ack'] = None
    if missing == 'cached_noop':
        home.update(source_noop=True, home_decision={'source_short_circuit':'MotorHome_and_CurrentPosition_zero'})
    proof = provider._home_proof(home, {})
    complete = all(proof[k] for k in ('home_predicate_active','stop_complete','speed_zero','set_home_valid','zero_readback'))
    assert complete is (missing is None)


@pytest.mark.parametrize('child_error', [False, True])
def test_homexy_task_join_restore_and_no_extra_proof_reads(child_error):
    obj = object.__new__(Serial206ProductionPrimitiveAdapter)
    calls = []
    both = threading.Barrier(2)
    def home(axis, **kw):
        assert kw == {'speed': 200, 'rehome': False, 'timeout_s': 30.0, 'require_switch_transition': False}
        calls.append(('start', axis))
        both.wait(timeout=1)
        calls.append(('finish', axis))
        if child_error and axis == 'x':
            raise RuntimeError('home child failed')
        return {'ok': True, 'source_return_code': 12 if axis == 'x' else 34,
                'source_noop': True, 'controller_home_proof_verified': False}
    obj.tester = SimpleNamespace(
        _oem_board_present=lambda b: True,
        motor_set_axis_param=lambda b,p,v,**k: calls.append(('sap',b,p,v)) or {'ok':True,'ack':{'status':100}},
        motor_oem_go_home=home,
        motor_get_position=lambda *a,**k: pytest.fail('HomeXY has no post-restore GAP proof queries'),
    )
    if child_error:
        with pytest.raises(RuntimeError, match='home child failed'):
            obj.home_xy()
    else:
        row = obj.home_xy()
        assert row['ok'] is True
        assert row['source_return'] == {'x':12, 'y':34}
        assert row['positions'] == {}
        assert row['reference_publication_required'] is False
        assert row['controller_command_acknowledged'] is False
    writes = [r for r in calls if r[0] == 'sap']
    assert writes[:4] == [('sap',5,4,200),('sap',5,5,200),('sap',4,4,200),('sap',4,5,200)]
    assert writes[4:] == ([] if child_error else [('sap',5,4,1700),('sap',5,5,350),('sap',4,4,1800),('sap',4,5,400)])
    assert {r[1] for r in calls[4:6]} == {'x','y'}
    assert all(r[0]=='finish' for r in calls[6:8])
