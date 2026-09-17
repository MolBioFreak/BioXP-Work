"""Mechanical lifecycle source-order qualification, physical leaves doubled."""
from dataclasses import replace
from types import SimpleNamespace
import threading

import pytest

from bioxp.oem_deck_movement import (
    compile_finite_plate_operation, execute_finite_plate_operation,
    WP8_OPERATION_INTENT_KEYS, DeckExecutionFailure,
)
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider as Provider
from tests.test_protocol_oem_pipette_composites import composite_bindings
from tests.test_protocol_oem_pipette_lifecycle_parts import helpers


def rig(*, door_ok=True, stop=True, fail=None, throws=False):
    p = object.__new__(Provider)
    p._lock = threading.RLock()
    p._wp8_stop_event = threading.Event()
    p._wp8_source_script_owner = None
    p._wp8_source_script_returned = False
    state = SimpleNamespace(workflow=SimpleNamespace(command_id='parent'))
    trace, plans = [], []
    child = object()
    partial = [{'step_id': 'original-child', 'result': {'ok': False, 'receipt': 'r'}}]

    def leaf(name, **extra):
        trace.append(name)
        if name == fail:
            if throws:
                exc = DeckExecutionFailure('physical failure', delivery_attempted=True,
                                           provider_results=[{'original': 'native'}])
                exc.oem_partial_results = partial
                exc.owned_children = (child,)
                exc.detail = {'receipt': 'failed-child'}
                raise exc
            return {'ok': False, 'owned_children': (child,), 'receipt': 'failed-child'}
        return {'ok': True, 'receipt': name, **extra}

    def machine(operation, inputs):
        # Real compiler; only the physical/canonical snapshot is doubled.
        assert not p._wp8_stop_event.is_set() or operation == 'park_gantry'
        return dict(thermal_door_open=True, door_is_open=True, board_present=True, script_running=False,
                    gripper_version=1, board_test_mode=False)

    doors = [0]
    def execute(plan, identity, s):
        assert s is state and identity.startswith('lifecycle:')
        plans.append(plan)
        name = plan['operation']
        if name == 'thermal_door':
            doors[0] += 1
            name = 'door' + str(doors[0])
        if name == 'lifecycle_check_door':
            return leaf(name, door_ok=door_ok)
        return leaf(name, source_return=False)

    p.wp8_operation_machine_state = machine
    h = p.build_oem_lifecycle_handlers(
        execute_plan=execute,
        safe_stop_tip_exit=lambda s: leaf('tips'),
        cancel_source=lambda s: leaf('cancel', source_stop_scripts=stop),
        home_gripper=lambda s: leaf('home'),
        shutdown_temperature=lambda s: leaf('shutdown'),
        unlatch=lambda s: leaf('unlatch'))
    return p, state, h, trace, plans, child, partial


@pytest.mark.parametrize('stop', [False, True])
def test_safe_stop_cancellation_then_conditional_home_without_signal(stop):
    p, s, h, trace, _, _, _ = rig(stop=stop)
    p.wp8_source_script_begin(command_id='parent')
    r = h['safe_stop_exit'](s)
    assert r['ok'] and trace == ['tips', 'cancel'] + (['home'] if stop else [])
    assert not p._wp8_stop_event.is_set() and not p._wp8_source_script_returned
    p.wp8_source_script_returned(command_id='parent')
    assert p._wp8_stop_event.is_set()


@pytest.mark.parametrize('stage', ['tips', 'cancel', 'home'])
@pytest.mark.parametrize('throws', [False, True])
def test_safe_exit_failure_retains_prior_rows_and_children(stage, throws):
    p, s, h, trace, _, child, partial = rig(fail=stage, throws=throws)
    r = h['safe_stop_exit'](s)
    assert not r['ok'] and r['owned_children'] == (child,)
    assert trace == ['tips', 'cancel', 'home'][:['tips', 'cancel', 'home'].index(stage)+1]
    assert len(r['source_children']) == len(trace)
    assert not p._wp8_stop_event.is_set()
    if throws:
        assert r['source_children'][-1]['oem_partial_results'] == partial
        assert r['source_children'][-1]['provider_results'] == [{'original': 'native'}]


@pytest.mark.parametrize('stop', [None, 1, 'true'])
def test_safe_exit_requires_typed_source_stop_intent(stop):
    _, s, h, trace, *_ = rig(stop=stop)
    assert not h['safe_stop_exit'](s)['ok']
    assert trace == ['tips', 'cancel']


@pytest.mark.parametrize('door_ok', [False, True])
def test_source_error_full_order_and_one_shot_return(door_ok):
    p, s, h, trace, plans, *_ = rig(door_ok=door_ok)
    p.wp8_source_script_begin(command_id='parent')
    p.wp8_source_script_returned(command_id='parent')
    r = h['source_error'](s)
    assert r['ok'] and r['source_unlock_completed'] and r['source_stop_scripts']
    assert trace == ['shutdown', 'lifecycle_check_door'] + (['home_gripper'] if door_ok else []) + ['door1', 'park_gantry', 'door2', 'unlatch', 'pipette_color']
    assert [p for p in plans if p['operation'] == 'park_gantry'][0]['children'][0]['arguments']['rehome'] is False
    assert plans[-1]['children'][0]['arguments'] == {'r':255, 'g':255, 'b':255}
    assert not p._wp8_stop_event.is_set()
    p.wp8_source_script_returned(command_id='parent')
    with pytest.raises(RuntimeError, match='source_script_not_returned'):
        h['source_error'](s)
    assert len(trace) == (8 if door_ok else 7)
    p.wp8_source_script_begin(command_id='next')
    p.wp8_source_script_returned(command_id='next')
    with pytest.raises(RuntimeError, match='source_script_not_returned'):
        h['source_error'](s)


@pytest.mark.parametrize('condition', ['never_entered', 'not_returned', 'wrong_owner', 'consumed'])
def test_source_error_guard_precedes_all_collection_and_finally(condition):
    p, s, h, trace, plans, *_ = rig()
    if condition != 'never_entered':
        p.wp8_source_script_begin(command_id='other' if condition == 'wrong_owner' else 'parent')
    if condition in ('wrong_owner', 'consumed'):
        p.wp8_source_script_returned(command_id=p._wp8_source_script_owner)
    if condition == 'consumed':
        p._wp8_stop_event.clear()
    if condition == 'never_entered':
        p._wp8_stop_event.set()  # Construction-time signal is not a return.
    with pytest.raises(RuntimeError, match='source_script_not_returned'):
        h['source_error'](s)
    assert trace == plans == []


@pytest.mark.parametrize('stage', ['shutdown', 'lifecycle_check_door', 'home_gripper', 'door1', 'park_gantry', 'door2', 'unlatch', 'pipette_color'])
@pytest.mark.parametrize('throws', [False, True])
def test_error_try_catch_finally_failure_matrix(stage, throws):
    p, s, h, trace, _, child, partial = rig(fail=stage, throws=throws)
    p.wp8_source_script_begin(command_id='parent')
    p.wp8_source_script_returned(command_id='parent')
    r = h['source_error'](s)
    assert not r['ok'] and r['owned_children'] == (child,)
    full = ['shutdown', 'lifecycle_check_door', 'home_gripper', 'door1', 'park_gantry']
    if stage in full:
        # When failure precedes the first door, unlock is the first door call.
        expected = full[:full.index(stage)+1]
        expected += ['door2' if stage in ('door1', 'park_gantry') else 'door1', 'unlatch', 'pipette_color']
        assert trace == expected
    else:
        assert trace == full + ['door2', 'unlatch', 'pipette_color'][:['door2', 'unlatch', 'pipette_color'].index(stage)+1]
    assert r['source_unlock_completed'] is (stage not in ('door2', 'unlatch'))
    if throws:
        failures = [x for x in r['source_children'] if not x['ok']]
        assert failures[0]['oem_partial_results'] == partial
        assert failures[0]['detail'] == {'receipt': 'failed-child'}
    assert not p._wp8_stop_event.is_set()


def test_unknown_door_predicate_fails_but_unlock_runs():
    p, s, h, trace, *_ = rig(door_ok=None)
    p.wp8_source_script_begin(command_id='parent')
    p.wp8_source_script_returned(command_id='parent')
    assert not h['source_error'](s)['ok']
    assert trace == ['shutdown', 'lifecycle_check_door', 'door1', 'unlatch', 'pipette_color']


@pytest.mark.parametrize('prior', [False, True])
@pytest.mark.parametrize('failure', [None, 'prepare', 'restore', 'door', 'resume'])
def test_compound_wake_retains_prior_door_and_short_circuits(prior, failure):
    p = object.__new__(Provider)
    trace = []
    def leaf(name, **kw):
        trace.append(name)
        return {'ok': failure != name, **kw}
    p.sleep = lambda t: trace.append(('sleep', t))
    p.mov_execution_machine_state = lambda: pytest.fail('must not read door after initialization')
    p.wp8_operation_machine_state = lambda op, args: {'door_is_open': not prior}
    def execute(plan, identity, state):
        assert plan['operation'] == 'thermal_door' and plan['opening'] is prior
        return leaf('door')
    prep = {'source_prior_door_open': prior, 'source_children': [{'ok':True, 'source_return':False}, {'ok':True, 'generation':'new'}]}
    h = p.build_oem_lifecycle_handlers(execute_plan=execute,
        wake_prepare=lambda s: leaf('prepare', **prep),
        initial_check=lambda s: pytest.fail('compound owns initial check'),
        initialize_motors=lambda s: pytest.fail('compound owns initialization'),
        restore_door_model=lambda value,s: leaf('restore', value=value),
        resume_temperature=lambda s: leaf('resume'))
    r = h['wake'](object())
    expected = ['prepare', ('sleep', .04)] + (['restore'] if prior else []) + ['door', 'resume']
    if failure in expected:
        expected = expected[:expected.index(failure)+1]
    assert trace == expected
    assert r['ok'] is (failure is None or (failure == 'restore' and not prior))
    assert r['source_children'][0]['source_children'] == prep['source_children']


@pytest.mark.parametrize('prior', [None, 1, 'false'])
def test_compound_wake_requires_explicit_prior_door(prior):
    p = object.__new__(Provider)
    p.sleep = lambda *a: pytest.fail('unknown prior state')
    h = p.build_oem_lifecycle_handlers(execute_plan=lambda *a: pytest.fail('no door'),
        wake_prepare=lambda s: {'ok':True, 'source_prior_door_open':prior},
        restore_door_model=lambda *a: pytest.fail('no restore'),
        resume_temperature=lambda *a: pytest.fail('no heat'))
    with pytest.raises(RuntimeError, match='ThermalDoorOpen'):
        h['wake'](object())


def test_failed_partial_nested_custody_and_uncertainty_survive_composition():
    p, s, _, _, _, _, _ = rig()
    child = object()
    partial = [{'step_id':'entered', 'result':{'ok':False, 'outcome_unknown':True,
                                              'owned_children':(child,)}}]
    def tips(state):
        exc = RuntimeError('native_tip_failure')
        exc.oem_partial_results = partial
        raise exc
    h = p.build_oem_lifecycle_handlers(execute_plan=lambda *a: pytest.fail('not reached'),
        safe_stop_tip_exit=tips, cancel_source=lambda s: pytest.fail('not reached'),
        home_gripper=lambda s: pytest.fail('not reached'))
    r = h['safe_stop_exit'](s)
    assert not r['ok'] and r['uncertain'] and r['owned_children'] == (child,)
    assert r['source_children'][0]['oem_partial_results'] is partial
    assert r['source_children'][0]['error'] == 'native_tip_failure'


def test_source_error_collection_exception_still_runs_unlock():
    p, s, h, trace, _, _, _ = rig()
    original = p.wp8_operation_machine_state
    def machine(op, inputs):
        if op == 'lifecycle_check_door':
            raise RuntimeError('canonical_collection_refused')
        return original(op, inputs)
    p.wp8_operation_machine_state = machine
    p.wp8_source_script_begin(command_id='parent')
    p.wp8_source_script_returned(command_id='parent')
    r = h['source_error'](s)
    assert not r['ok'] and trace == ['shutdown', 'door1', 'unlatch', 'pipette_color']
    assert r['source_children'][1]['error'] == 'canonical_collection_refused'


def test_actual_pipette_helper_connected_to_provider_safe_exit():
    args, s, n, entries, fx, _ = composite_bindings()
    n.tips = True
    p = object.__new__(Provider)
    def cancel(state):
        assert state is s and s.source_model.logical_tip_present is False
        fx.append(('cancel',))
        return {'ok':True, 'source_stop_scripts':True}
    h = p.build_oem_lifecycle_handlers(execute_plan=lambda *a: pytest.fail('unused'),
        safe_stop_tip_exit=lambda state: helpers(args)['safe_stop_tip_exit'](state, source_occurrence_id='safe:connected'),
        cancel_source=cancel,
        home_gripper=lambda state: fx.append(('home',)) or {'ok':True})
    r = h['safe_stop_exit'](s)
    assert r['ok'] and fx[-3:] == [('tip_state', {'tip_loaded':False}), ('cancel',), ('home',)]
    assert len(r['source_children'][0]['native_results']) == 8


def test_tip_query_failure_without_source_return_retains_native_result():
    args, s, n, *_ = composite_bindings()
    args['pipette_call'] = lambda *a: {'ok':False, 'receipt':'failed-query'}
    r = helpers(args)['safe_stop_tip_exit'](s, source_occurrence_id='safe:query-failed')
    assert not r['ok']
    assert r['native_results'][0]['result'] == {'ok':False, 'receipt':'failed-query'}


def test_tip_predicate_exception_preserves_query():
    args, s, n, *_ = composite_bindings()
    def predicate(*a):
        raise RuntimeError('source_collection_missing')
    args['source_bindings'] = replace(args['source_bindings'], tip_exists=predicate)
    with pytest.raises(RuntimeError) as error:
        helpers(args)['safe_stop_tip_exit'](s, source_occurrence_id='safe:predicate-failed')
    assert error.value.oem_partial_results[0]['operation'] == 'query_tip_status_all'


def test_missing_callbacks_do_not_fake_support():
    p, _, _, _, _, _, _ = rig()
    h = p.build_oem_lifecycle_handlers(execute_plan=lambda *a: pytest.fail('unused'))
    assert not {'safe_stop_exit', 'source_error', 'wake'} & h.keys()


@pytest.mark.parametrize('door_ok', [False, True])
def test_finite_door_predicate_preserves_boolean(door_ok):
    p = object.__new__(Provider)
    p.wp8_check_door_status = lambda *a, **k: {'ok': True, 'door_ok': door_ok}
    plan = compile_finite_plate_operation('lifecycle_check_door', source_leaf_available=True)
    r = execute_finite_plate_operation(plan, lambda c: p.execute_wp8_child(c, command_id='c', child_order=0, plan_digest=plan['plan_digest']))
    assert r['door_ok'] is door_ok
    assert WP8_OPERATION_INTENT_KEYS['lifecycle_check_door'] == frozenset()


@pytest.mark.parametrize('value', [False, True])
def test_finite_door_model_uses_canonical_publisher(value):
    p = object.__new__(Provider)
    trace = []
    p._wp8_publish_semantic = lambda **kw: trace.append(kw) or {'ok': True}
    plan = compile_finite_plate_operation('lifecycle_door_model', source_leaf_available=True, value=value)
    r = execute_finite_plate_operation(plan, lambda c: p.execute_wp8_child(c, command_id='c', child_order=0, plan_digest=plan['plan_digest']))
    assert r['ok'] and trace[0]['updates'] == {'thermal_door_open': value}
    assert trace[0]['command_id'] == 'c'
    assert WP8_OPERATION_INTENT_KEYS['lifecycle_door_model'] == frozenset({'value'})


@pytest.mark.parametrize('exists', [False, True])
def test_tip_exit_actual_vs_logical_and_exact_transcript(exists):
    args, s, n, entries, fx, facts = composite_bindings()
    n.tips = exists
    s.source_model.logical_tip_present = not exists
    r = helpers(args)['safe_stop_tip_exit'](s, source_occurrence_id='safe:1')
    assert r['ok'] and s.source_model.logical_tip_present is False
    assert n.calls == [('tips',)] + ([('eject', {'check_missing_tip':True, 'wait':True})] if exists else [])
    assert fx == [('sleep', 1.0)] + ([('waste',), ('publish_location', 6, 96), ('tip_state', {'tip_dirty':False}), ('move_z', 80000), ('move_x', 79000)] if exists else []) + [('tip_state', {'tip_loaded':False})]
    assert len(entries) == len(set(entries))
    assert all(x.startswith('safe:1:') for x in entries)


@pytest.mark.parametrize('stage', ['query', 'waste', 'publish_location', 'eject', 'tip_dirty', 'move_z', 'move_x', 'tip_loaded'])
def test_tip_exit_failure_stops_before_next_effect_and_preserves_publication(stage):
    args, s, n, entries, fx, facts = composite_bindings()
    n.tips = True
    s.source_model.logical_tip_present = True
    original = args['pipette_call']
    def call(name, op, action, state, key):
        r = original(name, op, action, state, key)
        if (stage == 'query' and name == 'query_tip_status_all') or (stage == 'eject' and name == 'eject_all_tips'):
            r = {**r, 'ok':False}
        return r
    args['pipette_call'] = call
    if stage == 'waste':
        args['move_to_waste'] = lambda *a: {'ok':False}
    elif stage in ('publish_location', 'move_z', 'move_x'):
        args['source_bindings'] = replace(args['source_bindings'], **{stage: lambda *a: {'ok':False}})
    elif stage in ('tip_dirty', 'tip_loaded'):
        prior = args['source_bindings'].tip_state
        args['source_bindings'] = replace(args['source_bindings'], tip_state=lambda changes,*a: {'ok':False} if stage in changes else prior(changes,*a))
    r = helpers(args)['safe_stop_tip_exit'](s, source_occurrence_id='safe:failed')
    assert not r['ok'] and s.source_model.logical_tip_present is True
    expected = ['query_tip_status_all', 'moveToWaste', 'updateLocation', 'eject_all_tips', 'tip_state', 'moveZ', 'moveX', 'tip_state']
    index = ['query','waste','publish_location','eject','tip_dirty','move_z','move_x','tip_loaded'].index(stage)
    assert [row['operation'] for row in r['native_results']] == expected[:index+1]
