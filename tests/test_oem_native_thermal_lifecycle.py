"""Native thermal recorder: only physical transport, clock and timer are doubled."""
from types import SimpleNamespace
import pytest

from bioxp.usb_driver import BioXpTester
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider as Provider


@pytest.fixture
def rig(monkeypatch):
    tester = object.__new__(BioXpTester)
    tester._oem_board_initialized = {6: True, 7: True}
    trace, sleeps, timers = [], [], []
    now = [0.0]
    replies = {}
    hook = [None]
    def tx(board, command, typ, bank, value, **kw):
        assert kw['attempts'] == 1
        trace.append((board, command, typ, bank, value))
        override = replies.get((board, command, typ, bank))
        if override is not None:
            return override() if callable(override) else override
        v = 10 if typ == 23 else 25000 if command == 143 or (command == 10 and typ == 4) else 1000
        return {'status': 100, 'value': v}
    def schedule(callback, delay=1.0):
        timer = SimpleNamespace(due=now[0] + delay, callback=callback, cancelled=False)
        timer.cancel = lambda: setattr(timer, 'cancelled', True)
        timers.append(timer)
        return timer
    def sleep(seconds):
        sleeps.append(seconds)
        now[0] += seconds
        if hook[0]:
            hook[0](seconds)
        # Native tick executes through a deterministic clock; no source body double.
        for t in list(timers):
            if not t.cancelled and t.due <= now[0]:
                t.cancelled = True
                t.callback()
        assert now[0] < 800, 'bounded native thermal recorder clock'
    tester.send_tmcl_retry = tx
    tester._oem_thermal_schedule = schedule
    monkeypatch.setattr('bioxp.usb_driver.time.sleep', sleep)
    monkeypatch.setattr('bioxp.usb_driver.time.monotonic', lambda: now[0])
    return SimpleNamespace(t=tester, trace=trace, sleeps=sleeps, now=now, replies=replies, hook=hook, timers=timers)


def test_shutdown_exact_order_no_lid_and_state(rig):
    t = rig.t
    for b, a in [(6, 0), (7, 0), (7, 1)]:
        t._oem_thermal_fan(b, a).update(tc_on=True, target_c=42, lid_target_c=70)
    t.oem_set_chiller_pwm()
    t.oem_turn_off_heater()
    assert rig.trace == [(7, 144, 0, 1, 0), (7, 144, 0, 0, 0), (6, 144, 0, 0, 0), (6, 144, 0, 0, 0)]
    assert t._oem_thermal_fan(6, 0)['lid_target_c'] == 70
    assert all(not t._oem_thermal_fan(b, a)['tc_on'] for b, a in [(6, 0), (7, 0), (7, 1)])
    assert t._oem_thermal_state(7, 1)['target'] == 100


@pytest.mark.parametrize('selector,bank', [('OC', 1), ('oc', 1), ('RC', 0), ('rc', 0)])
def test_selected_pwm_clamp(rig, selector, bank):
    assert rig.t.oem_set_chiller_pwm(selector, 200)['ok']
    assert rig.trace == [(7, 144, 0, bank, 100)]


def test_shutdown_null_reply_preserves_board_vs_controller_updates(rig):
    rig.replies[7, 144, 0, 1] = lambda: None
    rig.t._oem_thermal_fan(7, 1).update(tc_on=True, target_c=14)
    result = rig.t.oem_set_chiller_pwm()
    assert not result['ok'] and len(rig.trace) == 2
    assert rig.t._oem_thermal_fan(7, 1)['tc_on']  # controller returned before disable
    assert result['source_board_error_event'] == 'setPWM communication error!'
    rig.replies[6, 144, 0, 0] = lambda: None
    rig.t._oem_thermal_fan(6, 0).update(tc_on=True, target_c=14)
    assert not rig.t.oem_turn_off_heater()['ok']
    assert not rig.t._oem_thermal_fan(6, 0)['tc_on']  # board wrapper still disables


def test_lid_epilogue_exact_full_nonwait_transcript(rig):
    result = rig.t.oem_set_lid_temperature(30, 99, -20, False)
    assert result['ok'] and result['source_body_returned']
    assert rig.trace == [(6, 9, 8, 1, -20000), (6, 10, 23, 1, 0),
        (6, 10, 4, 1, 0), (6, 10, 7, 1, 0), (6, 140, 0, 1, 30000),
        (6, 10, 23, 1, 0), (6, 10, 4, 1, 0)]
    assert rig.sleeps == [1.0]
    assert rig.t._oem_thermal_state()['lid_duration'] == 0
    assert not rig.t._oem_thermal_fan(6, 0)['tc_on']
    assert rig.t._oem_thermal_fan(6, 0)['lid_target_c'] == -99
    assert not result['physical_effect_verified']


@pytest.mark.parametrize('actual,retry', [(25000, True), (30000, False), (31000, False)])
def test_lid_low_pwm_retry_predicate_and_only_one(rig, actual, retry):
    rig.replies[6, 10, 23, 1] = {'status': 100, 'value': 0}
    rig.replies[6, 10, 4, 1] = {'status': 100, 'value': actual}
    assert rig.t.oem_thermal_set_lid_temperature(30, 0, False)['ok']
    assert sum(row[1] == 140 for row in rig.trace) == (2 if retry else 1)
    assert rig.sleeps == ([1.0, 1.0] if retry else [1.0])


def test_tc_clamp_truncation_and_unconditional_pwm_retry(rig):
    rig.replies[6, 10, 23, 0] = {'status': 100, 'value': 4}
    rig.replies[6, 10, 4, 0] = {'status': 100, 'value': 110000}
    assert rig.t.oem_thermal_set_temperature(101.23456, 0, False)['ok']
    assert [row[4] for row in rig.trace if row[1] == 140] == [100000, 100000]
    assert not rig.t._oem_thermal_state()['satisfied']
    rig.trace.clear()
    rig.t.oem_thermal_set_lid_temperature(101.23456, 0, False)
    assert [row[4] for row in rig.trace if row[1] == 140] == [101234]


def test_bailout_changes_actual_wait_predicates_not_chiller_or_wire(rig):
    state = rig.t._oem_thermal_state()
    state.update(reached=False, satisfied=False, lid_reached=False, lid_satisfied=False)
    chiller = rig.t._oem_thermal_state(7, 0)
    chiller.update(reached=False, satisfied=False)
    assert rig.t.oem_thermal_bailout()['ok']
    assert all(state[k] for k in ('reached', 'satisfied', 'lid_reached', 'lid_satisfied'))
    assert not chiller['reached'] and not chiller['satisfied'] and not rig.trace


def test_active_wait_bailout_and_repeated_target_reset(rig):
    def release(seconds):
        if seconds == .01:
            rig.t.oem_thermal_bailout()
    rig.hook[0] = release
    result = rig.t.oem_set_tc_temperature(60, 100, 2.5)
    assert result['ok'] and result['source_wait_satisfied']
    assert rig.trace[0] == (6, 9, 7, 0, 2500)
    assert rig.sleeps[:2] == [.2, 1.0]
    assert rig.t._oem_thermal_fan(6, 0)['interval_s'] == 5
    rig.t.oem_thermal_set_temperature(70, 12, False)
    assert not rig.t._oem_thermal_state()['reached']
    assert not rig.t._oem_thermal_state()['satisfied']


def test_real_timer_reaches_and_satisfies_duration(rig):
    rig.replies[6, 10, 4, 0] = {'status': 100, 'value': 30000}
    result = rig.t.oem_thermal_set_temperature(30, 1, True)
    assert result['ok'] and result['source_wait_satisfied']
    assert .5 in rig.sleeps and .01 in rig.sleeps
    assert not rig.t._oem_thermal_board_timer_enabled


def test_lid_zero_duration_wait_does_not_wait_for_actual(rig):
    result = rig.t.oem_thermal_set_lid_temperature(80, 0, True)
    assert result['source_wait_satisfied']
    assert rig.sleeps == [1.0]


@pytest.mark.parametrize('user_stopped', [True, False])
def test_wait_no24v_branches(rig, user_stopped):
    def drop(seconds):
        if seconds == .01:
            rig.t._oem_24v_dropped = True
            rig.t._oem_user_stopped = user_stopped
    rig.hook[0] = drop
    result = rig.t.oem_thermal_set_temperature(60, 5, True)
    assert result['source_body_returned'] is user_stopped
    if user_stopped:
        assert result['source_user_stopped'] and not result['source_wait_satisfied']
    else:
        assert result['source_exception'] == 'Lost 24V power setTemperature2'
    assert rig.t.oem_no24v_state()


@pytest.mark.parametrize('name,args', [('oem_thermal_set_temperature', (30,)), ('oem_thermal_set_lid_temperature', (30,)), ('oem_chiller_set_temperature', (0, 20))])
def test_initial_no24v_emits_nothing(rig, name, args):
    rig.t._oem_24v_dropped = True
    assert not getattr(rig.t, name)(*args)['ok']
    assert rig.trace == []


def test_resume_all_retained_source_branches_and_bank_oddity(rig):
    t = rig.t
    t._oem_thermal_fan(6, 0).update(tc_on=True, target_c=40, lid_target_c=70)
    t._oem_thermal_fan(7, 0).update(tc_on=True, target_c=12)
    t._oem_thermal_fan(7, 1).update(tc_on=True, target_c=18)
    assert t.oem_resume_temperature()['ok']
    assert [row for row in rig.trace if row[1] == 140] == [
        (6, 140, 0, 0, 40000), (6, 140, 0, 1, 70000),
        (7, 140, 0, 0, 12000), (7, 140, 0, 1, 12000)]
    assert t._oem_thermal_fan(7, 1)['target_c'] == 12


def test_resume_defaults_and_absent_board_are_real_noops(rig):
    assert rig.t.oem_resume_temperature()['ok'] and not rig.trace
    rig.t._oem_board_presence = {}
    assert rig.t.oem_set_tc_temperature(30, 0, 1)['source_noop'] == 'm_board_null'
    assert rig.t.oem_set_lid_temperature(30, 0, -20, False)['source_noop'] == 'm_board_null'
    assert not rig.trace


def test_chiller_source_null_and_non100_return_and_pre_tx_fan_assignment(rig):
    rig.replies[7, 140, 0, 1] = lambda: None
    result = rig.t.oem_chiller_set_temperature(1, 15)
    assert result['source_return'] == -1 and result['source_body_returned']
    assert rig.t._oem_thermal_fan(7, 1)['target_c'] == 15
    assert rig.trace == [(7, 143, 0, 3, 0), (7, 10, 8, 1, 0), (7, 140, 0, 1, 15000)]
    rig.replies[7, 140, 0, 1] = {'status': 2, 'value': 0}
    result = rig.t.oem_chiller_set_temperature(1, 16)
    assert result['source_return'] == -1 and not result['ok']
    assert rig.t._oem_thermal_state(7, 1)['set']


def test_wait_timeout_callback_is_async_source_event_not_stop(rig):
    events = []
    def event(message):
        events.append(message)
        rig.t.oem_thermal_bailout()
    rig.t._oem_thermal_error_callback = event
    rig.replies[6, 10, 7, 0] = {'status': 100, 'value': 100000}
    result = rig.t.oem_thermal_set_temperature(60, 0, True)
    assert not result['ok'] and result['source_body_returned']
    assert events and not rig.t.oem_no24v_state()
    assert not any(row[1] == 3 for row in rig.trace)


def test_timer_strict_duration_and_lid_resend_oddity(rig):
    t, state = rig.t, rig.t._oem_thermal_state()
    state.update(set=True, reached=True, satisfied=False, duration=1, elapsed=1000)
    assert t._oem_thermal_timer_process(t._oem_thermal_result()) is False
    assert not state['satisfied']
    state['elapsed'] = 1001
    t._oem_thermal_timer_process(t._oem_thermal_result())
    assert state['satisfied']
    state.update(lid_set=True, lid_reached=False, lid_start_temp=25, lid_target=80,
                 lid_set_count=1, lid_elapsed=2, lid_needed=100)
    t._oem_thermal_timer_process(t._oem_thermal_result())
    assert rig.trace[-1] == (6, 140, 0, 1, 80000)


def test_manual_rate_guards_unchanged(rig):
    assert not rig.t.thermal_set_rates(1, -20, 1)['ok']
    assert rig.trace == []


@pytest.mark.parametrize('opcode,args,operation,values', [
    ('sp', ('30.5', '2', '2.5'), 'set_tc_temperature', {'temp_c': 30.5, 'duration': 2, 'rate_c_s': 2.5}),
    ('splid', ('30', '9', '-20'), 'set_lid_temperature', {'temp_c': 30., 'duration': 9, 'rate_c_s': -20., 'wait': True}),
    ('splid', ('30', '9', '-20', 'F'), 'set_lid_temperature', {'temp_c': 30., 'duration': 9, 'rate_c_s': -20., 'wait': False}),
    ('splid', ('30', '9', '-20', 'false'), 'set_lid_temperature', {'temp_c': 30., 'duration': 9, 'rate_c_s': -20., 'wait': True}),
    ('cc', ('OC', '12'), 'set_chiller_temperature', {'bank': 1, 'temp_c': 12.}),
    ('cc', ('RC', '15'), 'set_chiller_temperature', {'bank': 0, 'temp_c': 15.}),
])
def test_factory_frozen_arguments(opcode, args, operation, values):
    p = object.__new__(Provider)
    rows = []
    def execute(op, arguments, action, state):
        rows.append((op, arguments, action, state))
        return {'ok': True, 'source_return': -1}
    h = p.build_oem_native_handlers(settings={}, execute_plan=None, execute_thermal=execute)
    a, s = SimpleNamespace(params={'arguments': args}), object()
    out = h[opcode](a, s)
    assert rows == [(operation, values, a, s)]
    assert out['source_return'] == -1


@pytest.mark.parametrize('settings,opcode,args', [
    ({'MotionOnly': True}, 'sp', ('invalid',)),
    ({'MotionOnly': True}, 'splid', ()),
    ({'MotionOnly': True}, 'cc', ()),
    ({}, 'cc', ('oc', '12')), ({}, 'cc', ('OC', 'bad')), ({}, 'cc', ('RC', '2147483648')),
])
def test_factory_source_noop_branches(settings, opcode, args):
    p = object.__new__(Provider)
    def forbidden(*args):
        pytest.fail('source noop must not call transport')
    h = p.build_oem_native_handlers(settings=settings, execute_plan=None, execute_thermal=forbidden)
    out = h[opcode](SimpleNamespace(params={'arguments': args}), object())
    assert out['ok'] and out['source_noop'] and not out['delivery_attempted']




@pytest.mark.parametrize('bank', [0, 1])
def test_timer_timeout_strict_and_clock_reset(rig, bank):
    state = rig.t._oem_thermal_state()
    p = 'lid_' if bank else ''
    state['lid_target'] = 80
    state.update({p + 'set': True, p + 'reached': False,
                  p + 'needed': 10, p + 'elapsed': 350000 if bank else 50000})
    rig.t._oem_thermal_timer_process(rig.t._oem_thermal_result())
    state[p + 'elapsed'] += 1
    with pytest.raises(RuntimeError, match='took too long'):
        rig.t._oem_thermal_timer_process(rig.t._oem_thermal_result())
    assert state[p + 'elapsed'] == (350001 if bank else 0)


@pytest.mark.parametrize('bank', [0, 1])
def test_target_null_keeps_preexisting_set_state_but_duration_overload_runs(rig, bank):
    state = rig.t._oem_thermal_state()
    rig.replies[6, 140, 0, bank] = lambda: None
    out = rig.t.oem_thermal_set_lid_temperature(30, 2, False) if bank else rig.t.oem_thermal_set_temperature(30, 2, False)
    assert not out['ok'] and out['source_body_returned']
    assert not state['lid_set' if bank else 'set']
    assert state['lid_duration' if bank else 'duration'] == 2
    assert not state['lid_satisfied' if bank else 'satisfied']


def test_rejected_gp_is_zero_not_cached_target(rig):
    rig.replies[6, 10, 7, 0] = {'status': 2, 'value': 123456}
    out = rig.t._oem_thermal_result()
    assert rig.t._oem_thermal_gp(out, 6, 7, 0) == 0
    assert not out['ok']


def test_lid_null_source_read_does_not_invent_temperature(rig):
    rig.replies[6, 10, 4, 1] = lambda: None
    out = rig.t.oem_thermal_set_lid_temperature(30, 0, False)
    assert not out['ok'] and not out['source_body_returned']
    assert 'null source reply' in out['source_exception']
    assert not any(row[1] == 140 for row in rig.trace)


def test_unbound_source_error_wait_is_explicit_failure(rig):
    rig.replies[6, 10, 23, 0] = lambda: None
    out = rig.t.oem_thermal_set_temperature(60, 10, True)
    assert not out['ok'] and not out['source_body_returned']
    assert out['source_exception'] == 'source_thermal_error_callback_unbound'
    assert 'source_wait_satisfied' not in out


def test_uninitialized_tc_ramp_skips_only_ramp(rig):
    rig.t._oem_board_initialized[6] = False
    rig.hook[0] = lambda seconds: rig.t.oem_thermal_bailout() if seconds == .01 else None
    out = rig.t.oem_set_tc_temperature(60, 1, 2.5)
    assert out['ok'] and not any(row[1] == 9 for row in rig.trace)
    assert any(row[1] == 140 for row in rig.trace)


def test_no24v_finally_chiller_failure_preserves_preceding_native_effect(rig):
    def target():
        rig.t._oem_24v_dropped = True
        return {'status': 100, 'value': 0}
    rig.replies[7, 140, 0, 0] = target
    out = rig.t.oem_chiller_set_temperature(0, 12)
    assert out['source_return'] == 0 and not out['source_body_returned']
    assert out['source_exception'] == 'Lost 24V power set chiller temp'
    assert rig.t._oem_thermal_fan(7, 0)['tc_on']


def test_fan_interval_uses_existing_service_without_starting_absent_service(rig, monkeypatch):
    calls = []
    rig.t._oem_thermal_fan_interval(1)
    assert not calls
    fan = rig.t._oem_thermal_fan(6, 0)
    fan.update(running=True, timer=SimpleNamespace(cancel=lambda: calls.append('cancel')))
    monkeypatch.setattr(rig.t, '_oem_fan_schedule', lambda key: calls.append(key))
    rig.t._oem_thermal_fan_interval(5)
    assert calls == ['cancel', (6, 0)]
    assert fan['interval_s'] == 5


@pytest.mark.parametrize('control', ['bailout', 'abort'])
def test_real_thread_wait_can_be_released_without_claim_or_controller_lock(monkeypatch, control):
    import threading
    import time
    import bioxp.usb_driver as driver
    t = object.__new__(BioXpTester)
    waiting, done, controlled = threading.Event(), threading.Event(), threading.Event()
    events, results = [], []
    real_sleep = time.sleep
    def sleep(seconds):
        if seconds == .01:
            waiting.set()
            assert controlled.wait(2)
            real_sleep(.001)
        elif seconds != 1.0:
            real_sleep(min(seconds, .001))
    monkeypatch.setattr(driver, 'time', SimpleNamespace(sleep=sleep, monotonic=time.monotonic))
    def tx(board, command, typ, bank, value, **kwargs):
        events.append((board, command, typ, bank, value))
        return {'status': 100, 'value': 10 if typ == 23 else 25000 if typ == 4 else 1000}
    t.send_tmcl_retry = tx
    t.novo_router = SimpleNamespace(set_motor_abort_event=lambda *args: None)
    def run():
        try:
            results.append(t.oem_thermal_set_temperature(60, 200, True))
        finally:
            done.set()
    child = threading.Thread(target=run)
    child.start()
    try:
        assert waiting.wait(2)
        if control == 'bailout':
            t.oem_thermal_bailout()
        else:
            t.motor_oem_force_abort_motion()
        controlled.set()
        assert done.wait(2)
        assert results[0]['source_body_returned']
        assert bool(results[0].get('source_user_stopped')) == (control == 'abort')
        assert t.oem_no24v_state() == (control == 'abort')
        assert all(row[1] != 3 for row in events)
    finally:
        controlled.set()
        t.oem_thermal_bailout()
        child.join(2)
        assert not child.is_alive()




def test_exceptional_live_timer_retains_custody_and_origin_until_actual_abort(monkeypatch):
    import threading
    import time
    import json
    import bioxp.usb_driver as driver
    t = object.__new__(BioXpTester)
    origin, replacement, trace = [], [], []
    fault_seen = threading.Event()
    real_sleep, real_monotonic = time.sleep, time.monotonic
    clock_lock = threading.Lock()
    clock = [0.0]
    def monotonic():
        # Clock leaf forces a positive source lid stopwatch set-count, exposing
        # the genuine TimerProcess resend branch rather than mutating its state.
        with clock_lock:
            clock[0] += .002
            return clock[0]
    def sleep(seconds):
        if seconds == .01:
            t._oem_24v_dropped = True  # dropped power before userStopped
        real_sleep(.001)
    monkeypatch.setattr(driver, 'time', SimpleNamespace(sleep=sleep, monotonic=monotonic))
    def schedule(callback, delay=1):
        timer = threading.Timer(.003, callback)
        timer.daemon = True
        timer.start()
        return timer
    t._oem_thermal_schedule = schedule
    def notify(message):
        origin.append(message)
        fault_seen.set()
    t._oem_thermal_error_callback = notify
    t.novo_router = SimpleNamespace(set_motor_abort_event=lambda *args: None)
    def tx(board, command, typ, bank, value, **kw):
        trace.append((board, command, typ, bank, value))
        return {'status': 100, 'value': 10 if typ == 23 else 25000 if typ == 4 else 1000}
    t.send_tmcl_retry = tx
    out = t.oem_thermal_set_lid_temperature(80, 5, True)
    future = out['source_timer_future']
    try:
        assert not out['source_body_returned'] and out['source_timer_pending']
        assert out['source_timer_enabled'] and not future.done()
        frozen = json.dumps({k: v for k, v in out.items() if k != 'source_timer_future'}, sort_keys=True)
        t._oem_thermal_error_callback = replacement.append
        assert fault_seen.wait(2), 'original board timer must keep executing source samples'
        assert origin and replacement == []
        assert len([row for row in trace if row[1] == 140]) > 1, 'native source lid resend remains outstanding'
        assert not future.done(), 'failed source body return is not timer settlement'
        assert json.dumps({k: v for k, v in out.items() if k != 'source_timer_future'}, sort_keys=True) == frozen
        t.motor_oem_force_abort_motion()
        settled = future.result(timeout=2)
        assert settled['source_timer_stopped'] and settled['ok']
        count = len(trace)
        real_sleep(.025)
        assert len(trace) == count
        assert t.oem_no24v_state() and not t._oem_thermal_board_timer_enabled
    finally:
        t.motor_oem_force_abort_motion()
        future.result(timeout=2)


def test_factory_absence_not_advertised():
    h = object.__new__(Provider).build_oem_native_handlers(settings={}, execute_plan=None)
    assert not set(h).intersection({'sp', 'splid', 'cc'})
