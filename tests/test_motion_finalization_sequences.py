"""Independent fixed G-entry and thermal-search vectors from locked ClassCanLib IL.

Head.goHome RVA5ee0 IL0000..003b; Thermal.doorSearchHome RVAade8
IL0075..00c1 and checkMotorStopped. These tests do NOT accept the inherited
G active adapter. Door vectors include preclear and terminal branches.
No connected tester is constructed. Every ordered event includes its address.
"""
import pytest
from bioxp.usb_driver import BioXpTester
import bioxp.usb_driver as driver_module


class GEntry(BioXpTester):
    def __init__(self, *, power=False, initialized=True, home=True, position=0):
        self.calls = []
        self.power = power
        self.initialized = initialized
        self._oem_motor_home_cache = {(4, 2): home}
        self._oem_position_cache = {(4, 2): position}

    def _motion_oem_axis_profile(self, *args, **kwargs):
        return {'board': 4, 'motor': 2}

    def oem_no24v_state(self):
        self.calls.append(('No24V', self.power))
        return self.power

    def _oem_board_state(self):
        self.calls.append(('initialized', 4, self.initialized))
        return {4: self.initialized}

    def oem_current_board_lifecycle_generation(self):
        return 9

    def _motor_oem_go_home_adapted(self, *args, **kwargs):
        self.calls.append(('unaccepted_active_adapter', args, kwargs))
        return {'active_adapter': True}


@pytest.mark.parametrize('rehome', [False, True])
def test_g_cached_noop_has_no_controller_reads_or_fresh_proof(rehome):
    d = GEntry()
    result = d.motor_oem_go_home('g', speed=200, rehome=rehome)
    assert d.calls == [('No24V', False), ('initialized', 4, True)]
    assert result['source_return_code'] == 0
    assert result['completion_class'] == 'source_cached_noop'
    assert result['controller_command_acknowledged'] is False
    assert result['controller_terminal_state_verified'] is False
    assert result['controller_home_proof_verified'] is False
    assert result['physical_effect_verified'] is False


@pytest.mark.parametrize('initialized', [False, True])
def test_g_power_precedes_initialization_and_cached_noop(initialized):
    d = GEntry(power=True, initialized=initialized)
    with pytest.raises(RuntimeError, match='Lost 24V power go home'):
        d.motor_oem_go_home('g', speed=200, rehome=True)
    assert d.calls == [('No24V', True)]


def test_g_uninitialized_returns_one_without_controller_reads():
    d = GEntry(initialized=False)
    result = d.motor_oem_go_home('g', speed=200, rehome=True)
    assert d.calls == [('No24V', False), ('initialized', 4, False)]
    assert result['source_return_code'] == 1
    assert result['physical_effect_verified'] is False


@pytest.mark.parametrize('home,position', [(False, 0), (True, 1), (False, -1000)])
def test_g_non_noop_enters_source_moveleft_not_adapted_telemetry(home, position):
    d = GEntry(home=home, position=position)
    d.begin_bus_event_window = lambda: {}
    def source_left(board, **kwargs):
        assert (board, kwargs) == (4, {'speed': 200, 'motor': 2})
        raise RuntimeError('source MoveLeft sentinel')
    d.motor_move_left = source_left
    with pytest.raises(RuntimeError, match='source MoveLeft sentinel'):
        d.motor_oem_go_home('g', speed=200, rehome=False, timeout_s=7)
    assert d.calls == [('No24V', False), ('initialized', 4, True),
                       ('No24V', False), ('initialized', 4, True)]


class DoorSequence(BioXpTester):
    def __init__(self, monkeypatch, *, power=False, initialized=True,
                 home_at=None, stop_at=0, fresh=True, initial_home=False, final_home=True, serial=206, calibrated=True):
        self.calls = []
        self.power = power
        self.initialized = initialized
        self.home_at = home_at
        self.stop_at = stop_at
        self.fresh = fresh
        self.home_polls = 0
        self.speed_polls = 0
        self.status_reads = 0
        self.initial_home, self.final_home = initial_home, final_home
        self.serial, self.calibrated = serial, calibrated
        self.stage = "before"
        self._oem_position_cache = {(6, 0): 100}
        monkeypatch.setattr(driver_module.time, 'sleep',
                            lambda value: self.calls.append(('sleep', value)))

    def _motion_oem_axis_profile(self, *args, **kwargs):
        return {'board': 6, 'motor': 0, 'stall_guard': 6, 'home_speed': 50}

    def oem_no24v_state(self):
        self.calls.append(('No24V', self.power))
        return self.power

    def _oem_board_state(self):
        self.calls.append(('initialized', 6, self.initialized))
        return {6: self.initialized}

    def motor_query_motor_stop(self, board, motor=0):
        self.calls.append(('query138', board, motor))
        return {'wait_latch_reset': getattr(self, 'query138_nonnull', True)}

    def begin_bus_event_window(self, *, reset_wait_latch=True):
        self.calls.append(('cursor', reset_wait_latch))
        return {'after_sequence': 12 if reset_wait_latch else None}

    def _send_motor(self, board, command, kind, motor, value, **kwargs):
        assert kwargs == {'attempts': 1, 'wait_reply': True, 'write_timeout_ms': 55,
                          'read_timeout_ms': 60000, 'max_reads': 1, 'strict_match': True,
                          'allow_recover': False, 'ordinary_motor_retry': True}
        self.calls.append(('TX', board, command, kind, motor, value))
        return getattr(self, 'relative_ack', {'status': 100})

    def motor_oem_wait_target_reached(self, board, motor=0, *, timeout_s, event_window):
        self.calls.append(('event_wait', board, motor, timeout_s, event_window))
        return {'ok': getattr(self, 'event_success', True)}

    def motor_get_position(self, board, motor=0):
        self.calls.append(('queryActualPosition', board, motor))
        value = next(self.positions) if hasattr(self, 'positions') else 2100
        self._oem_position_cache[(board, motor)] = value
        return {'position': value, 'ack': {'status': 100}}

    def _motor_oem_door_source_settings(self):
        return {"serial": self.serial, "camera_calibrated": self.calibrated, "min_steps": 0, "max_steps": 160000}

    def motor_thermal_door_status(self):
        # Boundary marker for inherited composite reads, NOT an OEM oracle.
        self.calls.append(('unaccepted_composite_status', 6, 0))
        closed = self.status_reads > 0
        self.status_reads += 1
        return {'home': {'home': closed, 'value': 1 if closed else 0},
                'switches': {}, 'closed': closed, 'opened': False}

    def motor_set_axis_param(self, board, param, value, motor=0):
        self.calls.append(('SAP', board, motor, param, value))
        return {'ok': True}

    def motor_move_left(self, board, speed, motor=0):
        self.calls.append(('MoveLeft', board, motor, speed))
        self.stage = 'search'
        return {'ack': {'status': 100}}

    def motor_query_home_switch(self, board, motor=0):
        active = self.initial_home if self.stage == "before" else self.final_home if self.stage == "after" else self.home_at is not None and self.home_polls >= self.home_at
        self.calls.append(('queryHome', board, motor, active))
        if self.stage == "search":
            self.home_polls += 1
        return {'home': active, 'value': 1 if active else 0,
                'ack': {'status': 100}, 'reply_valid': True}

    def motor_get_speed(self, board, motor=0):
        speed = 0 if self.stop_at is not None and self.speed_polls >= self.stop_at else 7
        self.calls.append(('queryMotorSpeed', board, motor, speed))
        self.speed_polls += 1
        return {'speed': speed, 'speed_reply_valid': self.fresh,
                'ack': {'status': 100} if self.fresh else None}

    def motor_wait_stopped(self, *args, **kwargs):
        self.calls.append(('unaccepted_generic_wait', args, kwargs))
        return {'stopped': True}

    def motor_stop(self, board, motor=0):
        self.calls.append(('unaccepted_leaf_stop', board, motor))
        return {'ok': True}

    def motor_oem_board_stop(self, board, motor=0, *, axis_name=None):
        self.calls.append(('board_stopMotor', board, motor, axis_name))
        self.stage = 'after'
        return {'source_call_completed': True}

    def motor_set_home(self, board, motor=0):
        self.calls.append(('setHome', board, motor))
        return {'ack': {'status': 100}}


def prefix():
    return [('No24V', False), ('initialized', 6, True),
            ('initialized', 6, True), ('queryHome', 6, 0, False),
            ('SAP', 6, 0, 205, 6), ('No24V', False), ('initialized', 6, True),
            ('MoveLeft', 6, 0, 50), ('No24V', False)]


def suffix():
    return [('board_stopMotor', 6, 0, 'door'),
            ('initialized', 6, True), ('queryHome', 6, 0, True), ('setHome', 6, 0)]


def poll(active, speed):
    return [('initialized', 6, True), ('queryHome', 6, 0, active), ('sleep', .05),
            ('No24V', False), ('initialized', 6, True),
            ('queryMotorSpeed', 6, 0, speed)]


@pytest.mark.parametrize('startup,timeout', [(False, .01), (True, 90)])
def test_door_search_first_iteration_order_and_no_seen_nonzero_requirement(monkeypatch, startup, timeout):
    d = DoorSequence(monkeypatch)
    result = d.motor_oem_door_search_home(startup=startup, timeout_s=timeout)
    assert d.calls == prefix() + poll(False, 0) + suffix()
    assert result['wait']['source_counter'] == 299
    assert result['wait']['polls'] == 1
    assert result['wait']['controller_terminal_state_verified'] is True


@pytest.mark.parametrize('home_at,iterations,speed_polls', [
    (None, 302, 301), (0, 82, 81), (218, 300, 299), (219, 302, 301),
])
def test_door_exact_counter_postdecrement_and_81_boundary(monkeypatch, home_at, iterations, speed_polls):
    d = DoorSequence(monkeypatch, home_at=home_at, stop_at=None)
    result = d.motor_oem_door_search_home(timeout_s=.01)
    expected = prefix()
    for i in range(speed_polls):
        expected += poll(home_at is not None and i >= home_at, 7)
    expected += [('initialized', 6, True), ('queryHome', 6, 0, home_at is not None), ('sleep', .05)] + suffix()
    assert d.calls == expected
    assert d.home_polls == iterations
    assert d.speed_polls == speed_polls
    assert result['wait']['source_counter'] == -2
    assert result['wait']['timeout'] is True
    assert result['wait']['controller_terminal_state_verified'] is False


def test_door_cached_zero_is_not_fresh_stopped_evidence(monkeypatch):
    d = DoorSequence(monkeypatch, fresh=False)
    result = d.motor_oem_door_search_home()
    assert d.calls == prefix() + poll(False, 0) + suffix()
    assert result['wait']['stopped'] is True
    assert result['wait']['last_ack'] is None
    assert result['wait']['controller_terminal_state_verified'] is False


@pytest.mark.parametrize('initialized', [False, True])
def test_door_no24v_guard_before_initialized(monkeypatch, initialized):
    d = DoorSequence(monkeypatch, power=True, initialized=initialized)
    with pytest.raises(RuntimeError, match='Lost 24V power doorSearchHome'):
        d.motor_oem_door_search_home()
    assert d.calls == [('No24V', True)]


def test_door_uninitialized_void_noop_without_controller_evidence(monkeypatch):
    d = DoorSequence(monkeypatch, initialized=False)
    result = d.motor_oem_door_search_home()
    assert d.calls == [('No24V', False), ('initialized', 6, False)]
    assert result['source_board_return'] is None
    assert result['source_call_completed'] is True
    assert result['controller_terminal_state_verified'] is False
    assert result['physical_effect_verified'] is False


@pytest.mark.parametrize('power,initialized,reason', [
    (True, True, 'No24V'), (False, False, 'board_not_initialized')])
def test_door_stopped_predicate_guard_short_circuits_after_sleep(monkeypatch, power, initialized, reason):
    d = DoorSequence(monkeypatch, power=power, initialized=initialized)
    d.stage = 'search'
    result = d._motor_oem_door_search_wait(6, 0)
    expected = [('initialized', 6, initialized)]
    if initialized:
        expected += [('queryHome', 6, 0, False)]
    expected += [('sleep', .05), ('No24V', power)]
    if not power:
        expected += [('initialized', 6, initialized)]
    assert d.calls == expected
    assert result['stopped'] is True
    assert result['source_predicate_reason'] == reason
    assert result['controller_terminal_state_verified'] is False


def preclear_vector(reset=True):
    return [('No24V', False), ('initialized', 6, True), ('query138', 6, 0),
            ('cursor', reset), ('TX', 6, 4, 1, 0, 2000),
            ('event_wait', 6, 0, 30.0, {'after_sequence': 12 if reset else None}),
            ('queryActualPosition', 6, 0), ('No24V', False)]


@pytest.mark.parametrize('nonnull,ack,event_success', [
    (True, {'status': 100}, True), (False, None, True),
    (True, {'status': 2}, True), (True, {'status': 100}, False)])
def test_door_preclear_exact_relative_event_chain_and_normal_minus_one_continuation(monkeypatch, nonnull, ack, event_success):
    d = DoorSequence(monkeypatch, initial_home=True)
    d.query138_nonnull, d.relative_ack, d.event_success = nonnull, ack, event_success
    result = d.motor_oem_door_search_home(startup=True, timeout_s=.01)
    expected = [('No24V', False), ('initialized', 6, True),
                ('initialized', 6, True), ('queryHome', 6, 0, True), ('SAP', 6, 0, 205, 8)]
    expected += preclear_vector(nonnull)
    expected += prefix()[4:] + poll(False, 0) + suffix()
    assert d.calls == expected
    assert result['preclear_move']['source_return_code'] == (2100 if event_success else -1)
    assert result['preclear_wait']['ok'] is event_success
    assert result['physical_effect_verified'] is False
    assert result['status_before'] is None and result['status_after'] is None
    assert result['opened_before'] is None and result['opened_after'] is None


@pytest.mark.parametrize('cached,refresh,moves', [
    (157980, None, True), (157981, 100, True), (157981, 157981, False),
    (-1980, None, True), (-1981, -1981, False)])
def test_door_preclear_source_z_limit_margins_and_single_refresh(monkeypatch, cached, refresh, moves):
    d = DoorSequence(monkeypatch)
    d._oem_position_cache[(6, 0)] = cached
    d.positions = iter(([refresh] if refresh is not None else []) + [2100])
    result = d._motor_oem_door_preclear(6, 0, d._motor_oem_door_source_settings())
    expected = [('No24V', False), ('initialized', 6, True)]
    if refresh is not None:
        expected += [('queryActualPosition', 6, 0)]
    expected += preclear_vector()[2:] if moves else [('No24V', False)]
    assert d.calls == expected
    assert result['source_return_code'] == (2100 if moves else -1)


@pytest.mark.parametrize('serial,calibrated,sets_home,throws', [
    (206, True, False, True), (206, False, False, False),
    (9, True, True, False), (9, False, True, False),
    (10, True, False, True)])
def test_door_terminal_serial_calibration_branches_without_fake_home_proof(monkeypatch, serial, calibrated, sets_home, throws):
    d = DoorSequence(monkeypatch, serial=serial, calibrated=calibrated, final_home=False)
    expected = prefix()
    if serial < 10:
        expected = prefix()[:4] + [('SAP', 6, 0, 205, 8)] + preclear_vector() + prefix()[4:]
    expected += poll(False, 0) + [('board_stopMotor', 6, 0, 'door'),
                                ('initialized', 6, True), ('queryHome', 6, 0, False)]
    if sets_home:
        expected += [('setHome', 6, 0)]
    if throws:
        with pytest.raises(RuntimeError, match='^Failed to find door home$'):
            d.motor_oem_door_search_home()
    else:
        result = d.motor_oem_door_search_home()
        assert result['source_call_completed'] is True
        assert result['source_board_return'] is None
        assert result['closed_confirmed'] is False
        assert result['controller_home_proof_verified'] is False
        assert result['physical_effect_verified'] is False
    assert d.calls == expected


def test_door_null_final_switch_preserves_scalar_home_without_fresh_proof(monkeypatch):
    d = DoorSequence(monkeypatch)
    original = d.motor_query_home_switch
    def query(board, motor=0):
        row = original(board, motor)
        if d.stage == 'after':
            return {'home': True, 'value': 0, 'ack': None, 'reply_valid': False}
        return row
    d.motor_query_home_switch = query
    result = d.motor_oem_door_search_home()
    assert d.calls == prefix() + poll(False, 0) + suffix()
    assert result['set_home'] is not None
    assert result['closed_after'] is True
    assert result['closed_confirmed'] is False
    assert result['controller_home_proof_verified'] is False


@pytest.mark.parametrize('event_success,stage', [(True, 4), (False, 3)])
def test_door_preclear_queries_position_before_terminal_power_exception(monkeypatch, event_success, stage):
    d = DoorSequence(monkeypatch)
    d.event_success = event_success
    original = d.motor_get_position
    def position(board, motor=0):
        result = original(board, motor)
        d.power = True
        return result
    d.motor_get_position = position
    with pytest.raises(RuntimeError, match=f'moveSteps{stage}'):
        d._motor_oem_door_preclear(6, 0, d._motor_oem_door_source_settings())
    assert d.calls == preclear_vector()[:-1] + [('No24V', True)]


def test_door_settings_require_bound_snapshot_and_select_z_limits(monkeypatch):
    from types import SimpleNamespace
    import bioxp.oem_machine_bundle as bundle
    d = object.__new__(BioXpTester)
    snapshot = SimpleNamespace(machine_serial='206', camera_calibrated=False,
                               axis_limits={'z': {'min_steps': 13, 'max_steps': 165432},
                                            'g': {'min_steps': 99, 'max_steps': 3000}})
    monkeypatch.setattr(bundle, 'get_active_oem_machine_snapshot', lambda: snapshot)
    assert d._motor_oem_door_source_settings() == {
        'serial': 206, 'camera_calibrated': False, 'min_steps': 13, 'max_steps': 165432}
    def unavailable():
        raise bundle.OemMachineBundleError('snapshot missing')
    monkeypatch.setattr(bundle, 'get_active_oem_machine_snapshot', unavailable)
    with pytest.raises(bundle.OemMachineBundleError, match='snapshot missing'):
        d._motor_oem_door_source_settings()
