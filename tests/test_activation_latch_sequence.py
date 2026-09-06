"""ControlLib.checkDoorStatus:8670-8725 on the real preparation owner."""
import asyncio
import time

import pytest
from fastapi import HTTPException
from bioxp.motion_safety import prepare_motion_without_motion
from bioxp.operator_receipt_store import compact_response_summary
from test_motion_safety_control_plane import FakeMotionDriver, authority


class LatchDriver(FakeMotionDriver):
    def __init__(self, *, latch=1, rail_after_lock=0):
        super().__init__()
        self.io[3]['value'] = latch
        self.latched = False
        self.rail_after_lock = rail_after_lock

    def deck_io_set_type(self, io_type, value):
        assert io_type == 2
        self.calls.append(('solenoid', value))
        self.latched = value == 1
        return {'ack': {'status': 100}, 'ok': True, 'value_set': value}

    def motor_query_24v_sensor(self):
        self.calls.append(('query_24v',))
        value = self.rail_after_lock if self.latched else 1
        return {'ack': {'status': 100}, 'reply_valid': True, 'sample_valid': True,
                'safety_valid': value == 0, 'oem_scalar': value}


@pytest.fixture(autouse=True)
def no_real_wait(monkeypatch):
    monkeypatch.setattr(time, 'sleep', lambda seconds: None)


def test_activate_closes_latch_before_checking_24v(monkeypatch):
    driver = LatchDriver()
    monkeypatch.setattr(time, 'sleep', lambda seconds: driver.calls.append(('sleep', seconds)))
    result = prepare_motion_without_motion(driver, authority())
    assert result['ok'] is True
    assert driver.calls[:8] == [
        ('sleep', 0.5), ('query_io', 1), ('query_io', 3), ('solenoid', 1),
        ('sleep', 0.8), ('query_io', 1), ('query_io', 3), ('query_24v',),
    ]
    assert ('solenoid', 0) not in driver.calls
    assert all(call[0] not in {'move', 'home'} for call in driver.calls)
    assert result['physical_motion_commanded'] is False


def test_24v_still_missing_releases_latch_and_does_not_activate_boards(monkeypatch):
    driver = LatchDriver(rail_after_lock=1)
    monkeypatch.setattr(time, 'sleep', lambda seconds: driver.calls.append(('sleep', seconds)))
    result = prepare_motion_without_motion(driver, authority())
    assert result['ok'] is False
    assert driver.calls[-2:] == [('solenoid', 0), ('sleep', 0.3)]
    assert not any(call[0] in {'activate', 'deactivate', 'initialize_without_motion'} for call in driver.calls)
    assert result['failure_stage'] == 'rail_24v_readback'
    assert '24 V' in result['error']
    saved = compact_response_summary({'body': {'detail': result}})
    assert saved['body']['detail']['failure_stage'] == 'rail_24v_readback'
    assert saved['body']['detail']['error'] == result['error']


def test_latch_zero_does_not_take_the_oem_latch_one_branch():
    driver = LatchDriver(latch=0)
    result = prepare_motion_without_motion(driver, authority())
    assert result['ok'] is False
    assert ('solenoid', 1) not in driver.calls
    assert ('solenoid', 0) in driver.calls


@pytest.mark.parametrize('rail', [None, {'ack': None, 'oem_scalar': 0},
    {'ack': {'status': 100}, 'oem_scalar': 0, 'reply_valid': False, 'sample_valid': True, 'safety_valid': True},
    {'ack': {'status': 100}, 'oem_scalar': 0, 'reply_valid': True, 'sample_valid': False, 'safety_valid': True},
    {'ack': {'status': 100}, 'oem_scalar': 1, 'reply_valid': True, 'sample_valid': True, 'safety_valid': True}])
def test_invalid_voltage_evidence_never_passes(rail):
    driver = LatchDriver()
    driver.motor_query_24v_sensor = lambda: rail
    result = prepare_motion_without_motion(driver, authority())
    assert result['ok'] is False
    assert not any(call[0] in {'deactivate', 'activate'} for call in driver.calls)


def test_oem_ignores_solenoid_return_and_preserves_ack_evidence():
    driver = LatchDriver()
    def set_with_failed_ack(io_type, value):
        driver.latched = value == 1
        return {'ack': {'status': 3}, 'ok': False}
    driver.deck_io_set_type = set_with_failed_ack
    result = prepare_motion_without_motion(driver, authority())
    assert result['ok'] is True
    row = next(x for x in result['stage_ledger'] if x['stage_id'] == 'latch_solenoid')
    assert row['controller_evidence']['controller_acknowledged'] is False
    assert row['controller_evidence']['return_value_ignored'] is True


def test_component_refresh_does_not_actuate_latch(monkeypatch):
    driver = LatchDriver()
    waits = []
    monkeypatch.setattr(time, 'sleep', waits.append)
    result = prepare_motion_without_motion(driver, authority(), components=('x',), reuse_current_board_lifecycle=True)
    assert result['ok'] is False
    assert not any(call[0] in {'solenoid', 'deactivate', 'activate'} for call in driver.calls)
    assert waits == []


def test_failed_preparation_never_instructs_home_z(monkeypatch):
    from bioxp import api
    driver = LatchDriver(rail_after_lock=1)
    monkeypatch.setattr(api.Serial206MotionAuthority, 'from_active_snapshot', staticmethod(authority))
    monkeypatch.setattr(api, '_tester', driver)
    monkeypatch.setattr(api, '_get_tester', lambda: driver)
    monkeypatch.setattr(api.hardware_state, 'invalidate', lambda **kw: None)
    async def run_inline(label, func, timeout_s):
        return func()
    monkeypatch.setattr(api, '_run_blocking', run_inline)
    with pytest.raises(HTTPException) as raised:
        asyncio.run(api.motion_oem_prepare_without_motion())
    assert raised.value.status_code == 409
    assert raised.value.detail['failure_stage'] == 'rail_24v_readback'
    assert 'Home Z' not in raised.value.detail['next_required_action']
