"""Historical requested values replayed through actual adapter/driver/encoding.

Hardware replies are simulated; these tests do NOT reconstruct historical wire
bytes, prove historical starting positions, or establish physical direction.
Run in the qualification sandbox (no network, private /dev, read-only root).
"""
import json
import os
import struct
from pathlib import Path

import pytest
from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from bioxp.usb_driver import novo_decode
from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot
from tests.test_motor_receive_identity import setup_driver, receive


@pytest.mark.parametrize('requested', [0, 90000])
@pytest.mark.parametrize('start', [0, 50000, 100000])
def test_z_absolute_requested_values_encode_absolute_not_relative(monkeypatch, requested, start):
    bind_serial206_oem_snapshot(monkeypatch)
    driver = setup_driver()
    driver._oem_board_initialized = {4: True}
    driver._motor_noresp_streak = {}
    driver._motor_pace = lambda *a, **k: None
    registers = {(4, 1, 1): start, (4, 1, 3): 0}
    frames = []

    def hardware(board, cmd, param, motor, value, **kwargs):
        # Keep the real _send_motor and actual signed-int32 Novo encoder.
        raw = bytes(driver._build_frame(board, cmd, param, motor, value))
        decoded = novo_decode(raw)
        assert decoded[3:8] == bytes([board, 7, cmd, param, motor])
        assert struct.unpack('>i', decoded[8:12])[0] == value
        key = (board, motor, param)
        if cmd == 5:
            registers[key] = value
        if cmd == 4:
            frames.append({'board': board, 'command': cmd, 'mode': param,
                           'motor': motor, 'value': value, 'hex': raw.hex()})
            registers[(board, motor, 1)] = value
            receive(driver, board=board, motor=motor)
        return {'status': 100, 'value': registers.get(key, 0)}

    driver.send_tmcl = hardware
    driver.send_tmcl_retry = hardware
    adapter = object.__new__(Serial206ProductionPrimitiveAdapter)
    adapter.tester = driver
    result = adapter.z_move_absolute(requested_position_steps=requested, pseudo_home_steps=65000)
    effective = max(requested, 65000)
    assert result['ok'] is True
    assert result['requested_position_steps'] == requested
    assert result['effective_position_steps'] == effective
    assert result['before_position_steps'] == start
    assert result['target_position_steps'] == effective
    assert len(frames) == 1
    assert {k: frames[0][k] for k in ('board', 'command', 'mode', 'motor', 'value')} == {
        'board': 4, 'command': 4, 'mode': 0, 'motor': 1, 'value': effective}
    assert result['controller_command_acknowledged'] is True
    assert result['controller_terminal_state_verified'] is False
    assert result['physical_effect_verified'] is False
    assert result['after_position_steps'] is None  # OEM event return does not query arrival.
    output = os.environ.get('Z_ABSOLUTE_REPLAY_OUTPUT')
    if output:
        with Path(output).open('a') as stream:
            stream.write(json.dumps({'requested': requested, 'simulated_start': start,
                                     'pseudo_home': 65000, 'effective': effective,
                                     'simulated_delta': effective-start, 'wire': frames,
                                     'result': result}) + '\n')
