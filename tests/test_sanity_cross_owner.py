"""Section 5 deletion-only contracts, no live transport or captured fixtures."""
import ast
from pathlib import Path

import numpy as np
import pytest

from bioxp.vision.oem_inspection import check_label

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('module,names', [
    ('usb_driver', ('bus_health', 'led_logo_pwm_raw', 'led_rgb_scaled',
                    'led_firstpath_matrix', 'motor_move_right', 'motor_spin_test')),
    ('protocols/executor', ('register_child',)),
    ('oem_vision_acceptance', ('evaluate_check_camera_receipt', '_validate_attempt')),
    ('vision/oem_camera_led', ('read_register',)),
    ('vision/oem_inspection', ('label_dark_frame_valid',)),
])
def test_exact_dead_definitions_absent(module, names):
    tree = ast.parse((ROOT / f'src/bioxp/{module}.py').read_text())
    definitions = {n.name for n in ast.walk(tree)
                   if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))}
    assert not definitions.intersection(names)


def test_label_histogram_boundary_through_retained_two_exposure_primitive():
    # Algorithm controls, not fabricated camera captures. Flat histogram fails
    # before the second frame's dimensions matter; 2000 equal pixels pass the
    # first-stage inclusive boundary and proceed to the second-frame check.
    below = np.zeros((1, 1999), dtype=np.uint8)
    at = np.zeros((1, 2000), dtype=np.uint8)
    second = np.full((2, 2000), 80, dtype=np.uint8)
    assert not check_label(below, second)
    with pytest.raises(ValueError, match='matching frame dimensions'):
        check_label(at, second)


def test_canonical_child_consumption_still_records_identity():
    from bioxp.protocols.executor import ProtocolExecutor
    # Exercise the surviving actual result-consumption owner without starting
    # its worker pool or constructing transport.
    from bioxp.protocols.runtime_state import ProtocolRuntimeState, ProtocolWorkflowState
    executor = ProtocolExecutor(dry_run=True)
    executor._state = ProtocolRuntimeState(
        protocol_id='offline-identity-test', dry_run=True,
        workflow=ProtocolWorkflowState(command_id='offline-parent'))
    result = {'pending': True}
    executor._consume({'ok': True, 'command_id': 'canonical-child-1'}, result)
    executor._consume({'ok': True, 'command_id': 'canonical-child-1'}, {})
    assert executor._state.workflow.child_command_ids == ['canonical-child-1']
    assert result == {'ok': True, 'command_id': 'canonical-child-1'}
