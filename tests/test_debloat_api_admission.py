"""C5 actual admission boundary and retained store intent validation."""
import ast
from pathlib import Path
from types import SimpleNamespace
import pytest
from fastapi import HTTPException
from bioxp.operator_command_plane import OperatorCommandStore
from bioxp import operator_controls

def test_admission_omits_state_reconstruction_and_compile():
    # Execute the actual nested boundary independently of hardware installation.
    tree = ast.parse(Path(operator_controls.__file__).read_text())
    node = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'admit_wp8_operation')
    calls = []
    def admit(operation, **kwargs):
        calls.append((operation, kwargs))
        return {'command_id': 'child'}
    plane = SimpleNamespace(store=SimpleNamespace(admit_internal_wp8_operation=admit), _state=lambda: {'owner': 1})
    scope = {'Any': object, 'Mapping': dict, 'command_plane': plane,
             'refresh_deck_provider': lambda: object()}
    exec(compile(ast.Module(body=[node], type_ignores=[]), '<actual-admission-boundary>', 'exec'), scope)
    assert scope['admit_wp8_operation']('move_plate', inputs={'destination': 28}, idempotency_key='explicit') == {'command_id': 'child'}
    assert calls == [('move_plate', {'inputs': {'destination': 28}, 'state': {'owner': 1}, 'idempotency_key': 'explicit'})]
    # Real store still owns intent validation before any machine-state access.
    store = OperatorCommandStore.__new__(OperatorCommandStore)
    with pytest.raises(HTTPException) as exc:
        store.admit_internal_wp8_operation('not_an_operation', inputs={}, state={})
    assert exc.value.status_code == 422
    with pytest.raises(HTTPException) as exc:
        store.admit_internal_wp8_operation('move_plate', inputs={'fabricated_state': True}, state={})
    assert exc.value.status_code == 422
