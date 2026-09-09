"""Execute only the route AST, never import the live API runtime.

OEM installed BioXPControlLib.il:29681–29745, btnMoveYTo_Click:
IL_003e and IL_003f are ldc.i4.0 => moveY(target, false, false).
Diagnostic Board Test and source-default moveY are distinct entrypoints.
"""
import ast
import asyncio
from pathlib import Path
from types import SimpleNamespace


def test_manual_y_absolute_uses_nonwaiting_click_contract():
    source = Path(__file__).parents[1] / 'src/bioxp/api.py'
    tree = ast.parse(source.read_text())
    route = next(n for n in tree.body if isinstance(n, ast.AsyncFunctionDef)
                 and n.name == 'motion_oem_y_move_absolute')
    route.decorator_list = []
    calls = []
    async def run_blocking(label, call, **kwargs):
        return call()
    def provider(*args, **kwargs):
        calls.append((args, kwargs))
        return {'source_return_ok': True, 'physical_effect_verified': False}
    scope = {'_run_blocking': run_blocking, '_execute_serial206_y_call': provider,
             'OemYMoveAbsoluteRequest': SimpleNamespace}
    exec(compile(ast.Module(body=[route], type_ignores=[]), str(source), 'exec'), scope)
    result = asyncio.run(scope[route.name](SimpleNamespace(target_steps=1234)))
    assert calls == [(('move_absolute', 1234), {'wait_for_stop': False})]
    assert result['physical_effect_verified'] is False
