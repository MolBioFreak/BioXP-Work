"""Explain inherited barrier hangs without changing their semantic assertions."""
import asyncio

from test_operator_controls import make_app


def test_inherited_interrupt_mock_signature_prevents_barrier_entry(tmp_path, monkeypatch):
    app, _ = make_app(tmp_path, monkeypatch)
    plane = app.state.operator_command_plane
    entered = []
    async def inherited_deliver(action_id, *, interrupt_attempt_id):
        entered.append(action_id)
        return 200, {'ok': True}
    monkeypatch.setattr(plane, '_deliver_controller_interrupt_raw', inherited_deliver)
    request = {'schema_version': 'bioxp.operator_interrupt_request.v1',
               'idempotency_key': 'diagnose-inherited-hang', 'reason': 'offline',
               'observed_ownership_generation': 7,
               'observed_board_epoch_by_board': {'4': 10, '5': 11}}
    receipt = asyncio.run(plane.compat_invoke('oem.abort_all', request))
    print(receipt)
    assert not entered
    assert receipt['error'] == 'controller_interrupt_exception:TypeError'
    # Production passes observed_generation (operator_command_plane.py:7549),
    # while both hanging legacy mocks omit that keyword. The call fails before
    # either barrier can be set; it is not an execution-lock deadlock.
    assert 'observed_generation' not in __import__('inspect').signature(inherited_deliver).parameters
    assert receipt['controller_stop_acknowledged'] is False
