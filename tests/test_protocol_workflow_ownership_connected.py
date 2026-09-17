"""Connected canonical ownership through the installed private API."""
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_queued_intent import test_overlapping_continuing_native_fifo as _fifo


def test_existing_named_fifo_survives_canonical_membership(installed_retained, retained_rig, monkeypatch):
    _fifo(installed_retained, retained_rig, monkeypatch, 2)


def test_workflow_dispatches_exact_prepared_child_ahead_of_unrelated_queue(installed_retained, retained_rig, monkeypatch):
    import threading
    from types import SimpleNamespace
    from fastapi.testclient import TestClient
    from bioxp import operator_command_plane as plane_module
    from bioxp.oem_deck_movement import compile_finite_plate_operation
    from tests.test_deck_complete_admission import ready
    from tests.test_deck_queued_intent import submit

    app, provider, primitives, references, root = installed_retained
    ready(installed_retained, monkeypatch, retained_rig)
    plane = app.state.operator_command_plane
    entered, release = threading.Event(), threading.Event()
    def dispatch(command):
        entered.set()
        assert release.wait(8)
        plane.store.finish_workflow('parent', status='completed', payload={}, lifecycle_settled=True)
    plane.store.bind_workflow_dispatcher(dispatch)
    generation = provider.deck_owner_authority_stamps()['ownership_generation']
    plane.store.admit_workflow(command_id='parent', idempotency_key='prepared-parent',
        plan_fingerprint='test-plan', requested_inputs={'bundle': {}}, ownership_generation=generation,
        resources=['axis:x', 'axis:y', 'axis:z'], board_epochs={})
    compiled = compile_finite_plate_operation('script_snapshot', source_leaf_available=True)
    # This spy tests dispatch identity, not controller behavior; the FIFO test
    # above runs the retained native executor. Recompilation is forbidden here.
    captured = []
    monkeypatch.setattr(app.state, 'oem_wp8_operation_executor',
                        lambda **kwargs: captured.append(kwargs) or {'ok': True})
    monkeypatch.setattr(plane_module, 'compile_finite_plate_operation',
                        lambda *a, **k: (_ for _ in ()).throw(AssertionError('recompiled')))
    plane.start()
    try:
        assert entered.wait(5)
        _, queued = submit(TestClient(app), provider, 'queued-behind-parent')
        state = SimpleNamespace(workflow=SimpleNamespace(child_command_ids=[]))
        with plane.store.workflow_context('parent', source_occurrence_id='source-1'):
            result = app.state.oem_workflow_plan_executor(compiled, 'action', state)
        assert result['ok'], result
        assert captured[0]['plan'] == compiled
        assert captured[0]['command_id'] != 'parent'
        assert state.workflow.child_command_ids == [captured[0]['command_id']]
        assert plane.store.get_command(queued)['status'] == 'queued'
        assert plane.store.connection.execute('SELECT COUNT(*) FROM serial206_movement_commands WHERE command_id=?', ('parent',)).fetchone()[0] == 0
    finally:
        release.set()
        plane.stop()
