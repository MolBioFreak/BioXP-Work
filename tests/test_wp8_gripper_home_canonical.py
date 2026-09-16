"""Native first/cached G through finite cleanup and the canonical child ledger.

The standalone parent/source-return event is explicitly test-owned; the sibling
adapter tests qualify real application safe-stop lifetime and settlement.
"""
import json
import pytest
from tests.protocol_v1_integration_fixture import integrated_rig
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig
from tests.test_wp8_gripper_home_adapter import configure, assert_void


@pytest.mark.parametrize('version', [0, 1])
def test_first_and_cached_cleanup_canonical(integrated_rig, monkeypatch, version):
    from bioxp.oem_deck_movement import compile_finite_plate_operation, make_wp8_operation_executor
    rig = integrated_rig
    configure(rig, monkeypatch, version)
    store, provider = rig.store, rig.provider
    stamps = provider.deck_owner_authority_stamps()
    store.bind_workflow_dispatcher(lambda command: None)  # explicit standalone test caller
    parent = 'gripper-standalone-parent'
    store.admit_workflow(command_id=parent, idempotency_key=parent,
        plan_fingerprint='synthetic-gripper-cleanup',
        requested_inputs={'bundle': {'execution': {'runtime_state': {}}}},
        ownership_generation=stamps['ownership_generation'],
        resources=('axis:x', 'axis:y', 'axis:z', 'gripper'), board_epochs={})
    assert store.claim_next()['command_id'] == parent
    execute = make_wp8_operation_executor(provider_getter=lambda: provider, command_store=store)
    child_ids = []
    for ordinal in range(2):
        # Explicit source-return signal for this standalone finite invocation.
        provider._wp8_stop_event.set()
        plan = compile_finite_plate_operation('cleanup', source_leaf_available=True,
            cover_locations={4: 18, 5: 20}, tip_exists=False)
        with store.workflow_context(parent, source_occurrence_id=f'cleanup:{ordinal}'):
            child = store.admit_internal_wp8_operation('cleanup', inputs={},
                state=rig.app.state.operator_command_plane._state(),
                idempotency_key=f'gripper-cleanup:{ordinal}', prepared_plan=plan)
        cid = child['command_id']
        child_ids.append(cid)
        claimed = store.claim_next()
        assert claimed['command_id'] == cid
        result = execute(command_id=cid, plan=plan)
        assert result['ok'] is True, result
        receipt = next(row['result'] for row in result['completed_children']
                       if row['operation'] == 'sendGripperHome')
        assert_void(receipt)
        home = receipt['primitive_result']['home']
        if ordinal == 0:
            assert home['home_decision']['source_returned_normally'] is True
        else:
            assert home['completion_class'] == 'source_cached_noop'
            assert home['controller_home_proof_verified'] is False
        store.finish(cid, status='completed', payload={'response': result}, claimed=claimed)
        evidence = store.wp8_operation_evidence(cid)
        g = next(row for row in evidence['children'] if row['operation'] == 'sendGripperHome')
        assert g['terminal_state'] == 'completed'
        assert json.loads(g['terminal_evidence_json'])['result'] == receipt
    store.finish_workflow(parent, status='completed', payload={}, lifecycle_settled=True)
    fresh = rig.reopen({'job_id': parent})
    assert fresh['workflow']['command']['status'] == 'completed'
    assert [row['command_id'] for row in fresh['children'] if row['action_id'] == 'oem.deck._finite_operation'] == child_ids
    assert all(row['status'] == 'completed' for row in fresh['children'])
    for row in fresh['children']:
        if row['command_id'] in child_ids:
            persisted = json.loads(row['receipt_json'])
            results = persisted['terminal_evidence']['response']['completed_children']
            assert_void(next(r['result'] for r in results if r['operation'] == 'sendGripperHome'))
