"""RT-036 connected relocation producer through SQLite receipt consumption."""
import json
import pytest
from tests.test_wake_setup_debloat import rig, cycle
from tests.test_cover_carry_release_connected import connected
from bioxp.operator_command_plane import OperatorCommandStore
from bioxp.oem_vision_acceptance import assemble_inspect_cover_receipt


@pytest.fixture
def retained_rig(rig):
    driver, provider, frames, fault, root = rig
    driver._motor_last_tx_ts = {}
    cycle(driver)
    provider.generation_provider = lambda: 3
    store = OperatorCommandStore(root)
    provider.bind_deck_semantic_state_reader(store.deck_semantic_state)
    provider.bind_deck_semantic_state_publisher(store.publish_deck_owner_state)
    store.bind_deck_owner_authority_reader(provider.deck_owner_authority_stamps,
                                         scope=provider.deck_owner_authority_scope)
    yield provider, provider.primitives, provider.state_store, provider.reference_store, store, root
    store.stop()


@pytest.mark.parametrize('fail', [False, True])
def test_relocation_publication(connected, fail):
    rig = connected
    store, provider = rig.store, rig.provider
    stamps = provider.deck_owner_authority_stamps()
    state = {'ownership_generation': 3, 'serial206_initialization_provider': {
        'x_authority': {'current_board_lifecycle_generation': stamps['board_epoch_5']},
        'board4_authority': {'active_board_epoch': stamps['board_epoch_4']}}}
    admitted = store.admit_internal_wp8_operation('cover_inspection', inputs={
        'deck_inspection': True, 'screen_resolution_high': False, 'inspection_log_only': False},
        state=state, idempotency_key='inspection-boundary')
    cid = admitted['command_id']
    claimed = store.claim_next()
    store.persist_wp8_plan(cid, rig.plan, authority_stamps=stamps)
    provider._oem_cover_inspection_findings[cid] = provider._oem_cover_inspection_findings.pop('offline-inspection')
    owner = {**stamps, 'dispatch_attempt_id': claimed['dispatch_attempt_id'],
             'work_identity': 'child:8:coverInspectionRelocate', 'plan_digest': rig.plan['plan_digest']}
    if fail:
        _, positions = provider._wp8_calibration()
        rig.native.fail_at = ('g', positions['close'])
    from bioxp.oem_deck_movement import DeckExecutionFailure
    try:
        result = provider.execute_wp8_child({**rig.plan['children'][8], '_delivery_identity': owner},
            command_id=cid, child_order=8, plan_digest=rig.plan['plan_digest'])
    except DeckExecutionFailure as exc:
        assert fail
        result = {'ok': False, 'delivery_attempted': exc.delivery_attempted,
                  'exception': str(exc), 'provider_results': exc.provider_results}
    else:
        assert not fail
        assert result['final_cover_locations'] == {'output': 18, 'reagent': 20}
    import os, time
    from pathlib import Path
    started = time.perf_counter()
    encoded = json.dumps(result)
    encode_seconds = time.perf_counter() - started
    if os.environ.get('CORE_TRACE_DIR'):
        directory = Path(os.environ['CORE_TRACE_DIR'])
        directory.mkdir(parents=True, exist_ok=True)
        (directory / ('inspection-' + str(fail) + '.json')).write_text(json.dumps({
            'producer_bytes': len(encoded.encode()), 'encode_seconds': encode_seconds,
            'result': result}, indent=2))
    for task in provider._wp8_tasks.values():
        task['thread'].join(timeout=2)
    store.terminalize_wp8_child(cid, 8, state='failed' if fail else 'completed', result=result,
                               dispatch_attempt_id=claimed['dispatch_attempt_id'])
    reopened = OperatorCommandStore(rig.root)
    try:
        evidence = reopened.wp8_operation_evidence(cid)
        child = next(row for row in evidence['children'] if row['child_order'] == 8)
        readback = json.loads(child['terminal_evidence_json'])['result']
        assert readback['ok'] is (not fail)
        if fail:
            assert 'injected_native_transfer_failure' in repr(readback)
            assert child['terminal_state'] == 'failed'
        else:
            assert readback['relocations'] == result['relocations']
            receipt = assemble_inspect_cover_receipt(evidence, settings={
                'deck_inspection': True, 'screen_resolution_high': False, 'inspection_log_only': False})
            assert receipt['relocations'] == result['relocations']
            assert receipt['final_cover_locations'] == {'output': 18, 'reagent': 20}
            assert receipt['door_open_verified'] is True
    finally:
        reopened.stop()
