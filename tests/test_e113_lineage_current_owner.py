"""The inherited V14 fix at the real WP8 invoke/store boundary, offline only."""
import json
import os
from pathlib import Path

from tests.test_deck_scoped_authority import retained_rig
from tests.test_cover_carry_release_connected import connected
from tests.test_native_completion_handoff import handoff


def test_wp8_delivery_uses_current_owner_without_restamping_history(handoff, monkeypatch):
    from bioxp.oem_deck_movement import compile_finite_plate_operation
    rig = handoff
    store, provider = rig.store, rig.provider
    stamps = provider.deck_owner_authority_stamps()
    # A legitimate predecessor publication before a subsequent controller
    # owner epoch change. Only a fresh scratch canonical writer is mutated.
    historic = {**stamps, 'board_epoch_4': stamps['board_epoch_4'] + 1,
                'board_epoch_5': stamps['board_epoch_5'] + 1}
    store.bind_deck_owner_authority_reader(lambda: historic,
        scope=provider.deck_owner_authority_scope)
    store.publish_deck_owner_state(source_operation='updateLocation',
        source_command_id='offline-previous-owner', updates={'current_location': 'LOC_OC_COVER', 'current_well': 0}, **historic)
    before = store.deck_semantic_state()
    store.bind_deck_owner_authority_reader(provider.deck_owner_authority_stamps,
        scope=provider.deck_owner_authority_scope)
    admitted = rig.app.state.oem_wp8_operation_admitter('script_snapshot', inputs={},
        idempotency_key='e113-lineage-current-owner')
    claimed = store.claim_next()
    assert claimed['command_id'] == admitted['command_id']
    machine = provider.wp8_operation_machine_state('script_snapshot', {})
    plan = compile_finite_plate_operation('script_snapshot', source_leaf_available=True, **machine)
    from bioxp import oem_deck_schema_v8 as v8
    old_predicate = v8.NEW.strip().removeprefix('AND ')
    old_result = store.connection.execute('SELECT (' + old_predicate + ') FROM '
        "(SELECT 'dispatched' AS state) m, "
        "(SELECT 'oem.deck._finite_operation' AS action_id) c, "
        "(SELECT 'wp8_child' AS work_kind, ? AS ownership_generation, ? AS board_epoch_4, ? AS board_epoch_5) NEW, "
        '(SELECT ? AS ownership_generation, ? AS board_epoch_4, ? AS board_epoch_5) semantic',
        (stamps['ownership_generation'], stamps['board_epoch_4'], stamps['board_epoch_5'],
         before['ownership_generation'], before['board_epoch_4'], before['board_epoch_5'])).fetchone()[0]
    assert old_result == 0
    at_delivery = []
    original_record = store.record_delivery_attempt
    def record(*args, **kwargs):
        prior = store.deck_semantic_state()
        result = original_record(*args, **kwargs)
        at_delivery.append({'before': prior, 'after': store.deck_semantic_state(), 'identity': kwargs['work_identity']})
        return result
    monkeypatch.setattr(store, 'record_delivery_attempt', record)
    result = rig.app.state.oem_wp8_operation_executor(command_id=admitted['command_id'], plan=plan)
    assert result['ok'] is True
    attempts = [dict(r) for r in store.connection.execute('SELECT * FROM operator_plane_delivery_attempts WHERE command_id=?', (admitted['command_id'],))]
    assert attempts
    assert all(r['board_epoch_4'] == stamps['board_epoch_4'] and r['board_epoch_5'] == stamps['board_epoch_5'] for r in attempts)
    assert at_delivery[0]['before'] == at_delivery[0]['after'] == before
    # Validate the exact pre-V14 predicate against this genuinely stale
    # canonical publication; no frozen source module/trigger is edited.
    assert before['board_epoch_4'] != stamps['board_epoch_4']
    assert before['board_epoch_5'] != stamps['board_epoch_5']
    assert 'semantic' not in store.connection.execute("SELECT sql FROM sqlite_master WHERE name='operator_plane_delivery_attempts_lineage_insert'").fetchone()[0]
    store.finish(admitted['command_id'], status='completed', payload=result, claimed=claimed)
    assert store.get_command(admitted['command_id'])['status'] == 'completed'
    if os.environ.get('E113_LINEAGE_OUTPUT'):
        Path(os.environ['E113_LINEAGE_OUTPUT'], 'current-owner-invoke.json').write_text(json.dumps({'before': before, 'at_delivery': at_delivery, 'pre_v14_predicate': old_result, 'after': store.deck_semantic_state(), 'plan': plan, 'result': result, 'attempts': attempts, 'command': store.get_command(admitted['command_id'])}, indent=2))
