"""Read-only recovery projection must retain durable stage motor facts."""
import json
import shutil
import sqlite3
import subprocess
import sys
from pathlib import Path

import pytest
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_near_terminal import named_rig, run_named

MOTOR_FACTS = ('delivery_attempted', 'controller_command_acknowledged',
               'controller_completion_verified')


def _snapshot(root):
    with sqlite3.connect(root / 'bioxp_runtime.db') as db:
        return {table: db.execute('SELECT * FROM ' + table + ' ORDER BY 1,2').fetchall()
                for table in ('operator_plane_commands', 'operator_plane_deck_commands',
                              'operator_plane_deck_stages', 'operator_plane_delivery_attempts')}


def _reopen_and_verify(root, cid, moved, *, completion=None):
    if completion is None:
        completion = moved
    expected = dict(delivery_attempted=moved, controller_command_acknowledged=moved,
                    controller_completion_verified=completion)
    from bioxp.operator_command_plane import OperatorCommandStore
    store = OperatorCommandStore(root)  # normal startup, not injected recovery data
    try:
        after_startup = _snapshot(root)
        detail = store.command_detail_v2(cid)
        assert detail['status'] == 'ambiguous', detail
        assert detail['deck_movement']['semantic_state_committed'] is False
        assert detail['physical_effect_verified'] is False
        assert store.deck_recovery_blocker() is not None
        assert store.claim_next() is None
        assert all(detail['deck_movement'][key] is expected[key] for key in MOTOR_FACTS), json.dumps(detail)
        assert detail['deck_movement']['hardware_postcondition_verified'] is False
        # Projecting cannot backfill/rewrite parent, stage or delivery evidence.
        assert _snapshot(root) == after_startup
    finally:
        store.stop()
    script = ('import json,sys; from tests.test_deck_scoped_integration import fresh_process_receipts; '
              'print(json.dumps(fresh_process_receipts(sys.argv[1],sys.argv[2])))')
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', script,
        str(root), cid], text=True, timeout=12))
    for receipt in reopened.values():
        assert receipt['status'] == 'ambiguous'
    assert reopened['detail']['deck_movement']['semantic_state_committed'] is False
    assert all(reopened['detail']['deck_movement'][key] is expected[key] for key in MOTOR_FACTS)
    assert _snapshot(root) == after_startup
    return detail


@pytest.mark.parametrize('boundary,fault,noop', [
    ('ForceToHighHome', None, False), ('moveTo', None, False),
    ('moveTo', 'missing_event', False), ('moveTo', None, True),
], ids=['source-mutation-only', 'completed-motor', 'ack-without-completion', 'source-noop'])
def test_current_producer_process_loss_keeps_stage_facts(retained_rig, monkeypatch, boundary, fault, noop):
    provider, observations, runtime, references, store, root = retained_rig
    leaf, raw, execute = named_rig(retained_rig, monkeypatch,
        (26213, 42413) if noop else (0, 0), fault)
    terminalize = store.terminalize_deck_stage
    interrupted = []
    def crash(command_id, step, **kwargs):
        result = terminalize(command_id, step, **kwargs)
        if step.operation == boundary:
            interrupted.append(command_id)
            # Process loss bypasses normal Exception finalizers; durable stage
            # rows are exactly those written by the current native executor.
            raise SystemExit('isolated process loss after committed stage')
        return result
    monkeypatch.setattr(store, 'terminalize_deck_stage', crash)
    with pytest.raises(SystemExit, match='isolated process loss'):
        run_named(retained_rig, execute, 'LOC_OC', 'current-producer-crash')
    assert len(interrupted) == 1
    cid = interrupted[0]
    before = _snapshot(root)
    moved = boundary == 'moveTo' and not noop
    assert len(leaf.moves) == (2 if moved else 0)
    store.stop()
    detail = _reopen_and_verify(root, cid, moved, completion=moved and fault is None)
    after = _snapshot(root)
    for table in ('operator_plane_deck_stages', 'operator_plane_delivery_attempts'):
        assert before[table] == after[table]
    force = next(s for s in detail['deck_movement']['stages'] if s['operation'] == 'ForceToHighHome')
    assert force['terminal_state'] == 'completed'
    assert force['terminal_evidence']['controller_completion_verified'] is False


def test_actual_stopped_copy_retains_motor_completion_without_publication(tmp_path):
    src = Path('/home/dalab/.hermes/profiles/fresh/robot-audit/deck-tip-failure/finalization-repair/stopped-current.db')
    root = tmp_path / 'stopped-copy'
    root.mkdir()
    shutil.copy2(src, root / 'bioxp_runtime.db')
    cid = 'e13b85a2-7afd-4509-82b0-70581e209291'
    before = _snapshot(root)
    _reopen_and_verify(root, cid, True)
    after = _snapshot(root)
    for table in ('operator_plane_deck_stages', 'operator_plane_delivery_attempts'):
        assert before[table] == after[table]
    for table in ('operator_plane_commands', 'operator_plane_deck_commands'):
        assert [r for r in before[table] if r[0] != cid] == [r for r in after[table] if r[0] != cid]
