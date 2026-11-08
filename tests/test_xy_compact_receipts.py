"""Offline inert-wire API/store contracts; never physical-arrival qualification."""
import json
import os
from pathlib import Path

import pytest
from tests.test_movement_modes import connected, rig, submit  # noqa: F401


def surfaces(http, cid):
    compact = http.get('/operator/v2/actions/receipts/' + cid)
    assert compact.status_code == 200, compact.text
    detail = http.get('/operator/v2/actions/receipts/' + cid + '?detail=true').json()
    history = http.get('/operator/actions/history').json()
    dashboard = http.get('/operator/v2/dashboard').json()
    result = [compact.json(), detail,
              next(r for r in history['items'] if r['command_id'] == cid),
              next(r for r in dashboard['latest_receipts'] if r['command_id'] == cid)]
    out = os.environ.get('BIOXP_RECEIPT_CONTRACT_WIRE')
    if out:
        path = Path(out)
        wire = json.loads(path.read_text()) if path.exists() else {}
        wire.setdefault('receipts', {})[cid] = {
            'fixture_identity': 'offline-inert-wire-xy-receipt-contract',
            'compact': result[0], 'detail': detail, 'history': history, 'dashboard': dashboard}
        path.write_text(json.dumps(wire, indent=2))
    return result


def test_retarget_compact_retains_pending_identity_and_target(connected):
    http, provider, frames, fault, driver, positions = connected
    cid1, first = submit(http, 'oem.xy.retarget', 'receipt-contract-first-01',
                         {'target': {'x': 20000, 'y': 30000}})
    cid, detail = submit(http, 'oem.xy.retarget', 'receipt-contract-next-001',
                        {'target': {'x': 25000, 'y': 35000}})
    raw = detail['source_receipt']
    assert raw['xy_retarget']['motion_pending'] is True
    assert raw['xy_retarget']['replaced_command_id']
    result = surfaces(http, cid)
    for row in result:
        assert row['status'] == 'completed' and row['terminal'] is True
        assert row['physical_effect_verified'] is False
        assert row['xy_retarget'] == raw['xy_retarget']
    assert result[1]['source_receipt'] == raw
    assert raw['completion_verified'] is False
    assert [(b, v) for b, c, t, m, v in frames if c == 4 and b in (4, 5)] == [
        (5, 20000), (4, 30000), (5, 25000), (4, 35000)]
    with provider._lock:
        provider._pending_xy_retarget = None
    provider._retarget_pose_thread.join(3)


def test_sequence_compact_summary_keeps_full_children_detail_only(connected):
    http, provider, frames, fault, driver, positions = connected
    cid, detail = submit(http, 'oem.xy.move_sequence', 'receipt-contract-seq-001',
                        {'targets': [{'x': 1100, 'y': 2200}, {'x': 3300, 'y': 4400}]})
    raw = detail['source_receipt']
    assert len(raw['xy_sequence']['children']) == 2
    expected = {key: value for key, value in raw['xy_sequence'].items()
                if key in {'ok', 'failure', 'interrupted', 'next_target_index',
                           'failed_target_index', 'completed_targets'}}
    for row in surfaces(http, cid):
        assert row['xy_sequence'] == expected
        assert row['physical_effect_verified'] is False
    assert surfaces(http, cid)[1]['source_receipt'] == raw


@pytest.mark.parametrize('status, proof, extension', [
    ('completed', True, {'motion_pending': False, 'replaced_command_id': None}),
    ('failed', False, {'motion_pending': True}),
    ('outcome_unknown', False, {'motion_pending': True}),
    ('completed', False, None),
    ('failed', False, None),
    ('outcome_unknown', False, None),
])
def test_persisted_partial_and_old_receipts_do_not_infer_xy_truth(connected, status, proof, extension):
    http, provider, frames, fault, driver, positions = connected
    store = http.app.state.operator_receipt_store
    cid = 'offline-contract-%s-%s' % (status, proof)
    row = {'command_id': cid, 'idempotency_key': cid, 'action_id': 'fixture.unspecific',
           'source_identity': {'fixture_identity': 'offline-receipt-contract-not-hardware'},
           'started_at': 1700000000.0, 'ownership_generation': 7}
    claimed, created = store.claim(row)
    assert created
    row.update(status=status, finished_at=1700000001.0, physical_effect_verified=proof)
    if extension is not None:
        row['xy_retarget'] = extension
    store.put(row, _expected_status=claimed['status'])
    before = store.by_command(cid)
    for receipt in surfaces(http, cid):
        assert receipt['status'] == ('ambiguous' if status == 'outcome_unknown' else status)
        assert receipt['physical_effect_verified'] is proof
        if extension is None:
            assert 'xy_retarget' not in receipt
        else:
            assert receipt['xy_retarget'] == extension
        assert 'xy_sequence' not in receipt
    assert surfaces(http, cid)[1]['source_receipt'] == before
    assert not frames


@pytest.mark.parametrize('summary', [
    {'ok': False, 'failure': 'xy_sequence_child_failed', 'completed_targets': 1,
     'failed_target_index': 1},
    {'ok': False, 'failure': 'xy_sequence_interrupted', 'interrupted': True,
     'completed_targets': 0, 'next_target_index': 0},
])
def test_failed_sequence_summary_is_bounded_without_changing_ledger(connected, summary):
    http, provider, frames, fault, driver, positions = connected
    store = http.app.state.operator_receipt_store
    cid = 'offline-contract-failed-sequence'
    ledger = [{'index': i, 'command_id': f'{cid}:xy:{i}', 'diagnostic': 'x' * 1000}
              for i in range(100)]
    row = {'command_id': cid, 'idempotency_key': cid, 'action_id': 'oem.xy.move_sequence',
           'source_identity': {'fixture_identity': 'offline-receipt-contract-not-hardware'},
           'started_at': 1700000000.0, 'finished_at': 1700000001.0,
           'ownership_generation': 7, 'status': 'failed', 'physical_effect_verified': False,
           'xy_sequence': {**summary, 'children': ledger},
           'xy_retarget': {'motion_pending': True, 'target': {'unrecognized': 'x' * 10000},
                           'resolved_xy': {'x': 1234, 'y': 5678}}}
    store.put(row)
    for receipt in surfaces(http, cid):
        assert receipt['xy_sequence'] == summary
        assert receipt['xy_retarget']['motion_pending'] is True
        assert receipt['xy_retarget']['resolved_xy'] == {'x': 1234, 'y': 5678}
        assert len(json.dumps(receipt['xy_retarget'])) < 2048
        assert len(json.dumps(receipt['xy_sequence'])) < 1024
    assert surfaces(http, cid)[1]['source_receipt']['xy_sequence']['children'] == ledger
    assert not frames
