"""Cold lazy readers must be bound before, not during, their first claim."""
import asyncio
import json
import subprocess
import sys

import pytest
from fastapi import HTTPException

from bioxp.pipette.receipts import PipetteReceiptError
from bioxp.services.pipette_service import run_pipette_operation
from tests.test_deck_tip_query_publication import query_rig, query
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_scoped_authority import retained_rig
from tests.test_pipette_collection_owner import state


def test_passive_cold_snapshot_does_not_construct_or_query(query_rig):
    rig = query_rig
    transport = rig[8]
    identity = transport.collection_source_identity()
    assert all(t._driver is None for t in transport._transports)
    assert not rig[6]
    bound = transport.prepare_collection_source_identity()
    assert all(t._driver is not None for t in transport._transports)
    assert bound != identity and not rig[6]
    assert bound == transport.prepare_collection_source_identity()
    assert not rig[6]


def test_first_cold_query_claim_matches_result_reopen_and_replay(query_rig):
    rig = query_rig
    assert all(t._driver is None for t in rig[8]._transports)
    async def run(label, operation, **kwargs):
        return operation()
    def issue():
        return asyncio.run(run_pipette_operation('tip_status', lambda t: t.query_tip_status_all(),
            get_transport=lambda: rig[8], run_blocking=run, receipt_store=rig[5],
            runtime_binding={'idempotency_key': 'cold-query'}))
    issue()
    expected = state(rig)
    assert expected['tip_exists'] is False
    assert rig[6] == [0, 1, 2, 3]
    record = rig[5].connection.execute('SELECT source_identity_json FROM pipette_operations ORDER BY rowid DESC LIMIT 1').fetchone()
    claimed = json.loads(record[0])['collection_owner']
    current = rig[8].collection_source_identity()
    assert claimed['channels'] == [{k: row[k] for k in ('reader', 'reader_generation')} for row in current['channels']]
    script = 'import json,sys;from tests.test_pipette_collection_owner import fresh_read;print(json.dumps(fresh_read(sys.argv[1],json.loads(sys.argv[2]),int(sys.argv[3]))))'
    reopened = json.loads(subprocess.check_output([sys.executable, '-c', script, str(rig[4]), json.dumps(current), str(rig[1].generation_provider())], text=True, timeout=20))
    assert reopened == expected
    replay = issue()
    assert replay['replayed'] is True and rig[6] == [0, 1, 2, 3]
    assert state(rig) == expected


def test_reader_change_during_first_query_still_refuses_collection(query_rig):
    rig = query_rig
    def change(channel):
        if channel == 0:
            rig[8]._transports[0]._driver.bus.router.reader_generation += 1
    rig[7]['during'] = change
    query(rig, key='cold-reader-change')
    with pytest.raises(PipetteReceiptError, match='reader_or_stop_changed'):
        state(rig)


def test_rejected_generation_does_not_construct_cold_readers(query_rig):
    rig = query_rig
    async def run(label, operation, **kwargs):
        raise AssertionError('Rejected command entered physical execution')
    with pytest.raises(HTTPException) as caught:
        asyncio.run(run_pipette_operation('tip_status', lambda t: t.query_tip_status_all(),
            get_transport=lambda: rig[8], run_blocking=run, receipt_store=rig[5],
            runtime_binding={'idempotency_key': 'wrong-generation', 'ownership_generation': rig[1].generation_provider() + 1}))
    assert caught.value.status_code == 409
    assert all(t._driver is None for t in rig[8]._transports)
    assert not rig[6]


@pytest.mark.parametrize('channel', range(4))
def test_reader_construction_failure_does_not_guess_identity(query_rig, channel):
    rig = query_rig
    def fail():
        raise RuntimeError('reader construction refused')
    rig[8]._transports[channel]._driver_factory = fail
    with pytest.raises(RuntimeError, match='reader construction refused'):
        rig[8].prepare_collection_source_identity()
    assert not rig[6]
    assert rig[8]._transports[channel]._driver is None
