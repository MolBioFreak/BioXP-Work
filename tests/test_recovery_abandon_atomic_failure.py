"""Software abandonment rolls back when its existing SQLite owner cannot commit."""
import json
import sqlite3

import pytest
from tests.test_workflow_recovery_abandon import (
    integrated_rig, retained_rig, installed_retained, query_rig,
    connected_motion_wait, recovery_routes, failed_workflow, request, snapshot,
)


@pytest.mark.parametrize('table', [
    'operator_plane_recovery_acknowledgements',
    'operator_plane_transitions',
])
def test_abandon_writer_failure_preserves_custody_and_unknowns(failed_workflow, table):
    rig, job, _, child_id = failed_workflow
    store = rig.store
    before = snapshot(rig, [job['job_id'], child_id])
    semantic, safety = store.deck_semantic_state(), rig.safety()
    wire = list(rig.native.trace)
    url, body = request(rig, key='abandon-write-failure-'+table)
    denied = []

    def reject(action, arg1, arg2, database, trigger):
        if action == sqlite3.SQLITE_INSERT and arg1 == table:
            denied.append((action, arg1))
            return sqlite3.SQLITE_DENY
        return sqlite3.SQLITE_OK

    # The connection is shared with real dispatcher/readback workers. Installation
    # and removal use its owning lock; the fault itself is inside the real writer.
    with store._lock:
        store.connection.set_authorizer(reject)
    try:
        try:
            response = rig.client.post(url, json=body)
        except (sqlite3.DatabaseError, ExceptionGroup):
            response = None
        if response is not None:
            assert response.status_code >= 500, response.text
    finally:
        with store._lock:
            store.connection.set_authorizer(None)

    assert denied, 'the actual mutation never reached the selected SQLite boundary'
    assert snapshot(rig, [job['job_id'], child_id]) == before
    assert store.deck_semantic_state() == semantic and rig.safety() == safety
    assert rig.native.trace == wire
    with store._lock:
        assert store.connection.execute('SELECT workflow_command_id FROM operator_plane_lane').fetchone()[0] == job['job_id']
        assert store.connection.execute('SELECT count(*) FROM operator_plane_recovery_acknowledgements').fetchone()[0] == 0
        assert not store.connection.in_transaction

    # This is a rolled-back software-only write, not a retry of uncertain motion.
    response = rig.client.post(url, json=body)
    assert response.status_code == 200, response.text
    assert response.json()['outcome_remains'] == 'unknown'
    assert response.json()['abandoned_workflow_command_id'] == job['job_id']
    assert snapshot(rig, [job['job_id'], child_id]) == before
    assert rig.native.trace == wire
    assert store.deck_recovery_blocker() == 'deck_recovery_hold'
