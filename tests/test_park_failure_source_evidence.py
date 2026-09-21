"""Actual returned Park failure evidence survives WP8 and a cold SQLite read."""
import json
import subprocess
import sys

import pytest
from tests.protocol_v1_integration_fixture import integrated_rig
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_tip_query_publication import query_rig
from tests.test_protocol_live_prepare_connected import connected_motion_wait


@pytest.mark.parametrize('physical_return', ['failed_result', 'exception_without_evidence'])
def test_park_failure_truth_is_retained_not_invented(integrated_rig, monkeypatch, physical_return):
    rig = integrated_rig
    rig.store.publish_deck_owner_state(source_operation='updateLocation',
        source_command_id='offline-park-failure-predecessor',
        updates={'current_location':'LOC_STRIP1','current_well':0},
        **rig.provider.deck_owner_authority_stamps())
    returned = {'ok':False,'delivery_attempted':True,
        'controller_command_acknowledged':True,'controller_completion_verified':False,
        'failure':'isolated missing physical completion'}
    def motion_leaf(**kwargs):
        if physical_return == 'exception_without_evidence':
            raise RuntimeError('isolated unstructured physical exception')
        return dict(returned)
    monkeypatch.setattr(rig.provider.primitives,
        'oem_initialize_motion_scriptmove_to_waste', motion_leaf, raising=False)
    job = rig.start(rig.payload('park-failure-source-'+physical_return))
    done = rig.terminal(job)
    assert done['command']['status']=='ambiguous', done
    with rig.store._lock:
        row = rig.store.connection.execute(
            'SELECT c.command_id FROM operator_plane_commands c '
            'JOIN operator_commands a USING(command_id) '
            "WHERE a.parent_command_id=? AND c.status='ambiguous'",(job['job_id'],)).fetchone()
        child_id = row['command_id']
        rows = rig.store.connection.execute(
            'SELECT * FROM operator_plane_wp8_children WHERE command_id=? ORDER BY child_order',
            (child_id,)).fetchall()
        evidence = json.loads(rows[0]['terminal_evidence_json'])['result']
        assert rows[0]['operation']=='parkGantry' and rows[0]['terminal_state']=='ambiguous'
        assert all(row['terminal_state']=='planned' for row in rows[1:])
        path = rig.store.connection.execute('PRAGMA database_list').fetchone()[2]
    assert evidence['delivery_attempted'] is True
    assert rig.store.deck_semantic_state()['current_location']=='LOC_STRIP1'
    if physical_return=='failed_result':
        actual = evidence['provider_results'][0]
        assert actual['operation']=='park_gantry'
        assert actual['collection_tip_state']['tip_exists'] is False
        assert actual['collection_tip_state']['command_id']
        assert actual['collection_tip_state']['receipt_id']
        assert actual['source_children'][-1]['result']==returned
        assert 'source_location_update' not in actual
    else:
        assert 'provider_results' not in evidence
    code = "import sqlite3,json,sys; c=sqlite3.connect('file:'+sys.argv[1]+'?mode=ro',uri=True); print(c.execute('SELECT terminal_evidence_json FROM operator_plane_wp8_children WHERE command_id=? AND child_order=0',(sys.argv[2],)).fetchone()[0])"
    cold = subprocess.run([sys.executable,'-c',code,path,child_id],check=True,capture_output=True,text=True)
    assert json.loads(cold.stdout)['result']==evidence
