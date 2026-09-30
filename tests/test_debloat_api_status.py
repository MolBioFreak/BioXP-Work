"""A3 compact polling leaves the canonical constructor receipt intact."""
import json
from bioxp import api
from bioxp.lifecycle_state import CanonicalLifecycleOwner
from tests.test_pipette_constructor_collection import constructor


def test_status_does_not_copy_constructor_evidence(monkeypatch):
    owner = CanonicalLifecycleOwner()
    owner.transport_changed(True, reason='offline test')
    owner.run_stage('constructor_pipette_stage', lambda: {'ok': True, 'raw': 'x' * 190000})
    full = owner.projection()
    monkeypatch.setattr(api, 'lifecycle_state', owner)
    status = api._status_payload()
    assert 'startup' not in status['lifecycle']
    compact = status['startup']['stages']['constructor_pipette_stage']
    assert compact == {k: v for k, v in full['startup']['stages']['constructor_pipette_stage'].items()
                       if k not in {'evidence', 'history'}}
    assert owner.projection() == full
    assert len(json.dumps(status['startup'])) < len(json.dumps(full['startup']))


def test_compact_constructor_retains_sqlite_result(tmp_path, monkeypatch):
    from bioxp.can_driver import BioXpCanDriver
    from bioxp.pipette.receipts import PipetteReceiptStore
    from bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport
    from bioxp.oem_runtime_store import OEMRuntimeStore
    from bioxp.pipette import receipts as receipt_module
    # Explicit synthetic release identity, not fabricated controller state.
    monkeypatch.setattr(receipt_module, 'current_release_identity', lambda: {
        'verified': True, 'release_id': 'isolated-status-test',
        'source': {'manifest_sha256': '1' * 64, 'aggregate_sha256': '2' * 64}})
    monkeypatch.setattr(receipt_module, 'current_authority_identity', lambda: {
        'evidence_lock_identity_verified': True, 'evidence_lock_sha256': '3' * 64})
    monkeypatch.setattr(receipt_module, 'current_registry_sha256', lambda: '4' * 64)
    OEMRuntimeStore(tmp_path)
    leaves = []
    for channel in range(4):
        driver = BioXpCanDriver.__new__(BioXpCanDriver)
        driver.pipette_id = channel
        leaves.append(CanPipetteTransport(driver_factory=lambda driver=driver: driver, pipette_id=channel))
    receipts = PipetteReceiptStore(tmp_path)
    query_rig = (None, None, None, None, tmp_path, receipts, [], {}, FourPipetteTransport(leaves))
    owner = CanonicalLifecycleOwner()
    owner.transport_changed(True, reason='offline canonical receipt test')
    result = {}
    def action():
        attempt = owner.projection()['startup']['stages']['constructor_pipette_stage']['attempt_id']
        value, _ = constructor(query_rig, monkeypatch, key=attempt)
        result.update(value)
        return value
    owner.run_stage('constructor_pipette_stage', action)
    compact = owner.projection(compact_startup=True)['startup']['stages']['constructor_pipette_stage']
    assert compact['state'] == 'passed' and compact['attempt_id'], compact['error']
    assert 'evidence' not in compact
    import sqlite3
    database = query_rig[5].connection.execute('PRAGMA database_list').fetchone()[2]
    with sqlite3.connect(database) as reopened:
        row = reopened.execute('SELECT receipt_json FROM pipette_operations WHERE command_id=?',
                               (result['command_id'],)).fetchone()
    persisted = json.loads(row[0])
    assert persisted['result']['collection_source'] == result['collection_source']
    assert compact['attempt_id'] in row[0]
    # Matched populated producer data, not a workstation motion benchmark.
    import os, timeit
    from pathlib import Path
    full = owner.projection()
    current = owner.projection(compact_startup=True)
    before = {'startup': full['startup'], 'lifecycle': full}
    after = {'startup': current.pop('startup'), 'lifecycle': current}
    metrics = {
        'before_bytes': len(json.dumps(before).encode()),
        'after_bytes': len(json.dumps(after).encode()),
        'before_encode_s_100': timeit.timeit(lambda: json.dumps(before), number=100),
        'after_encode_s_100': timeit.timeit(lambda: json.dumps(after), number=100),
        'scope': 'startup/lifecycle portion; synthetic wire through real constructor and SQLite',
    }
    assert metrics['after_bytes'] < metrics['before_bytes']
    if os.environ.get('BIOXP_DEBLOAT_API_METRICS'):
        Path(os.environ['BIOXP_DEBLOAT_API_METRICS']).write_text(json.dumps(metrics, indent=2))
