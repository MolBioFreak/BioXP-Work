"""Complete real executor boundaries with inert leaves and private SQLite."""
import json
import os
import time
import copy
import pytest
from types import SimpleNamespace

from tests.deck_scoped_rig import retained_rig


@pytest.mark.parametrize("action_count", [2, 64])
def test_complete_large_workflow(retained_rig, monkeypatch, action_count):
    from bioxp.protocols.models import ProtocolDocument, ProtocolActionKind
    from bioxp.protocols.runtime_state import ProtocolRuntimeState
    from bioxp.services.protocol_service import bind_protocol_dispatcher
    from tests.test_protocol_workflow_connected import request_payload
    _, _, _, _, store, root = retained_rig
    payload = request_payload('lean')['document']
    payload['metadata'] = {'large_authored_input': 'IMMUTABLE_' * 300000}
    template = payload['stages'][0]['actions'][0]
    payload['stages'][0]['actions'] = [{**copy.deepcopy(template), 'action_id': f'action-{i}'} for i in range(action_count)]
    for stage in payload['stages']:
        for action in stage['actions']:
            action['stage_id'] = stage['stage_id']
    doc = ProtocolDocument.from_payload(payload)
    state = ProtocolRuntimeState.from_document(doc, dry_run=False, job_id='lean-parent')
    bundle = {'protocol': {'document': doc.to_payload(), 'source_type': 'document'},
        'execution': {'dry_run': False, 'runtime_state': state.to_payload()},
        'operator': {'reviews': []}, 'created_at': 'fixture-created'}
    calls = []
    bind_protocol_dispatcher(store, binding_factory=lambda *a, **k: (
        {ProtocolActionKind.LED: lambda action, state: calls.append(action.action_id) or {'ok': True}}, {}, {}),
        artifact_store=SimpleNamespace(save=lambda bundle: None))
    store.admit_workflow(command_id='lean-parent', idempotency_key='lean-parent', plan_fingerprint='lean',
        requested_inputs={'bundle': bundle}, ownership_generation=7, resources=(), board_epochs={})
    writes, other_writes = [], []
    store.connection.create_function('measure_workflow_write', 1, lambda text: writes.append(len(text.encode())) or 0)
    store.connection.execute("CREATE TEMP TRIGGER measure_workflow AFTER UPDATE OF receipt_json ON operator_commands WHEN NEW.command_id='lean-parent' BEGIN SELECT measure_workflow_write(NEW.receipt_json); END")
    store.connection.create_function('measure_other_write', 2, lambda kind, text: other_writes.append((kind, len(text.encode()))) or 0)
    store.connection.execute("CREATE TEMP TRIGGER measure_terminal AFTER UPDATE OF terminal_json ON operator_plane_commands WHEN NEW.command_id='lean-parent' BEGIN SELECT measure_other_write('terminal',NEW.terminal_json); END")
    store.connection.execute("CREATE TEMP TRIGGER measure_report AFTER INSERT ON operator_plane_command_versions WHEN NEW.command_id='lean-parent' BEGIN SELECT measure_other_write('report',NEW.row_json); END")
    store.connection.execute("CREATE TEMP TRIGGER measure_input AFTER UPDATE OF payload_json ON operator_plane_evidence WHEN NEW.command_id='lean-parent' BEGIN SELECT measure_other_write('input',NEW.payload_json); END")
    original_publish = store.publish_workflow
    timings, unchanged_writes, serialized_bytes, immutable_encodes = [], [], [], []
    import bioxp.operator_command_plane as plane
    encode = plane._canonical
    def measured(*args, **kwargs):
        encoded = []
        def count(value):
            text = encode(value)
            encoded.append(len(text.encode()))
            immutable_encodes.append('IMMUTABLE_' in text)
            return text
        started = time.perf_counter()
        with monkeypatch.context() as mp:
            mp.setattr(plane, '_canonical', count)
            result = original_publish(*args, **kwargs)
        timings.append(time.perf_counter() - started)
        serialized_bytes.append(sum(encoded))
        before = store.connection.total_changes
        original_publish(*args, **kwargs)
        unchanged_writes.append(store.connection.total_changes - before)
        return result
    monkeypatch.setattr(store, 'publish_workflow', measured)
    claimed = store.claim_next()
    assert claimed['command_id'] == 'lean-parent'
    assert store._renew_owner(lease_seconds=120)
    started = time.perf_counter()
    store._workflow_dispatcher(claimed)
    elapsed = time.perf_counter() - started
    final = store.get_workflow('lean-parent')
    assert final['status'] == 'completed'
    assert calls == [f'action-{i}' for i in range(action_count)]
    assert final['protocol'] == bundle['protocol']
    with monkeypatch.context() as mp:
        mp.setattr(store, '_workflow_inputs', lambda *a: (_ for _ in ()).throw(AssertionError('heavy input read'))) if hasattr(store, '_workflow_inputs') else None
        observation = store.get_workflow_observation('lean-parent')
        assert observation['execution']['runtime_state']['workflow'] == final['execution']['runtime_state']['workflow']
        assert store.list_workflow_summaries(limit=1)[0]['protocol_id'] == doc.protocol_id
    receipt = json.loads(store.connection.execute("SELECT receipt_json FROM operator_commands WHERE command_id='lean-parent'").fetchone()[0])
    evidence = {'action_count': action_count, 'elapsed_seconds': elapsed, 'publish_seconds': timings, 'receipt_writes': len(writes),
        'receipt_write_bytes': sum(writes), 'maximum_receipt_bytes': max(writes),
        'publication_python_encoded_bytes': sum(serialized_bytes), 'immutable_publication_encodes': sum(immutable_encodes),
        'other_write_counts': {kind: sum(k == kind for k, _ in other_writes) for kind in ('terminal', 'report', 'input')},
        'other_write_bytes': {kind: sum(n for k, n in other_writes if k == kind) for kind in ('terminal', 'report', 'input')},
        'duplicate_call_changes': unchanged_writes, 'authored_bytes': len(json.dumps(bundle['protocol']))}
    print('LEAN_MEASUREMENT=' + json.dumps(evidence, sort_keys=True))
    for trigger in ('measure_workflow', 'measure_terminal', 'measure_report', 'measure_input'):
        store.connection.execute('DROP TRIGGER ' + trigger)
    if os.environ.get('WORKFLOW_LEAN_BASELINE'):
        return
    assert max(writes) < 100000, evidence
    assert not any(unchanged_writes), evidence
    assert not any(immutable_encodes), evidence
    assert not any(kind == 'input' for kind, _ in other_writes), evidence
    assert 'protocol' not in receipt
    canonical = store.get_command('lean-parent')
    assert canonical['requested_inputs']['bundle']['protocol'] == bundle['protocol']
    assert canonical['terminal_evidence']['protocol'] == bundle['protocol']
    from bioxp.operator_command_receipts import CommandReceiptReader
    assert CommandReceiptReader(store.connection, store._lock).get_command('lean-parent')['terminal_evidence']['protocol'] == bundle['protocol']
    from bioxp.operator_receipt_store import OperatorHistoryReader
    history = OperatorHistoryReader(root)
    try:
        assert history.get_command('lean-parent')['terminal_evidence']['protocol'] == bundle['protocol']
    finally:
        history.close()
    for table, column in [('operator_commands', 'requested_inputs_json'), ('operator_plane_commands', 'requested_json'), ('operator_plane_command_versions', 'row_json')]:
        rows = store.connection.execute(f"SELECT {column} FROM {table} WHERE command_id='lean-parent'").fetchall()
        assert rows and all('IMMUTABLE_' not in row[0] for row in rows)
    authored = store.connection.execute("SELECT payload_json FROM operator_plane_evidence WHERE command_id='lean-parent' AND evidence_kind='workflow_input'").fetchall()
    assert len(authored) == 1 and json.loads(authored[0][0])['protocol'] == bundle['protocol']
    from bioxp.oem_runtime_store import verify_canonical_runtime_database
    verify_canonical_runtime_database(store.connection, full_data_check=True)
    from bioxp.services.protocol_service import ProtocolOperatorBundleStore
    exported = ProtocolOperatorBundleStore(root / 'exports').save(final)
    persisted = json.loads((root / 'exports' / 'lean-parent' / 'bundle.json').read_text())
    assert persisted['protocol'] == exported['protocol'] == bundle['protocol']
    assert '_workflow_receipt' not in persisted


def test_legacy_full_receipt_remains_readable(retained_rig):
    _, _, _, _, store, _ = retained_rig
    row = store.connection.execute("SELECT command_id,receipt_json,requested_inputs_json FROM operator_commands WHERE command_kind='protocol_workflow' ORDER BY sequence LIMIT 1").fetchone()
    assert row is not None
    original = json.loads(row['receipt_json'])
    assert 'protocol' in original and '_workflow_receipt' not in original
    read = store.get_workflow(row['command_id'])
    assert read['protocol'] == original['protocol']
    assert read['execution'] == original['execution']
    assert store.connection.execute('SELECT receipt_json FROM operator_commands WHERE command_id=?', (row['command_id'],)).fetchone()[0] == row['receipt_json']
