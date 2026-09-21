"""Exact-byte integrity memo with detached callers and unchanged lifecycle gates."""
import copy
import json
import sqlite3

import pytest
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider as Provider


@pytest.fixture
def rig(tmp_path):
    store = OEMRuntimeStore(tmp_path)
    provider = Provider(object(), state_store=store)
    provider._save_state(provider._new_state())
    yield provider, store
    store.close()


def test_warm_read_decodes_once_without_redump(rig, monkeypatch):
    provider, store = rig
    provider._load_state()  # warm exact-byte validation
    dump, load = json.dumps, json.loads
    calls = {'dump': 0, 'load': 0}
    def dumps(*a, **k):
        calls['dump'] += 1
        return dump(*a, **k)
    def loads(*a, **k):
        calls['load'] += 1
        return load(*a, **k)
    monkeypatch.setattr(json, 'dumps', dumps)
    monkeypatch.setattr(json, 'loads', loads)
    first = provider._load_state()
    assert calls == {'dump': 0, 'load': 1}
    first['machine_status']['tip_loaded'] = True
    assert provider._load_state()['machine_status']['tip_loaded'] is False


def test_durable_read_does_not_copy_whole_state_again(rig, monkeypatch):
    provider, store = rig
    original = copy.deepcopy
    def checked(value, *a, **k):
        assert not (isinstance(value, dict) and 'movement_ledger' in value)
        return original(value, *a, **k)
    monkeypatch.setattr(copy, 'deepcopy', checked)
    assert provider._load_state()['schema_version']


def test_changed_bytes_are_not_trusted_just_because_digest_matches(rig):
    _, store = rig
    store.read_oem_serial206_initialization_state()
    # Simulate a corrupted selected row without disabling any persisted trigger.
    real = store._db
    class Selected:
        def execute(self, sql, *args):
            cur = real.execute(sql, *args)
            if sql.startswith('SELECT * FROM serial206_authority_snapshots'):
                row = dict(cur.fetchone())
                payload = json.loads(row['state_json'])
                payload['machine_status']['tip_loaded'] = True
                row['state_json'] = json.dumps(payload, sort_keys=True, separators=(',', ':'))
                return type('Cursor', (), {'fetchone': lambda self: row})()
            return cur
    store._db = Selected()
    try:
        with pytest.raises(RuntimeError, match='hash are incoherent'):
            store.read_oem_serial206_initialization_state()
    finally:
        store._db = real


def test_external_owner_write_visible_on_next_read(rig, tmp_path):
    provider, store = rig
    state = provider._load_state()
    other = OEMRuntimeStore(tmp_path)
    try:
        state['machine_status']['tip_loaded'] = True
        other.write_oem_serial206_initialization_state(state)
        assert provider._load_state()['machine_status']['tip_loaded'] is True
    finally:
        other.close()


def test_template_never_constructs_missing_legacy_trays():
    state = Provider._new_state()
    del state['machine_status']
    value = Provider._upgrade_state(state)
    assert 'constructed_tip_trays' not in value['machine_status']
    assert 'construction_id' not in value['machine_status']
    assert value['machine_status']['tip_loaded'] is None


def test_current_reads_do_not_reconstruct_defaults(monkeypatch):
    provider = Provider(object())
    state = provider._new_state()
    Provider._upgrade_template()  # one immutable migration template
    def forbidden(): raise AssertionError('reconstructed machine defaults')
    monkeypatch.setattr(Provider, '_new_state', staticmethod(forbidden))
    result = provider._validate_state(provider._upgrade_state(state))
    assert result == state and result is not state


def test_memory_nan_still_rejected():
    provider = Provider(object())
    state = provider._new_state()
    state['machine_status']['extra'] = float('nan')
    with pytest.raises(ValueError): provider._save_state(state)
    assert provider._memory_state is None
