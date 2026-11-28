import asyncio

import pytest
from fastapi.testclient import TestClient

from bioxp import api
from tests.test_handler_repair_tips_and_fans import _tip_head


@pytest.mark.parametrize('invalid,still_mounted', [(False, True), (False, False), (True, True)])
def test_operator_confirmation_only_clears_selected_verified_empty(monkeypatch, tmp_path, invalid, still_mounted):
    group, present, _, invalid_query, wire = _tip_head(monkeypatch, [0, 1, 2])
    group.query_tip_status_all()
    present.difference_update([0, 2] if still_mounted else [0, 1, 2])
    wire.clear()
    if invalid:
        invalid_query.add(0)
    async def inline(_label, callback, **kwargs):
        return await asyncio.to_thread(callback)
    monkeypatch.setattr(api, '_run_blocking', inline)
    monkeypatch.setattr(api, '_get_pipette_transport', lambda: group)
    monkeypatch.setattr(api, '_serial206_oem_initialization_provider', None)
    from bioxp.oem_runtime_store import OEMRuntimeStore
    from bioxp.pipette import receipts
    monkeypatch.setattr(receipts, "current_release_identity", lambda: {"verified": True, "release_id": "offline",
        "source": {"manifest_sha256": "1" * 64, "aggregate_sha256": "2" * 64}})
    monkeypatch.setattr(receipts, "current_authority_identity", lambda: {"evidence_lock_identity_verified": True,
        "evidence_lock_sha256": "3" * 64})
    monkeypatch.setattr(receipts, "current_registry_sha256", lambda: "4" * 64)
    OEMRuntimeStore(tmp_path).close()
    store = receipts.PipetteReceiptStore(root=tmp_path)
    monkeypatch.setattr(api, '_pipette_receipts', store)
    response = TestClient(api.app).post('/liquid/confirm-tip-removal',
        json={'channels': [0, 1]}, headers={'Idempotency-Key': 'confirm-tip-test'})
    assert response.status_code == (502 if invalid else 200), response.text
    if invalid:
        assert group._expected_tip_channels == [0, 1, 2]
    else:
        result = response.json()
        assert result['reconciled_empty_channels'] == ([0] if still_mounted else [0, 1])
        assert result['still_loaded_channels'] == ([1] if still_mounted else [])
        assert result['physical_effect_verified'] is False
        assert group._expected_tip_channels == ([1, 2] if still_mounted else [2])
    row = store.connection.execute("SELECT status, requested_inputs_json FROM pipette_operations ORDER BY rowid DESC LIMIT 1").fetchone()
    assert 'intentionally_removed_channels' in row['requested_inputs_json']
    assert wire and all(command == '?31' for command, _ in wire)
