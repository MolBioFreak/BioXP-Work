"""D1 passive producer and D2/D3 frozen-schema maintenance regressions."""
import json
import sqlite3

import pytest

from tests.test_deck_tip_query_publication import query_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_scoped_authority import retained_rig


@pytest.mark.parametrize("passive", [True, False])
def test_tip_receipt_only_passive_success_drops_heavy_duplicate(query_rig, passive):
    from bioxp import api
    _, _, _, _, _, receipts, calls, _, transport = query_rig
    binding = {"caller_class": "lifecycle", "idempotency_key": "data-bounded-tip",
               "entrypoint_id": "hardware.snapshot.park_tip_observation" if passive else "liquid.tip_status"}
    result = api._query_and_publish_pipette_tip_status(runtime_binding=binding, readiness_bounded=passive)
    assert calls == [0, 1, 2, 3]
    row = receipts.connection.execute("SELECT receipt_json,status FROM pipette_operations WHERE command_id=?",
                                      (result["command_id"],)).fetchone()
    stored = json.loads(row["receipt_json"])
    print(json.dumps({"passive": passive, "receipt_bytes": len(row["receipt_json"].encode()),
                      "stored_result_bytes": len(json.dumps(stored["result"]).encode())}))
    assert row["status"] == "observed"
    assert stored["result"]["collection_source"] == result["collection_source"]
    assert result["deck_state_publication"]["status"] == "published"
    # The active call and deck publication still receive the complete result.
    assert "ack" in result["channels"][0]["result"]
    if passive:
        assert set(stored["result"]) <= {"ok", "outcome", "collection_source", "channels", "hardware_query_verified"}
        assert "deployment_identity" not in stored
        for compact, actual in zip(stored["result"]["channels"], result["channels"]):
            assert compact["tip_loaded"] is actual["tip_loaded"]
            assert compact["result"]["observed_at"] == actual["result"]["observed_at"]
            assert compact["result"]["source_tip_loaded"] is actual["result"]["source_tip_loaded"]
            assert set(compact["result"]) == {"observed_at", "source_tip_loaded"}
    else:
        assert "ack" in stored["result"]["channels"][0]["result"]
        assert "deployment_identity" in stored
    state = receipts.collection_state(identity=transport.collection_source_identity(),
        ownership_generation=stored["ownership_epoch"])
    assert state["tip_exists"] is False
    replay = receipts.replay_result(command_id=result["command_id"],
        pipette_operation_id=receipts.connection.execute(
            "SELECT pipette_operation_id FROM pipette_operations WHERE command_id=?", (result["command_id"],)).fetchone()[0])
    assert replay["replayed"] and replay["ok"]
    assert replay["collection_source"] == result["collection_source"]


def test_failed_passive_tip_keeps_diagnostics(query_rig):
    from bioxp import api
    from fastapi import HTTPException
    _, _, _, _, _, receipts, _, wire, _ = query_rig
    wire["missing"] = True
    with pytest.raises(HTTPException):
        api._query_and_publish_pipette_tip_status(runtime_binding={
            "caller_class": "lifecycle", "idempotency_key": "data-failed-tip",
            "entrypoint_id": "hardware.snapshot.park_tip_observation"}, readiness_bounded=True)
    stored = json.loads(receipts.connection.execute(
        "SELECT receipt_json FROM pipette_operations ORDER BY rowid DESC LIMIT 1").fetchone()[0])
    assert stored["result"]["ok"] is False
    assert "ack" in stored["result"]["channels"][0]["result"]
    assert "deployment_identity" in stored


def test_repeat_open_never_calls_migration_backup(tmp_path, monkeypatch):
    from bioxp import oem_runtime_store as owner
    first = owner.OEMRuntimeStore(tmp_path)
    first.close()
    backups = sorted(tmp_path.glob("*pre-v2*"))
    def forbidden(*args, **kwargs):
        pytest.fail("schema-stable open attempted a migration backup")
    monkeypatch.setattr(owner, "_verified_sqlite_backup", forbidden)
    for _ in range(3):
        store = owner.OEMRuntimeStore(tmp_path)
        owner.verify_canonical_runtime_database(store._db, full_data_check=True)
        assert store._db.execute("PRAGMA quick_check").fetchone()[0] == "ok"
        store.close()
    assert sorted(tmp_path.glob("*pre-v2*")) == backups


def test_retention_deterministic_read_only_and_transactional_keep(tmp_path):
    from bioxp.oem_runtime_store import OEMRuntimeStore
    from bioxp.runtime_retention import retain_runtime_rows
    store = OEMRuntimeStore(tmp_path)
    store._db.execute("INSERT INTO operator_commands(command_id,idempotency_key,action_id,status,started_at,updated_at,receipt_json) "
        "VALUES('old-named','old-named','oem.deck.move_to_location','completed','2020-01-01',1,'{\"historical\":true}')")
    store._db.execute("UPDATE operator_commands SET updated_at=2 WHERE command_id='old-named'")
    store.close()
    before = sqlite3.connect(tmp_path / 'bioxp_runtime.db')
    original = list(before.iterdump())
    before.close()
    dry = retain_runtime_rows(tmp_path, as_of=1790636400)
    assert dry == retain_runtime_rows(tmp_path, as_of=1790636400)
    applied = retain_runtime_rows(tmp_path, as_of=1790636400, apply=True)
    assert applied == {**dry, "dry_run": False}
    assert applied["removed_rows"] == applied["receipt_bytes_removed"] == 0
    after = sqlite3.connect(tmp_path / 'bioxp_runtime.db')
    assert list(after.iterdump()) == original
    assert after.execute("PRAGMA quick_check").fetchone()[0] == "ok"
    with pytest.raises(sqlite3.IntegrityError, match="append-only"):
        after.execute("DELETE FROM operator_plane_command_versions")
    after.close()


@pytest.mark.parametrize("as_of", [float('nan'), float('inf'), -1, 0])
def test_retention_rejects_invalid_clock_without_creating_database(tmp_path, as_of):
    from bioxp.runtime_retention import retain_runtime_rows
    with pytest.raises(ValueError):
        retain_runtime_rows(tmp_path, as_of=as_of, apply=True)
    assert not list(tmp_path.iterdir())
