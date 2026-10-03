"""Explicit custody actions through actual finite worker and source publications."""
import threading
import pytest
from tests.test_deck_scoped_authority import retained_rig
from tests.test_cover_carry_release_connected import connected
from tests.test_native_completion_handoff import handoff


@pytest.fixture(autouse=True)
def empty_inspection_predecessor(retained_rig):
    # This test deliberately reuses the empty-inventory inspection fixture.
    # Reset only the scratch copy via its actual semantic publisher; retained
    # source captures and historical records are never edited.
    provider, _, _, _, store, _ = retained_rig
    store.publish_deck_owner_state(source_operation="updatePlateLocation",
        source_command_id="method-fixture-empty-inventory", updates={"movable_plate_locations": {}},
        **provider.deck_owner_authority_stamps())


@pytest.mark.parametrize("failure", [False, True])
def test_explicit_catch_release_native_custody(handoff, monkeypatch, failure):
    from bioxp import api
    from bioxp.services.protocol_service import bind_protocol_dispatcher, create_protocol_job, ProtocolOperatorBundleStore
    rig = handoff
    store = rig.store
    original = rig.native.motor_oem_move_absolute
    def move(board, target, **kw):
        if failure and len(rig.admitted) == 2 and store.deck_semantic_state()["plate_on_gantry"] == 4:
            raise RuntimeError("method_release_physical_leaf_failure")
        return original(board, target, **kw)
    monkeypatch.setattr(rig.native, "motor_oem_move_absolute", move)
    actions = [{"action_id": "catch", "kind": "plate_catch", "params": {"plate": 4},
                "metadata": {"step_id": "custody", "child_index": 0}},
               {"action_id": "release", "kind": "plate_release", "params": {"destination": 18},
                "metadata": {"step_id": "custody", "child_index": 1}}]
    payload = {"idempotency_key": "explicit-custody", "live_execution": {"live_execution_ack": True},
               "document": {"protocol_id": "custody", "stages": [{"stage_id": "s", "actions": actions}]}}
    handlers = {"plate_catch": api._protocol_live_custody_handler, "plate_release": api._protocol_live_custody_handler}
    artifacts = ProtocolOperatorBundleStore(rig.root / "method-artifacts")
    bind_protocol_dispatcher(store, binding_factory=lambda *a, **k: (handlers, {}, {}), artifact_store=artifacts)
    done = threading.Event()
    finish = store.finish_workflow
    def finished(*args, **kwargs):
        result = finish(*args, **kwargs)
        done.set()
        return result
    monkeypatch.setattr(store, "finish_workflow", finished)
    job = create_protocol_job(payload, dry_run=False, command_store=store, store=artifacts,
        handlers=handlers, ownership_generation=rig.provider.deck_owner_authority_stamps()["ownership_generation"], board_epochs={})
    store.start(rig.plane._dispatch_one)
    assert done.wait(20), store.get_workflow(job["job_id"])
    result = store.get_workflow(job["job_id"])
    rows = result["execution"]["runtime_state"]["action_results"]
    import json
    (rig.root / "method-result.json").write_text(json.dumps(result, indent=2))
    assert len(rows) == 2, str(rig.root / "method-result.json")
    assert rows[0]["ok"] is True and rows[0]["metadata"] == actions[0]["metadata"]
    assert [r[0] for r in rig.admitted] == ["catch_plate", "release_plate"]
    for row, (_, _, cid) in zip(rows, rig.admitted):
        durable = store.wp8_operation_evidence(cid)["children"]
        assert [(x["child_order"], x["operation"], x["status"]) for x in row["child_outcomes"]] == [
            (x["child_order"], x["operation"], x["terminal_state"]) for x in durable]
    semantic = store.deck_semantic_state()
    if failure:
        assert rows[1]["ok"] is False and semantic["plate_on_gantry"] == 4
        assert result["command"]["status"] == "ambiguous"
        child = store.wp8_operation_evidence(rig.admitted[1][2])
        assert "method_release_physical_leaf_failure" in repr(child)
    else:
        assert result["command"]["status"] == "completed", result
        assert semantic["plate_on_gantry"] is None
        assert semantic["movable_plate_locations"]["OUTPUT_COVER"] == "LOC_OC_COVER_STORAGE"
