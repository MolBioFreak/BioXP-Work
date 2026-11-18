"""Recovery cannot use a one-time idle check as an exclusive pipette lease."""
import pytest
from tests.deck_scoped_rig import retained_rig


@pytest.mark.parametrize("order", ["pipette_first", "workflow_first"])
def test_launch_claim_exclusion_boundary(retained_rig, monkeypatch, order):
    from bioxp.pipette import receipts
    from bioxp.pipette.receipts import PipetteReceiptStore
    _, _, _, _, store, root = retained_rig
    monkeypatch.setattr(receipts, "_current_replay_identity", lambda: {})
    pumps = PipetteReceiptStore(root)
    store.bind_workflow_dispatcher(lambda _: pytest.fail("no execution in claim probe"))
    store.admit_workflow(command_id="launch", idempotency_key="launch", plan_fingerprint="fixture",
                         requested_inputs={"bundle": {}}, ownership_generation=7,
                         resources=["pipette", "axis:z", "axis:x", "axis:y"], board_epochs={})
    def claim_direct(key="pump"):
        claim, created = pumps.claim(operation="aspirate", requested_inputs={}, entrypoint_id="direct",
                                     caller_class="direct_api", control_class="physical_liquid_command",
                                     idempotency_key=key, ownership_generation=7)
        assert created and claim["status"] == "reserved"
        return claim["command_id"]
    assert store.claim_next()["command_id"] == "launch"
    if order == "pipette_first":
        claim_direct()
        with pytest.raises(ValueError, match="method_tip_preflight_resource_busy"):
            with store.saved_method_tip_preflight("launch"):
                pytest.fail("entered with an active pump")
    else:
        with pytest.raises(RuntimeError, match="injected"):
            with store.saved_method_tip_preflight("launch"):
                with pytest.raises(ValueError, match="method_tip_preflight_active"):
                    claim_direct()
                with store.workflow_context("launch", source_occurrence_id="lifecycle:saved_method_tip_preflight:0:query"):
                    nested = claim_direct("nested")
                assert store.connection.execute("SELECT parent_command_id FROM operator_commands WHERE command_id=?", (nested,)).fetchone()[0] == "launch"
                raise RuntimeError("injected")
        claim_direct()
        store.assert_workflow_current("launch")
