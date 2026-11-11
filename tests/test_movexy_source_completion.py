"""Native moveXY log-and-continue reaches the deck/SQLite parent, offline wire only."""
import json
from tests.test_debloat_core import named
from tests.test_wake_setup_debloat import rig


def test_movexy_timeout_publishes_source_success_without_controller_proof(named, monkeypatch):
    run, provider, store, counts, frames, fault = named
    driver = provider.primitives.tester
    waits = []
    def timeout(addresses, **kw):
        waits.append((addresses, kw))
        return {"ok": False, "timeout": True, "per_axis": {
            axis: {"ok": False, "target_reached": False, "failure": "timeout"}
            for axis in ("x", "y")}}
    monkeypatch.setattr(driver, "motor_wait_target_reached_many", timeout)
    position = driver.motor_get_position
    def stopped(board, **kw):
        result = position(board, **kw)
        if board == 4 and kw.get("motor", 0) == 0:
            result["position"] = 13 if waits else 1700
            result["ack"]["value"] = result["position"]
        return result
    monkeypatch.setattr(driver, "motor_get_position", stopped)
    before = store.deck_semantic_state()["semantic_state_revision"]
    result, cid = run("LOC_OC_COVER_STORAGE", camera_offset=True)
    assert result["ok"] is True, json.dumps(result, indent=2)
    assert waits[0][1]["timeout_s"] == 5.0
    assert waits[0][1]["sta_sequential"] is True
    assert result["source_calls_completed"] is True
    assert result["controller_completion_verified"] is False
    assert result["hardware_postcondition_verified"] is False
    assert result["physical_effect_verified"] is False
    stage = result["provider_results"][-1]
    assert stage["oem_requested_y_steps"] < 0
    assert stage["oem_effective_y_steps"] == 0
    assert stage["source_calls_completed"] is True
    assert stage["controller_completion_verified"] is False
    rows = store.connection.execute("SELECT operation, terminal_state FROM operator_plane_deck_stages WHERE command_id=? ORDER BY stage_order", (cid,)).fetchall()
    assert rows[-1]["operation"] == "moveTo"
    assert all(row["terminal_state"] == "completed" for row in rows)
    semantic = store.deck_semantic_state()
    assert semantic["current_location"] == "LOC_OC_COVER_STORAGE"
    assert semantic["semantic_state_revision"] > before
    parent = store.connection.execute("SELECT status, terminal_json AS receipt_json FROM operator_plane_commands WHERE command_id=?", (cid,)).fetchone()
    assert parent["status"] == "completed"
    assert '"controller_completion_verified":false' in parent["receipt_json"].replace(" ", "")
    # No counter-reset or offset rewrite is introduced by a timeout.
    assert not any(command == 5 and typ == 1 for _, command, typ, _, _ in frames)


import pytest

@pytest.mark.parametrize("failure", ["delivery", "controller", "transport", "power", "waiting_motor"])
def test_movexy_reporting_does_not_swallow_real_failures(named, monkeypatch, failure):
    run, provider, store, counts, frames, fault = named
    driver = provider.primitives.tester
    before = store.deck_semantic_state()["current_location"]
    if failure == "delivery":
        fault["reject"] = (5, 4, 0, 0)
    elif failure == "controller":
        original = provider.primitives.oem_move_to
        def controller_error(*args, **kw):
            result = original(*args, **kw)
            # Retain a genuine source return with explicit controller error.
            result["controller_completion_verified"] = False
            result["controller_terminal_state_verified"] = False
            xy = result["operations"][-1]
            xy["controller_terminal_state_verified"] = False
            xy["axis_evidence"]["x"]["controller_error_events"] = [{"status": 130}]
            return result
        monkeypatch.setattr(provider.primitives, "oem_move_to", controller_error)
    elif failure == "power":
        driver.oem_latch_24v_dropped(reason="offline reporting regression")
    else:
        def broken(*a, **kw):
            raise RuntimeError("offline " + failure)
        if failure == "waiting_motor":
            store.publish_deck_owner_state(source_operation="pipette_owner", source_command_id="fixture-tip",
                updates={"tip_loaded": True, "tip_dirty": False, "tip_location": -1},
                **provider.deck_owner_authority_stamps())
            monkeypatch.setattr(driver, "motor_oem_move_absolute", broken)
        else:
            monkeypatch.setattr(driver, "motor_wait_target_reached_many", broken)
    try:
        result, cid = run()
    except RuntimeError:
        pass
    else:
        assert result["ok"] is False
        assert result["semantic_state_committed"] is False
    assert store.deck_semantic_state()["current_location"] == before
