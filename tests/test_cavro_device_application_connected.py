"""Actual codec -> application -> finite provider -> transport -> receipt/API.

Requires committed command and transport producer modules, never fake codecs.
Only controlled endpoint frames are synthetic; no hardware is accessed.
"""
import json
import pytest
from bioxp import api
from bioxp.pipette.receipts import PipetteReceiptStore
from tests.test_cavro_transport_repairs import receiver
from tests.test_cavro_application import request, rig as wire_rig
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_method_runtime_connected import mount, submit, action
from tests.test_protocol_workflow_connected import await_job
from tests.test_cavro_surface_coordination import export


@pytest.mark.parametrize("failure", [False, True])
def test_actual_device_instruction_receiving(receiver, installed_retained, monkeypatch, tmp_path, failure):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    provider = installed_retained[1]
    provider.primitives.pipette_transport = receiver.group
    receipts = PipetteReceiptStore(installed_retained[4])
    monkeypatch.setattr(api, "_pipette_receipts", receipts)
    monkeypatch.setattr(api, "_require_motion_route_ready", lambda: None)
    if failure:
        receiver.fault["device"] = 1
    base = {"channels": [0, 1], "timeout_ms": 100}
    application = request(
        {"operation": "device_command", **base, "command": {"command": "report", "selector": 57}},
        {"operation": "device_command", **base, "command": {"command": "plunger_aspirate", "value": "22.5", "unit": "ul"}},
        {"operation": "device_program", **base, "program": {"mode": "execute", "commands": [
            {"command": "delay", "milliseconds": 20},
            {"command": "plunger_dispense", "value": "2.5", "unit": "ul"}]}},
        {"operation": "device_program", **base, "program": {"mode": "append", "commands": [
            {"command": "plunger_dispense", "value": 1, "unit": "ul"}]}})
    job = submit(app, client, [action("pipette_manual_physical", {"operation": "cavro_application",
        "application": application})], key="device-receiving-" + str(failure))
    done = await_job(client, job, lambda r: r["command"]["terminal"])
    source = done["execution"]["runtime_state"]["action_results"][0]["pipette_result"]
    export("device-native-" + str(failure), {"request": application, "result": done, "wire": receiver.wire})
    assert source["ok"] is not failure, source.get("error")
    report = source["events"][0]["result"]
    assert report["channels"][0]["result"]["report"]["value"] == 1.25
    assert report["channels"][0]["result"]["report"]["units"] == "psi"
    assert not report["completion_verified"]
    if failure:
        assert len(source["events"]) == 2
        assert source["events"][1]["result"]["channels"][0]["completion"]["ok"]
    else:
        assert len(source["events"]) == 4
        assert source["events"][2]["result"]["completion_verified"]
        assert not source["events"][3]["result"]["completion_verified"]
    rows = receipts.read(limit=100)
    assert rows
    assert PipetteReceiptStore(installed_retained[4]).read(limit=100) == rows
    finite_ids = [cid for cid in done["execution"]["runtime_state"]["workflow"]["child_command_ids"] if not cid.startswith("pipette_")]
    assert len(finite_ids) == 1
    durable = app.state.operator_command_plane.store.wp8_operation_evidence(finite_ids[0])
    assert durable["children"][0]["operation"] == "sourceCavroApplication"
    assert client.get("/protocol/jobs/" + job).json()["execution"]["runtime_state"] == done["execution"]["runtime_state"]


def test_corrected_recipe_air_and_target_cross_owner_receipts(wire_rig, installed_retained, monkeypatch, tmp_path):
    from tests.test_cavro_liquid import recipe
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    provider = installed_retained[1]
    provider.primitives.pipette_transport = wire_rig.group
    receipts = PipetteReceiptStore(installed_retained[4])
    monkeypatch.setattr(api, "_pipette_receipts", receipts)
    monkeypatch.setattr(api, "_require_motion_route_ready", lambda: None)
    raw = recipe()
    raw.pop("commanded_aspiration_ul")
    raw["selected_correction"] = {"calibration_id": "test-only", "revision": "r1", "context": {},
        "function": {"kind": "points", "points": [{"target_ul": 20, "commanded_ul": 22.75}]}}
    job = submit(app, client, [action("pipette_manual_physical", {"operation": "cavro_liquid_recipe",
        "recipe": raw})], key="corrected-receiving")
    done = await_job(client, job, lambda r: r["command"]["terminal"])
    source = done["execution"]["runtime_state"]["action_results"][0]["pipette_result"]
    assert source["ok"], source.get("error")
    aspirate = next(e for e in source["events"] if e["operation"] == "aspirate")
    account = aspirate["result"]["channels"][0]["result"]["accounting"]
    assert account["pneumatic_displacement_delta_ul"] == 22.75
    assert account["intended_liquid_delta_ul"] == 20
    assert account["measured_liquid_ul"] is None
    air = [e for e in source["events"] if e["operation"] == "aspirate air"]
    assert [e["inputs"]["liquid_phase"] for e in air] == ["leading_air", "trailing_air"]
    assert all(e["result"]["channels"][0]["result"]["accounting"]["intended_liquid_delta_ul"] == 0 for e in air)
    assert wire_rig.wire.count((0, "P22.75,1R")) == 1
    assert source["liquid_settings"]["selected_correction"]["application_count"] == 1
    export("corrected-receiving", {"recipe": raw, "result": done, "wire": wire_rig.wire,
        "receipts": receipts.read(limit=100)})
