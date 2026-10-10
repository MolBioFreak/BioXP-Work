"""Cross-owner Cavro producer, public capture and discovery receiving.

Physical inputs are controlled fixture frames; no robot or instrument is used.
"""
import json
import os
import time
import pytest
from decimal import Decimal, localcontext
from pathlib import Path

from bioxp import api
from bioxp.calibration_analysis import analyze
from bioxp.pipette.cavro_application import compile_application
from bioxp.pipette.cavro_liquid import compile_liquid_recipe, correction_at_target
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.novo_router import NovoFrame, PRESSURE_STREAM_IDS
from bioxp.protocols.method_contract import method_contract
from bioxp.protocols.operational_results import application_outcome
from tests.test_cavro_application import request, rig as wire_rig
from tests.test_cavro_liquid import recipe
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_method_runtime_connected import mount, submit, action
from tests.test_protocol_workflow_connected import await_job


def export(name, body):
    directory = os.environ.get("NON_OEM_NATIVE_EXPORT_DIR")
    if directory:
        path = Path(directory)
        path.mkdir(parents=True, exist_ok=True)
        (path / (name + ".json")).write_text(json.dumps(body, indent=2))


def test_actual_analysis_decimal_output_compiles_without_float_roundtrip():
    analysis = analyze({"kind": "pipette", "model": "affine_inverse_ols", "context": {}, "samples": [
        {"point_id": "low", "target_ul": "10", "commanded_ul": "10", "observation": {"kind": "volume", "measured_ul": "9"}},
        {"point_id": "high", "target_ul": "20", "commanded_ul": "20", "observation": {"kind": "volume", "measured_ul": "18"}},
    ]})
    raw = recipe()
    raw.pop("commanded_aspiration_ul")
    raw["selected_correction"] = {"calibration_id": "inert-fit", "revision": "r1", "context": {}, "function": analysis["correction"]}
    compiled = compile_liquid_recipe(raw)
    assert not compiled["issues"], compiled["issues"]
    expected = correction_at_target(raw["target_liquid_ul"], analysis["correction"])
    aspirate = next(op for op in compiled["operations"] if op["operation"] == "aspirate")
    assert aspirate["volume_ul"] == expected
    assert aspirate["target_liquid_ul"] == raw["target_liquid_ul"]
    assert Decimal(aspirate["wire_commands"][-1]["ascii"][1:-3]) == Decimal(expected)
    assert compiled["liquid_settings"]["selected_correction"]["application_count"] == 1
    export("analysis-to-recipe", {"analysis": analysis, "recipe": raw, "compiled": compiled})


def test_exact_point_producer_and_original_numeric_request_both_compile():
    analysis = analyze({"kind": "pipette", "model": "exact_point_offset", "context": {}, "samples": [
        {"point_id": "p", "target_ul": "20", "commanded_ul": "20", "observation": {"kind": "volume", "measured_ul": "19.125"}},
    ]})
    raw = recipe()
    original = compile_liquid_recipe(raw)
    assert not original["issues"]
    raw.pop("commanded_aspiration_ul")
    raw["selected_correction"] = {"calibration_id": "inert-point", "revision": "r1", "context": {}, "function": analysis["correction"]}
    compiled = compile_liquid_recipe(raw)
    assert not compiled["issues"], compiled["issues"]
    aspirate = next(op for op in compiled["operations"] if op["operation"] == "aspirate")
    assert aspirate["wire_commands"][-1]["ascii"] == "P20.875,1R"
    for invalid in (-1, "-1"):
        assert compile_application(request({"operation": "aspirate", "channels": [0], "timeout_ms": 20, "volume_ul": invalid, "speed_ul_s": 1}))["issues"]


def test_generated_method_contract_contains_actual_calibration_and_device_producers():
    from bioxp.calibration_workflow import calibration_contract
    from bioxp.pipette.cavro_commands import device_command_catalog
    contract = method_contract()
    assert contract["action_params"]["calibration"] == calibration_contract()["action_params"]
    assert contract["calibration"] == calibration_contract()
    assert contract["device_commands"] == device_command_catalog()
    names = contract["cavro_application"]["$defs"]
    assert "PressureCapture" in names and "PressureCaptureRequest" in names
    assert "DeviceCommandInstruction" in names
    export("integrated-native-method-contract", contract)


def test_geometry_survives_operational_projection():
    value = {"target_steps": 123, "reference_steps": 120, "speed_native": 7,
             "shared_head": {"shared_z": True, "inactive_mounted_tips_move": True,
                             "independent_channel_z": False, "unequal_authored_surface_references": True},
             "physical_surface_verified": False, "commanded_displacement_ul": "0.1234567890123456789",
             "liquid_phase": "tracked_aspirate", "start_mode": 1}
    assert application_outcome(value) == value


def test_public_capture_instruction_actual_finite_sqlite_and_http_readback(wire_rig, installed_retained, monkeypatch, tmp_path):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    # The retained fixture mounts only selected production routes. Include
    # the actual new reader, not a stand-in response or a second handler.
    app.add_api_route("/pipettes/pressure-captures/{stream_session_id}",
                      api.scientific_pressure_capture_read, methods=["GET"])
    provider = installed_retained[1]
    provider.primitives.pipette_transport = wire_rig.group
    receipts = PipetteReceiptStore(installed_retained[4])
    monkeypatch.setattr(api, "_pipette_receipts", receipts)
    monkeypatch.setattr(api, "_require_motion_route_ready", lambda: None)
    def inject(_):
        data = b"1\x00\x08"
        wire_rig.router._dispatch(NovoFrame(min(PRESSURE_STREAM_IDS), len(data), data, b"", time.monotonic(), "pressure"))
    wire_rig.group._sleep = inject
    application = request({"operation": "pressure_capture", "capture": {"channels": [0], "duration_ms": 20, "max_frames": 2}})
    before = list(wire_rig.wire)
    job = submit(app, client, [action("pipette_manual_physical", {"operation": "cavro_application", "application": application})], key="pressure-public-integration")
    done = await_job(client, job, lambda r: r["command"]["terminal"])
    result = done["execution"]["runtime_state"]["action_results"][0]["pipette_result"]
    assert result["ok"], result.get("error")
    capture = result["events"][0]["result"]
    assert capture["kind"] == "scientific_pressure_capture"
    session = capture["scientific_pressure"]["stream_session_id"]
    response = client.get("/pipettes/pressure-captures/" + session)
    assert response.status_code == 200, response.text
    readback = response.json()
    assert readback["chunks"][0]["scientific_result"]["samples"][0]["raw_pressure"] == 8
    assert PipetteReceiptStore(installed_retained[4]).scientific_pressure_capture(session) == readback
    assert client.get("/pipettes/pressure-captures/not-a-capture").status_code == 404
    assert wire_rig.wire == before  # Listening/readback never enables streaming.
    assert client.get("/protocol/jobs/" + job).json()["execution"]["runtime_state"] == done["execution"]["runtime_state"]
    export("pressure-public-receiving", {"application": application, "job": done, "capture": readback, "wire": wire_rig.wire})


from tests.test_cavro_transport_repairs import receiver
from tests.test_cavro_failure_receiving import fresh_read


@pytest.mark.parametrize("partial_failure", [False, True])
def test_buffered_diagnostic_public_finite_result_and_cold_receipt(
        receiver, installed_retained, monkeypatch, tmp_path, partial_failure):
    """Connect D2 answers with D3 failed receipts and the protocol projection."""
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    provider, root = installed_retained[1], installed_retained[4]
    provider.primitives.pipette_transport = receiver.group
    receipts = PipetteReceiptStore(root)
    monkeypatch.setattr(api, "_pipette_receipts", receipts)
    if partial_failure:
        receiver.fault["missing"] = 1
    before = {r["command_id"] for r in receipts.connection.execute("SELECT command_id FROM pipette_operations")}
    op = {"operation": "device_program", "channels": [0, 1] if partial_failure else [0],
          "timeout_ms": 20, "program": {"mode": "execute", "commands": [
              {"command": "diagnostic", "test": "minimum_moving_current"}]}}
    job_id = submit(app, client, [action("pipette_manual_physical", {
        "operation": "cavro_application", "application": request(op)})],
        key="integrated-diagnostic-" + str(partial_failure))
    job = await_job(client, job_id, lambda r: r["command"]["terminal"])
    outcome = job["execution"]["runtime_state"]["action_results"][0]["pipette_result"]
    assert outcome["ok"] is not partial_failure
    event = outcome["events"][0]
    returned = event["partial"]["pipette_partial_result"] if partial_failure else event["result"]
    emitted = returned["channels"][0]["result"]
    reports = emitted["reports"]
    assert len(reports) == 1 and reports[0]["value"] == 123
    assert reports[0]["units"] == "ma" and reports[0]["command_index"] == 0
    assert reports[0]["response"]["receive_sequence"] is not None
    assert emitted["buffer_context"]["preloaded_contents_verified"] is False
    assert emitted["response_descriptors"][0]["report_kind"].startswith("diagnostic_")
    rows = [dict(r) for r in receipts.connection.execute(
        "SELECT command_id,pipette_operation_id,status,receipt_json FROM pipette_operations ORDER BY command_id")
        if r["command_id"] not in before]
    assert len(rows) == 1
    retained = json.loads(rows[0]["receipt_json"])
    retained = retained if partial_failure else retained["result"]
    assert retained["channels"][0]["result"]["reports"] == reports
    if partial_failure:
        assert returned["failed_channel"] == retained["failed_channel"] == 1
        assert rows[0]["status"] == "failed"
    else:
        assert rows[0]["status"] == "completed"
        assert returned["completion_verified"] is True
        assert returned["channels"][0]["completion"]["active_answers"]
    cold = fresh_read(root)
    assert [r for r in cold["rows"] if r["command_id"] not in before] == rows
    assert client.get("/protocol/jobs/" + job_id).json()["execution"]["runtime_state"] == job["execution"]["runtime_state"]
    assert len(receiver.wire) == (2 if partial_failure else 1)
    export("integrated-public-diagnostic-" + str(partial_failure),
           {"job": job, "receipts": rows, "cold": cold, "wire": receiver.wire})
