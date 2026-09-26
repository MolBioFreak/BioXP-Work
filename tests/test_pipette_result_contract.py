"""Critical source data never depends on survival of diagnostic envelopes."""
import json
import pytest

from bioxp.operator_controls import _bounded_json, _workflow_terminal_result


@pytest.mark.parametrize("container", ["completed_children", "source_children", "provider_results"])
def test_source_fields_survive_bounding_without_inventing_outcomes(container):
    body = {"source": "ControlLib.calwithFluid:2782-2850", "run_id": "run-original",
            "body_completed": False, "source_return": False,
            "saved_revision_id": "saved-original", "active_revision_id": "active-original",
            "comparison_choice": None, "pending_restart": True,
            "error": "third station failed", "finalization_error": "Park failed",
            "measurements": [{"plate": "TC", "measured_raw_z": 88000,
                              "saved_revision_id": "saved-original"}],
            "source_events": [{"operation": "log", "result": "x" * 200000}]}
    source = {container: [body if container == "provider_results" else {"result": body}],
              "diagnostic_bulk": "x" * 200000, "ok": False}
    bounded = _bounded_json(source, 131072)
    expected = {"kind": "source_calwith_fluid", **{k: v for k, v in body.items() if k != "source_events"}}
    assert bounded["pipette_result"] == expected
    assert "sha256" not in bounded and "preview" not in bounded
    assert len(json.dumps(bounded)) < 131072
    receipt = {"command_id": "command-original", "status": "ambiguous",
               "terminal_evidence": {"response": bounded, "error": "wp8_operation_exception:DeckExecutionFailure"}}
    native = _workflow_terminal_result(receipt)
    assert native["pipette_result"] == expected
    assert native["ok"] is False and native["error"] == receipt["terminal_evidence"]["error"]
    assert "run_id" not in _bounded_json({"completed_children": [{"result": {
        "source_anchor": "ControlLib.zOffset:3387-3627", "samples": []}}]}, 131072)["pipette_result"]


def test_large_sample_set_and_partial_scan_remain_lossless():
    samples = [{"well": "A1", "position_steps": n} for n in range(5000)]
    scan = {"source_anchor": "ControlLib.zOffset:3387-3627", "ok": False,
            "samples": samples, "error": "detector failure", "steps": [{"bulk": "x" * 200000}]}
    body = {"source": "ControlLib.calwithFluid:2782-2850", "measurements": [],
            "source_events": [{"operation": "zOffset:TC", "result": scan}]}
    bounded = _bounded_json({"provider_results": [body]}, 131072)
    assert bounded["pipette_result"]["failed_scans"][0]["samples"] == samples
    assert "steps" not in bounded["pipette_result"]["failed_scans"][0]
    assert _bounded_json(scan, 131072)["samples"] == samples
