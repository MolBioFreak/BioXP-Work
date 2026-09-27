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


@pytest.mark.parametrize("kind", ["source_load_tips", "source_mix", "source_aspirate_air",
    "source_dispense_air", "source_purge", "diagnostic_pipette"])
@pytest.mark.parametrize("container", ["completed_children", "source_children", "provider_results"])
def test_additional_typed_outcome_is_idempotent_and_not_transport_bulk(kind, container):
    source = {"kind": kind, "ok": False, "source_return": None,
        "error": "partial source failure", "tip_location": -1,
        "native_results": [{"operation": "sent", "result": {"ok": False,
            "driver_result": {"bulk": "x" * 200000, "error": "wire failure"}}}],
        "tests": [{"number": 0, "channels": [{"channel": 0, "diagnosis": None,
            "display": "No data returned"}]}]}
    body = {container: [{"result": source}], "bulk": "x" * 200000}
    result = _bounded_json(body, 131072)["pipette_result"]
    assert result['kind'] == kind and result['source_return'] is None
    assert result['tip_location'] == -1 and not result['ok']
    assert result['tests'][0]['channels'][0]['diagnosis'] is None
    assert 'wire failure' in result['source_errors']
    assert 'driver_result' not in json.dumps(result)
    assert _bounded_json(result, 1) == result
    assert len(json.dumps(result)) < 2000


def test_exported_additional_results_are_complete_native_action_rows():
    from pathlib import Path
    data = json.loads((Path(__file__).parents[1] /
        'testdata/pipette_completion/additional-results.json').read_text())
    names = [row['name'] for row in data['cases']]
    assert len(names) == len(set(names)) == 21
    for case in data['cases']:
        action = case['action_result']
        assert action['command_id'] and action['action_id']
        assert case['request']['steps']
        assert _bounded_json(action, 1)['pipette_result'] == action['pipette_result']
    assert {'selected_load', 'matching_load', 'failed_reload', 'partial_diagnosis',
        'aspirate', 'dispense', 'dispense_all', 'diagnoses', 'initialize', 'eject',
        'get_data', 'last_error', 'plunger_up', 'plunger_down'} <= set(names)
