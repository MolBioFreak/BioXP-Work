"""Native authored metrology. Tests never acquire physical instruments."""
from copy import deepcopy
from decimal import Decimal

import pytest
from pydantic import ValidationError
from bioxp.calibration_analysis import analyze
from bioxp.calibration_measurements import MeasurementForm, MeasurementResponse
from bioxp.protocols.compiler import compile_native_protocol
from bioxp.services.protocol_service import ProtocolOperatorBundleStore, create_protocol_job, review_protocol_job


def form(module="rc", repetitions=2):
    return {"fields": [{"field_id": "probe", "label": "External probe", "unit": "degC",
        "module": module, "target": "37.000", "commanded": "37.000", "repetitions": repetitions,
        "context": {"probe_id": "operator-authored-fixture"}}]}


def prompt(identity, module="rc", repetitions=2):
    return {"kind": "pause_review", "action_id": identity, "source_occurrence_id": "repeat/" + identity,
            "params": {"measurement_form": form(module, repetitions)}, "pause_message": "Measure external probe"}


def reference(action, index=0):
    return {"stage_id": "one", "action_id": action, "source_occurrence_id": "repeat/" + action,
            "output": "measurements", "path": ["probe", index]}


def pipette(model="descriptive"):
    return {"kind": "pipette", "model": model, "context": {"channel": 0, "tip": "fixture", "liquid": "fixture"},
        "samples": [{"point_id": "a", "target_ul": "20.00", "commanded_ul": "21.0",
            "observation": {"kind": "mass_density", "mass_mg": "19.8", "density_mg_per_ul": ".99"}},
            {"point_id": "a", "target_ul": "20.00", "commanded_ul": "21.0",
            "observation": {"kind": "volume", "measured_ul": "20.2"}}]}


def test_exact_values_descriptive_and_selected_point_correction():
    req = pipette("exact_point_offset")
    req["evaluate_targets_ul"] = ["20.00"]
    result = analyze(req)
    point = result["points"][0]
    assert point["measured"] == ["20", "20.2"]
    assert point["mean"] == "20.1" and point["bias"] == "0.10"
    assert Decimal(point["sample_sd"]) > 0 and point["count"] == 2
    assert result["correction"] == {"kind": "points", "points": [{"target_ul": "20.00", "commanded_ul": "20.90"}]}
    assert result["evaluated"] == [{"target_ul": "20.00", "commanded_ul": "20.90", "outside_authored_domain":None}]
    assert result["resolved_samples"][0]["observation"]["mass_mg"] == "19.8"
    assert result["applied"] is False
    # Evaluation is the existing producer; no inferred interpolation.
    req["evaluate_targets_ul"] = ["19.0"]
    with pytest.raises(ValueError, match="exact unique"):
        analyze(req)


def test_affine_model_must_be_explicit_and_invertible():
    req = pipette("affine_inverse_ols")
    req["samples"] = [{"point_id": str(i), "target_ul": str(i), "commanded_ul": str(i),
                       "observation": {"kind": "volume", "measured_ul": str(2*i+1)}} for i in (10,20,30)]
    req["evaluate_targets_ul"] = ["25"]
    result = analyze(req)
    assert result["correction"] == {"kind": "affine", "scale": "0.5", "offset_ul": "-0.5"}
    assert result["evaluated"][0]["commanded_ul"] == "12.0"
    assert all(p["sample_sd"] is None for p in result["points"])
    req["domain_ul"] = {"minimum_ul":"10", "maximum_ul":"20"}
    req["verification_samples"] = [{"point_id":"held-out", "target_ul":"25", "commanded_ul":"12",
                                   "observation":{"kind":"volume","measured_ul":"24.5"}}]
    with_verification = analyze(req)
    assert with_verification["correction"] == result["correction"]
    assert with_verification["evaluated"][0]["outside_authored_domain"] is True
    assert with_verification["evaluated"][0]["commanded_ul"] == "12.0"
    assert with_verification["verification_points"][0]["bias"] == "-0.5"
    assert with_verification["affine_fit"]["response_slope"] == "2"
    assert all(r["residual_ul"] == "0" for r in with_verification["affine_fit"]["residuals"])
    req["samples"] = req["samples"][:1]
    with pytest.raises(ValueError, match="distinct commanded"):
        analyze(req)
    del req["model"]
    with pytest.raises(ValidationError):
        analyze(req)


@pytest.mark.parametrize("value", [True, "NaN", "Infinity", "", [], None])
def test_non_numeric_is_not_a_measurement(value):
    with pytest.raises(ValidationError):
        MeasurementResponse.model_validate({"measurements": {"probe": [value]}})


def test_dry_review_reopen_repeated_occurrences_preserves_exact_response(tmp_path):
    store = ProtocolOperatorBundleStore(tmp_path)
    actions = [prompt("p0"), prompt("p1"), {"kind": "note", "action_id": "done"}]
    doc = {"protocol_id": "measure", "stages": [{"stage_id": "one", "actions": actions}]}
    compiled = compile_native_protocol(doc)
    assert compiled.to_payload()["stages"][0]["actions"][0]["params"]["measurement_form"] == form()
    held = create_protocol_job({"document": doc}, store=store)
    job = held["job_id"]
    for index in range(2):
        store = ProtocolOperatorBundleStore(tmp_path)
        held = store.load(job)
        pending = held["operator"]["pending_review"]
        assert pending["source_occurrence_id"] == f"repeat/p{index}"
        assert pending["measurement_form"] == form()
        with pytest.raises(ValueError, match="occurrence"):
            review_protocol_job(job, store=store, request={"source_occurrence_id":"wrong", "measurements":{"probe":[1,2]}})
        with pytest.raises(ValueError, match="count"):
            review_protocol_job(job, store=store, request={"source_occurrence_id":pending["source_occurrence_id"], "measurements":{"probe":[1]}})
        held = review_protocol_job(job, store=store, reviewer="external", note="not a sensor", request={
            "source_occurrence_id":pending["source_occurrence_id"], "measurements":{"probe":["36.9000", 37.1]}})
    loaded = ProtocolOperatorBundleStore(tmp_path).load(job)
    assert loaded["status"] == "completed"
    reviews = loaded["execution"]["runtime_state"]["measurement_reviews"]
    assert [r["source_occurrence_id"] for r in reviews] == ["repeat/p0", "repeat/p1"]
    assert all(r["measurements"] == {"probe":["36.9000",37.1]} for r in reviews)
    assert all(r["job_id"] == job for r in reviews)
    assert len(loaded["execution"]["runtime_state"]["action_results"]) == 3
    assert len(loaded["operator"]["reviews"]) == 2


def test_ordinary_note_review_unchanged(tmp_path):
    store = ProtocolOperatorBundleStore(tmp_path)
    held = create_protocol_job({"document":{"protocol_id":"note", "stages":[{"stage_id":"one","actions":[
        {"kind":"pause_review","action_id":"pause","pause_message":"Keep ordinary note"}]}]}},store=store)
    assert held["operator"]["pending_review"] == {"stage_id":"one","action_id":"pause","reason":"Keep ordinary note"}
    done = review_protocol_job(held["job_id"],store=store,note="ordinary note")
    assert done["status"] == "completed"
    assert done["execution"]["runtime_state"]["measurement_reviews"] == []
