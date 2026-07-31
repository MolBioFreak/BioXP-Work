
import json
from src.bioxp.oem_homing_spec import get_program
from src.bioxp.oem_parity_artifacts import build_artifact, validate_artifact


def test_build_artifact_has_required_truth_and_safety_fields():
    artifact = build_artifact(get_program("initialize_motors"), mode="dry_run")
    assert artifact["artifact_format"] == "bioxp-oem-parity-v1"
    assert artifact["program"] == "initialize_motors"
    assert artifact["mode"] == "dry_run"
    assert artifact["opened_usb"] is False
    assert artifact["physical_motion"] is False
    assert artifact["raw_truth"]["axis_speeds"] is None
    assert artifact["g_current_invariant"]["required"] is True
    assert artifact["g_current_invariant"]["classification"] == "not_applicable_in_dry_run"
    assert validate_artifact(artifact)["ok"] is True
    json.dumps(artifact)
