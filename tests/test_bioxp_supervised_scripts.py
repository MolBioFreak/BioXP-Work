from pathlib import Path
import py_compile


REPO_ROOT = Path(__file__).resolve().parents[1]


def test_supervised_home_axis_requires_operator_home_confirmation_or_yes_flag():
    script = (REPO_ROOT / "scripts" / "bioxp_supervised_home_axis.sh").read_text()

    assert "ASSUME_YES=0" in script
    assert "--yes|-y" in script
    assert "Type HOME to proceed" in script
    assert '[[ "$reply" != "HOME" ]]' in script


def test_supervised_relative_move_preserves_controller_only_truth_warning():
    script = (REPO_ROOT / "scripts" / "bioxp_supervised_relative_move.sh").read_text()

    assert "Telemetry is controller-side evidence only" in script
    assert "physical_motion_confirmed" in script
    assert "Operator must be physically watching" in script


def test_oem_reference_challenge_is_live_only_and_records_required_proof():
    script_path = REPO_ROOT / "scripts" / "bioxp_oem_reference_challenge.py"
    script = script_path.read_text()

    py_compile.compile(str(script_path), doraise=True)
    assert "no dry-run/demo mode" in script
    assert "--execute" in script
    assert "CHALLENGE_TARGETS = {\"z\": -70000, \"x\": 50000, \"y\": 50000}" in script
    assert "require_switch_transition=True" in script
    assert "max_search_abs_delta" in script
    assert "MarkAxisReferencedCommand" in script
    assert "sensor_reference_artifacts" in script
