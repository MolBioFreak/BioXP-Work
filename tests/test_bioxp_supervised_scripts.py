from pathlib import Path


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


def test_legacy_oem_reference_challenge_is_absent():
    assert not (REPO_ROOT / "scripts" / "bioxp_oem_reference_challenge.py").exists()
