
from pathlib import Path
from bioxp.oem_homing_runtime import OemHomingDryRunRuntime


def test_dry_run_runtime_executes_all_programs_without_usb_or_motion(tmp_path):
    runtime = OemHomingDryRunRuntime(artifact_root=tmp_path)
    result = runtime.run("initialize_motors", write_artifact=True)
    assert result["ok"] is True
    assert result["opened_usb"] is False
    assert result["physical_motion"] is False
    assert result["program"] == "initialize_motors"
    assert result["artifact_path"]
    assert Path(result["artifact_path"]).exists()
    assert [step["step_id"] for step in result["steps_planned"]][:4] == [
        "z.axisSearchHome",
        "g.setMaxCurrent.before_clear",
        "g.clear.moveSteps",
        "g.axisSearchHome",
    ]


def test_unknown_program_fails_closed_without_artifact(tmp_path):
    runtime = OemHomingDryRunRuntime(artifact_root=tmp_path)
    result = runtime.run("bogus", write_artifact=True)
    assert result["ok"] is False
    assert result["failed_closed"] is True
    assert result["opened_usb"] is False
    assert result["physical_motion"] is False
    assert "unknown" in result["error"]
