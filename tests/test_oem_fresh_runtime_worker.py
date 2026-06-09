
from bioxp.oem_fresh_runtime_worker import OemFreshRuntimeWorker


def test_worker_runs_fresh_dry_run_command_and_serializes_history(tmp_path):
    worker = OemFreshRuntimeWorker(artifact_root=tmp_path)
    result = worker.submit({"command": "fresh_homing_dry_run", "program": "initialize_motors"})
    assert result["ok"] is True
    assert result["program"] == "initialize_motors"
    assert result["opened_usb"] is False
    assert result["physical_motion"] is False
    assert worker.history[-1]["command"] == "fresh_homing_dry_run"


def test_worker_rejects_live_command_until_stepwise_gate_supplies_contract(tmp_path):
    worker = OemFreshRuntimeWorker(artifact_root=tmp_path)
    result = worker.submit({"command": "fresh_homing_live", "program": "initialize_motors"})
    assert result["ok"] is False
    assert result["failed_closed"] is True
    assert "live_execution_not_enabled_in_fresh_worker" in result["blockers"]
