from src.bioxp.lifecycle_state import CanonicalLifecycleOwner
from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers
from src.bioxp.oem_runtime_store import OEMRuntimeStore
from src.bioxp.oem_runtime_types import OEMRuntimeCommand
from src.bioxp.oem_runtime_worker import OEMRuntimeWorker


def _live_command(name: str, tmp_path, *, params=None) -> OEMRuntimeCommand:
    return OEMRuntimeCommand(
        name=name,
        mode="live",
        source="test",
        params=params or {},
        operator_ack="OEM_TEST_ACK",
        artifact_root=str(tmp_path),
    )


def test_live_abortjob_calls_bound_z_abort_provider_with_oem_warning_situation(tmp_path):
    calls = []
    handlers = OEMRuntimeCommandHandlers(
        z_abort_provider=lambda command: calls.append(command.command_id) or {"ok": True, "z_state": "failed_latched"}
    )
    command = _live_command("abortjob", tmp_path, params={"action": "ABORT_JOB"})

    result = handlers.handle_abortjob(command)

    assert result["ok"] is True
    assert result["state"] == "aborting_job"
    assert result["warning_situation"] == "ABORT_JOB"
    assert calls == [command.command_id]


def test_abortjob_rejects_non_oem_warning_situation_without_provider_call(tmp_path):
    calls = []
    handlers = OEMRuntimeCommandHandlers(
        z_abort_provider=lambda command: calls.append(command.command_id) or {"ok": True}
    )

    result = handlers.handle_abortjob(
        _live_command("abortjob", tmp_path, params={"action": "MADE_UP_ABORT_MODE"})
    )

    assert result["ok"] is False
    assert result["blockers"] == ["invalid_oem_warning_situation"]
    assert calls == []


def test_live_z_recovery_never_claims_full_wakefrompause_readiness(tmp_path):
    success = OEMRuntimeCommandHandlers(
        z_resume_provider=lambda command: {"ok": True, "z_state": "referenced_ready"}
    ).handle_wakefrompause(_live_command("wakefrompause", tmp_path))
    ambiguous = OEMRuntimeCommandHandlers(
        z_resume_provider=lambda command: {"ok": True, "z_state": "prepared_unreferenced"}
    ).handle_wakefrompause(_live_command("wakefrompause", tmp_path))

    assert success["ok"] is True
    assert success["ready"] is False
    assert success["state"] == "z_recovered_full_wake_required"
    assert success["source_order"] == ["initialCheck", "z_rehome"]
    assert success["blockers"] == ["full_oem_wakefrompause_not_implemented"]
    assert ambiguous["ok"] is False
    assert ambiguous["ready"] is False


def test_failed_wakefrompause_preserves_paused_lifecycle(tmp_path, monkeypatch):
    from src.bioxp import oem_runtime_worker as worker_module

    owner = CanonicalLifecycleOwner()
    owner.transition("paused", reason="test_pause")
    monkeypatch.setattr(worker_module, "lifecycle_state", owner)
    worker = OEMRuntimeWorker(
        store=OEMRuntimeStore(tmp_path / "runtime"),
        handlers={"wakefrompause": lambda command: {"ok": False, "ready": False, "state": "recovery_required"}},
    )

    result = worker._run_command(OEMRuntimeCommand(name="wakefrompause", mode="dry_run", source="test"))

    assert result["ok"] is False
    assert owner.projection()["operation_state"] == "paused"


def test_successful_wakefrompause_releases_paused_lifecycle(tmp_path, monkeypatch):
    from src.bioxp import oem_runtime_worker as worker_module

    owner = CanonicalLifecycleOwner()
    owner.transition("paused", reason="test_pause")
    monkeypatch.setattr(worker_module, "lifecycle_state", owner)
    worker = OEMRuntimeWorker(
        store=OEMRuntimeStore(tmp_path / "runtime"),
        handlers={"wakefrompause": lambda command: {"ok": True, "ready": True, "state": "ready_for_job"}},
    )

    result = worker._run_command(OEMRuntimeCommand(name="wakefrompause", mode="dry_run", source="test"))

    assert result["ok"] is True
    assert owner.projection()["operation_state"] == "stopped"
