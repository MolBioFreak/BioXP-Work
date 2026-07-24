from src.bioxp.oem_runtime_commands import OEMRuntimeCommandHandlers
from src.bioxp.oem_runtime_store import OEMRuntimeStore
from src.bioxp.oem_runtime_types import OEMRuntimeCommand
from src.bioxp.oem_runtime_worker import OEMRuntimeWorker
from test_oem_initialization_controller import FakeInitTester


class Program:
    hardware = FakeInitTester()


def test_runtime_initialize_system_blocks_legacy_oem_initialization_controller(tmp_path):
    root = tmp_path / "artifact"
    worker = OEMRuntimeWorker(
        store=OEMRuntimeStore(tmp_path / "store"),
        handlers=OEMRuntimeCommandHandlers(startup_program=Program()).handlers(),
    )
    worker.enqueue(OEMRuntimeCommand(
        name="initializeSystem",
        mode="live",
        operator_ack="OEM_INITIALIZATION_RUN",
        artifact_root=str(root),
        params={"run_oem_initialization": True, "run_homing": False},
    ))

    result = worker.run_next_for_tests()

    assert result["ok"] is False
    assert result["result"]["operation_state"] == "error"
    assert result["result"]["handler_outcome"] == "failed_closed"
    assert "legacy_monolithic_initialization_cannot_bypass_canonical_startup_stages" in result["result"]["blockers"]
    assert not (root / "runtime_oem_initialization_controller.json").exists()


def test_runtime_live_oem_initialization_homing_request_is_blocked_before_provider_use(tmp_path):
    def factory():
        raise AssertionError("blocked homing request must not open provider")

    worker = OEMRuntimeWorker(
        store=OEMRuntimeStore(tmp_path / "store"),
        handlers=OEMRuntimeCommandHandlers(startup_program_factory=factory).handlers(),
    )
    worker.enqueue(OEMRuntimeCommand(
        name="initializeSystem",
        mode="live",
        operator_ack="OEM_INITIALIZATION_RUN_WITH_HOMING",
        artifact_root=str(tmp_path / "artifact"),
        params={"run_oem_initialization": True, "run_homing": True},
    ))

    result = worker.run_next_for_tests()

    assert result["ok"] is False
    assert "legacy_monolithic_initialization_cannot_bypass_canonical_startup_stages" in result["result"]["blockers"]
