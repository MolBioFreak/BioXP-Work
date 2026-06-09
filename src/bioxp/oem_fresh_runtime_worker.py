
"""Fresh OEM parity worker integration.

This adapter lets the OEM runtime lane dispatch fresh scaffold dry-runs while
continuing to fail closed for live execution.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

from .oem_homing_runtime import OemHomingDryRunRuntime


class OemFreshRuntimeWorker:
    def __init__(self, *, artifact_root: str | Path | None = None):
        self.artifact_root = Path(artifact_root) if artifact_root is not None else None
        self.history: list[dict[str, Any]] = []

    def submit(self, command: dict[str, Any]) -> dict[str, Any]:
        name = command.get("command")
        program = command.get("program") or "initialize_motors"
        record = {"command": name, "program": program}
        self.history.append(record)
        if name == "fresh_homing_dry_run":
            runtime = OemHomingDryRunRuntime(artifact_root=self.artifact_root)
            result = runtime.run(program, write_artifact=self.artifact_root is not None, operator_ack=command.get("operator_ack"))
            result["worker"] = "fresh_oem_parity"
            return result
        if name == "fresh_homing_live":
            return {
                "ok": False,
                "failed_closed": True,
                "worker": "fresh_oem_parity",
                "program": program,
                "opened_usb": False,
                "physical_motion": False,
                "blockers": ["live_execution_not_enabled_in_fresh_worker", "requires_stepwise_live_contract"],
            }
        return {
            "ok": False,
            "failed_closed": True,
            "worker": "fresh_oem_parity",
            "program": program,
            "opened_usb": False,
            "physical_motion": False,
            "blockers": ["unknown_fresh_worker_command"],
        }
