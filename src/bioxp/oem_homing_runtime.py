
"""No-USB/no-motion dry-run executor for fresh OEM homing specs."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .oem_homing_spec import get_program
from .oem_parity_artifacts import build_artifact, validate_artifact


class OemHomingDryRunRuntime:
    def __init__(self, *, artifact_root: str | Path | None = None):
        self.artifact_root = Path(artifact_root) if artifact_root is not None else None

    def run(self, program_name: str, *, write_artifact: bool = False, operator_ack: str | None = None) -> dict[str, Any]:
        try:
            program = get_program(program_name)
        except ValueError as exc:
            return {
                "ok": False,
                "failed_closed": True,
                "error": str(exc),
                "program": program_name,
                "mode": "dry_run",
                "opened_usb": False,
                "physical_motion": False,
            }
        artifact = build_artifact(program, mode="dry_run", operator_ack=operator_ack)
        validation = validate_artifact(artifact)
        if not validation["ok"]:
            artifact["ok"] = False
            artifact["failed_closed"] = True
            artifact["validation"] = validation
        if write_artifact:
            if self.artifact_root is None:
                artifact["ok"] = False
                artifact["failed_closed"] = True
                artifact["error"] = "artifact_root_required"
            else:
                self.artifact_root.mkdir(parents=True, exist_ok=True)
                path = self.artifact_root / f"{program.name}_dry_run.json"
                path.write_text(json.dumps(artifact, indent=2, sort_keys=True))
                artifact["artifact_path"] = str(path)
        return artifact
