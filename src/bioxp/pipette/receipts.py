from __future__ import annotations

import hashlib
import json
import os
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

from ..hardware_status import hardware_state
from ..oem_full_lifecycle import current_authority_identity, current_registry_sha256


class PipetteReceiptError(RuntimeError):
    """A pipette operation cannot be durably represented."""


_SECRET_KEYS = frozenset(
    {
        "password",
        "passwd",
        "secret",
        "token",
        "api_key",
        "apikey",
        "access_token",
        "refresh_token",
        "connection_string",
    }
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _redact(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {
            str(key): "[REDACTED]" if str(key).lower() in _SECRET_KEYS else _redact(item)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [_redact(item) for item in value]
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class PipetteReceiptStore:
    """Durable private journal for source-bound pipette operation receipts."""

    def __init__(self, root: str | Path | None = None) -> None:
        self.root = Path(root or os.environ.get("BIOXP_PIPETTE_RECEIPT_ROOT") or "/tmp/bioxp-oem-runtime/pipette")
        self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
        os.chmod(self.root, 0o700)
        self.path = self.root / "receipts.jsonl"
        self._lock = threading.RLock()

    def _source_identity(self) -> dict[str, Any]:
        repo = Path(__file__).resolve().parents[3]
        tracked = {
            "pipette_models": repo / "src/bioxp/pipette/models.py",
            "pipette_transport": repo / "src/bioxp/pipette/transport.py",
            "pipette_receipts": repo / "src/bioxp/pipette/receipts.py",
            "can_driver": repo / "src/bioxp/can_driver.py",
            "novo_router": repo / "src/bioxp/novo_router.py",
            "novo_usb_can": repo / "src/bioxp/novo_usb_can.py",
            "pipette_service": repo / "src/bioxp/services/pipette_service.py",
            "pipette_spec": repo / "docs/specs/2026-08-02-pipette-oem-gap-rectification-spec.md",
        }
        missing = [name for name, path in tracked.items() if not path.is_file()]
        if missing:
            raise PipetteReceiptError(f"pipette source identity is incomplete: {missing}")
        try:
            authority = current_authority_identity()
            registry_sha256 = current_registry_sha256()
        except Exception as exc:  # pragma: no cover - authority failure is environment-specific
            raise PipetteReceiptError(f"serial-206 evidence authority unavailable: {exc}") from exc
        return {
            "repository_root": str(repo),
            "source_sha256": {name: _sha256(path) for name, path in tracked.items()},
            "registry_sha256": registry_sha256,
            "evidence_authority": authority,
            "authority_verified": bool(authority.get("evidence_lock_identity_verified")),
        }

    @staticmethod
    def _truth(result: Mapping[str, Any]) -> dict[str, Any]:
        driver_result = result.get("driver_result")
        driver = driver_result if isinstance(driver_result, Mapping) else {}
        provenance_result = driver.get("provenance")
        provenance = provenance_result if isinstance(provenance_result, Mapping) else {}

        delivery = result.get("delivery_verified")
        if not isinstance(delivery, bool):
            driver_delivery = driver.get("delivery_verified")
            delivery = driver_delivery if isinstance(driver_delivery, bool) else bool(driver.get("tx_ok", False))

        controller = result.get("controller_acknowledged")
        if not isinstance(controller, bool):
            driver_controller = driver.get("controller_acknowledged")
            controller = driver_controller if isinstance(driver_controller, bool) else False

        completion = result.get("completion_verified")
        completion_explicit = isinstance(completion, bool)
        if not completion_explicit:
            driver_completion = driver.get("completion_verified")
            if isinstance(driver_completion, bool):
                completion = driver_completion
                completion_explicit = True
            else:
                completion = bool(provenance.get("completion_received", False))
        if not completion_explicit and not completion:
            completion = bool(
                result.get("ok") is True
                and result.get("outcome") in {"completion", "initialized"}
            )

        precondition = result.get("hardware_precondition_verified")
        if not isinstance(precondition, bool):
            precondition = False
        hardware = result.get("hardware_postcondition_verified")
        if not isinstance(hardware, bool):
            hardware = False
        # No caller in the authorized no-motion envelope can certify physical effect.
        return {
            "delivery_verified": bool(delivery),
            "controller_acknowledged": bool(controller),
            "completion_verified": bool(completion),
            "hardware_precondition_verified": bool(precondition),
            "hardware_postcondition_verified": bool(hardware),
            "physical_effect_verified": False,
            "physical_effect_claim_suppressed": True,
        }

    def record(
        self,
        *,
        operation: str,
        requested_inputs: Mapping[str, Any] | None,
        result: Mapping[str, Any],
        effective_inputs: Mapping[str, Any] | None = None,
        runtime_binding: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        if not isinstance(result, Mapping):
            raise PipetteReceiptError("receipt result must be a mapping")
        source_identity = self._source_identity()
        receipt = {
            "schema": "bioxp.pipette.receipt.v1",
            "receipt_id": uuid.uuid4().hex,
            "created_at": _utc_now(),
            "operation": str(operation),
            "requested_inputs": _redact(dict(requested_inputs or {})),
            "effective_inputs": _redact(dict(effective_inputs or {})),
            "result": _redact(dict(result)),
            "truth": self._truth(result),
            "runtime_binding": _redact(dict(runtime_binding or {"owner": "pipette_receipt_store"})),
            "ownership_epoch": int(getattr(hardware_state, "ownership_epoch", 0)),
            "source_identity": source_identity,
            "deployment_identity": {
                "deployed_sha": None,
                "deployed_sha_verified": False,
                "runtime_sha_verified": False,
                "status": "source_bound_not_deployed_proof",
            },
        }
        encoded = (json.dumps(receipt, sort_keys=True) + "\n").encode("utf-8")
        with self._lock:
            fd = os.open(self.path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o600)
            try:
                os.write(fd, encoded)
                os.fsync(fd)
            finally:
                os.close(fd)
            os.chmod(self.path, 0o600)
        return receipt

    def read(self, limit: int = 50) -> list[dict[str, Any]]:
        if not self.path.exists():
            return []
        rows = [json.loads(line) for line in self.path.read_text(encoding="utf-8").splitlines() if line.strip()]
        return rows[-max(1, int(limit)):]

    def latest(self) -> dict[str, Any] | None:
        rows = self.read(1)
        return rows[0] if rows else None
