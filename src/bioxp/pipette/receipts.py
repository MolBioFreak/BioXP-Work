from __future__ import annotations

import hashlib
import json
import os
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

from ..hardware_status import hardware_state
from ..oem_full_lifecycle import current_authority_identity, current_registry_sha256
from ..runtime_audit_store import RuntimeAuditDatabase, runtime_state_root
from .audit import PipetteAuditIntegrityError, normalize_pipette_result


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
        self._audit_database = RuntimeAuditDatabase(root=root)
        self.connection = self._audit_database.connection
        self.root = self._audit_database.root
        self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
        os.chmod(self.root, 0o700)
        self.path = self._audit_database.path
        self.receipts_path = None
        self._legacy_path = self.root / "receipts.jsonl"
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
        if not completion_explicit:
            completion = False

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
        command_id: str | None = None,
        pipette_operation_id: str | None = None,
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
        if command_id is None or pipette_operation_id is None:
            with self._lock:
                fd = os.open(self._legacy_path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o600)
                try:
                    os.write(fd, encoded)
                    os.fsync(fd)
                finally:
                    os.close(fd)
                os.chmod(self._legacy_path, 0o600)
        if command_id is not None and pipette_operation_id is not None:
            try:
                self._persist_normalized_result(
                    command_id=str(command_id),
                    pipette_operation_id=str(pipette_operation_id),
                    result=result,
                )
                self._audit_database.finalize_claim(
                    command_id=str(command_id),
                    pipette_operation_id=str(pipette_operation_id),
                    status="completed" if receipt["truth"]["completion_verified"] else "acknowledged",
                    outcome=str(result.get("outcome") or ("completed" if result.get("ok") is True else "failed")),
                    failure_code=None if result.get("ok") is True else str(result.get("error") or result.get("code") or "pipette_operation_failed"),
                    result=result,
                    effective_inputs=effective_inputs,
                    receipt_json=json.dumps(receipt, sort_keys=True),
                )
            except PipetteAuditIntegrityError as exc:
                raise PipetteReceiptError(str(exc)) from exc
            except Exception as exc:
                raise PipetteReceiptError(f"typed pipette audit persistence failed: {exc}") from exc
        return receipt

    def _persist_normalized_result(
        self,
        *,
        command_id: str,
        pipette_operation_id: str,
        result: Mapping[str, Any],
    ) -> dict[str, list[str]]:
        normalized = normalize_pipette_result(result)
        observation_ids: list[str] = []
        exchange_ids: list[str] = []
        event_ids: list[str] = []
        for row in normalized["channels"]:
            observation_ids.append(
                self.record_channel_observation(
                    command_id=command_id,
                    pipette_operation_id=pipette_operation_id,
                    **row,
                )
            )
        for row in normalized["exchanges"]:
            exchange_ids.append(
                self.record_transport_exchange(
                    command_id=command_id,
                    pipette_operation_id=pipette_operation_id,
                    **row,
                )
            )
        for row in normalized["events"]:
            event_ids.append(
                self.record_event(
                    command_id=command_id,
                    pipette_operation_id=pipette_operation_id,
                    **row,
                )
            )
        pressure_samples = list(normalized.get("pressure_samples") or [])
        pressure_stream_ids: list[str] = []
        pressure_chunk_ids: list[str] = []
        if pressure_samples:
            channels = sorted({int(sample.get("channel", result.get("channel", 0))) for sample in pressure_samples})
            stream_id = self.record_pressure_stream(
                command_id=command_id,
                pipette_operation_id=pipette_operation_id,
                channels=channels,
                sample_period_ms=result.get("sample_period_ms"),
                source_generation=result.get("source_generation", result.get("reader_generation", 0)),
                reader_generation=result.get("reader_generation"),
                offset_identity=result.get("offset_identity"),
            )
            pressure_stream_ids.append(stream_id)
            for channel in channels:
                chunk_samples = []
                for sample in pressure_samples:
                    if int(sample.get("channel", result.get("channel", 0))) != channel:
                        continue
                    chunk_samples.append(
                        {
                            **dict(sample),
                            "raw_pressure": sample.get("raw_pressure", sample.get("value")),
                            "corrected_pressure": sample.get("corrected_pressure", sample.get("value")),
                            "controller_timestamp": sample.get("controller_timestamp", sample.get("controller_time")),
                        }
                    )
                pressure_chunk_ids.append(
                    self.record_pressure_chunk(
                        stream_session_id=stream_id,
                        channel=channel,
                        chunk_sequence=0,
                        samples=chunk_samples,
                        units=str(result.get("pressure_units") or "unknown"),
                        offset_identity=result.get("offset_identity"),
                        chunk_schema="bioxp.pipette.pressure.chunk.v1",
                    )
                )
        return {
            "observation_ids": observation_ids,
            "exchange_ids": exchange_ids,
            "event_ids": event_ids,
            "pressure_stream_ids": pressure_stream_ids,
            "pressure_chunk_ids": pressure_chunk_ids,
        }

    def record_failure(
        self,
        *,
        command_id: str,
        pipette_operation_id: str,
        operation: str,
        failure_code: str,
        message: str,
        status: str = "failed",
        requested_inputs: Mapping[str, Any] | None = None,
        runtime_binding: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        result = {
            "ok": False,
            "outcome": "rejected" if status == "rejected" else "failed",
            "error": str(message),
            "failure_code": str(failure_code),
            "runtime_binding": dict(runtime_binding or {}),
        }
        try:
            self._audit_database.finalize_claim(
                command_id=str(command_id),
                pipette_operation_id=str(pipette_operation_id),
                status=str(status),
                outcome=str(result["outcome"]),
                failure_code=str(failure_code),
                result=result,
                effective_inputs={},
                receipt_json=json.dumps(result, sort_keys=True),
            )
            self._persist_normalized_result(
                command_id=str(command_id),
                pipette_operation_id=str(pipette_operation_id),
                result=result,
            )
        except Exception as exc:
            raise PipetteReceiptError(f"pipette failure persistence failed: {exc}") from exc
        return result

    def record_channel_observation(self, **kwargs: Any) -> str:
        return self._audit_database.record_channel_observation(**kwargs)

    def record_transport_exchange(self, **kwargs: Any) -> str:
        try:
            return self._audit_database.record_transport_exchange(**kwargs)
        except ValueError as exc:
            raise PipetteAuditIntegrityError(str(exc)) from exc

    def record_event(self, **kwargs: Any) -> str:
        return self._audit_database.record_event(**kwargs)

    def record_pressure_stream(self, **kwargs: Any) -> str:
        return self._audit_database.record_pressure_stream(**kwargs)

    def record_pressure_chunk(self, **kwargs: Any) -> str:
        return self._audit_database.record_pressure_chunk(**kwargs)

    def claim(
        self,
        *,
        operation: str,
        requested_inputs: Mapping[str, Any] | None,
        entrypoint_id: str,
        caller_class: str,
        control_class: str,
        idempotency_key: str,
        command_id: str | None = None,
        ownership_generation: int = 0,
        runtime_binding: Mapping[str, Any] | None = None,
    ) -> tuple[dict[str, Any], bool]:
        command = str(command_id or f"pipette_{uuid.uuid4().hex}")
        payload = {
            "command_id": command,
            "idempotency_key": str(idempotency_key),
            "action_id": f"pipette.{operation}",
            "operation": str(operation),
            "entrypoint_id": str(entrypoint_id),
            "caller_class": str(caller_class),
            "control_class": str(control_class),
            "ownership_generation": int(ownership_generation),
            "source_identity": dict(runtime_binding or {"authority": "robot_runtime"}),
            "requested_inputs": dict(requested_inputs or {}),
        }
        return self._audit_database.claim(payload, pipette=True)

    def migrate_legacy_jsonl(self) -> dict[str, Any]:
        if not self._legacy_path.exists():
            return {"status": "no_source", "source_path": str(self._legacy_path), "imported_count": 0}
        raw = self._legacy_path.read_bytes()
        source_sha256 = hashlib.sha256(raw).hexdigest()
        migration_id = f"pipette-jsonl:{source_sha256}"
        existing = self.connection.execute(
            "SELECT status,imported_count FROM runtime_migration_receipts WHERE migration_id=?",
            (migration_id,),
        ).fetchone()
        if existing is not None:
            return {
                "status": "already_imported",
                "source_path": str(self._legacy_path),
                "source_sha256": source_sha256,
                "imported_count": 0,
            }
        imported = 0
        quarantined = 0
        source_count = 0
        for line_number, line in enumerate(raw.splitlines(), start=1):
            if not line.strip():
                continue
            source_count += 1
            try:
                legacy = json.loads(line.decode("utf-8"))
                if not isinstance(legacy, Mapping):
                    raise ValueError("legacy receipt is not an object")
                receipt_id = str(legacy.get("receipt_id") or f"line-{line_number}")
                operation = str(legacy.get("operation") or "legacy_pipette")
                command_id = f"legacy.pipette.{receipt_id}"
                claim, created = self.claim(
                    operation=operation,
                    requested_inputs=legacy.get("requested_inputs") if isinstance(legacy.get("requested_inputs"), Mapping) else {},
                    entrypoint_id="migration.pipette_jsonl.v1",
                    caller_class="legacy_migration",
                    control_class="historical_import",
                    idempotency_key=f"legacy-pipette:{receipt_id}",
                    command_id=command_id,
                    ownership_generation=0,
                    runtime_binding={"legacy_receipt_id": receipt_id, "source_sha256": source_sha256},
                )
                if not created:
                    continue
                result = legacy.get("result") if isinstance(legacy.get("result"), Mapping) else {"ok": False, "outcome": "legacy_result_missing"}
                self.record(
                    operation=operation,
                    requested_inputs=legacy.get("requested_inputs") if isinstance(legacy.get("requested_inputs"), Mapping) else {},
                    effective_inputs=legacy.get("effective_inputs") if isinstance(legacy.get("effective_inputs"), Mapping) else {},
                    result={**dict(result), "legacy_receipt_id": receipt_id, "legacy_source_sha256": source_sha256},
                    runtime_binding={"legacy_receipt_id": receipt_id, "source_sha256": source_sha256},
                    command_id=claim["command_id"],
                    pipette_operation_id=claim["pipette_operation_id"],
                )
                imported += 1
            except Exception:
                quarantined += 1
        status = "completed_with_quarantine" if quarantined else "completed"
        self.connection.execute("BEGIN IMMEDIATE")
        try:
            self.connection.execute(
                """
                INSERT INTO runtime_migration_receipts(
                    migration_id,source_kind,source_digest,source_count,imported_count,quarantined_count,status,created_at
                ) VALUES(?,?,?,?,?,?,?,?)
                """,
                (
                    migration_id,
                    "pipette_receipts_jsonl",
                    source_sha256,
                    source_count,
                    imported,
                    quarantined,
                    status,
                    time.time(),
                ),
            )
            self.connection.execute("COMMIT")
        except Exception:
            if self.connection.in_transaction:
                self.connection.execute("ROLLBACK")
            raise
        return {
            "status": status,
            "source_path": str(self._legacy_path),
            "source_sha256": source_sha256,
            "source_count": source_count,
            "imported_count": imported,
            "quarantined_count": quarantined,
        }

    def read(self, limit: int = 50) -> list[dict[str, Any]]:
        selected_limit = max(1, min(int(limit), 200))
        rows = self.connection.execute(
            "SELECT receipt_json FROM pipette_operations WHERE receipt_json IS NOT NULL AND receipt_json <> '{}' ORDER BY updated_at DESC LIMIT ?",
            (selected_limit,),
        ).fetchall()
        if rows:
            return [json.loads(row["receipt_json"]) for row in rows]
        if not self._legacy_path.exists():
            return []
        legacy_rows = [json.loads(line) for line in self._legacy_path.read_text(encoding="utf-8").splitlines() if line.strip()]
        return legacy_rows[-selected_limit:]

    def latest(self) -> dict[str, Any] | None:
        rows = self.read(1)
        return rows[0] if rows else None
