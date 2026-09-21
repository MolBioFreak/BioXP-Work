"""Write-ahead journal for operator interrupt (stop/abort) requests — RCA F1.

A stop that was pressed is a fact that can never be erased by a dead process.
Every operator stop/abort attempt is therefore appended to this fsync'd,
append-only journal *before* any lock is taken and before any physical
delivery is attempted.  The SQLite projections
(``operator_plane_interrupt_attempts`` / ``..._evidence`` / ``..._history`` /
``interrupt_reconciliation_events``) remain the queryable projections, but the
journal is the authority that survives a crash between admission and delivery.

Record shape (one canonical JSON object per line)::

    {"schema": "bioxp.operator_interrupt_journal_event.v1",
     "sequence": 1, "pid": 4711,
     "interrupt_attempt_id": "...", "action_id": "oem.x.stop",
     "phase": "admitted", "recorded_at": 1789961411.497,
     "recorded_monotonic": 12345.6, ...}

Phases: ``admitted`` -> (``delivery_attempted`` -> ``delivered`` | ``failed``)
with terminal phases ``terminal`` / ``rejected`` / ``materialized``.  A ``terminal``
or ``rejected`` record ends the lineage for that attempt; ``materialized`` marks
that startup reconciliation turned an orphaned journal attempt into a durable
projection row.
"""

from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping

INTERRUPT_JOURNAL_SCHEMA = "bioxp.operator_interrupt_journal_event.v1"
INTERRUPT_JOURNAL_FILENAME = "operator_interrupt_journal.v1.jsonl"

#: Phases that end an attempt's lineage (no further work may be pending for it).
INTERRUPT_JOURNAL_TERMINAL_PHASES = frozenset({"terminal", "rejected", "materialized"})
#: Phases that only the write-ahead admission path may write.
INTERRUPT_JOURNAL_PHASES = frozenset(
    {
        "admitted",
        "delivery_attempted",
        "delivered",
        "failed",
        "reconciliation_pending",
        "terminal",
        "rejected",
        "materialized",
    }
)

_MAX_TEXT = 2000


def _journal_safe(value: Any, *, depth: int = 0) -> Any:
    """Bound and JSON-encode a journal field without raising on odd input."""

    if depth > 6:
        return "<depth>"
    if value is None or type(value) in {bool, int, float}:
        return value
    if isinstance(value, str):
        return value if len(value) <= _MAX_TEXT else value[:_MAX_TEXT] + "...[truncated]"
    if isinstance(value, Mapping):
        return {
            str(key)[:128]: _journal_safe(item, depth=depth + 1)
            for key, item in list(value.items())[:64]
        }
    if isinstance(value, (list, tuple, set, frozenset)):
        return [_journal_safe(item, depth=depth + 1) for item in list(value)[:64]]
    return str(value)[:_MAX_TEXT]


class InterruptJournal:
    """Append-only, fsync'd journal of interrupt admission/delivery facts."""

    def __init__(self, root: str | os.PathLike[str] | None = None, *,
                 filename: str = INTERRUPT_JOURNAL_FILENAME) -> None:
        if root is None:
            from bioxp.operator_receipt_store import runtime_state_root

            root = runtime_state_root(None)
        self.root = Path(root)
        self.path = self.root / str(filename)
        self._lock = threading.Lock()
        self._fd: int | None = None
        self._sequence = 0
        self._writes = 0
        self._last_error: str | None = None

    # -- writing ---------------------------------------------------------
    def _open_locked(self) -> int:
        if self._fd is None:
            self.root.mkdir(parents=True, exist_ok=True)
            created = not self.path.exists()
            self._fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
            if created:
                # Make the file's existence itself durable before the first record.
                try:
                    directory_fd = os.open(str(self.root), os.O_RDONLY)
                    try:
                        os.fsync(directory_fd)
                    finally:
                        os.close(directory_fd)
                except OSError:
                    pass
        return self._fd

    def record(self, *, interrupt_attempt_id: str, action_id: str, phase: str,
               **fields: Any) -> dict[str, Any]:
        """Append one durable event. Returns the written record.

        Raises only on identity errors; storage errors are raised so the caller
        can decide (an interrupt that cannot be recorded must say so, never
        proceed silently).
        """

        attempt_id = str(interrupt_attempt_id or "").strip()
        if not attempt_id:
            raise ValueError("interrupt attempt identity is required")
        phase = str(phase or "").strip()
        if phase not in INTERRUPT_JOURNAL_PHASES:
            raise ValueError(f"interrupt journal phase is invalid: {phase!r}")
        entry: dict[str, Any] = {
            "schema": INTERRUPT_JOURNAL_SCHEMA,
            "interrupt_attempt_id": attempt_id,
            "action_id": str(action_id or ""),
            "phase": phase,
            "pid": os.getpid(),
            "recorded_at": time.time(),
            "recorded_monotonic": time.monotonic(),
        }
        for key, value in fields.items():
            if key in entry:
                continue
            entry[str(key)] = _journal_safe(value)
        with self._lock:
            handle = self._open_locked()
            self._sequence += 1
            entry["sequence"] = self._sequence
            payload = (
                json.dumps(entry, separators=(",", ":"), sort_keys=True, default=str) + "\n"
            ).encode("utf-8")
            try:
                written = 0
                while written < len(payload):
                    written += os.write(handle, payload[written:])
                os.fsync(handle)
                self._writes += 1
            except OSError as exc:  # pragma: no cover - storage failure path
                self._last_error = f"{type(exc).__name__}: {exc}"
                raise
        entry["durable"] = True
        entry["journal_path"] = str(self.path)
        return entry

    def record_rejection(self, *, interrupt_attempt_id: str, action_id: str, reason: str,
                         **fields: Any) -> dict[str, Any]:
        """Record a refused stop press (bounded admission, never silent parking)."""

        return self.record(
            interrupt_attempt_id=interrupt_attempt_id,
            action_id=action_id,
            phase="rejected",
            reason=str(reason),
            admitted=False,
            delivery_attempted=False,
            physical_motion_commanded=False,
            automatic_retry=False,
            **fields,
        )

    # -- reading ---------------------------------------------------------
    def events(self, *, limit: int | None = None) -> list[dict[str, Any]]:
        if not self.path.exists():
            return []
        rows: list[dict[str, Any]] = []
        try:
            with open(self.path, "r", encoding="utf-8") as handle:
                for line in handle:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        row = json.loads(line)
                    except ValueError:
                        continue
                    if isinstance(row, dict):
                        rows.append(row)
        except OSError:
            return []
        if limit is not None and limit >= 0:
            return rows[-int(limit):]
        return rows

    def attempts(self) -> dict[str, dict[str, Any]]:
        """Group journal events by interrupt attempt id."""

        grouped: dict[str, dict[str, Any]] = {}
        for row in self.events():
            attempt_id = str(row.get("interrupt_attempt_id") or "")
            if not attempt_id:
                continue
            state = grouped.setdefault(
                attempt_id,
                {
                    "interrupt_attempt_id": attempt_id,
                    "action_id": str(row.get("action_id") or ""),
                    "phases": [],
                    "records": [],
                    "first_recorded_at": row.get("recorded_at"),
                    "terminal": False,
                },
            )
            state["phases"].append(str(row.get("phase") or ""))
            state["records"].append(row)
            state["action_id"] = str(row.get("action_id") or state["action_id"])
            state["last"] = row
            state["last_recorded_at"] = row.get("recorded_at")
            state["terminal"] = bool(
                set(state["phases"]) & INTERRUPT_JOURNAL_TERMINAL_PHASES
            )
        return grouped

    def unresolved_attempts(self) -> list[dict[str, Any]]:
        """Attempts whose lineage never reached a terminal phase (crash window)."""

        return [state for state in self.attempts().values() if not state["terminal"]]

    def status(self) -> dict[str, Any]:
        attempts = self.attempts()
        return {
            "path": str(self.path),
            "writes": self._writes,
            "events": sum(len(state["records"]) for state in attempts.values()),
            "attempts": len(attempts),
            "unresolved": sum(1 for state in attempts.values() if not state["terminal"]),
            "last_error": self._last_error,
        }

    # -- startup reconciliation -----------------------------------------
    @staticmethod
    def delivery_attempted(state: Mapping[str, Any]) -> bool:
        return "delivery_attempted" in set(state.get("phases") or [])

    def materialize_startup(
        self,
        projector: Callable[[Mapping[str, Any]], Any],
    ) -> dict[str, Any]:
        """Materialize durable projection rows for orphaned attempts.

        ``projector`` receives one attempt state and must make it durable in the
        operator-plane projections (it must not deliver anything).  A projection
        that raises stays unresolved and is reported, never silently dropped.
        """

        materialized: list[str] = []
        failures: dict[str, str] = {}
        for state in self.unresolved_attempts():
            attempt_id = str(state["interrupt_attempt_id"])
            try:
                result = projector(state)
            except Exception as exc:
                failures[attempt_id] = f"{type(exc).__name__}: {exc}"[:500]
                continue
            try:
                self.record(
                    interrupt_attempt_id=attempt_id,
                    action_id=str(state.get("action_id") or ""),
                    phase="materialized",
                    materialization=result if isinstance(result, Mapping) else None,
                    materialized_from_phases=list(state.get("phases") or []),
                )
            except OSError as exc:  # pragma: no cover - storage failure path
                failures[attempt_id] = f"journal:{type(exc).__name__}: {exc}"[:500]
                continue
            materialized.append(attempt_id)
        return {
            "ok": not failures,
            "materialized": materialized,
            "failed": failures,
            "unresolved": [
                state["interrupt_attempt_id"] for state in self.unresolved_attempts()
            ],
        }

    def close(self) -> None:
        with self._lock:
            if self._fd is not None:
                try:
                    os.close(self._fd)
                finally:
                    self._fd = None


def journal_events_for_attempt(
    journal: InterruptJournal | None, interrupt_attempt_id: str | None,
) -> list[dict[str, Any]]:
    if journal is None or not interrupt_attempt_id:
        return []
    return [
        row
        for row in journal.events()
        if str(row.get("interrupt_attempt_id")) == str(interrupt_attempt_id)
    ]


def iter_phases(rows: Iterable[Mapping[str, Any]]) -> list[str]:
    return [str(row.get("phase")) for row in rows]
