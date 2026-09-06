"""Optional caller-owned exchange evidence; never a command/admission authority.

No USB, API, database, or device-global last-result state lives here. Owners may
supply their existing ID and an evidence-only sink. A copied worker context keeps
the handle alive after the request resets its token. Reset does not close or
flush the handle: the actual worker owner must explicitly flush on exit.
"""
from __future__ import annotations

from contextlib import contextmanager
from contextvars import ContextVar
from copy import deepcopy
import logging
import threading
import uuid
from typing import Any, Callable, Iterator


_LOG = logging.getLogger(__name__)


class ExchangeObserver:
    def __init__(self, command_id: str | None, sink: Callable[[dict[str, Any]], None] | None = None):
        self.command_id = command_id
        self.trace_id = uuid.uuid4().hex
        self._sink = sink
        self._lock = threading.Lock()
        self._exchanges: dict[str, dict[str, Any]] = {}
        self._calls: dict[str, int] = {}
        self._errors: list[dict[str, str]] = []

    def append(self, exchange: dict[str, Any]) -> None:
        row = deepcopy(exchange)
        exchange_id = row["exchange_id"]
        transaction_id = row["transaction_id"]
        with self._lock:
            if exchange_id in self._exchanges:
                return
            ordinal = self._calls.setdefault(transaction_id, len(self._calls) + 1)
            row.update(
                command_id=self.command_id,
                trace_id=self.trace_id,
                transport_call_ordinal=ordinal,
                durable_ownership_claimed=False,
            )
            self._exchanges[exchange_id] = row

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return deepcopy({
                "transport_exchanges": list(self._exchanges.values()),
                "transport_retention_errors": self._errors,
            })

    def retention_failure(self, stage: str, error: Exception) -> None:
        with self._lock:
            self._errors.append({"stage": stage, "class": type(error).__name__, "message": str(error)[:512]})

    def flush(self) -> None:
        """Checkpoint a detached snapshot; sink must merge/dedupe, not finalize.

        No buffer is cleared on success or failure. This is not crash-proof
        durability. A disk failure remains visible in the caller-owned handle.
        """
        if self._sink is not None:
            try:
                self._sink(self.snapshot())
            except Exception as exc:
                record_retention_failure("sink", exc, owner=self)


_CURRENT: ContextVar[ExchangeObserver | None] = ContextVar("bioxp_command_exchange_owner", default=None)


@contextmanager
def exchange_scope(
    command_id: str | None = None,
    *,
    sink: Callable[[dict[str, Any]], None] | None = None,
) -> Iterator[ExchangeObserver]:
    """Reuse an existing root; anonymous calls acquire only a local trace ID."""
    existing = _CURRENT.get()
    if existing is not None:
        yield existing
        return
    owner = ExchangeObserver(command_id, sink)
    token = _CURRENT.set(owner)
    try:
        yield owner
    finally:
        _CURRENT.reset(token)


def record_retention_failure(stage: str, error: Exception, *, owner: ExchangeObserver | None = None) -> None:
    # Even a broken observer/logger must not replace a controller exception or
    # affect the source return, continuation, reconnect, or retry policy.
    try:
        selected = owner if owner is not None else _CURRENT.get()
        if selected is not None:
            selected.retention_failure(stage, error)
        _LOG.error("Transport exchange retention failed (%s): %s", stage, type(error).__name__)
    except Exception:
        pass


def publish_exchange(exchange: dict[str, Any]) -> None:
    owner = _CURRENT.get()
    if owner is not None:
        try:
            owner.append(exchange)
        except Exception as exc:
            record_retention_failure("append", exc, owner=owner)
