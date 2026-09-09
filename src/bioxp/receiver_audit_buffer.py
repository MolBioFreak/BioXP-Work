"""Bounded host evidence handoff, never a controller command queue.

Only the writer thread opens the governed store. In-flight records count against
both bounds. A successful offer is VOLATILE, not a durable receipt. Defaults are
provisional engineering limits, not approved throughput/capacity guarantees.
"""
from __future__ import annotations

from collections import deque
import json
import os
import threading
import uuid
from typing import Any

from .runtime_audit_store import RuntimeAuditDatabase


_OWNER_LOCK = threading.Lock()
_OWNER_BUFFER = None


def current_receiver_audit_buffer():
    """Read the retained process writer; never construct or recover one.

    Callers release this short registry lock before status reads or drain waits.
    This also covers a USB owner already released before application shutdown.
    """
    with _OWNER_LOCK:
        return _OWNER_BUFFER


class ReceiverAuditBuffer:
    SOURCE = "novo_receiver_audit"

    def __init__(self, *, root=None, max_records=4096, max_bytes=4 * 1024 * 1024,
                 database_factory=RuntimeAuditDatabase):
        if type(max_records) is not int or type(max_bytes) is not int or min(max_records, max_bytes) < 1:
            raise ValueError("receiver audit bounds must be positive integers")
        self.max_records, self.max_bytes = max_records, max_bytes
        self.session = uuid.uuid4().hex  # host-only; not device correlation
        self._root, self._factory = root, database_factory
        self._condition = threading.Condition()
        self._queue = deque()
        self._records = self._bytes = 0
        self._inflight_records = self._inflight_bytes = 0
        self._offered = self._accepted = self._committed = 0
        self._overflow = self._failed = self._rejected = 0
        self._gap_committed = (0, 0, 0)
        self._closing = self._finished = self._clean = False
        self._closed_committed = False
        self._error = None
        self._opened = False
        self._thread = threading.Thread(target=self._run, name="bioxp-receiver-audit", daemon=True)
        self._thread.start()

    @classmethod
    def from_environment(cls):
        # A USB owner may be discarded/reconstructed by API release/reconnect.
        # Retain the one process writer even across replacement driver objects;
        # otherwise each reconnect could strand another blocked SQLite thread.
        global _OWNER_BUFFER
        with _OWNER_LOCK:
            if _OWNER_BUFFER is None or not _OWNER_BUFFER.alive:
                _OWNER_BUFFER = cls(
                    max_records=int(os.environ.get("BIOXP_RECEIVER_AUDIT_MAX_RECORDS", "4096")),
                    max_bytes=int(os.environ.get("BIOXP_RECEIVER_AUDIT_MAX_BYTES", "4194304")))
            return _OWNER_BUFFER

    @property
    def alive(self):
        return self._thread.is_alive()

    def offer(self, kind: str, payload: dict[str, Any]) -> bool:
        # Private producers supply bounded primitive fields, not arbitrary objects.
        encoded = json.dumps(payload, separators=(",", ":"), ensure_ascii=True).encode("ascii")
        size = len(encoded) + len(kind.encode("utf-8"))
        with self._condition:
            self._offered += 1
            if self._closing or self._finished:
                self._rejected += 1
                return False
            if self._records >= self.max_records or self._bytes + size > self.max_bytes:
                self._overflow += 1  # drop newest evidence only, never backpressure USB
                self._condition.notify()
                return False
            self._accepted += 1
            self._records += 1
            self._bytes += size
            self._queue.append((kind, encoded, size, self._offered))
            self._condition.notify()
            return True

    def status(self):
        with self._condition:
            return {"session": self.session, "max_records": self.max_records,
                    "max_bytes": self.max_bytes, "offered": self._offered,
                    "accepted_volatile": self._accepted, "buffered_records": self._records,
                    "buffered_bytes": self._bytes, "committed_records": self._committed,
                    "queued_records": self._records - self._inflight_records,
                    "inflight_records": self._inflight_records, "inflight_bytes": self._inflight_bytes,
                    "overflow_records": self._overflow, "write_failed_records": self._failed,
                    "rejected_records": self._rejected, "committed_gap_counts": self._gap_committed,
                    "writer_open_committed": self._opened, "writer_error": self._error,
                    "closing": self._closing, "finished": self._finished,
                    "close_accounting_committed": self._closed_committed,
                    "clean_drain_committed": self._clean and self._gap_committed ==
                        (self._overflow, self._failed, self._rejected),
                    "healthy": self._opened and not self._error and not self._overflow
                               and not self._rejected and not self._closing,
                    "durable_ownership_claimed": False}

    def close(self, timeout_s=0.0):
        """Stop offers; optionally wait OUTSIDE receiver/USB cleanup. Idempotent.

        A timeout does not cancel a SQLite commit or claim a clean drain. The
        retained owner remains queryable until the writer exits.
        """
        with self._condition:
            self._closing = True
            self._condition.notify()
        self._thread.join(max(0.0, float(timeout_s)))
        return self.status()

    def _event(self, db, kind, payload, sequence=None):
        # Dedicated writer connection must own the transaction: record_event's
        # nested-transaction return must never be mistaken for a commit.
        if db.connection.in_transaction:
            raise RuntimeError("receiver writer has an unexpected outer transaction")
        result = db.record_event(command_id=None, pipette_operation_id=None,
            event_source=self.SOURCE, event_kind=kind,
            event_payload={"audit_session": self.session, **payload},
            source_sequence=sequence, reader_generation=payload.get("owner_generation"),
            semantic_validity="host_observation")
        if db.connection.in_transaction:
            raise RuntimeError("receiver event returned without outer commit")
        return result

    def _gaps(self, db):
        with self._condition:
            counts = (self._overflow, self._failed, self._rejected)
        if counts != self._gap_committed:
            self._event(db, "gap", {"overflow_records": counts[0],
                "write_failed_or_commit_uncertain_records": counts[1], "rejected_records": counts[2],
                "counts_scope": "this_process_observed_only", "crash_lost_count": None})
            with self._condition:
                self._gap_committed = counts

    def _run(self):
        db = None
        try:
            db = self._factory(self._root)
            # Existing append-only runtime_events is the lifecycle/gap authority;
            # no auxiliary journal, migration, schema or retention change.
            previous = db.connection.execute(
                "SELECT event_kind,event_json FROM runtime_events WHERE event_source=? "
                "AND event_kind IN ('opened','closed') ORDER BY event_id DESC LIMIT 1",
                (self.SOURCE,)).fetchone()
            self._event(db, "opened", {"previous_lifecycle": previous[0] if previous else None,
                "previous_session": json.loads(previous[1]).get("audit_session") if previous else None,
                "unclean_previous_session": previous[0] != "closed" if previous else None,
                "crash_lost_count": None,
                "pre_open_ingress_loss_unknown": True})
            with self._condition:
                self._opened = True
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._queue or self._closing
                        or (self._overflow, self._failed, self._rejected) != self._gap_committed)
                    batch = []
                    batch_bytes = 0
                    while self._queue and len(batch) < 128:
                        size = self._queue[0][2]
                        if batch and batch_bytes + size > 256 * 1024:
                            break
                        batch.append(self._queue.popleft())
                        batch_bytes += size
                    self._inflight_records = len(batch)
                    self._inflight_bytes = batch_bytes
                    closing = self._closing
                if batch:
                    try:
                        events = []
                        for kind, encoded, size, sequence in batch:
                            payload = json.loads(encoded)
                            events.append(dict(command_id=None, pipette_operation_id=None,
                                event_source=self.SOURCE, event_kind=kind,
                                event_payload={"audit_session": self.session, **payload},
                                source_sequence=sequence, reader_generation=payload.get("owner_generation"),
                                semantic_validity="host_observation"))
                        db.record_event_batch(events)
                    except Exception as exc:
                        with self._condition:
                            self._failed += len(batch)
                            self._error = repr(exc)[:512]
                    else:
                        with self._condition:
                            self._committed += len(batch)
                    finally:
                        with self._condition:
                            self._records -= len(batch)
                            self._bytes -= batch_bytes
                            self._inflight_records = self._inflight_bytes = 0
                # Gap failure terminates this writer rather than hot-looping or
                # silently retrying ambiguous commits. No controller retry exists.
                self._gaps(db)
                with self._condition:
                    drained = not self._queue and self._records == 0
                if closing and drained:
                    with self._condition:
                        close_payload = {"accepted_volatile": self._accepted,
                            "committed_records": self._committed,
                            "all_accepted_committed": self._failed == 0,
                            "gap_counts": (self._overflow, self._failed, self._rejected),
                            "offered_through": self._offered,
                            "counts_scope": "offers_observed_at_close_snapshot",
                            "later_rejected_offers_unknown": True,
                            "lossless": not (self._overflow or self._failed or self._rejected)}
                    self._event(db, "closed", close_payload)
                    with self._condition:
                        self._gap_committed = close_payload["gap_counts"]
                        self._closed_committed = True
                        self._clean = self._failed == 0
                    break
        except Exception as exc:
            with self._condition:
                # Terminal failure must close admission before releasing the lock:
                # db.close() may block, but no writer will drain another offer.
                self._closing = True
                self._error = repr(exc)[:512]
                self._failed += self._records
                self._queue.clear()
                self._records = self._bytes = 0
                self._inflight_records = self._inflight_bytes = 0
        finally:
            if db is not None:
                try:
                    db.close()
                except Exception as exc:
                    with self._condition:
                        self._error = repr(exc)[:512]
            with self._condition:
                self._finished = True
                self._condition.notify_all()
