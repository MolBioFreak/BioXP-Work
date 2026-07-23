"""Canonical, query-only BioXP hardware observation state.

GET handlers consume only copies returned by :class:`HardwareStateOwner`.  The
collector is deliberately generic: production binds query-only domain readers
to it, while ownership and atomic publication remain centralized here.
"""
from __future__ import annotations

import copy
import threading
import time
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Callable, Iterable, Mapping

from .lifecycle_state import lifecycle_state


CANONICAL_DOMAINS = (
    "transport",
    "boards",
    "axes",
    "range",
    "power",
    "interlock",
    "latch",
    "gripper",
    "thermal",
    "chiller",
    "pipette",
    "camera",
    "shadow_readback",
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


@dataclass(frozen=True)
class CollectionContext:
    ownership_epoch: int
    started_at: str
    allow_recover: bool = False


class HardwareStateOwner:
    """Thread-safe owner of one completed canonical hardware snapshot."""

    def __init__(self, *, fresh_for_s: float = 30.0) -> None:
        self._lock = threading.RLock()
        self._collection_lock = threading.Lock()
        self._epoch = 0
        self._snapshot: dict[str, Any] | None = None
        self._fresh_for_s = max(0.0, float(fresh_for_s))
        self._ownership = {
            "transport": "unbound",
            "usb": "unbound",
            "router": "unbound",
            # No transport observation has occurred yet.  Unknown is not false.
            "CAN_READY": None,
        }
        self._invalidated_at = _utc_now()
        self._invalidation_reason = "not_collected"

    @property
    def ownership_epoch(self) -> int:
        with self._lock:
            return self._epoch

    def ownership_projection(self) -> dict[str, Any]:
        with self._lock:
            return {
                "ownership_epoch": self._epoch,
                "ownership": copy.deepcopy(self._ownership),
                "invalidated_at": self._invalidated_at,
                "invalidation_reason": self._invalidation_reason,
            }

    def change_ownership(
        self,
        *,
        reason: str,
        transport: str | None = None,
        usb: str | None = None,
        router: str | None = None,
    ) -> int:
        """Advance the epoch and immediately invalidate hardware/camera facts."""
        with self._lock:
            if transport is not None:
                self._ownership["transport"] = str(transport)
            if usb is not None:
                self._ownership["usb"] = str(usb)
            if router is not None:
                self._ownership["router"] = str(router)
                if str(router) == "running":
                    # A running reader/router proves software ownership only.
                    # CAN readiness remains unknown until explicit query evidence.
                    self._ownership["CAN_READY"] = None
                elif str(router) == "stopped":
                    self._ownership["CAN_READY"] = False
                elif str(router) == "unbound":
                    self._ownership["CAN_READY"] = None
            self._epoch += 1
            self._snapshot = None
            self._invalidated_at = _utc_now()
            self._invalidation_reason = str(reason)
            lifecycle_state.transport_changed(self._ownership["CAN_READY"], reason=str(reason))
            return self._epoch

    def invalidate(self, *, reason: str) -> None:
        """Invalidate the completed projection without changing transport ownership."""
        with self._lock:
            self._snapshot = None
            self._invalidated_at = _utc_now()
            self._invalidation_reason = str(reason)

    def completed_snapshot(self) -> dict[str, Any] | None:
        with self._lock:
            return copy.deepcopy(self._snapshot)

    def _cache_state(self, snapshot: dict[str, Any] | None, domains: Iterable[str]) -> tuple[str, float | None]:
        if snapshot is None or snapshot.get("ownership_epoch") != self._epoch:
            return "missing", None
        now = time.time()
        ages: list[float] = []
        observations = snapshot.get("domains") or {}
        for domain in domains:
            row = observations.get(domain)
            observed_unix = row.get("observed_unix") if isinstance(row, dict) else None
            if not isinstance(observed_unix, (int, float)):
                return "missing", None
            ages.append(max(0.0, now - float(observed_unix)))
        age = max(ages) if ages else max(0.0, now - float(snapshot.get("completed_unix", now)))
        return ("fresh" if age <= self._fresh_for_s else "stale"), age

    def project(self, *domains: str) -> dict[str, Any]:
        requested = tuple(dict.fromkeys(str(item) for item in domains))
        with self._lock:
            snapshot = copy.deepcopy(self._snapshot)
            cache_state, age_s = self._cache_state(snapshot, requested)
            base = {
                "snapshot_id": None if snapshot is None else snapshot.get("snapshot_id"),
                "ownership_epoch": self._epoch,
                "cache_state": cache_state,
                "freshness": {
                    "state": cache_state,
                    "age_s": None if age_s is None else round(age_s, 3),
                    "fresh_for_s": self._fresh_for_s,
                },
                "requested_domains": list(requested),
                "ownership": copy.deepcopy(self._ownership),
                "provenance": "POST /hardware/snapshot/collect",
                "lifecycle": lifecycle_state.projection(),
            }
            if cache_state == "missing" or snapshot is None:
                return {
                    **base,
                    "available": False,
                    "domains": {
                        domain: {
                            "status": "unknown",
                            "observation": None,
                            "error": "canonical_snapshot_missing_or_invalidated",
                            "observed_at": None,
                            "provenance": None,
                            "freshness": {"state": "missing", "age_s": None, "fresh_for_s": self._fresh_for_s},
                        }
                        for domain in requested
                    },
                    "invalidation_reason": self._invalidation_reason,
                    "invalidated_at": self._invalidated_at,
                }
            rows = snapshot.get("domains") or {}
            projected_rows: dict[str, Any] = {}
            now = time.time()
            for domain in requested:
                row = copy.deepcopy(rows.get(domain))
                if isinstance(row, dict):
                    observed_unix = row.get("observed_unix")
                    domain_age = None if not isinstance(observed_unix, (int, float)) else max(0.0, now - float(observed_unix))
                    domain_state = "missing" if domain_age is None else ("fresh" if domain_age <= self._fresh_for_s else "stale")
                    row["freshness"] = {"state": domain_state, "age_s": None if domain_age is None else round(domain_age, 3), "fresh_for_s": self._fresh_for_s}
                projected_rows[domain] = row
            return {
                **base,
                "available": True,
                "completed_at": snapshot.get("completed_at"),
                "domains": projected_rows,
            }

    def collect(
        self,
        domains: Iterable[str],
        collectors: Mapping[str, Callable[[CollectionContext], Any]],
    ) -> dict[str, Any]:
        """Serialize query-only collection and atomically publish on completion."""
        requested = tuple(dict.fromkeys(str(item) for item in domains))
        unknown = [item for item in requested if item not in collectors]
        if unknown:
            raise ValueError(f"unsupported hardware snapshot domains: {', '.join(unknown)}")
        with self._collection_lock:
            with self._lock:
                epoch = self._epoch
                ownership = copy.deepcopy(self._ownership)
            started_at = _utc_now()
            context = CollectionContext(ownership_epoch=epoch, started_at=started_at, allow_recover=False)
            rows: dict[str, Any] = {}
            for domain in requested:
                observed_unix = time.time()
                observed_at = _utc_now()
                try:
                    observation = collectors[domain](context)
                except Exception as exc:
                    rows[domain] = {
                        "status": "error",
                        "observation": None,
                        "error": str(exc),
                        "observed_at": observed_at,
                        "observed_unix": observed_unix,
                        "provenance": {
                            "route": "POST /hardware/snapshot/collect",
                            "collector": domain,
                            "allow_recover": False,
                        },
                        "freshness": {"state": "fresh", "age_s": 0.0, "fresh_for_s": self._fresh_for_s},
                    }
                else:
                    rows[domain] = {
                        "status": "observed",
                        "observation": copy.deepcopy(observation),
                        "error": None,
                        "observed_at": observed_at,
                        "observed_unix": observed_unix,
                        "provenance": {
                            "route": "POST /hardware/snapshot/collect",
                            "collector": domain,
                            "allow_recover": False,
                        },
                        "freshness": {"state": "fresh", "age_s": 0.0, "fresh_for_s": self._fresh_for_s},
                    }
            completed_unix = time.time()
            snapshot = {
                "schema_version": "bioxp.hardware_snapshot.v1",
                "snapshot_id": uuid.uuid4().hex,
                "ownership_epoch": epoch,
                "ownership": ownership,
                "started_at": started_at,
                "completed_at": _utc_now(),
                "completed_unix": completed_unix,
                "requested_domains": list(requested),
                "domains": rows,
                "publication": "atomic_completed_snapshot",
            }
            with self._lock:
                if self._epoch != epoch:
                    return {
                        "ok": False,
                        "published": False,
                        "error": "ownership_epoch_changed_during_collection",
                        "collection_epoch": epoch,
                        "ownership_epoch": self._epoch,
                        "snapshot": snapshot,
                    }
                self._snapshot = copy.deepcopy(snapshot)
                self._invalidation_reason = None
                self._invalidated_at = None
            return {"ok": True, "published": True, "snapshot": copy.deepcopy(snapshot)}


hardware_state = HardwareStateOwner()
