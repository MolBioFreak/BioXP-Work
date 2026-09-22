"""Guarded automatic reference re-establishment when 24V power returns.

OEM parity anchor: GenBotApp.BioXPMainWindow.initializeSystem() runs
ControlLib.initializeMotion() at every application start, including after an
unexpected shutdown (saved status 3/4 -> "*** Unexpected shutdown occurred ***"
-> initializeMotion() -> inspectCover()).  Operator directive 2026-09-21:
re-homing must NOT run automatically in general (no service-start trigger), but
when the operator brings 24V power up the machine must re-reference itself.

Design: watch the deck-board 24V sense (OEM query24VSensor) at a bounded
cadence.  A power-down is armed by an explicit ``no24v`` reading, or by a
sustained run of invalid/no-reply samples (the machine is off while the PC and
this service stay up).  A subsequent valid 24V-present sample is a power-up
edge.  On an edge, if the motion references are not all established and the
machine is idle, run exactly one guarded OEM initializeMotors sequence -- the
same path the operator route runs.  One attempt per edge; no retry loops; state
changes are persisted so service restarts never invent edges.
"""

from __future__ import annotations

import asyncio
import json
import os
import time
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

SCHEMA_VERSION = "bioxp.reference_recovery.v1"
DOWN_STREAK_REQUIRED = 3
PENDING_MAX_S = 600.0
TICK_S = 10.0


def _atomic_write_json(path: Path, payload: Mapping[str, Any]) -> None:
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n")
    os.replace(tmp, path)


class ReferenceRecoveryMonitor:
    """Watch the 24V rail; on a power-up edge with invalid references and an
    idle machine, run one guarded reference re-establishment."""

    def __init__(
        self,
        *,
        rail_reader: Callable[[], Mapping[str, Any] | None],
        references_ready: Callable[[], bool | None],
        idle_check: Callable[[], tuple[bool, str | None]],
        rereference: Callable[[], Mapping[str, Any]],
        state_path: Path,
        tick_s: float = TICK_S,
        down_streak_required: int = DOWN_STREAK_REQUIRED,
        pending_max_s: float = PENDING_MAX_S,
        clock: Callable[[], float] = time.time,
    ) -> None:
        self._rail_reader = rail_reader
        self._references_ready = references_ready
        self._idle_check = idle_check
        self._rereference = rereference
        self._state_path = Path(state_path)
        self._tick_s = float(tick_s)
        self._down_streak_required = int(down_streak_required)
        self._pending_max_s = float(pending_max_s)
        self._clock = clock
        self._armed = False
        self._down_streak = 0
        self._last_powered: bool | None = None
        self._last_edge_at: float | None = None
        self._pending_since: float | None = None
        self._last_attempt_at: float | None = None
        self._last_outcome: str | None = None
        self._last_skip_reason: str | None = None
        self._attempts = 0
        self._load()

    # -- persistence ------------------------------------------------

    def _load(self) -> None:
        try:
            payload = json.loads(self._state_path.read_text())
        except Exception:
            return
        if not isinstance(payload, Mapping):
            return
        self._armed = bool(payload.get("armed_down", False))
        self._down_streak = int(payload.get("down_streak") or 0)
        last_powered = payload.get("last_powered")
        self._last_powered = bool(last_powered) if isinstance(last_powered, bool) else None
        self._last_edge_at = payload.get("last_edge_at") if isinstance(payload.get("last_edge_at"), (int, float)) else None
        self._pending_since = payload.get("pending_since") if isinstance(payload.get("pending_since"), (int, float)) else None
        self._last_attempt_at = payload.get("last_attempt_at") if isinstance(payload.get("last_attempt_at"), (int, float)) else None
        self._last_outcome = str(payload["last_outcome"]) if payload.get("last_outcome") is not None else None
        self._attempts = int(payload.get("attempts") or 0)

    def _persist(self) -> None:
        try:
            _atomic_write_json(self._state_path, {
                "schema_version": SCHEMA_VERSION,
                "armed_down": self._armed,
                "down_streak": self._down_streak,
                "last_powered": self._last_powered,
                "last_edge_at": self._last_edge_at,
                "pending_since": self._pending_since,
                "last_attempt_at": self._last_attempt_at,
                "last_outcome": self._last_outcome,
                "attempts": self._attempts,
            })
        except Exception:
            # Power-recovery bookkeeping must never take down the service.
            pass

    # -- observation -------------------------------------------------

    def tick(self) -> None:
        now = self._clock()
        rail: Mapping[str, Any] | None = None
        try:
            candidate = self._rail_reader()
        except Exception:
            candidate = None
        if isinstance(candidate, Mapping):
            rail = candidate

        if rail is None:
            # No reply: the machine may be fully off (the PC stays up).  Only a
            # sustained run counts as power-down; single hiccups stay unknown.
            self._down_streak += 1
            if self._down_streak >= self._down_streak_required:
                if not self._armed:
                    self._armed = True
                self._last_powered = False
        else:
            self._down_streak = 0
            no24v = rail.get("no24v")
            if no24v is True:
                self._armed = True
                self._last_powered = False
            elif no24v is False:
                if self._armed:
                    self._armed = False
                    self._last_edge_at = now
                    self._pending_since = now
                    self._last_outcome = "edge_observed"
                self._last_powered = True
            # no24v None (malformed sample): keep the last known state.

        self._maybe_attempt(now)
        self._persist()

    def _maybe_attempt(self, now: float) -> None:
        if self._pending_since is None:
            return
        try:
            ready = self._references_ready()
        except Exception:
            ready = None
        if ready is True:
            self._pending_since = None
            self._last_skip_reason = None
            if self._last_outcome == "edge_observed":
                self._last_outcome = "references_already_valid"
            return
        if now - self._pending_since > self._pending_max_s:
            self._pending_since = None
            self._last_outcome = "abandoned_machine_busy"
            return
        try:
            idle, why = self._idle_check()
        except Exception as exc:
            idle, why = False, f"idle_probe_failed:{type(exc).__name__}"
        if not idle:
            self._last_skip_reason = why or "machine_busy"
            return
        self._last_skip_reason = None
        self._pending_since = None
        self._last_attempt_at = now
        self._attempts += 1
        try:
            result = self._rereference()
        except Exception as exc:
            self._last_outcome = f"failed:{type(exc).__name__}: {exc}"
            return
        if isinstance(result, Mapping) and result.get("ok") is True:
            self._last_outcome = "completed"
        else:
            failure = None
            if isinstance(result, Mapping):
                failure = result.get("failure") or result.get("state") or result.get("error")
            self._last_outcome = f"failed:{failure or 'unknown'}"

    # -- loop / status ----------------------------------------------

    async def run(self) -> None:
        while True:
            await asyncio.sleep(self._tick_s)
            try:
                await asyncio.to_thread(self.tick)
            except asyncio.CancelledError:
                raise
            except Exception:
                # Keep the watch alive; the next tick re-reads the rail.
                continue

    def status(self) -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "last_rail_powered": self._last_powered,
            "armed_down": self._armed,
            "down_streak": self._down_streak,
            "last_edge_at": self._last_edge_at,
            "pending_since": self._pending_since,
            "last_attempt_at": self._last_attempt_at,
            "last_outcome": self._last_outcome,
            "last_skip_reason": self._last_skip_reason,
            "attempts": self._attempts,
        }
