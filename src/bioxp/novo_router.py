from __future__ import annotations

import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any, Callable


PIPETTE_FUNCTIONS = (0, 1, 3, 4, 6)
PIPETTE_RX_IDS = frozenset(
    (0x100 | function | (channel << 3)) | 0x400
    for channel in range(4)
    for function in PIPETTE_FUNCTIONS
)
UNPROVEN_ASYNC_IDS = frozenset((0x482, 0x48A, 0x492, 0x49A))
PRESSURE_STREAM_IDS = frozenset(0x505 | (channel << 3) for channel in range(4))


@dataclass(frozen=True)
class NovoFrame:
    arbitration_id: int
    dlc: int
    data: bytes
    raw: bytes
    received_at: float
    classification: str

    def message(self) -> Any:
        return SimpleNamespace(
            arbitration_id=self.arbitration_id,
            dlc=self.dlc,
            data=list(self.data),
            is_extended_id=False,
            timestamp=self.received_at,
            classification=self.classification,
            raw=list(self.raw),
        )

    def provenance(self) -> dict[str, Any]:
        return {
            "classification": self.classification,
            "arbitration_id": self.arbitration_id,
            "dlc": self.dlc,
            "data": list(self.data),
            "raw": list(self.raw),
            "received_at": self.received_at,
        }


@dataclass(frozen=True)
class MatchResult:
    matched: bool
    terminal: bool = False
    classification: str = "not_matched"
    outcome: str | None = None


@dataclass
class _PendingTransaction:
    transaction_id: str
    matcher_name: str
    matcher: Callable[[NovoFrame], MatchResult]
    registered_at: float
    owner_generation: int
    event: threading.Event = field(default_factory=threading.Event)
    frames: list[NovoFrame] = field(default_factory=list)
    skipped: deque[dict[str, Any]] = field(default_factory=lambda: deque(maxlen=64))
    skipped_total: int = 0
    outcome: str | None = None
    received_at: float | None = None


@dataclass
class _PipetteCompletion:
    registered_at: float
    deadline_at: float
    owner_generation: int
    event: threading.Event = field(default_factory=threading.Event)
    frame: NovoFrame | None = None


class NovoRouterError(RuntimeError):
    pass


class NovoRouter:
    """The sole production consumer of the Novo USB IN endpoint."""

    def __init__(
        self,
        *,
        ep_in: Any,
        ep_out: Any,
        decode: Callable[[bytes], bytes],
        read_size: int = 256,
        read_timeout_ms: int = 100,
        queue_size: int = 256,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if ep_in is None or ep_out is None:
            raise NovoRouterError("Novo router requires connected IN and OUT endpoints")
        self.ep_in = ep_in
        self.ep_out = ep_out
        self._decode = decode
        self.read_size = int(read_size)
        self.read_timeout_ms = int(read_timeout_ms)
        self._clock = clock
        self.transaction_lock = threading.Lock()
        self._pending_lock = threading.Lock()
        self._pending: _PendingTransaction | None = None
        self._completion_lock = threading.Lock()
        self._pipette_completions: dict[int, _PipetteCompletion] = {}
        self._stop = threading.Event()
        self._reader: threading.Thread | None = None
        self._queues = {
            name: deque(maxlen=int(queue_size))
            for name in (
                "tmcl", "pipette", "pipette_multipart", "pressure",
                "valid_async", "malformed", "stale", "unknown_async", "unknown",
            )
        }
        self._diagnostics: deque[dict[str, Any]] = deque(maxlen=int(queue_size))
        self._pressure_epoch = 0
        self._pressure_epoch_started_at: float | None = None
        self._reader_generation = 0

    @property
    def running(self) -> bool:
        return bool(self._reader is not None and self._reader.is_alive())

    def start(self) -> None:
        if self.running:
            return
        if self._reader is not None:
            raise NovoRouterError("Novo reader cannot be started twice")
        self._stop.clear()
        self._reader_generation += 1
        self._reader = threading.Thread(
            target=self._receive_loop,
            name=f"bioxp-novo-router-{self._reader_generation}",
            daemon=True,
        )
        self._reader.start()

    def shutdown(self, *, join_timeout_s: float = 2.0) -> None:
        self._stop.set()
        reader = self._reader
        if reader is not None and reader is not threading.current_thread():
            reader.join(timeout=max(0.0, float(join_timeout_s)))
            if reader.is_alive():
                raise NovoRouterError("Novo reader did not stop before USB release")
        self._reader = None
        # Any waiter or completion registered under the previous reader owner is
        # stale after shutdown/rebind, even if a late USB frame arrives.
        self._reader_generation += 1
        with self._pending_lock:
            pending = self._pending
            self._pending = None
        if pending is not None:
            pending.outcome = "shutdown"
            pending.event.set()
        with self._completion_lock:
            completions = list(self._pipette_completions.values())
            self._pipette_completions.clear()
        for completion in completions:
            completion.event.set()

    @staticmethod
    def _decode_record(decoded: bytes, raw: bytes, received_at: float) -> NovoFrame:
        if len(decoded) < 5:
            raise NovoRouterError("decoded Novo record is shorter than module-id plus DLC")
        arbitration_id = int.from_bytes(decoded[0:4], "big")
        dlc = int(decoded[4])
        if len(decoded) != 5 + dlc:
            raise NovoRouterError(
                f"decoded Novo DLC mismatch: dlc={dlc}, available={max(0, len(decoded) - 5)}"
            )
        data = bytes(decoded[5:])
        if arbitration_id in PRESSURE_STREAM_IDS:
            classification = "pressure"
        elif arbitration_id in PIPETTE_RX_IDS:
            function = arbitration_id & 0x7
            classification = "pipette_multipart" if function in (3, 4) else "pipette"
        elif arbitration_id in UNPROVEN_ASYNC_IDS:
            classification = "unknown_async"
        elif arbitration_id == 0 and dlc == 8:
            classification = "valid_async" if data[1] in (128, 129, 130, 132) else "tmcl"
        elif 0x500 <= arbitration_id <= 0x5FF:
            classification = "unknown"
        elif dlc == 0:
            classification = "valid_async"
        else:
            classification = "unknown"
        return NovoFrame(arbitration_id, dlc, data, raw, received_at, classification)

    def _receive_loop(self) -> None:
        while not self._stop.is_set():
            try:
                raw = bytes(self.ep_in.read(self.read_size, timeout=self.read_timeout_ms))
            except Exception as exc:
                if self._stop.is_set():
                    break
                if exc.__class__.__name__ == "USBTimeoutError" or isinstance(exc, TimeoutError):
                    continue
                self._diagnostics.append({"classification": "reader_error", "error": repr(exc), "at": self._clock()})
                continue
            received_at = self._clock()
            try:
                frame = self._decode_record(self._decode(raw), raw, received_at)
            except Exception as exc:
                row = {"classification": "malformed", "raw": list(raw), "error": str(exc), "received_at": received_at}
                self._queues["malformed"].append(row)
                self._diagnostics.append(row)
                self._record_skipped(row)
                continue
            self._dispatch(frame)

    def _record_skipped(self, row: dict[str, Any]) -> None:
        with self._pending_lock:
            pending = self._pending
            if pending is not None:
                pending.skipped_total += 1
                pending.skipped.append(dict(row))

    def _dispatch(self, frame: NovoFrame) -> None:
        completion_matched = False
        if (
            frame.arbitration_id in PIPETTE_RX_IDS
            and (frame.arbitration_id & 0x7) == 1
            and frame.dlc > 0
        ):
            channel = (frame.arbitration_id & 0x78) >> 3
            with self._completion_lock:
                completion = self._pipette_completions.get(channel)
                if (
                    completion is not None
                    and completion.owner_generation == self._reader_generation
                    and self._clock() <= completion.deadline_at
                ):
                    completion.frame = frame
                    completion.event.set()
                    completion_matched = True
        matched = False
        with self._pending_lock:
            pending = self._pending
            if pending is not None:
                decision = pending.matcher(frame)
                if decision.matched:
                    matched = True
                    if len(pending.frames) >= 64:
                        pending.outcome = "malformed"
                        pending.received_at = frame.received_at
                        pending.event.set()
                        return
                    pending.frames.append(frame)
                    if decision.terminal:
                        pending.outcome = decision.outcome or "completion"
                        pending.received_at = frame.received_at
                        pending.event.set()
                else:
                    pending.skipped_total += 1
                    pending.skipped.append(frame.provenance())
        if matched:
            return
        if completion_matched:
            return
        queue_name = frame.classification
        if pending is None and queue_name in {"pipette", "pipette_multipart"}:
            queue_name = "stale"
        if queue_name not in self._queues:
            queue_name = "unknown"
        self._queues[queue_name].append(frame)

    def transact_many(
        self,
        raw_txs: list[bytes] | tuple[bytes, ...],
        *,
        matcher: Callable[[NovoFrame], MatchResult] | None,
        matcher_name: str,
        timeout_s: float,
        write_timeout_ms: int,
        provenance: dict[str, Any],
    ) -> dict[str, Any]:
        frames = [bytes(frame) for frame in raw_txs]
        if not frames:
            raise NovoRouterError("Novo multi-frame transaction requires at least one frame")
        return self.transact(
            b"".join(frames),
            matcher=matcher,
            matcher_name=matcher_name,
            timeout_s=timeout_s,
            write_timeout_ms=write_timeout_ms,
            provenance={
                **dict(provenance),
                "tx_frame_count": len(frames),
                "tx_frames": [list(frame) for frame in frames],
                "tx_write_policy": "one_bulk_transfer_registered_once",
            },
        )

    def transact(
        self,
        raw_tx: bytes,
        *,
        matcher: Callable[[NovoFrame], MatchResult] | None,
        matcher_name: str,
        timeout_s: float,
        write_timeout_ms: int,
        provenance: dict[str, Any],
    ) -> dict[str, Any]:
        if not self.running:
            raise NovoRouterError("Novo router is not running")
        with self.transaction_lock:
            transaction_id = uuid.uuid4().hex
            registered_at = self._clock()
            pending = None
            if matcher is not None:
                pending = _PendingTransaction(
                    transaction_id,
                    matcher_name,
                    matcher,
                    registered_at,
                    self._reader_generation,
                )
                with self._pending_lock:
                    if self._pending is not None:
                        raise NovoRouterError("a Novo transaction matcher is already registered")
                    self._pending = pending
            tx_at = self._clock()
            try:
                self.ep_out.write(raw_tx, timeout=int(write_timeout_ms))
            except Exception:
                with self._pending_lock:
                    if self._pending is pending:
                        self._pending = None
                raise
            base = {
                "transaction_id": transaction_id,
                "owner_generation": self._reader_generation,
                "matcher": matcher_name,
                "registration_timestamp": registered_at,
                "tx_timestamp": tx_at,
                "timeout_ms": int(round(float(timeout_s) * 1000.0)),
                "tx_raw": list(raw_tx),
                **provenance,
            }
            if pending is None:
                return {**base, "ok": True, "outcome": "tx_only", "receive_timestamp": None, "frames": [], "skipped_frames": []}
            completed = pending.event.wait(max(0.0, float(timeout_s)))
            with self._pending_lock:
                if self._pending is pending:
                    self._pending = None
            malformed_seen = any(row.get("classification") == "malformed" for row in pending.skipped)
            generation_changed = self._reader_generation != pending.owner_generation
            outcome = (
                "transport_rebound"
                if generation_changed
                else (pending.outcome if completed else ("malformed" if malformed_seen else "timeout"))
            )
            observed = pending.frames[-1] if pending.frames else None
            multipart_frames = [frame for frame in pending.frames if frame.classification == "pipette_multipart"]
            multipart_projection = {
                "present": bool(multipart_frames),
                "part_count": len(multipart_frames),
                "parts": [frame.provenance() for frame in multipart_frames],
                "reassembled_data": [byte for frame in multipart_frames for byte in frame.data],
                "reassembly_policy": "arrival_order_same_channel_transaction",
                "oem_chunk_index_equivalence": "unresolved",
            }
            terminal_success = bool(
                completed and outcome not in {"ack", "malformed", "shutdown", "transport_rebound"}
            )
            return {
                **base,
                "ok": terminal_success,
                "outcome": outcome,
                "receive_timestamp": pending.received_at,
                "observed_rx_id": observed.arbitration_id if observed is not None else None,
                "observed_rx_dlc": observed.dlc if observed is not None else None,
                "observed_rx_raw": list(observed.raw) if observed is not None else None,
                "frames": [frame.provenance() for frame in pending.frames],
                "ack_received": any(frame.dlc == 0 for frame in pending.frames),
                "completion_received": bool(terminal_success and outcome != "ack"),
                "multipart_received": bool(multipart_frames),
                "multipart": multipart_projection,
                "wait_policy": {
                    "mode": "single_shared_deadline",
                    "classification": "SAFETY-HARDENING",
                    "apartment_equivalence": "unresolved",
                },
                "skipped_count": pending.skipped_total,
                "skipped_frames": list(pending.skipped),
                "skipped_frames_truncated": pending.skipped_total > len(pending.skipped),
            }

    def queue_snapshot(self, name: str) -> list[Any]:
        if name not in self._queues:
            raise KeyError(name)
        return list(self._queues[name])

    def queue_clear(self, name: str) -> int:
        if name not in self._queues:
            raise KeyError(name)
        queue = self._queues[name]
        cleared = len(queue)
        queue.clear()
        return cleared

    def calculate_pressure_offsets(self) -> dict[int, float]:
        samples: dict[int, list[float]] = {channel: [] for channel in range(4)}
        for frame in self._queues["pressure"]:
            if self._pressure_epoch_started_at is not None and frame.received_at < self._pressure_epoch_started_at:
                continue
            channel = (frame.arbitration_id & 0x78) >> 3
            data = frame.data
            if not data or not (ord("0") < data[0] <= ord("3")):
                continue
            count = data[0] - ord("0")
            if len(data) != count * 2 + 1:
                continue
            values = [int.from_bytes(data[1 + index * 2:3 + index * 2], "big", signed=True) for index in range(count)]
            if values:
                samples[channel].append(sum(values) / len(values))
        return {
            channel: (sum(values) / len(values) if values else 0.0)
            for channel, values in samples.items()
        }

    def begin_pressure_epoch(self) -> dict[str, Any]:
        """Start the OEM stream-on pressure-offset sample window."""
        self._pressure_epoch += 1
        self._pressure_epoch_started_at = self._clock()
        self._queues["pressure"].clear()
        return {
            "epoch": self._pressure_epoch,
            "started_at": self._pressure_epoch_started_at,
        }

    def prepare_pipette_completion(self, channel: int, timeout_s: float) -> None:
        channel = int(channel)
        if channel not in range(4):
            raise ValueError(f"invalid pipette channel: {channel}")
        with self._completion_lock:
            existing = self._pipette_completions.get(channel)
            if existing is not None and not existing.event.is_set():
                raise NovoRouterError(f"pipette {channel} completion is already registered")
            registered_at = self._clock()
            self._pipette_completions[channel] = _PipetteCompletion(
                registered_at=registered_at,
                deadline_at=registered_at + max(0.0, float(timeout_s)),
                owner_generation=self._reader_generation,
            )

    def wait_pipette_completion(self, channel: int, timeout_s: float) -> dict[str, Any]:
        channel = int(channel)
        with self._completion_lock:
            completion = self._pipette_completions.get(channel)
        if completion is None:
            return {"ok": False, "channel": channel, "outcome": "completion_not_registered"}
        remaining_contract = max(0.0, completion.deadline_at - self._clock())
        signaled = completion.event.wait(min(max(0.0, float(timeout_s)), remaining_contract))
        with self._completion_lock:
            self._pipette_completions.pop(channel, None)
        frame = completion.frame
        generation_changed = completion.owner_generation != self._reader_generation
        valid = bool(
            signaled
            and not generation_changed
            and frame is not None
            and frame.data
            and frame.data[0] == 0x20
        )
        return {
            "ok": valid,
            "channel": channel,
            "owner_generation": completion.owner_generation,
            "generation_changed": generation_changed,
            "registration_timestamp": completion.registered_at,
            "deadline_timestamp": completion.deadline_at,
            "receive_timestamp": frame.received_at if frame is not None else None,
            "outcome": "completion" if valid else ("malformed" if frame is not None else "timeout"),
            "observed_rx_id": frame.arbitration_id if frame is not None else None,
            "observed_rx_dlc": frame.dlc if frame is not None else None,
            "observed_rx_raw": list(frame.raw) if frame is not None else None,
            "data": list(frame.data) if frame is not None else None,
        }

    @staticmethod
    def tmcl_matcher(
        *,
        board_id: int,
        command: int,
        strict: bool = True,
        require_command_echo: bool = True,
    ) -> Callable[[NovoFrame], MatchResult]:
        def match(frame: NovoFrame) -> MatchResult:
            if frame.classification != "tmcl" or len(frame.data) != 8:
                return MatchResult(False, classification=frame.classification)
            if strict and frame.data[0] != int(board_id):
                return MatchResult(False, classification="tmcl_wrong_board_or_command")
            if strict and require_command_echo and frame.data[2] != int(command):
                return MatchResult(False, classification="tmcl_wrong_board_or_command")
            return MatchResult(True, terminal=True, classification="tmcl", outcome="completion")
        return match

    @staticmethod
    def pipette_matcher(
        *,
        channel: int,
        expected_function: int,
        initialization: bool = False,
        allow_multipart: bool = False,
    ) -> Callable[[NovoFrame], MatchResult]:
        channel = int(channel)
        expected_function = int(expected_function)
        if channel not in range(4):
            raise ValueError(f"invalid pipette channel: {channel}")
        if expected_function not in PIPETTE_FUNCTIONS:
            raise ValueError(f"invalid pipette response function: {expected_function}")

        def match(frame: NovoFrame) -> MatchResult:
            if frame.arbitration_id not in PIPETTE_RX_IDS:
                return MatchResult(False, classification=frame.classification)
            observed_channel = (frame.arbitration_id & 0x78) >> 3
            function = frame.arbitration_id & 0x7
            if observed_channel != channel:
                return MatchResult(False, classification="pipette_wrong_channel")
            if function in (3, 4):
                if not allow_multipart:
                    return MatchResult(False, classification="pipette_unexpected_multipart")
                return MatchResult(True, terminal=False, classification="pipette_multipart", outcome="multipart")
            if function != expected_function:
                return MatchResult(False, classification="pipette_wrong_function")
            if initialization and frame.dlc == 0:
                return MatchResult(True, terminal=True, classification="pipette_immediate_ack", outcome="ack")
            if initialization:
                return MatchResult(False, classification="pipette_delayed_completion")
            outcome = "completion" if not initialization else "delayed_completion"
            return MatchResult(True, terminal=True, classification="pipette", outcome=outcome)

        return match
