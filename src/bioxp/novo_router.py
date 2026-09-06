from __future__ import annotations

import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass, field, replace
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
    receive_sequence: int | None = None
    receive_owner: str | None = None
    owner_generation: int | None = None

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
            "receive_sequence": self.receive_sequence,
            "receive_owner": self.receive_owner,
            "owner_generation": self.owner_generation,
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
    owner_token: str
    command_family: int | None
    command_name: str | None
    expected_rx_id: int | None = None
    transaction_id: str | None = None
    tx_started_at: float | None = None
    event: threading.Event = field(default_factory=threading.Event)
    ack_frame: NovoFrame | None = None
    frame: NovoFrame | None = None
    duplicate_terminal_count: int = 0
    rejected_reason: str | None = None


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
        self._interrupted_pipette_completions: dict[tuple[int, str], _PipetteCompletion] = {}
        self._pipette_completion_taints: dict[int, dict[str, Any]] = {}
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
        self._receive_sequence = 0
        self._receive_owner = uuid.uuid4().hex
        self._motor_event_lock = threading.RLock()
        self._motor_signals: dict[tuple[int, int], NovoFrame] = {}
        self._motor_resets: dict[tuple[int, int], tuple[int, int]] = {}
        self._motor_consumed: dict[tuple[int, int], tuple[int, int]] = {}

    def receive_cursor(self) -> dict[str, Any]:
        with self._motor_event_lock:
            return {"after_sequence": self._receive_sequence,
                    "receive_owner": self._receive_owner,
                    "owner_generation": self._reader_generation}

    @staticmethod
    def _motor_key(frame: NovoFrame) -> tuple[int, int] | None:
        if (frame.arbitration_id == 0 and frame.dlc == 8
                and frame.data[1] == 128 and frame.data[6] <= 2):
            return (frame.data[0], frame.data[6])
        return None

    def reset_motor_event(self, board: int, motor: int, *, reset: bool = True, initial_signals=None) -> None:
        """ClassMotor.queryMotorStop: Reset only after a nonnull return."""
        key = (int(board), int(motor))
        with self._motor_event_lock:
            if reset:
                if initial_signals is not None:
                    initial_signals.discard(key)
                self._motor_signals.pop(key, None)
                self._motor_resets[key] = (self._reader_generation, self._receive_sequence)
            elif self._motor_resets.get(key, (None,))[0] != self._reader_generation:
                # A null query does not reset, including in a shared XY window.
                self._motor_resets[key] = (self._reader_generation, -1)

    def take_motor_events(self, targets, event_window=None, *, initial_signals=None) -> list[NovoFrame | tuple[int, int]]:
        """Atomic AutoResetEvent consumption: WaitAll consumes none on timeout.

        This is a host latch, NOT correlation to a device command. An unlabelled
        late wire event can Set the current latch, exactly as in the OEM source.
        """
        keys = {(int(board), int(motor)) for board, motor in targets}
        window = event_window if isinstance(event_window, dict) else {}
        with self._motor_event_lock:
            if (window.get("receive_owner", self._receive_owner) != self._receive_owner
                    or window.get("owner_generation", self._reader_generation) != self._reader_generation):
                return []
            frames = []
            for key in sorted(keys):
                frame = self._motor_signals.get(key)
                if frame is None or frame.owner_generation != self._reader_generation:
                    if initial_signals is not None and key in initial_signals:
                        frames.append(key)  # Source-initialized latch, NOT a wire frame.
                        continue
                    return []
                reset = self._motor_resets.get(key)
                after = (reset[1] if reset and reset[0] == self._reader_generation
                         else window.get("after_sequence"))
                if after is not None and frame.receive_sequence <= after:
                    if initial_signals is not None and key in initial_signals:
                        # Filtering receive proof does not Reset the independent
                        # construction signal. Consume/coalesce both below.
                        frames.append(key)
                        continue
                    return []
                frames.append(frame)
            for key in keys:
                if initial_signals is not None:
                    initial_signals.discard(key)
                self._motor_signals.pop(key, None)
                self._motor_consumed[key] = (self._reader_generation, self._receive_sequence)
            return frames

    def motor_event_disposition(self, frame: NovoFrame) -> str:
        with self._motor_event_lock:
            if frame.receive_owner != self._receive_owner or frame.owner_generation != self._reader_generation:
                return "previous_receive_owner"
            key = self._motor_key(frame)
            if key is None:
                return "not_target_event"
            for marks, reason in ((self._motor_resets, "reset"), (self._motor_consumed, "consumed")):
                mark = marks.get(key)
                if mark and mark[0] == frame.owner_generation and frame.receive_sequence <= mark[1]:
                    return reason
            signal = self._motor_signals.get(key)
            return "set" if signal is frame else "coalesced_set"

    @property
    def running(self) -> bool:
        return bool(self._reader is not None and self._reader.is_alive())

    @property
    def reader_generation(self) -> int:
        return int(self._reader_generation)

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
            completions = [
                *self._pipette_completions.values(),
                *self._interrupted_pipette_completions.values(),
            ]
            self._pipette_completions.clear()
            self._interrupted_pipette_completions.clear()
            self._pipette_completion_taints.clear()
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
        # Host receive identity, never a device command/transaction identifier.
        with self._motor_event_lock:
            if frame.receive_sequence is None:
                self._receive_sequence += 1
                frame = replace(frame, receive_sequence=self._receive_sequence,
                                receive_owner=self._receive_owner,
                                owner_generation=self._reader_generation)
            else:
                # Retained records cannot re-enter as fresh receives. New reads
                # of identical wire bytes get distinct ingress identities.
                self._diagnostics.append({**frame.provenance(), "classification":
                    "duplicate_receive_record" if frame.receive_owner == self._receive_owner
                    and frame.owner_generation == self._reader_generation else "previous_receive_owner"})
                return
            key = self._motor_key(frame)
            if key is not None:
                if key in self._motor_signals and self._motor_signals[key].owner_generation == self._reader_generation:
                    self._diagnostics.append({**frame.provenance(), "classification": "motor_coalesced_set"})
                self._motor_signals[key] = frame
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
                        if self._pending is pending:
                            self._pending = None
                        pending.event.set()
                else:
                    pending.skipped_total += 1
                    pending.skipped.append(frame.provenance())
        if matched:
            if frame.arbitration_id in PIPETTE_RX_IDS and frame.dlc == 0:
                channel = (frame.arbitration_id & 0x78) >> 3
                function = frame.arbitration_id & 0x7
                with self._completion_lock:
                    completion = self._pipette_completions.get(channel)
                    if (
                        completion is not None
                        and completion.owner_generation == self._reader_generation
                        and completion.command_family == function
                        and (
                            completion.expected_rx_id is None
                            or frame.arbitration_id == completion.expected_rx_id
                        )
                    ):
                        completion.ack_frame = frame
            return

        completion_matched = False
        if frame.arbitration_id in PIPETTE_RX_IDS:
            channel = (frame.arbitration_id & 0x78) >> 3
            function = frame.arbitration_id & 0x7
            with self._completion_lock:
                completion = self._pipette_completions.get(channel)
                if (
                    completion is not None
                    and completion.owner_generation == self._reader_generation
                    and completion.command_family == function
                    and (
                        completion.expected_rx_id is None
                        or frame.arbitration_id == completion.expected_rx_id
                    )
                    and completion.transaction_id is not None
                    and completion.tx_started_at is not None
                ):
                    if frame.dlc == 0:
                        completion.ack_frame = frame
                        completion_matched = True
                    elif frame.dlc == 2 and len(frame.data) == 2:
                        if completion.frame is None:
                            completion.frame = frame
                        else:
                            completion.duplicate_terminal_count += 1
                            self._diagnostics.append({
                                **frame.provenance(),
                                "classification": "pipette_duplicate_terminal",
                            })
                        completion.event.set()
                        completion_matched = True
        if completion_matched:
            return
        queue_name = frame.classification
        if pending is None and queue_name in {"pipette", "pipette_multipart"}:
            queue_name = "stale"
        if queue_name not in self._queues:
            queue_name = "unknown"
        self._queues[queue_name].append(frame)

    def _bind_completion_owner_from_provenance(
        self,
        provenance: dict[str, Any],
        *,
        transaction_id: str,
        tx_started_at: float,
    ) -> None:
        owner_token = provenance.get("completion_owner_token")
        channel = provenance.get("channel")
        if owner_token is None:
            return
        if channel is None:
            raise NovoRouterError("completion owner token requires a pipette channel")
        self.bind_pipette_completion(
            int(channel),
            owner_token=str(owner_token),
            transaction_id=str(transaction_id),
            tx_started_at=float(tx_started_at),
        )

    def _clear_oem_pipette_receive_queue(self, provenance: dict[str, Any]) -> int:
        """Mirror the OEM receive-queue clear before a pipette send."""
        if type(provenance.get("channel")) is not int or type(provenance.get("command_family")) is not int:
            return 0
        cleared = 0
        for name in ("pipette", "pipette_multipart", "stale"):
            queue = self._queues[name]
            retained = deque(maxlen=queue.maxlen)
            while queue:
                item = queue.popleft()
                if isinstance(item, NovoFrame) and item.arbitration_id in PIPETTE_RX_IDS:
                    cleared += 1
                else:
                    retained.append(item)
            queue.extend(retained)
        return cleared

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
        if not self.running:
            raise NovoRouterError("Novo router is not running")
        with self.transaction_lock:
            cleared_pipette_replies = self._clear_oem_pipette_receive_queue(provenance)
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
                    self._pending = pending
            tx_at = self._clock()
            self._bind_completion_owner_from_provenance(
                provenance,
                transaction_id=transaction_id,
                tx_started_at=tx_at,
            )
            write_timestamps: list[float] = []
            try:
                # OEM CanInterfaceBoard.WritePacket() is called once for each
                # CAN frame.  Keep one transaction/pending owner, but never
                # concatenate framed USB records into one bulk write.
                for raw_tx in frames:
                    self.ep_out.write(raw_tx, timeout=int(write_timeout_ms))
                    write_timestamps.append(self._clock())
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
                "tx_raw": list(b"".join(frames)),
                "tx_frame_count": len(frames),
                "tx_frames": [list(frame) for frame in frames],
                "tx_write_timestamps": write_timestamps,
                "tx_write_completed_at": write_timestamps[-1] if write_timestamps else tx_at,
                **provenance,
                "tx_write_policy": "one_frame_per_oem_sendcommand",
                "oem_receive_queue_cleared": cleared_pipette_replies,
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
                    "mode": "oem_synchronous_send_wait",
                    "classification": "OEM-DIRECT",
                    "source_anchor": "ClassNovo.TransmitMessage; ClassNovoCANUSB.sendCommand",
                },
                "skipped_count": pending.skipped_total,
                "skipped_frames": list(pending.skipped),
                "skipped_frames_truncated": pending.skipped_total > len(pending.skipped),
            }

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
            cleared_pipette_replies = self._clear_oem_pipette_receive_queue(provenance)
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
                    self._pending = pending
            tx_at = self._clock()
            self._bind_completion_owner_from_provenance(
                provenance,
                transaction_id=transaction_id,
                tx_started_at=tx_at,
            )
            try:
                self.ep_out.write(raw_tx, timeout=int(write_timeout_ms))
                tx_write_completed_at = self._clock()
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
                "tx_write_completed_at": tx_write_completed_at,
                "timeout_ms": int(round(float(timeout_s) * 1000.0)),
                "tx_raw": list(raw_tx),
                **provenance,
                "oem_receive_queue_cleared": cleared_pipette_replies,
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
                    "mode": "oem_synchronous_send_wait",
                    "classification": "OEM-DIRECT",
                    "source_anchor": "ClassNovo.TransmitMessage; ClassNovoCANUSB.sendCommand",
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

    def calculate_pressure_offset_evidence(self) -> dict[int, dict[str, Any]]:
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
            samples[channel].extend(float(value) for value in values)
        return {
            channel: {
                "offset": (sum(values) / len(values) if values else None),
                "sample_count": len(values),
                "samples": list(values),
                "valid": bool(values),
                "units": "controller_pressure_counts",
                "pressure_epoch": self._pressure_epoch,
                "pressure_epoch_started_at": self._pressure_epoch_started_at,
            }
            for channel, values in samples.items()
        }

    def calculate_pressure_offsets(self) -> dict[int, float]:
        return {
            channel: float(row["offset"])
            for channel, row in self.calculate_pressure_offset_evidence().items()
            if row.get("valid") is True and row.get("offset") is not None
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

    def prepare_pipette_completion(
        self,
        channel: int,
        timeout_s: float,
        *,
        command_family: int | None = None,
        command_name: str | None = None,
        expected_rx_id: int | None = None,
        replace_owner_token: str | None = None,
        replacement_reason: str | None = None,
    ) -> str:
        channel = int(channel)
        if channel not in range(4):
            raise ValueError(f"invalid pipette channel: {channel}")
        if command_family is not None and int(command_family) not in PIPETTE_FUNCTIONS:
            raise ValueError(f"invalid pipette command family: {command_family}")
        if expected_rx_id is not None:
            expected_rx_id = int(expected_rx_id)
            expected_channel = (expected_rx_id >> 3) & 0x3
            expected_family = expected_rx_id & 0x7
            if expected_rx_id not in PIPETTE_RX_IDS:
                raise ValueError(f"invalid pipette expected RX ID: 0x{expected_rx_id:03x}")
            if expected_channel != channel or (
                command_family is not None and expected_family != int(command_family)
            ):
                raise ValueError("pipette expected RX ID does not match channel/family")
        with self._completion_lock:
            taint = self._pipette_completion_taints.get(channel)
            if taint is not None:
                raise NovoRouterError(
                    f"pipette {channel} completion lifecycle is tainted; router rebind is required"
                )
            existing = self._pipette_completions.get(channel)
            if existing is not None:
                replacement_authorized = bool(
                    isinstance(replace_owner_token, str)
                    and replace_owner_token
                    and existing.owner_token == replace_owner_token
                    and command_family == 0
                    and existing.command_family == 1
                    and replacement_reason == "interrupted_by_terminate"
                    and not existing.event.is_set()
                )
                if replacement_authorized:
                    existing.rejected_reason = "interrupted_by_terminate"
                    existing.event.set()
                    self._interrupted_pipette_completions[
                        (channel, existing.owner_token)
                    ] = existing
                    self._pipette_completions.pop(channel, None)
                else:
                    raise NovoRouterError(f"pipette {channel} completion is already registered")
            registered_at = self._clock()
            legacy_owner = command_family is None and command_name is None
            owner_token = uuid.uuid4().hex
            self._pipette_completions[channel] = _PipetteCompletion(
                registered_at=registered_at,
                deadline_at=registered_at + max(0.0, float(timeout_s)),
                owner_generation=self._reader_generation,
                owner_token=owner_token,
                command_family=None if command_family is None else int(command_family),
                command_name=None if command_name is None else str(command_name),
                expected_rx_id=expected_rx_id,
                transaction_id="legacy-unbound-owner" if legacy_owner else None,
                tx_started_at=registered_at if legacy_owner else None,
            )
            return owner_token


    def bind_pipette_completion(
        self,
        channel: int,
        *,
        owner_token: str | None = None,
        transaction_id: str,
        tx_started_at: float,
    ) -> None:
        channel = int(channel)
        with self._completion_lock:
            completion = self._pipette_completions.get(channel)
            if completion is None:
                raise NovoRouterError(f"pipette {channel} completion is not registered")
            if owner_token is not None and completion.owner_token != str(owner_token):
                raise NovoRouterError(f"pipette {channel} completion owner mismatch")
            if completion.transaction_id is not None:
                raise NovoRouterError(f"pipette {channel} completion is already bound")
            completion.transaction_id = str(transaction_id)
            completion.tx_started_at = float(tx_started_at)

    def wait_pipette_completion(
        self,
        channel: int,
        timeout_s: float,
        *,
        owner_token: str | None = None,
    ) -> dict[str, Any]:
        channel = int(channel)
        requested_owner = str(owner_token) if owner_token is not None else None
        owner_mismatch = False
        with self._completion_lock:
            completion = self._pipette_completions.get(channel)
            archived = False
            if (
                requested_owner
                and (completion is None or completion.owner_token != requested_owner)
            ):
                interrupted_completion = self._interrupted_pipette_completions.get(
                    (channel, requested_owner)
                )
                if interrupted_completion is not None:
                    completion = interrupted_completion
                    archived = True
                else:
                    owner_mismatch = completion is not None
        if owner_mismatch:
            return {
                "ok": False,
                "channel": channel,
                "outcome": "completion_owner_token_mismatch",
                "requested_owner_token": requested_owner,
                "owner_token": completion.owner_token if completion is not None else None,
            }
        if completion is None:
            return {"ok": False, "channel": channel, "outcome": "completion_not_registered"}

        remaining_contract = max(0.0, completion.deadline_at - self._clock())
        signaled = completion.event.wait(min(max(0.0, float(timeout_s)), remaining_contract))
        with self._completion_lock:
            if archived:
                self._interrupted_pipette_completions.pop(
                    (channel, completion.owner_token),
                    None,
                )
            elif self._pipette_completions.get(channel) is completion:
                self._pipette_completions.pop(channel, None)
        frame = completion.frame
        generation_changed = completion.owner_generation != self._reader_generation
        valid = bool(
            signaled
            and not generation_changed
            and completion.rejected_reason is None
            and completion.transaction_id is not None
            and completion.tx_started_at is not None
            and completion.ack_frame is not None
            and frame is not None
            and frame.received_at >= completion.tx_started_at
            and (
                completion.expected_rx_id is None
                or frame.arbitration_id == completion.expected_rx_id
            )
            and (frame.arbitration_id & 0x7) == completion.command_family
            and frame.dlc == 2
            and len(frame.data) == 2
            and frame.data[0] == 0x20
        )
        if not valid and not generation_changed and completion.rejected_reason is None:
            with self._completion_lock:
                self._pipette_completion_taints[channel] = {
                    "reason": "malformed_completion" if frame is not None else "completion_timeout",
                    "owner_token": completion.owner_token,
                    "transaction_id": completion.transaction_id,
                    "command_name": completion.command_name,
                    "tainted_at": self._clock(),
                }
        return {
            "ok": valid,
            "channel": channel,
            "owner_token": completion.owner_token,
            "command_family": completion.command_family,
            "command_name": completion.command_name,
            "expected_rx_id": completion.expected_rx_id,
            "transaction_id": completion.transaction_id,
            "tx_started_at": completion.tx_started_at,
            "owner_generation": completion.owner_generation,
            "generation_changed": generation_changed,
            "registration_timestamp": completion.registered_at,
            "deadline_timestamp": completion.deadline_at,
            "receive_timestamp": frame.received_at if frame is not None else None,
            "immediate_ack_received": completion.ack_frame is not None,
            "immediate_ack": completion.ack_frame.provenance() if completion.ack_frame is not None else None,
            "duplicate_terminal_count": completion.duplicate_terminal_count,
            "outcome": "completion" if valid else (
                completion.rejected_reason
                or ("malformed" if frame is not None else "timeout")
            ),
            "observed_rx_id": frame.arbitration_id if frame is not None else None,
            "observed_rx_dlc": frame.dlc if frame is not None else None,
            "observed_rx_raw": list(frame.raw) if frame is not None else None,
            "data": list(frame.data) if frame is not None else None,
        }

    def pipette_completion_taint(self, channel: int) -> dict[str, Any] | None:
        """Return passive evidence for a fail-closed completion channel."""
        with self._completion_lock:
            taint = self._pipette_completion_taints.get(int(channel))
            return dict(taint) if taint is not None else None

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
        multipart_parts = 0

        def match(frame: NovoFrame) -> MatchResult:
            nonlocal multipart_parts
            if frame.arbitration_id not in PIPETTE_RX_IDS:
                return MatchResult(False, classification=frame.classification)
            observed_channel = (frame.arbitration_id & 0x78) >> 3
            function = frame.arbitration_id & 0x7
            if observed_channel != channel:
                return MatchResult(False, classification="pipette_wrong_channel")
            if function in (3, 4):
                if not allow_multipart:
                    return MatchResult(False, classification="pipette_unexpected_multipart")
                if function == 3:
                    if multipart_parts != 0:
                        return MatchResult(False, classification="pipette_multipart_wrong_order")
                    multipart_parts = 1
                elif multipart_parts == 0:
                    return MatchResult(False, classification="pipette_multipart_wrong_order")
                else:
                    multipart_parts += 1
                return MatchResult(True, terminal=False, classification="pipette_multipart", outcome="multipart")
            if function != expected_function:
                return MatchResult(False, classification="pipette_wrong_function")
            if expected_function == 6 and frame.dlc <= 0:
                return MatchResult(False, classification="pipette_empty_report")
            if initialization:
                return MatchResult(
                    True,
                    terminal=True,
                    classification="pipette_immediate_ack" if frame.dlc == 0 else "pipette",
                    outcome="ack" if frame.dlc == 0 else "completion",
                )
            return MatchResult(True, terminal=True, classification="pipette", outcome="completion")

        return match
