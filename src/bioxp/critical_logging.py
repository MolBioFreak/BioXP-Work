"""Storage boundary for the approved command/outcome + critical-fault log.

Wire bytes, polling traces and sampled telemetry are transient transport data,
not command history. This projection must never be used for controller matching,
command admission, motion scheduling, or to calculate a physical postcondition.
"""
from __future__ import annotations

from collections.abc import Mapping
from typing import Any


_DIAGNOSTIC_FIELDS = frozenset({
    "transport_exchanges", "transport_retention_errors", "exchanges",
    "tx_raw", "rx_raw", "observed_rx_raw", "raw_tx", "raw_rx", "raw",
    "tx_bytes", "rx_bytes", "tx_bytes_json", "rx_bytes_json", "raw_exchange",
    "raw_exchange_json", "frames", "skipped_frames", "multipart", "parts",
    "pressure_samples", "pressure_chunks", "pressure_stream", "samples",
    "trace", "traceback", "debug", "logs", "event_history", "diagnostics",
})


def critical_receipt(value: Any) -> Any:
    """Copy retained command data without raw transport/telemetry attachments.

    Identities, parameters, outcome, error, timestamps and derived semantic truth
    remain intact. Live results and operational state are not mutated.
    """
    if isinstance(value, Mapping):
        wire_record = "arbitration_id" in value or (
            "observed_rx_id" in value and "observed_rx_dlc" in value
        )
        return {
            key: critical_receipt(item)
            for key, item in value.items()
            if key not in _DIAGNOSTIC_FIELDS and not (wire_record and key == "data")
        }
    if isinstance(value, (list, tuple)):
        return [critical_receipt(item) for item in value]
    return value
