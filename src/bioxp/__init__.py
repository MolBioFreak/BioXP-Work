from __future__ import annotations

from importlib import import_module
from typing import Any

__all__ = [
    "BioXpTester",
    "BioXpCanDriver",
    "BoardAssy",
    "MotorAxis",
    "process_pipette_message",
    "PIPETTE_DATA_QUERY_LABELS",
    "lifecycle_state",
]


def __getattr__(name: str) -> Any:
    if name == "lifecycle_state":
        return import_module(".lifecycle_state", __name__)
    if name == "BioXpTester":
        return import_module(".usb_driver", __name__).BioXpTester
    if name in {"BioXpCanDriver", "BoardAssy", "MotorAxis", "process_pipette_message", "PIPETTE_DATA_QUERY_LABELS"}:
        try:
            module = import_module(".can_driver", __name__)
        except Exception:  # pragma: no cover - legacy optional dependency surface
            return None
        return getattr(module, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
