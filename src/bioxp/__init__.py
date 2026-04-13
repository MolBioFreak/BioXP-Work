from __future__ import annotations

from importlib import import_module
from typing import Any

__all__ = [
    "BioXpTester",
    "BioXpCanDriver",
    "BoardAssy",
    "MotorAxis",
]


def __getattr__(name: str) -> Any:
    if name == "BioXpTester":
        return import_module(".usb_driver", __name__).BioXpTester
    if name in {"BioXpCanDriver", "BoardAssy", "MotorAxis"}:
        try:
            module = import_module(".can_driver", __name__)
        except Exception:  # pragma: no cover - legacy optional dependency surface
            return None
        return getattr(module, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
