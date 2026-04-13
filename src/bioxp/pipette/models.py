from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any


_MAX_VOLUME_UL = 1000.0


class PipetteTipAction(str, Enum):
    LOAD = "load"
    EJECT = "eject"


class PipetteError(Exception):
    def __init__(
        self,
        message: str,
        *,
        code: str = "pipette_error",
        status_code: int = 400,
        details: dict[str, Any] | None = None,
    ) -> None:
        super().__init__(message)
        self.message = str(message)
        self.code = str(code)
        self.status_code = int(status_code)
        self.details = dict(details or {})

    def to_payload(self) -> dict[str, Any]:
        return {
            "ok": False,
            "error": self.code,
            "message": self.message,
            "details": dict(self.details),
        }


class PipetteTransportUnavailableError(PipetteError):
    def __init__(self, message: str = "Pipette transport is unavailable.", *, details: dict[str, Any] | None = None) -> None:
        super().__init__(message, code="transport_unavailable", status_code=503, details=details)


class PipetteNotReadyError(PipetteError):
    def __init__(self, message: str = "Pipette must be initialized before this operation.", *, details: dict[str, Any] | None = None) -> None:
        super().__init__(message, code="not_initialized", status_code=409, details=details)


class PipetteTipStateError(PipetteError):
    def __init__(self, message: str = "Tip state does not allow this operation.", *, details: dict[str, Any] | None = None) -> None:
        super().__init__(message, code="tip_state", status_code=409, details=details)


class PipetteCommandError(PipetteError):
    def __init__(self, message: str = "Pipette command failed.", *, details: dict[str, Any] | None = None) -> None:
        super().__init__(message, code="command_failed", status_code=502, details=details)


class PipetteValidationError(PipetteError):
    def __init__(self, message: str, *, details: dict[str, Any] | None = None) -> None:
        super().__init__(message, code="validation_error", status_code=400, details=details)


def normalize_pressure_profile(value: Any) -> str:
    profile = str(value or "1R").strip().upper()
    if not profile:
        raise PipetteValidationError("pressure_profile must not be empty")
    if len(profile) > 2 or not profile.isalnum():
        raise PipetteValidationError("pressure_profile must be 1-2 alphanumeric characters")
    return profile


def validate_volume_ul(value: Any) -> float:
    volume_ul = float(value)
    if volume_ul <= 0.0:
        raise PipetteValidationError("volume_ul must be greater than 0")
    if volume_ul > _MAX_VOLUME_UL:
        raise PipetteValidationError(f"volume_ul must be <= {_MAX_VOLUME_UL:g}")
    return volume_ul


@dataclass(frozen=True)
class PipetteInitCommand:
    pressure_profile: str = "1R"
    prime_volume_ul: float | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "pressure_profile", normalize_pressure_profile(self.pressure_profile))
        if self.prime_volume_ul is not None:
            object.__setattr__(self, "prime_volume_ul", validate_volume_ul(self.prime_volume_ul))

    @classmethod
    def from_request(cls, req: Any) -> "PipetteInitCommand":
        return cls(
            pressure_profile=getattr(req, "pressure_profile", "1R"),
            prime_volume_ul=getattr(req, "prime_volume_ul", None),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "pressure_profile": self.pressure_profile,
            "prime_volume_ul": self.prime_volume_ul,
        }


@dataclass(frozen=True)
class PipetteTipCommand:
    action: PipetteTipAction

    def __post_init__(self) -> None:
        if not isinstance(self.action, PipetteTipAction):
            object.__setattr__(self, "action", PipetteTipAction(str(self.action)))

    @classmethod
    def from_request(cls, req: Any) -> "PipetteTipCommand":
        return cls(action=getattr(req, "action"))

    def to_payload(self) -> dict[str, Any]:
        return {"action": self.action.value}


@dataclass(frozen=True)
class PipetteAspirateCommand:
    volume_ul: float
    pressure_profile: str = "1R"

    def __post_init__(self) -> None:
        object.__setattr__(self, "volume_ul", validate_volume_ul(self.volume_ul))
        object.__setattr__(self, "pressure_profile", normalize_pressure_profile(self.pressure_profile))

    @classmethod
    def from_request(cls, req: Any) -> "PipetteAspirateCommand":
        return cls(volume_ul=getattr(req, "volume_ul"), pressure_profile=getattr(req, "pressure_profile", "1R"))

    def to_payload(self) -> dict[str, Any]:
        return {
            "volume_ul": float(self.volume_ul),
            "pressure_profile": self.pressure_profile,
        }


@dataclass(frozen=True)
class PipetteDispenseCommand:
    volume_ul: float
    pressure_profile: str = "1R"
    blow_out: bool = False

    def __post_init__(self) -> None:
        object.__setattr__(self, "volume_ul", validate_volume_ul(self.volume_ul))
        object.__setattr__(self, "pressure_profile", normalize_pressure_profile(self.pressure_profile))
        object.__setattr__(self, "blow_out", bool(self.blow_out))

    @classmethod
    def from_request(cls, req: Any) -> "PipetteDispenseCommand":
        return cls(
            volume_ul=getattr(req, "volume_ul"),
            pressure_profile=getattr(req, "pressure_profile", "1R"),
            blow_out=bool(getattr(req, "blow_out", False)),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "volume_ul": float(self.volume_ul),
            "pressure_profile": self.pressure_profile,
            "blow_out": bool(self.blow_out),
        }


@dataclass(frozen=True)
class PipetteMixCommand:
    volume_ul: float
    cycles: int
    pressure_profile: str = "1R"

    def __post_init__(self) -> None:
        object.__setattr__(self, "volume_ul", validate_volume_ul(self.volume_ul))
        cycles = int(self.cycles)
        if cycles < 1 or cycles > 50:
            raise PipetteValidationError("cycles must be between 1 and 50")
        object.__setattr__(self, "cycles", cycles)
        object.__setattr__(self, "pressure_profile", normalize_pressure_profile(self.pressure_profile))

    @classmethod
    def from_request(cls, req: Any) -> "PipetteMixCommand":
        return cls(
            volume_ul=getattr(req, "volume_ul"),
            cycles=getattr(req, "cycles"),
            pressure_profile=getattr(req, "pressure_profile", "1R"),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "volume_ul": float(self.volume_ul),
            "cycles": int(self.cycles),
            "pressure_profile": self.pressure_profile,
        }
