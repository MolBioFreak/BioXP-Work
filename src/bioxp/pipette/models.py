from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping


_MAX_VOLUME_UL = 1000.0
_MAX_TEXT_LENGTH = 120


class PipetteTipAction(str, Enum):
    LOAD = "load"
    EJECT = "eject"


class PipetteStatusCode(int, Enum):
    NOT_DETERMINED = 0
    STATUS_FALSE = 1
    STATUS_TRUE = 2


class PipetteErrorCode(int, Enum):
    NO_ERROR = 32
    INITIALIZATION_ERROR = 33
    INVALID_COMMAND = 34
    INVALID_OPERAND = 35
    PRESSURE_ERROR = 36
    OVER_PRESSURE = 37
    LIQUIE_LEVEL_DETECT_FAILURE = 38
    DEVICE_NOT_INITIALIZED = 39
    TIP_EJECT_FAILURE = 40
    PLUNGER_OVERLOAD = 41
    TIP_LOST = 42
    NOT_USED = 43
    EXTENDED_ERROR = 44
    NVMEM_ACCESS_FAILURE = 45
    COMMAND_BUFFFER_EMPTY = 46
    COMMAND_BUFFER_OVERFLOW = 47


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


class PipettePreflightError(PipetteError):
    def __init__(self, message: str = "Pipette preflight failed.", *, details: dict[str, Any] | None = None) -> None:
        super().__init__(message, code="preflight_failed", status_code=409, details=details)


class PipetteValidationError(PipetteError):
    def __init__(self, message: str, *, details: dict[str, Any] | None = None) -> None:
        super().__init__(message, code="validation_error", status_code=400, details=details)


def _mapping_get(value: Any, key: str, default: Any = None) -> Any:
    if isinstance(value, Mapping):
        return value.get(key, default)
    return getattr(value, key, default)


def _normalize_optional_text(value: Any, *, field_name: str, max_length: int = _MAX_TEXT_LENGTH, uppercase: bool = False) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    if len(text) > max_length:
        raise PipetteValidationError(f"{field_name} must be <= {max_length} characters")
    return text.upper() if uppercase else text


def _normalize_required_text(value: Any, *, field_name: str, max_length: int = _MAX_TEXT_LENGTH, uppercase: bool = False) -> str:
    text = _normalize_optional_text(value, field_name=field_name, max_length=max_length, uppercase=uppercase)
    if text is None:
        raise PipetteValidationError(f"{field_name} must not be empty")
    return text


def _normalize_optional_nonnegative_volume(value: Any, *, field_name: str) -> float | None:
    if value is None:
        return None
    volume_ul = float(value)
    if volume_ul < 0.0:
        raise PipetteValidationError(f"{field_name} must be >= 0")
    if volume_ul > _MAX_VOLUME_UL:
        raise PipetteValidationError(f"{field_name} must be <= {_MAX_VOLUME_UL:g}")
    return volume_ul


def _normalize_optional_int(value: Any, *, field_name: str) -> int | None:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise PipetteValidationError(f"{field_name} must be an integer") from exc


def _normalize_metadata(value: Any) -> dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise PipetteValidationError("metadata must be an object")
    return dict(value)


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
class LiquidLocation:
    location_id: str
    well_id: str | None = None
    plate_name: str | None = None
    z_offset_steps: int | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "location_id", _normalize_required_text(self.location_id, field_name="location_id"))
        object.__setattr__(self, "well_id", _normalize_optional_text(self.well_id, field_name="well_id", uppercase=True))
        object.__setattr__(self, "plate_name", _normalize_optional_text(self.plate_name, field_name="plate_name"))
        object.__setattr__(self, "z_offset_steps", _normalize_optional_int(self.z_offset_steps, field_name="z_offset_steps"))

    @classmethod
    def from_value(cls, value: Any) -> "LiquidLocation | None":
        if value is None:
            return None
        if isinstance(value, cls):
            return value
        return cls(
            location_id=_mapping_get(value, "location_id"),
            well_id=_mapping_get(value, "well_id"),
            plate_name=_mapping_get(value, "plate_name"),
            z_offset_steps=_mapping_get(value, "z_offset_steps"),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "location_id": self.location_id,
            "well_id": self.well_id,
            "plate_name": self.plate_name,
            "z_offset_steps": self.z_offset_steps,
        }


@dataclass(frozen=True)
class PipetteOperationContext:
    source: LiquidLocation | None = None
    destination: LiquidLocation | None = None
    liquid_class: str | None = None
    tip_id: str | None = None
    air_gap_ul: float | None = None
    operator: str | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "source", LiquidLocation.from_value(self.source))
        object.__setattr__(self, "destination", LiquidLocation.from_value(self.destination))
        object.__setattr__(self, "liquid_class", _normalize_optional_text(self.liquid_class, field_name="liquid_class"))
        object.__setattr__(self, "tip_id", _normalize_optional_text(self.tip_id, field_name="tip_id"))
        object.__setattr__(self, "air_gap_ul", _normalize_optional_nonnegative_volume(self.air_gap_ul, field_name="air_gap_ul"))
        object.__setattr__(self, "operator", _normalize_optional_text(self.operator, field_name="operator"))
        object.__setattr__(self, "metadata", _normalize_metadata(self.metadata))

    @classmethod
    def from_request(cls, req: Any, *, default_destination_key: str = "destination") -> "PipetteOperationContext":
        destination = _mapping_get(req, default_destination_key)
        if destination is None:
            destination = _mapping_get(req, "dest")
        return cls(
            source=_mapping_get(req, "source"),
            destination=destination,
            liquid_class=_mapping_get(req, "liquid_class"),
            tip_id=_mapping_get(req, "tip_id"),
            air_gap_ul=_mapping_get(req, "air_gap_ul"),
            operator=_mapping_get(req, "operator"),
            metadata=_mapping_get(req, "metadata", {}),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "source": None if self.source is None else self.source.to_payload(),
            "destination": None if self.destination is None else self.destination.to_payload(),
            "liquid_class": self.liquid_class,
            "tip_id": self.tip_id,
            "air_gap_ul": self.air_gap_ul,
            "operator": self.operator,
            "metadata": dict(self.metadata),
        }


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
            pressure_profile=_mapping_get(req, "pressure_profile", "1R"),
            prime_volume_ul=_mapping_get(req, "prime_volume_ul", None),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "pressure_profile": self.pressure_profile,
            "prime_volume_ul": self.prime_volume_ul,
        }


@dataclass(frozen=True)
class PipetteTipCommand:
    action: PipetteTipAction
    tip_id: str | None = None
    operator: str | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not isinstance(self.action, PipetteTipAction):
            object.__setattr__(self, "action", PipetteTipAction(str(self.action)))
        object.__setattr__(self, "tip_id", _normalize_optional_text(self.tip_id, field_name="tip_id"))
        object.__setattr__(self, "operator", _normalize_optional_text(self.operator, field_name="operator"))
        object.__setattr__(self, "metadata", _normalize_metadata(self.metadata))

    @classmethod
    def from_request(cls, req: Any) -> "PipetteTipCommand":
        return cls(
            action=_mapping_get(req, "action"),
            tip_id=_mapping_get(req, "tip_id"),
            operator=_mapping_get(req, "operator"),
            metadata=_mapping_get(req, "metadata", {}),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "action": self.action.value,
            "tip_id": self.tip_id,
            "operator": self.operator,
            "metadata": dict(self.metadata),
        }


@dataclass(frozen=True)
class PipetteAspirateCommand:
    volume_ul: float
    pressure_profile: str = "1R"
    source: LiquidLocation | Mapping[str, Any] | Any | None = None
    liquid_class: str | None = None
    tip_id: str | None = None
    air_gap_ul: float | None = None
    operator: str | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "volume_ul", validate_volume_ul(self.volume_ul))
        object.__setattr__(self, "pressure_profile", normalize_pressure_profile(self.pressure_profile))
        context = PipetteOperationContext(
            source=self.source,
            liquid_class=self.liquid_class,
            tip_id=self.tip_id,
            air_gap_ul=self.air_gap_ul,
            operator=self.operator,
            metadata=self.metadata,
        )
        object.__setattr__(self, "source", context.source)
        object.__setattr__(self, "liquid_class", context.liquid_class)
        object.__setattr__(self, "tip_id", context.tip_id)
        object.__setattr__(self, "air_gap_ul", context.air_gap_ul)
        object.__setattr__(self, "operator", context.operator)
        object.__setattr__(self, "metadata", context.metadata)

    @classmethod
    def from_request(cls, req: Any) -> "PipetteAspirateCommand":
        return cls(
            volume_ul=_mapping_get(req, "volume_ul"),
            pressure_profile=_mapping_get(req, "pressure_profile", "1R"),
            source=_mapping_get(req, "source"),
            liquid_class=_mapping_get(req, "liquid_class"),
            tip_id=_mapping_get(req, "tip_id"),
            air_gap_ul=_mapping_get(req, "air_gap_ul"),
            operator=_mapping_get(req, "operator"),
            metadata=_mapping_get(req, "metadata", {}),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "volume_ul": float(self.volume_ul),
            "pressure_profile": self.pressure_profile,
            "source": None if self.source is None else self.source.to_payload(),
            "liquid_class": self.liquid_class,
            "tip_id": self.tip_id,
            "air_gap_ul": self.air_gap_ul,
            "operator": self.operator,
            "metadata": dict(self.metadata),
        }


@dataclass(frozen=True)
class PipetteDispenseCommand:
    volume_ul: float
    pressure_profile: str = "1R"
    blow_out: bool = False
    destination: LiquidLocation | Mapping[str, Any] | Any | None = None
    liquid_class: str | None = None
    tip_id: str | None = None
    air_gap_ul: float | None = None
    operator: str | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "volume_ul", validate_volume_ul(self.volume_ul))
        object.__setattr__(self, "pressure_profile", normalize_pressure_profile(self.pressure_profile))
        object.__setattr__(self, "blow_out", bool(self.blow_out))
        context = PipetteOperationContext(
            destination=self.destination,
            liquid_class=self.liquid_class,
            tip_id=self.tip_id,
            air_gap_ul=self.air_gap_ul,
            operator=self.operator,
            metadata=self.metadata,
        )
        object.__setattr__(self, "destination", context.destination)
        object.__setattr__(self, "liquid_class", context.liquid_class)
        object.__setattr__(self, "tip_id", context.tip_id)
        object.__setattr__(self, "air_gap_ul", context.air_gap_ul)
        object.__setattr__(self, "operator", context.operator)
        object.__setattr__(self, "metadata", context.metadata)

    @classmethod
    def from_request(cls, req: Any) -> "PipetteDispenseCommand":
        destination = _mapping_get(req, "destination")
        if destination is None:
            destination = _mapping_get(req, "dest")
        return cls(
            volume_ul=_mapping_get(req, "volume_ul"),
            pressure_profile=_mapping_get(req, "pressure_profile", "1R"),
            blow_out=bool(_mapping_get(req, "blow_out", False)),
            destination=destination,
            liquid_class=_mapping_get(req, "liquid_class"),
            tip_id=_mapping_get(req, "tip_id"),
            air_gap_ul=_mapping_get(req, "air_gap_ul"),
            operator=_mapping_get(req, "operator"),
            metadata=_mapping_get(req, "metadata", {}),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "volume_ul": float(self.volume_ul),
            "pressure_profile": self.pressure_profile,
            "blow_out": bool(self.blow_out),
            "destination": None if self.destination is None else self.destination.to_payload(),
            "liquid_class": self.liquid_class,
            "tip_id": self.tip_id,
            "air_gap_ul": self.air_gap_ul,
            "operator": self.operator,
            "metadata": dict(self.metadata),
        }


@dataclass(frozen=True)
class PipetteMixCommand:
    volume_ul: float
    cycles: int
    pressure_profile: str = "1R"
    location: LiquidLocation | Mapping[str, Any] | Any | None = None
    liquid_class: str | None = None
    tip_id: str | None = None
    operator: str | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "volume_ul", validate_volume_ul(self.volume_ul))
        cycles = int(self.cycles)
        if cycles < 1 or cycles > 50:
            raise PipetteValidationError("cycles must be between 1 and 50")
        object.__setattr__(self, "cycles", cycles)
        object.__setattr__(self, "pressure_profile", normalize_pressure_profile(self.pressure_profile))
        context = PipetteOperationContext(
            source=self.location,
            destination=self.location,
            liquid_class=self.liquid_class,
            tip_id=self.tip_id,
            operator=self.operator,
            metadata=self.metadata,
        )
        object.__setattr__(self, "location", context.source)
        object.__setattr__(self, "liquid_class", context.liquid_class)
        object.__setattr__(self, "tip_id", context.tip_id)
        object.__setattr__(self, "operator", context.operator)
        object.__setattr__(self, "metadata", context.metadata)

    @classmethod
    def from_request(cls, req: Any) -> "PipetteMixCommand":
        location = _mapping_get(req, "location")
        if location is None:
            location = _mapping_get(req, "source")
        if location is None:
            location = _mapping_get(req, "destination")
        if location is None:
            location = _mapping_get(req, "dest")
        return cls(
            volume_ul=_mapping_get(req, "volume_ul"),
            cycles=_mapping_get(req, "cycles"),
            pressure_profile=_mapping_get(req, "pressure_profile", "1R"),
            location=location,
            liquid_class=_mapping_get(req, "liquid_class"),
            tip_id=_mapping_get(req, "tip_id"),
            operator=_mapping_get(req, "operator"),
            metadata=_mapping_get(req, "metadata", {}),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "volume_ul": float(self.volume_ul),
            "cycles": int(self.cycles),
            "pressure_profile": self.pressure_profile,
            "location": None if self.location is None else self.location.to_payload(),
            "liquid_class": self.liquid_class,
            "tip_id": self.tip_id,
            "operator": self.operator,
            "metadata": dict(self.metadata),
        }


@dataclass(frozen=True)
class PipetteTerminateCommand:
    operator: str | None = None
    reason: str | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "operator", _normalize_optional_text(self.operator, field_name="operator"))
        object.__setattr__(self, "reason", _normalize_optional_text(self.reason, field_name="reason"))
        object.__setattr__(self, "metadata", _normalize_metadata(self.metadata))

    @classmethod
    def from_request(cls, req: Any) -> "PipetteTerminateCommand":
        return cls(
            operator=_mapping_get(req, "operator"),
            reason=_mapping_get(req, "reason"),
            metadata=_mapping_get(req, "metadata", {}),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "operator": self.operator,
            "reason": self.reason,
            "metadata": dict(self.metadata),
        }


@dataclass(frozen=True)
class PipetteHeartbeatCommand:
    enabled: bool

    @classmethod
    def from_request(cls, req: Any) -> "PipetteHeartbeatCommand":
        return cls(enabled=bool(_mapping_get(req, "enabled")))

    def to_payload(self) -> dict[str, Any]:
        return {"enabled": bool(self.enabled)}


@dataclass(frozen=True)
class PipetteDiagnosticCommand:
    number: int

    def __post_init__(self) -> None:
        number = int(self.number)
        if number < 0 or number > 9:
            raise PipetteValidationError("diagnostic number must be an ASCII digit 0..9")
        object.__setattr__(self, "number", number)

    def to_payload(self) -> dict[str, Any]:
        return {"number": int(self.number)}


@dataclass(frozen=True)
class PipetteErrorLogCommand:
    raw_byte: int = 0

    def __post_init__(self) -> None:
        value = int(self.raw_byte)
        if value < 0 or value > 255:
            raise PipetteValidationError("error-log raw_byte must be between 0 and 255")
        object.__setattr__(self, "raw_byte", value)

    def to_payload(self) -> dict[str, Any]:
        return {"raw_byte": int(self.raw_byte)}
