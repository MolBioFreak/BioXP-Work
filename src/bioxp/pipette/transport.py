from __future__ import annotations

from typing import Any, Callable, Protocol

from .. import BioXpCanDriver
from .models import (
    PipetteAspirateCommand,
    PipetteCommandError,
    PipetteDispenseCommand,
    PipetteInitCommand,
    PipetteMixCommand,
    PipetteNotReadyError,
    PipetteTipAction,
    PipetteTipCommand,
    PipetteTipStateError,
    PipetteTransportUnavailableError,
)


class PipetteTransport(Protocol):
    def get_status(self) -> dict[str, Any]: ...

    def initialize(self, command: PipetteInitCommand) -> dict[str, Any]: ...

    def set_tip(self, command: PipetteTipCommand) -> dict[str, Any]: ...

    def aspirate(self, command: PipetteAspirateCommand) -> dict[str, Any]: ...

    def dispense(self, command: PipetteDispenseCommand) -> dict[str, Any]: ...

    def mix(self, command: PipetteMixCommand) -> dict[str, Any]: ...

    def close(self) -> None: ...


class CanPipetteTransport:
    def __init__(
        self,
        *,
        driver_factory: Callable[[], Any] | None = None,
        channel: str = "can0",
        bitrate: int = 1_000_000,
    ) -> None:
        self._driver_factory = driver_factory or self._default_driver_factory(channel=channel, bitrate=bitrate)
        self._driver: Any | None = None
        self._initialized = False
        self._tip_loaded = False
        self._pressure_profile = "1R"
        self._last_command: str | None = None
        self._channel = channel
        self._bitrate = int(bitrate)

    @staticmethod
    def _default_driver_factory(*, channel: str, bitrate: int) -> Callable[[], Any]:
        def _factory() -> Any:
            if BioXpCanDriver is None:
                raise PipetteTransportUnavailableError(
                    "python-can backend is unavailable for the BioXP pipette transport.",
                    details={"channel": channel, "bitrate": int(bitrate)},
                )
            return BioXpCanDriver(channel=channel, bitrate=bitrate)

        return _factory

    def _get_driver(self) -> Any:
        if self._driver is None:
            self._driver = self._driver_factory()
        return self._driver

    def _status_payload(self, **extra: Any) -> dict[str, Any]:
        payload = {
            "ok": True,
            "transport": "can",
            "channel": self._channel,
            "bitrate": self._bitrate,
            "available": BioXpCanDriver is not None,
            "initialized": bool(self._initialized),
            "tip_loaded": bool(self._tip_loaded),
            "pressure_profile": self._pressure_profile,
            "last_command": self._last_command,
        }
        payload.update(extra)
        return payload

    def _require_initialized(self) -> None:
        if not self._initialized:
            raise PipetteNotReadyError()

    def _require_tip_loaded(self) -> None:
        if not self._tip_loaded:
            raise PipetteTipStateError("Tip must be loaded before this operation.")

    def get_status(self) -> dict[str, Any]:
        return self._status_payload()

    def initialize(self, command: PipetteInitCommand) -> dict[str, Any]:
        driver = self._get_driver()
        init_result = driver.pipette_initialize(pressure_profile=command.pressure_profile)
        self._initialized = True
        self._pressure_profile = command.pressure_profile
        self._last_command = "initialize"
        payload = self._status_payload(command="initialize", driver_result=init_result)
        if command.prime_volume_ul is not None:
            aspirate_result = driver.aspirate(command.prime_volume_ul, tip_pressure_profile=command.pressure_profile)
            dispense_result = driver.dispense(command.prime_volume_ul, tip_pressure_profile=command.pressure_profile)
            payload["prime"] = {
                "volume_ul": float(command.prime_volume_ul),
                "aspirate": aspirate_result,
                "dispense": dispense_result,
            }
        return payload

    def set_tip(self, command: PipetteTipCommand) -> dict[str, Any]:
        self._require_initialized()
        driver = self._get_driver()
        if command.action is PipetteTipAction.LOAD:
            driver_result = driver.pipette_load_tip()
            self._tip_loaded = True
        elif command.action is PipetteTipAction.EJECT:
            driver_result = driver.pipette_eject_tip()
            self._tip_loaded = False
        else:  # pragma: no cover - enum exhaustiveness
            raise PipetteCommandError(f"Unsupported tip action: {command.action!r}")
        self._last_command = f"tip:{command.action.value}"
        return self._status_payload(command="tip", action=command.action.value, driver_result=driver_result)

    def aspirate(self, command: PipetteAspirateCommand) -> dict[str, Any]:
        self._require_initialized()
        self._require_tip_loaded()
        driver = self._get_driver()
        driver_result = driver.aspirate(command.volume_ul, tip_pressure_profile=command.pressure_profile)
        self._pressure_profile = command.pressure_profile
        self._last_command = "aspirate"
        return self._status_payload(command="aspirate", volume_ul=float(command.volume_ul), driver_result=driver_result)

    def dispense(self, command: PipetteDispenseCommand) -> dict[str, Any]:
        self._require_initialized()
        self._require_tip_loaded()
        driver = self._get_driver()
        driver_result = driver.dispense(command.volume_ul, tip_pressure_profile=command.pressure_profile, blow_out=command.blow_out)
        self._pressure_profile = command.pressure_profile
        self._last_command = "dispense"
        return self._status_payload(
            command="dispense",
            volume_ul=float(command.volume_ul),
            blow_out=bool(command.blow_out),
            driver_result=driver_result,
        )

    def mix(self, command: PipetteMixCommand) -> dict[str, Any]:
        self._require_initialized()
        self._require_tip_loaded()
        driver = self._get_driver()
        cycle_results = []
        for cycle_index in range(1, int(command.cycles) + 1):
            aspirate_result = driver.aspirate(command.volume_ul, tip_pressure_profile=command.pressure_profile)
            dispense_result = driver.dispense(command.volume_ul, tip_pressure_profile=command.pressure_profile, blow_out=False)
            cycle_results.append(
                {
                    "cycle": cycle_index,
                    "aspirate": aspirate_result,
                    "dispense": dispense_result,
                }
            )
        self._pressure_profile = command.pressure_profile
        self._last_command = "mix"
        return self._status_payload(
            command="mix",
            cycles=int(command.cycles),
            volume_ul=float(command.volume_ul),
            cycle_results=cycle_results,
        )

    def close(self) -> None:
        driver = self._driver
        self._driver = None
        if driver is None:
            return
        close_fn = getattr(driver, "close", None)
        if callable(close_fn):
            close_fn()


def build_default_pipette_transport() -> CanPipetteTransport:
    return CanPipetteTransport()
