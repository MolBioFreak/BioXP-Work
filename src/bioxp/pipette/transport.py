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
        pipette_id: int = 0,
        response_timeout_s: float = 1.0,
    ) -> None:
        self._driver_factory = driver_factory or self._default_driver_factory(
            channel=channel,
            bitrate=bitrate,
            pipette_id=pipette_id,
            response_timeout_s=response_timeout_s,
        )
        self._driver: Any | None = None
        self._initialized = False
        self._tip_loaded = False
        self._pressure_profile = "1R"
        self._last_command: str | None = None
        self._last_transaction: dict[str, Any] | None = None
        self._last_tip_status: dict[str, Any] | None = None
        self._last_pressure: dict[str, Any] | None = None
        self._liquid_level_ul = 0.0
        self._channel = channel
        self._bitrate = int(bitrate)
        self._pipette_id = int(pipette_id)
        self._response_timeout_s = float(response_timeout_s)

    @staticmethod
    def _default_driver_factory(
        *,
        channel: str,
        bitrate: int,
        pipette_id: int,
        response_timeout_s: float,
    ) -> Callable[[], Any]:
        def _factory() -> Any:
            if BioXpCanDriver is None:
                raise PipetteTransportUnavailableError(
                    "python-can backend is unavailable for the BioXP pipette transport.",
                    details={"channel": channel, "bitrate": int(bitrate), "pipette_id": int(pipette_id)},
                )
            return BioXpCanDriver(
                channel=channel,
                bitrate=bitrate,
                pipette_id=pipette_id,
                response_timeout_s=response_timeout_s,
            )

        return _factory

    def _get_driver(self) -> Any:
        if self._driver is None:
            self._driver = self._driver_factory()
        return self._driver

    def _status_payload(self, **extra: Any) -> dict[str, Any]:
        ok = bool(extra.pop("ok", True))
        payload = {
            "ok": ok,
            "transport": "can",
            "channel": self._channel,
            "bitrate": self._bitrate,
            "pipette_id": self._pipette_id,
            "available": BioXpCanDriver is not None or self._driver is not None,
            "initialized": bool(self._initialized),
            "software_initialized": bool(self._initialized),
            "tip_loaded": bool(self._tip_loaded),
            "software_tip_loaded": bool(self._tip_loaded),
            "pressure_profile": self._pressure_profile,
            "last_command": self._last_command,
            "last_transaction": self._last_transaction,
            "hardware_tip_status": self._last_tip_status,
            "hardware_pressure": self._last_pressure,
            "hardware_truth_level": "software_shadow",
            "ack_required": True,
            "response_timeout_s": self._response_timeout_s,
            "liquid_level_ul": self._liquid_level_ul,
        }
        payload.update(extra)
        return payload

    def _require_initialized(self) -> None:
        if not self._initialized:
            raise PipetteNotReadyError(details={"software_initialized": bool(self._initialized)})

    def _assert_driver_result(self, command_name: str, result: Any) -> dict[str, Any]:
        if not isinstance(result, dict):
            raise PipetteCommandError(
                f"Pipette {command_name} returned an invalid driver result.",
                details={"driver_result": repr(result)},
            )
        self._last_transaction = result
        if not result.get("ok", False):
            reason = result.get("error") or result.get("message") or result.get("ack", {}).get("error") or "driver result was not ok"
            raise PipetteCommandError(
                f"Pipette {command_name} failed: {reason}",
                details={"command": command_name, "driver_result": result},
            )
        return result

    def _safe_query_tip_status(self, driver: Any, *, required: bool = False) -> dict[str, Any] | None:
        query_fn = getattr(driver, "query_tip_status", None)
        if not callable(query_fn):
            if required:
                raise PipetteCommandError(
                    "Pipette hardware tip-status query is unavailable; refusing to rely on software shadow tip state.",
                    details={"required": True},
                )
            return None
        try:
            result = query_fn()
        except Exception as exc:
            if required:
                raise PipetteCommandError(
                    f"Pipette hardware tip-status query failed: {exc}",
                    details={"exception": repr(exc)},
                ) from exc
            result = {"ok": False, "error": str(exc)}
        if isinstance(result, dict):
            self._last_tip_status = result
            if result.get("ok") and "tip_loaded" in result:
                self._tip_loaded = bool(result.get("tip_loaded"))
            elif required:
                raise PipetteCommandError(
                    "Pipette hardware tip-status query did not return a successful readback.",
                    details={"hardware_tip_status": result},
                )
            return result
        if required:
            raise PipetteCommandError(
                "Pipette hardware tip-status query returned an invalid payload.",
                details={"hardware_tip_status": repr(result)},
            )
        return None

    def _safe_query_pressure(self, driver: Any) -> dict[str, Any] | None:
        query_fn = getattr(driver, "query_pressure", None)
        if not callable(query_fn):
            return None
        try:
            result = query_fn()
        except Exception as exc:
            result = {"ok": False, "error": str(exc)}
        if isinstance(result, dict):
            self._last_pressure = result
            return result
        return None

    def _require_tip_loaded(self, driver: Any) -> dict[str, Any] | None:
        tip_status = self._safe_query_tip_status(driver, required=True)
        if tip_status is not None and tip_status.get("tip_loaded") is not True:
            raise PipetteTipStateError(
                "Hardware tip-status query does not report a loaded tip.",
                details={"hardware_tip_status": tip_status},
            )
        if not self._tip_loaded:
            raise PipetteTipStateError("Tip must be loaded before this operation.")
        return tip_status

    def get_status(self) -> dict[str, Any]:
        try:
            driver = self._get_driver()
        except PipetteTransportUnavailableError as exc:
            return self._status_payload(
                ok=False,
                available=False,
                error=exc.code,
                message=exc.message,
                details=exc.details,
                hardware_truth_level="unavailable",
            )
        except Exception as exc:
            return self._status_payload(
                ok=False,
                available=False,
                error="transport_open_failed",
                message=str(exc),
                hardware_truth_level="unavailable",
            )

        tip_status = self._safe_query_tip_status(driver)
        pressure = self._safe_query_pressure(driver)
        hardware_ok = any(
            isinstance(payload, dict) and payload.get("ok")
            for payload in (tip_status, pressure)
        )
        can_ids = getattr(driver, "pipette_can_ids", None)
        return self._status_payload(
            hardware_tip_status=tip_status,
            hardware_pressure=pressure,
            hardware_truth_level="hardware_query" if hardware_ok else "software_shadow",
            can_ids=can_ids() if callable(can_ids) else None,
        )

    def initialize(self, command: PipetteInitCommand) -> dict[str, Any]:
        driver = self._get_driver()
        init_result = self._assert_driver_result(
            "initialize",
            driver.pipette_initialize(pressure_profile=command.pressure_profile),
        )
        self._initialized = True
        self._pressure_profile = command.pressure_profile
        self._last_command = "initialize"
        payload = self._status_payload(
            command="initialize",
            driver_result=init_result,
            requested=command.to_payload(),
            hardware_truth_level="acknowledged_command",
        )
        if command.prime_volume_ul is not None:
            tip_status = self._require_tip_loaded(driver)
            aspirate_result = self._assert_driver_result(
                "prime_aspirate",
                driver.aspirate(command.prime_volume_ul, tip_pressure_profile=command.pressure_profile),
            )
            dispense_result = self._assert_driver_result(
                "prime_dispense",
                driver.dispense(command.prime_volume_ul, tip_pressure_profile=command.pressure_profile),
            )
            payload["prime"] = {
                "volume_ul": float(command.prime_volume_ul),
                "tip_status": tip_status,
                "aspirate": aspirate_result,
                "dispense": dispense_result,
            }
        return payload

    def set_tip(self, command: PipetteTipCommand) -> dict[str, Any]:
        self._require_initialized()
        driver = self._get_driver()
        if command.action is PipetteTipAction.LOAD:
            load_fn = getattr(driver, "pipette_load_tip", None)
            if not callable(load_fn):
                raise PipetteCommandError("Pipette load-tip readback is unavailable.")
            driver_result = self._assert_driver_result("tip:load", load_fn())
            tip_status = self._safe_query_tip_status(driver, required=True)
            if tip_status is not None and tip_status.get("tip_loaded") is not True:
                raise PipetteTipStateError(
                    "Requested tip load, but hardware query did not detect a loaded tip.",
                    details={"hardware_tip_status": tip_status, "driver_result": driver_result},
                )
            self._tip_loaded = True
        elif command.action is PipetteTipAction.EJECT:
            driver_result = self._assert_driver_result("tip:eject", driver.pipette_eject_tip())
            tip_status = self._safe_query_tip_status(driver)
            if tip_status is not None and tip_status.get("ok") and tip_status.get("tip_loaded") is True:
                raise PipetteTipStateError(
                    "Pipette eject was acknowledged, but hardware query still reports a loaded tip.",
                    details={"hardware_tip_status": tip_status, "driver_result": driver_result},
                )
            self._tip_loaded = False
        else:  # pragma: no cover - enum exhaustiveness
            raise PipetteCommandError(f"Unsupported tip action: {command.action!r}")
        self._last_command = f"tip:{command.action.value}"
        return self._status_payload(
            command="tip",
            action=command.action.value,
            requested=command.to_payload(),
            driver_result=driver_result,
            hardware_tip_status=self._last_tip_status,
            hardware_truth_level="hardware_query",
        )

    def aspirate(self, command: PipetteAspirateCommand) -> dict[str, Any]:
        self._require_initialized()
        driver = self._get_driver()
        tip_status = self._require_tip_loaded(driver)
        driver_result = self._assert_driver_result(
            "aspirate",
            driver.aspirate(command.volume_ul, tip_pressure_profile=command.pressure_profile),
        )
        self._pressure_profile = command.pressure_profile
        self._last_command = "aspirate"
        self._liquid_level_ul += float(command.volume_ul)
        return self._status_payload(
            command="aspirate",
            requested=command.to_payload(),
            volume_ul=float(command.volume_ul),
            driver_result=driver_result,
            hardware_tip_status=tip_status,
            hardware_truth_level="acknowledged_command_with_tip_readback",
        )

    def dispense(self, command: PipetteDispenseCommand) -> dict[str, Any]:
        self._require_initialized()
        driver = self._get_driver()
        tip_status = self._require_tip_loaded(driver)
        driver_result = self._assert_driver_result(
            "dispense",
            driver.dispense(command.volume_ul, tip_pressure_profile=command.pressure_profile, blow_out=command.blow_out),
        )
        self._pressure_profile = command.pressure_profile
        self._last_command = "dispense"
        self._liquid_level_ul = max(0.0, self._liquid_level_ul - float(command.volume_ul))
        return self._status_payload(
            command="dispense",
            requested=command.to_payload(),
            volume_ul=float(command.volume_ul),
            blow_out=bool(command.blow_out),
            driver_result=driver_result,
            hardware_tip_status=tip_status,
            hardware_truth_level="acknowledged_command_with_tip_readback",
        )

    def mix(self, command: PipetteMixCommand) -> dict[str, Any]:
        self._require_initialized()
        driver = self._get_driver()
        tip_status = self._require_tip_loaded(driver)
        cycle_results = []
        for cycle_index in range(1, int(command.cycles) + 1):
            aspirate_result = self._assert_driver_result(
                f"mix:{cycle_index}:aspirate",
                driver.aspirate(command.volume_ul, tip_pressure_profile=command.pressure_profile),
            )
            dispense_result = self._assert_driver_result(
                f"mix:{cycle_index}:dispense",
                driver.dispense(command.volume_ul, tip_pressure_profile=command.pressure_profile, blow_out=False),
            )
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
            requested=command.to_payload(),
            cycles=int(command.cycles),
            volume_ul=float(command.volume_ul),
            cycle_results=cycle_results,
            hardware_tip_status=tip_status,
            hardware_truth_level="acknowledged_command_with_tip_readback",
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
