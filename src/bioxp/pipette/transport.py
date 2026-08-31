from __future__ import annotations

import os
import inspect
import threading
import time
from typing import Any, Callable, Mapping, Protocol

from .. import BioXpCanDriver
try:
    from ..novo_usb_can import BioXpNovoUsbDriver
except Exception:  # pragma: no cover - surfaced lazily in status payloads
    BioXpNovoUsbDriver = None  # type: ignore[assignment]
from .models import (
    PipetteAspirateCommand,
    PipetteCommandError,
    PipetteDiagnosticCommand,
    PipetteDispenseCommand,
    PipetteErrorLogCommand,
    PipetteHeartbeatCommand,
    PipetteInitCommand,
    PipetteMixCommand,
    PipetteNotReadyError,
    PipetteTerminateCommand,
    PipetteTipAction,
    PipetteTipCommand,
    PipetteTipStateError,
    PipetteTransportUnavailableError,
)


def _call_eject_tip(
    driver: Any,
    *,
    initialized: bool,
    wait_for_completion: bool = True,
) -> Any:
    eject_fn = getattr(driver, "pipette_eject_tip")
    try:
        parameters = inspect.signature(eject_fn).parameters
    except (TypeError, ValueError):
        parameters = {}
    kwargs: dict[str, Any] = {}
    if "initialized" in parameters:
        kwargs["initialized"] = bool(initialized)
    if "wait_for_completion" in parameters:
        kwargs["wait_for_completion"] = bool(wait_for_completion)
    return eject_fn(**kwargs)


class PipetteTransport(Protocol):
    def get_status(self) -> dict[str, Any]: ...

    def initialize(self, command: PipetteInitCommand) -> dict[str, Any]: ...

    def set_tip(self, command: PipetteTipCommand) -> dict[str, Any]: ...

    def aspirate(self, command: PipetteAspirateCommand) -> dict[str, Any]: ...

    def dispense(self, command: PipetteDispenseCommand) -> dict[str, Any]: ...

    def mix(self, command: PipetteMixCommand) -> dict[str, Any]: ...

    def dispense_all(self) -> dict[str, Any]: ...

    def aspirate_air(
        self,
        volume_ul: float,
        *,
        air_type: int = 1,
        front_air: bool = True,
        wait_for_completion: bool = True,
    ) -> dict[str, Any]: ...

    def dispense_air(
        self,
        volume_ul: float,
        *,
        dispense_type: int = 0,
        wait_for_completion: bool = True,
    ) -> dict[str, Any]: ...

    def start_fluid_detection(self, *, wait_for_completion: bool = True) -> dict[str, Any]: ...

    def terminate(self, command: PipetteTerminateCommand | None = None) -> dict[str, Any]: ...

    def set_top_speed(self, velocity: float, channels: list[int] | None = None) -> dict[str, Any]: ...

    def heartbeat(self, command: PipetteHeartbeatCommand) -> dict[str, Any]: ...

    def close(self) -> None: ...


class CanPipetteTransport:
    def __init__(
        self,
        *,
        driver_factory: Callable[[], Any] | None = None,
        channel: str = "can0",
        bitrate: int = 1_000_000,
        pipette_id: int = 0,
        response_timeout_s: float = 60.0,
        transport_name: str = "can",
        transport_details: dict[str, Any] | None = None,
        error_callback: Callable[[int, int], None] | None = None,
    ) -> None:
        self._driver_factory = driver_factory or self._default_driver_factory(
            channel=channel,
            bitrate=bitrate,
            pipette_id=pipette_id,
            response_timeout_s=response_timeout_s,
        )
        self._transport_name = str(transport_name)
        self._transport_details = dict(transport_details or {})
        self._driver: Any | None = None
        self._initialized = False
        self._tip_loaded = False
        self._pressure_profile = "1R"
        self._last_command: str | None = None
        self._last_transaction: dict[str, Any] | None = None
        self._last_tip_status: dict[str, Any] | None = None
        self._last_pressure: dict[str, Any] | None = None
        self._liquid_level_ul = 0.0
        self._front_air_level_ul = 0.0
        self._rear_air_level_ul = 0.0
        self._top_speed = 1000.0
        self._channel = channel
        self._bitrate = int(bitrate)
        self._pipette_id = int(pipette_id)
        self._response_timeout_s = float(response_timeout_s)
        self._error_callback = error_callback

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
            if callable(self._error_callback):
                setattr(self._driver, "_pipette_error_callback", self._error_callback)
        return self._driver

    def _status_payload(self, **extra: Any) -> dict[str, Any]:
        ok = bool(extra.pop("ok", True))
        delivery_verified = bool(extra.pop("delivery_verified", False))
        controller_acknowledged = extra.pop("controller_acknowledged", None)
        completion_verified = bool(extra.pop("completion_verified", False))
        semantic_query_response_verified = bool(extra.pop("semantic_query_response_verified", False))
        hardware_precondition_verified = bool(extra.pop("hardware_precondition_verified", False))
        hardware_postcondition_verified = bool(extra.pop("hardware_postcondition_verified", False))
        state_reconciled = bool(extra.pop("state_reconciled", False))
        state_reconciliation_source = extra.pop("state_reconciliation_source", None)
        physical_effect_verified = bool(extra.pop("physical_effect_verified", False))
        raw_message_state = getattr(self._driver, "_pipette_message_state", {}) if self._driver is not None else {}
        message_state = dict(raw_message_state) if isinstance(raw_message_state, Mapping) else {}
        error_queue = message_state.get("error_queue", [])
        if not isinstance(error_queue, list):
            error_queue = []
        payload = {
            "ok": ok,
            "transport": self._transport_name,
            "channel": self._channel,
            "bitrate": self._bitrate,
            "pipette_id": self._pipette_id,
            "transport_details": dict(self._transport_details),
            "available": BioXpCanDriver is not None or self._driver is not None,
            "initialized": bool(self._initialized),
            "software_initialized": bool(self._initialized),
            "tip_loaded": bool(self._tip_loaded),
            "software_tip_loaded": bool(self._tip_loaded),
            "pressure_profile": self._pressure_profile,
            "top_speed": self._top_speed,
            "last_command": self._last_command,
            "last_transaction": self._last_transaction,
            "pipette_message_state": message_state,
            "oem_initialization_counter": int(message_state.get("initialization_counter", 0) or 0),
            "oem_diagnosis": message_state.get("diagnosis"),
            "oem_error_queue": [int(value) for value in error_queue],
            "oem_process_error_code": message_state.get("error_code"),
            "hardware_tip_status": self._last_tip_status,
            "hardware_pressure": self._last_pressure,
            "hardware_truth_level": "software_shadow",
            "ack_required": True,
            "delivery_verified": delivery_verified,
            "controller_acknowledged": controller_acknowledged,
            "completion_verified": completion_verified,
            "semantic_query_response_verified": semantic_query_response_verified,
            "hardware_precondition_verified": hardware_precondition_verified,
            "hardware_postcondition_verified": hardware_postcondition_verified,
            "state_reconciled": state_reconciled,
            "state_reconciliation_source": state_reconciliation_source,
            "physical_effect_verified": physical_effect_verified,
            "response_timeout_s": self._response_timeout_s,
            "liquid_level_ul": self._liquid_level_ul,
            "front_air_level_ul": self._front_air_level_ul,
            "rear_air_level_ul": self._rear_air_level_ul,
        }
        payload.update(extra)
        return payload

    @staticmethod
    def _driver_evidence(result: Mapping[str, Any]) -> dict[str, bool]:
        delivery = result.get("delivery_verified")
        if not isinstance(delivery, bool):
            delivery = bool(result.get("tx_ok", False))

        controller = result.get("controller_acknowledged")
        if not isinstance(controller, bool):
            ack_result = result.get("ack")
            ack = ack_result if isinstance(ack_result, Mapping) else {}
            controller = bool(ack.get("ok") and ack.get("received"))

        completion = result.get("completion_verified")
        if not isinstance(completion, bool):
            provenance_result = result.get("provenance")
            provenance = provenance_result if isinstance(provenance_result, Mapping) else {}
            completion = bool(provenance.get("completion_received", False))

        return {
            "delivery_verified": bool(delivery),
            "controller_acknowledged": bool(controller),
            "completion_verified": bool(completion),
        }

    def apply_completed_effect(self, command: str, result: Mapping[str, Any]) -> dict[str, Any]:
        """Apply deterministic host accounting only after correlated completion."""
        operation = str(command).replace(" ", "_")
        volume = float(result.get("volume_ul", 0.0) or 0.0)
        if operation == "aspirate":
            self._liquid_level_ul += volume
        elif operation == "dispense":
            dispense_type = int(result.get("dispense_type", 0) or 0)
            if dispense_type == 0:
                self._liquid_level_ul = max(0.0, self._liquid_level_ul - volume)
            elif dispense_type == 1:
                self._front_air_level_ul = max(0.0, self._front_air_level_ul - volume)
            else:
                self._rear_air_level_ul -= volume
        elif operation == "mix":
            self._liquid_level_ul += volume
        elif operation == "dispense_all":
            self._liquid_level_ul = 0.0
            self._front_air_level_ul = 0.0
            self._rear_air_level_ul = 0.0
        elif operation == "aspirate_air":
            if bool(result.get("front_air", True)):
                self._front_air_level_ul += volume
            else:
                self._rear_air_level_ul += volume
        elif operation == "dispense_air":
            dispense_type = int(result.get("dispense_type", 0) or 0)
            if dispense_type == 0:
                self._liquid_level_ul -= volume
                if self._liquid_level_ul < 0.0:
                    self._rear_air_level_ul += self._liquid_level_ul
                    self._liquid_level_ul = 0.0
            elif dispense_type == 1:
                self._front_air_level_ul -= volume
                if self._front_air_level_ul < 0.0:
                    self._liquid_level_ul += self._front_air_level_ul
                    self._front_air_level_ul = 0.0
            else:
                self._rear_air_level_ul -= volume
        return {
            "state_reconciled": True,
            "state_reconciliation_source": "correlated_controller_completion_accounting",
            "liquid_level_ul": self._liquid_level_ul,
            "front_air_level_ul": self._front_air_level_ul,
            "rear_air_level_ul": self._rear_air_level_ul,
        }

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
            result = dict(result)
            provenance = result.get("provenance")
            reader_generation = (
                provenance.get("reader_generation", provenance.get("owner_generation"))
                if isinstance(provenance, Mapping)
                else result.get("reader_generation")
            )
            result["observed_at"] = time.time()
            result["reader_generation"] = reader_generation
            self._reader_generation = reader_generation
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

    def _require_tip_loaded(
        self,
        driver: Any,
        *,
        query_hardware: bool = True,
    ) -> dict[str, Any] | None:
        tip_status = (
            self._safe_query_tip_status(driver, required=True)
            if query_hardware
            else self._last_tip_status
        )
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
        if command.prime_volume_ul is not None:
            raise PipetteCommandError(
                "Initialization priming remains blocked; this pass enables no liquid mutation.",
                details={"prime_volume_ul": command.prime_volume_ul, "liquid_mutation_enabled": False},
            )
        driver = self._get_driver()
        init_result = self._assert_driver_result(
            "initialize",
            driver.pipette_initialize(pressure_profile=command.pressure_profile),
        )
        self._initialized = bool(init_result.get("initialized_after_valid_completion"))
        self._pressure_profile = command.pressure_profile
        self._last_command = "initialize"
        payload = self._status_payload(
            command="initialize",
            driver_result=init_result,
            requested=command.to_payload(),
            **self._driver_evidence(init_result),
            physical_effect_verified=False,
            hardware_truth_level="acknowledged_command",
        )
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
            driver_result = self._assert_driver_result(
                "tip:eject",
                _call_eject_tip(driver, initialized=bool(self._initialized)),
            )
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
            **self._driver_evidence(driver_result),
            hardware_postcondition_verified=bool(
                isinstance(self._last_tip_status, dict)
                and self._last_tip_status.get("hardware_truth_level") == "hardware_query"
            ),
            physical_effect_verified=False,
            hardware_tip_status=self._last_tip_status,
            hardware_truth_level="hardware_query",
        )

    def aspirate(
        self,
        command: PipetteAspirateCommand,
        *,
        wait_for_completion: bool = True,
        verify_tip: bool = True,
    ) -> dict[str, Any]:
        self._require_initialized()
        driver = self._get_driver()
        tip_status = self._require_tip_loaded(driver, query_hardware=verify_tip)
        driver_result = self._assert_driver_result(
            "aspirate",
            driver.aspirate(
                command.volume_ul,
                tip_pressure_profile=command.pressure_profile,
                wait_for_completion=wait_for_completion,
            ),
        )
        self._pressure_profile = command.pressure_profile
        self._last_command = "aspirate"
        evidence = self._driver_evidence(driver_result)
        state = (
            self.apply_completed_effect("aspirate", {"volume_ul": float(command.volume_ul)})
            if evidence["completion_verified"]
            else {"state_reconciled": False, "state_reconciliation_source": None}
        )
        return self._status_payload(
            command="aspirate",
            requested=command.to_payload(),
            volume_ul=float(command.volume_ul),
            driver_result=driver_result,
            **evidence,
            hardware_precondition_verified=bool(
                tip_status and tip_status.get("hardware_truth_level") == "hardware_query"
            ),
            hardware_postcondition_verified=False,
            **state,
            physical_effect_verified=False,
            hardware_tip_status=tip_status,
            hardware_truth_level="acknowledged_command_with_tip_readback",
        )

    def dispense(
        self,
        command: PipetteDispenseCommand,
        *,
        wait_for_completion: bool = True,
        verify_tip: bool = True,
    ) -> dict[str, Any]:
        self._require_initialized()
        driver = self._get_driver()
        tip_status = self._require_tip_loaded(driver, query_hardware=verify_tip)
        # Both recovered collection Dispense overloads call the child with
        # host-accounting type 0. The child still transmits literal D...,1R.
        dispense_type = 0
        driver_result = self._assert_driver_result(
            "dispense",
            driver.dispense(
                command.volume_ul,
                tip_pressure_profile=command.pressure_profile,
                blow_out=command.blow_out,
                dispense_type=dispense_type,
                wait_for_completion=wait_for_completion,
            ),
        )
        self._pressure_profile = command.pressure_profile
        self._last_command = "dispense"
        evidence = self._driver_evidence(driver_result)
        state = (
            self.apply_completed_effect(
                "dispense",
                {"volume_ul": float(command.volume_ul), "dispense_type": dispense_type},
            )
            if evidence["completion_verified"]
            else {"state_reconciled": False, "state_reconciliation_source": None}
        )
        return self._status_payload(
            command="dispense",
            requested=command.to_payload(),
            volume_ul=float(command.volume_ul),
            blow_out=bool(command.blow_out),
            dispense_type=dispense_type,
            driver_result=driver_result,
            **evidence,
            hardware_precondition_verified=bool(
                tip_status and tip_status.get("hardware_truth_level") == "hardware_query"
            ),
            hardware_postcondition_verified=False,
            **state,
            physical_effect_verified=False,
            hardware_tip_status=tip_status,
            hardware_truth_level="acknowledged_command_with_tip_readback",
        )

    def mix(
        self,
        command: PipetteMixCommand,
        *,
        wait_for_completion: bool = True,
        verify_tip: bool = True,
    ) -> dict[str, Any]:
        self._require_initialized()
        driver = self._get_driver()
        # OEM ClassPipette.Mix does not issue QueryTipStatus.
        tip_status = None
        mix_fn = getattr(driver, "mix", None)
        if not callable(mix_fn):
            raise PipetteCommandError(
                "OEM composite Mix implementation is unavailable on the pipette driver.",
                details={"required_driver_method": "mix"},
            )
        driver_result = self._assert_driver_result(
            "mix",
            mix_fn(command.volume_ul, command.cycles, tip_pressure_profile=command.pressure_profile, wait_for_completion=wait_for_completion),
        )
        self._pressure_profile = command.pressure_profile
        self._last_command = "mix"
        evidence = self._driver_evidence(driver_result)
        state = (
            self.apply_completed_effect("mix", {"volume_ul": float(command.volume_ul)})
            if evidence["completion_verified"]
            else {"state_reconciled": False, "state_reconciliation_source": None}
        )
        cycle_results = driver_result.get("cycles", [])
        return self._status_payload(
            command="mix",
            requested=command.to_payload(),
            cycles=int(command.cycles),
            volume_ul=float(command.volume_ul),
            cycle_results=cycle_results,
            driver_result=driver_result,
            **evidence,
            hardware_precondition_verified=bool(
                tip_status and tip_status.get("hardware_truth_level") == "hardware_query"
            ),
            hardware_postcondition_verified=False,
            **state,
            physical_effect_verified=False,
            hardware_tip_status=tip_status,
            hardware_truth_level="acknowledged_command_with_tip_readback",
        )

    def dispense_all(
        self,
        *,
        wait_for_completion: bool = True,
        verify_tip: bool = True,
    ) -> dict[str, Any]:
        self._require_initialized()
        driver = self._get_driver()
        tip_status = self._require_tip_loaded(driver, query_hardware=verify_tip)
        result = self._assert_driver_result("dispense_all", driver.dispense_all(wait_for_completion=wait_for_completion))
        previous_level = self._liquid_level_ul
        evidence = self._driver_evidence(result)
        state = (
            self.apply_completed_effect("dispense_all", {})
            if evidence["completion_verified"]
            else {"state_reconciled": False, "state_reconciliation_source": None}
        )
        self._last_command = "dispense_all"
        return self._status_payload(
            command="dispense_all",
            previous_liquid_level_ul=previous_level,
            requested_wire_command="A0R",
            driver_result=result,
            **evidence,
            hardware_precondition_verified=bool(
                tip_status and tip_status.get("hardware_truth_level") == "hardware_query"
            ),
            hardware_postcondition_verified=False,
            **state,
            physical_effect_verified=False,
            hardware_tip_status=tip_status,
            hardware_truth_level="acknowledged_command_with_tip_readback",
        )

    def aspirate_air(self, volume_ul: float, *, air_type: int = 1, front_air: bool = True, wait_for_completion: bool = True) -> dict[str, Any]:
        self._require_initialized()
        driver = self._get_driver()
        result = self._assert_driver_result(
            "aspirate_air",
            driver.aspirate_air(float(volume_ul), air_type=int(air_type), wait_for_completion=wait_for_completion),
        )
        volume = float(volume_ul)
        evidence = self._driver_evidence(result)
        state = (
            self.apply_completed_effect(
                "aspirate_air",
                {"volume_ul": volume, "front_air": bool(front_air)},
            )
            if evidence["completion_verified"]
            else {"state_reconciled": False, "state_reconciliation_source": None}
        )
        self._last_command = "aspirate_air"
        return self._status_payload(
            command="aspirate_air",
            volume_ul=float(volume_ul),
            front_air=bool(front_air),
            driver_result=result,
            **evidence,
            hardware_postcondition_verified=False,
            **state,
            physical_effect_verified=False,
            hardware_tip_status=None,
            hardware_truth_level="controller_acknowledged",
        )

    def dispense_air(self, volume_ul: float, *, dispense_type: int = 0, wait_for_completion: bool = True) -> dict[str, Any]:
        self._require_initialized()
        selected_type = int(dispense_type)
        if selected_type not in {0, 1, 2}:
            raise PipetteCommandError("OEM dispense type must be 0, 1, or 2.", details={"dispense_type": selected_type})
        driver = self._get_driver()
        tip_status = self._require_tip_loaded(driver)
        result = self._assert_driver_result(
            "dispense_air",
            driver.dispense_air(float(volume_ul), dispense_type=selected_type, wait_for_completion=wait_for_completion),
        )
        volume = float(volume_ul)
        evidence = self._driver_evidence(result)
        state = (
            self.apply_completed_effect(
                "dispense_air",
                {"volume_ul": volume, "dispense_type": selected_type},
            )
            if evidence["completion_verified"]
            else {"state_reconciled": False, "state_reconciliation_source": None}
        )
        self._last_command = "dispense_air"
        return self._status_payload(
            command="dispense_air",
            volume_ul=float(volume_ul),
            dispense_type=selected_type,
            driver_result=result,
            **evidence,
            hardware_precondition_verified=bool(
                tip_status and tip_status.get("hardware_truth_level") == "hardware_query"
            ),
            hardware_postcondition_verified=False,
            **state,
            physical_effect_verified=False,
            hardware_tip_status=tip_status,
            hardware_truth_level="acknowledged_command_with_tip_readback",
        )

    def current_completion_owner_token(self) -> str | None:
        driver = self._get_driver()
        current = getattr(driver, "current_pipette_completion_owner_token", None)
        if callable(current):
            token = current()
        else:
            token = getattr(driver, "_pipette_completion_owner_token", None)
        return token if isinstance(token, str) and token else None

    def wait_for_completion(
        self,
        timeout_s: float,
        *,
        owner_token: str | None = None,
    ) -> dict[str, Any]:
        driver = self._get_driver()
        wait_fn = getattr(driver, "wait_pipette_command_completion", None)
        if not callable(wait_fn):
            return {"ok": False, "outcome": "completion_wait_unavailable", "channel": self._pipette_id}
        try:
            result = wait_fn(float(timeout_s), owner_token=owner_token)
        except TypeError:
            if owner_token is not None:
                raise
            result = wait_fn(float(timeout_s))
        if isinstance(result, dict):
            self._last_transaction = result
            return result
        return {"ok": False, "outcome": "invalid_completion_result", "result": repr(result), "channel": self._pipette_id}

    def terminate(
        self,
        command: PipetteTerminateCommand | None = None,
        *,
        wait_for_completion: bool = True,
    ) -> dict[str, Any]:
        del command
        driver = self._get_driver()
        result = self._assert_driver_result(
            "terminate",
            driver.terminate_pipette(wait_for_completion=wait_for_completion),
        )
        self._last_command = "terminate"
        return self._status_payload(
            command="terminate",
            driver_result=result,
            initialized=self._initialized,
            **self._driver_evidence(result),
            physical_effect_verified=False,
            hardware_truth_level="controller_acknowledged",
        )

    def set_top_speed(self, velocity: float, *, wait_for_completion: bool = True) -> dict[str, Any]:
        driver = self._get_driver()
        value = float(velocity)
        try:
            raw = driver.set_top_speed(value, wait_for_completion=wait_for_completion)
        except TypeError:
            if wait_for_completion is not True:
                raise
            raw = driver.set_top_speed(value)
        result = self._assert_driver_result("set_top_speed", raw)
        self._top_speed = float(value)
        self._last_command = "set_top_speed"
        return self._status_payload(
            command="set_top_speed",
            requested={"velocity": value},
            effective={"velocity": value},
            driver_result=result,
            **self._driver_evidence(result),
            physical_effect_verified=False,
            hardware_truth_level="controller_acknowledged",
        )

    def heartbeat(self, command: PipetteHeartbeatCommand) -> dict[str, Any]:
        driver = self._get_driver()
        result = self._assert_driver_result("heartbeat", driver.heartbeat(bool(command.enabled)))
        self._last_command = "heartbeat_enable" if command.enabled else "heartbeat_disable"
        return self._status_payload(
            command=self._last_command,
            requested=command.to_payload(),
            driver_result=result,
            **self._driver_evidence(result),
            physical_effect_verified=False,
            hardware_truth_level="controller_acknowledged",
        )

    def query_pressure(self) -> dict[str, Any]:
        result = self._get_driver().query_pressure()
        self._last_pressure = result if isinstance(result, dict) else None
        semantic_ok = bool(
            isinstance(result, dict)
            and result.get("ok") is True
            and result.get("semantic_ok") is True
            and result.get("pressure") is not None
        )
        return {
            "ok": semantic_ok,
            "semantic_ok": semantic_ok,
            "pressure": result.get("pressure") if isinstance(result, dict) else None,
            "result": result,
            "hardware_truth_level": "hardware_query",
        }

    def query_firmware(self, number: int = 1) -> dict[str, Any]:
        result = self._get_driver().query_firmware(int(number))
        self._last_transaction = result
        return {"ok": bool(isinstance(result, dict) and result.get("ok")), "query": f"&{int(number)}", "result": result, "hardware_truth_level": "hardware_query"}

    def query_error_log(self, command: PipetteErrorLogCommand | None = None) -> dict[str, Any]:
        selected = command or PipetteErrorLogCommand()
        result = self._get_driver().query_error_log(selected.raw_byte)
        self._last_transaction = result
        return {"ok": bool(isinstance(result, dict) and result.get("ok")), "query": "Q:<raw-byte>1", "requested": selected.to_payload(), "result": result, "hardware_truth_level": "hardware_query"}

    def execute_diagnoses(self, command: PipetteDiagnosticCommand, *, wait_for_completion: bool = True) -> dict[str, Any]:
        result = self._get_driver().execute_diagnoses(command.number, wait_for_completion=wait_for_completion)
        self._last_transaction = result
        return {"ok": bool(isinstance(result, dict) and result.get("ok")), "requested": command.to_payload(), "result": result, "hardware_truth_level": "hardware_query"}

    def get_data(self, query: str) -> dict[str, Any]:
        result = self._get_driver().get_data(query)
        self._last_transaction = result
        semantic_ok = bool(
            isinstance(result, dict)
            and result.get("ok") is True
            and result.get("semantic_ok") is True
        )
        return {"ok": semantic_ok, "semantic_ok": semantic_ok, "query": str(query), "result": result, "hardware_truth_level": "hardware_query"}

    def get_all_data(
        self,
        queries: list[str] | tuple[str, ...],
        *,
        wake_if_needed: bool = True,
    ) -> dict[str, Any]:
        driver = self._get_driver()
        wake = None
        if wake_if_needed and not self._initialized:
            wake_byte = 0x20 | int(self._pipette_id)
            wake = driver._send_packet(
                0x080,
                [wake_byte, wake_byte],
                command_name="pipette_wake_address",
            )
        results = [driver.get_data(query, wake_if_needed=False) for query in queries]
        self._last_transaction = {"wake": wake, "results": results}
        semantic_ok = bool(
            len(results) == len(queries)
            and all(
                isinstance(result, dict)
                and result.get("ok") is True
                and result.get("semantic_ok") is True
                and result.get("query") == str(query)
                for query, result in zip(queries, results)
            )
        )
        return {
            "ok": semantic_ok,
            "semantic_ok": semantic_ok,
            "wake": wake,
            "queries": list(queries),
            "results": results,
            "wake_count": 1 if wake is not None else 0,
            "wake_if_needed": bool(wake_if_needed),
            "oem_source_anchor": "ClassPipette.getData:211-261",
        }

    def start_fluid_detection(self, *, wait_for_completion: bool = True) -> dict[str, Any]:
        result = self._assert_driver_result(
            "start_fluid_detection",
            self._get_driver().start_fluid_detection(wait_for_completion=wait_for_completion),
        )
        self._last_command = "start_fluid_detection"
        return self._status_payload(
            command="start_fluid_detection",
            driver_result=result,
            **self._driver_evidence(result),
            physical_effect_verified=False,
            hardware_truth_level="controller_acknowledged",
        )

    def close(self) -> None:
        driver = self._driver
        self._driver = None
        if driver is None:
            return
        close_fn = getattr(driver, "close", None)
        if callable(close_fn):
            close_fn()


class FourPipetteTransport:
    """Production ClassPipetteCollection-shaped owner for channels 0..3."""

    CHANNELS = (0, 1, 2, 3)
    TIP_STATUS_FRESHNESS_S = 5.0
    OEM_DATA_QUERIES = (
        "?40", "?41", "?42", "?44", "?45", "?47",
        "?51", "?52", "?53", "?54", "?55", "?58", "?59",
    )

    def __init__(
        self,
        transports: list[CanPipetteTransport],
        *,
        sleep: Callable[[float], None] = time.sleep,
        liquid_mutation_enabled: bool = False,
        error_callback: Callable[[int, int], None] | None = None,
        forceabort: Callable[[], bool] | None = None,
    ) -> None:
        if len(transports) != 4:
            raise ValueError("OEM production pipette collection requires exactly four channels")
        self._transports = list(transports)
        self._sleep = sleep
        self._transaction_lock = threading.RLock()
        self._interrupt_epoch = 0
        self._last_group_transaction: dict[str, Any] | None = None
        self._tip_type = 201  # OEM enum value UNKNOWN
        self._tip_location = -1
        self._allow_to_stop = True
        self._forceabort = forceabort or (lambda: False)
        self._liquid_mutation_enabled = bool(liquid_mutation_enabled)
        self._fluid_detection_timestamps: dict[int, float | None] = {channel: None for channel in self.CHANNELS}
        self._last_error: dict[str, Any] | None = None
        self._error_callback = error_callback
        if callable(error_callback):
            for transport in self._transports:
                transport._error_callback = self._record_pipette_error

    def _record_pipette_error(self, channel: int, error_code: int) -> None:
        self._last_error = {
            "channel": int(channel),
            "error_code": int(error_code),
            "source": "ClassPipetteCollection.handlePipetteMessage",
        }
        if callable(self._error_callback):
            self._error_callback(int(channel), int(error_code))

    def _channel_status(self, channel: int, transport: CanPipetteTransport) -> dict[str, Any]:
        driver = transport._get_driver()
        tip = transport._safe_query_tip_status(driver)
        pressure = transport._safe_query_pressure(driver)
        return {
            **transport._status_payload(
                hardware_tip_status=tip,
                hardware_pressure=pressure,
                hardware_truth_level="hardware_query" if any(
                    isinstance(row, dict) and row.get("ok") for row in (tip, pressure)
                ) else "unavailable",
            ),
            "channel": int(channel),
        }

    def get_status(self) -> dict[str, Any]:
        rows = [
            {
                **transport._status_payload(hardware_truth_level="cached_transport_state"),
                "channel": index,
                "available": bool(transport._transport_details.get("shared_bioxp_usb_runtime"))
                if transport._transport_name == "novo_usb_can" else True,
            }
            for index, transport in enumerate(self._transports)
        ]
        return {
            "ok": all(bool(row.get("available")) for row in rows),
            "transport": "novo_usb_can",
            "channels": rows,
            "channel_count": 4,
            "group_status_spacing_ms": 30,
            "live_query_performed": False,
            "last_group_transaction": self._last_group_transaction,
            "liquid_mutation_enabled": self._liquid_mutation_enabled,
            "tip_type": self._tip_type,
            "tip_location": self._tip_location,
            "allow_to_stop": self._allow_to_stop,
            "fluid_detection_timestamps": dict(self._fluid_detection_timestamps),
            "last_error": self._last_error,
            "physical_effect_verified": False,
        }

    def readback_all(self, *, include_data: bool = False) -> dict[str, Any]:
        """Actively query all four pipettes without sending a command-family mutation."""
        rows: list[dict[str, Any]] = []
        with self._transaction_lock:
            for channel, transport in enumerate(self._transports):
                driver = transport._get_driver()
                firmware = driver.query_firmware(1)
                status = driver.query_status()
                tip = transport._safe_query_tip_status(driver, required=True)
                pressure = None
                if (
                    isinstance(tip, Mapping)
                    and tip.get("ok") is True
                    and tip.get("semantic_ok") is True
                    and tip.get("tip_loaded") is True
                ):
                    pressure = driver.query_pressure()
                    if isinstance(pressure, Mapping):
                        transport._last_pressure = dict(pressure)
                data = transport.get_all_data(
                    list(self.OEM_DATA_QUERIES),
                    wake_if_needed=False,
                ) if include_data else None
                semantic_ok = bool(
                    isinstance(firmware, Mapping)
                    and firmware.get("ok") is True
                    and firmware.get("semantic_ok") is True
                    and isinstance(status, Mapping)
                    and status.get("ok") is True
                    and status.get("semantic_ok") is True
                    and isinstance(tip, Mapping)
                    and tip.get("ok") is True
                    and tip.get("semantic_ok") is True
                    and (
                        pressure is None
                        or (
                            isinstance(pressure, Mapping)
                            and pressure.get("ok") is True
                            and pressure.get("semantic_ok") is True
                        )
                    )
                    and (
                        data is None
                        or (
                            isinstance(data, Mapping)
                            and data.get("ok") is True
                            and data.get("semantic_ok") is True
                        )
                    )
                )
                rows.append({
                    "channel": channel,
                    "semantic_ok": semantic_ok,
                    "firmware": firmware,
                    "status": status,
                    "tip": tip,
                    "pressure": pressure,
                    "data": data,
                })
                self._sleep(0.030)
        ok = bool(len(rows) == 4 and all(row["semantic_ok"] for row in rows))
        return {
            "ok": ok,
            "semantic_ok": ok,
            "available": ok,
            "channel_count": 4,
            "channels_constructed_unconditionally": [0, 1, 2, 3],
            "channels": rows,
            "include_data": bool(include_data),
            "live_query_performed": True,
            "truth_source": "live_hardware_queries",
            "hardware_truth_level": "hardware_query",
            "delivery_verified": False,
            "controller_acknowledged": False,
            "completion_verified": False,
            "hardware_postcondition_verified": False,
            "physical_effect_verified": False,
            "oem_source_anchor": "ClassPipetteCollection constructor/readback; ClassPipette QueryFirmware/Q1/?31/?57/getData",
        }

    def _run_group_cycle(self, command: PipetteInitCommand, *, cycle: str) -> dict[str, Any]:
        self._require_physical_command_admission(cycle)
        sends: list[dict[str, Any]] = []
        for channel, transport in enumerate(self._transports):
            driver = transport._get_driver()
            if transport._initialized:
                # ClassPipette.initiate(false) skips the wake frame once initialized,
                # but still waits 100 ms and transmits WR.
                self._sleep(0.100)
                driver_result = transport._assert_driver_result(
                    "initialize",
                    driver.pipette_initiate_group(),
                )
                result = {
                    **transport._status_payload(
                        command="initialize",
                        driver_result=driver_result,
                        requested=command.to_payload(),
                        hardware_truth_level="acknowledged_command",
                    ),
                    "driver_result": driver_result,
                }
            else:
                result = transport.initialize(command)
            sends.append({"channel": channel, "result": result})
            if not result.get("driver_result", {}).get("immediate_ack_received"):
                for registered in range(channel + 1):
                    self._transports[registered]._get_driver().wait_pipette_initialization_completion(0.0)
                return {
                    "ok": False,
                    "cycle": cycle,
                    "outcome": "invalid_or_missing_immediate_group_ack",
                    "sends": sends,
                    "completion_timeout_ms": 10_000,
                }

        # Exact collection-level waitforcompletion("Reinitialize pipette", 10000):
        # one deadline for all four completions, never four serial extensions.
        deadline = time.monotonic() + 10.0
        completions: list[dict[str, Any]] = []
        for channel, transport in enumerate(self._transports):
            remaining = max(0.0, deadline - time.monotonic())
            result = transport._get_driver().wait_pipette_initialization_completion(remaining)
            completions.append({"channel": channel, "result": result})
            transport._initialized = bool(result.get("ok"))
        if not all(row["result"].get("ok") for row in completions):
            return {
                "ok": False,
                "cycle": cycle,
                "outcome": "group_completion_timeout_or_error",
                "sends": sends,
                "delayed_completions": completions,
                "completion_timeout_ms": 10_000,
            }

        first_driver = self._transports[0]._get_driver()
        router = getattr(getattr(first_driver, "bus", None), "router", None)
        pressure_epoch = router.begin_pressure_epoch() if router is not None else None
        stream_on = [
            {"channel": channel, "result": transport._get_driver().enable_pressure_stream(True)}
            for channel, transport in enumerate(self._transports)
        ]
        self._sleep(1.000)
        stream_off = [
            {"channel": channel, "result": transport._get_driver().enable_pressure_stream(False)}
            for channel, transport in enumerate(self._transports)
        ]
        pressure_offset_evidence = (
            router.calculate_pressure_offset_evidence() if router is not None else {}
        )
        pressure_offsets = {
            channel: row["offset"]
            for channel, row in pressure_offset_evidence.items()
            if isinstance(row, Mapping) and row.get("valid") is True
        }
        stream_on_ok = all(row["result"].get("ok") for row in stream_on)
        stream_off_ok = all(row["result"].get("ok") for row in stream_off)
        stream_ok = stream_on_ok and stream_off_ok
        delivery_verified = all(
            row["result"].get("delivery_verified") is True
            for row in sends + stream_on + stream_off
        )
        controller_acknowledged = all(
            row["result"].get("controller_acknowledged") is True
            for row in sends + stream_on + stream_off
        )
        completion_verified = all(
            row["result"].get("ok") is True
            for row in completions
        )
        return {
            "ok": bool(stream_ok),
            "cycle": cycle,
            "outcome": "completion" if stream_ok else "pressure_stream_command_failed",
            "sends": sends,
            "delayed_completions": completions,
            "completion_timeout_ms": 10_000,
            "pressure_stream": {
                "on": stream_on,
                "wait_ms": 1_000,
                "off": stream_off,
                "selected_channels": list(self.CHANNELS),
                "terminal_state": "stopped" if stream_off_ok else "stop_unconfirmed",
            },
            "pressure_epoch": pressure_epoch,
            "pressure_offsets": pressure_offsets,
            "pressure_offset_evidence": pressure_offset_evidence,
            "pressure_offsets_valid": bool(
                len(pressure_offset_evidence) == 4
                and all(row.get("valid") is True for row in pressure_offset_evidence.values())
            ),
            "delivery_verified": delivery_verified,
            "controller_acknowledged": controller_acknowledged,
            "completion_verified": completion_verified,
            "pressure_offset_order": "new_epoch_stream_on_1000ms_stream_off_calculate",
        }

    def _query_condition(self) -> list[dict[str, Any]]:
        return [
            {"channel": channel, "result": transport._get_driver().query_firmware(1)}
            for channel, transport in enumerate(self._transports)
        ]

    def _query_status(self) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        for channel, transport in enumerate(self._transports):
            rows.append({"channel": channel, "result": transport._get_driver().query_status()})
            self._sleep(0.030)
        self._sleep(0.001)
        return rows

    def initiate_group_once_for_oem_initialize_motion(self, *, cycle: str) -> dict[str, Any]:
        """Execute exactly one `ClassPipetteCollection.initiateGroup()` cycle.

        `ControlLib.initializeMotion` owns the initial/retry branch.  Keeping that
        branch outside this primitive prevents the transport from silently doing
        both attempts inside one command and gives the durable lifecycle one
        receipt for each direct OEM call.
        """
        selected = str(cycle).strip()
        if selected not in {"initializeMotion.initial", "initializeMotion.retry"}:
            raise ValueError("cycle must be initializeMotion.initial or initializeMotion.retry")
        with self._transaction_lock:
            result = self._run_group_cycle(PipetteInitCommand(), cycle=selected)
            return {
                **dict(result),
                "oem_source_anchor": "ClassPipetteCollection.initiateGroup:677-693; ControlLib.initializeMotion:8832,8835",
                "single_group_cycle": True,
                "retry_selected_by_transport": False,
            }

    def checked_pipette_status_for_oem_initialize_motion(self, *, attempt: str) -> dict[str, Any]:
        """Exact four-channel `checkedPipetteStatus()` query and error gate."""
        selected = str(attempt).strip()
        if selected not in {"initial", "retry"}:
            raise ValueError("attempt must be initial or retry")
        with self._transaction_lock:
            rows = self._query_status()
        exact_rows: list[dict[str, Any]] = []
        for row in rows:
            result = row.get("result") if isinstance(row, Mapping) else None
            error_code = result.get("oem_error_code") if isinstance(result, Mapping) else None
            exact_rows.append(
                {
                    "channel": row.get("channel") if isinstance(row, Mapping) else None,
                    "reply_received": result.get("reply_received") if isinstance(result, Mapping) else False,
                    "semantic_ok": result.get("semantic_ok") if isinstance(result, Mapping) else False,
                    "error_code": error_code,
                    "error_free": result.get("oem_error_free") if isinstance(result, Mapping) else False,
                    "result": result,
                }
            )
        ok = bool(
            len(exact_rows) == 4
            and all(
                row["reply_received"] is True
                and row["semantic_ok"] is True
                and type(row["error_code"]) is int
                and row["error_code"] == 0
                and row["error_free"] is True
                for row in exact_rows
            )
        )
        return {
            "ok": ok,
            "attempt": selected,
            "channels": exact_rows,
            "channel_count": len(exact_rows),
            "hardware_truth_level": "hardware_query",
            "query_spacing_ms": 30,
            "post_query_settle_ms": 1,
            "oem_source_anchor": "ClassPipetteCollection.checkedPipetteStatus:726-748; ControlLib.initializeMotion:8833,8836",
            "failure": None if ok else "checkedPipetteStatus_returned_false",
        }

    def initialize(self, command: PipetteInitCommand) -> dict[str, Any]:
        started = time.monotonic()
        initial_group = self._run_group_cycle(command, cycle="constructor_initiateGroup")
        if not initial_group.get("ok"):
            self._last_group_transaction = {
                "ok": False,
                "outcome": "initial_group_cycle_failed",
                "channels": initial_group.get("sends", []),
                "initial_group": initial_group,
                "pressure_stream": dict(initial_group.get("pressure_stream") or {}),
                "pressure_epoch": initial_group.get("pressure_epoch"),
                "pressure_offsets": dict(initial_group.get("pressure_offsets") or {}),
                "pressure_offset_evidence": dict(initial_group.get("pressure_offset_evidence") or {}),
                "pressure_offsets_valid": initial_group.get("pressure_offsets_valid") is True,
                "pressure_offset_order": initial_group.get("pressure_offset_order"),
                "group_wait_ms": 10_000,
                "liquid_mutation_enabled": self._liquid_mutation_enabled,
            }
            return dict(self._last_group_transaction)

        condition = self._query_condition()
        condition_ok = all(row["result"].get("ok") for row in condition)
        first_status = self._query_status()
        first_status_ok = all(row["result"].get("ok") for row in first_status)
        retry_group = None
        retry_status = None
        if condition_ok and not first_status_ok:
            retry_group = self._run_group_cycle(command, cycle="single_conditional_status_retry")
            if retry_group.get("ok"):
                retry_status = self._query_status()
        final_status = retry_status if retry_status is not None else first_status
        final_status_ok = all(row["result"].get("ok") for row in final_status)
        ok = bool(condition_ok and final_status_ok and (retry_group is None or retry_group.get("ok")))
        pressure_group = (
            retry_group
            if isinstance(retry_group, Mapping)
            and (
                retry_group.get("pressure_stream")
                or retry_group.get("pressure_offset_evidence")
                or retry_group.get("pressure_epoch") is not None
            )
            else initial_group
        )
        self._last_group_transaction = {
            "ok": ok,
            "outcome": "completion" if ok else "condition_or_status_failed",
            "channels": initial_group.get("sends", []),
            "initial_group": initial_group,
            "condition_readback": condition,
            "status_readback_first": first_status,
            "single_conditional_retry_performed": retry_group is not None,
            "retry_group": retry_group,
            "status_readback_retry": retry_status,
            "status_readback_final": final_status,
            "pressure_stream": dict(pressure_group.get("pressure_stream") or {}),
            "pressure_epoch": pressure_group.get("pressure_epoch"),
            "pressure_offsets": dict(pressure_group.get("pressure_offsets") or {}),
            "pressure_offset_evidence": dict(pressure_group.get("pressure_offset_evidence") or {}),
            "pressure_offsets_valid": pressure_group.get("pressure_offsets_valid") is True,
            "pressure_offset_order": pressure_group.get("pressure_offset_order"),
            "group_wait_ms": 10_000,
            "pipette_transaction_timeout_ms": 60_000,
            "elapsed_ms": int(round((time.monotonic() - started) * 1000.0)),
            "liquid_mutation_enabled": self._liquid_mutation_enabled,
        }
        return dict(self._last_group_transaction)

    def _selected_channels(self, channels: list[int] | tuple[int, ...] | None = None) -> list[int]:
        selected = list(self.CHANNELS if channels is None else channels)
        if not selected or any(type(channel) is not int or channel not in self.CHANNELS for channel in selected):
            raise ValueError("channels must be a non-empty subset of integer channel IDs 0..3")
        if len(set(selected)) != len(selected):
            raise ValueError("channels must not contain duplicates")
        return selected

    def _tip_eligibility(
        self,
        channels: list[int] | tuple[int, ...] | None = None,
    ) -> tuple[list[int], dict[int, dict[str, Any]]]:
        selected = self._selected_channels(channels)
        eligible: list[int] = []
        ledger: dict[int, dict[str, Any]] = {}
        now = time.time()
        for channel in selected:
            transport = self._transports[channel]
            status = transport._last_tip_status
            observed_at = status.get("observed_at") if isinstance(status, Mapping) else None
            generation = status.get("reader_generation") if isinstance(status, Mapping) else None
            driver = getattr(transport, "__dict__", {}).get("_driver")
            router = getattr(getattr(driver, "bus", None), "router", None)
            current_generation = getattr(router, "reader_generation", None)
            if not isinstance(current_generation, int):
                current_generation = getattr(transport, "_reader_generation", None)
            if not isinstance(status, Mapping):
                reason = "missing_tip_observation"
            elif status.get("ok") is not True or status.get("semantic_ok") is not True:
                reason = "invalid_tip_observation"
            elif status.get("hardware_truth_level") != "hardware_query":
                reason = "non_hardware_tip_observation"
            elif not isinstance(observed_at, (int, float)) or now - float(observed_at) > self.TIP_STATUS_FRESHNESS_S:
                reason = "stale_tip_observation"
            elif not isinstance(generation, int):
                reason = "missing_reader_generation"
            elif not isinstance(current_generation, int):
                reason = "missing_current_reader_generation"
            elif generation != current_generation:
                reason = "reader_generation_mismatch"
            elif status.get("tip_loaded") is not True:
                reason = "tip_not_loaded"
            else:
                reason = "eligible"
                eligible.append(channel)
            ledger[channel] = {
                "channel": channel,
                "eligible": reason == "eligible",
                "reason": reason,
                "observed_at": observed_at,
                "reader_generation": generation,
                "current_reader_generation": current_generation,
                "freshness_limit_s": self.TIP_STATUS_FRESHNESS_S,
            }
        return eligible, ledger

    def _cached_tip_channels(
        self,
        channels: list[int] | tuple[int, ...] | None = None,
    ) -> list[int]:
        eligible, _ledger = self._tip_eligibility(channels)
        return eligible

    def reinitialize_pipette(self) -> dict[str, Any]:
        """Separate OEM reinitializePipette path: all WR sends, then one 10 s wait."""
        self._require_physical_command_admission("reinitialize")
        with self._transaction_lock:
            sends: list[dict[str, Any]] = []
            for channel, transport in enumerate(self._transports):
                driver = transport._get_driver()
                result = transport._assert_driver_result("reinitialize_pipette", driver.pipette_initiate_group())
                sends.append({"channel": channel, "result": result})

            deadline = time.monotonic() + 10.0
            completions: list[dict[str, Any]] = []
            for channel, transport in enumerate(self._transports):
                remaining = max(0.0, deadline - time.monotonic())
                completion = transport._get_driver().wait_pipette_initialization_completion(remaining)
                completions.append({"channel": channel, "completion": completion})
                transport._initialized = bool(completion.get("ok"))

            status = self._query_status()
            process_status_codes = [row["result"].get("oem_process_error_code") for row in status]
            error_codes = [row["result"].get("oem_error_code") for row in status]
            ok = bool(
                all(row.get("result", {}).get("ok") for row in sends)
                and all(row.get("completion", {}).get("ok") for row in completions)
                and all(code == 32 for code in process_status_codes)
                and all(code == 0 for code in error_codes)
            )
            self._tip_type = 201
            self._last_group_transaction = {
                "ok": ok,
                "outcome": "reinitialized" if ok else "reinitialize_status_not_all_0",
                "sends": sends,
                "completions": completions,
                "status": status,
                "process_status_codes": process_status_codes,
                "error_codes": error_codes,
                "tip_type": self._tip_type,
                "group_wait_ms": 10_000,
                "oem_source_anchor": "ClassPipetteCollection.reinitializePipette",
            }
            return dict(self._last_group_transaction)

    def checked_pipette_condition(self) -> dict[str, Any]:
        with self._transaction_lock:
            rows = self._query_condition()
        ok = bool(len(rows) == 4 and all(row.get("result", {}).get("ok") for row in rows))
        return {
            "ok": ok,
            "channels": rows,
            "channel_count": len(rows),
            "hardware_truth_level": "hardware_query",
            "null_reply_is_failure": True,
            "oem_source_anchor": "ClassPipetteCollection.checkedPipetteCondition: QueryFirmware(1)",
        }

    def checked_pipette_status(self) -> dict[str, Any]:
        return self.checked_pipette_status_for_oem_initialize_motion(attempt="initial")

    def read_pressure(self, channels: list[int] | None = None) -> dict[str, Any]:
        selected, eligibility = self._tip_eligibility(channels)
        rows = [{"channel": channel, "result": self._transports[channel].query_pressure()} for channel in selected]
        return {
            "ok": bool(selected) and all(
                row["result"].get("ok") is True and row["result"].get("semantic_ok") is True
                for row in rows
            ),
            "channels": rows,
            "channel_count": len(rows),
            "eligibility_source": "semantic_hardware_tip_readback",
            "eligibility": eligibility,
            "hardware_truth_level": "hardware_query",
            "oem_source_anchor": "ClassPipetteCollection.readPressure:248-258; ClassPipette.QueryPressure:622-628",
        }

    def set_top_speed(self, velocity: float, channels: list[int] | None = None) -> dict[str, Any]:
        self._require_physical_command_admission("set_top_speed")
        value = float(velocity)
        selected, eligibility = self._tip_eligibility(channels)
        if not selected:
            return {
                "ok": False,
                "outcome": "no_eligible_channels",
                "velocity": value,
                "channels": [],
                "eligibility": eligibility,
                "oem_source_anchor": "ClassPipetteCollection.SetTopSpeed",
            }
        if float(self._transports[0]._top_speed) == value:
            return {
                "ok": True,
                "outcome": "unchanged_channel_zero_speed",
                "velocity": value,
                "channels": selected,
                "eligibility": eligibility,
                "oem_source_anchor": "ClassPipetteCollection.SetTopSpeed:756-758",
            }
        with self._transaction_lock:
            rows = []
            for channel in selected:
                result = self._transports[channel].set_top_speed(value)
                rows.append({"channel": channel, "result": result})
                self._sleep(0.010)
            return {
                "ok": bool(rows) and all(row["result"].get("ok") for row in rows),
                "velocity": value,
                "channels": rows,
                "eligibility": eligibility,
                "oem_source_anchor": "ClassPipetteCollection.SetTopSpeed",
            }

    def query_firmware(self, number: int = 1, channels: list[int] | None = None) -> dict[str, Any]:
        selected = self._selected_channels(channels)
        rows = [{"channel": channel, "result": self._transports[channel].query_firmware(number)} for channel in selected]
        return {"ok": all(row["result"].get("ok") for row in rows), "channels": rows, "query": f"&{int(number)}", "hardware_truth_level": "hardware_query"}

    def query_error_log(self, command: PipetteErrorLogCommand | None = None, channels: list[int] | None = None) -> dict[str, Any]:
        selected = self._selected_channels(channels)
        rows = [{"channel": channel, "result": self._transports[channel].query_error_log(command)} for channel in selected]
        return {"ok": all(row["result"].get("ok") for row in rows), "channels": rows, "query": "Q:<raw-byte>1", "hardware_truth_level": "hardware_query"}

    def execute_diagnoses(self, command: PipetteDiagnosticCommand, channels: list[int] | None = None) -> dict[str, Any]:
        self._require_physical_command_admission("diagnoses")
        selected, eligibility = self._tip_eligibility(channels)
        if not selected:
            return {
                "ok": False,
                "outcome": "no_eligible_channels",
                "channels": [],
                "diagnoses": [],
                "eligibility": eligibility,
                "requested": command.to_payload(),
                "oem_source_anchor": "ClassPipetteCollection.diagnoses:261-283",
            }
        with self._transaction_lock:
            rows: list[dict[str, Any]] = []
            for position, channel in enumerate(selected):
                result = self._transports[channel].execute_diagnoses(command, wait_for_completion=False)
                rows.append({"channel": channel, "result": result})
                if position + 1 < len(selected):
                    self._sleep(0.001)
            deadline = time.monotonic() + 4.0
            diagnoses: list[dict[str, Any]] = []
            for row in rows:
                channel = int(row["channel"])
                completion = self._transports[channel].wait_for_completion(max(0.0, deadline - time.monotonic()))
                row["completion"] = completion
                state = completion.get("pipette_message_state", {}) if isinstance(completion, dict) else {}
                diagnoses.append({"channel": channel, "diagnosis": state.get("diagnosis"), "completion": completion})
            ok = bool(rows) and all(
                row["result"].get("ok") is True and row.get("completion", {}).get("ok") is True
                for row in rows
            )
            return {
                "ok": ok,
                "channels": rows,
                "diagnoses": diagnoses,
                "eligibility": eligibility,
                "requested": command.to_payload(),
                "timeout_ms": 4_000,
                "wait_policy": "shared_deadline_after_all_selected_diagnosis_sends",
                "oem_source_anchor": "ClassPipetteCollection.diagnoses:261-283",
            }

    def get_data(self, query: str | None = None, channels: list[int] | None = None) -> dict[str, Any]:
        selected = self._selected_channels(channels)
        queries = [str(query)] if query is not None else list(self.OEM_DATA_QUERIES)
        rows: list[dict[str, Any]] = []
        for channel in selected:
            transport = self._transports[channel]
            aggregate = None
            get_all = getattr(transport, "get_all_data", None)
            if query is None and callable(get_all):
                aggregate_result = get_all(queries)
                aggregate = aggregate_result if isinstance(aggregate_result, Mapping) else None
                results = list(aggregate.get("results", [])) if aggregate is not None else []
            else:
                results = [transport.get_data(item) for item in queries]
            rows.append({"channel": channel, "results": results, "aggregate": aggregate})
        expected_query_count = len(queries)
        semantic_ok = bool(
            len(rows) == len(selected)
            and len(rows) > 0
            and all(
                len(row["results"]) == expected_query_count
                and all(
                    result.get("ok") is True
                    and result.get("semantic_ok") is True
                    and result.get("query") == expected_query
                    for expected_query, result in zip(queries, row["results"])
                )
                and (
                    row["aggregate"] is None
                    or row["aggregate"].get("semantic_ok") is True
                )
                for row in rows
            )
        )
        return {
            "ok": semantic_ok,
            "semantic_ok": semantic_ok,
            "channels": rows,
            "query": None if query is None else str(query),
            "queries": queries,
            "expected_channel_count": len(selected),
            "expected_query_count": expected_query_count,
            "aggregate_oem_sweep": query is None,
            "hardware_truth_level": "hardware_query",
            "oem_source_anchor": "ClassPipette.getData:211-261",
        }

    @staticmethod
    def _completion_owner_token(transport: Any) -> str | None:
        current = getattr(transport, "current_completion_owner_token", None)
        if callable(current):
            token = current()
        else:
            driver_getter = getattr(transport, "_get_driver", None)
            driver = driver_getter() if callable(driver_getter) else None
            token = getattr(driver, "_pipette_completion_owner_token", None)
        return token if isinstance(token, str) and token else None

    @staticmethod
    def _wait_channel_completion(
        transport: Any,
        timeout_s: float,
        *,
        owner_token: str | None,
    ) -> dict[str, Any]:
        wait = getattr(transport, "wait_for_completion")
        try:
            return wait(float(timeout_s), owner_token=owner_token)
        except TypeError:
            if owner_token is not None:
                raise
            return wait(float(timeout_s))

    def terminate(self, command: PipetteTerminateCommand | None = None) -> dict[str, Any]:
        self._require_physical_command_admission("terminate")
        with self._transaction_lock:
            self._interrupt_epoch += 1
            interrupt_epoch = self._interrupt_epoch
            rows = []
            for channel, transport in enumerate(self._transports):
                result = transport.terminate(command, wait_for_completion=False)
                rows.append({
                    "channel": channel,
                    "result": result,
                    "completion_owner_token": self._completion_owner_token(transport),
                })
        deadline = time.monotonic() + 8.0
        for row in rows:
            channel = int(row["channel"])
            completion = self._wait_channel_completion(
                self._transports[channel],
                max(0.0, deadline - time.monotonic()),
                owner_token=row.get("completion_owner_token"),
            )
            row["completion"] = completion
        self._allow_to_stop = True
        ok = all(
            row["result"].get("ok") is True
            and row.get("completion", {}).get("ok") is True
            for row in rows
        )
        self._last_error = None if ok else {"operation": "terminate", "channels": rows}
        return {
            "ok": ok,
            "channels": rows,
            "timeout_ms": 8_000,
            "wait_policy": "shared_deadline_after_all_four_TR_sends",
            "interrupt_epoch": interrupt_epoch,
            "allow_to_stop": self._allow_to_stop,
            "physical_effect_verified": False,
            "oem_source_anchor": "ClassPipetteCollection.terminatecommands:1325-1334",
        }

    def heartbeat(self, command: PipetteHeartbeatCommand) -> dict[str, Any]:
        self._require_physical_command_admission("heartbeat")
        rows = [{"channel": channel, "result": transport.heartbeat(command)} for channel, transport in enumerate(self._transports)]
        return {"ok": all(row["result"].get("ok") for row in rows), "channels": rows, "requested": command.to_payload()}

    def disable_heartbeat(self) -> dict[str, Any]:
        return self.heartbeat(PipetteHeartbeatCommand(enabled=False))

    def _require_physical_command_admission(self, operation: str) -> None:
        if not self._liquid_mutation_enabled:
            raise PipetteCommandError(
                f"Pipette {operation} remains blocked by the accepted production safety envelope.",
                details={
                    "operation": operation,
                    "liquid_mutation_enabled": False,
                    "physical_command_admitted": False,
                    "channel_count": 4,
                    "physical_effect_verified": False,
                },
            )

    def _require_liquid_mutation(self, operation: str) -> None:
        self._require_physical_command_admission(operation)

    def _run_group_liquid_operation(
        self,
        operation: str,
        channels: list[int],
        callback: Callable[[int, CanPipetteTransport, bool], dict[str, Any]],
        *,
        timeout_ms: int,
        defer_completion: bool = True,
        pre_send_delay_s: float = 0.0,
        post_send_delay_s: float = 0.0,
        timeout_failure_sleep_s: float = 0.0,
        check_forceabort_after_wait: bool = False,
    ) -> dict[str, Any]:
        self._require_liquid_mutation(operation)
        self._allow_to_stop = False
        rows: list[dict[str, Any]] = []
        operation_interrupt_epoch = self._interrupt_epoch
        try:
            # Hold the transaction lock only while registering/sending the group.
            # Completion waits must remain interruptible by terminatecommands().
            with self._transaction_lock:
                operation_interrupt_epoch = self._interrupt_epoch
                for channel in channels:
                    if pre_send_delay_s:
                        self._sleep(float(pre_send_delay_s))
                    result = callback(channel, self._transports[channel], defer_completion)
                    rows.append({
                        "channel": channel,
                        "result": result,
                        "completion_owner_token": self._completion_owner_token(self._transports[channel]),
                    })
                    if post_send_delay_s:
                        self._sleep(float(post_send_delay_s))
        except Exception as exc:
            self._last_error = {"operation": operation, "error": repr(exc), "channels": rows}
            raise

        completion_rows: list[dict[str, Any]] = []
        if defer_completion:
            deadline = time.monotonic() + max(0.0, int(timeout_ms) / 1000.0)
            for row in rows:
                channel = int(row["channel"])
                remaining = max(0.0, deadline - time.monotonic())
                completion = self._wait_channel_completion(
                    self._transports[channel],
                    remaining,
                    owner_token=row.get("completion_owner_token"),
                )
                row["completion"] = completion
                completion_rows.append({"channel": channel, "result": completion})
                with self._transaction_lock:
                    interrupted = self._interrupt_epoch != operation_interrupt_epoch
                    row["interrupted_by_terminate"] = interrupted
                    if isinstance(completion, Mapping) and completion.get("ok") is True and not interrupted:
                        state = self._transports[channel].apply_completed_effect(operation, row["result"])
                        row["result"].update(state)
                        row["result"]["completion_verified"] = True
                    else:
                        row["result"]["completion_verified"] = False
                        row["result"]["state_reconciled"] = False
                        row["result"]["state_reconciliation_source"] = None
            if timeout_failure_sleep_s and any(
                row.get("completion", {}).get("ok") is not True for row in rows
            ):
                self._sleep(float(timeout_failure_sleep_s))
        if check_forceabort_after_wait and self._forceabort():
            raise PipetteCommandError("Stopped by user or force abort")

        with self._transaction_lock:
            interrupted_by_terminate = self._interrupt_epoch != operation_interrupt_epoch
        if interrupted_by_terminate:
            for row in rows:
                row["interrupted_by_terminate"] = True
                row["result"]["completion_verified"] = False
                row["result"]["state_reconciled"] = False
                row["result"]["state_reconciliation_source"] = None

        ok = not interrupted_by_terminate and all(
            row["result"].get("ok") is True
            and (not defer_completion or row.get("completion", {}).get("ok") is True)
            for row in rows
        )
        self._last_group_transaction = {
            "ok": ok,
            "operation": operation,
            "channels": rows,
            "channel_count": len(channels),
            "timeout_ms": int(timeout_ms),
            "allow_to_stop": self._allow_to_stop,
            "liquid_mutation_enabled": self._liquid_mutation_enabled,
            "delivery_verified": all(row["result"].get("delivery_verified") is True for row in rows),
            "controller_acknowledged": all(row["result"].get("controller_acknowledged") is True for row in rows),
            "completion_verified": bool(
                defer_completion
                and not interrupted_by_terminate
                and all(row.get("completion", {}).get("ok") is True for row in rows)
            ),
            "state_reconciled": all(row["result"].get("state_reconciled") is True for row in rows),
            "interrupted_by_terminate": interrupted_by_terminate,
            "operation_interrupt_epoch": operation_interrupt_epoch,
            "terminal_interrupt_epoch": self._interrupt_epoch,
            "physical_effect_verified": False,
            "send_order": list(channels),
            "completion_order": [row["channel"] for row in completion_rows],
            "wait_policy": "shared_deadline_after_all_selected_channel_sends" if defer_completion else "oem_composite_sequence_no_constituent_completion_waits",
        }
        return dict(self._last_group_transaction)

    @staticmethod
    def _oem_timeout_ms(volume_ul: float, speed: float, offset_ms: int) -> int:
        return 5 * int(float(volume_ul) / float(speed) * 1000.0) + int(offset_ms)

    @staticmethod
    def _oem_standard_dispense_timeout_ms(volume_ul: float, speed: float) -> int:
        return int(5.0 * float(volume_ul) / float(speed) * 1000.0) + 5_000

    @staticmethod
    def _oem_explicit_dispense_timeout_ms(volume_ul: float, speed: float) -> int:
        return 5 * int(float(volume_ul) / float(speed) * 1000.0) + 4_000

    def _tip_location_channels(self) -> list[int]:
        return list(self.CHANNELS) if self._tip_location == -1 else [self._tip_location]

    def _verify_standard_tip_selection(self, selected: list[int]) -> dict[str, Any]:
        before = self.query_tip_status_all()
        loaded = set(before["channels_with_tips"])
        if any(channel not in loaded for channel in selected):
            raise PipetteTipStateError(
                "Standard OEM liquid operation requires a hardware-confirmed loaded tip on every selected channel.",
                details={"selected_channels": selected, "loaded_channels": sorted(loaded)},
            )
        return before

    def _prepare_explicit_speed_overload(
        self,
        channels: tuple[int, ...] | list[int],
        speed: float,
    ) -> tuple[list[int], dict[str, Any]]:
        selected = self._selected_channels(list(channels))
        rows: list[dict[str, Any]] = []
        for channel in selected:
            transport = self._transports[channel]
            try:
                result = transport.set_top_speed(float(speed), wait_for_completion=False)
            except TypeError:
                result = transport.set_top_speed(float(speed))
            rows.append({
                "channel": channel,
                "result": result,
                "completion_owner_token": self._completion_owner_token(transport),
            })
            self._sleep(0.010)
        deadline = time.monotonic() + 7.0
        for row in rows:
            channel = int(row["channel"])
            row["completion"] = self._wait_channel_completion(
                self._transports[channel],
                max(0.0, deadline - time.monotonic()),
                owner_token=row.get("completion_owner_token"),
            )
        self._sleep(0.010)
        speed_phase = {
            "ok": all(
                row["result"].get("ok") is True
                and row.get("completion", {}).get("ok") is True
                for row in rows
            ),
            "channels": rows,
            "timeout_ms": 7_000,
            "post_wait_delay_ms": 10,
            "wait_policy": "shared_deadline_after_all_selected_speed_sends",
        }
        if speed_phase["ok"] is not True:
            raise PipetteCommandError(
                "OEM explicit-overload speed-setting phase did not complete.",
                details={"speed_phase": speed_phase},
            )
        return selected, speed_phase

    def _liquid_timeout_ms(self, volume_ul: float, channels: list[int]) -> int:
        speeds = [float(self._transports[channel]._top_speed) for channel in channels]
        return self._oem_timeout_ms(volume_ul, min(speeds), 4_000)

    def dispense_all(self, channels: list[int] | None = None) -> dict[str, Any]:
        self._require_liquid_mutation("dispense all")
        if self._forceabort():
            raise PipetteCommandError("Stopped by user or force abort")
        before = None
        if channels is None:
            selected = self._tip_location_channels()
            before = self._verify_standard_tip_selection(selected)
            speed_channel = self._tip_location if self._tip_location != -1 else 0
            level_channel = speed_channel
        else:
            selected = self._selected_channels(channels)
            speed_channel = selected[0]
            level_channel = 0
        source_return = float(self._transports[0]._liquid_level_ul)
        timeout_ms = self._oem_timeout_ms(
            self._transports[level_channel]._liquid_level_ul,
            self._transports[speed_channel]._top_speed,
            4_000,
        )
        result = self._run_group_liquid_operation(
            "dispense_all",
            selected,
            lambda _channel, transport, defer: transport.dispense_all(
                wait_for_completion=not defer,
                verify_tip=False,
            ),
            timeout_ms=timeout_ms,
            pre_send_delay_s=0.030,
            check_forceabort_after_wait=True,
        )
        return {
            **result,
            "planned_wire_command": "A0R",
            "before_tip_status": before,
            "source_return": source_return,
        }

    def aspirate_air(self, volume_ul: float, channels: list[int] | None = None, *, front_air: bool = True) -> dict[str, Any]:
        selected = self._selected_channels(channels)
        volume = float(volume_ul)
        if volume < 0:
            raise ValueError("volume_ul must be non-negative")
        result = self._run_group_liquid_operation(
            "aspirate air",
            selected,
            lambda _channel, transport, defer: transport.aspirate_air(volume, front_air=front_air, wait_for_completion=not defer),
            timeout_ms=self._liquid_timeout_ms(volume, selected),
        )
        return {**result, "volume_ul": volume, "front_air": bool(front_air), "planned_command": f"P{volume},1R"}

    def dispense_air(self, volume_ul: float, dispense_type: int = 0, channels: list[int] | None = None) -> dict[str, Any]:
        selected = self._selected_channels(channels)
        volume = float(volume_ul)
        if volume < 0:
            raise ValueError("volume_ul must be non-negative")
        selected_type = int(dispense_type)
        if selected_type not in {0, 1, 2}:
            raise ValueError("dispense_type must be 0, 1, or 2")
        result = self._run_group_liquid_operation(
            "dispense air",
            selected,
            lambda _channel, transport, defer: transport.dispense_air(volume, dispense_type=selected_type, wait_for_completion=not defer),
            timeout_ms=self._liquid_timeout_ms(volume, selected),
        )
        return {**result, "volume_ul": volume, "dispense_type": selected_type, "planned_command": f"D{volume},{selected_type}R"}

    def mix_all(self, count: int, vol: float, vigorous: int = 100) -> dict[str, Any]:
        self._require_physical_command_admission("mix_all")
        if self._forceabort():
            raise PipetteCommandError("Stopped by user or force abort")
        cycles = int(count)
        if cycles < 1:
            raise ValueError("count must be at least one")
        vigor = int(vigorous)
        if vigor < 0 or vigor > 100:
            raise ValueError("vigorous must be between 0 and 100")
        selected = self._selected_channels()
        self._require_liquid_mutation("mix all")
        effective_volume = float(self._tip_type)
        with self._transaction_lock:
            self._allow_to_stop = False
            rows: list[dict[str, Any]] = []
            for channel in selected:
                transport = self._transports[channel]
                base_speed = float(transport._top_speed)
                effective_speed = int(base_speed * vigor / 100.0)
                speed_result = transport.set_top_speed(effective_speed)
                mix_result = transport.mix(PipetteMixCommand(volume_ul=effective_volume, cycles=cycles))
                self._sleep(0.100)
                restore_result = transport.set_top_speed(int(base_speed))
                rows.append({"channel": channel, "speed": speed_result, "mix": mix_result, "restore": restore_result})
            result = {
                "ok": all(row["mix"].get("ok") is True and row["restore"].get("ok") is True for row in rows),
                "operation": "mix_all",
                "count": cycles,
                "requested_volume_ul": float(vol),
                "effective_volume_ul": effective_volume,
                "vigorous": vigor,
                "channels": rows,
                "oem_volume_source": "tip_type_derived",
                "planned_wire_command": "composite P/D; no dedicated Mix command",
                "liquid_mutation_enabled": self._liquid_mutation_enabled,
                "physical_effect_verified": False,
            }
            self._last_group_transaction = dict(result)
            return result

    def detect_fluid(self, *, dry_run: bool = False) -> dict[str, Any]:
        if self._forceabort():
            raise PipetteCommandError("Stopped by user or force abort")
        if dry_run:
            return {
                "ok": False,
                "outcome": "dry_run_only",
                "planned_command": "BR",
                "would_move_z": False,
                "timeout_ms": 15_000,
                "physical_effect_verified": False,
            }
        selected = self._selected_channels()
        result = self._run_group_liquid_operation(
            "fluid detection",
            selected,
            lambda channel, transport, defer: transport.start_fluid_detection(wait_for_completion=not defer),
            timeout_ms=15_000,
        )
        now = time.time()
        for row in result["channels"]:
            state = (
                row.get("completion", {}).get("pipette_message_state", {})
                or row["result"].get("driver_result", {}).get("pipette_message_state", {})
            )
            if row.get("completion", {}).get("ok") is True and state.get("fluid_timestamp") is not None:
                self._fluid_detection_timestamps[int(row["channel"])] = state["fluid_timestamp"]
        return {**result, "planned_command": "BR", "wait": True}

    def get_fluid_timestamp(self, channel: int) -> float | None:
        selected = self._selected_channels([channel])[0]
        return self._fluid_detection_timestamps[selected]

    def waitforcompletion(self, job: str, timeout_ms: int) -> dict[str, Any]:
        deadline = time.monotonic() + max(0.0, int(timeout_ms) / 1000.0)
        rows: list[dict[str, Any]] = []
        for channel, transport in enumerate(self._transports):
            remaining = max(0.0, deadline - time.monotonic())
            driver = transport._get_driver()
            wait_fn = getattr(driver, "wait_pipette_command_completion", None)
            if not callable(wait_fn):
                result = {"ok": False, "outcome": "completion_wait_unavailable", "channel": channel}
            else:
                result = wait_fn(remaining)
            rows.append({"channel": channel, "result": result})
        return {
            "ok": all(row["result"].get("ok") is True for row in rows),
            "job": str(job),
            "timeout_ms": int(timeout_ms),
            "channels": rows,
            "outcome": "completion" if all(row["result"].get("ok") is True for row in rows) else "completion_timeout_or_error",
            "wait_policy": "single_shared_deadline_safety_hardening",
        }

    def queryIndividualTipStatus(self) -> list[bool]:  # noqa: N802
        return [bool(row["tip_loaded"]) for row in self.query_tip_status_all()["channels"]]

    def loadTip(self, tip_type: int, tip_location: int = -1) -> dict[str, Any]:  # noqa: N802
        self._tip_type = int(tip_type)
        self._tip_location = int(tip_location)
        return {
            "ok": True,
            "outcome": "host_state_only",
            "tip_type": self._tip_type,
            "tip_location": self._tip_location,
            "hardware_commanded": False,
            "physical_effect_verified": False,
        }

    def set_tip(self, command: PipetteTipCommand) -> dict[str, Any]:
        if command.action is PipetteTipAction.LOAD:
            metadata = dict(command.metadata)
            return self.loadTip(int(metadata.get("tip_type", 201)), int(metadata.get("tip_location", -1)))
        metadata = dict(command.metadata)
        channels = self._selected_channels(metadata.get("channels")) if metadata.get("channels") is not None else None
        return self.eject_all_tips(
            check_missing_tip=bool(metadata.get("check_missing_tip", True)),
            wait=bool(metadata.get("wait", True)),
            channels=channels,
        )

    def _eject_tip_channels_once(
        self,
        channels: list[int],
        *,
        wait_for_completion: bool,
        timeout_s: float = 8.0,
    ) -> list[dict[str, Any]]:
        sends: list[dict[str, Any]] = []
        for channel in channels:
            transport = self._transports[channel]
            result = _call_eject_tip(
                transport._get_driver(),
                initialized=bool(transport._initialized),
                wait_for_completion=wait_for_completion,
            )
            if not isinstance(result, dict) or result.get("ok") is not True:
                raise PipetteCommandError(
                    "Pipette eject did not receive a valid controller acknowledgement.",
                    details={"channel": channel, "result": result, "sends": sends},
                )
            sends.append({"channel": channel, "result": result})

        if wait_for_completion and sends:
            deadline = time.monotonic() + max(0.0, float(timeout_s))
            for row in sends:
                channel = int(row["channel"])
                wait_fn = getattr(self._transports[channel]._get_driver(), "wait_pipette_command_completion", None)
                if callable(wait_fn) and row["result"].get("outcome") == "tx_only":
                    completion = wait_fn(max(0.0, deadline - time.monotonic()))
                    row["completion"] = completion
                    if not isinstance(completion, dict) or completion.get("ok") is not True:
                        raise PipetteCommandError(
                            "Pipette eject completion was not verified.",
                            details={"channel": channel, "completion": completion, "sends": sends},
                        )
        return sends

    def eject_all_tips(
        self,
        *,
        check_missing_tip: bool = True,
        wait: bool = True,
        channels: list[int] | None = None,
    ) -> dict[str, Any]:
        self._require_physical_command_admission("eject_all_tips")
        selected = self._selected_channels(channels)
        with self._transaction_lock:
            before = self.query_tip_status_all()
            loaded = set(before["channels_with_tips"])
            missing = [channel for channel in selected if channel not in loaded]
            if check_missing_tip and missing:
                raise PipetteTipStateError(
                    "OEM ejectAllTips detected a missing tip on a selected channel.",
                    details={"selected_channels": selected, "missing_channels": missing, "before": before},
                )

            targeted = [channel for channel in selected if channel in loaded]
            sends = self._eject_tip_channels_once(
                targeted,
                wait_for_completion=bool(wait),
                timeout_s=8.0,
            )
            if not wait:
                # ClassPipetteCollection.ejectAllTips(wait=false) sleeps before
                # invoking verifyEjectTip().
                self._sleep(0.500)
            after = self.query_tip_status_all()
            remaining = [channel for channel in selected if channel in set(after["channels_with_tips"])]
            retried = False
            if remaining:
                # OEM verifyEjectTip(-1) performs one second pass and waits up
                # to six seconds before its final query.
                retried = True
                retry_sends = self._eject_tip_channels_once(
                    remaining,
                    wait_for_completion=True,
                    timeout_s=6.0,
                )
                sends.extend(retry_sends)
                after = self.query_tip_status_all()
                remaining = [channel for channel in selected if channel in set(after["channels_with_tips"])]
            if remaining:
                raise PipetteTipStateError(
                    "OEM ejectAllTips verification still reports loaded tips after one retry.",
                    details={
                        "remaining_channels": remaining,
                        "before": before,
                        "sends": sends,
                        "after": after,
                        "retry_performed": retried,
                    },
                )
            self._tip_location = -1
            self._tip_type = 201
            return {
                "ok": True,
                "outcome": "verified_empty_after_retry" if retried else "verified_empty",
                "channels_targeted": selected,
                "before": before,
                "sends": sends,
                "after": after,
                "check_missing_tip": bool(check_missing_tip),
                "wait_requested": bool(wait),
                "hardware_postcondition_verified": True,
                "physical_effect_verified": False,
                "oem_source_anchor": "ClassPipetteCollection.ejectAllTips:1176-1235; verifyEjectTip:1265-1323",
            }

    def KeepTip(self, tip: int) -> dict[str, Any]:  # noqa: N802
        self._require_physical_command_admission("keep_tip")
        keep = self._selected_channels([int(tip)])[0]
        with self._transaction_lock:
            before = self.query_tip_status_all()
            if keep not in set(before["channels_with_tips"]):
                raise PipetteTipStateError(
                    "KeepTip requires a hardware-confirmed tip on the selected channel.",
                    details={"tip": keep, "before": before},
                )
            eject_channels = [channel for channel in before["channels_with_tips"] if channel != keep]
            result = self.eject_all_tips(check_missing_tip=False, wait=True, channels=eject_channels) if eject_channels else {
                "ok": True,
                "outcome": "nothing_to_eject",
                "channels_targeted": [],
                "before": before,
                "sends": [],
                "after": before,
                "hardware_postcondition_verified": True,
                "physical_effect_verified": False,
            }
            after = self.query_tip_status_all()
            if after["channels_with_tips"] != [keep]:
                raise PipetteTipStateError(
                    "KeepTip postcondition did not leave exactly the requested channel loaded.",
                    details={"keep": keep, "after": after, "ejection": result},
                )
            self._tip_location = keep
            return {"ok": True, "outcome": "kept_one_tip", "tip": keep, "ejection": result, "after": after, "physical_effect_verified": False}

    def verifyEjectTip(self, tip: int = -1) -> dict[str, Any]:  # noqa: N802
        before = self.query_tip_status_all()
        loaded = before["channels_with_tips"]
        if int(tip) == -1:
            ok = not loaded
        else:
            selected = self._selected_channels([int(tip)])[0]
            ok = loaded == [selected]
        return {
            "ok": bool(ok),
            "tip": int(tip),
            "channels_with_tips": loaded,
            "hardware_postcondition_verified": True,
            "physical_effect_verified": False,
            "oem_source_anchor": "ClassPipetteCollection.verifyEjectTip:1265-1323",
        }

    def query_tip_status_all(self) -> dict[str, Any]:
        """Return exact four-channel OEM `?31` hardware readback."""
        with self._transaction_lock:
            channels: list[dict[str, Any]] = []
            for channel, transport in enumerate(self._transports):
                driver = transport._get_driver()
                result = transport._safe_query_tip_status(driver, required=True)
                if (
                    not isinstance(result, dict)
                    or result.get("ok") is not True
                    or result.get("semantic_ok") is not True
                    or result.get("hardware_truth_level") != "hardware_query"
                    or type(result.get("tip_loaded")) is not bool
                ):
                    raise PipetteCommandError(
                        "Pipette hardware tip-status query did not return exact four-channel readback.",
                        details={"channel": channel, "result": result},
                    )
                loaded = result["tip_loaded"]
                channels.append({"channel": channel, "tip_loaded": loaded, "result": result})
            loaded_channels = [row["channel"] for row in channels if row["tip_loaded"]]
            return {
                "ok": True,
                "tip_count": len(loaded_channels),
                "channels_with_tips": loaded_channels,
                "channels": channels,
                "hardware_query_verified": True,
                "hardware_truth_level": "hardware_query",
                "physical_effect_verified": False,
                "oem_source_anchor": "ClassPipetteCollection.queryTipStatus:1336-1357",
            }

    def eject_all_tips_for_oem_startup(
        self,
        *,
        operator_ack: str,
        expected_channels_with_tips: list[int],
    ) -> dict[str, Any]:
        """Fixed startup-only E1R sequence with mandatory postcondition readback."""
        self._require_physical_command_admission("eject_all_tips_for_oem_startup")
        with self._transaction_lock:
            return self._eject_all_tips_for_oem_startup_locked(
                operator_ack=operator_ack,
                expected_channels_with_tips=expected_channels_with_tips,
            )

    def _eject_all_tips_for_oem_startup_locked(
        self,
        *,
        operator_ack: str,
        expected_channels_with_tips: list[int],
    ) -> dict[str, Any]:
        if type(operator_ack) is not str or operator_ack != "EJECT_STALE_STARTUP_TIPS":
            raise PipetteCommandError("Literal startup-tip operator acknowledgement is required.")
        if (
            not isinstance(expected_channels_with_tips, list)
            or any(type(channel) is not int or channel not in self.CHANNELS for channel in expected_channels_with_tips)
            or len(set(expected_channels_with_tips)) != len(expected_channels_with_tips)
        ):
            raise PipetteCommandError("Expected startup tip channels must be unique exact integers 0..3.")
        expected = sorted(expected_channels_with_tips)
        before = self.query_tip_status_all()
        actual = before["channels_with_tips"]
        if actual != expected:
            raise PipetteCommandError(
                "Pipette tip state changed since authorization; refusing startup ejection.",
                details={"expected_channels_with_tips": expected, "actual_channels_with_tips": actual},
            )
        sends: list[dict[str, Any]] = []
        for channel in expected:
            driver = self._transports[channel]._get_driver()
            result = _call_eject_tip(driver, initialized=bool(self._transports[channel]._initialized))
            if (
                not isinstance(result, dict)
                or result.get("ok") is not True
                or not isinstance(result.get("ack"), dict)
                or result["ack"].get("outcome") != "ack"
            ):
                post_attempt_readback = None
                post_attempt_error = None
                try:
                    post_attempt_readback = self.query_tip_status_all()
                except PipetteCommandError as exc:
                    post_attempt_error = exc.to_payload()
                raise PipetteCommandError(
                    "Pipette eject did not receive an exact immediate acknowledgement.",
                    details={
                        "failed_channel": channel,
                        "result": result,
                        "sends": sends,
                        "post_attempt_readback": post_attempt_readback,
                        "post_attempt_readback_error": post_attempt_error,
                        "physical_effect_verified": False,
                    },
                )
            sends.append({"channel": channel, "result": result})
        after = self.query_tip_status_all()
        if after["tip_count"] != 0:
            raise PipetteCommandError(
                "Post-eject hardware readback still reports loaded tips.",
                details={"after": after, "sends": sends, "physical_effect_verified": False},
            )
        return {
            "ok": True,
            "outcome": "verified_empty",
            "channels_targeted": expected,
            "before": before,
            "sends": sends,
            "after": after,
            "immediate_acknowledgements_verified": True,
            "hardware_postcondition_verified": True,
            "physical_effect_verified": False,
            "oem_source_anchors": [
                "ClassPipetteCollection.ejectAllTips:1176-1235",
                "ClassPipetteCollection.verifyEjectTip:1265-1323",
                "ClassPipetteCollection.queryTipStatus:1336-1357",
            ],
        }

    @staticmethod
    def _mutation_blocked(operation: str) -> dict[str, Any]:
        raise PipetteCommandError(
            f"Pipette {operation} remains blocked by the accepted production safety envelope.",
            details={"operation": operation, "liquid_mutation_enabled": False, "channel_count": 4},
        )

    def aspirate(self, command: PipetteAspirateCommand) -> dict[str, Any]:
        before = None
        speed_phase = None
        if command.speed is None:
            selected = self._tip_location_channels()
            before = self._verify_standard_tip_selection(selected)
            speed_channel = self._tip_location if self._tip_location != -1 else 0
            speed = float(self._transports[speed_channel]._top_speed)
        else:
            selected, speed_phase = self._prepare_explicit_speed_overload(
                command.channels or (),
                command.speed,
            )
            speed = float(command.speed)
        timeout_ms = self._oem_timeout_ms(command.volume_ul, speed, 4_000)
        result = self._run_group_liquid_operation(
            "aspirate",
            selected,
            lambda _channel, transport, defer: transport.aspirate(
                command,
                wait_for_completion=not defer,
                verify_tip=False,
            ),
            timeout_ms=timeout_ms,
            post_send_delay_s=0.010,
            timeout_failure_sleep_s=1.0,
        )
        return {
            **result,
            "requested": command.to_payload(),
            "pressure_profile": command.pressure_profile,
            "before_tip_status": before,
            "speed_phase": speed_phase,
            "overload": "explicit_channels_speed" if command.speed is not None else "standard_tip_location",
        }

    def dispense(self, command: PipetteDispenseCommand) -> dict[str, Any]:
        before = None
        speed_phase = None
        if command.speed is None:
            selected = self._tip_location_channels()
            before = self._verify_standard_tip_selection(selected)
            speed_channel = self._tip_location if self._tip_location != -1 else 0
            speed = float(self._transports[speed_channel]._top_speed)
            timeout_ms = self._oem_standard_dispense_timeout_ms(command.volume_ul, speed)
        else:
            selected, speed_phase = self._prepare_explicit_speed_overload(
                command.channels or (),
                command.speed,
            )
            speed = float(command.speed)
            timeout_ms = self._oem_explicit_dispense_timeout_ms(command.volume_ul, speed)
        result = self._run_group_liquid_operation(
            "dispense",
            selected,
            lambda _channel, transport, defer: transport.dispense(
                command,
                wait_for_completion=not defer,
                verify_tip=False,
            ),
            timeout_ms=timeout_ms,
            pre_send_delay_s=0.005 if command.volume_ul <= 5.0 else 0.0,
            post_send_delay_s=0.010,
            timeout_failure_sleep_s=1.0,
        )
        return {
            **result,
            "requested": command.to_payload(),
            "pressure_profile": command.pressure_profile,
            "before_tip_status": before,
            "speed_phase": speed_phase,
            "source_return": 0,
            "overload": "explicit_channels_speed" if command.speed is not None else "standard_tip_location",
        }

    def mix(self, command: PipetteMixCommand) -> dict[str, Any]:
        if self._tip_location == -1:
            raise PipetteCommandError("OEM one-channel Mix requires a resolved TipLocation")
        selected = [self._tip_location]
        sequence_wait_ms = int(command.cycles) * 3_000 - 1_500
        timeout_ms = sequence_wait_ms + self._liquid_timeout_ms(command.volume_ul, selected)
        result = self._run_group_liquid_operation(
            "mix",
            selected,
            lambda _channel, transport, _defer: transport.mix(
                command,
                wait_for_completion=False,
                verify_tip=False,
            ),
            timeout_ms=timeout_ms,
            defer_completion=False,
        )
        return {
            **result,
            "requested": command.to_payload(),
            "pressure_profile": command.pressure_profile,
            "sequence_wait_ms": sequence_wait_ms,
        }

    def close(self) -> None:
        for transport in self._transports:
            transport.close()


def build_default_pipette_transport(
    *,
    shared_usb: Any | None = None,
    error_callback: Callable[[int, int], None] | None = None,
    forceabort: Callable[[], bool] | None = None,
) -> FourPipetteTransport:
    transport = os.environ.get("BIOXP_PIPETTE_TRANSPORT", "novo_usb").strip().lower().replace("-", "_")
    response_timeout_s = 60.0
    transports: list[CanPipetteTransport] = []
    if transport in {"can", "socketcan", "can0"}:
        raise PipetteTransportUnavailableError(
            "SocketCAN is not an accepted production fallback; the shared NovoRouter is required.",
            details={"selected": transport, "shared_router_required": True},
        )

    if transport in {"novo", "novo_usb", "usb", "pyusb"}:
        alt = int(os.environ.get("BIOXP_USB_ALT", "1"))

        for pipette_id in FourPipetteTransport.CHANNELS:
            def _factory(channel_id: int = pipette_id) -> Any:
                if BioXpNovoUsbDriver is None or shared_usb is None:
                    raise PipetteTransportUnavailableError(
                        "Novo USB-CAN requires the shared BioXpTester/NovoRouter owner.",
                        details={"transport": transport, "channel": channel_id, "shared_router": False},
                    )
                return BioXpNovoUsbDriver(shared_usb=shared_usb, alt=alt, pipette_id=channel_id, response_timeout_s=60.0)

            transports.append(CanPipetteTransport(
                driver_factory=_factory, channel="novo-usb-shared", bitrate=0,
                pipette_id=pipette_id, response_timeout_s=60.0, transport_name="novo_usb_can",
                transport_details={
                    "source": "OEM Novo.Devices.CanInterfaceBoard over one shared NovoRouter",
                    "vid": "0x03eb", "pid": "0x2423", "alt": alt,
                    "shared_bioxp_usb_runtime": shared_usb is not None,
                },
            ))
        return FourPipetteTransport(
            transports,
            error_callback=error_callback,
            forceabort=forceabort,
        )

    raise ValueError(f"Unsupported BIOXP_PIPETTE_TRANSPORT={transport!r}; expected novo_usb or socketcan")
