from __future__ import annotations

import os
import threading
import time
from typing import Any, Callable, Protocol

from .. import BioXpCanDriver
try:
    from ..novo_usb_can import BioXpNovoUsbDriver
except Exception:  # pragma: no cover - surfaced lazily in status payloads
    BioXpNovoUsbDriver = None  # type: ignore[assignment]
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
        response_timeout_s: float = 60.0,
        transport_name: str = "can",
        transport_details: dict[str, Any] | None = None,
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


class FourPipetteTransport:
    """Production ClassPipetteCollection-shaped owner for channels 0..3."""

    CHANNELS = (0, 1, 2, 3)

    def __init__(self, transports: list[CanPipetteTransport], *, sleep: Callable[[float], None] = time.sleep) -> None:
        if len(transports) != 4:
            raise ValueError("OEM production pipette collection requires exactly four channels")
        self._transports = list(transports)
        self._sleep = sleep
        self._transaction_lock = threading.RLock()
        self._last_group_transaction: dict[str, Any] | None = None

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
            "liquid_mutation_enabled": False,
        }

    def _run_group_cycle(self, command: PipetteInitCommand, *, cycle: str) -> dict[str, Any]:
        sends: list[dict[str, Any]] = []
        for channel, transport in enumerate(self._transports):
            driver = transport._get_driver()
            if transport._initialized:
                # ClassPipette.initiate(false) skips the wake frame once initialized,
                # but still waits 100 ms and transmits WR.
                self._sleep(0.100)
                driver_result = driver.pipette_initiate_group()
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

        # ClassPipetteCollection.waitforcompletion(..., 10000) owns one shared
        # deadline for all four channel events.
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
        pressure_offsets = router.calculate_pressure_offsets() if router is not None else {}
        stream_ok = all(row["result"].get("ok") for row in stream_on + stream_off)
        return {
            "ok": bool(stream_ok),
            "cycle": cycle,
            "outcome": "completion" if stream_ok else "pressure_stream_command_failed",
            "sends": sends,
            "delayed_completions": completions,
            "completion_timeout_ms": 10_000,
            "pressure_stream": {"on": stream_on, "wait_ms": 1_000, "off": stream_off},
            "pressure_epoch": pressure_epoch,
            "pressure_offsets": pressure_offsets,
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

    def initialize(self, command: PipetteInitCommand) -> dict[str, Any]:
        started = time.monotonic()
        initial_group = self._run_group_cycle(command, cycle="constructor_initiateGroup")
        if not initial_group.get("ok"):
            self._last_group_transaction = {
                "ok": False,
                "outcome": "initial_group_cycle_failed",
                "channels": initial_group.get("sends", []),
                "initial_group": initial_group,
                "group_wait_ms": 10_000,
                "liquid_mutation_enabled": False,
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
            "group_wait_ms": 10_000,
            "pipette_transaction_timeout_ms": 60_000,
            "elapsed_ms": int(round((time.monotonic() - started) * 1000.0)),
            "liquid_mutation_enabled": False,
        }
        return dict(self._last_group_transaction)

    def query_tip_status_all(self) -> dict[str, Any]:
        """Return exact four-channel OEM `?31` hardware readback."""
        with self._transaction_lock:
            channels: list[dict[str, Any]] = []
            for channel, transport in enumerate(self._transports):
                result = transport._get_driver().query_tip_status()
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
                transport._last_tip_status = dict(result)
                transport._tip_loaded = loaded
                channels.append({"channel": channel, "tip_loaded": loaded, "result": result})
            loaded_channels = [row["channel"] for row in channels if row["tip_loaded"]]
            return {
                "ok": True,
                "tip_count": len(loaded_channels),
                "channels_with_tips": loaded_channels,
                "channels": channels,
                "hardware_query_verified": True,
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
            result = driver.pipette_eject_tip()
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
            "physical_effect_verified": True,
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

    def set_tip(self, command: PipetteTipCommand) -> dict[str, Any]:
        del command
        return self._mutation_blocked("tip mutation")

    def aspirate(self, command: PipetteAspirateCommand) -> dict[str, Any]:
        del command
        return self._mutation_blocked("aspirate")

    def dispense(self, command: PipetteDispenseCommand) -> dict[str, Any]:
        del command
        return self._mutation_blocked("dispense")

    def mix(self, command: PipetteMixCommand) -> dict[str, Any]:
        del command
        return self._mutation_blocked("mix")

    def close(self) -> None:
        for transport in self._transports:
            transport.close()


def build_default_pipette_transport(*, shared_usb: Any | None = None) -> FourPipetteTransport:
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
        return FourPipetteTransport(transports)

    raise ValueError(f"Unsupported BIOXP_PIPETTE_TRANSPORT={transport!r}; expected novo_usb or socketcan")
