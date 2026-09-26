"""OEM diagnostic callers, not constructor or explicit-stroke API aliases.

Collection anchors refer to ClassPipetteCollection.cs in the retained OEM SSD.
Cached eligibility here intentionally does not use _tip_eligibility: the source
buttons consume m_PipetteHasTip, not observation age/provenance/generation.
"""
from __future__ import annotations

from typing import Annotated, Any, Literal, Union
from pydantic import BaseModel, ConfigDict, Field, field_validator

from .models import PipetteDiagnosticCommand, PipetteErrorLogCommand, PipetteInitCommand
from .models import PipetteCommandError


class _Input(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)


class _Selected(_Input):
    channels: list[Annotated[int, Field(ge=0, le=3)]] = Field(max_length=4)

    @field_validator("channels")
    @classmethod
    def unique(cls, value):
        if len(set(value)) != len(value):
            raise ValueError("duplicate channel")
        return value


class DiagnosticLiquid(_Selected):
    action: Literal["aspirate", "dispense"]
    volume_ul: float = Field(ge=0)
    speed: int = Field(gt=0)


class DiagnosticEject(_Selected):
    action: Literal["eject"]


class DiagnosticSimple(_Input):
    action: Literal["dispense_all", "diagnoses", "initialize", "get_data", "last_error"]


class DiagnosticPlunger(_Input):
    action: Literal["plunger_up", "plunger_down"]
    steps: int = Field(ge=0, le=2147483647)


DiagnosticRequest = Annotated[Union[DiagnosticLiquid, DiagnosticEject,
    DiagnosticSimple, DiagnosticPlunger], Field(discriminator="action")]


def _cached(group):
    return [i for i, t in enumerate(group._transports) if t._tip_loaded]


def _source_liquid_send(t, action, volume):
    # Dedicated source path: do not relax the unrelated explicit-stroke API's
    # tip gate. The OEM diagnostic aspirate keeps lost tips selected and sends.
    t._require_initialized()
    driver = t._get_driver()
    call = getattr(driver, action)
    raw = t._assert_driver_result(action, call(volume, tip_pressure_profile="1R",
                                              wait_for_completion=False))
    t._last_command = action
    return {"ok": raw.get("ok") is True, "volume_ul": volume,
            "driver_result": raw, **t._driver_evidence(raw)}


def _firmware_text(row):
    raw = row.get("result") or {}
    return raw.get("firmware")


def run_collection_diagnostic(group, request, record, result=None):
    """Run under the finite owner; record uses the existing receipt owner.

    No outer transaction lock across waits: addressed TR must stay interruptible.
    record(name, callable) preserves partial evidence and original exceptions.
    """
    action = request.action
    if result is None:
        result = {"action": action, "physical_effect_verified": False}
    if action in {"aspirate", "dispense"}:
        selected = sorted(request.channels)
        lost = []
        for channel in _cached(group):
            query = record("query_all_pipette_tip_states", lambda t, ch=channel:
                t.query_tip_status_for_oem_script(ch))
            if query["source_return"] != 1:
                group._transports[channel]._tip_loaded = False
                lost.append(channel)
        if action == "dispense":
            selected = [ch for ch in selected if ch not in lost]
        result.update(selected_channels=selected, lost_tip_channels=lost,
                      cached_tip_channels=_cached(group))
        if request.volume_ul > 0:
            # Source outer SetTopSpeed uses cached tips and channel-zero equality;
            # reuse channel setters without the generic freshness policy.
            cached = _cached(group)
            outer_speed = record("set_top_speed", lambda t: _set_cached_speed(t, request.speed, cached))
            if outer_speed.get("interrupted_by_terminate"):
                return result
            speed_channels = [ch for ch in selected if ch in cached]
            speed_phase = record("set_top_speed", lambda t:
                _explicit_speed(t, speed_channels, request.speed))
            if speed_phase.get("interrupted_by_terminate"):
                return result
            timeout = (group._oem_timeout_ms(request.volume_ul, request.speed, 4000)
                       if action == "aspirate" else
                       group._oem_explicit_dispense_timeout_ms(request.volume_ul, request.speed))
            result["stroke"] = record(action, lambda t: t._run_group_liquid_operation(
                action, selected, lambda ch, channel, defer:
                    _source_liquid_send(channel, action, request.volume_ul),
                timeout_ms=timeout, post_send_delay_s=.010,
                timeout_failure_sleep_s=1.0))
        return result
    if action == "dispense_all":
        selected = _cached(group)
        result["cached_tip_channels"] = selected
        result["dispense_all"] = record("dispense_all", lambda t: t._run_group_liquid_operation(
            "dispense_all", selected, lambda ch, channel, defer:
                _source_dispense_all(channel), timeout_ms=7000))
    elif action == "diagnoses":
        result["tests"] = []
        for number, label in enumerate(("Plunger Force Test", "Plunger Step-loss Test", "Pressure Sensor Test")):
            selected = _cached(group)
            body = record("diagnoses", lambda t, n=number, channels=selected:
                t._run_group_liquid_operation("diagnoses", channels,
                    lambda ch, channel, defer: channel.execute_diagnoses(
                        PipetteDiagnosticCommand(number=n), wait_for_completion=False),
                    timeout_ms=4000, post_send_delay_s=.001, set_allow_to_stop=False))
            values = {row["channel"]: (row.get("completion", {}).get("pipette_message_state") or {}).get("diagnosis")
                      for row in body["channels"]}
            result["tests"].append({"number": number, "label": label, "result": body,
                "channels": [{"channel": ch, "diagnosis": values.get(ch),
                    "display": (values[ch].replace(" `", "") if isinstance(values.get(ch), str)
                                and values[ch] else "No data returned")} for ch in range(4)]})
            if body.get("interrupted_by_terminate"):
                break
    elif action == "initialize":
        # Reuse the existing one-cycle/status helpers, NOT initialize() (which
        # includes constructor condition/firmware logic). Source ignores waits.
        attempts = []
        result["attempts"] = attempts
        for attempt in ("initial", "retry"):
            group_result = record("initialize", lambda t, a=attempt: t._run_group_cycle(
                PipetteInitCommand(), cycle=f"diagnostic.{a}", continue_after_completion_failure=True))
            if group_result.get("interrupted_by_terminate"):
                attempts.append({"attempt": attempt, "group": group_result, "status": None})
                return result
            status = record("status", lambda t, a=attempt:
                t.checked_pipette_status_for_oem_initialize_motion(attempt=a))
            attempts.append({"attempt": attempt, "group": group_result, "status": status})
            if status["ok"]:
                break
        result["attempts"] = attempts
    elif action == "eject":
        result["ejected_channels"] = []
        for channel in range(4):
            query = record("query_all_pipette_tip_states", lambda t, ch=channel:
                t.query_tip_status_for_oem_script(ch))
            if query["source_return"] == 1 and channel in request.channels:
                record("eject_tip", lambda t, ch=channel: {"ok": True, "channels":
                    t._eject_tip_channels_once([ch], wait_for_completion=False)})
                result["ejected_channels"].append(channel)
    elif action == "get_data":
        result["channels"] = []
        for channel in range(4):
            part = record("firmware", lambda t, ch=channel: t._transports[ch].query_firmware(0))
            # retriveADPInformation indexes these three fields before &1;
            # preserve its thrown malformed-reply behavior, not empty success.
            fields = (_firmware_text(part) or "").split(" ")
            part_number, revision = fields[0], fields[1] + " " + fields[2]
            firmware = record("firmware", lambda t, ch=channel: t._transports[ch].query_firmware(1))
            data = record("data", lambda t, ch=channel: t.get_data(channels=[ch]))
            result["channels"].append({"channel": channel, "part_number": part_number,
                "revision": revision,
                "firmware": _firmware_text(firmware), "information": [part, firmware], "data": data})
    elif action == "last_error":
        body = record("error_log", lambda t: t.query_error_log(PipetteErrorLogCommand(raw_byte=1)))
        result["channels"] = []
        for row in body["channels"]:
            raw = row["result"].get("result") or {}
            value = raw.get("oem_error_code")
            result["channels"].append({"channel": row["channel"], "error": value,
                "display": f"0x{value:02X}" if type(value) is int else None, "result": row["result"]})
    return result


def _explicit_speed(group, channels, speed):
    epoch = group._interrupt_epoch
    try:
        phase = (group._prepare_explicit_speed_overload(channels, speed)[1] if channels else
                 {"ok": True, "channels": [], "timeout_ms": 7000})
    except PipetteCommandError as exc:
        # The reused helper promotes wait=false to an exception for its ordinary
        # API. This source caller ignores exactly that wait, not send exceptions.
        phase = exc.details.get("speed_phase") if isinstance(exc.details, dict) else None
        if phase is None:
            raise
    if not channels:
        group._sleep(.010)
    phase["interrupted_by_terminate"] = epoch != group._interrupt_epoch
    return phase


def _set_cached_speed(group, speed, channels):
    if group._forceabort():
        raise PipetteCommandError("Stopped by user or force abort")
    if group._transports[0]._top_speed == speed:
        return {"ok": True, "outcome": "unchanged_channel_zero_speed", "channels": []}
    return _explicit_speed(group, channels, speed)


def _source_dispense_all(channel):
    channel._require_initialized()
    raw = channel._assert_driver_result("dispense_all",
        channel._get_driver().dispense_all(wait_for_completion=False))
    return {"ok": raw.get("ok") is True, "driver_result": raw, **channel._driver_evidence(raw)}


def run_diagnostic_inline(provider, diagnostic, *, command_id, owner_identity):
    from pydantic import TypeAdapter
    request = TypeAdapter(DiagnosticRequest).validate_python(diagnostic)
    events = []
    anchors = {"aspirate": "ClassPipetteCollection:418-473,844-882",
        "dispense": "ClassPipetteCollection:475-531,1055-1093",
        "dispense_all": "ClassPipetteCollection:533-555",
        "diagnoses": "ClassPipetteCollection:261-283,571-582",
        "initialize": "ClassPipetteCollection:584-604,677-693,726-748",
        "eject": "ClassPipetteCollection:606-624,1237-1240",
        "get_data": "ClassPipetteCollection:302-309,640-659",
        "last_error": "ClassPipetteCollection:557-569",
        "plunger_up": "ControlLib:1426-1433", "plunger_down": "ControlLib:1417-1424"}
    result = {"action": request.action, "events": events, "physical_effect_verified": False,
              "source_anchor": anchors[request.action]}

    def record(name, call):
        identity = {**owner_identity, "source_identity":
            f"{owner_identity['source_identity']}:diagnostic:{len(events)}:{name}"}
        provider._wp8_execution_fence_checker(command_id, boundary=identity["source_identity"])
        event = {"operation": name, "source_identity": identity["source_identity"]}
        events.append(event)
        epoch = provider.primitives.pipette_transport._interrupt_epoch
        try:
            body = provider._manual_pipette_receipt_runner(name, call, command_id, identity, diagnostic)
            if epoch != provider.primitives.pipette_transport._interrupt_epoch:
                body["interrupted_by_terminate"] = True
        except Exception as exc:
            event.update(error=str(exc), evidence=getattr(exc, "details", None))
            raise
        event["result"] = body
        return body

    try:
        if request.action in {"plunger_up", "plunger_down"}:
            # These are native motion primitives, not pipette receipt operations.
            for name, call in (("setZaxisCurrentmax31", lambda: provider.primitives.z_set_current_max(31)),
                    ("moveStepsZ", lambda: provider.primitives.z_move_steps(steps=
                        -request.steps if request.action == "plunger_up" else request.steps))):
                provider._wp8_execution_fence_checker(command_id, boundary=name)
                body = call()
                events.append({"operation": name, "result": body})
                if body.get("ok") is False:
                    result.update(ok=False, completed=False, failure=name)
                    return result
        else:
            run_collection_diagnostic(provider.primitives.pipette_transport, request, record, result)
        result.update(completed=True,
            controller_outcome_ok=all(e["result"].get("ok") is not False for e in events))
        interrupted = any(e["result"].get("interrupted_by_terminate") for e in events)
        if request.action in {"aspirate", "dispense", "dispense_all", "diagnoses", "initialize"}:
            # These particular source void callers ignore the wait Boolean;
            # initialize uses only first status to select its one retry. Do not
            # convert missing completion/status evidence into a new source gate.
            result.update(ok=not interrupted, source_return_completed=not interrupted,
                          interrupted_by_terminate=interrupted)
        else:
            result["ok"] = result["controller_outcome_ok"]
    except Exception as exc:
        result.update(ok=False, completed=False, error=str(exc), exception_type=type(exc).__name__)
    return {**result, "delivery_attempted": bool(events)}
