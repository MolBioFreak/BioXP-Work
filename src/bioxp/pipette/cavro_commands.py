"""Pure original-generation Cavro ADP command/program codec (30053815-C).

No IO, retries, implicit startup, calibration or hardware admission policy. Numeric
manufacturer domains are diagnostics, not coercion or refusal. This opt-in typed
surface does not modify legacy encoders. A compiled envelope is an instruction,
never evidence that its effects completed or its persistent data was read back.
"""
from __future__ import annotations

from copy import deepcopy
from decimal import Decimal
from typing import Annotated, Any, Literal, Union, get_args

from pydantic import BaseModel, BeforeValidator, ConfigDict, Field, StrictFloat, StrictInt, StrictStr, TypeAdapter, field_validator

# Decimal strings retain authored zeros and precision. JSON numbers retain their
# Python shortest round-trip spelling, without a second binary-float conversion.
DecimalText = Annotated[StrictStr, Field(pattern=r"^-?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)$")]


def _preserve_decimal(value: Any) -> Any:
    return format(value, "f") if isinstance(value, Decimal) else value


Number = Annotated[Union[StrictInt, Annotated[StrictFloat, Field(allow_inf_nan=False)], DecimalText],
                   BeforeValidator(_preserve_decimal)]


class CommandModel(BaseModel):
    model_config = ConfigDict(extra="forbid")

    @field_validator("selector", "start_mode", mode="before", check_fields=False)
    @classmethod
    def _integer_selector(cls, value: Any) -> Any:
        if type(value) is not int:
            raise ValueError("Selector must be an integer, not a boolean or coerced number")
        return value


class Initialize(CommandModel):
    command: Literal["initialize"]


class EjectTip(CommandModel):
    command: Literal["eject_tip"]


class PlungerMove(CommandModel):
    command: Literal["plunger_absolute", "plunger_aspirate", "plunger_dispense"]
    value: Number
    unit: Literal["increments", "ul"]


class Speed(CommandModel):
    command: Literal["top_speed", "start_speed", "cutoff_speed"]
    value: Number
    unit: Literal["increments_per_second", "ul_per_second"]


class Slope(CommandModel):
    command: Literal["slope"]
    ramp_up: Number
    ramp_down: Number


class SpeedDefault(CommandModel):
    command: Literal["speed_default"]
    speed: Literal["top", "start", "cutoff"]


class Backlash(CommandModel):
    command: Literal["backlash"]
    increments: Number


class BeginLld(CommandModel):
    command: Literal["begin_lld"]
    start_mode: Literal[0, 1] = 0


class LldParameter(CommandModel):
    command: Literal["lld_parameter"]
    parameter: Literal["threshold_adc", "threshold_duration_ms", "start_position_increments",
                       "speed_increments_per_second", "maximum_travel_increments", "direction",
                       "sample_interval_ms", "samples_per_can_message", "hybrid_interval_ms"]
    value: Number


class DigitalOutput(CommandModel):
    command: Literal["digital_output"]
    level: Literal["low", "high"]


class EventSubscription(CommandModel):
    command: Literal["event_subscription"]
    event: Literal["pressure", "tip_loss", "digital_rising", "digital_falling",
                   "clld_rising", "clld_falling", "lld_success"]
    mode: Literal["disabled", "always", "during_lld"]


class BufferControl(CommandModel):
    command: Literal["buffer_execute", "buffer_replay", "buffer_clear", "terminate", "reset"]


class LoopStart(CommandModel):
    command: Literal["loop_start"]


class LoopEnd(CommandModel):
    command: Literal["loop_end"]
    repetitions: Number


class Delay(CommandModel):
    command: Literal["delay"]
    milliseconds: Number


class Halt(CommandModel):
    command: Literal["halt"]
    resume_on: Literal["command_or_input", "command_only"] = "command_or_input"
    input_edge: Literal["falling", "rising"] = "falling"


class UserByteWrite(CommandModel):
    command: Literal["user_byte_write"]
    location: StrictInt
    value: StrictInt


class UserByteRead(CommandModel):
    command: Literal["user_byte_read"]
    location: StrictInt = 0


class StoredProgramRun(CommandModel):
    command: Literal["stored_program_run"]
    location: StrictInt


class SaveSettings(CommandModel):
    command: Literal["save_settings"]


class RestoreSettings(CommandModel):
    command: Literal["restore_settings"]
    source: Literal["nvram", "factory"]


class Status(CommandModel):
    command: Literal["status"]
    extended: bool = False


class Firmware(CommandModel):
    command: Literal["firmware"]
    kind: Literal["part_revision", "application"] = "part_revision"


class Report(CommandModel):
    command: Literal["report"]
    selector: Literal[0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 15, 16, 17, 18, 19, 20,
                      23, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
                      40, 41, 42, 43, 44, 45, 46, 47, 48, 50, 51, 52, 53, 54, 55,
                      56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 67, 76, 77,
                      80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 99]


class ErrorLog(CommandModel):
    command: Literal["error_log"]
    index: StrictInt = 1


class CommandLog(CommandModel):
    command: Literal["command_log"]
    index: StrictInt = 1
    length: StrictInt = 1


class RawPressure(CommandModel):
    command: Literal["raw_pressure"]


class Configuration(CommandModel):
    command: Literal["configuration"]
    setting: Literal[
        "rs485_ch1_commands", "rs485_ch2_commands", "rs485_ch1_tip_loss",
        "startup_program_enabled", "startup_program_disabled", "rs485_protocol_auto",
        "rs485_protocol_oem", "rs485_protocol_dt", "rs485_baud_9600", "rs485_baud_38400",
        "rs485_baud_115200", "can_baud_100k", "can_baud_125k", "can_baud_250k",
        "can_baud_500k", "can_baud_1m", "can_heartbeat_enabled", "can_heartbeat_disabled",
        "output_plld", "output_clld_mirror", "output_hlld", "output_general_purpose",
        "input_general_purpose", "input_emergency_brake", "lld_active_low", "lld_active_high",
        "rs485_stream_enabled", "rs485_stream_disabled", "rs485_stream_oem", "rs485_stream_dt",
        "pressure_module_enabled", "pressure_module_disabled", "tip_module_enabled", "tip_module_disabled"]


class HeartbeatPeriod(CommandModel):
    command: Literal["heartbeat_period"]
    half_seconds: Number


class Diagnostic(CommandModel):
    command: Literal["diagnostic"]
    test: Literal["minimum_moving_current", "step_loss_detection", "pressure_sensor"]


# Stored programs contain the same real command types, not raw ASCII strings.
# Recursive storage itself is excluded structurally at compile time.
class StoredProgramWrite(CommandModel):
    command: Literal["stored_program_write"]
    location: StrictInt
    commands: list[DeviceCommand]


DeviceCommand = Annotated[Union[
    Initialize, EjectTip, PlungerMove, Speed, SpeedDefault, Slope, Backlash, BeginLld, LldParameter,
    DigitalOutput, EventSubscription, BufferControl, LoopStart, LoopEnd, Delay, Halt,
    UserByteWrite, UserByteRead, StoredProgramRun, StoredProgramWrite, SaveSettings,
    RestoreSettings, Status, Firmware, Report, ErrorLog, CommandLog, RawPressure,
    Configuration, HeartbeatPeriod, Diagnostic,
], Field(discriminator="command")]
StoredProgramWrite.model_rebuild()
DEVICE_COMMAND_ADAPTER = TypeAdapter(DeviceCommand)


class DeviceProgram(CommandModel):
    """Append commands to the controller buffer, optionally executing that buffer.

    Reports and immediate controls are separate commands, never buffer members.
    A prior controller buffer is unknown: compilation cannot assert its length,
    content, initialization, physical effects or completion.
    """
    mode: Literal["append", "execute"]
    commands: list[DeviceCommand]


DEVICE_PROGRAM_ADAPTER = TypeAdapter(DeviceProgram)


def numeric_spelling(value: Number) -> str:
    if isinstance(value, str):
        return value
    return format(Decimal(str(value)), "f")


LLD_PARAMETERS = {
    "threshold_adc": (0, 1, 1000, "adc_counts"),
    "threshold_duration_ms": (1, 1, 100, "ms"),
    "start_position_increments": (2, 0, 44000, "increments"),
    "speed_increments_per_second": (3, 100, 1000, "increments_per_second"),
    "maximum_travel_increments": (4, 500, 44000, "increments"),
    "direction": (5, 0, 1, None),
    "sample_interval_ms": (6, 1, 20, "ms"),
    "samples_per_can_message": (7, 1, 3, "samples"),
    "hybrid_interval_ms": (8, 1, 100, "ms"),
}
EVENTS = {name: n for n, name in enumerate(("pressure", "tip_loss", "digital_rising",
          "digital_falling", "clld_rising", "clld_falling", "lld_success"))}
CONFIGURATION_CODES = dict(zip(get_args(Configuration.model_fields["setting"].annotation),
    (1, 2, 3, 30, 31, 35, 36, 37, 41, 47, 48, 51, 52, 53, 54, 55, 60, 61,
     70, 71, 72, 73, 74, 75, 76, 77, 80, 81, 82, 83, 90, 91, 92, 93)))
RESET_REQUIRED_CODES = {1, 2, 3, 30, 31, 35, 36, 37, 41, 47, 48, 51, 52, 53, 54, 55, 90, 91, 92, 93}

# (name, units, printed page). Current-setting units are not inferred from d0.
REPORTS = {
    0: ("commanded_plunger_position", "increments", 45),
    1: ("true_home_plunger_position_with_calibration", "increments", 45),
    2: ("plunger_position", "mm", 45), 3: ("plunger_position", "ul", 45),
    4: ("backlash", "increments", 45),
    6: ("start_speed", "increments_per_second", 45),
    7: ("top_speed", "increments_per_second", 45),
    8: ("cutoff_speed", "increments_per_second", 45),
    9: ("ramp_up", "slope_code", 45), 10: ("ramp_down", "slope_code", 45),
    11: ("high_current_setting", None, 45), 12: ("low_current_setting", None, 45),
    15: ("active_current_level", None, 45), 16: ("maximum_plunger_position", "increments", 45),
    17: ("configured_total_volume", "ul", 45),
    18: ("start_speed", "ul_per_second", 45), 19: ("top_speed", "ul_per_second", 45),
    20: ("cutoff_speed", "ul_per_second", 45), 23: ("part_revision", None, 45),
    26: ("enabled_events", None, 45), 27: ("samples_per_can_message", "samples", 45),
    28: ("heartbeat_period", "seconds", 45), 29: ("pump_busy", None, 45),
    30: ("digital_input", None, 45), 31: ("tip_present", None, 46),
    32: ("threshold", "adc_counts", 46), 33: ("threshold_duration", "ms", 46),
    34: ("lld_start_position", "increments", 46),
    35: ("lld_speed", "increments_per_second", 46), 36: ("lld_maximum_travel", "increments", 46),
    37: ("lld_direction", None, 46), 38: ("sample_interval", "ms", 46),
    39: ("hybrid_interval", "ms", 46), 40: ("operating_time", "minutes", 46),
    41: ("power_ups", "count", 46), 42: ("initializations_total", "count", 46),
    43: ("initializations_since_reset", "count", 46),
    44: ("calculated_plunger_travel", "meters", 46),
    45: ("plunger_moves_total_excluding_init_eject", "count", 46),
    46: ("plunger_moves_since_reset_excluding_init_eject", "count", 46),
    47: ("ejections_total", "count", 46), 48: ("ejections_since_reset", "count", 46),
    50: ("supply_voltage", "volts", 46), 51: ("supply_voltage_max", "volts", 46),
    52: ("supply_voltage_min", "volts", 46), 53: ("temperature", "fahrenheit", 46),
    54: ("temperature_max", "fahrenheit", 46), 55: ("temperature_min", "fahrenheit", 46),
    56: ("pressure", "adc_counts", 46), 57: ("pressure", "psi", 46),
    58: ("pressure_max", "psi", 46), 59: ("pressure_min", "psi", 46),
    60: ("firmware_downloads", "count", 46), 61: ("firmware_compilation_time", None, 47),
    62: ("firmware_compilation_date", None, 47), 63: ("application_version", None, 47),
    64: ("boot_part_revision", None, 47), 65: ("application_checksum_32bit", None, 47),
    67: ("buffer_nonempty", None, 47), 76: ("communication_configuration", None, 47),
    77: ("io_module_configuration", None, 48),
    **{n: ("stored_program", None, 47) for n in range(80, 96)},
    96: ("self_test_program", None, 47), 99: ("loaded_program", None, 47),
}
REPORT_FIELDS = {
    76: ["rs485_baud", "can_baud", "rs485_protocol", "pressure_stream_protocol"],
    77: ["digital_input_mode", "digital_output_mode", "pressure_streaming", "pressure_module", "tip_module"],
}

# Documentary data, not Pydantic admission constraints. Shared by the encoder
# and public catalog so a UI can display domains without inventing gates.
SPEED_DOMAINS = {
    "top_speed": {
        "increments_per_second": ("1.0", "40000.0", 1, 35),
        "ul_per_second": ("0.025", "1000.000", 3, 35)},
    "start_speed": {
        "increments_per_second": ("100", "4000", 0, 36),
        "ul_per_second": ("2.500", "100.000", 3, 36)},
    "cutoff_speed": {
        "increments_per_second": ("100", "8000", 0, 36),
        "ul_per_second": ("2.500", "200.000", 3, 36)},
}


def _numeric_domain_catalog() -> list[dict]:
    rows = [{"command": command, "field": "value", "units": units,
             "minimum": domain[0], "maximum": domain[1], "decimal_places": domain[2],
             "printed_page": domain[3]} for command, domains in SPEED_DOMAINS.items()
            for units, domain in domains.items()]
    rows.extend({"command": "lld_parameter", "parameter": name, "field": "value",
                 "units": domain[3], "minimum": str(domain[1]), "maximum": str(domain[2]),
                 "decimal_places": 0, "printed_page": 55} for name, domain in LLD_PARAMETERS.items())
    rows.extend({"command": name, "field": field, "minimum": str(low), "maximum": str(high),
                 "decimal_places": 0, "printed_page": page} for name, field, low, high, page in [
        ("slope", "ramp_up", 1, 40, 37), ("slope", "ramp_down", 1, 40, 37),
        ("backlash", "increments", 0, 500, 38), ("loop_end", "repetitions", 0, 50000, 39),
        ("delay", "milliseconds", 0, 30000, 40), ("user_byte_write", "location", 0, 15, 38),
        ("user_byte_write", "value", 0, 255, 38), ("user_byte_read", "location", 0, 15, 44),
        ("stored_program_write", "location", 0, 15, 42), ("stored_program_run", "location", 0, 15, 42),
        ("error_log", "index", 1, 100, 44), ("command_log", "index", 1, 1000, 44),
        ("command_log", "length", 1, 256, 44), ("heartbeat_period", "half_seconds", 1, 20, 53)])
    return [{**row, "source": "30053815-C", "blocking": False} for row in rows]


def _warning(code: str, message: str, page: int, **details: Any) -> dict:
    return {"code": code, "message": message, "source": "30053815-C",
            "printed_page": page, "blocking": False, **details}


def _domain(warnings: list, field: str, value: Number, low: Any, high: Any,
            places: int, page: int) -> None:
    spelling = numeric_spelling(value)
    number = Decimal(spelling)
    if not Decimal(str(low)) <= number <= Decimal(str(high)):
        warnings.append(_warning("documented_range", "Authored value outside published range; unchanged.",
                                 page, field=field, value=spelling, minimum=str(low), maximum=str(high)))
    exponent = number.as_tuple().exponent
    assert isinstance(exponent, int)  # Number's schema excludes NaN/Infinity.
    if max(0, -exponent) > places:
        warnings.append(_warning("documented_precision", "Authored spelling exceeds published decimal places; unchanged.",
                                 page, field=field, value=spelling, decimal_places=places))


def _envelope(ascii: str, requested: dict, *, wait_mode: str = "completion", report_kind: str | None = None,
              units: str | None = None, persistent: bool = False, warnings: list | None = None,
              can_command_type: int = 1) -> dict:
    return {"ascii": ascii, "can_command_type": can_command_type, "wait_mode": wait_mode,
            "report_kind": report_kind, "units": units, "persistent": persistent,
            "warnings": warnings or [], "requested": requested}


def compile_device_command(payload: DeviceCommand | dict) -> dict:
    """Compile one explicit command. Never sends or asserts controller success."""
    model = DEVICE_COMMAND_ADAPTER.validate_python(payload)
    r = model.model_dump(mode="json")
    name = r["command"]
    warnings: list[dict] = []
    units = None
    persistent = False
    wait = "completion"
    report_kind = None
    frame = 1
    suffix = "R"
    if isinstance(model, (Initialize, EjectTip)):
        wire = "W" if isinstance(model, Initialize) else "E"
        warnings.append(_warning("source_unresolved_variants", "Only parameterless published action is encoded; full action table unavailable.", 33))
    elif isinstance(model, PlungerMove):
        wire = {"plunger_absolute": "A", "plunger_aspirate": "P", "plunger_dispense": "D"}[name]
        wire += numeric_spelling(model.value) + (",1" if model.unit == "ul" else ",0")
        units = model.unit
        warnings.append(_warning("source_unresolved_domain", "Pages 33–34 absent; orphaned p35 continuation is not an all-A/P/D range authority. No clamp or rounding.", 35))
    elif isinstance(model, Speed):
        wire = {"top_speed": "V", "start_speed": "v", "cutoff_speed": "c"}[name]
        ul = model.unit == "ul_per_second"
        wire += numeric_spelling(model.value) + (",1" if ul else ",0")
        units = model.unit
        low, high, places, page = SPEED_DOMAINS[name][model.unit]
        _domain(warnings, "value", model.value, low, high, places, page)
        if ul:
            warnings.append(_warning("device_unit_conversion", "Controller converts using configured maximum volume; host leaves value unchanged.",
                                     35 if name == "top_speed" else 36,
                                     increment_rounding="0.1" if name == "top_speed" else "1"))
        warnings.append(_warning("programmed_not_effective_speed", "Controller may temporarily limit start/cutoff to top speed while reporting programmed values.", 37))
        if name == "cutoff_speed" and Decimal(numeric_spelling(model.value)) > Decimal(100 if ul else 4000):
            warnings.append(_warning("manufacturer_recommendation", "p36 advises against cutoff above 100 ul/s; contextual pipetting guide recipes differ. No refusal.", 36))
    elif isinstance(model, SpeedDefault):
        wire = {"top": "V", "start": "v", "cutoff": "c"}[model.speed]
        units = "increments_per_second"
        warnings.append(_warning("documentary_default_not_readback", "Omitted speed parameter restores documented increment-unit default; no device value is inferred.", 35 if model.speed == "top" else 36))
    elif isinstance(model, Slope):
        wire = "L" + numeric_spelling(model.ramp_up) + "," + numeric_spelling(model.ramp_down)
        units = "slope_code_20000_increments_per_second_squared"
        for field in ("ramp_up", "ramp_down"):
            _domain(warnings, field, r[field], 1, 40, 0, 37)
    elif isinstance(model, Backlash):
        wire = "K" + numeric_spelling(model.increments)
        units = "increments"
        _domain(warnings, "increments", model.increments, 0, 500, 0, 38)
    elif isinstance(model, BeginLld):
        wire = "B" + str(model.start_mode)
        warnings.append(_warning("source_unresolved_trajectory", "B0 starts at p2; B1 starts at current plunger position. Full post-detection trajectory is not established by retained chapter 6.", 54))
    elif isinstance(model, LldParameter):
        selector, low, high, units = LLD_PARAMETERS[model.parameter]
        wire = f"p{selector}," + numeric_spelling(model.value)
        _domain(warnings, "value", model.value, low, high, 0, 55)
    elif isinstance(model, DigitalOutput):
        wire = "J" + str(int(model.level == "high"))
        warnings.append(_warning("configuration_applicability", "J applies only in general-purpose output mode U73; codec does not query or change mode.", 38))
    elif isinstance(model, EventSubscription):
        wire = f"o{EVENTS[model.event]},{('disabled', 'always', 'during_lld').index(model.mode)}"
        if model.mode == "during_lld" and model.event not in ("clld_rising", "clld_falling"):
            warnings.append(_warning("documented_applicability", "during_lld mode is documented only for cLLD events; authored command unchanged.", 54))
        warnings.append(_warning("event_precedence", "If digital and cLLD events are both enabled only cLLD events are generated. Event settings cannot be saved in NVRAM.", 54))
    elif isinstance(model, BufferControl):
        wire = {"buffer_execute": "R", "buffer_replay": "X", "buffer_clear": "C", "terminate": "T", "reset": "!R"}[name]
        suffix = ""
        wait = "control"
    elif isinstance(model, LoopStart):
        wire = "g"
        suffix, wait = "", "ack"
        warnings.append(_warning("loop_context", "Append marker only; pair with G and explicit R in a device program.", 39))
    elif isinstance(model, LoopEnd):
        wire = "G" + numeric_spelling(model.repetitions)
        suffix, wait = "", "ack"
        _domain(warnings, "repetitions", model.repetitions, 0, 50000, 0, 39)
        if Decimal(numeric_spelling(model.repetitions)) == 0:
            warnings.append(_warning("endless_loop", "G0 repeats until an explicit terminate; no automatic Stop or retry.", 39))
    elif isinstance(model, Delay):
        wire = "M" + numeric_spelling(model.milliseconds)
        units = "ms"
        _domain(warnings, "milliseconds", model.milliseconds, 0, 30000, 0, 40)
        warnings.append(_warning("device_delay_quantization", "Controller treats <10 ms as zero and otherwise rounds to nearest 10 ms; wire unchanged.", 40))
    elif isinstance(model, Halt):
        wire = f"H{int(model.resume_on == 'command_only')},{int(model.input_edge == 'rising')}"
        wait = "control"
    elif isinstance(model, (UserByteWrite, UserByteRead)):
        _domain(warnings, "location", model.location, 0, 15, 0, 38 if isinstance(model, UserByteWrite) else 44)
        wire = (">" if isinstance(model, UserByteWrite) else "<") + str(model.location)
        if isinstance(model, UserByteWrite):
            wire += "," + str(model.value)
            persistent = True
            _domain(warnings, "value", model.value, 0, 255, 0, 38)
        else:
            wait, report_kind, frame, suffix, units = "report", "user_byte", 6, "", "byte"
    elif isinstance(model, StoredProgramWrite):
        body, effects, nested_warnings = _compile_members(model.commands, stored=True)
        wire = f"s{model.location}," + body
        persistent = True
        warnings.extend(nested_warnings)
        _domain(warnings, "location", model.location, 0, 15, 0, 42)
        if len(body.encode("ascii")) > 128:
            warnings.append(_warning("documented_program_length", "Stored string exceeds 128 ASCII bytes; unchanged.", 42, length=len(body)))
    elif isinstance(model, StoredProgramRun):
        wire = "e" + str(model.location)
        wait = "control"
        _domain(warnings, "location", model.location, 0, 15, 0, 42)
        warnings.append(_warning("control_transfer", "Loads stored string into buffer and transfers control; contents and their persistence/motion effects are unknown until separately read.", 42))
    elif isinstance(model, SaveSettings):
        wire, persistent = "i", True
    elif isinstance(model, RestoreSettings):
        wire = "r" + str(int(model.source == "factory"))
    elif isinstance(model, (Status, Firmware, Report, ErrorLog, CommandLog, RawPressure)):
        wait, frame, suffix = "report", 6, ""
        if isinstance(model, Status):
            wire = "Q" + str(int(model.extended))
            report_kind = "extended_status" if model.extended else "status"
            warnings.append(_warning("source_unresolved_errors", "Status byte contains error and READY; complete extended error table pp60–64 unavailable. Q polling interval at least 100 ms (p32).", 43))
        elif isinstance(model, Firmware):
            wire = "&" + str(int(model.kind == "application"))
            report_kind = model.kind
        elif isinstance(model, Report):
            wire = "?" + str(model.selector)
            report_kind, units, _ = REPORTS[model.selector]
        elif isinstance(model, ErrorLog):
            wire, report_kind = ":" + str(model.index), "error_log"
            _domain(warnings, "index", model.index, 1, 100, 0, 44)
        elif isinstance(model, CommandLog):
            wire, report_kind = f"={model.index},{model.length}", "command_log_lifo"
            _domain(warnings, "index", model.index, 1, 1000, 0, 44)
            _domain(warnings, "length", model.length, 1, 256, 0, 44)
        else:
            wire, report_kind, units = "#", "pressure_adc_hex16_no_status", "adc_counts"
    elif isinstance(model, Configuration):
        code = CONFIGURATION_CODES[model.setting]
        wire, persistent = f"U{code}", True
        if code in RESET_REQUIRED_CODES:
            warnings.append(_warning("reset_required_for_effect", "Configuration is stored; power cycle or explicit soft reset needed for effect. No reset is added.", 50 if code < 60 else 51))
    elif isinstance(model, HeartbeatPeriod):
        wire = "b" + numeric_spelling(model.half_seconds)
        units, persistent = "half_seconds", True
        _domain(warnings, "half_seconds", model.half_seconds, 1, 20, 0, 53)
    elif isinstance(model, Diagnostic):
        selector = ("minimum_moving_current", "step_loss_detection", "pressure_sensor").index(model.test)
        wire, report_kind = f"d{selector}", "diagnostic_" + model.test
        units = "ma" if selector == 0 else None
        # Diagnostic is active, BUSY while running, and returns an answer. It is
        # NOT a frame-6 passive report; completion owner must retain its answer.
        warnings.append(_warning("active_diagnostic", "Active diagnostic may move plunger; BUSY while active and returns an answer for each d in a string.", 56))
    else:  # pragma: no cover - exhaustive union
        raise TypeError(type(model).__name__)
    return _envelope(wire + suffix, r, wait_mode=wait, report_kind=report_kind, units=units,
                     persistent=persistent, warnings=warnings, can_command_type=frame)


_NON_EMBEDDABLE = {"buffer_execute", "buffer_replay", "buffer_clear", "terminate", "reset", "stored_program_write"}


def command_effects(payload: DeviceCommand | dict) -> dict:
    """Describe authored effects only; never update physical/liquid counters."""
    c = DEVICE_COMMAND_ADAPTER.validate_python(payload)
    r = c.model_dump(mode="json")
    compiled = compile_device_command(c)
    effect = {"command": c.command, "requested": r, "persistent_write": compiled["persistent"],
              "report_kind": compiled["report_kind"], "units": compiled["units"],
              "observed": False, "completion_proven": False}
    if isinstance(c, PlungerMove):
        effect.update(kind="plunger_displacement", reference="absolute" if c.command == "plunger_absolute" else "relative",
                      direction={"plunger_absolute": None, "plunger_aspirate": "aspirate", "plunger_dispense": "dispense"}[c.command],
                      value=r["value"], delivered_liquid_measured=False)
    elif isinstance(c, BufferControl):
        effect.update(kind="buffer_control", action={"buffer_execute": "execute_or_resume", "buffer_replay": "replay_from_beginning",
                      "buffer_clear": "clear", "terminate": "terminate", "reset": "reset"}[c.command])
    elif isinstance(c, Halt):
        effect.update(kind="halt", ready_is_not_program_completion=True,
                      resume_on=c.resume_on, input_edge=c.input_edge, debounce_ms=20, additional_delay_ms_max=10)
    elif isinstance(c, BeginLld):
        effect.update(kind="lld", initial_position="p2" if c.start_mode == 0 else "current", post_detection_trajectory="source_unresolved")
    elif isinstance(c, StoredProgramRun):
        effect.update(kind="control_transfer", target_location=c.location, target_effects="unknown")
    elif isinstance(c, StoredProgramWrite):
        effect.update(kind="store_program", target_location=c.location, stored_commands_executed=False)
    elif isinstance(c, Configuration):
        code = CONFIGURATION_CODES[c.setting]
        effect.update(kind="configuration", code=code, reset_required=code in RESET_REQUIRED_CODES,
                      reset_added=False, setting=c.setting)
    elif isinstance(c, EventSubscription):
        effect.update(kind="event_subscription", event=c.event, mode=c.mode, nvram_save_supported=False)
    elif isinstance(c, HeartbeatPeriod):
        effect.update(kind="heartbeat_period", half_seconds=r["half_seconds"], auto_saved_by_device=True)
    elif isinstance(c, Diagnostic):
        effect.update(kind="active_diagnostic", returns_answer=True, may_move=True)
    elif compiled["wait_mode"] == "report":
        effect.update(kind="report", status_byte=c.command != "raw_pressure", measured_delivery=False)
    elif isinstance(c, RestoreSettings):
        effect.update(kind="restore_working_settings", source=c.source, nvram_changed=False)
    elif isinstance(c, SaveSettings):
        effect.update(kind="save_working_settings", saved_commands=["v", "V", "c", "L", "h", "K", "p"])
    elif isinstance(c, (Initialize, EjectTip)):
        effect.update(kind="initialization" if isinstance(c, Initialize) else "tip_ejection", may_move=True)
    else:
        effect.update(kind="device_setting" if c.command not in ("loop_start", "loop_end", "delay") else c.command)
    return effect


def _compile_members(commands: list, *, stored: bool = False) -> tuple[str, list, list]:
    pieces, effects, warnings = [], [], []
    depth = pairs = 0
    transferred = False
    for index, payload in enumerate(commands):
        c = DEVICE_COMMAND_ADAPTER.validate_python(payload)
        if c.command in _NON_EMBEDDABLE:
            raise ValueError(f"{c.command} is not a buffer member; submit separately")
        compiled = compile_device_command(c)
        if compiled["can_command_type"] == 6:
            raise ValueError(f"{c.command} is not a buffer member; submit separately")
        reachable = not transferred
        if transferred:
            warnings.append(_warning("unreachable_after_transfer", "Commands after e are not executed in the original string.", 42, command_index=index))
            if stored:
                raise ValueError("Stored string e must be last (30053815-C p42)")
        if c.command == "loop_start":
            depth += 1
            pairs += 1
        elif c.command == "loop_end":
            depth -= 1
            if depth < 0:
                raise ValueError("loop_end without loop_start")
        elif c.command == "stored_program_run":
            transferred = True
        pieces.append(compiled["ascii"].removesuffix("R"))
        effect = command_effects(c)
        effect.update(command_index=index, disposition="authored_not_observed",
                      reachable_in_authored_string=reachable)
        effects.append(effect)
        warnings.extend({**w, "command_index": index} for w in compiled["warnings"] if w["code"] != "loop_context")
    if depth:
        raise ValueError("Unclosed loop; loops cannot cross device-program strings")
    if pairs > 20:
        warnings.append(_warning("documented_loop_count", "Program exceeds twenty g/G pairs; unchanged.", 39, pairs=pairs))
    return "".join(pieces), effects, warnings


def compile_device_program(payload: DeviceProgram | dict) -> dict:
    program = DEVICE_PROGRAM_ADAPTER.validate_python(payload)
    r = program.model_dump(mode="json")
    body, effects, warnings = _compile_members(program.commands)
    wire = body + ("R" if program.mode == "execute" else "")
    if not wire:
        raise ValueError("Empty append has no command to encode; use buffer_execute to execute an existing buffer")
    if len(wire.encode("ascii")) > 256:
        warnings.append(_warning("documented_buffer_length", "Authored string exceeds 256 characters; unchanged. Prior buffer contents remain unknown.", 32, length=len(wire)))
    warnings.append(_warning("prior_buffer_unknown", "Append/execute can extend a previously loaded buffer; codec neither clears nor reads it automatically.", 32))
    special = not effects or any(e["kind"] in ("halt", "control_transfer") or e["command"] == "loop_end" and Decimal(numeric_spelling(e["requested"]["repetitions"])) == 0 for e in effects)
    result = _envelope(wire, r, wait_mode="ack" if program.mode == "append" else ("control" if special else "completion"),
                       persistent=program.mode == "execute" and any(e["persistent_write"] and e["reachable_in_authored_string"] for e in effects), warnings=warnings)
    result.update(program=r, command_effects=effects, buffer_action="append" if program.mode == "append" else "append_and_execute",
                  effects_deferred=program.mode == "append")
    return result


def device_command_schema() -> dict:
    return DEVICE_COMMAND_ADAPTER.json_schema()


def device_program_schema() -> dict:
    return DEVICE_PROGRAM_ADAPTER.json_schema()


def device_command_catalog() -> dict:
    # A copy keeps data-only consumers from mutating this producer's tables.
    from .cavro_command_catalog import FAMILY_COVERAGE
    return deepcopy({"schema_version": 1, "generation": "original_adp", "manual": "30053815-C",
                     "families": FAMILY_COVERAGE,
                     "numeric_domains": _numeric_domain_catalog(),
                     "source_unresolved_numeric_domains": ["plunger_absolute", "plunger_aspirate", "plunger_dispense", "drive_current_setter"],
                     "reduced_pressure_report": {"command": "raw_pressure", "ascii": "#", "can_command_type": 6,
                         "status_byte": False, "payload": "four ASCII hexadecimal digits, signed 16-bit two's-complement ADC counts",
                         "units": "adc_counts", "source": "30053815-C", "printed_pages": [49, 50],
                         "oem_rs485_etx_present": False, "oem_sequence_optional": True, "oem_repeat_flag_allowed": False},
                     "reports": [{"selector": n, "name": v[0], "units": v[1], "printed_page": v[2],
                                  "fields": REPORT_FIELDS.get(n), "can_command_type": 6,
                                  "status_byte": True} for n, v in REPORTS.items()],
                     "configuration": [{"setting": name, "code": code, "persistent": True,
                                        "reset_required": code in RESET_REQUIRED_CODES} for name, code in CONFIGURATION_CODES.items()],
                     "lld_parameters": [{"parameter": name, "selector": v[0], "minimum": v[1],
                                         "maximum": v[2], "units": v[3], "printed_page": 55}
                                        for name, v in LLD_PARAMETERS.items()],
                     "semantics": {"ack_is_completion": False, "persistent_means": "explicit NVRAM write requested, not verified",
                                   "append_effects": "deferred until explicit execution", "halt_ready_is_completion": False,
                                   "automatic_reset": False, "automatic_retry": False,
                                   "reports_while_busy": True, "q_minimum_poll_ms": 100,
                                   "error_during_execution": "terminates execution and erases buffer",
                                   "delay_controls": {"R": "ends delay and continues", "T": "ends delay and halts; R resumes remainder", "X": "error14 BUSY"},
                                   "terminate": "graceful A/P/D/B/W/M; other individual commands complete first; X replays beginning",
                                   "reset": "highest priority, cannot embed, ! immediately followed by R"}})


def device_command_examples() -> dict:
    commands = [
        {"command": "plunger_aspirate", "value": "1.175", "unit": "ul"},
        {"command": "plunger_absolute", "value": 100, "unit": "increments"},
        {"command": "report", "selector": 57}, {"command": "raw_pressure"},
        {"command": "begin_lld", "start_mode": 1},
        {"command": "heartbeat_period", "half_seconds": 15},
        {"command": "configuration", "setting": "output_general_purpose"},
        {"command": "restore_settings", "source": "factory"},
        {"command": "buffer_clear"}, {"command": "terminate"},
        {"command": "diagnostic", "test": "minimum_moving_current"},
        {"command": "stored_program_write", "location": 2, "commands": [
            {"command": "loop_start"}, {"command": "delay", "milliseconds": 20},
            {"command": "loop_end", "repetitions": 2}]},
    ]
    program = {"mode": "append", "commands": [{"command": "top_speed", "value": "50.000", "unit": "ul_per_second"},
                {"command": "plunger_aspirate", "value": "1.175", "unit": "ul"}]}
    return {"commands": [{"request": c, "compiled": compile_device_command(c), "effect": command_effects(c)} for c in commands],
            "programs": [{"request": program, "compiled": compile_device_program(program)}]}
