"""Original ADP source vectors; pure codec, no controller observations.

Run with unshare -Urn and the unchanged z_stop_offline_guard pytest plugin.
30053815-C page citations are assertions' source, not simulated hardware proof.
"""
import json
import os
from decimal import Decimal
from pathlib import Path

import pytest
from pydantic import ValidationError

from bioxp.pipette.cavro_commands import (
    DEVICE_COMMAND_ADAPTER, DEVICE_PROGRAM_ADAPTER, REPORTS,
    compile_device_command as command, compile_device_program as program,
    command_effects, device_command_catalog, device_command_examples,
    device_command_schema, device_program_schema, numeric_spelling,
)


ENVELOPE = {"ascii", "can_command_type", "wait_mode", "report_kind", "units", "persistent", "warnings", "requested"}


# Explicit expected wires, not generated from producer tables. pp35–56 and
# publisher 399156 p2 (WR/ER). Reports deliberately have no execute suffix.
COMMAND_CASES = [
    ({"command": "initialize"}, "WR", "completion", False),
    ({"command": "eject_tip"}, "ER", "completion", False),
    ({"command": "plunger_absolute", "value": "1.175", "unit": "ul"}, "A1.175,1R", "completion", False),
    ({"command": "plunger_aspirate", "value": 42, "unit": "increments"}, "P42,0R", "completion", False),
    ({"command": "plunger_dispense", "value": "0.025", "unit": "ul"}, "D0.025,1R", "completion", False),
    ({"command": "top_speed", "value": "50.125", "unit": "ul_per_second"}, "V50.125,1R", "completion", False),
    ({"command": "start_speed", "value": 1000, "unit": "increments_per_second"}, "v1000,0R", "completion", False),
    ({"command": "cutoff_speed", "value": "50.000", "unit": "ul_per_second"}, "c50.000,1R", "completion", False),
    ({"command": "speed_default", "speed": "top"}, "VR", "completion", False),
    ({"command": "speed_default", "speed": "start"}, "vR", "completion", False),
    ({"command": "speed_default", "speed": "cutoff"}, "cR", "completion", False),
    ({"command": "slope", "ramp_up": 20, "ramp_down": 10}, "L20,10R", "completion", False),
    ({"command": "backlash", "increments": 50}, "K50R", "completion", False),
    ({"command": "begin_lld", "start_mode": 0}, "B0R", "completion", False),
    ({"command": "begin_lld", "start_mode": 1}, "B1R", "completion", False),
    ({"command": "digital_output", "level": "high"}, "J1R", "completion", False),
    ({"command": "event_subscription", "event": "tip_loss", "mode": "always"}, "o1,1R", "completion", False),
    ({"command": "buffer_execute"}, "R", "control", False),
    ({"command": "buffer_replay"}, "X", "control", False),
    ({"command": "buffer_clear"}, "C", "control", False),
    ({"command": "terminate"}, "T", "control", False),
    ({"command": "reset"}, "!R", "control", False),
    ({"command": "loop_start"}, "g", "ack", False),
    ({"command": "loop_end", "repetitions": 2}, "G2", "ack", False),
    ({"command": "delay", "milliseconds": 37}, "M37R", "completion", False),
    ({"command": "halt", "resume_on": "command_or_input", "input_edge": "falling"}, "H0,0R", "control", False),
    ({"command": "halt", "resume_on": "command_only", "input_edge": "rising"}, "H1,1R", "control", False),
    ({"command": "user_byte_write", "location": 15, "value": 255}, ">15,255R", "completion", True),
    ({"command": "user_byte_read", "location": 15}, "<15", "report", False),
    ({"command": "stored_program_write", "location": 2, "commands": []}, "s2,R", "completion", True),
    ({"command": "stored_program_run", "location": 2}, "e2R", "control", False),
    ({"command": "save_settings"}, "iR", "completion", True),
    ({"command": "restore_settings", "source": "nvram"}, "r0R", "completion", False),
    ({"command": "restore_settings", "source": "factory"}, "r1R", "completion", False),
    ({"command": "status"}, "Q0", "report", False),
    ({"command": "status", "extended": True}, "Q1", "report", False),
    ({"command": "firmware", "kind": "part_revision"}, "&0", "report", False),
    ({"command": "firmware", "kind": "application"}, "&1", "report", False),
    ({"command": "report", "selector": 57}, "?57", "report", False),
    ({"command": "error_log", "index": 100}, ":100", "report", False),
    ({"command": "command_log", "index": 1000, "length": 256}, "=1000,256", "report", False),
    ({"command": "raw_pressure"}, "#", "report", False),
    ({"command": "configuration", "setting": "input_emergency_brake"}, "U75R", "completion", True),
    ({"command": "heartbeat_period", "half_seconds": 15}, "b15R", "completion", True),
    ({"command": "diagnostic", "test": "minimum_moving_current"}, "d0R", "completion", False),
    ({"command": "diagnostic", "test": "step_loss_detection"}, "d1R", "completion", False),
    ({"command": "diagnostic", "test": "pressure_sensor"}, "d2R", "completion", False),
]


@pytest.mark.parametrize("payload,wire,wait,persistent", COMMAND_CASES)
def test_original_source_wires_and_exact_envelope(payload, wire, wait, persistent):
    result = command(payload)
    assert set(result) == ENVELOPE
    assert result["ascii"] == wire
    assert result["can_command_type"] == (6 if wait == "report" else 1)
    assert result["wait_mode"] == wait
    assert result["persistent"] is persistent
    assert result["requested"] == DEVICE_COMMAND_ADAPTER.validate_python(payload).model_dump(mode="json")
    assert command(result["requested"]) == result
    assert json.loads(json.dumps(result)) == result


@pytest.mark.parametrize("name,value,wire,units", [
    ("threshold_adc", 15, "p0,15R", "adc_counts"),
    ("threshold_duration_ms", 5, "p1,5R", "ms"),
    ("start_position_increments", 0, "p2,0R", "increments"),
    ("speed_increments_per_second", 150, "p3,150R", "increments_per_second"),
    ("maximum_travel_increments", 44000, "p4,44000R", "increments"),
    ("direction", 1, "p5,1R", None),
    ("sample_interval_ms", 20, "p6,20R", "ms"),
    ("samples_per_can_message", 3, "p7,3R", "samples"),
    ("hybrid_interval_ms", 50, "p8,50R", "ms"),
])
def test_all_pressure_parameters_p55(name, value, wire, units):
    result = command({"command": "lld_parameter", "parameter": name, "value": value})
    assert (result["ascii"], result["units"], result["persistent"]) == (wire, units, False)


CONFIGS = [
    ("rs485_ch1_commands", 1), ("rs485_ch2_commands", 2), ("rs485_ch1_tip_loss", 3),
    ("startup_program_enabled", 30), ("startup_program_disabled", 31),
    ("rs485_protocol_auto", 35), ("rs485_protocol_oem", 36), ("rs485_protocol_dt", 37),
    ("rs485_baud_9600", 41), ("rs485_baud_38400", 47), ("rs485_baud_115200", 48),
    ("can_baud_100k", 51), ("can_baud_125k", 52), ("can_baud_250k", 53),
    ("can_baud_500k", 54), ("can_baud_1m", 55),
    ("can_heartbeat_enabled", 60), ("can_heartbeat_disabled", 61),
    ("output_plld", 70), ("output_clld_mirror", 71), ("output_hlld", 72),
    ("output_general_purpose", 73), ("input_general_purpose", 74),
    ("input_emergency_brake", 75), ("lld_active_low", 76), ("lld_active_high", 77),
    ("rs485_stream_enabled", 80), ("rs485_stream_disabled", 81), ("rs485_stream_oem", 82),
    ("rs485_stream_dt", 83), ("pressure_module_enabled", 90), ("pressure_module_disabled", 91),
    ("tip_module_enabled", 92), ("tip_module_disabled", 93),
]


@pytest.mark.parametrize("setting,code", CONFIGS)
def test_configuration_persistence_and_no_implicit_reset_pp50_52(setting, code):
    result = command({"command": "configuration", "setting": setting})
    assert result["ascii"] == f"U{code}R"
    assert result["persistent"] is True
    reset_codes = {1, 2, 3, 30, 31, 35, 36, 37, 41, 47, 48, 51, 52, 53, 54, 55, 90, 91, 92, 93}
    assert ("reset_required_for_effect" in {w["code"] for w in result["warnings"]}) == (code in reset_codes)
    assert "!" not in result["ascii"]


@pytest.mark.parametrize("event,number", [("pressure", 0), ("tip_loss", 1), ("digital_rising", 2),
    ("digital_falling", 3), ("clld_rising", 4), ("clld_falling", 5), ("lld_success", 6)])
@pytest.mark.parametrize("mode,number2", [("disabled", 0), ("always", 1), ("during_lld", 2)])
def test_all_event_choices_p54_no_new_mode_admission_gate(event, number, mode, number2):
    result = command({"command": "event_subscription", "event": event, "mode": mode})
    assert result["ascii"] == f"o{number},{number2}R"
    assert result["persistent"] is False
    warning_codes = {w["code"] for w in result["warnings"]}
    assert ("documented_applicability" in warning_codes) == (number2 == 2 and number not in (4, 5))


REPORT_SELECTORS = [0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 15, 16, 17, 18, 19, 20,
    23, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44,
    45, 46, 47, 48, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65,
    67, 76, 77, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 99]


@pytest.mark.parametrize("selector", REPORT_SELECTORS)
def test_every_published_report_is_report_not_actuator(selector):
    result = command({"command": "report", "selector": selector})
    assert result["ascii"] == f"?{selector}"
    assert result["can_command_type"] == 6
    assert result["wait_mode"] == "report"
    assert result["report_kind"]
    assert not result["persistent"]
    effect = command_effects(result["requested"])
    assert effect["kind"] == "report"
    assert effect["status_byte"] is True
    assert effect["measured_delivery"] is False


@pytest.mark.parametrize("selector,units", [(0, "increments"), (1, "increments"), (2, "mm"),
    (3, "ul"), (11, None), (12, None), (15, None), (17, "ul"), (28, "seconds"),
    (32, "adc_counts"), (40, "minutes"), (44, "meters"), (50, "volts"),
    (53, "fahrenheit"), (56, "adc_counts"), (57, "psi"), (58, "psi"), (59, "psi")])
def test_report_units_do_not_confuse_adc_psi_or_delivery(selector, units):
    assert command({"command": "report", "selector": selector})["units"] == units


def test_reduced_pressure_and_active_diagnostics_are_not_ordinary_reports():
    raw = command({"command": "raw_pressure"})
    assert raw["report_kind"] == "pressure_adc_hex16_no_status"
    assert raw["units"] == "adc_counts"
    assert command_effects(raw["requested"])["status_byte"] is False
    diagnostic = command({"command": "diagnostic", "test": "minimum_moving_current"})
    assert diagnostic["can_command_type"] == 1
    assert diagnostic["wait_mode"] == "completion"
    assert diagnostic["report_kind"] == "diagnostic_minimum_moving_current"
    assert diagnostic["units"] == "ma"
    assert command_effects(diagnostic["requested"])["kind"] == "active_diagnostic"
    assert command_effects(diagnostic["requested"])["returns_answer"] is True


@pytest.mark.parametrize("value,wire", [("1.175", "1.175"), ("01.1750", "01.1750"),
    ("0.30000000000000004", "0.30000000000000004"),
    (0.1 + 0.2, "0.30000000000000004"), (1e-7, "0.0000001"), (-0.0, "-0.0"),
    (".025", ".025"), ("50.0000", "50.0000"), ("1101", "1101"), (-1, "-1")])
def test_numeric_spelling_preserved_not_scientifically_rounded(value, wire):
    assert numeric_spelling(value) == wire
    result = command({"command": "plunger_aspirate", "value": value, "unit": "ul"})
    assert result["ascii"] == f"P{wire},1R"
    assert result["requested"]["value"] == value
    assert {w["code"] for w in result["warnings"]} == {"source_unresolved_domain"}


@pytest.mark.parametrize("payload,wire,code,page", [
    ({"command": "top_speed", "value": 1001, "unit": "ul_per_second"}, "V1001,1R", "documented_range", 35),
    ({"command": "top_speed", "value": "50.1234", "unit": "ul_per_second"}, "V50.1234,1R", "documented_precision", 35),
    ({"command": "top_speed", "value": "8000.01", "unit": "increments_per_second"}, "V8000.01,0R", "documented_precision", 35),
    ({"command": "slope", "ramp_up": 0, "ramp_down": 41}, "L0,41R", "documented_range", 37),
    ({"command": "backlash", "increments": 501}, "K501R", "documented_range", 38),
    ({"command": "heartbeat_period", "half_seconds": 21}, "b21R", "documented_range", 53),
    ({"command": "lld_parameter", "parameter": "sample_interval_ms", "value": 0}, "p6,0R", "documented_range", 55),
    ({"command": "command_log", "index": 0, "length": 257}, "=0,257", "documented_range", 44),
    ({"command": "user_byte_write", "location": 16, "value": 256}, ">16,256R", "documented_range", 38),
])
def test_numeric_domains_are_nonblocking_source_diagnostics(payload, wire, code, page):
    result = command(payload)
    assert result["ascii"] == wire
    assert any(w["code"] == code and w["printed_page"] == page for w in result["warnings"])
    assert all(w["blocking"] is False and w["source"] == "30053815-C" for w in result["warnings"])


@pytest.mark.parametrize("value", [True, False, float("nan"), float("inf"), "nan", "1e3", "1R!", "1,0", "1\n", " 1", None])
def test_new_numeric_forms_reject_structural_injection_and_nonfinite(value):
    with pytest.raises(ValidationError):
        command({"command": "plunger_aspirate", "value": value, "unit": "ul"})


@pytest.mark.parametrize("payload", [{"command": "raw", "ascii": "WR"},
    {"command": "report", "selector": 5}, {"command": "report", "selector": 100},
    {"command": "drive_current", "value": 10}, {"command": "initialize", "variant": 1},
    {"command": "eject_tip", "variant": 0}, {"command": "configuration", "setting": "can_common_boot"},
    {"command": "plunger_aspirate", "value": 1, "unit": "ml"}])
def test_source_unresolved_variants_not_fabricated(payload):
    with pytest.raises(ValidationError):
        command(payload)


def test_meaningful_displacement_vs_liquid_and_lld_initial_position_effects():
    for name, reference, direction in [("plunger_absolute", "absolute", None),
        ("plunger_aspirate", "relative", "aspirate"), ("plunger_dispense", "relative", "dispense")]:
        effect = command_effects({"command": name, "value": "1.175", "unit": "ul"})
        assert effect["reference"] == reference and effect["direction"] == direction
        assert effect["value"] == "1.175" and effect["units"] == "ul"
        assert effect["delivered_liquid_measured"] is False
        assert effect["observed"] is False and effect["completion_proven"] is False
    assert command_effects({"command": "begin_lld", "start_mode": 0})["initial_position"] == "p2"
    assert command_effects({"command": "begin_lld", "start_mode": 1})["initial_position"] == "current"


def test_working_settings_restore_and_nvram_writes_distinct():
    save = command_effects({"command": "save_settings"})
    assert save["persistent_write"] is True
    assert save["saved_commands"] == ["v", "V", "c", "L", "h", "K", "p"]
    restore = command_effects({"command": "restore_settings", "source": "factory"})
    assert restore["kind"] == "restore_working_settings"
    assert restore["persistent_write"] is False and restore["nvram_changed"] is False
    assert command({"command": "heartbeat_period", "half_seconds": 15})["persistent"] is True
    assert command({"command": "event_subscription", "event": "pressure", "mode": "always"})["persistent"] is False


MOVES = [{"command": "top_speed", "value": "50.000", "unit": "ul_per_second"},
         {"command": "plunger_aspirate", "value": "1.175", "unit": "ul"}]


def test_program_append_execute_and_commands_are_not_interchangeable():
    append = program({"mode": "append", "commands": MOVES})
    execute = program({"mode": "execute", "commands": MOVES})
    assert append["ascii"] == "V50.000,1P1.175,1"
    assert execute["ascii"] == append["ascii"] + "R"
    assert append["wait_mode"] == "ack" and execute["wait_mode"] == "completion"
    assert append["buffer_action"] == "append" and execute["buffer_action"] == "append_and_execute"
    assert append["effects_deferred"] is True and execute["effects_deferred"] is False
    assert execute["program"] == execute["requested"] == {"mode": "execute", "commands": MOVES}
    assert execute["command_effects"][1]["kind"] == "plunger_displacement"
    assert all(e["observed"] is False for e in execute["command_effects"])
    assert command({"command": "buffer_execute"})["ascii"] == "R"
    assert command({"command": "buffer_replay"})["ascii"] == "X"
    assert command({"command": "buffer_clear"})["ascii"] == "C"
    assert "C" not in execute["ascii"]  # no implicit clear of unknown prior buffer


def test_nested_and_independent_loops_preserve_device_program_not_host_expansion():
    requests = [{"command": "loop_start"}, {"command": "loop_start"}, MOVES[1],
                {"command": "loop_end", "repetitions": 3}, {"command": "loop_end", "repetitions": 2},
                {"command": "loop_start"}, {"command": "delay", "milliseconds": 20},
                {"command": "loop_end", "repetitions": 1}]
    result = program({"mode": "execute", "commands": requests})
    assert result["ascii"] == "ggP1.175,1G3G2gM20G1R"
    assert len(result["command_effects"]) == len(requests)
    assert sum(e["kind"] == "plunger_displacement" for e in result["command_effects"]) == 1
    assert "loop_context" not in {w["code"] for w in result["warnings"]}


def test_endless_loop_halt_and_control_transfer_not_false_completion():
    loop = program({"mode": "execute", "commands": [{"command": "loop_start"}, MOVES[1],
                    {"command": "loop_end", "repetitions": 0}]})
    assert loop["ascii"] == "gP1.175,1G0R"
    assert loop["wait_mode"] == "control"
    assert "endless_loop" in {w["code"] for w in loop["warnings"]}
    halted = program({"mode": "execute", "commands": [{"command": "halt"}, MOVES[1]]})
    assert halted["ascii"] == "H0,0P1.175,1R" and halted["wait_mode"] == "control"
    assert halted["command_effects"][0]["ready_is_not_program_completion"] is True
    transfer = program({"mode": "execute", "commands": [{"command": "stored_program_run", "location": 2}, MOVES[1]]})
    assert transfer["ascii"] == "e2P1.175,1R" and transfer["wait_mode"] == "control"
    assert "unreachable_after_transfer" in {w["code"] for w in transfer["warnings"]}
    assert transfer["command_effects"][0]["target_effects"] == "unknown"


@pytest.mark.parametrize("payload", [
    {"command": "report", "selector": 57}, {"command": "status"}, {"command": "raw_pressure"},
    {"command": "buffer_execute"}, {"command": "buffer_replay"}, {"command": "buffer_clear"},
    {"command": "terminate"}, {"command": "reset"},
    {"command": "stored_program_write", "location": 0, "commands": []},
])
def test_reports_and_immediate_controls_cannot_be_embedded(payload):
    with pytest.raises(ValueError, match="not a buffer member"):
        program({"mode": "execute", "commands": [payload]})


@pytest.mark.parametrize("commands", [[{"command": "loop_start"}], [{"command": "loop_end", "repetitions": 2}]])
def test_structural_loops_must_balance(commands):
    with pytest.raises(ValueError, match="loop"):
        program({"mode": "append", "commands": commands})


def test_program_persistence_is_deferred_for_append_only():
    requests = [{"command": "heartbeat_period", "half_seconds": 15}, {"command": "save_settings"}]
    append = program({"mode": "append", "commands": requests})
    execute = program({"mode": "execute", "commands": requests})
    assert append["ascii"] == "b15i" and append["persistent"] is False
    assert execute["ascii"] == "b15iR" and execute["persistent"] is True
    assert all(e["persistent_write"] for e in append["command_effects"])
    assert append["effects_deferred"] is True


def test_store_is_not_execute_and_stored_transfer_has_no_return():
    request = {"command": "stored_program_write", "location": 3, "commands": [
        {"command": "loop_start"}, MOVES[1], {"command": "loop_end", "repetitions": 2},
        {"command": "stored_program_run", "location": 4}]}
    result = command(request)
    assert result["ascii"] == "s3,gP1.175,1G2e4R"
    effect = command_effects(request)
    assert effect["stored_commands_executed"] is False
    assert effect["target_location"] == 3 and effect["persistent_write"] is True
    with pytest.raises(ValueError, match="must be last"):
        command({"command": "stored_program_write", "location": 0, "commands": [
            {"command": "stored_program_run", "location": 1}, MOVES[1]]})
    with pytest.raises(ValueError, match="Unclosed loop"):
        command({"command": "stored_program_write", "location": 0, "commands": [{"command": "loop_start"}]})


def test_oversize_programs_and_loop_counts_warn_without_silent_truncation():
    requests = [{"command": "delay", "milliseconds": 30000}] * 50
    result = program({"mode": "execute", "commands": requests})
    assert result["ascii"] == "M30000" * 50 + "R"
    assert "documented_buffer_length" in {w["code"] for w in result["warnings"]}
    stored = command({"command": "stored_program_write", "location": 0, "commands": requests})
    assert stored["ascii"] == "s0," + "M30000" * 50 + "R"
    assert "documented_program_length" in {w["code"] for w in stored["warnings"]}
    many_pairs = [{"command": "loop_start"}, {"command": "loop_end", "repetitions": 1}] * 21
    assert "documented_loop_count" in {w["code"] for w in program({"mode": "execute", "commands": many_pairs})["warnings"]}


def test_device_timing_and_control_semantics_source_catalog_not_new_host_policy():
    timing = command({"command": "delay", "milliseconds": 7})
    assert timing["ascii"] == "M7R"
    assert "device_delay_quantization" in {w["code"] for w in timing["warnings"]}
    semantics = device_command_catalog()["semantics"]
    assert semantics["ack_is_completion"] is False
    assert semantics["halt_ready_is_completion"] is False
    assert semantics["automatic_reset"] is False and semantics["automatic_retry"] is False
    assert semantics["delay_controls"]["X"] == "error14 BUSY"
    assert semantics["error_during_execution"] == "terminates execution and erases buffer"
    for name, action in [("buffer_execute", "execute_or_resume"), ("buffer_replay", "replay_from_beginning"),
                         ("buffer_clear", "clear"), ("terminate", "terminate"), ("reset", "reset")]:
        assert command_effects({"command": name})["action"] == action


def test_catalog_exactly_accounts_all_29_families_and_named_residuals():
    catalog = device_command_catalog()
    families = {r["id"]: r for r in catalog["families"]}
    assert len(catalog["families"]) == len(families) == 29
    assert set(families) == {f"CF-{n:02d}" for n in range(1, 30)}
    assert families["CF-06"]["command_names"] == []
    assert families["CF-29"]["command_names"] == []
    assert families["CF-06"]["source_residual"]
    assert families["CF-29"]["source_residual"]
    assert {r["id"] for r in catalog["families"] if r["source_residual"]} == {
        "CF-01", "CF-02", "CF-06", "CF-07", "CF-18", "CF-26", "CF-27", "CF-29"}
    assert set(REPORTS) == set(REPORT_SELECTORS)
    assert all(r["sources"] and r["prior_ids"] for r in families.values())
    catalog["families"].clear()
    assert len(device_command_catalog()["families"]) == 29


def test_real_schemas_and_examples_round_trip_with_producer():
    schema = device_command_schema()
    assert schema["discriminator"]["propertyName"] == "command"
    assert schema["$defs"]["PlungerMove"]["properties"]["unit"]["enum"] == ["increments", "ul"]
    assert set(schema["$defs"]["Report"]["properties"]["selector"]["enum"]) == set(REPORT_SELECTORS)
    assert schema["$defs"]["Configuration"]["properties"]["setting"]["enum"] == [x[0] for x in CONFIGS]
    assert device_program_schema()["properties"]["mode"]["enum"] == ["append", "execute"]
    examples = device_command_examples()
    for example in examples["commands"]:
        assert command(example["request"]) == example["compiled"]
        assert command_effects(example["request"]) == example["effect"]
        assert DEVICE_COMMAND_ADAPTER.validate_json(json.dumps(example["request"]))
    for example in examples["programs"]:
        assert program(example["request"]) == example["compiled"]
        assert DEVICE_PROGRAM_ADAPTER.validate_json(json.dumps(example["request"]))
    assert json.loads(json.dumps(device_command_catalog())) == device_command_catalog()


def test_empty_execute_is_explicit_resume_but_empty_append_is_not_a_wire_command():
    assert program({"mode": "execute", "commands": []})["ascii"] == "R"
    assert program({"mode": "execute", "commands": []})["wait_mode"] == "control"
    with pytest.raises(ValueError, match="Empty append"):
        program({"mode": "append", "commands": []})


def test_decimal_python_input_never_passes_through_binary_float():
    result = command({"command": "plunger_aspirate", "value": Decimal("1.1750000000000000000001"), "unit": "ul"})
    assert result["ascii"] == "P1.1750000000000000000001,1R"
    assert result["requested"]["value"] == "1.1750000000000000000001"


@pytest.mark.parametrize("selector", [True, False, 1.0, "1"])
def test_selectors_do_not_coerce_boolean_float_or_string(selector):
    with pytest.raises(ValidationError):
        command({"command": "report", "selector": selector})
    with pytest.raises(ValidationError):
        command({"command": "begin_lld", "start_mode": selector})


def test_unreachable_program_effects_do_not_claim_the_requested_write_executes():
    result = program({"mode": "execute", "commands": [
        {"command": "stored_program_run", "location": 2}, {"command": "save_settings"}]})
    assert result["ascii"] == "e2iR"  # preserve authored order, do not rewrite
    assert result["persistent"] is False  # e target remains unknown, not claimed nonpersistent
    assert result["command_effects"][0]["target_effects"] == "unknown"
    assert result["command_effects"][1]["persistent_write"] is True
    assert result["command_effects"][1]["reachable_in_authored_string"] is False


def test_every_discriminator_has_a_source_vector():
    covered = {row[0]["command"] for row in COMMAND_CASES} | {"lld_parameter"}
    assert covered == set(device_command_schema()["discriminator"]["mapping"])


@pytest.mark.parametrize("domain", device_command_catalog()["numeric_domains"])
def test_exported_numeric_domains_agree_with_nonblocking_codec(domain):
    if domain["command"] == "lld_parameter":
        payload = {"command": "lld_parameter", "parameter": domain["parameter"], "value": 0}
    else:
        payload = next(dict(row[0]) for row in COMMAND_CASES if row[0]["command"] == domain["command"])
    if domain["command"] in ("top_speed", "start_speed", "cutoff_speed"):
        payload["unit"] = domain["units"]
    payload[domain["field"]] = int(Decimal(domain["maximum"])) + 1
    result = command(payload)
    warning = next(w for w in result["warnings"] if w["code"] == "documented_range" and w["field"] == domain["field"])
    assert Decimal(warning["minimum"]) == Decimal(domain["minimum"])
    assert Decimal(warning["maximum"]) == Decimal(domain["maximum"])
    assert warning["printed_page"] == domain["printed_page"]
    assert warning["blocking"] is domain["blocking"] is False


def test_export_actual_producer_contracts_and_examples(tmp_path):
    output = Path(os.environ.get("NON_OEM_NATIVE_EXPORT_DIR", str(tmp_path))) / "cavro-commands"
    output.mkdir(parents=True, exist_ok=True)
    exports = {"device-command.schema.json": device_command_schema(),
               "device-program.schema.json": device_program_schema(),
               "catalog.json": device_command_catalog(), "examples.json": device_command_examples()}
    for name, value in exports.items():
        path = output / name
        path.write_text(json.dumps(value, indent=2) + "\n")
        assert json.loads(path.read_text()) == value
