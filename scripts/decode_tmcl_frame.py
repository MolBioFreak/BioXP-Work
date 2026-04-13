#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import json
import re
import struct
import sys
from collections import defaultdict, deque
from pathlib import Path
from typing import Any, Iterable

TMCL_STATUS = {
    100: "Success",
    1: "Wrong checksum",
    2: "Invalid command",
    3: "Wrong type",
    4: "Invalid value",
    5: "EEPROM locked",
    6: "Command not available",
    7: "Busy",
    8: "Temperature above maximum allowed",
    9: "Temperature below minimum allowed",
    10: "Nest RTD delta greater than allowed",
    11: "Current exceeded maximum allowed",
    12: "Temperature diverged from reference",
    13: "Voltage drop",
    14: "Max current reference deviation exceeded",
    128: "Target position reached",
    129: "Not initialized",
    130: "Stall guard detected",
    132: "Door/Latch sensor changed",
    133: "RTD failure",
}

BOARD_NAMES = {
    0x04: "HEAD_BOARD",
    0x05: "DECK_BOARD",
    0x06: "THERMAL_CONTROLLER",
    0x07: "CHILLER_BOARD",
}

AXIS_NAMES = {
    0x04: {0: "Y", 1: "Z", 2: "GRIPPER"},
    0x05: {0: "X"},
    0x06: {0: "THERMAL_DOOR"},
    0x07: {
        0: "REAGENT_CHILLER",
        1: "OUTPUT_CHILLER",
        3: "OUTPUT_TEMP_RTD",
        4: "OUTPUT_PEDESTAL_RTD",
    },
}

CMD_NAMES = {
    1: "ROR",
    2: "ROL",
    3: "MST",
    4: "MVP",
    5: "SAP",
    6: "GAP",
    7: "STAP",
    8: "RSAP",
    9: "SGP",
    10: "GGP",
    11: "STGP",
    12: "RSGP",
    13: "RFS",
    14: "SIO",
    15: "GIO",
    30: "SCO",
    31: "GCO",
    32: "CCO",
    33: "AAP",
    50: "SET_LED",
    64: "ACTIVATE_BOARD",
    138: "QUERY_MOTOR_STOP",
    140: "SET_TARGET_TEMPERATURE",
    141: "SET_HEATSINK_FAN",
    143: "READ_SENSOR_TEMPERATURE",
    144: "SET_PWM_OVERRIDE",
    153: "DUMP_TC_HISTORY",
    173: "QUERY_FIRMWARE_VERSION",
}

MVP_TYPES = {
    0: "ABSOLUTE_OR_HOME",
    1: "RELATIVE",
    2: "COORDINATE",
}

RFS_TYPES = {
    0: "START_REFERENCE_SEARCH",
    1: "STOP_REFERENCE_SEARCH",
    2: "QUERY_REFERENCE_SEARCH",
}

LED_MASKS = {
    0: "RED",
    1: "GREEN",
    2: "BLUE",
    3: "TOP_OR_LOGO",
}

MOTOR_PARAMS = {
    1: "ACTUAL_POSITION",
    3: "ACTUAL_SPEED",
    4: "MAX_SPEED",
    5: "MAX_ACCELERATION",
    6: "RUN_CURRENT",
    7: "STANDBY_CURRENT",
    8: "TARGET_REACHED",
    9: "LEFT_SWITCH_STATE",
    10: "RIGHT_SWITCH_STATE",
    12: "RIGHT_SWITCH_DISABLE",
    13: "LEFT_SWITCH_DISABLE",
    138: "RAMP_MODE",
    153: "RDIV",
    154: "PDIV",
    162: "CHOPPER_BLANK_TIME",
    163: "CHOPPER_CONST_TOFF",
    164: "DISABLE_SHORT_TO_GND_PROTECTION",
    165: "HIGH_SIDE_SENSE",
    166: "CHOPPER_MODE",
    167: "HYSTERESIS_DECREMENT",
    168: "HYSTERESIS_END",
    169: "HYSTERESIS_START",
    170: "CHOPPER_OFF_TIME",
    171: "SMART_ENERGY_MIN_CURRENT",
    172: "SMART_ENERGY_CURRENT_DOWN_STEP",
    173: "SMART_ENERGY_HYSTERESIS",
    174: "SMART_ENERGY_CURRENT_UP_STEP",
    175: "SMART_ENERGY_HYSTERESIS_START",
    176: "SMART_ENERGY_HYSTERESIS_END",
    177: "STALLGUARD_FILTER",
    178: "STALLGUARD_THRESHOLD",
    179: "SLOPE_CONTROL_HIGH_SIDE",
    180: "SLOPE_CONTROL_LOW_SIDE",
    181: "SHORT_DETECT_TIMER",
    182: "SHORT_DETECT_RECOVERY_TIME",
    204: "FREEWHEELING",
    205: "STALLGUARD_THRESHOLD_OEM",
}

THERMAL_GP_PARAMS = {
    4: "CURRENT_TEMPERATURE_MILLIC",
    2: "MAX_HEAT_CURRENT_MILLIAMPS",
    3: "MAX_COOL_CURRENT_MILLIAMPS",
    7: "HEAT_RAMP_MILLIC_PER_S",
    8: "COOL_RAMP_MILLIC_PER_S",
    9: "PROPORTIONAL_GAIN_X1000",
    10: "INTEGRAL_GAIN_X1000",
    11: "DERIVATIVE_GAIN_X1000",
    12: "FEED_FORWARD_GAIN_X1000",
    13: "TEC_CURRENT_MILLIAMPS",
    14: "CURRENT_REFERENCE_MILLIAMPS",
    19: "TEMPERATURE_GAIN_RAW",
    20: "TEMPERATURE_OFFSET_RAW",
    21: "FAN_SPEED_RAW",
    22: "TEMPERATURE_REFERENCE_MILLIC",
    23: "PWM_OVERRIDE_PERCENT",
}

CHILLER_GP_PARAMS = {
    4: "CURRENT_TEMPERATURE_MILLIC",
    2: "MAX_HEAT_CURRENT_MILLIAMPS",
    3: "MAX_COOL_CURRENT_MILLIAMPS",
    7: "HEAT_RAMP_MILLIC_PER_S",
    8: "COOL_RAMP_MILLIC_PER_S",
    9: "PROPORTIONAL_GAIN_X1000",
    10: "INTEGRAL_GAIN_X1000",
    11: "DERIVATIVE_GAIN_X1000",
    12: "FEED_FORWARD_GAIN_X1000",
    13: "TEC_CURRENT_MILLIAMPS",
    14: "CURRENT_REFERENCE_MILLIAMPS",
    19: "TEMPERATURE_GAIN_RAW",
    20: "TEMPERATURE_OFFSET_RAW",
    21: "FAN_SPEED_RAW",
    22: "TEMPERATURE_REFERENCE_MILLIC",
    23: "PWM_OVERRIDE_PERCENT",
}

THERMAL_TARGETS = {
    0: "THERMAL_NEST",
    1: "THERMAL_LID",
    2: "THERMAL_PEDESTAL",
}

THERMAL_SENSOR_AXES = {
    0: "THERMAL_SENSOR_0",
    1: "THERMAL_SENSOR_1",
    2: "THERMAL_PEDESTAL_SENSOR",
}

CHILLER_CONTROLLER_BANKS = {
    0: "REAGENT_CHILLER_CONTROLLER",
    1: "OUTPUT_CHILLER_CONTROLLER",
}

CHILLER_SENSOR_AXES = {
    0: "REAGENT_CHILLER_SENSOR",
    1: "REAGENT_PEDESTAL_SENSOR",
    3: "OUTPUT_CHILLER_SENSOR",
    4: "OUTPUT_PEDESTAL_SENSOR",
}

IO_QUERY_TYPES = {
    0: "24V_SENSOR",
    1: "DOOR_SENSOR",
    2: "SOLENOID_STATE",
    3: "LATCH_SENSOR",
}

KNOWN_HEARTBEATS = {
    (0x7E, 0x00, 0x00, 0x04, 0x82, 0x00, 0x86, 0x7E),
    (0x7E, 0x00, 0x00, 0x04, 0x8A, 0x00, 0x8E, 0x7E),
    (0x7E, 0x00, 0x00, 0x04, 0x92, 0x00, 0x96, 0x7E),
    (0x7E, 0x00, 0x00, 0x04, 0x9A, 0x00, 0x9E, 0x7E),
    (0x7E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x04, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8D, 0x7E),
    (0x7E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x05, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8E, 0x7E),
    (0x7E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x06, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8F, 0x7E),
    (0x7E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x07, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x7E),
}

BOOLEAN_MOTOR_PARAMS = {8, 9, 10, 12, 13}


def fmt_hex(data: list[int]) -> str:
    return " ".join(f"{byte:02X}" for byte in data)



def _validate_bytes(values: Iterable[int]) -> list[int]:
    out: list[int] = []
    for value in values:
        ivalue = int(value)
        if not 0 <= ivalue <= 0xFF:
            raise ValueError(f"byte out of range: {ivalue}")
        out.append(ivalue)
    return out



def _extract_frame_field(obj: dict[str, Any]) -> tuple[list[int] | None, dict[str, Any]]:
    metadata = dict(obj)
    for key in ("frame", "bytes", "hex"):
        if key not in obj:
            continue
        frame_value = obj[key]
        metadata.pop(key, None)
        if isinstance(frame_value, list):
            return _validate_bytes(frame_value), metadata
        if isinstance(frame_value, str):
            return parse_frame_text(frame_value), metadata
        raise ValueError(f"unsupported frame field type for {key}: {type(frame_value).__name__}")
    return None, metadata



def parse_frame_text(text: str) -> list[int]:
    raw = text.strip()
    if not raw:
        raise ValueError("empty input")

    if raw.startswith("{") and raw.endswith("}"):
        obj = json.loads(raw)
        if not isinstance(obj, dict):
            raise ValueError("JSON frame record must be an object")
        frame, _ = _extract_frame_field(obj)
        if frame is None:
            raise ValueError("JSON object does not contain a frame/bytes/hex field")
        return frame

    if raw.startswith("[") and raw.endswith("]"):
        inner = raw[1:-1].strip()
        if not inner:
            return []
        tokens = [token for token in re.split(r"[\s,]+", inner) if token]
        return _validate_bytes(int(token, 0) for token in tokens)

    prefixed = re.match(r"^(?P<direction>tx|rx|event|frame)\s*[:|-]?\s*(?P<body>.+)$", raw, flags=re.IGNORECASE)
    if prefixed:
        raw = prefixed.group("body").strip()

    looks_hex = bool(re.search(r"[A-Fa-fx]", raw)) or bool(re.fullmatch(r"[0-9A-Fa-f]+", raw))
    compact = re.sub(r"[^0-9A-Fa-f]", "", raw)
    if looks_hex and compact and len(compact) % 2 == 0 and re.fullmatch(r"[0-9A-Fa-f]+", compact):
        return list(bytes.fromhex(compact))

    tokens = [token for token in re.split(r"[\s,;:]+", raw.replace(",", " ")) if token]
    if not tokens:
        raise ValueError("no bytes found")
    return _validate_bytes(int(token, 0) for token in tokens)



def checksum_info(frame: list[int]) -> dict[str, Any] | None:
    if len(frame) < 3 or frame[0] != 0x7E or frame[-1] != 0x7E:
        return None
    computed = sum(frame[1:-2]) & 0xFF
    expected = frame[-2]
    return {
        "expected": expected,
        "computed": computed,
        "valid": computed == expected,
    }



def axis_name(board_id: int, axis: int) -> str | None:
    return AXIS_NAMES.get(board_id, {}).get(axis)



def command_name(cmd: int) -> str:
    return CMD_NAMES.get(cmd, f"UNKNOWN_{cmd}")



def board_name(board_id: int) -> str:
    return BOARD_NAMES.get(board_id, f"UNKNOWN_BOARD_0x{board_id:02X}")



def scale_interpretation(board_id: int, param: int, value: int) -> dict[str, Any] | None:
    if board_id == 0x06 and param in THERMAL_GP_PARAMS:
        scaled = value / 1000.0 if param in {2, 3, 4, 7, 8, 9, 10, 11, 12, 13, 14, 22} else value
        return {
            "parameter_name": THERMAL_GP_PARAMS[param],
            "scaled_value": scaled,
        }
    if board_id == 0x07 and param in CHILLER_GP_PARAMS:
        scaled = value / 1000.0 if param in {2, 3, 4, 7, 8, 9, 10, 11, 12, 13, 14, 22} else value
        return {
            "parameter_name": CHILLER_GP_PARAMS[param],
            "scaled_value": scaled,
        }
    return None



def interpret_request(board_id: int, cmd: int, cmd_type: int, motor: int, value: int) -> dict[str, Any]:
    out: dict[str, Any] = {}
    axis = axis_name(board_id, motor)

    if cmd in (1, 2, 3):
        if axis:
            out["motion_axis"] = axis
        if cmd in (1, 2):
            out["speed_or_velocity"] = value
    elif cmd == 4:
        out["move_mode"] = MVP_TYPES.get(cmd_type, f"TYPE_{cmd_type}")
        if axis:
            out["motion_axis"] = axis
        out["position_or_delta"] = value
    elif cmd in (5, 6):
        out["parameter_name"] = MOTOR_PARAMS.get(cmd_type, f"PARAM_{cmd_type}")
        if axis:
            out["motion_axis"] = axis
        scaled = scale_interpretation(board_id, cmd_type, value)
        if scaled:
            out.update({key: val for key, val in scaled.items() if key != "parameter_name"})
    elif cmd in (9, 10):
        scaled = scale_interpretation(board_id, cmd_type, value)
        out["parameter_name"] = (scaled or {}).get("parameter_name", f"GP_PARAM_{cmd_type}")
        if scaled:
            out.update({key: val for key, val in scaled.items() if key != "parameter_name"})
        if board_id == 0x06 and cmd_type == 4:
            out["thermal_target"] = THERMAL_TARGETS.get(motor, f"BANK_{motor}")
        elif board_id == 0x07 and cmd_type == 4:
            out["chiller_bank"] = CHILLER_CONTROLLER_BANKS.get(motor, f"BANK_{motor}")
    elif cmd == 13:
        out["reference_search_mode"] = RFS_TYPES.get(cmd_type, f"TYPE_{cmd_type}")
        if axis:
            out["motion_axis"] = axis
    elif cmd == 138:
        if axis:
            out["motion_axis"] = axis
        if value:
            bits = [bit for bit in range(8) if value & (1 << bit)]
            out["axis_mask_bits"] = bits
            out["axis_mask_names"] = [axis_name(board_id, bit) or f"AXIS_{bit}" for bit in bits]
        else:
            out["query_scope"] = "single_axis"
    elif cmd == 14:
        if board_id == 0x05 and motor == 2 and cmd_type in (0, 1):
            out["io_usage"] = "ELECTRONIC_SWITCH_MODULE_RELAY"
            out["relay_name"] = "DECK_MOTOR_24V" if cmd_type == 0 else "HEAD_MOTOR_24V"
            out["bank"] = motor
            out["set_value"] = value
        elif board_id == 0x05 and cmd_type == 2:
            out["io_usage"] = "LATCH_SOLENOID_CONTROL"
            out["set_value"] = value
        else:
            out["io_usage"] = "GENERIC_SIO"
            out["type_label"] = IO_QUERY_TYPES.get(cmd_type)
            out["bank_or_motor"] = motor
            out["set_value"] = value
    elif cmd == 15:
        out["io_query"] = IO_QUERY_TYPES.get(cmd_type, f"TYPE_{cmd_type}")
    elif cmd == 50:
        approx_u8 = max(0, min(255, int(round((value * 255.0) / 1024.0))))
        out.update(
            {
                "led_target": LED_MASKS.get(motor, f"MASK_{motor}"),
                "tmcl_scaled_intensity": value,
                "approx_u8_intensity": approx_u8,
            }
        )
    elif cmd == 64:
        out["action"] = "ACTIVATE" if value else "DEACTIVATE"
    elif cmd == 140:
        out["target_temperature_c"] = value / 1000.0
        if board_id == 0x06:
            out["thermal_target"] = THERMAL_TARGETS.get(motor, f"BANK_{motor}")
        elif board_id == 0x07:
            out["chiller_bank"] = CHILLER_CONTROLLER_BANKS.get(motor, f"BANK_{motor}")
    elif cmd == 141:
        out["fan_speed_raw"] = value
        if board_id == 0x06:
            out["thermal_target"] = THERMAL_TARGETS.get(motor, f"BANK_{motor}")
        elif board_id == 0x07:
            out["chiller_bank"] = CHILLER_CONTROLLER_BANKS.get(motor, f"BANK_{motor}")
    elif cmd == 143:
        if board_id == 0x06:
            out["thermal_sensor"] = THERMAL_SENSOR_AXES.get(motor, f"AXIS_{motor}")
        elif board_id == 0x07:
            out["chiller_sensor"] = CHILLER_SENSOR_AXES.get(motor, f"AXIS_{motor}")
        else:
            out["sensor_axis"] = motor
    elif cmd == 144:
        out["pwm_override_percent"] = value
        if board_id == 0x06:
            out["thermal_target"] = THERMAL_TARGETS.get(motor, f"BANK_{motor}")
        elif board_id == 0x07:
            out["chiller_bank"] = CHILLER_CONTROLLER_BANKS.get(motor, f"BANK_{motor}")
    elif cmd == 153:
        out["action"] = "DUMP_TC_HISTORY"
    elif cmd == 173:
        out["action"] = "QUERY_FIRMWARE"
    return out



def interpret_reply(board_id: int, status: int, cmd: int, value: int, raw_value_bytes: list[int]) -> dict[str, Any]:
    out: dict[str, Any] = {
        "status_name": TMCL_STATUS.get(status, f"UNKNOWN_STATUS_{status}"),
    }
    if cmd == 173:
        out["firmware_hex"] = "-".join(f"{byte:02X}" for byte in raw_value_bytes)
    elif cmd == 50:
        out["ack_for"] = "SET_LED"
    elif cmd == 64:
        out["ack_for"] = "ACTIVATE_BOARD"
    elif cmd == 138:
        out["ack_for"] = "QUERY_MOTOR_STOP"
    elif cmd == 140:
        out["ack_for"] = "SET_TARGET_TEMPERATURE"
    elif cmd == 141:
        out["ack_for"] = "SET_HEATSINK_FAN"
    elif cmd == 143:
        out["ack_for"] = "READ_SENSOR_TEMPERATURE"
        if status == 100:
            out["temperature_c"] = value / 1000.0
    elif cmd == 144:
        out["ack_for"] = "SET_PWM_OVERRIDE"
    elif cmd == 153:
        out["ack_for"] = "DUMP_TC_HISTORY"
    elif cmd == 15:
        out["io_value"] = value
    return out



def _value_dict(raw_value: list[int]) -> dict[str, Any]:
    signed = struct.unpack(">i", bytes(raw_value))[0]
    unsigned = int.from_bytes(bytes(raw_value), "big", signed=False)
    return {
        "signed": signed,
        "unsigned": unsigned,
        "hex": f"0x{unsigned:08X}",
        "raw_bytes": raw_value,
    }



def decode_tmcl_frame(frame: list[int]) -> dict[str, Any]:
    result: dict[str, Any] = {
        "length": len(frame),
        "bytes": frame,
        "hex": fmt_hex(frame),
        "is_known_heartbeat": tuple(frame) in KNOWN_HEARTBEATS,
        "checksum": checksum_info(frame),
    }

    if tuple(frame) in KNOWN_HEARTBEATS and len(frame) == 8:
        result["kind"] = "heartbeat_short"
        result["heartbeat_code"] = frame[4]
        return result

    if tuple(frame) in KNOWN_HEARTBEATS and len(frame) == 16:
        board_id = frame[6]
        status = frame[7]
        result.update(
            {
                "kind": "heartbeat_long",
                "board": {"id": board_id, "name": board_name(board_id)},
                "status": {"id": status, "name": TMCL_STATUS.get(status, f"UNKNOWN_STATUS_{status}")},
                "command": {"id": frame[8], "name": command_name(frame[8])},
            }
        )
        return result

    if len(frame) == 16 and frame[0] == 0x7E and frame[-1] == 0x7E:
        if frame[1] == 0 and frame[2] == 0 and frame[3] == 0 and frame[4] == 0 and frame[5] == 0x08:
            board_id = frame[6]
            status = frame[7]
            cmd = frame[8]
            raw_value = frame[9:13]
            value = _value_dict(raw_value)
            result.update(
                {
                    "kind": "reply",
                    "board": {"id": board_id, "name": board_name(board_id)},
                    "status": {"id": status, "name": TMCL_STATUS.get(status, f"UNKNOWN_STATUS_{status}")},
                    "command": {"id": cmd, "name": command_name(cmd)},
                    "value": value,
                    "reserved": frame[13],
                    "interpretation": interpret_reply(board_id, status, cmd, value["signed"], raw_value),
                }
            )
            return result

        if frame[1] == 0 and frame[2] == 0 and frame[3] == 0 and frame[5] == 0x08 and frame[4] != 0:
            board_id = frame[4]
            cmd = frame[6]
            cmd_type = frame[7]
            motor = frame[8]
            raw_value = frame[9:13]
            value = _value_dict(raw_value)
            result.update(
                {
                    "kind": "request",
                    "board": {"id": board_id, "name": board_name(board_id)},
                    "command": {"id": cmd, "name": command_name(cmd)},
                    "type": cmd_type,
                    "motor": motor,
                    "value": value,
                    "reserved": frame[13],
                    "interpretation": interpret_request(board_id, cmd, cmd_type, motor, value["signed"]),
                }
            )
            return result

    result["kind"] = "unknown_or_non_tmcl"
    return result



def _enrich_reply_from_request(request: dict[str, Any], reply: dict[str, Any]) -> dict[str, Any]:
    board_id = request["board"]["id"]
    cmd = request["command"]["id"]
    cmd_type = request["type"]
    motor = request["motor"]
    value = reply["value"]["signed"]

    enriched = dict(request.get("interpretation", {}))
    enriched.update(reply.get("interpretation", {}))
    enriched["request_type"] = cmd_type
    enriched["request_motor"] = motor

    if cmd == 6:
        enriched.setdefault("parameter_name", MOTOR_PARAMS.get(cmd_type, f"PARAM_{cmd_type}"))
        if cmd_type in BOOLEAN_MOTOR_PARAMS:
            enriched["state"] = bool(value)
        scaled = scale_interpretation(board_id, cmd_type, value)
        if scaled:
            enriched.update(scaled)
    elif cmd == 10:
        scaled = scale_interpretation(board_id, cmd_type, value)
        if scaled:
            enriched.update(scaled)
        enriched.setdefault("parameter_name", (scaled or {}).get("parameter_name", f"GP_PARAM_{cmd_type}"))
    elif cmd == 15:
        enriched["io_active"] = bool(value)
    elif cmd == 138:
        if request["value"]["signed"]:
            bits = [bit for bit in range(8) if value & (1 << bit)]
            enriched["stopped_axis_mask_bits"] = bits
            enriched["stopped_axis_mask_names"] = [axis_name(board_id, bit) or f"AXIS_{bit}" for bit in bits]
        else:
            enriched["is_stopped"] = bool(value)
    elif cmd == 143 and reply["status"]["id"] == 100:
        enriched["temperature_c"] = value / 1000.0
    elif cmd in {5, 9, 13, 14, 50, 64, 140, 141, 144, 153}:
        enriched["acknowledged"] = reply["status"]["id"] == 100

    enriched["reply_value"] = value
    return enriched



def _timestamp_ns(metadata: dict[str, Any]) -> int | None:
    for key in ("timestamp_ns", "ts_ns", "time_ns"):
        if key in metadata:
            try:
                return int(metadata[key])
            except (TypeError, ValueError):
                return None
    return None



def parse_session_line(text: str) -> dict[str, Any]:
    raw = text.strip()
    if not raw:
        return {"skip": True}
    if raw.startswith("#"):
        return {"skip": True}

    if raw.startswith("{") and raw.endswith("}"):
        obj = json.loads(raw)
        if not isinstance(obj, dict):
            raise ValueError("session JSON line must be an object")
        frame, metadata = _extract_frame_field(obj)
        if frame is None:
            return {"skip": True, "metadata_only": metadata}
        return {"frame": frame, "metadata": metadata}

    metadata: dict[str, Any] = {}
    match = re.match(r"^(?P<direction>tx|rx|event|frame)\s*[:|-]?\s*(?P<body>.+)$", raw, flags=re.IGNORECASE)
    if match:
        metadata["direction"] = match.group("direction").lower()
        raw = match.group("body").strip()

    return {"frame": parse_frame_text(raw), "metadata": metadata}



def decode_many(inputs: list[str]) -> list[dict[str, Any]]:
    out = []
    for item in inputs:
        frame = parse_frame_text(item)
        out.append({"input": item, "decoded": decode_tmcl_frame(frame)})
    return out



def decode_session_lines(lines: Iterable[str]) -> dict[str, Any]:
    frames: list[dict[str, Any]] = []
    metadata_records: list[dict[str, Any]] = []

    for line_number, item in enumerate(lines, start=1):
        parsed = parse_session_line(item)
        if parsed.get("skip"):
            if parsed.get("metadata_only"):
                metadata_records.append(parsed["metadata_only"])
            continue

        metadata = dict(parsed.get("metadata", {}))
        decoded = decode_tmcl_frame(parsed["frame"])
        record = {
            "index": len(frames),
            "line_number": line_number,
            **metadata,
            "decoded": decoded,
        }
        frames.append(record)

    pending: dict[tuple[int, int], deque[dict[str, Any]]] = defaultdict(deque)
    transactions: list[dict[str, Any]] = []
    unmatched_replies: list[dict[str, Any]] = []

    for record in frames:
        decoded = record["decoded"]
        kind = decoded.get("kind")
        if kind == "request":
            key = (decoded["board"]["id"], decoded["command"]["id"])
            pending[key].append(record)
            continue
        if kind != "reply":
            continue

        key = (decoded["board"]["id"], decoded["command"]["id"])
        if not pending[key]:
            unmatched_replies.append(
                {
                    "frame_index": record["index"],
                    "line_number": record["line_number"],
                    "board": decoded["board"],
                    "command": decoded["command"],
                    "status": decoded["status"],
                }
            )
            continue

        request_record = pending[key].popleft()
        request = copy.deepcopy(request_record["decoded"])
        reply = copy.deepcopy(decoded)
        reply["request_context"] = {
            "type": request["type"],
            "motor": request["motor"],
            "value": request["value"],
            "interpretation": request.get("interpretation", {}),
        }
        reply["interpretation"] = _enrich_reply_from_request(request, reply)

        transaction = {
            "index": len(transactions),
            "board": request["board"],
            "command": request["command"],
            "request_frame_index": request_record["index"],
            "reply_frame_index": record["index"],
            "request_line_number": request_record["line_number"],
            "reply_line_number": record["line_number"],
            "request": request,
            "reply": reply,
            "status": reply["status"],
        }

        req_ts = _timestamp_ns(request_record)
        reply_ts = _timestamp_ns(record)
        if req_ts is not None and reply_ts is not None and reply_ts >= req_ts:
            transaction["latency_ms"] = round((reply_ts - req_ts) / 1_000_000.0, 3)

        transactions.append(transaction)

    pending_requests: list[dict[str, Any]] = []
    for queue in pending.values():
        for record in queue:
            decoded = record["decoded"]
            pending_requests.append(
                {
                    "frame_index": record["index"],
                    "line_number": record["line_number"],
                    "board": decoded["board"],
                    "command": decoded["command"],
                    "type": decoded["type"],
                    "motor": decoded["motor"],
                }
            )

    return {
        "mode": "transactions",
        "frames": frames,
        "transactions": transactions,
        "unmatched_replies": unmatched_replies,
        "pending_requests": pending_requests,
        "summary": {
            "frame_records": len(frames),
            "metadata_records": len(metadata_records),
            "transaction_count": len(transactions),
            "unmatched_reply_count": len(unmatched_replies),
            "pending_request_count": len(pending_requests),
        },
    }



def _read_input_lines(paths: list[str], argv_frames: list[str]) -> list[str]:
    lines = list(argv_frames)
    for raw_path in paths:
        path = Path(raw_path)
        lines.extend(path.read_text(encoding="utf-8").splitlines())
    if not lines and not sys.stdin.isatty():
        lines = [line.rstrip("\n") for line in sys.stdin]
    return [line for line in lines if line.strip()]



def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Decode BioXP TMCL USB frames or correlated request/reply sessions.")
    parser.add_argument(
        "frames",
        nargs="*",
        help="Frame text. Supports raw TMCL hex, decimal byte lists, or capture JSON lines.",
    )
    parser.add_argument(
        "--input",
        action="append",
        default=[],
        help="Read additional frame/session lines from a file. May be specified multiple times.",
    )
    parser.add_argument(
        "--session",
        "--transactions",
        dest="session",
        action="store_true",
        help="Decode the input as a multi-line session and correlate request/reply transactions.",
    )
    parser.add_argument("--compact", action="store_true", help="Emit compact one-line JSON.")
    return parser



def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    lines = _read_input_lines(args.input, list(args.frames))
    if not lines:
        parser.error("provide at least one frame string, input file, or pipe lines on stdin")

    decoded = decode_session_lines(lines) if args.session else decode_many(lines)
    if args.compact:
        print(json.dumps(decoded, separators=(",", ":"), sort_keys=True))
    else:
        print(json.dumps(decoded, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
