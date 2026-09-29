"""Retained ClassBioXPScriptHandler expansion into the existing native pipeline.

Mechanical/thermal translation is source code, not execution or physical proof.
Unported liquid generators remain explicit errors; no runnable prefix is returned.
Anchors refer to the retained BioXPCommonLib/ClassBioXPScriptHandler.cs.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

from .compiler import compile_prepared_oem_protocol
from .models import ProtocolDocument, OEM_SOURCE_DEFAULT_NOOPS, _payload
from .oem_xml_import import import_oem_xml_protocol

_PLATES = {
    "PL_POOL": "POOL_PLATE", "PL_OUTPUT": "OUTPUT_PLATE", "PL_REAGENT": "REAGENT_PLATE",
    "CV_BIOSECURITY": "BIO_SECURITY_COVER", "CV_OUTPUT": "OUTPUT_COVER", "CV_REAGENT": "REAGENT_COVER",
}
# ClassGlobals.cs:134-191: covers select the first cover/BSC match; plates
# fall through to the final entry. In particular LOC_RC's plate entry is UNKNOWN.
_LOCATIONS = {
    "LOC_BSCS": ("LOC_BSCS",), "LOC_TC": ("LOC_BSC", "LOC_P_TC"),
    "LOC_RC": ("LOC_RC_COVER", "UNKNOWN"), "LOC_RCS": ("LOC_RC_COVER_STORAGE",),
    "LOC_OC": ("LOC_OC_COVER", "LOC_P_OC"), "LOC_OCS": ("LOC_OC_COVER_STORAGE",),
    "LOC_MS": ("LOC_P_MS",), "LOC_HOLDER": ("LOC_HOLDER",),
}


def _location(plate: str, token: str) -> str:
    choices = _LOCATIONS.get(token, (token,))
    if "COVER" in plate:
        for choice in choices:
            if "COVER" in choice or "BSC" in choice:
                return choice
    return choices[-1]


class UnexpandedOemGenerator(NotImplementedError):
    """A source generator exists but has not been ported, not a physical gate."""

    def __init__(self, source_file: str, commands: list[dict[str, Any]]):
        self.source_file, self.commands = source_file, commands
        super().__init__(f"Unported OEM liquid generators in {source_file}: " +
                         ", ".join(f"line{row['source_key']}:{row['verb']}" for row in commands))


def expand_oem_xml_protocol(path: str | Path, *, source_settings: Mapping[str, Any],
                            metadata: Mapping[str, Any] | None = None) -> ProtocolDocument:
    """Import once, expand once, then use compile_prepared_oem_protocol unchanged.

    The caller supplies the captured source settings already used by the native
    binding factory. MotionOnly is a source selection, not execution_mode. Other
    preparation/model fields pass through to the existing preparation owner.
    """
    imported = import_oem_xml_protocol(path)
    return expand_imported_oem_protocol(imported.document, source_settings=source_settings, metadata=metadata)


def expand_imported_oem_protocol(document: ProtocolDocument, *, source_settings: Mapping[str, Any],
                                 metadata: Mapping[str, Any] | None = None) -> ProtocolDocument:
    source = _payload(document.metadata)
    rows = sorted((row for row in source["source_map"] if "source_key" in row), key=lambda row: row["source_key"])
    # Retain the existing explicit generator gap, with all occurrences rather
    # than an unsafe successful prefix. Do not infer liquid algorithms by name.
    missing = [{"source_key": row["source_key"], "verb": row["attributes"]["cmd"].split(" ")[0],
                "raw_cmd": row["attributes"]["cmd"]} for row in rows
               if row["attributes"].get("cmd", "").split(" ")[0] in {"MT", "FP", "LA", "SA"}]
    if missing:
        raise UnexpandedOemGenerator(source["source_file"], missing)
    settings = _payload(source_settings)
    motion_only = settings["MotionOnly"]
    operations: list[dict[str, Any]] = []
    current_temp, current_lid, high_temp = 25.0, 25.0, 0.0
    fan_emitted = False
    door_open = False  # ClassVirtualBioXP constructor logical state, not a sensor.
    park = False
    current_plate = None
    loop = None
    loop_count = 0

    def emit(op: str, args: Any, row: Mapping[str, Any] | None, *, iteration: int | None = None,
             argument_type: str | None = None) -> None:
        ordinal = len(operations) + 1
        item = {"oem_opcode": op, "arguments": args, "source_key": ordinal * 10,
                "source_occurrence_id": f"xml:{ordinal}",
                "metadata": {"xml_source_key": row["source_key"] if row else None,
                             "xml_attributes": dict(row["attributes"]) if row else {},
                             "loop_iteration": iteration}}
        if argument_type:
            item["argument_type"] = argument_type
        operations.append(item)

    emit("iniPipette", [], None)  # translateScript:362, in addition to lifecycle.
    for row in rows:
        attributes = row["attributes"]
        text = "step " + attributes["step"] if "step" in attributes else attributes.get("cmd", "")
        if not text:
            continue
        tokens = text.split(" ")  # source literal-space semantics, not split().
        verb, args = tokens[0], tokens[1:]
        if not motion_only:
            if loop is not None:
                if text.startswith("LOOP"):
                    # translateLoopScript:548-586: accumulator persists across
                    # iterations; only the flag is cleared after a consuming SP.
                    translated = []
                    dwell, pending = 0, False
                    for iteration in range(loop_count):
                        for inner in loop:
                            parts = inner["attributes"].get("cmd", "").split(" ")
                            if parts[0] == "DWELL":
                                pending = True
                                dwell += int(parts[1])
                            elif parts[0] == "SP":
                                current_temp = float(parts[1][1:])
                                duration = str(int(parts[2][3:]) + dwell) if pending else parts[2][3:]
                                ramp = ("-" if pending else "") + parts[3][1:]
                                translated.append(("sp", [f"{current_temp:.2f}", duration, ramp], inner, iteration))
                                pending = False
                            elif parts[0] == "LED":
                                translated.append(("led", parts[1:4], inner, iteration))
                            # The source loop ignores every other verb.
                    if not fan_emitted and high_temp > 50:
                        emit("fon", [], row)
                        fan_emitted = True
                    if high_temp < current_temp:
                        high_temp += 6
                        high_temp = current_lid = high_temp + 6
                        emit("splid", [f"{high_temp:.2f}", "10", "1.0", "T"], row)
                    for op, values, inner, iteration in translated:
                        emit(op, values, inner, iteration=iteration)
                    loop, high_temp = None, 0.0
                else:
                    loop.append(row)
                    if verb == "SP":
                        high_temp = max(float(args[0][1:]), high_temp)
                continue
            if text.startswith("LOOP"):
                loop, loop_count, high_temp = [], int(args[0]), current_temp
                continue
        children = []
        if verb in {"SP", "TCD"} and not (verb == "SP" and motion_only):
            children.append(("//", [str(row["source_key"]), "-", verb]))
        if verb == "step":
            children.append(("step", args))
        elif verb in {"CC", "WAIT"}:
            if not motion_only:
                children.append((verb.lower(), args[:2] if verb == "CC" else args[:1]))
        elif verb == "LED":
            children.append(("led", args[:3]))
        elif verb == "DELAYPOINT":
            children.append(("delaypoint", []))
        elif verb == "TCD":
            if args[0] == "DO":
                children.extend((("dopen", []), ("iniPipette", [])))
                door_open = True
            elif args[0] == "DC":
                children.append(("dclose", []))
                door_open, park = False, True
        elif verb == "PP":
            plates = [_PLATES[token] for token in args if token in {"PL_POOL", "PL_OUTPUT", "PL_REAGENT"}]
            children.append(("pressp", plates))
            if plates:
                current_plate = plates[-1]
        elif verb in {"MP", "MC"}:
            plate = _PLATES[args[0]]
            if plate in {"POOL_PLATE", "BIO_SECURITY_COVER"} and not door_open:
                raise ValueError(f"Cannot move bio-security cover when door is closed, line {row['source_key']}")
            target = _location(plate, args[1])
            if target in {"LOC_BSC", "LOC_P_TC"} and not door_open:
                raise ValueError(f"Cannot access TC cover when door is closed, line {row['source_key']}")
            if target in {"LOC_RC_COVER_STORAGE", "LOC_OC_COVER_STORAGE"} and door_open:
                raise ValueError(f"Cannot access Reagent Cover Storage when door is open, line {row['source_key']}")
            children.append(("//", [str(row["source_key"]), "-", "MC" if "COVER" in plate else "MP", plate]))
            children.extend((("catchPlate", [plate]), ("releasePlate", [target] +
                             (["pressplate"] if len(args) > 2 and args[2] == "PRESS" else []))))
            current_plate = plate
        elif verb == "ET":
            # With no liquid generator in this subset m_tipon remains false.
            # The source still moves to WASTE before deciding whether to eject.
            if current_plate != "WASTE_BIN":
                destination = None if settings["StartMode"] == 3 else 6
                emit("mov", {"m_destination": destination, "m_well": None}, row, argument_type="ClassMoveTo")
            current_plate = "WASTE_BIN"
        elif verb == "SP" and not motion_only:
            if park:
                children.append(("park", []))
                park = False
            temperature = float(args[0][1:])
            duration, ramp = args[1][3:], args[2][1:]
            if temperature + 5 < high_temp:
                high_temp = current_lid = temperature + 6
                children.extend((("splid", [f"{high_temp:.2f}", "10", "-20.0", "F"]),
                                 ("sp", [f"{temperature:.2f}", duration, "-" + ramp])))
            else:
                high_temp = temperature + 6
                if current_temp > temperature:
                    children.append(("sp", [f"{temperature:.2f}", duration, "-" + ramp]))
                else:
                    children.append(("splid", [f"{high_temp:.2f}", "10", "1.0", "T" if current_lid < high_temp else "F"]))
                    current_lid = high_temp
                    children.append(("sp", [f"{temperature:.2f}", duration, ramp]))
            current_temp = temperature
        elif verb == "SP":
            pass
        elif verb in OEM_SOURCE_DEFAULT_NOOPS:
            children.append((verb, args))
        else:
            raise NotImplementedError(f"Unported OEM source command line{row['source_key']}: {text}")
        if not fan_emitted and high_temp > 50:
            emit("fon", [], row)
            fan_emitted = True
        for op, values in children:
            emit(op, values, row)
    # As in translateScript an unterminated thermal loop is not flushed.
    captured = dict(metadata or {})
    captured.update(source_settings=settings, xml_source_sha256=source["source_sha256"],
                    xml_source_file=source["source_file"], xml_source_map=source["source_map"],
                    generator="retained-ClassBioXPScriptHandler-mechanical-thermal",
                    imported_command_count=source["coverage"]["command_nodes_total"])
    return compile_prepared_oem_protocol({"protocol_id": document.protocol_id, "metadata": captured, "operations": operations})


def system_check_protocol(*, source_settings: Mapping[str, Any],
                          metadata: Mapping[str, Any] | None = None) -> ProtocolDocument:
    """One definition: the retained XML, passed through the same generator."""
    return expand_oem_xml_protocol(Path(__file__).resolve().parents[3] / "scripts" / "Inital Test Script.xml",
                                   source_settings=source_settings, metadata=metadata)
