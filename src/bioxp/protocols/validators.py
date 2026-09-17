from __future__ import annotations

from typing import Any, Mapping, Iterable

from ..domain.capabilities import CapabilityName
from .models import ProtocolActionKind, ProtocolDocument, OEM_OPERATION_FORMS, OEM_TYPED_FIELDS, OEM_RAW_MIN_ARGUMENTS


_ACTION_CAPABILITY_MAP: dict[ProtocolActionKind, CapabilityName | None] = {
    ProtocolActionKind.MOVE: CapabilityName.MOTION,
    ProtocolActionKind.HOME: CapabilityName.MOTION,
    ProtocolActionKind.PIPETTE_INIT: CapabilityName.PIPETTE,
    ProtocolActionKind.PIPETTE_TIP: CapabilityName.PIPETTE,
    ProtocolActionKind.PIPETTE_ASPIRATE: CapabilityName.PIPETTE,
    ProtocolActionKind.PIPETTE_DISPENSE: CapabilityName.PIPETTE,
    ProtocolActionKind.PIPETTE_MIX: CapabilityName.PIPETTE,
    ProtocolActionKind.INSPECT: CapabilityName.INSPECTION,
    ProtocolActionKind.BARCODE_READ: CapabilityName.BARCODE,
    ProtocolActionKind.PAUSE_REVIEW: None,
    ProtocolActionKind.NOTE: None,
    ProtocolActionKind.LED: None,
    ProtocolActionKind.WAIT: None,
    ProtocolActionKind.PLATE_PREPARE: CapabilityName.MOTION,
    ProtocolActionKind.PLATE_MOVE: CapabilityName.MOTION,
    ProtocolActionKind.THERMAL_DOOR: CapabilityName.THERMAL,
    ProtocolActionKind.MOVE_COVER: CapabilityName.MOTION,
    ProtocolActionKind.CHILLER_SETPOINT: CapabilityName.CHILLER,
    ProtocolActionKind.THERMAL_SETPOINT: CapabilityName.THERMAL,
    ProtocolActionKind.LOOP_MARKER: None,
    ProtocolActionKind.SEAL_SEPARATE: CapabilityName.MOTION,
    ProtocolActionKind.LIQUID_ADJUST: CapabilityName.PIPETTE,
    ProtocolActionKind.TIP_EJECT: CapabilityName.PIPETTE,
}


def infer_required_capability(kind: ProtocolActionKind) -> CapabilityName | None:
    return _ACTION_CAPABILITY_MAP.get(kind)


def validate_protocol_document(document: ProtocolDocument) -> ProtocolDocument:
    if document.version != 1:
        raise ValueError("Unsupported protocol document version")
    if document.metadata.get("execution_mode", "normal") != "normal":
        raise ValueError("Only normal production execution mode is supported")
    if not document.stages:
        raise ValueError("Protocol document must include at least one stage.")

    stage_ids: set[str] = set()
    action_ids: set[str] = set()
    occurrences: set[str] = set()
    for stage in document.stages:
        if stage.stage_id in stage_ids:
            raise ValueError(f"Duplicate stage_id '{stage.stage_id}' in protocol document")
        stage_ids.add(stage.stage_id)

        if not stage.actions and not document.metadata.get("requires_generator_expansion"):
            raise ValueError(f"Stage '{stage.stage_id}' must include at least one action")

        for action in stage.actions:
            if action.stage_id != stage.stage_id:
                raise ValueError(
                    f"Action '{action.action_id}' is attached to stage '{action.stage_id}', expected '{stage.stage_id}'"
                )
            if action.action_id in action_ids:
                raise ValueError(f"Duplicate action_id '{action.action_id}' in protocol document")
            action_ids.add(action.action_id)
            if action.kind == ProtocolActionKind.OEM_OPERATION:
                opcode = action.oem_opcode
                if not isinstance(opcode, str) or opcode not in OEM_OPERATION_FORMS:
                    raise ValueError(f"Unsupported OEM opcode at '{action.action_id}'")
                occurrence = action.source_occurrence_id
                if not isinstance(occurrence, str) or not occurrence.strip() or occurrence in occurrences:
                    raise ValueError("OEM source occurrence must be nonempty and unique")
                occurrences.add(occurrence)
                if action.source_key is not None and type(action.source_key) not in (str, int):
                    raise ValueError("OEM source key must be a string or integer")
                if set(action.params) - {"arguments", "argument_type"}:
                    raise ValueError(f"Unknown OEM parameter fields at '{occurrence}'")
                arguments = action.params.get("arguments")
                form = OEM_OPERATION_FORMS[opcode]
                if form == "raw":
                    if action.params.get("argument_type", "raw") != "raw" or not isinstance(arguments, tuple) or any(type(arg) is not str for arg in arguments):
                        raise ValueError(f"OEM {opcode} requires raw string arguments")
                    if len(arguments) < OEM_RAW_MIN_ARGUMENTS.get(opcode, 0):
                        raise ValueError(f"Missing OEM {opcode} arguments at '{occurrence}'")
                elif action.params.get("argument_type") != form or not isinstance(arguments, Mapping):
                    raise ValueError(f"OEM {opcode} requires typed {form} arguments")
                elif set(arguments) - OEM_TYPED_FIELDS[form]:
                    raise ValueError(f"Unknown fields in OEM {form} arguments")
            elif action.oem_opcode is not None:
                raise ValueError("OEM opcode requires oem_operation kind")

    return document


def _validate_oem_options(form: str, arguments: Mapping[str, Any]) -> None:
    """Source Class* field types only; nullable/default science stays native."""
    from math import isfinite
    fields = OEM_TYPED_FIELDS.get(form, frozenset({"m_volume", "m_speed", "m_delay"}))
    if set(arguments) - fields:
        raise ValueError(f"Unknown fields in OEM {form} arguments")
    if form == "ClassMoveTo":
        from ..oem_deck_movement import prepared_class_move_to_intent, well_id_from_label
        try:
            intent = prepared_class_move_to_intent(arguments, script_line=0)
            if intent.well is not None:
                if type(intent.well) not in (str, int):
                    raise ValueError("ClassMoveTo well requires source enum")
                well_id_from_label(intent.well)
        except (KeyError, TypeError) as exc:
            raise ValueError("Invalid ClassMoveTo source enum") from exc
        return
    nested = {"m_aspirateOptions": "ClassAspirate", "m_dispenseAllOptions": "ClassDispenseAll", "m_dispenseOptions": "DispenseOptions"}
    integers = {"m_repeat", "m_delay", "m_aspiratecushion", "m_dispensecushion", "m_shakeoffcount"}
    booleans = {"m_ntd", "m_purge", "m_dispensehigh", "m_orbit"}
    for key, value in arguments.items():
        if value is None:
            continue
        if key in nested:
            if not isinstance(value, Mapping):
                raise ValueError(f"OEM {key} requires source option fields")
            if key == "m_aspirateOptions" and "m_overaspirate" in value:
                raise ValueError("AspirateOptions does not contain m_overaspirate")
            _validate_oem_options(nested[key], value)
        elif key in integers:
            if type(value) is not int or not -(2**31) <= value < 2**31:
                raise ValueError(f"OEM {key} requires Int32 or null")
        elif key in booleans:
            if type(value) is not bool:
                raise ValueError(f"OEM {key} requires boolean or null")
        elif key in {"m_tipDip", "m_mixType"}:
            if type(value) is not str:
                raise ValueError(f"OEM {key} requires string or null")
        elif type(value) not in (int, float) or not isfinite(value):
            raise ValueError(f"OEM {key} requires finite number or null")


def validate_oem_selected_dependencies(document: ProtocolDocument, *, capabilities: Iterable[str] = ()) -> None:
    """Pure selected settings/native/CV check before canonical admission."""
    capabilities = frozenset(capabilities)
    settings = document.metadata.get("source_settings", {})
    if not isinstance(settings, Mapping):
        raise ValueError("OEM source_settings must be captured fields")
    def setting(name: str, kind: type):
        if name not in settings or type(settings[name]) is not kind:
            raise ValueError(f"Missing or invalid captured OEM setting: {name}")
        return settings[name]
    def require(name: str):
        if name not in capabilities:
            raise ValueError(f"Selected OEM dependency unavailable: {name}")
    for stage in document.stages:
        for action in stage.actions:
            if action.kind != ProtocolActionKind.OEM_OPERATION:
                continue
            op, args = action.oem_opcode, action.params["arguments"]
            assert op is not None
            if OEM_OPERATION_FORMS[op] != "raw":
                _validate_oem_options(OEM_OPERATION_FORMS[op], args)
            else:
                from ..oem_serial206_initialization import Serial206OemInitializationProvider as Provider
                from ..oem_deck_movement import OEM_PLATE_NAME_ORDINALS, canonical_plate_name
                from ..oem_compat.pathing import LOCATION_ID_TO_NAME
                if op in {"led", "so", "cutseal"}:
                    for token in (args[:3] if op == "led" else args[:1]):
                        Provider._oem_int32(token)
                if op in {"catch", "catchPlate", "release", "releasePlate"}:
                    plate = Provider._oem_enum(args[0], OEM_PLATE_NAME_ORDINALS)
                    if plate is not None:
                        canonical_plate_name(plate)
                    elif op in {"release", "releasePlate"}:
                        location = Provider._oem_enum(args[0], {v: k for k, v in LOCATION_ID_TO_NAME.items()}, ignore_case=False)
                        if location not in LOCATION_ID_TO_NAME:
                            raise ValueError("Invalid source release location")
                    else:
                        raise ValueError("Invalid source catch plate")
                if op == "ms":
                    from math import isfinite
                    if not isfinite(float(args[1])):
                        raise ValueError("Invalid source strip volume")
            if op in {"aa", "da", "masp", "dsa", "mmix"}:
                stream = setting("LogPressure", bool)
                if op == "da":
                    require("dispense_air_for_oem_script")
                elif stream:
                    if op in {"masp", "mmix"}:
                        require("aspirate_pressure_stream")
                    if op == "dsa":
                        require("dispense_pressure_stream")
            if op == "snapshot":
                setting("CheckSnapTips", bool)
            if op in {"dopen", "dclose"}:
                inspect = setting("DeckInspection", bool)
                mode = setting("StartMode", int)
                if op == "dopen" and inspect and mode in (1, 2):
                    require("ReadBarcode")
            if op == "cutseal":
                setting("CutZ_Offset", int)
            if op == "ldtip":
                setting("StartMode", int)
                camera = setting("CameraInstalled", bool)
                calibrated = setting("CameraCalibrated", bool)
                if camera and calibrated:
                    require("checkTips")
            if op == "ejt":
                setting("CheckSnapTips", bool)
    if settings.get("JobName") is not None:
        setting("JobName", str)
        for name in ("CameraXOffset", "CameraYOffset", "CameraZOffset"):
            setting(name, int)
    if document.metadata.get("oem_prepare"):
        require("prepare_inspections")


def validate_protocol_support(
    document: ProtocolDocument, *, oem_handlers: Mapping[str, Any],
    handlers: Mapping[ProtocolActionKind, Any],
    lifecycle_handlers: Mapping[str, Any] | None = None,
    required_lifecycle: Iterable[str] = (),
) -> ProtocolDocument:
    """Check the whole selected binding set; no native calls or runnable prefix."""
    validate_protocol_document(document)
    if document.metadata.get("source_type") == "oem_xml" or document.metadata.get("input_mode") == "oem_xml" or document.metadata.get("requires_generator_expansion"):
        raise ValueError("OEM XML requires separately qualified generator expansion")
    missing = []
    for stage in document.stages:
        for action in stage.actions:
            if action.params.get("requires_virtual_bioxp_state") or action.params.get("macro_verb") or action.kind == ProtocolActionKind.LOOP_MARKER:
                raise ValueError("Unexpanded OEM macro is not executable prepared input")
            if action.kind == ProtocolActionKind.OEM_OPERATION:
                # These are source markers/gates owned by the sole executor.
                if action.oem_opcode not in {"step", "delaypoint", "wait"} and not callable(oem_handlers.get(action.oem_opcode or "")):
                    missing.append(f"{action.source_occurrence_id}:{action.oem_opcode}")
                selected_preflight = getattr(oem_handlers.get(action.oem_opcode or ""), "preflight", None)
                if callable(selected_preflight):
                    selected_preflight(action)
            elif action.kind not in {ProtocolActionKind.NOTE, ProtocolActionKind.PAUSE_REVIEW} and not callable(handlers.get(action.kind)):
                missing.append(f"{action.action_id}:{action.kind.value}")
    for name in required_lifecycle:
        if not callable((lifecycle_handlers or {}).get(name)):
            missing.append(f"lifecycle:{name}")
    if missing:
        raise ValueError("Unbound protocol operations: " + ", ".join(missing))
    return document
