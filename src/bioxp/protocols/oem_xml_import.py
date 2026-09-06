from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping
import xml.etree.ElementTree as ET

from .models import ProtocolAction, ProtocolActionKind, ProtocolDocument, ProtocolStage
from .validators import infer_required_capability
from ..oem_compat.scripts import OemScriptCommand, OemScriptTranslator


@dataclass(frozen=True)
class UnsupportedOemCommand:
    verb: str
    raw_cmd: str
    stage_id: str
    source_node: Mapping[str, Any] = field(default_factory=dict)
    reason: str | None = None

    def to_payload(self) -> dict[str, Any]:
        return {
            "verb": self.verb,
            "raw_cmd": self.raw_cmd,
            "stage_id": self.stage_id,
            "source_node": dict(self.source_node),
            "reason": self.reason,
        }


@dataclass(frozen=True)
class OemXmlCoverage:
    source_file: str
    step_markers_total: int
    command_nodes_total: int
    supported_command_count: int
    unsupported_command_count: int
    supported_verbs: Mapping[str, int] = field(default_factory=dict)
    unsupported_verbs: Mapping[str, int] = field(default_factory=dict)
    unsupported_commands: tuple[UnsupportedOemCommand, ...] = ()

    @property
    def coverage_ratio(self) -> float:
        if self.command_nodes_total <= 0:
            return 1.0
        return self.supported_command_count / self.command_nodes_total

    def to_payload(self) -> dict[str, Any]:
        return {
            "source_file": self.source_file,
            "step_markers_total": int(self.step_markers_total),
            "command_nodes_total": int(self.command_nodes_total),
            "supported_command_count": int(self.supported_command_count),
            "unsupported_command_count": int(self.unsupported_command_count),
            "coverage_ratio": self.coverage_ratio,
            "supported_verbs": dict(self.supported_verbs),
            "unsupported_verbs": dict(self.unsupported_verbs),
            "unsupported_commands": [command.to_payload() for command in self.unsupported_commands],
        }


@dataclass(frozen=True)
class ImportedOemProtocol:
    document: ProtocolDocument
    coverage: OemXmlCoverage
    source_path: str
    experiment: Mapping[str, Any] = field(default_factory=dict)
    inventory: Mapping[str, Any] = field(default_factory=dict)

    def to_payload(self) -> dict[str, Any]:
        return {
            "source_path": self.source_path,
            "experiment": dict(self.experiment),
            "inventory": dict(self.inventory),
            "coverage": self.coverage.to_payload(),
            "document": self.document.to_payload(),
        }


SUPPORTED_OEM_VERBS = frozenset(
    {
        "LED",
        "WAIT",
        "DELAYPOINT",
        "PP",
        "TCD",
        "MC",
        "CC",
        "SP",
        "DWELL",
        "LOOP",
        "SS",
        "LA",
        "ET",
        "MP",
        "MT",
        "FP",
        "RT",
        "SA",
        "ST",
        "SW",
        "TT",
        "ZW",
    }
)


def _slugify(value: Any, *, fallback: str) -> str:
    text = "" if value is None else str(value).strip().lower()
    if not text:
        return fallback
    out: list[str] = []
    last_dash = False
    for char in text:
        if char.isalnum():
            out.append(char)
            last_dash = False
            continue
        if not last_dash:
            out.append("-")
            last_dash = True
    slug = "".join(out).strip("-")
    return slug or fallback


def _as_float(value: str | None) -> float | None:
    if value is None:
        return None
    return float(str(value).strip())


def _as_int(value: str | None) -> int | None:
    if value is None:
        return None
    return int(float(str(value).strip()))


def _prefixed_value(tokens: Iterable[str], prefix: str) -> str | None:
    for token in tokens:
        if token.startswith(prefix):
            return token[len(prefix) :]
    return None


_TIP_OPTIONS = {"NTY", "NTN", "KT", "ST", "FLY", "FLN", "TH"}


def _is_location_token(token: str) -> bool:
    text = str(token).upper()
    return text.startswith("PL_") or text == "TROUGH"


def _find_location(args: list[str], *, start: int = 0) -> tuple[int | None, str | None, str | None]:
    for idx in range(start, len(args)):
        if _is_location_token(args[idx]):
            zone = args[idx + 1] if idx + 1 < len(args) and args[idx + 1].upper().startswith("Z") else None
            return idx, args[idx], zone
    return None, None, None


def _prefixed_number(token: str, prefixes: tuple[str, ...]) -> tuple[str, float | int] | None:
    upper = str(token).upper()
    for prefix in prefixes:
        if upper.startswith(prefix) and len(upper) > len(prefix):
            raw_value = upper[len(prefix) :]
            try:
                value = float(raw_value)
            except ValueError:
                continue
            if value.is_integer():
                return prefix, int(value)
            return prefix, value
    return None


def _option_values(tokens: list[str]) -> dict[str, float | int]:
    values: dict[str, float | int] = {}
    for token in tokens:
        parsed = _prefixed_number(token, ("DUR", "AM", "MR", "DM", "RC", "A", "S", "D", "W", "V"))
        if parsed is not None:
            key, value = parsed
            values[key] = value
    return values


def _segment_after(
    args: list[str],
    marker: str,
    *,
    start: int = 0,
    end_markers: set[str] | None = None,
    last: bool = False,
) -> list[str]:
    marker = marker.upper()
    matches = [idx for idx in range(start, len(args)) if args[idx].upper() == marker]
    if not matches:
        return []
    idx = matches[-1] if last else matches[0]
    out: list[str] = []
    end_markers = {m.upper() for m in (end_markers or set())}
    for token in args[idx + 1 :]:
        upper = token.upper()
        if upper in end_markers:
            break
        if upper.startswith("V") or upper in _TIP_OPTIONS or upper in {"T50", "T200"}:
            break
        out.append(token)
    return out


def _tip_type(args: list[str]) -> str | None:
    for token in args:
        upper = token.upper()
        if upper in {"T50", "T200"}:
            return upper
    return None


def _tip_option(args: list[str]) -> str | None:
    for token in args:
        upper = token.upper()
        if upper in _TIP_OPTIONS:
            return upper
    return None


def _macro_base(verb: str, args: list[str], semantic_action: str) -> dict[str, Any]:
    return {
        "semantic_action": semantic_action,
        "macro_verb": verb,
        "raw_tokens": list(args),
        "requires_virtual_bioxp_state": True,
        "requires_ack_readback": True,
    }


def _compile_oem_macro_params(verb: str, args: list[str]) -> dict[str, Any] | None:
    verb = verb.upper()
    volume = _as_float(_prefixed_value(args, "V"))
    tip_type = _tip_type(args)
    tip_option = _tip_option(args)

    if verb == "SA" and len(args) >= 2:
        return {
            **_macro_base(verb, args, "material_agitate"),
            "material_id": args[0],
            "tip_type": tip_type or args[1],
            "repeat_count": _as_int(_prefixed_value(args, "RC")),
        }

    if verb == "ST" and args:
        loc_idx, dest_loc, dest_zone = _find_location(args)
        after_loc = 0 if loc_idx is None else loc_idx + 1
        return {
            **_macro_base(verb, args, "material_transfer"),
            "material_id": args[0],
            "dest_location_id": dest_loc,
            "dest_zone": dest_zone,
            "volume_ul": volume,
            "tip_type": tip_type,
            "tip_option": tip_option,
            "option_groups": {
                "AO": _option_values(_segment_after(args, "AO", end_markers={"PL_POOL", "PL_OUTPUT", "TROUGH"})),
                "DAO": _option_values(_segment_after(args, "DAO", start=after_loc, end_markers={"MO", "DBO"})),
                "MO": _option_values(_segment_after(args, "MO", start=after_loc, end_markers={"DBO"}, last=True)),
                "DBO": _option_values(_segment_after(args, "DBO", start=after_loc)),
            },
        }

    if verb == "TT" and args:
        scalar_options = _option_values(args)
        scalar_options.pop("V", None)
        return {
            **_macro_base(verb, args, "trough_stage"),
            "material_id": args[0],
            "volume_ul": volume,
            "tip_type": tip_type,
            "scalar_options": scalar_options,
        }

    if verb == "ZW" and len(args) >= 4:
        first_idx, target_loc, target_zone = _find_location(args)
        _second_idx, source_loc, source_zone = _find_location(args, start=(first_idx or 0) + 1)
        return {
            **_macro_base(verb, args, "zone_wash"),
            "target_location_id": target_loc,
            "target_zone": target_zone,
            "source_location_id": source_loc,
            "source_zone": source_zone,
            "volume_ul": volume,
            "tip_type": tip_type,
            "wash_count": _as_int(_prefixed_value(args, "W")),
            "mix_repeat": _as_int(_prefixed_value(args, "MR")),
            "delay_or_mix_delay": _as_int(_prefixed_value(args, "DM")),
            "material_id": args[-1],
        }

    if verb == "SW" and len(args) >= 4:
        _loc_idx, target_loc, target_zone = _find_location(args, start=1)
        return {
            **_macro_base(verb, args, "standard_wash"),
            "wash_material_id": args[0],
            "target_location_id": target_loc,
            "target_zone": target_zone,
            "volume_ul": volume,
            "tip_type": tip_type,
            "wash_count": _as_int(_prefixed_value(args, "W")),
            "mix_repeat": _as_int(_prefixed_value(args, "MR")),
            "delay_or_mix_delay": _as_int(_prefixed_value(args, "DM")),
            "material_id": args[-1],
        }

    if verb == "RT" and args:
        loc_idx, target_loc, target_zone = _find_location(args)
        after_loc = 0 if loc_idx is None else loc_idx + 1
        return {
            **_macro_base(verb, args, "re_elute"),
            "material_id": args[0],
            "target_location_id": target_loc,
            "target_zone": target_zone,
            "volume_ul": volume,
            "tip_type": tip_type,
            "tip_option": tip_option,
            "option_groups": {
                "DAO": _option_values(_segment_after(args, "DAO", start=after_loc, end_markers={"MO", "DBO"})),
                "MO": _option_values(_segment_after(args, "MO", start=after_loc, end_markers={"DBO"}, last=True)),
                "DBO": _option_values(_segment_after(args, "DBO", start=after_loc)),
            },
        }

    return None


def _source_node_payload(
    *,
    node: ET.Element,
    path: Path,
    script_position: int,
    raw_cmd: str | None = None,
    verb: str | None = None,
    step: str | None = None,
) -> dict[str, Any]:
    payload = {
        "source_type": "oem_xml",
        "source_file": path.name,
        "source_path": str(path),
        "script_position": int(script_position),
        "tag": node.tag,
        "attributes": dict(node.attrib),
        "raw_cmd": raw_cmd,
        "verb": verb,
        "step": step,
    }
    sourceline = getattr(node, "sourceline", None)
    if sourceline is not None:
        payload["xml_line"] = int(sourceline)
    return payload


def _experiment_metadata(root: ET.Element) -> dict[str, Any]:
    node = root.find("./experiment/data")
    return dict(node.attrib) if node is not None else {}


def _inventory_metadata(root: ET.Element) -> dict[str, Any]:
    tips = root.find("./tips")
    reagents = root.find("./reagents")
    oligos = root.find("./oligos")
    strip_entries = [
        {
            "tag": child.tag,
            "attributes": dict(child.attrib),
        }
        for child in root
        if child.tag.startswith("strip")
    ]
    return {
        "tip_count": len(list(tips)) if tips is not None else 0,
        "tip_ids": [child.attrib.get("id") for child in list(tips or ()) if child.attrib.get("id")],
        "reagent_count": len(list(reagents)) if reagents is not None else 0,
        "oligo_count": len(list(oligos)) if oligos is not None else 0,
        "strip_blocks": strip_entries,
    }


def _stage_id_for_step(step_value: str | None, *, ordinal: int) -> str:
    normalized = str(step_value).strip() if step_value is not None else ""
    if normalized.isdigit():
        return f"step-{int(normalized):02d}"
    return f"stage-{ordinal:02d}-{_slugify(normalized, fallback='unnamed')}"


def _compile_supported_action(
    *,
    stage_id: str,
    node: ET.Element,
    path: Path,
    script_position: int,
    raw_cmd: str,
    step: str | None,
) -> ProtocolAction | None:
    tokens = raw_cmd.split()
    if not tokens:
        return None
    verb = tokens[0].upper()
    args = tokens[1:]
    source_node = _source_node_payload(
        node=node,
        path=path,
        script_position=script_position,
        raw_cmd=raw_cmd,
        verb=verb,
        step=step,
    )
    action_id = f"{stage_id}-{_slugify(node.tag, fallback=f'cmd-{script_position:03d}') }"

    kind: ProtocolActionKind
    params: dict[str, Any]
    description: str
    review_required = False
    pause_message: str | None = None

    if verb == "LED" and len(args) >= 3:
        kind = ProtocolActionKind.LED
        params = {"red": _as_int(args[0]), "green": _as_int(args[1]), "blue": _as_int(args[2])}
        description = f"Set chassis LED to RGB ({params['red']}, {params['green']}, {params['blue']})"
    elif verb == "WAIT" and args:
        seconds = _as_float(args[0])
        kind = ProtocolActionKind.WAIT
        params = {"duration_s": seconds}
        description = f"Wait {seconds:g} seconds"
    elif verb == "DELAYPOINT":
        kind = ProtocolActionKind.PAUSE_REVIEW
        params = {"oem_marker": "DELAYPOINT"}
        description = "Pause at OEM delay point"
        review_required = True
        pause_message = "OEM delay point reached"
    elif verb == "PP" and args:
        kind = ProtocolActionKind.PLATE_PREPARE
        params = {"plate_ids": list(args)}
        description = f"Press or seat plate '{args[0]}'"
    elif verb == "TCD" and args:
        mode = args[0].upper()
        kind = ProtocolActionKind.THERMAL_DOOR
        params = {"door_command": mode, "door_state": {"DO": "open", "DC": "closed"}.get(mode, mode.lower())}
        description = f"Thermal cycler door command '{mode}'"
    elif verb == "MC" and len(args) >= 2:
        kind = ProtocolActionKind.MOVE_COVER
        params = {
            "cover_id": args[0],
            "target_location": args[1],
            "extra_args": args[2:],
        }
        description = f"Move cover '{args[0]}' to '{args[1]}'"
    elif verb == "CC" and len(args) >= 2:
        setpoint_c = _as_float(args[1])
        kind = ProtocolActionKind.CHILLER_SETPOINT
        params = {"channel": args[0], "setpoint_c": setpoint_c, "extra_args": args[2:]}
        description = f"Set chiller channel '{args[0]}' to {setpoint_c:g} C"
    elif verb == "SP":
        target_c = _as_float(_prefixed_value(args, "T"))
        duration_s = _as_float(_prefixed_value(args, "DUR"))
        ramp_c_per_s = _as_float(_prefixed_value(args, "R"))
        kind = ProtocolActionKind.THERMAL_SETPOINT
        params = {
            "target_c": target_c,
            "duration_s": duration_s,
            "ramp_c_per_s": ramp_c_per_s,
            "tokens": list(args),
        }
        description = f"Thermal setpoint {target_c:g} C for {duration_s:g} s"
    elif verb == "DWELL" and args:
        seconds = _as_float(args[0])
        kind = ProtocolActionKind.WAIT
        params = {"duration_s": seconds, "oem_marker": "DWELL"}
        description = f"Dwell for {seconds:g} seconds"
    elif verb == "LOOP":
        count = _as_int(args[0]) if args else None
        kind = ProtocolActionKind.LOOP_MARKER
        params = {"count": count, "loop_role": "start" if count is not None else "end"}
        description = "Start OEM loop" if count is not None else "End OEM loop"
    elif verb == "SS":
        kind = ProtocolActionKind.SEAL_SEPARATE
        params = {}
        description = "Separate OEM seals"
    elif verb == "LA" and len(args) >= 3:
        kind = ProtocolActionKind.LIQUID_ADJUST
        params = {
            "plate_id": args[0],
            "zone": args[1],
            "volume_ul": _as_float(args[2]),
            "extra_args": args[3:],
        }
        description = f"Adjust liquid for '{args[0]}' zone '{args[1]}'"
    elif verb == "ET":
        kind = ProtocolActionKind.TIP_EJECT
        params = {}
        description = "Eject current tip set"
    elif verb == "FP":
        translated = OemScriptTranslator().translate_command(
            OemScriptCommand(script_position - 1, verb, raw_cmd, tuple(args), node.tag)
        )
        kind = ProtocolActionKind.PIPETTE_INIT
        params = {
            "semantic_action": "fluid_prep",
            **dict(translated.metadata or {}),
        }
        description = "OEM fluid preparation command"
    elif verb == "MT":
        translated = OemScriptTranslator().translate_command(
            OemScriptCommand(script_position - 1, verb, raw_cmd, tuple(args), node.tag)
        )
        kind = ProtocolActionKind.PIPETTE_ASPIRATE
        params = {
            "semantic_action": "liquid_transfer",
            **dict(translated.metadata or {}),
        }
        description = "OEM liquid transfer command"
    elif verb in {"RT", "SA", "ST", "SW", "TT", "ZW"}:
        macro_params = _compile_oem_macro_params(verb, list(args))
        if macro_params is None:
            return None
        if verb in {"SA", "SW", "ZW"}:
            kind = ProtocolActionKind.PIPETTE_MIX
        elif verb in {"RT", "ST", "TT"}:
            kind = ProtocolActionKind.PIPETTE_ASPIRATE
        else:
            kind = ProtocolActionKind.PIPETTE_DISPENSE
        params = macro_params
        description = f"OEM {macro_params['semantic_action']} macro ({verb})"
    elif verb == "MP" and len(args) >= 2:
        kind = ProtocolActionKind.PLATE_MOVE
        params = {
            "plate_id": args[0],
            "target_location": args[1],
            "move_mode": args[2] if len(args) >= 3 else None,
            "extra_args": args[3:] if len(args) >= 4 else [],
        }
        description = f"Move plate '{args[0]}' to '{args[1]}'"
    else:
        return None

    return ProtocolAction(
        action_id=action_id,
        stage_id=stage_id,
        kind=kind,
        params=params,
        description=description,
        required_capability=infer_required_capability(kind),
        review_required=review_required,
        pause_message=pause_message,
        metadata={
            "source_node": source_node,
            "raw_cmd": raw_cmd,
            "oem_verb": verb,
        },
    )


def import_oem_xml_protocol(path: str | Path) -> ImportedOemProtocol:
    source_path = Path(path).expanduser().resolve()
    tree = ET.parse(source_path)
    root = tree.getroot()
    script = root.find("./script")
    if script is None:
        raise ValueError(f"OEM XML file '{source_path}' does not contain a <script> section")

    experiment = _experiment_metadata(root)
    inventory = _inventory_metadata(root)
    supported_verbs = Counter()
    unsupported_verbs = Counter()
    unsupported_commands: list[UnsupportedOemCommand] = []
    command_nodes_total = 0
    step_markers_total = 0
    stage_ordinal = 0
    stages: list[ProtocolStage] = []
    current_stage_id: str | None = None
    current_stage_title: str | None = None
    current_stage_metadata: dict[str, Any] = {}
    current_actions: list[ProtocolAction] = []
    current_stage_unsupported: list[dict[str, Any]] = []
    current_step_value: str | None = None

    def ensure_stage() -> None:
        nonlocal stage_ordinal, current_stage_id, current_stage_title, current_stage_metadata
        nonlocal current_actions, current_stage_unsupported, current_step_value
        if current_stage_id is not None:
            return
        stage_ordinal += 1
        current_step_value = None
        current_stage_id = f"stage-{stage_ordinal:02d}-preamble"
        current_stage_title = "Preamble"
        current_stage_metadata = {"source": "oem_xml_preamble"}
        current_actions = []
        current_stage_unsupported = []

    def flush_stage() -> None:
        nonlocal current_stage_id, current_stage_title, current_stage_metadata
        nonlocal current_actions, current_stage_unsupported, current_step_value
        if current_stage_id is None:
            return
        stage_metadata = dict(current_stage_metadata)
        if current_stage_unsupported:
            stage_metadata["unsupported_commands"] = list(current_stage_unsupported)
            stage_metadata["unsupported_command_count"] = len(current_stage_unsupported)
        if not current_actions and current_stage_unsupported:
            current_actions = [
                ProtocolAction(
                    action_id=f"{current_stage_id}-unsupported-summary",
                    stage_id=current_stage_id,
                    kind=ProtocolActionKind.NOTE,
                    params={"unsupported_command_count": len(current_stage_unsupported)},
                    description="Stage contains unsupported OEM commands that require manual translation",
                    review_required=True,
                    pause_message="Manual review required: unsupported OEM commands present in stage",
                    metadata={
                        "source": "oem_xml_unsupported_stage",
                        "unsupported_commands": list(current_stage_unsupported),
                    },
                )
            ]
        if current_actions:
            stages.append(
                ProtocolStage(
                    stage_id=current_stage_id,
                    title=current_stage_title,
                    actions=tuple(current_actions),
                    metadata=stage_metadata,
                )
            )
        current_stage_id = None
        current_stage_title = None
        current_stage_metadata = {}
        current_actions = []
        current_stage_unsupported = []
        current_step_value = None

    for script_position, node in enumerate(list(script), start=1):
        step_value = node.attrib.get("step")
        if step_value is not None:
            step_markers_total += 1
            flush_stage()
            stage_ordinal += 1
            current_step_value = str(step_value)
            current_stage_id = _stage_id_for_step(current_step_value, ordinal=stage_ordinal)
            current_stage_title = f"Step {current_step_value}"
            current_stage_metadata = {
                "source_node": _source_node_payload(
                    node=node,
                    path=source_path,
                    script_position=script_position,
                    step=current_step_value,
                ),
                "oem_step": current_step_value,
            }
            current_actions = []
            current_stage_unsupported = []
            continue

        raw_cmd = str(node.attrib.get("cmd", "")).strip()
        if not raw_cmd:
            continue
        ensure_stage()
        command_nodes_total += 1
        verb = raw_cmd.split()[0].upper()
        action = _compile_supported_action(
            stage_id=current_stage_id,
            node=node,
            path=source_path,
            script_position=script_position,
            raw_cmd=raw_cmd,
            step=current_step_value,
        )
        if action is None:
            unsupported_verbs[verb] += 1
            unsupported = UnsupportedOemCommand(
                verb=verb,
                raw_cmd=raw_cmd,
                stage_id=current_stage_id,
                source_node=_source_node_payload(
                    node=node,
                    path=source_path,
                    script_position=script_position,
                    raw_cmd=raw_cmd,
                    verb=verb,
                    step=current_step_value,
                ),
                reason="unsupported_oem_verb",
            )
            unsupported_commands.append(unsupported)
            current_stage_unsupported.append(unsupported.to_payload())
            continue

        supported_verbs[verb] += 1
        current_actions.append(action)

    flush_stage()

    if not stages:
        raise ValueError(f"OEM XML file '{source_path}' did not yield any protocol stages")

    experiment_name = experiment.get("name") or source_path.stem
    coverage = OemXmlCoverage(
        source_file=source_path.name,
        step_markers_total=step_markers_total,
        command_nodes_total=command_nodes_total,
        supported_command_count=sum(supported_verbs.values()),
        unsupported_command_count=sum(unsupported_verbs.values()),
        supported_verbs=dict(sorted(supported_verbs.items())),
        unsupported_verbs=dict(sorted(unsupported_verbs.items())),
        unsupported_commands=tuple(unsupported_commands),
    )
    document = ProtocolDocument(
        protocol_id=f"oem-{_slugify(experiment_name, fallback=source_path.stem)}",
        stages=tuple(stages),
        metadata={
            "source_type": "oem_xml",
            "source_file": source_path.name,
            "source_path": str(source_path),
            "experiment": experiment,
            "inventory": inventory,
            "coverage": coverage.to_payload(),
        },
    )
    return ImportedOemProtocol(
        document=document,
        coverage=coverage,
        source_path=str(source_path),
        experiment=experiment,
        inventory=inventory,
    )


def generate_oem_fixture_coverage_report(paths: Iterable[str | Path]) -> dict[str, Any]:
    imports = [import_oem_xml_protocol(path) for path in paths]
    aggregate_supported = Counter()
    aggregate_unsupported = Counter()
    command_nodes_total = 0
    supported_command_count = 0
    unsupported_command_count = 0

    for imported in imports:
        aggregate_supported.update(imported.coverage.supported_verbs)
        aggregate_unsupported.update(imported.coverage.unsupported_verbs)
        command_nodes_total += imported.coverage.command_nodes_total
        supported_command_count += imported.coverage.supported_command_count
        unsupported_command_count += imported.coverage.unsupported_command_count

    return {
        "file_count": len(imports),
        "files": [
            {
                "source_file": imported.coverage.source_file,
                "protocol_id": imported.document.protocol_id,
                "coverage": imported.coverage.to_payload(),
            }
            for imported in imports
        ],
        "aggregate": {
            "command_nodes_total": command_nodes_total,
            "supported_command_count": supported_command_count,
            "unsupported_command_count": unsupported_command_count,
            "coverage_ratio": 1.0 if command_nodes_total <= 0 else supported_command_count / command_nodes_total,
            "supported_verbs": dict(sorted(aggregate_supported.items())),
            "unsupported_verbs": dict(sorted(aggregate_unsupported.items())),
        },
    }
