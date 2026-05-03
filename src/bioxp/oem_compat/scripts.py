from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class OemScriptCommand:
    index: int
    verb: str
    raw: str
    args: tuple[str, ...]
    source: str


@dataclass
class OemScript:
    root_tag: str
    commands: list[OemScriptCommand]
    source: str = ""

    @classmethod
    def from_file(cls, path: str | Path) -> "OemScript":
        p = Path(path)
        return cls.from_text(p.read_text(errors="replace"), source=str(p))

    @classmethod
    def from_text(cls, text: str, *, source: str = "inline") -> "OemScript":
        root = ET.fromstring(text)
        commands: list[OemScriptCommand] = []
        for elem in root.iter():
            raw = elem.attrib.get("cmd")
            if raw is None and "step" in elem.attrib:
                raw = f"step {elem.attrib['step']}"
            if not raw:
                continue
            raw = raw.strip()
            if not raw:
                continue
            parts = raw.split()
            commands.append(OemScriptCommand(len(commands), parts[0].upper(), raw, tuple(parts[1:]), elem.tag))
        return cls(root_tag=root.tag, commands=commands, source=source)


@dataclass(frozen=True)
class TranslatedCommand:
    index: int
    command: str
    raw: str
    args: tuple[str, ...] = ()
    supported: bool = True
    metadata: dict[str, object] | None = None


@dataclass(frozen=True)
class TranslatedScript:
    commands: list[TranslatedCommand]


class OemScriptTranslator:
    """Small source-grounded translator for common ClassBioXPScriptHandler verbs."""

    def translate(self, script: OemScript) -> TranslatedScript:
        return TranslatedScript([self.translate_command(c) for c in script.commands])

    @staticmethod
    def _flag_value(args: tuple[str, ...], flag: str) -> str | None:
        flag = flag.upper()
        for token in args:
            if token.upper().startswith(flag):
                return token[len(flag) :]
        return None

    @classmethod
    def _liquid_transfer_metadata(cls, args: tuple[str, ...]) -> dict[str, object]:
        # OEM MT commands are structured token streams. For prep/test handoff we
        # retain the full token stream while extracting the source/destination
        # plate-location/well and volume needed by semantic dry-runs.
        location_positions = [idx for idx, token in enumerate(args) if token.upper().startswith("PL_") or token.upper() == "TROUGH"]
        source_idx = location_positions[0] if location_positions else None
        dest_idx = location_positions[1] if len(location_positions) > 1 else None

        def location_at(idx: int | None) -> tuple[str | None, str | None]:
            if idx is None:
                return None, None
            location_id = args[idx]
            well_id = None
            for token in args[idx + 1 : idx + 5]:
                if token.upper().startswith("/WL") or token.upper().startswith("/ML") or token.upper().startswith("/ZN"):
                    well_id = token[1:]
                    break
            return location_id, well_id

        src_loc, src_well = location_at(source_idx)
        dst_loc, dst_well = location_at(dest_idx)
        volume_text = cls._flag_value(args, "/VL")
        return {
            "semantic_action": "liquid_transfer",
            "source_location_id": src_loc,
            "source_well_id": src_well,
            "dest_location_id": dst_loc,
            "dest_well_id": dst_well,
            "volume_ul": None if volume_text is None else float(volume_text),
            "requires_ack_readback": True,
            "raw_tokens": list(args),
        }

    @classmethod
    def _fluid_prep_metadata(cls, args: tuple[str, ...]) -> dict[str, object]:
        return {
            "semantic_action": "fluid_prep",
            "location_id": args[0] if args else None,
            "well_id": cls._flag_value(args, "/ML") or cls._flag_value(args, "/WL") or cls._flag_value(args, "/ZN"),
            "requires_ack_readback": True,
            "raw_tokens": list(args),
        }

    @classmethod
    def _macro_metadata(cls, verb: str, args: tuple[str, ...]) -> dict[str, object]:
        semantic = {
            "RT": "re_elute",
            "SA": "material_agitate",
            "ST": "material_transfer",
            "SW": "standard_wash",
            "TT": "trough_stage",
            "ZW": "zone_wash",
        }[verb]
        return {
            "semantic_action": semantic,
            "macro_verb": verb,
            "requires_virtual_bioxp_state": True,
            "requires_ack_readback": True,
            "raw_tokens": list(args),
        }

    def translate_command(self, cmd: OemScriptCommand) -> TranslatedCommand:
        verb = cmd.verb.upper()
        if verb == "LED":
            return TranslatedCommand(cmd.index, "led", cmd.raw, cmd.args)
        if verb == "WAIT":
            return TranslatedCommand(cmd.index, "wait", cmd.raw, cmd.args)
        if verb == "TCD":
            return TranslatedCommand(cmd.index, "tcd", cmd.raw, cmd.args)
        if verb == "ET":
            return TranslatedCommand(cmd.index, "ejt", cmd.raw, cmd.args)
        if verb == "CC":
            return TranslatedCommand(cmd.index, "cc", cmd.raw, cmd.args)
        if verb == "DELAYPOINT":
            return TranslatedCommand(cmd.index, "delaypoint", cmd.raw, cmd.args)
        if verb == "PP":
            return TranslatedCommand(cmd.index, "pressp", cmd.raw, cmd.args)
        if verb == "SP":
            return TranslatedCommand(cmd.index, "sp", cmd.raw, cmd.args)
        if verb == "STEP":
            return TranslatedCommand(cmd.index, "step", cmd.raw, cmd.args)
        if verb == "MT":
            return TranslatedCommand(cmd.index, "liquid_transfer", cmd.raw, cmd.args, metadata=self._liquid_transfer_metadata(cmd.args))
        if verb == "FP":
            return TranslatedCommand(cmd.index, "fluid_prep", cmd.raw, cmd.args, metadata=self._fluid_prep_metadata(cmd.args))
        if verb in {"RT", "SA", "ST", "SW", "TT", "ZW"}:
            return TranslatedCommand(cmd.index, verb.lower(), cmd.raw, cmd.args, metadata=self._macro_metadata(verb, cmd.args))
        if verb in {"LA", "MC", "MP", "DWELL", "LOOP"}:
            return TranslatedCommand(cmd.index, verb.lower(), cmd.raw, cmd.args)
        return TranslatedCommand(cmd.index, "unsupported", cmd.raw, cmd.args, supported=False)
