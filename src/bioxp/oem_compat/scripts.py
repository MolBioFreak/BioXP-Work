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


@dataclass(frozen=True)
class TranslatedScript:
    commands: list[TranslatedCommand]


class OemScriptTranslator:
    """Small source-grounded translator for common ClassBioXPScriptHandler verbs."""

    def translate(self, script: OemScript) -> TranslatedScript:
        return TranslatedScript([self.translate_command(c) for c in script.commands])

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
        if verb in {"FP", "LA", "MC", "MP", "MT", "SA", "DWELL", "LOOP"}:
            return TranslatedCommand(cmd.index, verb.lower(), cmd.raw, cmd.args)
        return TranslatedCommand(cmd.index, "unsupported", cmd.raw, cmd.args, supported=False)
