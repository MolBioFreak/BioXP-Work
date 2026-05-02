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
            if not raw:
                continue
            raw = raw.strip()
            if not raw:
                continue
            parts = raw.split()
            verb = parts[0].upper()
            commands.append(
                OemScriptCommand(
                    index=len(commands),
                    verb=verb,
                    raw=raw,
                    args=tuple(parts[1:]),
                    source=elem.tag,
                )
            )
        return cls(root_tag=root.tag, commands=commands, source=source)
