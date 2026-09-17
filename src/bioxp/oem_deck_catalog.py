from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from typing import Any

from .oem_compat.pathing import LOCATION_NAME_TO_ID
from .oem_compat.position_table import PositionTable


@dataclass(frozen=True)
class DeckDestination:
    target: str
    panel_label: str
    location_name: str
    location_id: int
    branch: str
    aliases: tuple[str, ...] = ()


_ROWS = (
    ("LOC_MS", "LOC_MS", "LOC_MS", "ordinary"), ("LOC_OC", "LOC_OC", "LOC_OC", "ordinary"),
    ("TECANRACK2", "TIP TRAY 2", "TECANRACK2", "ordinary"), ("TECANRACK4", "TIP TRAY_4", "TECANRACK4", "ordinary"),
    ("LOC_P_MS", "LOC_MS_PLATE", "LOC_P_MS", "ordinary"), ("LOC_P_OC", "LOC_OC_PLATE", "LOC_P_OC", "ordinary"),
    ("LOC_OC_COVER", "LOC_OC_COVER", "LOC_OC_COVER", "ordinary"), ("LOC_TC", "LOC_TC", "LOC_TC", "ordinary"),
    ("LOC_TC_BARCODE", "LOC_TC_BARCODE", "LOC_TC", "barcode"), ("LOC_RC", "LOC_RC", "LOC_RC", "ordinary"),
    ("LOC_RC_BARCODE", "LOC_RC_BARCODE", "LOC_RC", "barcode"), ("TECANRACK1", "TIP TRAY 1", "TECANRACK1", "ordinary"),
    ("TECANRACK3", "TIP TRAY_3", "TECANRACK3", "ordinary"), ("LOC_P_TC", "LOC_TC_PLATE", "LOC_P_TC", "ordinary"),
    ("LOC_BSC", "LOC_TC_COVER", "LOC_BSC", "ordinary"), ("LOC_RC_COVER", "LOC_RC_COVER", "LOC_RC_COVER", "ordinary"),
    ("LOC_STRIP1", "STRIP 1", "LOC_STRIP1", "ordinary"), ("LOC_STRIP2", "STRIP 2", "LOC_STRIP2", "ordinary"),
    ("LOC_STRIP3", "STRIP 3", "LOC_STRIP3", "ordinary"), ("LOC_STRIP4", "STRIP 4", "LOC_STRIP4", "ordinary"),
    ("LOC_OC_COVER_STORAGE", "LOC_OC_COVER_S", "LOC_OC_COVER_STORAGE", "ordinary"),
    ("LOC_RC_COVER_STORAGE", "LOC_RC_COVER_S", "LOC_RC_COVER_STORAGE", "ordinary"),
    ("LOC_TROUGH", "LOC_TROUGH1", "LOC_TROUGH", "ordinary"), ("LOC_BSCS", "LOC_BSCS", "LOC_BSCS", "ordinary"),
    ("WASTE_BIN", "Waste Bin", "WASTE_BIN", "ordinary"), ("LOC_PARK", "Park", "LOC_PARK", "park"),
)
_ALIASES = {
    "OC chiller": "LOC_OC", "Output Chiller": "LOC_OC", "Output Tray": "LOC_OC",
    "Reagent Chiller": "LOC_RC", "Thermal Cycler": "LOC_TC", "Magnetic Station": "LOC_MS",
    "Waste Bin": "WASTE_BIN", "Park": "LOC_PARK",
}


def configured_location_names() -> tuple[str, ...]:
    return tuple(dict.fromkeys(row[2] for row in _ROWS))


def public_target_keys() -> frozenset[str]:
    return frozenset(row[0] for row in _ROWS)


class DeckCatalog:
    def __init__(self, entries: tuple[DeckDestination, ...], position_table_sha256: str) -> None:
        self._entries = entries
        self.position_table_sha256 = position_table_sha256
        self._targets = {entry.target: entry for entry in entries}
        self._aliases = {alias.casefold(): target for alias, target in _ALIASES.items()}
        canonical = {"schema_version": "bioxp.oem_deck_catalog.v1", "position_table_sha256": position_table_sha256, "rows": self.rows()}
        self.revision = hashlib.sha256(json.dumps(canonical, sort_keys=True, separators=(",", ":")).encode()).hexdigest()

    @classmethod
    def from_position_table(cls, table: PositionTable) -> "DeckCatalog":
        missing = [name for name in configured_location_names() if not cls._contains(table, name)]
        if missing:
            raise ValueError(f"catalog destination absent from Serial-206 PositionTable: {', '.join(missing)}")
        aliases_by_target: dict[str, list[str]] = {}
        for alias, target in _ALIASES.items():
            aliases_by_target.setdefault(target, []).append(alias)
        entries = tuple(DeckDestination(target, label, name, LOCATION_NAME_TO_ID[name], branch, tuple(aliases_by_target.get(target, ()))) for target, label, name, branch in _ROWS)
        return cls(entries, table.digest)

    @staticmethod
    def _contains(table: PositionTable, name: str) -> bool:
        try:
            table.resolve(location_id=name)
        except KeyError:
            return False
        return True

    def resolve(self, target: str) -> DeckDestination:
        try:
            return self._targets[str(target)]
        except KeyError as exc:
            raise KeyError(f"unknown finite deck target: {target}") from exc

    def resolve_alias(self, alias: str) -> DeckDestination:
        target = self._aliases.get(str(alias).casefold())
        if target is None:
            raise KeyError(f"unknown finite deck alias: {alias}")
        return self.resolve(target)

    def rows(self) -> list[dict[str, Any]]:
        return [{"target": row.target, "panel_label": row.panel_label, "location_name": row.location_name, "location_id": row.location_id, "branch": row.branch, "aliases": list(row.aliases), "position_table_sha256": self.position_table_sha256} for row in self._entries]
