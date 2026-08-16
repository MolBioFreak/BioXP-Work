#!/usr/bin/env python3
"""Build the row-level OEM pipette application call-site denominator.

The generator reads only lock-pinned decompiled Controlsuite sources. It does not
load BioXP, access USB/CAN, or issue any robot command.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path
from typing import Any

SOURCE_SPECS = (
    ("decompiled_src/BioXPControlLib/ControlLib.cs", ("m_PipetteControl",)),
    ("decompiled_src/BioXPControlLib/WindowBoardTest.cs", ("m_PipetteControl",)),
    (
        "decompiled_src_genbotapp/GenBotApp/BioXPMainWindow.cs",
        ("m_control.m_PipetteControl",),
    ),
)

METHOD_RE = re.compile(
    r"^\s*(?:public|private|protected|internal)\s+"
    r"(?:(?:static|unsafe|virtual|override|sealed|async)\s+)*"
    r"(?:[A-Za-z_][A-Za-z0-9_<>,.\[\]?]*\s+)?"
    r"(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*\("
)


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _normalized_expression(line: str) -> str:
    return " ".join(line.strip().split())


def _source_member(lines: list[str], line_index: int) -> str:
    current = "<class-body>"
    for candidate in lines[: line_index + 1]:
        match = METHOD_RE.match(candidate)
        if match is not None:
            current = match.group("name")
    return current


def _classify_expression(expression: str) -> tuple[str, str, str]:
    if "new ClassPipetteCollection" in expression:
        return (
            "oem_literal",
            "blocked",
            "constructor ownership and distinct BoardTest/application lifecycle policy require WP3 integration",
        )
    if "m_diagnosticPanel.m_PipetteControl" in expression or "m_pipette." in expression:
        return (
            "product_extension",
            "blocked",
            "OEM operator control binding is source-anchored and awaits the typed WP4 operator mapping",
        )
    return (
        "oem_literal",
        "blocked",
        "row-level OEM call site is source-anchored and awaits command/state/application semantic reconciliation",
    )


def extract_application_call_sites(oem_root: Path) -> tuple[list[dict[str, Any]], dict[str, str]]:
    rows: list[dict[str, Any]] = []
    source_hashes: dict[str, str] = {}
    for relative_path, symbols in SOURCE_SPECS:
        source = oem_root / relative_path
        data = source.read_bytes()
        source_hash = _sha256(data)
        source_hashes[relative_path] = source_hash
        lines = data.decode("utf-8-sig").splitlines()
        for line_index, line in enumerate(lines):
            if not any(symbol in line for symbol in symbols):
                continue
            expression = _normalized_expression(line)
            classification, status, blocker = _classify_expression(expression)
            rows.append(
                {
                    "source_file": relative_path,
                    "source_sha256": source_hash,
                    "source_line": line_index + 1,
                    "source_member": _source_member(lines, line_index),
                    "call_expression": expression,
                    "classification": classification,
                    "status": status,
                    "blocker": blocker,
                }
            )

    rows.sort(key=lambda row: (row["source_file"], row["source_line"], row["call_expression"]))
    for index, row in enumerate(rows, start=1):
        row["id"] = f"APP-CS-{index:04d}"
    return rows, source_hashes


def _replace_gap_status(matrix: dict[str, Any], gap_id: str, *, status: str, blocker: str) -> None:
    for row in matrix["gap_rows"]:
        if row.get("id") == gap_id:
            row["status"] = status
            row["blocker"] = blocker
            return
    raise ValueError(f"missing gap row {gap_id}")


def build_matrix(*, matrix_path: Path, oem_root: Path) -> dict[str, Any]:
    matrix = json.loads(matrix_path.read_text(encoding="utf-8"))
    rows, source_hashes = extract_application_call_sites(oem_root)
    if not rows:
        raise ValueError("no OEM pipette application call sites were extracted")

    matrix["denominator"]["application_call_sites"] = rows
    matrix["authority"]["application_call_site_sources"] = source_hashes
    matrix["matrix_invariants"]["application_call_site_count"] = len(rows)
    matrix["matrix_invariants"]["application_call_site_unclassified_count"] = sum(
        1 for row in rows if not row.get("classification") or not row.get("status")
    )
    _replace_gap_status(
        matrix,
        "G-002",
        status="blocked",
        blocker=(
            f"{len(rows)} application call-site rows are materialized; row-level Linux semantic "
            "reconciliation and evidence remain open"
        ),
    )
    return matrix


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix", type=Path, required=True)
    parser.add_argument("--oem-root", type=Path, required=True)
    args = parser.parse_args()

    matrix = build_matrix(matrix_path=args.matrix, oem_root=args.oem_root)
    args.matrix.write_text(json.dumps(matrix, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
