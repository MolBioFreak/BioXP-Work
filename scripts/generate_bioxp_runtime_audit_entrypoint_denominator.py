from __future__ import annotations

import argparse
import ast
import hashlib
import json
import re
from pathlib import Path
from typing import Any, Iterable

SCHEMA = "bioxp.runtime_audit.entrypoint_denominator.v1"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _relative(root: Path, path: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def _source_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _ast(path: Path) -> ast.AST:
    return ast.parse(_source_text(path), filename=str(path))


def _call_names(node: ast.AST) -> set[str]:
    names: set[str] = set()
    for child in ast.walk(node):
        if isinstance(child, ast.Call):
            try:
                names.add(ast.unparse(child.func))
            except Exception:
                continue
    return names


def _add_row(rows: list[dict[str, Any]], *, family: str, root: Path, path: Path,
             line: int, identifier: str, control_class: str,
             requires_durable_claim: bool, transport_effect: str,
             dispatch_path: str, verification_id: str) -> None:
    relative = _relative(root, path)
    row_id = f"{family}:{relative}:{line}:{identifier}"
    rows.append({
        "id": row_id,
        "family": family,
        "source_file": relative,
        "source_sha256": _sha256(path),
        "source_line": int(line),
        "identifier": identifier,
        "control_class": control_class,
        "requires_durable_claim": bool(requires_durable_claim),
        "transport_effect": transport_effect,
        "dispatch_path": dispatch_path,
        "verification_id": verification_id,
    })


def _route_rows(root: Path, rows: list[dict[str, Any]]) -> None:
    path = root / "src/bioxp/api.py"
    tree = _ast(path)
    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        for decorator in node.decorator_list:
            if not isinstance(decorator, ast.Call):
                continue
            if not isinstance(decorator.func, ast.Attribute):
                continue
            if not isinstance(decorator.func.value, ast.Name) or decorator.func.value.id != "app":
                continue
            if not decorator.args or not isinstance(decorator.args[0], ast.Constant):
                continue
            route = decorator.args[0].value
            if not isinstance(route, str) or not route.startswith("/liquid"):
                continue
            method = decorator.func.attr.upper()
            calls = _call_names(node)
            projection = route in {"/liquid/status", "/liquid/application/status"}
            plan_only = route.endswith("/plan")
            hardware_query = method == "GET" and route in {
                "/liquid/data",
                "/liquid/pressure",
                "/liquid/condition",
                "/liquid/status/readback",
                "/liquid/fluid-detection/{channel}/timestamp",
            }
            if projection:
                control_class = "projection_only"
                effect = "none"
                claim = False
                dispatch = "projection_only"
            elif plan_only:
                control_class = "operator_plan"
                effect = "command_or_plan"
                claim = False
                dispatch = "plan_only"
            elif hardware_query:
                control_class = "hardware_query"
                effect = "query"
                claim = True
                dispatch = "coordinator" if any(name.startswith("run_pipette_") for name in calls) else "direct_transport"
            else:
                control_class = "physical_liquid_command" if method == "POST" else "operator_plan"
                effect = "command_or_plan"
                claim = control_class == "physical_liquid_command"
                dispatch = "coordinator" if any(name.startswith("run_pipette_") for name in calls) else "direct_transport"
            _add_row(
                rows,
                family="robot_route",
                root=root,
                path=path,
                line=node.lineno,
                identifier=f"{method} {route}",
                control_class=control_class,
                requires_durable_claim=claim,
                transport_effect=effect,
                dispatch_path=dispatch,
                verification_id=f"RW1.ROUTE.{method}.{route}",
            )


def _protocol_rows(root: Path, rows: list[dict[str, Any]]) -> None:
    path = root / "src/bioxp/api.py"
    tree = _ast(path)
    for node in ast.walk(tree):
        if not isinstance(node, ast.FunctionDef) or node.name != "_protocol_live_pipette_handler":
            continue
        for child in ast.walk(node):
            if not isinstance(child, ast.If):
                continue
            test = ast.unparse(child.test)
            if "ProtocolActionKind." not in test:
                continue
            identifier = test
            dispatch = "coordinator" if any(name.startswith("run_pipette_") for name in _call_names(child)) else "direct_transport"
            _add_row(
                rows,
                family="protocol_handler",
                root=root,
                path=path,
                line=child.lineno,
                identifier=identifier,
                control_class="physical_liquid_command",
                requires_durable_claim=True,
                transport_effect="command",
                dispatch_path=dispatch,
                verification_id=f"RW1.PROTOCOL.{child.lineno}",
            )


def _lifecycle_rows(root: Path, rows: list[dict[str, Any]]) -> None:
    candidates = [
        root / "src/bioxp/api.py",
        root / "src/bioxp/oem_serial206_initialization.py",
    ]
    for path in candidates:
        tree = _ast(path)
        adapter_coordinator = path.name == "oem_serial206_initialization.py" and "def _run_audited_pipette" in _source_text(path)
        for node in ast.walk(tree):
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            name = node.name.lower()
            if not any(token in name for token in ("pipette", "tip_status", "eject_all", "initialize")):
                continue
            calls = _call_names(node)
            if not any("pipette" in call.lower() or "tip_status" in call.lower() for call in calls):
                continue
            if path.name == "api.py" and name not in {"_constructor_pipette_action", "liquid_init"}:
                continue
            dispatch = "coordinator" if adapter_coordinator or any(
                token in call for call in calls for token in ("run_pipette_", "_run_serial206_pipette_audit")
            ) else "direct_transport"
            _add_row(
                rows,
                family="lifecycle_caller",
                root=root,
                path=path,
                line=node.lineno,
                identifier=node.name,
                control_class="hardware_query" if "query" in name or "status" in name else "pipette_state_command",
                requires_durable_claim=True,
                transport_effect="query_or_command",
                dispatch_path=dispatch,
                verification_id=f"RW1.LIFECYCLE.{path.name}.{node.name}",
            )


def _transport_rows(root: Path, rows: list[dict[str, Any]]) -> None:
    for path in sorted((root / "src/bioxp").rglob("*.py")):
        if "__pycache__" in path.parts:
            continue
        tree = _ast(path)
        for node in ast.walk(tree):
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            if node.name.startswith("_"):
                continue
            calls = _call_names(node)
            if not any(token in " ".join(calls).lower() for token in ("pipette", "can", "novo", "query_pressure", "query_tip_status")):
                continue
            if not any(token in node.name.lower() for token in ("pipette", "tip", "pressure", "aspirat", "dispens", "mix", "eject", "fluid", "heartbeat", "diagnos", "error", "data", "status", "initialize", "terminate", "speed")):
                continue
            _add_row(
                rows,
                family="transport_method",
                root=root,
                path=path,
                line=node.lineno,
                identifier=node.name,
                control_class="callback_event" if any(token in node.name.lower() for token in ("callback", "event", "process_message")) else "hardware_query" if "query" in node.name.lower() or "status" in node.name.lower() else "pipette_state_command",
                requires_durable_claim=True,
                transport_effect="transport_owner_method",
                dispatch_path="transport_owner",
                verification_id=f"RW1.TRANSPORT.{_relative(root, path)}.{node.name}",
            )


def _bms_rows(root: Path | None, rows: list[dict[str, Any]]) -> None:
    if root is None:
        return
    candidates = [
        root / "platform/api/routers/bioxp/operator_controls.py",
        root / "platform/api/services/bioxp/robot_client.py",
        root / "platform/api/services/bioxp/operator_models.py",
    ]
    for path in candidates:
        if not path.is_file():
            raise FileNotFoundError(path)
        tree = _ast(path)
        for node in ast.walk(tree):
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            name = node.name.lower()
            if not any(token in name for token in ("report", "export", "robot", "pipette", "operator_control", "summary", "detail")):
                continue
            _add_row(
                rows,
                family="bms_relay",
                root=root,
                path=path,
                line=node.lineno,
                identifier=node.name,
                control_class="read_only_relay",
                requires_durable_claim=False,
                transport_effect="robot_api_relay",
                dispatch_path="typed_relay",
                verification_id=f"RW4.BMS.{_relative(root, path)}.{node.name}",
            )
    frontend = [
        root / "platform/frontend/src/components/BioXpCockpit.tsx",
        root / "platform/frontend/src/components/BioXpOperatorControlTabs.tsx",
        root / "platform/frontend/src/components/BioXpOperatorReports.tsx",
        root / "platform/frontend/src/components/BioXpPipetteControlPanel.tsx",
    ]
    for path in frontend:
        if not path.is_file():
            raise FileNotFoundError(path)
        text = _source_text(path)
        for index, match in enumerate(re.finditer(r"(?:reports?|pipette|liquid|export|refresh)", text, flags=re.I), start=1):
            line = text.count("\n", 0, match.start()) + 1
            _add_row(
                rows,
                family="cockpit_action",
                root=root,
                path=path,
                line=line,
                identifier=f"{path.name}:{index}",
                control_class="read_only_report_surface" if "report" in path.name.lower() else "pipette_surface",
                requires_durable_claim=False,
                transport_effect="typed_bms_action",
                dispatch_path="typed_client",
                verification_id=f"RW4.UI.{path.name}.{line}",
            )


def generate_denominator(robot_root: str | Path, *, bms_root: str | Path | None = None,
                         output_path: str | Path | None = None) -> dict[str, Any]:
    robot = Path(robot_root).resolve()
    bms = None if bms_root is None else Path(bms_root).resolve()
    rows: list[dict[str, Any]] = []
    _route_rows(robot, rows)
    _protocol_rows(robot, rows)
    _lifecycle_rows(robot, rows)
    _transport_rows(robot, rows)
    _bms_rows(bms, rows)
    rows.sort(key=lambda row: (row["family"], row["source_file"], row["source_line"], row["identifier"]))
    identities = [(row["family"], row["source_file"], row["source_line"], row["identifier"]) for row in rows]
    duplicate_count = len(identities) - len(set(identities))
    unclassified_count = sum(1 for row in rows if not row["control_class"] or not row["verification_id"])
    bypass_count = sum(1 for row in rows if row["requires_durable_claim"] and row["dispatch_path"] == "direct_transport")
    missing_sources = []
    for row in rows:
        source = (robot if row["source_file"].startswith("src/") else bms)
        if source is None:
            continue
        if not (source / row["source_file"]).is_file():
            missing_sources.append(row["source_file"])
    payload = {
        "schema": SCHEMA,
        "generator": "scripts/generate_bioxp_runtime_audit_entrypoint_denominator.py",
        "robot_root": "robot_source",
        "bms_root": None if bms is None else "bms_source",
        "rows": rows,
        "invariants": {
            "row_count": len(rows),
            "unclassified_count": unclassified_count,
            "direct_transport_bypass_count": bypass_count,
            "duplicate_identity_count": duplicate_count,
            "missing_source_count": len(missing_sources),
            "missing_sources": sorted(set(missing_sources)),
        },
    }
    if output_path is not None:
        output = Path(output_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_bytes(json.dumps(payload, indent=2, sort_keys=True).encode("utf-8") + b"\n")
    return payload


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--bms-root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    payload = generate_denominator(args.robot_root, bms_root=args.bms_root, output_path=args.output)
    if payload["invariants"]["unclassified_count"] or payload["invariants"]["direct_transport_bypass_count"] or payload["invariants"]["duplicate_identity_count"] or payload["invariants"]["missing_source_count"]:
        raise SystemExit(json.dumps(payload["invariants"], sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
