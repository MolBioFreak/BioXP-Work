from __future__ import annotations

from hashlib import sha256
import json
from typing import Any, Mapping

from ..domain.capabilities import CapabilityName
from .models import ProtocolAction, ProtocolActionKind, ProtocolDocument, ProtocolStage, normalize_action_kind, _capture, _payload
from .validators import infer_required_capability, validate_protocol_document


def _action_from_mapping(stage_id: str, index: int, data: Mapping[str, Any]) -> ProtocolAction:
    kind = normalize_action_kind(data.get("kind") or data.get("type"))
    action_id = str(data.get("action_id") or f"{stage_id}:action{index}")
    known_keys = {
        "action_id", "stage_id", "kind", "type", "params", "description",
        "review_required", "pause_message", "message", "metadata",
        "required_capability", "oem_opcode", "source_occurrence_id", "source_key",
    }
    params_payload = data.get("params")
    params = dict(params_payload) if isinstance(params_payload, Mapping) else {
        key: value for key, value in data.items() if key not in known_keys
    }
    pause_message = data.get("pause_message") or data.get("message")
    capability = data.get("required_capability")
    return ProtocolAction(
        action_id=action_id,
        stage_id=str(data.get("stage_id", stage_id)),
        kind=kind,
        params=params,
        description=data.get("description"),
        required_capability=(CapabilityName(capability) if capability is not None else
                             None if "required_capability" in data else infer_required_capability(kind)),
        review_required=bool(data.get("review_required", kind == ProtocolActionKind.PAUSE_REVIEW)),
        pause_message=pause_message,
        metadata=dict(data.get("metadata") or {}),
        oem_opcode=data.get("oem_opcode"),
        source_occurrence_id=data.get("source_occurrence_id"),
        source_key=data.get("source_key"),
    )


def _metadata(data: Mapping[str, Any], known_keys: set[str]) -> dict[str, Any]:
    metadata = dict(data.get("metadata") or {})
    metadata.update({key: value for key, value in data.items() if key not in known_keys | {"metadata"}})
    return metadata


def _stage_from_mapping(index: int, data: Mapping[str, Any]) -> ProtocolStage:
    stage_id = str(data.get("stage_id") or data.get("id") or f"stage_{index}")
    return ProtocolStage(
        stage_id=stage_id,
        title=data.get("title"),
        actions=tuple(_action_from_mapping(stage_id, i, action) for i, action in enumerate(data.get("actions", ()), 1)),
        review_required=bool(data.get("review_required", False)),
        metadata=_metadata(data, {"stage_id", "id", "title", "actions", "review_required"}),
    )


def compile_native_protocol(data: Mapping[str, Any]) -> ProtocolDocument:
    if "operations" in data:
        if "stages" in data:
            raise ValueError("Prepared operations and native stages cannot be combined")
        return compile_prepared_oem_protocol(data)
    document = ProtocolDocument(
        protocol_id=str(data.get("protocol_id") or data.get("id") or "protocol"),
        version=int(data.get("version", 1)),
        stages=tuple(_stage_from_mapping(i, stage) for i, stage in enumerate(data.get("stages", ()), 1)),
        metadata=_metadata(data, {"protocol_id", "id", "version", "stages"}),
    )
    return validate_protocol_document(document)


def compile_prepared_oem_protocol(data: Mapping[str, Any]) -> ProtocolDocument:
    """Capture already expanded source membership. Never generate or expand recipes."""
    captured = _payload(_capture(data))
    if set(captured) - {"protocol_id", "version", "metadata", "operations"}:
        raise ValueError("Unknown prepared protocol fields")
    if captured.get("version", 1) != 1:
        raise ValueError("Unsupported prepared protocol version")
    operations = captured.get("operations")
    if not isinstance(operations, list) or not operations:
        raise ValueError("Prepared protocol requires an ordered operations sequence")
    metadata = dict(captured.get("metadata") or {})
    metadata.setdefault("execution_mode", "normal")
    metadata["input_mode"] = "oem_prepared"
    metadata["source_sha256"] = sha256(json.dumps(captured, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
    actions = []
    for ordinal, operation in enumerate(operations, 1):
        if not isinstance(operation, Mapping) or set(operation) - {
            "oem_opcode", "arguments", "argument_type", "source_key", "source_occurrence_id", "metadata",
        }:
            raise ValueError(f"Invalid prepared operation at ordinal {ordinal}")
        occurrence = operation.get("source_occurrence_id", f"oem:{ordinal}")
        params = {"arguments": operation.get("arguments")}
        if "argument_type" in operation:
            params["argument_type"] = operation["argument_type"]
        source_metadata = dict(operation.get("metadata") or {})
        source_metadata["source_ordinal"] = ordinal
        actions.append(ProtocolAction(
            action_id=occurrence, stage_id="oem", kind=ProtocolActionKind.OEM_OPERATION,
            oem_opcode=operation.get("oem_opcode"), source_occurrence_id=occurrence,
            source_key=operation.get("source_key"), params=params, metadata=source_metadata,
        ))
    return validate_protocol_document(ProtocolDocument(
        protocol_id=str(captured.get("protocol_id") or "oem-prepared"),
        stages=(ProtocolStage(stage_id="oem", actions=tuple(actions)),), metadata=metadata,
    ))


def compile_oem_core_script(text: str, *, protocol_id: str = "oem-core-script", metadata: Mapping[str, Any] | None = None) -> ProtocolDocument:
    """Lower the raw source-token/opcode string path, retaining newline ordering."""
    operations = []
    source_map = []
    for ordinal, line in enumerate(text.split("\n"), 1):
        stripped = line.strip()
        source_map.append({"source_ordinal": ordinal, "raw_line": line})
        if not stripped or stripped.startswith("//"):
            continue
        # OEM splits on literal spaces, preserving empty argument tokens.
        tokens = stripped.split(" ")
        if len(tokens) < 2:
            raise ValueError(f"Missing raw source token/opcode at line {ordinal}")
        try:
            int(tokens[0])
        except ValueError as exc:
            raise ValueError(f"Invalid raw source key at line {ordinal}") from exc
        operations.append({
            "source_key": tokens[0], "source_occurrence_id": f"raw:{ordinal}",
            "oem_opcode": tokens[1], "arguments": tokens[2:],
            "metadata": {"raw_line": line, "raw_ordinal": ordinal},
        })
    document = compile_prepared_oem_protocol({"protocol_id": protocol_id, "metadata": dict(metadata or {}), "operations": operations})
    captured_metadata = _payload(document.metadata)
    captured_metadata.update(input_mode="oem_core_script", source_sha256=sha256(text.encode()).hexdigest(), source_map=source_map)
    return ProtocolDocument(protocol_id=document.protocol_id, stages=document.stages, metadata=captured_metadata)
