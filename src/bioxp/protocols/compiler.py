from __future__ import annotations

from typing import Any, Mapping

from .models import ProtocolAction, ProtocolDocument, ProtocolStage, normalize_action_kind
from .validators import infer_required_capability, validate_protocol_document


def _action_from_mapping(stage_id: str, index: int, data: Mapping[str, Any]) -> ProtocolAction:
    kind = normalize_action_kind(data.get("kind") or data.get("type"))
    action_id = str(data.get("action_id") or f"{stage_id}:action{index}")
    known_keys = {
        "action_id",
        "kind",
        "type",
        "params",
        "description",
        "review_required",
        "pause_message",
        "message",
    }
    params_payload = data.get("params")
    if isinstance(params_payload, Mapping):
        params = dict(params_payload)
    else:
        params = {
            key: value
            for key, value in data.items()
            if key not in known_keys
        }
    review_required = bool(data.get("review_required", kind.value == "pause_review"))
    pause_message = data.get("pause_message") or data.get("message")
    return ProtocolAction(
        action_id=action_id,
        stage_id=stage_id,
        kind=kind,
        params=params,
        description=(str(data.get("description")) if data.get("description") is not None else None),
        required_capability=infer_required_capability(kind),
        review_required=review_required,
        pause_message=(str(pause_message) if pause_message is not None else None),
        metadata={},
    )


def _stage_from_mapping(index: int, data: Mapping[str, Any]) -> ProtocolStage:
    stage_id = str(data.get("stage_id") or data.get("id") or f"stage_{index}")
    actions = tuple(
        _action_from_mapping(stage_id, action_index, action_data)
        for action_index, action_data in enumerate(data.get("actions", ()), start=1)
    )
    metadata = {
        key: value
        for key, value in data.items()
        if key not in {"stage_id", "id", "title", "actions", "review_required"}
    }
    return ProtocolStage(
        stage_id=stage_id,
        title=(str(data.get("title")) if data.get("title") is not None else None),
        actions=actions,
        review_required=bool(data.get("review_required", False)),
        metadata=metadata,
    )


def compile_native_protocol(data: Mapping[str, Any]) -> ProtocolDocument:
    stages = tuple(
        _stage_from_mapping(index, stage_data)
        for index, stage_data in enumerate(data.get("stages", ()), start=1)
    )
    metadata = {
        key: value
        for key, value in data.items()
        if key not in {"protocol_id", "id", "version", "stages"}
    }
    document = ProtocolDocument(
        protocol_id=str(data.get("protocol_id") or data.get("id") or "protocol"),
        version=int(data.get("version", 1)),
        stages=stages,
        metadata=metadata,
    )
    return validate_protocol_document(document)
