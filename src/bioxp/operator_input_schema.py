"""Lossless, self-contained form schemas from the robot's native OpenAPI.

This is presentation metadata, not another input validator or admission rule.
Request models and the existing command owner retain execution authority.
"""
from __future__ import annotations

from copy import deepcopy
from typing import Any, Mapping


def native_form_schema(schema: Mapping[str, Any], document: Mapping[str, Any]) -> dict[str, Any]:
    """Retain native structure and close local component references into $defs.

    Nullable unions, typed enums, false/zero/empty defaults, array items and
    object fields must survive catalog extraction. References are retained
    rather than recursively inlined, so recursive native models stay finite.
    No remote reference is fetched or treated as a locally resolved model.
    """
    components = document.get("components", {}).get("schemas", {})
    definitions: dict[str, Any] = {}
    visiting: set[str] = set()

    def visit(value: Any) -> Any:
        if isinstance(value, Mapping):
            result = {key: visit(item) for key, item in value.items() if key != "$ref"}
            reference = value.get("$ref")
            if isinstance(reference, str):
                prefix = "#/components/schemas/"
                if reference.startswith(prefix):
                    token = reference[len(prefix):]
                    name = token.replace("~1", "/").replace("~0", "~")
                    target = components.get(name) if isinstance(components, Mapping) else None
                    if isinstance(target, Mapping):
                        result["$ref"] = f"#/$defs/{token}"
                        if name not in definitions and name not in visiting:
                            visiting.add(name)
                            definitions[name] = visit(target)
                            visiting.remove(name)
                    else:
                        result["$ref"] = reference
                else:
                    result["$ref"] = reference
            return result
        if isinstance(value, (list, tuple)):
            return [visit(item) for item in value]
        return deepcopy(value)

    result = visit(schema)
    if definitions:
        result["$defs"] = {**result.get("$defs", {}), **definitions}
    return result
