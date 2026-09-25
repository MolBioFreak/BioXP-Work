from copy import deepcopy

from bioxp.operator_input_schema import native_form_schema
from bioxp.operator_controls import _extract_inputs


def test_native_schema_preserves_structure_null_and_literal_defaults():
    schema = {
        "anyOf": [{"type": "array", "items": {"type": "integer", "enum": [0, 1, 2, 3]}}, {"type": "null"}],
        "default": [], "description": "Exact channels; null retains native selection.",
    }
    assert native_form_schema(schema, {}) == schema
    for value in (False, 0, "", [], {}, None):
        original = {"type": "object", "default": value}
        assert native_form_schema(original, {}) == original


def test_local_refs_close_without_expanding_recursive_models():
    document = {"components": {"schemas": {
        "Node": {"type": "object", "properties": {
            "enabled": {"type": "boolean", "default": False},
            "next": {"anyOf": [{"$ref": "#/components/schemas/Node"}, {"type": "null"}]},
        }},
    }}}
    before = deepcopy(document)
    result = native_form_schema({"$ref": "#/components/schemas/Node", "description": "native"}, document)
    assert result["$ref"] == "#/$defs/Node"
    assert result["description"] == "native"
    assert result["$defs"]["Node"]["properties"]["next"]["anyOf"][0]["$ref"] == "#/$defs/Node"
    assert document == before


def test_reference_closure_keeps_nested_models_and_only_used_definitions():
    document = {"components": {"schemas": {
        "Step": {"type": "object", "properties": {"target": {"$ref": "#/components/schemas/Target"}}},
        "Target": {"type": "object", "properties": {"well": {"type": "string", "pattern": "^[A-H](?:[1-9]|1[0-2])$"}}},
        "Unused": {"type": "string"},
    }}}
    result = native_form_schema({"type": "array", "items": {"$ref": "#/components/schemas/Step"}}, document)
    assert set(result["$defs"]) == {"Step", "Target"}
    assert result["items"]["$ref"] == "#/$defs/Step"
    assert result["$defs"]["Step"]["properties"]["target"]["$ref"] == "#/$defs/Target"


def test_external_and_unresolved_refs_are_not_fabricated_or_fetched():
    for ref in ("https://example.invalid/native.json", "#/components/schemas/Unknown"):
        assert native_form_schema({"$ref": ref}, {}) == {"$ref": ref}


def test_catalog_retains_nullable_native_objects_and_large_defaults():
    default = ["native-value"] * 600
    document = {"components": {"schemas": {
        "Request": {"type": "object", "required": ["channels"], "properties": {
            "channels": {"anyOf": [{"type": "array", "items": {"type": "integer", "enum": [0, 1, 2, 3]}}, {"type": "null"}], "default": None},
            "source": {"anyOf": [{"$ref": "#/components/schemas/Location"}, {"type": "null"}], "default": None},
            "labels": {"type": "array", "items": {"type": "string"}, "default": default},
        }},
        "Location": {"type": "object", "properties": {"well": {"type": "string"}, "offset": {"type": "integer", "default": 0}}},
    }}}
    operation = {"requestBody": {"required": True, "content": {"application/json": {"schema": {"$ref": "#/components/schemas/Request"}}}}}
    specs, locations = _extract_inputs(operation, document)
    fields = {row["name"]: row for row in specs}
    assert fields["channels"]["required"] is True
    assert fields["channels"]["json_schema"]["anyOf"][1] == {"type": "null"}
    assert fields["channels"]["json_schema"]["anyOf"][0]["items"]["enum"] == [0, 1, 2, 3]
    assert fields["source"]["json_schema"]["$defs"]["Location"]["properties"]["offset"]["default"] == 0
    assert fields["labels"]["default"] == default
    assert locations["source"] == {"location": "body", "wire_name": "source"}


def test_schema_metadata_never_becomes_wire_input():
    specs, locations = _extract_inputs({"parameters": [{"name": "channel", "in": "path", "required": True, "schema": {"type": "integer", "enum": [0, 1, 2, 3]}}]}, {})
    assert specs[0]["json_schema"]["enum"] == [0, 1, 2, 3]
    assert locations == {"channel": {"location": "path", "wire_name": "channel"}}
