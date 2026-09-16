"""Pure captured-input tests. Run only in the task's denied-IO robot runner."""
from hashlib import sha256
import json

import pytest

from bioxp.protocols import (
    ProtocolAction, ProtocolActionKind, ProtocolDocument, ProtocolStage,
    compile_native_protocol, compile_oem_core_script, compile_prepared_oem_protocol,
    import_oem_xml_protocol, validate_protocol_support,
)
from bioxp.protocols.models import OEM_OPERATION_FORMS, OEM_RAW_MIN_ARGUMENTS


def prepared(*operations, metadata=None):
    return compile_prepared_oem_protocol({"protocol_id": "captured", "metadata": metadata or {}, "operations": list(operations)})


def raw(opcode="park", arguments=None, **kwargs):
    return {"oem_opcode": opcode, "arguments": arguments if arguments is not None else [], **kwargs}


def actions(document):
    return [action for stage in document.stages for action in stage.actions]


def test_native_authoring_hydration_retains_all_metadata_and_source_fields():
    payload = {
        "protocol_id": "native", "version": 1, "metadata": {"nested": {"ids": [1, 2]}},
        "stages": [{"stage_id": "s", "metadata": {"stage": [None]}, "actions": [{
            "action_id": "a", "stage_id": "s", "kind": "note", "params": {"note": ["one"]},
            "description": "source marker", "review_required": False, "pause_message": None,
            "metadata": {"source": {"repeat": 2}}, "source_occurrence_id": "marker:2",
            "source_key": 17, "oem_opcode": None, "required_capability": None,
        }]}],
    }
    document = compile_native_protocol(payload)
    assert document.to_payload() == ProtocolDocument.from_payload(payload).to_payload()
    assert compile_native_protocol(document.to_payload()).to_payload() == document.to_payload()
    payload["metadata"]["nested"]["ids"].append(3)
    payload["stages"][0]["actions"][0]["params"]["note"].append("two")
    assert document.metadata["nested"]["ids"] == (1, 2)
    assert actions(document)[0].params["note"] == ("one",)
    with pytest.raises(TypeError):
        document.metadata["nested"]["ids"] = ()
    output = document.to_payload()
    output["stages"][0]["actions"][0]["metadata"]["source"]["repeat"] = 99
    assert actions(document)[0].metadata["source"]["repeat"] == 2
    json.dumps(document.to_payload(), allow_nan=False)


def test_direct_model_construction_freezes_all_levels():
    params = {"arguments": ["1"]}
    metadata = {"bindings": [{"id": "a"}]}
    action = ProtocolAction("a", "s", ProtocolActionKind.NOTE, params=params, metadata=metadata)
    stage_actions = [action]
    stages = [ProtocolStage("s", actions=stage_actions, metadata=metadata)]
    document = ProtocolDocument("p", stages=stages, metadata=metadata)
    params["arguments"].append("2")
    metadata["bindings"][0]["id"] = "mutated"
    stage_actions.clear()
    stages.clear()
    assert actions(document)[0].params["arguments"] == ("1",)
    assert document.stages[0].metadata["bindings"][0]["id"] == "a"


@pytest.mark.parametrize("bad", [lambda: None, object(), {1: "key"}, float("nan"), float("inf")])
def test_capture_rejects_live_objects_and_non_json_values(bad):
    with pytest.raises(ValueError):
        ProtocolAction("a", "s", ProtocolActionKind.NOTE, params={"value": bad})


@pytest.mark.parametrize("opcode,form", list(OEM_OPERATION_FORMS.items()))
def test_complete_source_roster_can_be_captured_without_claiming_native_support(opcode, form):
    operation = raw(opcode, ["opaque"] * OEM_RAW_MIN_ARGUMENTS.get(opcode, 0))
    if form != "raw":
        operation.update(arguments={}, argument_type=form)
    document = prepared(operation)
    action = actions(document)[0]
    assert action.oem_opcode == opcode
    assert action.kind == ProtocolActionKind.OEM_OPERATION
    assert ProtocolDocument.from_payload(document.to_payload()).to_payload() == document.to_payload()


def test_prepared_membership_order_is_not_numeric_key_order_and_not_expanded_twice():
    document = prepared(raw("step", ["7"], source_key=90), raw(source_key=2), raw("step", ["7"], source_key=90))
    assert [a.source_key for a in actions(document)] == [90, 2, 90]
    assert [a.source_occurrence_id for a in actions(document)] == ["oem:1", "oem:2", "oem:3"]
    assert compile_native_protocol(document.to_payload()).to_payload() == document.to_payload()
    assert document.metadata["input_mode"] == "oem_prepared"


def test_typed_nullable_enum_options_lossless_and_detached():
    args = {"m_destination": {"enum_type": "plateName", "value": "REAGENT_PLATE"},
            "m_well": None, "m_piersOption": {"m_piercoption": "d"},
            "m_oldWell": False, "m_material": None}
    document = prepared({"oem_opcode": "mov", "argument_type": "ClassMoveTo", "arguments": args})
    saved = document.to_payload()
    args["m_destination"]["enum_type"] = "locationID"
    assert actions(document)[0].params["arguments"]["m_destination"]["enum_type"] == "plateName"
    assert saved == ProtocolDocument.from_payload(json.loads(json.dumps(saved))).to_payload()


@pytest.mark.parametrize("operation", [raw("bogus"), raw("LOOP", ["2"]), raw("mov"),
    raw("led", ["1", "2"]), raw("wait", [1]), raw("ampmix", {"m_repeat": 1}, argument_type="ClassMix"),
    raw("ampmix", {"native_method": "arbitrary"}, argument_type="ClassAmpMix")])
def test_late_invalid_operation_returns_no_partial_document(operation):
    with pytest.raises(ValueError):
        prepared(raw(), operation)


def test_duplicate_occurrences_refused_even_if_source_step_numbers_may_repeat():
    with pytest.raises(ValueError, match="Duplicate action_id|unique"):
        prepared(raw(source_occurrence_id="same"), raw(source_occurrence_id="same"))


@pytest.mark.parametrize("mode", ["DBC", "MotionOnly", "trade_show", "dry_run"])
def test_excluded_execution_modes_are_not_normalized(mode):
    with pytest.raises(ValueError, match="normal"):
        prepared(raw(), metadata={"execution_mode": mode})


def test_unknown_representation_version_refused_on_authoring_and_hydration():
    with pytest.raises(ValueError, match="version"):
        compile_prepared_oem_protocol({"version": 2, "operations": [raw()]})
    payload = prepared(raw()).to_payload()
    payload["version"] = 2
    with pytest.raises(ValueError, match="version"):
        ProtocolDocument.from_payload(payload)


def test_raw_core_newline_order_leading_key_comments_and_exact_spacing():
    text = "90 step 4\n// source comment\n\n2 wait  1\n9 park\n"
    document = compile_oem_core_script(text)
    assert [a.source_key for a in actions(document)] == ["90", "2", "9"]
    assert actions(document)[1].params["arguments"] == ("", "1")
    assert [a.source_occurrence_id for a in actions(document)] == ["raw:1", "raw:4", "raw:5"]
    assert len(document.metadata["source_map"]) == 6
    assert document.metadata["source_sha256"] == sha256(text.encode()).hexdigest()
    assert document.metadata["input_mode"] == "oem_core_script"


@pytest.mark.parametrize("text", ["park", "notnumeric park", "1 mov x", "1 park\n2 unknown"])
def test_raw_core_bad_or_typed_only_input_is_not_a_prefix(text):
    with pytest.raises(ValueError):
        compile_oem_core_script(text)


def test_selected_binding_and_lifecycle_closure_checked_before_any_handler():
    calls = []
    leaf = lambda *_: calls.append("called")
    document = prepared(raw(), raw("sp", ["opaque", "opaque", "opaque"]))
    with pytest.raises(ValueError, match="oem:2:sp.*lifecycle:epilogue_lid"):
        validate_protocol_support(document, handlers={}, oem_handlers={"park": leaf}, required_lifecycle=["epilogue_lid"])
    assert calls == []
    # This only qualifies closure against explicit offline leaves, not thermal support.
    assert validate_protocol_support(document, handlers={}, oem_handlers={"park": leaf, "sp": leaf},
        required_lifecycle=["epilogue_lid"], lifecycle_handlers={"epilogue_lid": leaf}) is document
    assert calls == []


def test_executor_owned_source_markers_and_wait_need_no_external_handler():
    document = prepared(raw("step", ["7"]), raw("delaypoint"), raw("wait", ["0"]))
    assert validate_protocol_support(document, handlers={}, oem_handlers={}) is document


def test_disguised_macro_cannot_enter_as_generic_pipette_action():
    document = compile_native_protocol({"stages": [{"actions": [{"kind": "pipette_mix", "params": {"macro_verb": "SA"}}]}]})
    with pytest.raises(ValueError, match="Unexpanded"):
        validate_protocol_support(document, handlers={ProtocolActionKind.PIPETTE_MIX: lambda *_: {"ok": True}}, oem_handlers={})


def test_xml_numeric_order_comments_empty_markers_and_repeated_steps(tmp_path):
    path = tmp_path / "source.xml"
    data = b'<root><!--header--><script><line10 cmd="WAIT 2"/><line2 step="1"/><!--keep--><line3 step="1"/><line4 cmd="WAIT 1"/><line5/><line6 step="1"/></script></root>'
    path.write_bytes(data)
    imported = import_oem_xml_protocol(path)
    document = imported.document
    assert [a.source_key for a in actions(document)] == [4, 10]
    assert len(document.stages) == 3
    assert len({s.stage_id for s in document.stages}) == 3
    assert document.stages[0].actions == ()
    assert any(row.get("comment") == "keep" for row in document.metadata["source_map"])
    assert any(row.get("source_key") == 5 for row in document.metadata["source_map"])
    assert document.metadata["source_sha256"] == sha256(data).hexdigest()
    assert ProtocolDocument.from_payload(document.to_payload()).to_payload() == document.to_payload()
    with pytest.raises(ValueError, match="generator"):
        validate_protocol_support(document, handlers={kind: lambda *_: {"ok": True} for kind in ProtocolActionKind}, oem_handlers={})


@pytest.mark.parametrize("nodes", ['<line01 cmd="WAIT 1"/><line1 cmd="WAIT 2"/>', '<lineX cmd="WAIT 1"/>'])
def test_xml_bad_numeric_or_duplicate_keys_reject_before_preview(tmp_path, nodes):
    path = tmp_path / "source.xml"
    path.write_text(f"<root><script>{nodes}</script></root>")
    with pytest.raises(ValueError, match="source key"):
        import_oem_xml_protocol(path)


def test_xml_unknown_late_node_is_retained_but_whole_preview_nonexecutable(tmp_path):
    path = tmp_path / "source.xml"
    path.write_text('<root><script><line1 cmd="WAIT 1"/><line2 cmd="UNKNOWN x"/></script></root>')
    imported = import_oem_xml_protocol(path)
    assert imported.coverage.unsupported_command_count == 1
    assert imported.document.metadata["coverage"]["unsupported_commands"][0]["raw_cmd"] == "UNKNOWN x"
    with pytest.raises(ValueError, match="generator"):
        validate_protocol_support(imported.document, handlers={ProtocolActionKind.WAIT: lambda *_: {"ok": True}}, oem_handlers={})


def test_changed_file_bytes_do_not_mutate_captured_plan_and_have_new_identity(tmp_path):
    path = tmp_path / "source.xml"
    path.write_text('<root><script><line1 cmd="WAIT 1"/></script></root>')
    before = import_oem_xml_protocol(path).document
    saved = before.to_payload()
    path.write_text('<root><script><line1 cmd="WAIT 2"/></script></root>')
    after = import_oem_xml_protocol(path).document
    assert before.to_payload() == saved
    assert before.metadata["source_sha256"] != after.metadata["source_sha256"]


def test_prepared_digest_changes_for_order_options_and_source_binding():
    first = prepared(raw(source_key=1), raw(source_key=2), metadata={"settings_identity": "one"})
    second = prepared(raw(source_key=2), raw(source_key=1), metadata={"settings_identity": "one"})
    third = prepared(raw(source_key=1), raw(source_key=2), metadata={"settings_identity": "two"})
    assert len({p.metadata["source_sha256"] for p in [first, second, third]}) == 3
