"""Connected logical model and source sweep with explicit native doubles."""
from types import SimpleNamespace
import pytest

from bioxp.protocols.runtime_state import ProtocolSourceModel, SourceTray, SourceWell
from bioxp.services.pipette_service import build_oem_pipette_handlers
from tests.test_protocol_oem_pipette import Native, action, bindings


def sweep_bindings(tips=False):
    args, state, native, entries, effects = bindings(Native(tips=tips))
    state.source_model.sweep_locations = lambda tray, clearall: ["B2"] if tray == 1 else None
    state.source_model.get_tip_tray_location = lambda tray: 17 + tray
    def effect(name, *values):
        effects.append((name, *values))
        return {"ok": True}
    args.update(tip_load_move=lambda loc, col, row, a, s: effect("load_move", loc, col, row),
                move_z_home=lambda a, s: effect("home"),
                set_z_current_max=lambda a, s: effect("current"),
                remove_tip=lambda tray, well, a, s: effect("remove", tray, well),
                script_move_to_waste=lambda a, s: effect("script_waste"))
    return args, state, native, entries, effects


def test_sweep_uses_distinct_source_load_sequence_and_numeric_suffix():
    args, state, native, entries, effects = sweep_bindings()
    result = build_oem_pipette_handlers(**args)["sweep"](action("sweep"), state)
    assert effects == [("load_move", 18, 1, 1), ("location", 18, 2), ("home",),
                       ("current",), ("remove", 1, "B2"), ("script_waste",),
                       ("z", 80000), ("x", 79000)]
    assert native.calls == [("tips",), ("eject", {"check_missing_tip": True, "wait": True}), ("tips",)]
    assert result["ok"] is True
    assert "source_pause_scripts" not in result
    assert state.source_model.allow_to_stop is True
    assert state.source_model.logical_tip_present is True
    assert len(entries) == len(set(entries)) == 11


def test_sweep_tip_exist_signals_source_error_at_boundary():
    args, state, _, _, _ = sweep_bindings(tips=True)
    events = []
    args["source_error_event"] = lambda message, a, s: events.append(message)
    result = build_oem_pipette_handlers(**args)["sweep"](action("sweep"), state)
    assert events == ["Eject tip failed"]
    assert result["source_pause_scripts"] is True
    assert result["source_error_event"] == "Eject tip failed"


def test_sweep_empty_source_selection_does_not_issue_generic_cleanup():
    args, state, native, entries, effects = sweep_bindings()
    state.source_model.sweep_locations = lambda *_: None
    result = build_oem_pipette_handlers(**args)["sweep"](action("sweep"), state)
    assert result["ok"] is True
    assert native.calls == entries == effects == []


def test_sweep_entry_fence_retains_completed_leaves_and_source_stop_flag():
    args, state, native, entries, effects = sweep_bindings()
    guard = args["before_native_entry"]
    def fence(identity, state):
        if identity.endswith("loadTip.MoveZHome"):
            raise RuntimeError("parent stopped")
        guard(identity, state)
    args["before_native_entry"] = fence
    with pytest.raises(RuntimeError, match="parent stopped") as caught:
        build_oem_pipette_handlers(**args)["sweep"](action("sweep"), state)
    assert len(caught.value.oem_partial_results) == 2
    assert native.calls == []
    assert state.source_model.allow_to_stop is False


@pytest.mark.parametrize("key", [37, "prepared-key", None])
def test_la_connected_source_model_preserves_original_key_and_group_mutation(key):
    model = ProtocolSourceModel(trays={"REAGENT_PLATE": SourceTray("r", 3, [SourceWell(None, 10, 100) for _ in range(96)])})
    state = SimpleNamespace(source_model=model)
    handler = build_oem_pipette_handlers(before_native_entry=lambda *a: None)["la"]
    result = handler(action("la", ("REAGENT_PLATE", "A1", "-2.5"), key), state)
    assert result["source_return"] == [key]
    assert [model.trays["REAGENT_PLATE"].wells[i].volume for i in (0, 24, 48, 72)] == [12.5] * 4
    hydrated = ProtocolSourceModel.from_payload(model.to_payload())
    assert hydrated.to_payload() == model.to_payload()


def test_retip_connected_source_model_excludes_hotel_and_nonreuse():
    model = ProtocolSourceModel(tip_trays=[
        SourceTray("0", 16, [SourceWell("Reuse", 0, 0, True), SourceWell("rm", 0, 0, True), SourceWell("Reuse", 0, 0, False)]),
        SourceTray("4", 15, [SourceWell("Reuse", 0, 0, True)]),
    ])
    published = []
    def publish(tray, wells, *rest):
        published.append((tray, wells))
        return {"ok": True}
    handler = build_oem_pipette_handlers(before_native_entry=lambda *a: None, publish_tip_transition=publish)["retip"]
    result = handler(action("retip"), SimpleNamespace(source_model=model))
    assert published == [("0", [0])]
    assert result["ok"] is True
    assert model.tip_trays[0].wells[0].empty is False
    assert model.tip_trays[0].wells[1].empty is True
    assert model.tip_trays[1].wells[0].empty is True
