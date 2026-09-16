"""Prepared logical model: exact MachineStatus/Well/TipTray source semantics.

Native positions, readiness, TipExist and canonical occupancy are intentionally
absent; composite tests supply native recorder leaves through the real service.
"""
import json
import struct
from concurrent.futures import Future
from dataclasses import replace

import pytest

from bioxp.protocols.runtime_state import ProtocolSourceModel, ProtocolRuntimeState, SourceTray, SourceWell
from bioxp.services.pipette_service import build_oem_pipette_handlers
from tests.test_protocol_oem_pipette import action
from tests.test_protocol_oem_pipette_composites import composite_bindings


def tray(name="pool", location=0, volume=60, tip_type=None, empty=False):
    return SourceTray(name, location, [SourceWell(None, volume, 300, empty) for _ in range(96)], tip_type=tip_type)


def model():
    pool, reagent = tray(), tray("reagent", 3, 25)
    pool.wells[0].content = "pool-fluid"
    reagent.wells[0].content = "reagent-fluid"
    return ProtocolSourceModel(trays={"POOL_PLATE": pool, "REAGENT_PLATE": reagent},
        tip_trays=[tray(str(i), loc, 0, 50) for i, loc in enumerate((7, 8, 9, 10, 15))])


def test_retained_current_tray_wins_over_different_current_location():
    m = model()
    assert m.current_well_volume(3, 0, current_tray=0) == 60
    m.update_fluid_name(3, 0, current_tray=0)
    assert m.fluid_name == "pool-fluid"
    m.update_fluid_level(3, 0, -1, -10, current_tray=0)
    assert [m.trays["POOL_PLATE"].wells[i].volume for i in (0,24,48,72)] == [50]*4
    assert m.trays["REAGENT_PLATE"].wells[0].volume == 25


def test_missing_current_tray_only_level_falls_back_to_location():
    m = model()
    assert m.current_well_volume(3, 0, current_tray=21) == 0
    m.update_fluid_name(3, 0, current_tray=21)
    assert m.fluid_name is None
    m.update_fluid_level(3, 0, 2, -5, current_tray=21)
    assert m.trays["REAGENT_PLATE"].wells[0].volume == 20
    assert m.trays["REAGENT_PLATE"].wells[24].volume == 25


def test_location_16_is_source_level_noop_even_with_retained_tray():
    m = model()
    before = m.to_payload()
    m.update_fluid_level(16, 0, -1, -10, current_tray=0)
    assert m.to_payload() == before


def test_group_fluid_mutates_independently_without_rollback_or_clamp():
    m = model()
    wells = m.trays["POOL_PLATE"].wells
    wells[0].volume, wells[24].volume = 2, 299
    m.fluid_name = "incoming"
    m.update_fluid_level(0, 0, -1, 4, current_tray=0)
    assert [wells[i].volume for i in (0,24,48,72)] == [6,299,64,64]
    assert wells[0].content == "pool-fluid" and wells[48].content == "incoming"
    m.update_fluid_level(0, 0, -1, -10, current_tray=0)
    assert [wells[i].volume for i in (0,24,48,72)] == [6,289,54,54]


def test_fluid_delta_is_source_single_not_double():
    m = model()
    m.update_fluid_level(0, 0, 0, 0.1, current_tray=0)
    assert m.trays["POOL_PLATE"].wells[0].volume == 60 + struct.unpack("f", struct.pack("f", 0.1))[0]


def test_strip_color_selection_row_read_and_negative_group_level():
    m = ProtocolSourceModel(strips=[SourceTray(str(i), 11+i,
        [SourceWell("strip", 2+i, 100) for _ in range(8)], strip_color="X") for i in range(4)])
    assert m.current_well_volume(12, "B1", current_tray=8) == 3
    m.update_fluid_name(12, "B1", current_tray=8)
    assert m.fluid_name is None
    m.update_fluid_level(12, "B1", 2, -5, current_tray=8)
    assert [w.volume for w in m.strips[1].wells] == [3,-2,3,-2,3,-2,3,-2]
    assert m.select_strip("strip", 3.001) == (12,0)
    assert m.select_strip("missing", 1) == (11,-1)


def test_group_tip_selection_source_order_zones_and_hotel_exclusion():
    m = model(); m.tip_zone_index = 7
    for t in m.tip_trays:
        for w in t.wells: w.empty = True
    t = m.tip_trays[0]
    # A1 is full but wrong zones; B1 matches before A2 in source order.
    for start in (0,12,1):
        for n, off in enumerate((0,24,48,72)):
            t.wells[start+off].empty = False
            t.wells[start+off].zone_index = (99 if start == 0 else 7+n*2)
    assert m.select_tip(50) == (0,7,"B1")
    m.tip_removed(0,"B1")
    assert m.select_tip(50) == (0,7,"A2")
    assert m.select_tip(200) is None


def test_single_tip_returns_last_present_candidate_on_unmatched_zone():
    m = model(); m.tip_zone_index = 7
    for t in m.tip_trays:
        for w in t.wells: w.empty = True
    for i in (48,61):
        m.tip_trays[0].wells[i].empty = False
        m.tip_trays[0].wells[i].zone_index = 99
    assert m.select_tip(50,2) == (0,7,"B2")
    m.tip_removed(0,"B2",2)
    assert m.tip_trays[0].wells[61].empty
    assert not m.tip_trays[0].wells[48].empty


def test_restore_preserves_group_vs_single_and_model_zone_not_argument():
    m = model(); m.tip_zone_index = 9
    m.tip_removed(0,"A1")
    m.tip_restored(0,"A1","Reuse",100)
    assert [(m.tip_trays[0].wells[i].content,m.tip_trays[0].wells[i].zone_index,m.tip_trays[0].wells[i].empty)
        for i in (0,24,48,72)] == [("Reuse",9,True),("Reuse",11,True),("Reuse",13,True),("Reuse",15,True)]
    m.tip_restored(0,48,"rm",100)
    assert m.tip_trays[0].wells[48].zone_index == 9
    assert m.tip_trays[0].wells[24].content == "Reuse"
    m.tip_zone_index = None
    m.tip_restored(0,48,"Reuse",100)
    assert m.tip_trays[0].wells[48].zone_index == 9
    ids = dict(m.retip_wells())["0"]
    m.retip_committed("0",ids)
    assert all(not m.tip_trays[0].wells[i].empty for i in (0,24,48,72))


def test_hotel_source_selection_reset_and_all_96_load():
    m = model(); t = m.tip_trays[4]
    for w in t.wells: w.empty = True; w.content = "Reuse"; w.zone_index = 8
    assert m.tip_hotel_empty()
    assert m.next_hotel_tip() == "H1" and t.tip_type == 201 and t.tray_empty
    m.hotel_loaded()
    assert len(t.wells) == 96 and all(not w.empty and w.content is None and w.zone_index == 8 for w in t.wells)
    assert not t.tray_empty
    for name in ("H1","F1","D1","B1"):
        assert m.next_hotel_tip() == name
        m.tip_removed(4,name,hotel=True)
    assert m.tip_hotel_empty()
    assert m.next_hotel_tip() == "H1"
    assert all(w.empty for w in t.wells)


def test_sweep_labels_not_availability_and_column_before_base_row():
    m = model(); t = m.tip_trays[0]
    for start in (0,12,1):
        for off in (0,24,48,72): t.wells[start+off].content = "rm"; t.wells[start+off].empty = True
    t.wells[13].content = "rm"
    assert m.sweep_locations(0) == ["A1","B1","A2"]
    assert m.sweep_locations(0,True) == ["A1","B1","A2","B2"]


def test_wire_roundtrip_copies_logical_model_and_has_no_native_authority():
    m = model(); m.fluid_name="fluid"; m.tip_zone_index=7; m.old_tip_well="A2"
    m.tip_trays[0].tray_empty=False; m.tip_trays[0].wells[0].zone_index=7
    m.add_pressure_base([1,2,3,4])
    state = ProtocolRuntimeState("p",False,source_model=m)
    payload = json.loads(json.dumps(state.to_payload()))
    restored = ProtocolRuntimeState.from_payload(payload)
    assert restored.to_payload() == payload
    restored.source_model.tip_trays[0].wells[0].content="changed"
    assert m.tip_trays[0].wells[0].content is None
    assert not {"current_location","current_tray","tip_exists","reference_ready","pierced"} & set(payload["source_model"])
    assert m.pressure_history == [[0,0,0,0,1],[0,0,0,0,2],[0,0,0,0,3],[0,0,0,0,4]]


@pytest.mark.parametrize("opcode,options", [
    ("masp", {"m_volume":None,"m_air":0,"m_delay":0}),
    ("dsa", {"m_purge":False,"m_delay":0}),
])
def test_actual_composite_model_binding_uses_retained_current_tray(opcode,options):
    args,state,n,entries,effects,facts = composite_bindings()
    state.source_model = model()
    # Unmapped WASTE_BIN keeps CurrentTray; no location-based prepared fallback.
    facts.update(current_location=6,current_tray=0,fluid_level=10)
    result = build_oem_pipette_handlers(**args)[opcode](action(opcode,options),state)
    assert result["ok"]
    expected = 0 if opcode == "masp" else 70
    assert [state.source_model.trays["POOL_PLATE"].wells[i].volume for i in (0,24,48,72)] == [expected]*4
    assert state.source_model.trays["REAGENT_PLATE"].wells[0].volume == 25
    if opcode == "masp":
        assert ("asp",60) in n.calls and state.source_model.fluid_name == "pool-fluid"
    assert len(entries) == len(set(entries))


@pytest.mark.parametrize("pipette", [-1, 2])
def test_load_and_eject_real_logical_model_after_canonical_publication(pipette):
    args,state,n,entries,effects,facts = composite_bindings()
    state.source_model = model()
    facts.update(tip_type=201,tip_location=pipette)
    handlers = build_oem_pipette_handlers(**args)
    handlers["ldtip"](action("ldtip",["T50","T",str(pipette)]),state)
    ids = [0,24,48,72] if pipette == -1 else [48]
    assert all(state.source_model.tip_trays[0].wells[i].empty for i in ids)
    assert state.source_model.old_tip_well == "A1"
    assert state.source_model.logical_tip_present is True
    state.source_model.tip_zone_index = 7
    facts.update(current_location=7,current_location_name="TECANRACK1")
    def owned(name,call,a,s):
        future=Future()
        try: future.set_result(call())
        except Exception as exc: future.set_exception(exc)
        return future
    args["source_bindings"] = replace(args["source_bindings"],start_child=owned)
    build_oem_pipette_handlers(**args)["ejt"](action("ejt",["Reuse"]),state)
    assert all(state.source_model.tip_trays[0].wells[i].content == "Reuse" for i in ids)
    # Eject labels do not fabricate availability. Retip's canonical publisher
    # must succeed before the prepared logical availability changes.
    assert all(state.source_model.tip_trays[0].wells[i].empty for i in ids)
    build_oem_pipette_handlers(**args)["retip"](action("retip"),state)
    assert all(not state.source_model.tip_trays[0].wells[i].empty for i in ids)


def test_rejected_canonical_retip_keeps_real_prepared_availability():
    args,state,n,entries,effects,facts = composite_bindings()
    state.source_model = model()
    well=state.source_model.tip_trays[0].wells[0]
    well.empty=True; well.content="Reuse"
    args["publish_tip_transition"] = lambda *args: {"ok":False,"failure_code":"owner_changed"}
    result=build_oem_pipette_handlers(**args)["retip"](action("retip"),state)
    assert result["ok"] is False and well.empty


def test_failed_native_aspirate_does_not_mutate_real_fluid_model():
    args,state,n,entries,effects,facts = composite_bindings()
    state.source_model = model()
    before=state.source_model.to_payload()
    def failed(command):
        raise RuntimeError("native transfer failed")
    n.aspirate=failed
    with pytest.raises(RuntimeError,match="native transfer failed") as error:
        build_oem_pipette_handlers(**args)["masp"](action("masp",{"m_volume":10,"m_air":0,"m_delay":0}),state)
    assert error.value.oem_partial_results
    assert state.source_model.to_payload() == before
