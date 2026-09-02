from __future__ import annotations

from dataclasses import FrozenInstanceError

import pytest

from bioxp.oem_deck_movement import (
    ClassMoveToIntent,
    DeckExecutionFailure,
    OEM_PLATE_NAME_ORDINALS,
    compile_mov_execution,
    execute_mov_execution,
)


def state(**changes):
    value = {
        "save_tip": False,
        "old_well": False,
        "old_location": 18,
        "old_well_text": "A1",
        "plate_locations": {0: 23, 1: 21, 2: 3, 7: 11, 8: 12, 9: 13, 10: 14, 11: 16, 21: 6},
        "plate_pierced": {},
        "well_pierced": {},
        "strip_pierced": {},
        "trough_version": 0,
        "base_y": 900,
        "z_low": 19000,
        "position_table_by_location": {
            "16": {"base_coordinates": {"y": 1000}, "z_low": 20000},
        },
        "authority_digest": "a" * 64,
        "board_epoch_4": 1,
        "board_epoch_5": 2,
    }
    value.update(changes)
    return value


def test_class_move_to_intent_is_immutable_and_has_only_caller_inputs():
    intent = ClassMoveToIntent(script_line=7, plate_name=2, well="H12", material="m", continuation="r")
    with pytest.raises(FrozenInstanceError):
        intent.well = 1
    with pytest.raises(TypeError):
        ClassMoveToIntent(script_line=1, plate_name=2, x=1)  # type: ignore[call-arg]


def test_class_move_to_intent_requires_exactly_one_typed_destination():
    with pytest.raises(ValueError, match="exactly one destination"):
        ClassMoveToIntent(script_line=1)
    with pytest.raises(ValueError, match="exactly one destination"):
        ClassMoveToIntent(script_line=1, plate_name=2, location_id=6)
    with pytest.raises(ValueError, match="unexpected plate"):
        ClassMoveToIntent(script_line=1, location_id=7)


def test_wp7_plate_ordinals_bind_elution_and_waste_to_oem_enum_values():
    assert OEM_PLATE_NAME_ORDINALS["ELUTION_PLATE"] == 16
    assert OEM_PLATE_NAME_ORDINALS["WASTE_BIN"] == 21
    assert {OEM_PLATE_NAME_ORDINALS[k] for k in ("POOL_PLATE", "OUTPUT_PLATE", "REAGENT_PLATE", "STRIP_ONE", "STRIP_TWO", "STRIP_THREE", "STRIP_FOUR", "TROUGH")} == {0, 1, 2, 7, 8, 9, 10, 11}


def test_well_precedence_and_exact_material_error():
    calls = []
    get_next = lambda plate, material, offset: calls.append((plate, material, offset)) or 9
    explicit = compile_mov_execution(ClassMoveToIntent(1, plate_name=2, well="B2", material="x"), state(), get_next_well=get_next)
    assert explicit.well_id == 13 and explicit.well_source == "explicit" and calls == []
    material = compile_mov_execution(ClassMoveToIntent(1, plate_name=2, material="x"), state(), get_next_well=get_next)
    assert material.well_id == 9 and material.well_source == "material" and calls == [(2, "x", 0.0)]
    assert compile_mov_execution(ClassMoveToIntent(1, plate_name=2), state()).well_id == 0
    with pytest.raises(ValueError, match="^Unknow well$"):
        compile_mov_execution(ClassMoveToIntent(1, plate_name=2, material="x"), state(), get_next_well=lambda *_: 96)


def test_station_and_direct_destination_resolution():
    for plate, expected in ((0, 2), (1, 1), (21, 6)):
        assert compile_mov_execution(ClassMoveToIntent(1, plate_name=plate), state()).destination == expected
    direct6 = compile_mov_execution(ClassMoveToIntent(1, location_id=6), state())
    direct16 = compile_mov_execution(ClassMoveToIntent(1, location_id=16), state())
    assert (direct6.destination, direct6.plate_name) == (6, 21)
    assert (direct16.destination, direct16.plate_name) == (16, 11)
    assert [s.operation for s in direct6.steps[:3]] == ["scriptmoveTo", "updateLocation", "updatePlateLocation"]


def test_old_well_is_terminal_and_preserves_numeric_enum_parse_hazard():
    plan = compile_mov_execution(ClassMoveToIntent(44, plate_name=0, continuation="r"), state(old_well=True))
    assert plan.source_branch == "old_well_terminal"
    assert [step.operation for step in plan.steps] == ["scriptmoveTo", "updateLocation"]
    assert plan.steps[0].arguments == {
        "destination": 18, "column": 0, "row": 0,
        "positionflag": 1, "runInParallel": True,
    }
    assert plan.steps[1].arguments == {"location_id": 18, "well_id": 1}
    assert plan.well_id == 1
    assert plan.source_hazards == ("confirmed_oem_old_well_numeric_enum_parse",)


def test_saved_tip_elution_forces_old_well_terminal_branch():
    plan = compile_mov_execution(ClassMoveToIntent(2, plate_name=21, continuation="t"), state(save_tip=True))
    assert plan.source_branch == "old_well_terminal"
    assert plan.machine_state_updates == {"save_tip": False, "old_well": True}


def test_normal_order_and_continuation_predicates_are_finite():
    for code, operation in (("r", "rPunchFoil"), ("w", "troughOscillation"), ("h", "hokeypokey"), ("d", "scriptmoveTo"), ("t", "CirclePunch")):
        plate = {"r": 2, "w": 11, "h": 0, "d": 1, "t": 0}[code]
        plan = compile_mov_execution(ClassMoveToIntent(1, plate_name=plate, continuation=code), state())
        assert [s.operation for s in plan.steps[:3]] == ["scriptmoveTo", "updateLocation", "updatePlateLocation"]
        assert plan.steps[-1].operation == operation
    r_plan = compile_mov_execution(ClassMoveToIntent(1, plate_name=2, continuation="r"), state())
    assert r_plan.steps[-1].arguments == {"plate_name": 2, "location_id": 3}
    h_plan = compile_mov_execution(ClassMoveToIntent(1, plate_name=0, continuation="h"), state())
    assert h_plan.steps[-1].arguments == {"destination": 2, "column": 0, "row": 0}
    d_plan = compile_mov_execution(ClassMoveToIntent(1, plate_name=1, continuation="d"), state())
    assert d_plan.steps[-1].operation == "scriptmoveTo"
    assert d_plan.steps[-1].arguments == {"destination": 1, "well": 0, "positionflag": -1}
    t_plan = compile_mov_execution(ClassMoveToIntent(1, plate_name=0, continuation="t"), state())
    assert t_plan.steps[-1].arguments == {"destination": 2, "column": 0, "row": 0}
    assert len(compile_mov_execution(ClassMoveToIntent(1, plate_name=0, continuation="?"), state()).steps) == 3
    trough = compile_mov_execution(ClassMoveToIntent(1, plate_name=11, continuation="w"), state())
    assert trough.steps[-1].arguments["location_id"] == 16
    assert trough.steps[-1].arguments["actions"] == (
        ("moveZ", 20000),
        ("moveY", 1600),
        ("moveY", 400),
        ("moveY", 1600),
        ("moveY", 400),
        ("moveY", 1000),
        ("moveZ", 16000),
        ("moveZ", 20000),
        ("moveZ", 16000),
        ("moveZ", 20000),
        ("moveZ", 10000),
        ("MoveZHome", None),
    )


def test_continuation_pierced_predicates_are_tip_and_strip_specific() -> None:
    tip_loaded = state(tip_location=5, well_pierced={"2:0:1": True})
    assert len(compile_mov_execution(ClassMoveToIntent(1, plate_name=2, continuation="r"), tip_loaded).steps) == 3
    same_well_other_tip = state(tip_location=-1, well_pierced={"2:0:1": True})
    assert compile_mov_execution(ClassMoveToIntent(1, plate_name=2, continuation="r"), same_well_other_tip).steps[-1].operation == "rPunchFoil"
    strip = state(strip_pierced={"7:0": True})
    assert len(compile_mov_execution(ClassMoveToIntent(1, plate_name=7, continuation="r"), strip).steps) == 3


class Provider:
    def __init__(self, terminal=True):
        self.calls = []
        self.terminal = terminal

    def _terminal(self, value=17):
        return {"ok": True, "source_return": value, "delivery_attempted": True, "controller_command_acknowledged": True, "controller_completion_verified": self.terminal}

    def scriptmoveTo(self, **kwargs):
        self.calls.append(("scriptmoveTo", kwargs))
        return self._terminal()

    def updateLocation(self, **kwargs):
        self.calls.append(("updateLocation", kwargs))
        return {"ok": True, "delivery_attempted": False, "semantic_update_ready": True}

    def updatePlateLocation(self, **kwargs):
        self.calls.append(("updatePlateLocation", kwargs))
        return {"ok": True, "delivery_attempted": False, "semantic_update_ready": True}

    def CirclePunch(self, **kwargs):
        self.calls.append(("CirclePunch", kwargs)); return self._terminal(5)

    def moveY(self, value):
        self.calls.append(("moveY", value)); return self._terminal()

    def moveZ(self, value):
        self.calls.append(("moveZ", value)); return self._terminal()

    def sleep(self, value):
        self.calls.append(("sleep", value)); return self._terminal()

    def MoveZHome(self):
        self.calls.append(("MoveZHome", None)); return self._terminal()


class Store:
    def __init__(self): self.events = []
    def persist_mov_execution_plan(self, command_id, plan): self.events.append(("persist", [s.operation for s in plan.steps]))
    def record_delivery_attempt(self, command_id, **marker):
        self.events.append(("delivery_attempt", marker["work_identity"]))
        return {"attempt_sequence": len(self.events), "dispatch_attempt_id": "dispatch-1"}
    def terminalize_mov_execution_stage(self, command_id, step, **evidence): self.events.append((step.operation, evidence))
    def publish_mov_execution_transition(self, command_id, transition): self.events.append(("publish", transition))


class FencedStore(Store):
    def assert_deck_execution_current(self, command_id, *, boundary=None):
        self.events.append(("fence", boundary))


class PreTxFailureProvider(Provider):
    def scriptmoveTo(self, **_kwargs):
        raise DeckExecutionFailure("pre-TX validation rejected", delivery_attempted=False)


def test_execution_persists_all_children_before_io_and_publishes_in_source_order():
    provider, store = Provider(), Store()
    plan = compile_mov_execution(ClassMoveToIntent(8, plate_name=0, continuation="t"), state())
    result = execute_mov_execution("c", plan, provider=provider, command_store=store)
    assert store.events[0] == ("persist", ["scriptmoveTo", "updateLocation", "updatePlateLocation", "CirclePunch"])
    assert [name for name, _ in provider.calls] == ["scriptmoveTo", "updateLocation", "updatePlateLocation", "CirclePunch"]
    assert result["source_return_disposition"] == "ignored"
    assert result["semantic_state_committed"] is True


def test_execution_checks_interrupt_fence_before_first_plan_write():
    provider, store = Provider(), FencedStore()
    plan = compile_mov_execution(ClassMoveToIntent(8, plate_name=0, continuation="t"), state())
    execute_mov_execution("c", plan, provider=provider, command_store=store)
    assert store.events[:2] == [
        ("fence", "before_plan_write"),
        ("persist", ["scriptmoveTo", "updateLocation", "updatePlateLocation", "CirclePunch"]),
    ]


def test_pre_tx_provider_rejection_is_failed_without_ambiguity():
    provider, store = PreTxFailureProvider(), Store()
    plan = compile_mov_execution(ClassMoveToIntent(8, plate_name=0, continuation="t"), state())
    result = execute_mov_execution("c", plan, provider=provider, command_store=store)
    assert result["delivery_attempted"] is False
    assert result["ambiguity_state"] == "failed"
    script_stage = next(event for event in store.events if event[0] == "scriptmoveTo")
    assert script_stage[1]["state"] == "failed"


def test_missing_terminal_proof_blocks_all_semantic_publication():
    provider, store = Provider(terminal=False), Store()
    plan = compile_mov_execution(ClassMoveToIntent(8, plate_name=0, continuation="t"), state())
    result = execute_mov_execution("c", plan, provider=provider, command_store=store)
    assert [name for name, _ in provider.calls] == ["scriptmoveTo"]
    assert not any(event[0] == "publish" for event in store.events)
    assert result["ambiguity_state"] == "ambiguous"


def test_xy_aggregate_is_one_parent_with_exact_siblings_and_join():
    plan = compile_mov_execution(ClassMoveToIntent(1, plate_name=0), state(), movement_children=("moveX", "moveY"))
    step = plan.steps[0]
    assert step.operation == "scriptmoveTo"
    assert [child.operation for child in step.source_children] == ["moveX", "moveY"]
    assert step.join == "Task.WaitAll"


def test_w_continuation_executes_exact_oscillation_and_home_order():
    provider, store = Provider(), Store()
    plan = compile_mov_execution(ClassMoveToIntent(1, plate_name=11, continuation="w"), state())
    result = execute_mov_execution("c", plan, provider=provider, command_store=store)
    assert result["ok"] is True
    assert provider.calls[3:] == [
        ("moveZ", 20000),
        ("moveY", 1600),
        ("moveY", 400),
        ("moveY", 1600),
        ("moveY", 400),
        ("moveY", 1000),
        ("moveZ", 16000),
        ("moveZ", 20000),
        ("moveZ", 16000),
        ("moveZ", 20000),
        ("moveZ", 10000),
        ("MoveZHome", None),
    ]


def test_trough_oscillation_rechecks_interrupt_before_each_nested_child():
    class InterruptingStore(FencedStore):
        def assert_deck_execution_current(self, command_id, *, boundary=None):
            super().assert_deck_execution_current(command_id, boundary=boundary)
            if boundary == "before_nested_child:3:1:moveY":
                raise RuntimeError("deck_execution_interrupted")

    provider, store = Provider(), InterruptingStore()
    plan = compile_mov_execution(
        ClassMoveToIntent(1, plate_name=11, continuation="w"), state()
    )
    with pytest.raises(RuntimeError, match="deck_execution_interrupted"):
        execute_mov_execution("c", plan, provider=provider, command_store=store)
    assert provider.calls[3:] == [("moveZ", 20000)]


def test_d_continuation_reuses_scriptmoveto_with_minus_one():
    provider, store = Provider(), Store()
    plan = compile_mov_execution(ClassMoveToIntent(1, plate_name=1, well="B2", continuation="d"), state())
    result = execute_mov_execution("c", plan, provider=provider, command_store=store)
    assert result["ok"] is True
    assert provider.calls[-1] == ("scriptmoveTo", {"destination": 1, "well": 13, "positionflag": -1})
