"""Inline zOffset binding under an existing WP8/ordinary-protocol command claim.

No child command is enqueued: a future diagnostic/calibration finite caller can
invoke run_z_offset_inline five times with different source_occurrence_id values.
"""
from __future__ import annotations

from typing import Any, Mapping

from .oem_calibration import CALIBRATION_PLATES, CalibrationExecutionError
from .oem_fluid_workflows import FluidScanBindings, z_offset
from ..oem_compat.position_table import well_id_from_label
from ..oem_deck_movement import compile_finite_plate_operation
from ..services.pipette_service import build_oem_prefill_subprocedures
from .oem_fluid_callers import FluidCallerBindings, calwith_fluid


def run_z_offset_inline(provider: Any, *, plate: str, speed: int, transfer_fluid: bool,
                        skip_steps: int, command_id: str, owner_identity: Mapping[str, Any],
                        state: Any, settings: Mapping[str, Any]) -> dict[str, Any]:
    """Run one source scan with unique nested identities and the parent's fence.

    State is the ordinary protocol runtime state (including source_model), not
    a second owner or a fabricated source model. Hardware transport is shared.
    """
    if plate not in CALIBRATION_PLATES:
        raise ValueError("unknown OEM calibration plate")
    base = str(owner_identity["source_identity"])
    serial = 0

    def identity(name: str) -> dict[str, Any]:
        nonlocal serial
        serial += 1
        return {**owner_identity, "source_identity": f"{base}:scan:{serial}:{name}"}

    def fence(label: str) -> None:
        provider._wp8_execution_fence_checker(command_id, boundary=label)

    def finite(name: str, **inputs: Any) -> dict[str, Any]:
        nested = identity(name)
        fence(nested["source_identity"])
        plan = compile_finite_plate_operation(name, source_leaf_available=True, **inputs)
        return provider._wp8_execute_nested_plan(plan=plan, command_id=command_id,
                                                 owner_identity=nested)

    def leaf(name: str, args: Mapping[str, Any]) -> dict[str, Any]:
        nested = identity(name)
        fence(nested["source_identity"])
        context = dict(command_id=command_id, child_order=serial, plan_digest=nested["source_identity"])
        if name == "updatePlateLocation":
            from ..oem_deck_movement import canonical_plate_name, plate_name_for_storage
            from ..oem_compat.pathing import LOCATION_ID_TO_NAME
            # The scan reads movable plate inventory, not the manual deck
            # button's latch observation. Preserve the existing publisher.
            semantic = provider._canonical_deck_semantic_state(require_latch_observation=False)
            movable = dict(semantic.get("movable_plate_locations") or {})
            plate_name = plate_name_for_storage(canonical_plate_name(args["plate"]))
            movable[plate_name] = LOCATION_ID_TO_NAME[args["location"]]
            return provider._wp8_publish_semantic(operation=name, updates={
                "movable_plate_locations": movable}, **context)
        if name == "updateLocation":
            return provider.wp8_update_location(name, args, **context)
        if name == "clearTipLoaded":
            return provider.wp8_clear_tip_loaded(name, args, **context)
        raise ValueError(name)

    def receipt(name: str, call: Any) -> dict[str, Any]:
        nested = identity(name)
        fence(nested["source_identity"])
        return provider._manual_pipette_receipt_runner(name, call, command_id, nested,
                                                        {"plate": plate, "speed": speed})

    def execute_plan(plan: Mapping[str, Any], action: Any, runtime_state: Any) -> dict[str, Any]:
        return provider._wp8_execute_nested_plan(plan=plan, command_id=command_id,
            owner_identity=identity(f"prefill:{action.source_occurrence_id}:{plan['operation']}"))

    source = provider.build_oem_pipette_source_callbacks(
        execute_plan=execute_plan, start_child=lambda *args: None,
        stopped=lambda: False, settings=settings)
    from dataclasses import replace
    from ..oem_compat.position_table import load_bound_oem_position_table
    from ..oem_compat.pathing import LOCATION_ID_TO_NAME

    def source_facts(action: Any, runtime_state: Any) -> dict[str, Any]:
        machine = provider.mov_execution_machine_state()
        status = provider.primitives.pipette_transport.get_status()
        channel = status["channels"][0]
        location = machine["current_location"]
        location_name = LOCATION_ID_TO_NAME[location]
        position = load_bound_oem_position_table().resolve(location_id=location_name)
        return {"current_location": location, "current_well": machine["current_well"],
            "current_tray": provider._deck_semantic_state_reader().get("current_tray"),
            "tip_type": status["tip_type"],
            "tip_location": machine["tip_location"], "fluid_level": channel["liquid_level_ul"],
            "speed": channel["top_speed"], "current_location_name": location_name,
            "z_high": position.z_high}

    source_bindings = replace(source["source_bindings"], facts=source_facts)
    aspirate, dispense = build_oem_prefill_subprocedures(
        state=state, source_occurrence_id=base,
        before_native_entry=lambda label, runtime_state: fence(label),
        pipette_call=lambda name, call, action, runtime_state, step_id: receipt(name, call),
        source_bindings=source_bindings, settings=settings)

    def move(destination: int, well: str | int, flag: int) -> dict[str, Any]:
        value = well_id_from_label(well)
        row, column = divmod(value, 12)
        nested = identity("scriptmoveTo")
        fence(nested["source_identity"])
        plan = compile_finite_plate_operation("pipette_script_move", source_leaf_available=True,
            destination=destination, column=column, row=row, position_flag=flag,
            run_in_parallel=False)
        # zOffset ignores scriptmoveTo's returned Boolean. Dispatch its real
        # compiled child directly; the ordinary nested executor would turn a
        # false receipt into an invented exception and stop the source loop.
        return provider.execute_wp8_child(
            {**plan["children"][0], "_delivery_identity": nested},
            command_id=command_id, child_order=0, plan_digest=plan["plan_digest"])

    def tips() -> dict[str, Any]:
        nested = identity("loadTips")
        fence(nested["source_identity"])
        return provider.wp8_load_tips("sourceLoadTips", {"tip_type": 50, "force_new_tip": True},
                                      command_id=command_id, owner_identity=nested)

    def query() -> dict[str, Any]:
        # zOffset calls the same source queryTipStatus(-1) as loadTips. The
        # generic all-channel query imposes a stricter readback contract that
        # the OEM does not use here; keep its uncertainty in the child receipt.
        raw = receipt("query_all_pipette_tip_states",
                      lambda t: t.query_tip_status_for_oem_load_tips())
        return {"ok": raw["ok"], "tip_exists": raw["source_tip_exists"],
                "controller_evidence": raw}

    bindings = FluidScanBindings(
        facts=provider.mov_execution_machine_state,
        publish_plate=lambda location, plate_id: leaf("updatePlateLocation", {"location": location, "plate": plate_id}),
        load_tips=tips, move=move,
        publish=lambda location, well: leaf("updateLocation", {"destination": location, "well": well}),
        aspirate=aspirate, dispense=dispense,
        detect=lambda requested_speed: finite("measure_fluid_height", speed=requested_speed),
        query_tips=query,
        eject=lambda: receipt("eject_all_tips", lambda t: t.eject_all_tips(
            check_missing_tip=True, wait=True, channels=None)),
        clear_tip_loaded=lambda: leaf("clearTipLoaded", {}),
    )
    try:
        return z_offset(plate, bindings, speed=speed, transfer_fluid=transfer_fluid,
                        skip_steps=skip_steps)
    except CalibrationExecutionError as exc:
        return {**exc.evidence, "error": str(exc)}


def run_calwith_fluid_inline(provider: Any, *, command_id: str,
                             owner_identity: Mapping[str, Any], state: Any,
                             pipette_settings: Mapping[str, Any],
                             calibration_settings: Any) -> dict[str, Any]:
    """ControlLib.calwithFluid under the ordinary finite protocol claim.

    The WPF comparison dialog has no native operator choice here. Keep all
    per-plate saved revisions and the prior revision visible; never infer an
    acceptance decision from successful motion or a missing UI callback.
    """
    events: list[dict[str, Any]] = []
    occurrence = 0

    def identity(name: str) -> dict[str, Any]:
        nonlocal occurrence
        occurrence += 1
        return {**owner_identity, "source_identity":
                f"{owner_identity['source_identity']}:cal:{occurrence}:{name}"}

    def record(name: str, result: Any) -> Any:
        events.append({"operation": name, "result": result})
        return result

    def finite(name: str, **inputs: Any) -> dict[str, Any]:
        nested = identity(name)
        provider._wp8_execution_fence_checker(command_id, boundary=nested["source_identity"])
        machine = provider.wp8_operation_machine_state(name, inputs)
        plan = compile_finite_plate_operation(name, source_leaf_available=True,
                                               **{**machine, **inputs})
        return record(name, provider._wp8_execute_nested_plan(
            plan=plan, command_id=command_id, owner_identity=nested))

    def controller(name: str, call: Any) -> dict[str, Any]:
        nested = identity(name)
        provider._wp8_execution_fence_checker(command_id, boundary=nested["source_identity"])
        raw = record(name, call())
        # setMaxAcc and the sleep in this caller are void OEM calls. Preserve
        # the returned controller evidence, but do not make a false receipt a
        # new source refusal; actual exceptions still unwind as in ControlLib.
        return dict(raw)

    def scan(plate: str, speed: int, transfer: bool, skip: int) -> dict[str, Any]:
        nested = identity(f"zOffset:{plate}")
        provider._wp8_execution_fence_checker(command_id, boundary=nested["source_identity"])
        raw = record(f"zOffset:{plate}", run_z_offset_inline(provider, plate=plate,
            speed=speed, transfer_fluid=transfer, skip_steps=skip,
            command_id=command_id, owner_identity=nested, state=state,
            settings=pipette_settings))
        if raw.get("ok") is not True or type(raw.get("source_return")) is not int:
            raise RuntimeError(f"calwithFluid:zOffset:{plate}:failed:{raw.get('error')}")
        return raw

    def tips_exist() -> bool:
        # ClassPipetteCollection.TipExist reads cached TipLoaded fields; it
        # does not send a fresh status query or require a readback contract.
        raw = provider.primitives.pipette_transport.get_status()
        loaded = any(row["tip_loaded"] for row in raw["channels"])
        record("TipExist", {"tip_exists": loaded, "source": "cached_transport_state"})
        return loaded

    def eject() -> None:
        nested = identity("ejectAllTips")
        provider._wp8_execution_fence_checker(command_id, boundary=nested["source_identity"])
        record("ejectAllTips", provider._manual_pipette_receipt_runner(
            "eject_all_tips", lambda transport: transport.eject_all_tips(
                check_missing_tip=True, wait=True, channels=None), command_id, nested,
            {"source": "ControlLib.calwithFluid:finally"}))
        # OEM discards ejectAllTips's Boolean. Keep its receipt, then execute
        # the rest of the finally unless the call itself raised an exception.

    def reset_status() -> None:
        # ClassMachineStatus.resetStatus:655-689 reloads five tip trays,
        # reconstructs plate/strip wells, and resets movable locations to
        # source defaults. These are logical declarations, not physical proof.
        from ..oem_job_preparation import (construct_new_machine_source_model,
                                           reset_loaded_job_tip_inventory)
        from ..oem_deck_movement import OEM_MOVABLE_OBJECT_DEFAULT_LOCATIONS
        reset_loaded_job_tip_inventory(state, execute_native=lambda name, inputs, _: finite(name, **inputs))
        defaults = construct_new_machine_source_model()
        state.source_model.trays = defaults.trays
        state.source_model.strips = defaults.strips
        state.source_model.fluid_name = None
        nested = identity("resetStatus:locations")
        provider._wp8_execution_fence_checker(command_id, boundary=nested["source_identity"])
        record("resetStatus", provider._wp8_publish_semantic(
            operation="updatePlateLocation", updates={
                "movable_plate_locations": dict(OEM_MOVABLE_OBJECT_DEFAULT_LOCATIONS)},
            command_id=command_id, child_order=occurrence,
            plan_digest=nested["source_identity"]))

    bindings = FluidCallerBindings(
        log_file=lambda: record("setFileName", {"path": r"c:\logfile\fluid-level.txt",
                                                "log_only": True}),
        initiate_group=lambda: None,
        catch_plate=lambda plate: finite("catch_plate", plate=plate, run_in_parallel=False),
        release_plate=lambda destination, press: finite("release_plate",
            destination=destination, press_plate=press, run_in_parallel=False),
        press_plates=lambda plates: finite("press_plates", plates=plates, run_in_parallel=False),
        scan=scan, mark_strip=lambda: record("markStrip", _mark_calibration_strip(state)),
        park=lambda: finite("park_gantry", rehome=False),
        error=lambda name, exc: record("source_error", {"method": name, "message": str(exc),
            "traceback": __import__("traceback").format_exception(exc)}),
        reset_status=reset_status,
        tip_exists=tips_exist,
        move_waste=lambda: finite("pipette_waste", location=6, offset_x=0,
                                  offset_y=0, run_in_parallel=False),
        sleep_ms=lambda milliseconds: controller("Sleep", lambda: provider.wp8_sleep(
            "Sleep", {"milliseconds": milliseconds})),
        eject_tips=eject,
        completed=lambda: record("Completed fluid calibration", {"log_only": True}),
        finish_ui=lambda: record("finish_ui", {"ui_not_bound": True}),
        set_z_acceleration=lambda value: controller("setMaxAcc(z)",
            lambda: provider.primitives.z_set_max_acc(value)),
        # No WPF comparison dialog is connected to a running finite claim.
        compare=lambda before, after: record("comparison_pending", {
            "before_revision_id": before["saved_revision_id"],
            "after_revision_id": after["saved_revision_id"]}) and None,
        restore=calibration_settings.restore,
    )
    controller("setMaxAcc(z):outer", lambda: provider.primitives.z_set_max_acc(176))
    result = calwith_fluid(bindings, calibration_settings,
        calibration_settings.active_snapshot.fluid_reference,
        machine_calibrated=calibration_settings.active_snapshot.machine_calibrated)
    # A source finally exception can skip the worker's normal final readback;
    # retain the exact durable revision and comparison evidence regardless.
    saved = calibration_settings.read()
    result.setdefault("saved_revision_id", saved["saved_revision_id"])
    result.setdefault("pending_restart", saved["pending_restart"])
    result.setdefault("comparison_choice", None)
    return {**result, "ok": result["body_completed"] and "finalization_error" not in result,
            "delivery_attempted": True, "source_events": events,
            "comparison_gap": "OEM resultComparison requires an operator choice; no dialog is bound"
                              if result.get("comparison_choice") is None else None,
            "physical_effect_verified": False}


def _mark_calibration_strip(state: Any) -> dict[str, Any]:
    # m_strip[1] is the second strip, not the first tray or a physical read.
    state.source_model.strips[1].strip_color = "X"
    return {"strip_index": 1, "strip_color": "X", "logical_only": True}
