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
    aspirate, dispense = build_oem_prefill_subprocedures(
        state=state, source_occurrence_id=base,
        before_native_entry=lambda label, runtime_state: fence(label),
        pipette_call=lambda name, call, action, runtime_state, step_id: receipt(name, call),
        source_bindings=source["source_bindings"], settings=settings)

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
        raw = receipt("query_tip_status_all", lambda t: t.query_tip_status_all())
        channels = raw.get("channels")
        if channels is None and isinstance(raw.get("result"), Mapping):
            channels = raw["result"].get("channels")
        return {"ok": raw.get("ok", True), "tip_exists": any(
            isinstance(channel, Mapping) and channel.get("tip_loaded") is True
            for channel in (channels or ())), "controller_evidence": raw}

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
