"""Typed manual source calls, inline inside the ordinary finite owner.

No run_job/ldtip lifecycle or alternate motion logic: newloadTips is deliberately
called directly, so its already-matching-tip early return remains observable.
"""
from typing import Any, Mapping
from types import SimpleNamespace

from ..services.pipette_service import _OemPipetteBody, build_oem_pipette_handlers


def run_manual_source_inline(provider: Any, request: Mapping[str, Any], *,
        command_id: str, owner_identity: Mapping[str, Any], state: Any,
        settings: Mapping[str, Any]) -> dict[str, Any]:
    from ..manual_pipetting import SOURCE_MANUAL_MODELS, SourceLoadTips, SourceMix, SourcePurge
    step = SOURCE_MANUAL_MODELS[request["operation"]].model_validate(dict(request))
    if isinstance(step, SourceLoadTips):
        from ..oem_machine_bundle import get_active_oem_machine_snapshot
        snapshot = get_active_oem_machine_snapshot()
        settings = {**settings,
            "StartMode": {"DevMode": 0, "WebMode": 1, "LocalMode": 2, "TradeShowMode": 3}[
                snapshot.operation_parameters["Mode"]],
            "CameraInstalled": snapshot.fields["machine.camera_installed"].value,
            "CameraCalibrated": snapshot.camera_calibrated,
            "OverPressChecked": False}  # source two-state checkbox HasValue
        # Rehydrate selection from the existing inventory owner, including
        # removals by a preceding manual request. Never restore/construct it.
        reader = getattr(provider, "_tip_tray_state_reader", None)
        if callable(reader):
            for index, tray in enumerate(state.source_model.tip_trays):
                retained = reader(index)
                if retained.get("occupancy") is not None:
                    metadata = provider._read_constructed_tip_tray(index)
                    if metadata is not None:
                        tray.tip_type = {"T50": 50, "T200": 200, "T201": 201}[metadata["tip_type"]]
                        tray.location = metadata["location"]
                    for well, occupied in zip(tray.wells, retained["occupancy"]):
                        well.empty = not occupied
    base = owner_identity["source_identity"]
    serial = 0

    def identity(name):
        nonlocal serial
        serial += 1
        return {**owner_identity, "source_identity": f"{base}:manual-source:{serial}:{name}"}

    def fence(label, runtime_state):
        provider._wp8_execution_fence_checker(command_id, boundary=label)

    def execute(plan, action, runtime_state):
        nested = identity(plan["operation"])
        fence(nested["source_identity"], runtime_state)
        return provider._wp8_execute_nested_plan(plan=plan, command_id=command_id,
                                                 owner_identity=nested)

    def receipt(name, call, action, runtime_state, step_id):
        nested = identity(step_id)
        fence(nested["source_identity"], runtime_state)
        result = provider._manual_pipette_receipt_runner(name, call, command_id, nested,
                                                       step.model_dump())
        if name == "eject_all_tips" and result.get("ok") is True:
            # Collection.ejectAllTips:1234 resets MachineStatus.TipLocation;
            # transport owns the corresponding cached reset. Publish it before
            # subsequent pickup, including when that later pickup throws.
            from ..oem_deck_movement import compile_finite_plate_operation
            publication = execute(compile_finite_plate_operation("pipette_tip_state",
                source_leaf_available=True, changes={"tip_location":
                    provider.primitives.pipette_transport.get_status()["tip_location"]}), action, runtime_state)
            result = {**result, "tip_location_publication": publication}
        return result

    def no_child(*args):
        raise AssertionError("these synchronous source procedures do not start background children")

    callbacks = provider.build_oem_pipette_source_callbacks(execute_plan=execute,
        start_child=no_child, stopped=lambda: False, settings=settings)
    from dataclasses import replace
    from ..oem_compat.position_table import load_bound_oem_position_table
    from ..oem_compat.pathing import LOCATION_ID_TO_NAME

    def facts(action, runtime):
        machine = provider.mov_execution_machine_state()
        status = provider.primitives.pipette_transport.get_status()
        location = machine["current_location"]
        name = LOCATION_ID_TO_NAME[location]
        return {"current_location": location, "current_well": machine["current_well"],
            "current_tray": provider._deck_semantic_state_reader().get("current_tray"),
            "tip_type": status["tip_type"], "tip_location": machine["tip_location"],
            "fluid_level": status["channels"][0]["liquid_level_ul"],
            "speed": status["channels"][0]["top_speed"], "current_location_name": name,
            "z_high": load_bound_oem_position_table().resolve(location_id=name).z_high}

    callbacks["source_bindings"] = replace(callbacks["source_bindings"], facts=facts,
        tip_exists=lambda *args: any(c["tip_loaded"] for c in
            provider.primitives.pipette_transport.get_status()["channels"]))
    # This is an invocation frame, not a fabricated prepared job/document.
    action = SimpleNamespace(source_occurrence_id=base, source_key=step.operation,
                             params={"arguments": ()})
    body = _OemPipetteBody(action, state, callbacks["source_bindings"], receipt, fence, settings)
    callbacks["lift_for_air"] = lambda action, runtime: body.lift_air()
    try:
        if isinstance(step, SourceLoadTips):
            before = body.facts()
            returned = body.newload(step.tip_type, step.force_new_tip, step.pipette)
            after = body.facts()
            return {**body.result(), "source_return": returned,
                "requested_pipette": step.pipette, "tip_location": after["tip_location"],
                "already_matching_tip_type": before["tip_type"] == step.tip_type and not step.force_new_tip,
                "alignment_published": any(row["operation"] == "loadTip" for row in body.steps)
                    and after["tip_location"] == step.pipette}
        if isinstance(step, SourcePurge):
            body.purge(step.speed, amp=step.amp, ntd=step.ntd)
            return {**body.result(), "source_return": None}
        if isinstance(step, SourceMix):
            opcode = "mmix"
            action.params["arguments"] = {
                "m_aspirateOptions": {"m_volume": step.volume_ul, "m_air": step.air_ul,
                    "m_speed": step.aspirate_speed, "m_delay": step.aspirate_delay_ms},
                "m_dispenseAllOptions": {"m_speed": step.dispense_speed,
                    "m_delay": step.dispense_delay_ms},
                "m_repeat": step.cycles, "m_mixType": step.mix_type,
                "m_tipDip": " *" if step.tip_dip else ""}
        else:
            opcode = "aa" if step.operation == "source_aspirate_air" else "da"
            action.params["arguments"] = (str(step.volume_ul),)
        handlers = build_oem_pipette_handlers(before_native_entry=fence,
                                              pipette_call=receipt, **callbacks)
        return handlers[opcode](action, state)
    except Exception as exc:
        if not hasattr(exc, "oem_partial_results"):
            setattr(exc, "oem_partial_results", list(body.steps))
        raise
