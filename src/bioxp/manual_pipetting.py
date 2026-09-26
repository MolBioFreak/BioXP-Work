"""Narrow manual pipetting authoring over the ordinary native protocol owner.

Move uses source scriptmoveTo geometry: the well is aligned using the REAL
machine TipLocation (-1 means the four-tip reference, otherwise channel 0..3).
Liquid channels select plungers only; they never rewrite loaded-tip custody.
Lower/lift and liquid strokes operate IN PLACE. Only an authored move changes
XY. No implicit preparation, tip loading, fluid detection, sweep, lid or Park.
"""
from __future__ import annotations

from typing import Annotated, Any, Callable, Literal, Mapping, Union

from pydantic import BaseModel, ConfigDict, Field, field_validator

from .oem_compat.pathing import LOCATION_ID_TO_NAME
from .oem_compat.position_table import well_id_from_label
from .oem_deck_movement import compile_finite_plate_operation
from .pipette.models import PipetteAspirateCommand, PipetteDispenseCommand, PipetteValidationError
from .protocols.models import ProtocolAction, ProtocolActionKind, ProtocolDocument, ProtocolStage
from .protocols.validators import validate_protocol_document


class _Request(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)


class _Location(_Request):
    location_id: int

    @field_validator("location_id")
    @classmethod
    def location_known(cls, value: int) -> int:
        if value not in LOCATION_ID_TO_NAME or value == 32:
            raise ValueError("expected OEM locationID, not UNKNOWN")
        return value


class ManualMove(_Location):
    operation: Literal["move"]
    well: str | int
    position_flag: Literal[0, 1, 2]  # pseudo-home / calibrated high / calibrated low

    @field_validator("position_flag", mode="before")
    @classmethod
    def integer_flag(cls, value: Any) -> Any:
        if type(value) is not int:
            raise ValueError("position_flag must be an integer")
        return value

    @field_validator("well")
    @classmethod
    def valid_well(cls, value: str | int) -> str | int:
        well_id_from_label(value)
        return value


class ManualLower(_Location):
    operation: Literal["lower"]


class ManualLift(_Location):
    operation: Literal["lift"]
    # Required explicit null selects zHigh; integer means zLow - height_steps.
    height_steps: int | None


class ManualLiquid(_Request):
    operation: Literal["aspirate", "dispense"]
    channels: list[Annotated[int, Field(ge=0, le=3)]] = Field(min_length=1, max_length=4)
    volume_ul: float
    speed: float


class ManualMix(_Request):
    operation: Literal["mix"]
    channels: list[Annotated[int, Field(ge=0, le=3)]] = Field(min_length=1, max_length=4)
    volume_ul: float
    aspirate_speed: float
    dispense_speed: float
    cycles: int = Field(ge=1, le=50)


class ManualLoadTip(_Request):
    operation: Literal["load_tip"]
    tray: int = Field(ge=1, le=5)
    well: str = Field(pattern=r"^[ABab](?:[1-9]|1[0-2])$")
    overpress: bool = False
    lift_z: bool = False


class SourceLoadTips(_Request):
    """newloadTips, not calibration loadTips or manual tray/well pickup.

    A matching tip type with force_new_tip=False returns without realignment.
    Pipette is the OEM zero-based index; -1 is the fixed four-head group.
    """
    operation: Literal["source_load_tips"]
    tip_type: Literal[50, 200]
    pipette: int = Field(ge=-1, le=3)
    force_new_tip: bool


class SourceMix(_Request):
    """ControlLib.mmix scientific procedure, not repeated manual strokes."""
    operation: Literal["source_mix"]
    volume_ul: float
    air_ul: float = 15.0
    aspirate_speed: float = 100.0
    dispense_speed: float = 20.0
    aspirate_delay_ms: int | None = None
    dispense_delay_ms: int | None = None
    cycles: int = Field(default=2, ge=0)
    mix_type: Literal["N", "H", "C"] = "N"
    tip_dip: bool = True


class SourceAir(_Request):
    operation: Literal["source_aspirate_air", "source_dispense_air"]
    volume_ul: float


class SourcePurge(_Request):
    operation: Literal["source_purge"]
    speed: float = 30.0
    amp: bool = False
    ntd: bool = False


SOURCE_MANUAL_MODELS = {"source_load_tips": SourceLoadTips, "source_mix": SourceMix,
    "source_aspirate_air": SourceAir, "source_dispense_air": SourceAir,
    "source_purge": SourcePurge}


class ManualMeasureFluidHeight(_Request):
    operation: Literal["measure_fluid_height"]
    speed: int = 300


class ManualFluidOffset(_Request):
    operation: Literal["source_fluid_offset"]
    plate: Literal["TC", "MS", "OC", "RC", "STRIP", "OCMS"]
    speed: int = 300
    transfer_fluid: bool = True
    skip_steps: int = Field(default=4, ge=1)


class DiagnosticDetectFluid(_Request):
    operation: Literal["diagnostic_detect_fluid"]


class ManualCalwithFluid(_Request):
    operation: Literal["source_calwith_fluid"]


ManualStep = Annotated[Union[ManualMove, ManualLower, ManualLift, ManualLiquid, ManualMix,
    ManualLoadTip, ManualMeasureFluidHeight, ManualFluidOffset, DiagnosticDetectFluid,
    ManualCalwithFluid, SourceLoadTips, SourceMix, SourceAir, SourcePurge], Field(discriminator="operation")]


class ManualPipettingRequest(_Request):
    protocol_id: str = Field(min_length=1)
    steps: list[ManualStep] = Field(min_length=1)


def compile_manual_pipetting(request: ManualPipettingRequest | Mapping[str, Any]) -> ProtocolDocument:
    """Compile only the operator's ordered steps; submit via /protocol/execute.

    Mix is explicit repeated aspiration/dispense strokes, NOT OEM mixAll or mmix
    (which have additional semantics). Speeds and channels are never defaulted.
    """
    req = request if isinstance(request, ManualPipettingRequest) else ManualPipettingRequest.model_validate(request)
    actions: list[ProtocolAction] = []

    def add(kind: ProtocolActionKind, params: dict[str, Any], step: int) -> None:
        actions.append(ProtocolAction(action_id=f"manual-{step}-{len(actions)}", stage_id="manual",
            kind=kind, params=params, metadata={"manual_step": step}))

    def liquid(operation: str, channels: list[int], volume: float, speed: float, step: int) -> None:
        command_type = PipetteAspirateCommand if operation == "aspirate" else PipetteDispenseCommand
        try:
            command = command_type(volume_ul=volume, channels=channels, speed=speed)
        except PipetteValidationError as exc:
            raise ValueError(str(exc)) from exc
        kind = ProtocolActionKind.PIPETTE_ASPIRATE if operation == "aspirate" else ProtocolActionKind.PIPETTE_DISPENSE
        add(kind, command.to_payload(), step)

    for index, step in enumerate(req.steps):
        if isinstance(step, (ManualMove, ManualLower, ManualLift)):
            add(ProtocolActionKind.PIPETTE_POSITION, step.model_dump(), index)
        elif isinstance(step, (ManualLoadTip, ManualMeasureFluidHeight, ManualFluidOffset,
                               DiagnosticDetectFluid, ManualCalwithFluid,
                               SourceLoadTips, SourceMix, SourceAir, SourcePurge)):
            add(ProtocolActionKind.PIPETTE_MANUAL_PHYSICAL, step.model_dump(), index)
        elif isinstance(step, ManualLiquid):
            liquid(step.operation, step.channels, step.volume_ul, step.speed, index)
        else:
            for _ in range(step.cycles):
                liquid("aspirate", step.channels, step.volume_ul, step.aspirate_speed, index)
                liquid("dispense", step.channels, step.volume_ul, step.dispense_speed, index)
    return validate_protocol_document(ProtocolDocument(protocol_id=req.protocol_id,
        stages=(ProtocolStage(stage_id="manual", title="Manual pipetting", actions=tuple(actions)),),
        metadata={"manual_scope": "explicit_steps_only", "well_alignment": "source_machine_tip_location"}))


def manual_position_plan(params: Mapping[str, Any]) -> dict[str, Any]:
    """Validate untrusted native params again, then compile existing finite leaves."""
    models = {"move": ManualMove, "lower": ManualLower, "lift": ManualLift}
    try:
        model = models[params["operation"]]
    except (KeyError, TypeError):
        raise ValueError("unknown manual positioning operation") from None
    step = model.model_validate(dict(params))
    inputs: dict[str, Any] = {"location": step.location_id}
    if isinstance(step, ManualMove):
        operation = "manual_pipette_move"
        inputs.update(well=well_id_from_label(step.well), position_flag=step.position_flag)
    elif isinstance(step, ManualLower):
        operation = "pipette_lower"
    else:
        operation = "pipette_lift"
        inputs["height"] = step.height_steps
    return compile_finite_plate_operation(operation, source_leaf_available=True, **inputs)


def bind_manual_position_handler(*, command_store: Any, execute_plan: Callable,
                                 require_motion_ready: Callable[[], None]) -> Callable:
    """Bind to app.state.oem_workflow_plan_executor; never spawn a worker here.

    Add returned callable to _protocol_live_handlers()[PIPETTE_POSITION]. Pass
    _protocol_command_store(), app.state.oem_workflow_plan_executor and
    _require_motion_route_ready. The generic native executor supplies action/state.
    """
    def handle(action: ProtocolAction, state: Any) -> Mapping[str, Any]:
        plan = manual_position_plan(action.params)
        require_motion_ready()
        identity = f"manual:{action.action_id}"
        with command_store.workflow_context(state.job_id, source_occurrence_id=identity):
            command_store.assert_workflow_current(state.job_id)
            result = execute_plan(plan, action, state)
        # Preserve canonical failures and raw child receipts. Never promote
        # completed status or absent outcome to proof of physical placement.
        return {**dict(result), "physical_effect_verified": False}
    return handle


def manual_physical_plan(params: Mapping[str, Any]) -> dict[str, Any]:
    if params.get("operation") in SOURCE_MANUAL_MODELS:
        step = SOURCE_MANUAL_MODELS[params["operation"]].model_validate(dict(params))
        return compile_finite_plate_operation("manual_source_pipette", source_leaf_available=True,
                                             request=step.model_dump())
    models = {"load_tip": ManualLoadTip, "measure_fluid_height": ManualMeasureFluidHeight,
              "source_fluid_offset": ManualFluidOffset,
              "diagnostic_detect_fluid": DiagnosticDetectFluid,
              "source_calwith_fluid": ManualCalwithFluid}
    try:
        model = models[params["operation"]]
    except (KeyError, TypeError):
        raise ValueError("unknown manual physical operation") from None
    step = model.model_validate(dict(params))
    operation = ("diagnostic_detect_fluid" if isinstance(step, DiagnosticDetectFluid) else
                 "source_calwith_fluid" if isinstance(step, ManualCalwithFluid) else
                 "manual_load_tip" if isinstance(step, ManualLoadTip) else
                 "source_fluid_offset" if isinstance(step, ManualFluidOffset) else "measure_fluid_height")
    return compile_finite_plate_operation(operation, source_leaf_available=True,
                                         **step.model_dump(exclude={"operation"}))


def bind_manual_physical_handler(*, command_store: Any, execute_plan: Callable,
        require_motion_ready: Callable[[], None], provider_getter: Callable,
        receipt_store_getter: Callable, calibration_settings_getter: Callable | None = None) -> Callable:
    """Ordinary native binding; receipt work runs inline inside the finite owner.

    No fake OEM opcode/arguments, separate scheduler, or request-selected callback.
    The provider and receipt store are the application's existing shared owners.
    """
    import asyncio
    from .services.pipette_service import run_pipette_operation

    def receipt(name: str, call, command_id, identity, inputs):
        provider = provider_getter()
        async def inline(label, body, *, timeout_s):
            command_store.assert_deck_execution_current(command_id, boundary="manual_pipette_inline")
            return body()
        operation = {"query_tip_status_all": "query_all_pipette_tip_states"}.get(name, name)
        return asyncio.run(run_pipette_operation(operation, call,
            get_transport=lambda: provider.primitives.pipette_transport,
            run_blocking=inline, receipt_store=receipt_store_getter(),
            requested_inputs={"manual_inputs": dict(inputs), "source_occurrence_id": identity["source_identity"]},
            runtime_binding={"idempotency_key": identity["source_identity"],
                "entrypoint_id": "protocol.pipette_manual_physical", "caller_class": "protocol_manual",
                "parent_operator_command_id": command_id}))

    def handle(action: ProtocolAction, state: Any) -> Mapping[str, Any]:
        plan = manual_physical_plan(action.params)
        require_motion_ready()
        with command_store.workflow_context(state.job_id, source_occurrence_id=f"manual:{action.action_id}"):
            command_store.assert_workflow_current(state.job_id)
            provider = provider_getter()
            provider._manual_pipette_receipt_runner = receipt
            if plan["operation"] in {"source_fluid_offset", "diagnostic_detect_fluid", "source_calwith_fluid", "manual_source_pipette"}:
                from .runtime_state import get_active_oem_runtime_state_store
                from .pipette.manual_settings import read_pipette_operation_settings
                from .oem_job_preparation import construct_new_machine_source_model
                # The OEM ClassMachineStatus constructor always creates its
                # logical plate/well objects, even for a diagnostic manual
                # action without RunJob preparation. An ordinary manual
                # document has no prepared source_model; model those default
                # empty wells without asserting anything about physical fluid.
                if not state.source_model.trays:
                    state.source_model = construct_new_machine_source_model()
                provider._manual_pipette_source_state = state
                provider._manual_pipette_source_settings = read_pipette_operation_settings(
                    get_active_oem_runtime_state_store())["runtime_values"]
                if plan["operation"] == "source_calwith_fluid":
                    if calibration_settings_getter is None:
                        raise RuntimeError("calibration settings service not bound")
                    provider._manual_calibration_settings = calibration_settings_getter()
            result = execute_plan(plan, action, state)
        children = result.get("completed_children") or result.get("source_children") or []
        saved_revision_id = (children[0].get("result") or {}).get("saved_revision_id") if children else None
        return {**dict(result), "physical_effect_verified": False,
                "calibration_persisted": plan["operation"] == "source_calwith_fluid" and
                saved_revision_id is not None}
    return handle
