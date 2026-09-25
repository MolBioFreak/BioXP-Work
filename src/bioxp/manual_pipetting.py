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


ManualStep = Annotated[Union[ManualMove, ManualLower, ManualLift, ManualLiquid, ManualMix], Field(discriminator="operation")]


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
