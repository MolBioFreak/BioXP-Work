"""Calibration actions within the existing ordered native protocol owner."""
from __future__ import annotations

from decimal import Decimal
from typing import Annotated, Literal

from pydantic import Field, TypeAdapter

from .calibration_analysis import AnalysisRequest, ANALYSIS_ADAPTER, AnalysisResult, analyze
from .calibration_measurements import Input, Identity, Operand, ResultReference, operand, resolve_reference, MeasurementForm, MeasurementResponse
from .oem_calibration_settings import CalibrationSettingsPatch, PositionCalibrationPatch, CalibrationRunResponse


class AnalysisAction(Input):
    operation: Literal["analyze"]
    analysis: AnalysisRequest


class HeightCalculate(Input):
    operation: Literal["height_calculate"]
    scope_id: Identity
    plate: Literal["TC", "MS", "OC", "RC", "STRIP"]
    measured_z: Operand


class HeightApply(Input):
    operation: Literal["height_apply"]
    calculation: ResultReference


class HeightDecision(Input):
    operation: Literal["height_decision"]
    scope_id: Identity
    decision: Literal["accept", "restore"]


CalibrationAction = Annotated[AnalysisAction | HeightCalculate | HeightApply | HeightDecision, Field(discriminator="operation")]
CALIBRATION_ACTION_ADAPTER = TypeAdapter(CalibrationAction)

class HeightAdjustment(Input):
    scope_id: str
    run_id: str
    plate: str
    measured_raw_z: int
    calculation: dict
    patch: CalibrationSettingsPatch
    fluid_reference_revision: str
    applied: Literal[False]


class HeightCalculatedResult(Input):
    ok: Literal[True]
    height_adjustment: HeightAdjustment


class HeightAppliedResult(Input):
    ok: Literal[True]
    run_id: str
    applied: Literal[True]
    saved_revision_id: str
    active_revision_id: str
    positions: list[PositionCalibrationPatch]


class HeightDecisionResult(Input):
    ok: Literal[True]
    run_id: str
    calibration_run: CalibrationRunResponse


class AnalysisActionResult(Input):
    ok: Literal[True]
    analysis: AnalysisResult


class CalibrationFailure(Input):
    ok: Literal[False]
    error: str
    operation: str
    unavailable_operand_or_calculation: Literal[True]


CALIBRATION_RESULT_ADAPTER = TypeAdapter(HeightCalculatedResult | HeightAppliedResult | HeightDecisionResult | AnalysisActionResult | CalibrationFailure)


ID_NAMES = {2: "LOC_TC", 0: "LOC_MS", 1: "LOC_OC", 3: "LOC_RC",
            11: "LOC_STRIP1", 12: "LOC_STRIP2", 13: "LOC_STRIP3", 14: "LOC_STRIP4"}


def calibration_contract():
    return {"measurement_form": MeasurementForm.model_json_schema(),
            "measurement_response": MeasurementResponse.model_json_schema(),
            "analysis_request": ANALYSIS_ADAPTER.json_schema(),
            "analysis_result": AnalysisResult.model_json_schema(),
            "action_params": CALIBRATION_ACTION_ADAPTER.json_schema(),
            "action_result": CALIBRATION_RESULT_ADAPTER.json_schema(),
            "result_reference": ResultReference.model_json_schema(),
            "native_kind": "calibration",
            "thermal_application": "source_write_semantics_unresolved"}


def calibration_handler(settings_getter):
    """Lazy existing settings owner: analysis never acquires hardware/settings."""
    def execute(action, state):
        req = CALIBRATION_ACTION_ADAPTER.validate_python(action.to_payload()["params"])
        try:
            if isinstance(req, AnalysisAction):
                return {"ok": True, "analysis": analyze(req.analysis, state)}
            settings = settings_getter()
            if isinstance(req, HeightCalculate):
                number = Decimal(str(operand(req.measured_z, state)))
                if number != number.to_integral_value():
                    raise ValueError("height measurement must be integer source steps; no implicit rounding")
                z = int(number)
                if req.scope_id not in state.calibration_scopes:
                    run = settings.begin_run(machine_calibrated=settings.active_snapshot.machine_calibrated)
                    state.calibration_scopes[req.scope_id] = run["run_id"]
                    settings.update_run(run["run_id"], job_id=state.job_id, scope_id=req.scope_id,
                        source_occurrence_id=action.source_occurrence_id or action.action_id)
                run_id = state.calibration_scopes[req.scope_id]
                by_name = {r["name"]: r for r in settings.read()["saved_positions"]}
                from .pipette.oem_calibration import calibration_adjustment
                ref = settings.active_snapshot.fluid_reference
                calculation = calibration_adjustment(req.plate, z,
                    z_lows={i: by_name[name]["zLow"] for i, name in ID_NAMES.items()}, fluid_reference=ref)
                patch = CalibrationSettingsPatch(positions=tuple(PositionCalibrationPatch(
                    name=ID_NAMES[i], zLow=value) for i,value in calculation["position_table_z_low_updates"].items()))
                return {"ok": True, "height_adjustment": {
                    "scope_id": req.scope_id, "run_id": run_id, "plate": req.plate,
                    "measured_raw_z": z, "calculation": calculation,
                    "patch": patch.model_dump(mode="json", exclude_unset=True),
                    "fluid_reference_revision": str(ref["REVISION"]),
                    "applied": False}}
            if isinstance(req, HeightApply):
                # Only a prior successful calculate action may supply a write;
                # arbitrary browser measurements cannot masquerade as patches.
                if req.calculation.output != "result" or req.calculation.path != ["height_adjustment"]:
                    raise ValueError("height_apply selects the earlier height_adjustment output")
                calc = resolve_reference(req.calculation, state)
                if state.calibration_scopes.get(calc["scope_id"]) != calc["run_id"]:
                    raise ValueError("calculation scope unavailable in this job")
                patch = CalibrationSettingsPatch.model_validate(calc["patch"])
                saved = settings.save(patch, liquid_reference_revision=calc["fluid_reference_revision"],
                    run_id=calc["run_id"], measurement={"plate": calc["plate"],
                    "measured_raw_z": calc["measured_raw_z"],
                    "calculated_z_lows": {p.name:p.zLow for p in patch.positions},
                    "settings_updates": calc["calculation"]["settings_updates"],
                    "source_occurrence_id": action.source_occurrence_id or action.action_id,
                    "calculation_reference": req.calculation.model_dump()})
                run = settings.store.read_calibration_run(calc["run_id"])
                settings.update_run(calc["run_id"], decision_status="pending" if run["machine_calibrated"] else "no_previous_values")
                return {"ok": True, "run_id": calc["run_id"], "applied": True,
                    "saved_revision_id": saved["saved_revision_id"],
                    "active_revision_id": saved["active_revision_id"],
                    "positions": calc["patch"]["positions"]}
            run_id = state.calibration_scopes.get(req.scope_id)
            if run_id is None:
                raise ValueError("calibration scope unavailable in this job")
            settings.update_run(run_id, body_completed=True)
            run = settings.decide_run(run_id, req.decision)
            return {"ok": True, "run_id": run_id, "calibration_run": run}
        except (ValueError, KeyError, TypeError, ArithmeticError) as exc:
            # A missing selected operand is this action's result, never a
            # readiness condition imposed on unrelated methods or controls.
            return {"ok": False, "error": str(exc), "operation": req.operation,
                    "unavailable_operand_or_calculation": True}
    def handle(action, state):
        result = execute(action, state)
        CALIBRATION_RESULT_ADAPTER.validate_python(result)
        return result
    return handle
