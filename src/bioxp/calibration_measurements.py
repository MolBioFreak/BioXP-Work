"""Operator observations and run-local operands; no sensor or science defaults."""
from __future__ import annotations

from copy import deepcopy
from decimal import Decimal, InvalidOperation
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, StrictFloat, StrictInt, StrictStr, AfterValidator, model_validator


def finite_number(value):
    try:
        number = Decimal(str(value))
    except InvalidOperation as exc:
        raise ValueError("finite decimal measurement required") from exc
    if not number.is_finite():
        raise ValueError("finite decimal measurement required")
    return value


# Strings retain the operator's exact decimal spelling; JSON numbers retain
# their value/type. Never coerce a bool into a scientific numeric value.
NumericText = Annotated[StrictStr, Field(pattern=r"^[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?$")]
ExactNumber = Annotated[NumericText | StrictInt | StrictFloat, AfterValidator(finite_number)]
Identity = Annotated[str, Field(strict=True, min_length=1)]


class Input(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class MeasurementField(Input):
    field_id: Identity
    label: Identity
    unit: Literal["degC", "uL", "mg", "g", "steps"]
    module: Literal["rc", "oc", "tc", "pipette"]
    location: str | None = None
    zone: str | None = None
    target: ExactNumber | None = None
    commanded: ExactNumber | None = None
    controller_reported: ExactNumber | None = None
    repetitions: Annotated[int, Field(strict=True, ge=1)] = 1
    context: dict = Field(default_factory=dict)


class MeasurementForm(Input):
    fields: list[MeasurementField] = Field(min_length=1)

    @model_validator(mode="after")
    def unique_fields(self):
        if len({f.field_id for f in self.fields}) != len(self.fields):
            raise ValueError("measurement field IDs must be unique")
        return self


class MeasurementResponse(Input):
    measurements: dict[str, list[ExactNumber]]


class PendingMeasurement(Input):
    stage_id: Identity
    action_id: Identity
    source_occurrence_id: Identity
    measurement_form: MeasurementForm


class PendingMeasurementReview(PendingMeasurement):
    reason: str | None


class MeasurementRecord(PendingMeasurement):
    measurements: dict[str, list[ExactNumber]]
    reviewer: str
    note: str | None
    job_id: str | None
    source: Literal["operator_external_measurement"]


class ResultReference(Input):
    """Exact earlier native occurrence, never latest-by-label or cross-job."""
    stage_id: Identity
    action_id: Identity
    source_occurrence_id: Identity
    output: Literal["measurements", "result"]
    path: list[StrictStr | StrictInt] = Field(default_factory=list)


Operand = ExactNumber | ResultReference


def pending_measurement(action):
    form = action.to_payload()["params"].get("measurement_form")
    if form is None:
        return None
    return {"stage_id": action.stage_id, "action_id": action.action_id,
            "source_occurrence_id": action.source_occurrence_id or action.action_id,
            "measurement_form": MeasurementForm.model_validate(form).model_dump(mode="json", exclude_unset=True)}


def accept_measurements(state, measurements, *, reviewer, note=None):
    pending = state.pending_measurement
    if pending is None:
        if measurements is not None:
            raise ValueError("current review has no measurement form")
        return None
    response = MeasurementResponse.model_validate({"measurements": measurements})
    form = MeasurementForm.model_validate(pending["measurement_form"])
    if set(response.measurements) != {f.field_id for f in form.fields}:
        raise ValueError("response must contain exactly the authored measurement fields")
    if any(len(response.measurements[f.field_id]) != f.repetitions for f in form.fields):
        raise ValueError("response count must match authored repetitions")
    record = {**deepcopy(pending), "measurements": deepcopy(response.measurements),
              "reviewer": reviewer, "note": note, "job_id": state.job_id,
              "source": "operator_external_measurement"}
    MeasurementRecord.model_validate(record)
    state.measurement_reviews.append(record)
    state.pending_measurement = None
    return record


def resolve_reference(reference: ResultReference, state):
    rows = state.measurement_reviews if reference.output == "measurements" else state.action_results
    matches = [r for r in rows if all(r.get(k) == getattr(reference, k)
               for k in ("stage_id", "action_id", "source_occurrence_id"))]
    if len(matches) != 1:
        raise ValueError("run-local measurement/result occurrence unavailable or ambiguous")
    row = matches[0]
    if reference.output == "result" and row.get("ok") is not True:
        raise ValueError("referenced action did not complete successfully")
    value = row.get("measurements") if reference.output == "measurements" else row
    try:
        for key in reference.path:
            if isinstance(key, int) and (not isinstance(value, (list, tuple)) or key < 0):
                raise KeyError(key)
            value = value[key]
    except (KeyError, IndexError, TypeError) as exc:
        raise ValueError("run-local selected output unavailable") from exc
    return deepcopy(value)


def operand(value: Operand, state=None):
    if isinstance(value, ResultReference):
        if state is None:
            raise ValueError("runtime operand needs its original job")
        return finite_number(resolve_reference(value, state))
    return value
