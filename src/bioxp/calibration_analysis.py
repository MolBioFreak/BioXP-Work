"""Descriptive metrology with explicitly selected fits; no implicit application."""
from __future__ import annotations

from decimal import Decimal, localcontext
from typing import Annotated, Literal

from pydantic import Field, TypeAdapter

from .calibration_measurements import Input, Identity, Operand, ExactNumber, operand


class VolumeObservation(Input):
    kind: Literal["volume"]
    measured_ul: Operand


class MassObservation(Input):
    kind: Literal["mass_density"]
    mass_mg: Operand
    density_mg_per_ul: Operand


class PipetteSample(Input):
    point_id: Identity
    target_ul: Operand
    commanded_ul: Operand
    observation: Annotated[VolumeObservation | MassObservation, Field(discriminator="kind")]


class CorrectionDomain(Input):
    minimum_ul: ExactNumber
    maximum_ul: ExactNumber


class PipetteAnalysis(Input):
    kind: Literal["pipette"]
    # Exact-point additive correction and inverse linear response are different
    # scientific choices. Neither is chosen from the number of samples.
    model: Literal["descriptive", "exact_point_offset", "affine_inverse_ols"]
    context: dict
    samples: list[PipetteSample] = Field(min_length=1)
    evaluate_targets_ul: list[ExactNumber] = Field(default_factory=list)
    domain_ul: CorrectionDomain | None = None
    verification_samples: list[PipetteSample] = Field(default_factory=list)


class ThermalSample(Input):
    point_id: Identity
    module: Literal["rc", "oc", "tc"]
    zone: str | None = None
    location: str | None = None
    target_c: Operand
    commanded_c: Operand
    controller_reported_c: Operand | None = None
    measured_c: Operand
    context: dict = Field(default_factory=dict)


class ThermalAnalysis(Input):
    kind: Literal["thermal"]
    samples: list[ThermalSample] = Field(min_length=1)


AnalysisRequest = Annotated[PipetteAnalysis | ThermalAnalysis, Field(discriminator="kind")]
ANALYSIS_ADAPTER = TypeAdapter(AnalysisRequest)


class PointStatistics(Input):
    point_id: str
    count: int
    target: str
    commanded: str
    measured: list[str]
    mean: str
    bias: str
    sample_sd: str | None
    cv_percent: str | None


class PointCorrection(Input):
    target_ul: str
    commanded_ul: str


class ExactPointCorrection(Input):
    kind: Literal["points"]
    points: list[PointCorrection]


class AffineCorrection(Input):
    kind: Literal["affine"]
    scale: str
    offset_ul: str


Correction = Annotated[ExactPointCorrection | AffineCorrection, Field(discriminator="kind")]


class CorrectionEvaluation(Input):
    target_ul: ExactNumber
    commanded_ul: str
    outside_authored_domain: bool | None = None


class FitResidual(Input):
    point_id: str
    measured_mean_ul: str
    predicted_ul: str
    residual_ul: str


class AffineFit(Input):
    response_slope: str
    response_intercept_ul: str
    weighting: Literal["equal_point_means"] = "equal_point_means"
    residuals: list[FitResidual]


class AnalysisResult(Input):
    ok: Literal[True] = True
    kind: Literal["pipette", "thermal"]
    requested: AnalysisRequest
    resolved_samples: list[PipetteSample | ThermalSample]
    points: list[PointStatistics]
    correction: Correction | None
    evaluated: list[CorrectionEvaluation] = Field(default_factory=list)
    applied: Literal[False] = False
    affine_fit: AffineFit | None = None
    verification_points: list[PointStatistics] = Field(default_factory=list)
    thermal_application: Literal["source_write_semantics_unresolved"] | None = None


def text(value):
    return format(value, "f")


def statistics(point_id, target, command, values):
    n = len(values)
    mean = sum(values) / Decimal(n)
    sd = (sum((x - mean) ** 2 for x in values) / Decimal(n - 1)).sqrt() if n > 1 else None
    return PointStatistics(point_id=point_id, count=n, target=text(target), commanded=text(command),
        measured=[text(v) for v in values], mean=text(mean), bias=text(mean - target),
        sample_sd=None if sd is None else text(sd),
        cv_percent=None if sd is None or mean == 0 else text(sd / abs(mean) * 100))


def analyze(request, state=None):
    req = ANALYSIS_ADAPTER.validate_python(request)
    with localcontext() as ctx:
        ctx.prec = 28
        groups = {}
        resolved = []
        def number(x):
            return Decimal(str(operand(x, state)))
        for sample in req.samples:
            raw = sample.model_dump(mode="python", exclude_unset=True)
            if isinstance(sample, PipetteSample):
                for key in ("target_ul", "commanded_ul"):
                    raw[key] = operand(getattr(sample, key), state)
                obs = sample.observation
                for key in (("measured_ul",) if isinstance(obs, VolumeObservation) else ("mass_mg", "density_mg_per_ul")):
                    raw["observation"][key] = operand(getattr(obs, key), state)
            else:
                for key in ("target_c", "commanded_c", "controller_reported_c", "measured_c"):
                    value = getattr(sample, key)
                    if value is not None:
                        raw[key] = operand(value, state)
            resolved.append(raw)
            if isinstance(req, PipetteAnalysis):
                target, command = number(sample.target_ul), number(sample.commanded_ul)
                obs = sample.observation
                if isinstance(obs, VolumeObservation):
                    measured = number(obs.measured_ul)
                else:
                    density = number(obs.density_mg_per_ul)
                    if density <= 0:
                        raise ValueError("authored density must be positive for mass conversion")
                    measured = number(obs.mass_mg) / density
                identity = (target, command)
            else:
                target, command, measured = number(sample.target_c), number(sample.commanded_c), number(sample.measured_c)
                # Do not pool different modules/zones/locations under one label.
                identity = (target, command, sample.module, sample.zone, sample.location)
                if sample.controller_reported_c is not None:
                    number(sample.controller_reported_c)
            if sample.point_id not in groups:
                groups[sample.point_id] = (identity, [])
            if groups[sample.point_id][0] != identity:
                raise ValueError("one point_id must retain the same target, command and module/location")
            groups[sample.point_id][1].append(measured)
        points = [statistics(k, identity[0], identity[1], values) for k, (identity, values) in groups.items()]
        correction = None
        evaluated = []
        affine_fit = None
        verification_points = []
        if isinstance(req, PipetteAnalysis):
            if req.domain_ul is not None and number(req.domain_ul.minimum_ul) > number(req.domain_ul.maximum_ul):
                raise ValueError("authored correction domain has inverted bounds")
            if req.verification_samples:
                verification_points = analyze({"kind":"pipette", "model":"descriptive", "context":req.context,
                    "samples":[s.model_dump(mode="json", exclude_unset=True) for s in req.verification_samples]}, state)["points"]
            if req.model == "exact_point_offset":
                if len({Decimal(p.target) for p in points}) != len(points):
                    raise ValueError("exact-point correction requires unique targets")
                correction = {"kind": "points", "points": [
                    {"target_ul": p.target, "commanded_ul": text(Decimal(p.commanded) + Decimal(p.target) - Decimal(p.mean))}
                    for p in points]}
            elif req.model == "affine_inverse_ols":
                # Unweighted OLS over authored point means: measured=a*command+b.
                xs, ys = [Decimal(p.commanded) for p in points], [Decimal(p.mean) for p in points]
                xmean, ymean = sum(xs) / len(xs), sum(ys) / len(ys)
                denominator = sum((x - xmean) ** 2 for x in xs)
                if not denominator:
                    raise ValueError("affine fit needs distinct commanded points")
                slope = sum((x-xmean)*(y-ymean) for x,y in zip(xs,ys)) / denominator
                if not slope:
                    raise ValueError("constant measured response has no inverse affine correction")
                intercept = ymean - slope * xmean
                correction = {"kind": "affine", "scale": text(1 / slope), "offset_ul": text(-intercept / slope)}
                affine_fit = {"response_slope":text(slope), "response_intercept_ul":text(intercept),
                    "residuals":[{"point_id":p.point_id, "measured_mean_ul":p.mean,
                        "predicted_ul":text(slope * Decimal(p.commanded) + intercept),
                        "residual_ul":text(Decimal(p.mean) - (slope * Decimal(p.commanded) + intercept))} for p in points]}
            if req.evaluate_targets_ul and correction is None:
                raise ValueError("evaluation requires an explicitly selected correction model")
            if correction is not None:
                # The existing native recipe function is the ONLY evaluation
                # authority. This action returns a selected model, never edits a
                # recipe, controller or settings and never applies it twice.
                from .pipette.cavro_liquid import correction_at_target
                evaluated = [{"target_ul": t, "commanded_ul": correction_at_target(t, correction),
                    "outside_authored_domain":None if req.domain_ul is None else not (number(req.domain_ul.minimum_ul) <= number(t) <= number(req.domain_ul.maximum_ul))}
                             for t in req.evaluate_targets_ul]
        result = AnalysisResult(kind=req.kind, requested=req.model_dump(mode="json", exclude_unset=True),
            resolved_samples=resolved, points=points, correction=correction, evaluated=evaluated,
            affine_fit=affine_fit, verification_points=verification_points,
            thermal_application="source_write_semantics_unresolved" if isinstance(req, ThermalAnalysis) else None)
        return result.model_dump(mode="json")
