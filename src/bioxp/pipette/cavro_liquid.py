"""Explicit resolved liquid recipes -> original ADP ordered application phases.

This is NOT a class resolver. The BMS owner supplies selected/resolved values
and their ledger. No Water defaults, cross-recipe interpolation, implicit tip
pickup, implicit geometry, or inferred total multi aspiration is performed.
"""
from __future__ import annotations

from copy import deepcopy
from decimal import Decimal
from typing import Any, Literal

from pydantic import Field, ValidationError

from .cavro_application import (_Input, ApplicationRequest, IMPLEMENTATION,
                                 Instruction, compile_application, _json_copy)


class Segment(_Input):
    volume_ul: float = Field(ge=0)
    speed_ul_s: float = Field(gt=0)


class Air(_Input):
    volume_ul: float = Field(ge=0)
    speed_ul_s: float = Field(gt=0)


class Aliquot(_Input):
    # Lists of native application motion instructions, not location metadata.
    before: list[Instruction]
    segments: list[Segment]
    after: list[Instruction]


class Multi(_Input):
    sample_count: int = Field(ge=0)
    sample_volume_ul: float = Field(ge=0)
    conditioning_volume_ul: float = Field(ge=0)
    conditioning_back_to_source_count: int = Field(ge=0)
    conditioning_speed_ul_s: float = Field(gt=0)
    conditioning_before: list[Instruction]
    conditioning_after: list[Instruction]
    excess_volume_ul: float = Field(ge=0)
    # None explicitly retains excess; otherwise an authored destination/motion.
    excess: Aliquot | None
    reaspiration: Air | None
    dispense_to_reaspiration_delay_ms: int | None = Field(ge=0)
    aliquots: list[Aliquot]


class Recipe(_Input):
    mode: Literal["single", "multi"]
    channels: list[int]
    timeout_ms: int = Field(gt=0)
    target_liquid_ul: float = Field(ge=0)
    # Mandatory corrected total displacement even for multi. No guessed sum of
    # sample/conditioning/excess can masquerade as a manufacturer correction.
    commanded_aspiration_ul: float = Field(ge=0)
    aspiration_speed_ul_s: float = Field(gt=0)
    aspiration_delay_ms: int = Field(ge=0)
    leading_air: Air | None
    trailing_air: Air | None
    before_leading_air: list[Instruction]
    before_liquid: list[Instruction]
    after_liquid: list[Instruction]
    before_dispense: list[Instruction]
    after_dispense: list[Instruction]
    dispense_segments: list[Segment]
    # Full empty is independent of segment volume and is never per multi sample.
    final_empty_speed_ul_s: float | None = Field(gt=0)
    final_empty_before: list[Instruction]
    multi: Multi | None
    phase_settings: dict[Literal["leading_air", "aspirate", "trailing_air", "dispense"], dict[str, Any]] = Field(default_factory=dict)
    liquid_settings: dict[str, Any] = Field(default_factory=dict)


def correction_at_target(target_ul: Any, correction: dict[str, Any]) -> str:
    """Exact point or explicitly authored affine calibration, no interpolation.

    Return a decimal string to keep arithmetic exact; callers choose/native
    encode the number explicitly. Raw points and coefficients stay in ledger.
    """
    target = Decimal(str(target_ul))
    if not target.is_finite():
        raise ValueError("nonfinite target")
    if correction.get("kind") == "points":
        matches = [p for p in correction["points"] if Decimal(str(p["target_ul"])) == target]
        if len(matches) != 1:
            raise ValueError("exact unique correction point required; no interpolation")
        value = Decimal(str(matches[0]["commanded_ul"]))
    elif correction.get("kind") == "affine":
        value = target * Decimal(str(correction["scale"])) + Decimal(str(correction["offset_ul"]))
    else:
        raise ValueError("unsupported correction function")
    if not value.is_finite() or value < 0:
        raise ValueError("invalid corrected displacement")
    return format(value, "f")


def compile_liquid_recipe(request: dict[str, Any]) -> dict[str, Any]:
    raw = _json_copy(request)
    try:
        r = Recipe.model_validate(raw)
        if (r.mode == "multi") != (r.multi is not None):
            raise ValueError("multi parameters required exactly for multi mode")
        if r.multi is not None:
            if r.dispense_segments:
                raise ValueError("multi segments belong to each aliquot, not the single dispense field")
            if r.multi.sample_count != len(r.multi.aliquots):
                raise ValueError("sample_count must equal the complete authored aliquot list")
            if r.multi.reaspiration is not None and r.multi.dispense_to_reaspiration_delay_ms is None:
                raise ValueError("re-aspiration requires an explicit dispense-to-re-aspiration delay")
        operations = []
        base = {"channels": r.channels, "timeout_ms": r.timeout_ms}
        def append_motion(values):
            operations.extend(v.model_dump(exclude_unset=True) for v in values)
        def settings(phase):
            if phase in r.phase_settings:
                operations.append({"operation": "settings", **base, "values": r.phase_settings[phase]})
        def stroke(kind, volume, speed, **extra):
            operations.append({"operation": kind, **base, "volume_ul": volume, "speed_ul_s": speed, **extra})
        def dispense(segments):
            for s in segments:
                stroke("dispense", s.volume_ul, s.speed_ul_s)
        append_motion(r.before_leading_air)
        if r.leading_air is not None:
            settings("leading_air")
            stroke("leading_air", r.leading_air.volume_ul, r.leading_air.speed_ul_s)
        append_motion(r.before_liquid)
        settings("aspirate")
        stroke("aspirate", r.commanded_aspiration_ul, r.aspiration_speed_ul_s, target_liquid_ul=r.target_liquid_ul)
        operations.append({"operation": "delay", "duration_ms": r.aspiration_delay_ms})
        append_motion(r.after_liquid)
        if r.trailing_air is not None:
            settings("trailing_air")
            stroke("trailing_air", r.trailing_air.volume_ul, r.trailing_air.speed_ul_s)
        settings("dispense")
        if r.multi is None:
            append_motion(r.before_dispense)
            dispense(r.dispense_segments)
            append_motion(r.after_dispense)
        else:
            m = r.multi
            append_motion(m.conditioning_before)
            for _ in range(m.conditioning_back_to_source_count):
                stroke("dispense", m.conditioning_volume_ul, m.conditioning_speed_ul_s)
            append_motion(m.conditioning_after)
            append_motion(r.before_dispense)
            for aliquot in m.aliquots:
                append_motion(aliquot.before)
                dispense(aliquot.segments)
                if m.reaspiration is not None:
                    operations.append({"operation": "delay", "duration_ms": m.dispense_to_reaspiration_delay_ms})
                    stroke("reaspirate", m.reaspiration.volume_ul, m.reaspiration.speed_ul_s)
                append_motion(aliquot.after)
            if m.excess is not None:
                append_motion(m.excess.before)
                dispense(m.excess.segments)
                append_motion(m.excess.after)
            append_motion(r.after_dispense)
        append_motion(r.final_empty_before)
        if r.final_empty_speed_ul_s is not None:
            operations.append({"operation": "empty", **base, "speed_ul_s": r.final_empty_speed_ul_s})
        application = {"implementation": IMPLEMENTATION, "operations": operations,
                       "liquid_settings": {**r.liquid_settings, "resolved_recipe": raw}}
        compiled = compile_application(application)
        return {**compiled, "recipe_requested": raw,
                "application": None if compiled["issues"] else application}
    except ValidationError as exc:
        issues = [{"code": "invalid_recipe", "category": "representation",
                   "path": "/" + "/".join(map(str, e["loc"])), "message": e["msg"]}
                  for e in exc.errors(include_url=False, include_context=False)]
    except (ValueError, TypeError) as exc:
        issues = [{"code": "invalid_recipe", "category": "representation", "path": "/", "message": str(exc)}]
    return {"recipe_requested": raw, "operations": None, "application": None, "issues": issues}
