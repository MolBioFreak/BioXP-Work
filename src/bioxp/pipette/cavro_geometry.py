"""Authored shared-Z geometry. No machine scale, vessel shape or tip defaults.

Offsets are signed in the authored axis direction; a detected stop is not an
independently measured meniscus. Decimal conversions must encode whole native
steps/speed units exactly, rather than silently rounding scientific inputs.
"""
from __future__ import annotations
from decimal import Decimal
from typing import Annotated, Any, Literal, Mapping, Union
from pydantic import BaseModel, ConfigDict, Field


class GeometryInput(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)


class AxisCalibration(GeometryInput):
    calibration_id: str = Field(min_length=1)
    steps_per_mm: float = Field(gt=0)
    native_speed_per_mm_s: float = Field(gt=0)
    positive_steps_direction: Literal["down", "up"]


class AuthoredReference(GeometryInput):
    kind: Literal["authored"]
    position_steps: int
    reference_id: str = Field(min_length=1)


class DetectedReference(GeometryInput):
    kind: Literal["detected"]
    detection_id: str = Field(min_length=1)
    # Signed correction from the stopped head coordinate to the authored
    # surface reference. Null preserves the stop coordinate, without claiming
    # that penetration has been measured or corrected.
    penetration_correction_steps: int | None = None


SurfaceReference = Annotated[Union[AuthoredReference, DetectedReference], Field(discriminator="kind")]


class NativeOffset(GeometryInput):
    unit: Literal["steps"]
    offset_steps: int
    speed_native: int = Field(gt=0)


class MetricOffset(GeometryInput):
    unit: Literal["mm"]
    depth_mm: float  # positive into liquid, negative retract
    speed_mm_s: float = Field(gt=0)
    calibration: AxisCalibration
    tip_offset_mm: float  # signed head-coordinate correction, explicitly authored


SurfaceOffset = Annotated[Union[NativeOffset, MetricOffset], Field(discriminator="unit")]


class SurfaceMove(GeometryInput):
    operation: Literal["surface_move"]
    reference: SurfaceReference
    displacement: SurfaceOffset
    intent: Literal["immersion", "retract", "contact", "authored"]


class TipProfile(GeometryInput):
    profile_id: str = Field(min_length=1)
    nominal_volume_ul: float = Field(gt=0)
    manufacturer: str
    sku: str
    filtered: bool
    length_mm: float | None = None
    orifice_mm: float | None = None
    usable_capacity_ul: float | None = None
    rack_id: str | None = None
    # A declaration is context, not automatic physical pickup or fit proof.
    geometry_provenance: str | None = None


class MountedTip(GeometryInput):
    channel: int = Field(ge=0, le=3)
    profile: TipProfile
    well_id: str | None = None
    surface_steps: int | None = None
    tip_offset_steps: int | None = None


class SharedHead(GeometryInput):
    mounted_tips: list[MountedTip]
    trajectory_policy: Literal["authored_common_z"]
    context_id: str = Field(min_length=1)


class PickupCalibration(GeometryInput):
    calibration_id: str = Field(min_length=1)
    rack_id: str = Field(min_length=1)
    # Existing calibrated station/well positioning, never an inferred T50 map.
    positioning: dict[str, Any]
    pickup: SurfaceMove
    retract: SurfaceMove | None
    channel_wells: dict[str, str]
    provenance: str = Field(min_length=1)


class ProfilePickup(GeometryInput):
    operation: Literal["profile_pickup"]
    profile: TipProfile
    calibration: PickupCalibration


def exact_native(value: Decimal, label: str) -> int:
    if not value.is_finite() or value != value.to_integral_value():
        raise ValueError(f"{label}: authored calibration does not produce integral native units; no rounding")
    return int(value)


def resolve_surface_move(move: Mapping[str, Any], detections: Mapping[str, Any]) -> dict[str, Any]:
    reference, offset = move["reference"], move["displacement"]
    if reference["kind"] == "authored":
        base = reference["position_steps"]
        observation = None
    else:
        observation = detections.get(reference["detection_id"])
        if observation is None or type(observation.get("final_position_steps")) is not int:
            raise ValueError(f"surface reference operand unavailable: {reference['detection_id']}")
        base = observation["final_position_steps"] + (reference.get("penetration_correction_steps") or 0)
    if offset["unit"] == "steps":
        delta, speed = offset["offset_steps"], offset["speed_native"]
    else:
        calibration = offset["calibration"]
        sign = 1 if calibration["positive_steps_direction"] == "down" else -1
        delta = exact_native((Decimal(str(offset["depth_mm"])) * sign + Decimal(str(offset["tip_offset_mm"])))
                             * Decimal(str(calibration["steps_per_mm"])), "Z displacement")
        speed = exact_native(Decimal(str(offset["speed_mm_s"])) * Decimal(str(calibration["native_speed_per_mm_s"])), "Z speed")
    return {"target_steps": base + delta, "speed_native": speed,
            "reference_steps": base, "authored": dict(move), "detection": observation,
            "physical_surface_verified": False, "shared_z": True,
            "inactive_mounted_tips_move": True}


def shared_head_evidence(context: Mapping[str, Any] | None, channels: list[int]) -> dict[str, Any]:
    tips = (context or {}).get("mounted_tips", [])
    effective = [t["surface_steps"] - t["tip_offset_steps"] for t in tips
                 if t.get("surface_steps") is not None and t.get("tip_offset_steps") is not None]
    return {"shared_z": True, "pump_channels": list(channels),
            "inactive_mounted_tips_move": True, "authored_context": context,
            "unequal_authored_surface_references": len(set(effective)) > 1,
            "independent_channel_z": False, "physical_fit_verified": False,
            "geometry_limit": "One authored head trajectory; pump selection does not isolate mounted tips."}
