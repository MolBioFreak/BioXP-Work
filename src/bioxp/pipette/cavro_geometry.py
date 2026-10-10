"""Authored shared-Z geometry. No machine scale, vessel shape or tip defaults.

Offsets are signed in the authored axis direction; a detected stop is not an
independently measured meniscus. Decimal conversions must encode whole native
steps/speed units exactly, rather than silently rounding scientific inputs.
"""
from __future__ import annotations
from decimal import Decimal, localcontext, Inexact
from typing import Annotated, Any, Literal, Mapping, Union
from pydantic import AfterValidator, BaseModel, ConfigDict, Field
from .cavro_commands import Number


def _positive_number(value):
    if Decimal(str(value)) <= 0:
        raise ValueError("expected a positive number")
    return value


PositiveNumber = Annotated[Number, AfterValidator(_positive_number)]


class GeometryInput(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)


class AxisCalibration(GeometryInput):
    calibration_id: str = Field(min_length=1)
    steps_per_mm: PositiveNumber
    native_speed_per_mm_s: PositiveNumber
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


class AreaPoint(GeometryInput):
    height_mm: Number
    area_mm2: Number


class VesselGeometry(GeometryInput):
    vessel_id: str
    provenance: str
    shape: str | None = None
    bottom: str | None = None
    # Explicit mathematical model, never inferred from the shape label.
    cross_section_model: Literal["constant", "linear_area"] | None = None
    area_mm2: PositiveNumber | None = None
    area_points: list[AreaPoint] | None = None
    fill_ul: Number | None = None
    fill_provenance: str | None = None
    dead_volume_ul: Number | None = None
    inaccessible_volume_ul: Number | None = None
    surface_timestamp: str | None = None


class VesselReference(GeometryInput):
    kind: Literal["vessel"]
    vessel: VesselGeometry
    selected_level: Literal["surface", "bottom", "accessible_floor"]
    bottom_head_steps: int
    calibration: AxisCalibration
    tip_correction_mm: Number
    # Missing optional evidence is not a new execution gate. This is an
    # authored fallback, not a generated physical assumption.
    fallback_position_steps: int


class Vector3(GeometryInput):
    x: Number
    y: Number
    z: Number


class ChannelWell(GeometryInput):
    channel: int = Field(ge=0, le=3)
    row: int
    column: int
    well_id: str


class RackTransform(GeometryInput):
    calibration_id: str
    provenance: str
    origin_mm: Vector3
    row_pitch_mm: Vector3
    column_pitch_mm: Vector3
    tool_offset_mm: Vector3
    reference_tip_offset_mm: Vector3
    reference_channel: int = Field(ge=0, le=3)
    channel_wells: list[ChannelWell]
    # Full signed affine rows, native steps per millimetre. Includes rotation
    # and sign, no borrowed OEM XY or Z scaling.
    x_steps_per_mm: Vector3
    y_steps_per_mm: Vector3
    z_steps_per_mm: Vector3
    native_origin_steps: Vector3


class TipFootprint(GeometryInput):
    offset_mm: Vector3
    radius_mm: Number | None = None
    provenance: str


class RackReference(GeometryInput):
    kind: Literal["rack"]
    transform: RackTransform
    fallback_position_steps: int


SurfaceReference = Annotated[Union[AuthoredReference, DetectedReference, VesselReference, RackReference], Field(discriminator="kind")]



class NativeOffset(GeometryInput):
    unit: Literal["steps"]
    offset_steps: int
    speed_native: int = Field(gt=0)


class MetricOffset(GeometryInput):
    unit: Literal["mm"]
    depth_mm: Number  # positive into liquid, negative retract
    speed_mm_s: PositiveNumber
    calibration: AxisCalibration
    tip_offset_mm: Number  # signed head-coordinate correction, explicitly authored


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
    length_mm: Number | None = None
    orifice_mm: Number | None = None
    usable_capacity_ul: Number | None = None
    seal_geometry: str | None = None
    seal_diameter_mm: Number | None = None
    rack_id: str | None = None
    # A declaration is context, not automatic physical pickup or fit proof.
    geometry_provenance: str | None = None


class MountedTip(GeometryInput):
    channel: int = Field(ge=0, le=3)
    profile: TipProfile
    well_id: str | None = None
    surface_steps: int | None = None
    tip_offset_steps: int | None = None
    footprint: TipFootprint | None = None
    vessel: VesselGeometry | None = None


class SharedHead(GeometryInput):
    mounted_tips: list[MountedTip]
    trajectory_policy: Literal["authored_common_z"]
    context_id: str = Field(min_length=1)


class PickupCalibration(GeometryInput):
    calibration_id: str = Field(min_length=1)
    rack_id: str = Field(min_length=1)
    # Existing calibrated station/well positioning, never an inferred T50 map.
    positioning: dict[str, Any]
    rack_transform: RackTransform | None = None
    pickup: SurfaceMove
    retract: SurfaceMove | None
    channel_wells: dict[str, str]
    provenance: str = Field(min_length=1)


class ProfilePickup(GeometryInput):
    operation: Literal["profile_pickup"]
    profile: TipProfile
    calibration: PickupCalibration


def decimal_work_precision(*values) -> int:
    """Digit/exponent budget for exact finite input arithmetic, not tolerance."""
    def budget(value):
        if isinstance(value, Mapping):
            return sum(budget(v) for v in value.values())
        if isinstance(value, (list, tuple)):
            return sum(budget(v) for v in value)
        if isinstance(value, bool) or value is None:
            return 0
        try:
            number = Decimal(str(value))
            if number.is_finite():
                digits = number.as_tuple()
                return len(digits.digits) + abs(int(digits.exponent))
        except (ValueError, ArithmeticError):
            pass
        return 0
    return 80 + sum(budget(v) for v in values)


def exact_native(value: Decimal, label: str) -> int:
    if not value.is_finite() or value != value.to_integral_value():
        raise ValueError(f"{label}: authored calibration does not produce integral native units; no rounding")
    return int(value)


def resolve_displacement(offset: Mapping[str, Any]) -> tuple[int, int]:
    if offset["unit"] == "steps":
        delta, speed = offset["offset_steps"], offset["speed_native"]
    else:
        calibration = offset["calibration"]
        sign = 1 if calibration["positive_steps_direction"] == "down" else -1
        depth, tip, scale, velocity, speed_scale = [Decimal(str(v)) for v in (
            offset["depth_mm"], offset["tip_offset_mm"], calibration["steps_per_mm"],
            offset["speed_mm_s"], calibration["native_speed_per_mm_s"])]
        with localcontext() as context:
            context.prec = sum(len(v.as_tuple().digits) + abs(int(v.as_tuple().exponent))
                               for v in (depth, tip, scale, velocity, speed_scale)) + 20
            delta = exact_native((depth * sign + tip) * scale, "Z displacement")
            speed = exact_native(velocity * speed_scale, "Z speed")
    return delta, speed


def resolve_surface_move(move: Mapping[str, Any], detections: Mapping[str, Any]) -> dict[str, Any]:
    reference = move["reference"]
    if reference["kind"] == "rack":
        observation = resolve_rack_transform(reference["transform"])
        base = observation["target_steps"]["z"] if observation["geometry_used"] else reference["fallback_position_steps"]
    elif reference["kind"] == "vessel":
        observation = vessel_reference(reference)
        base = observation["position_steps"]
    elif reference["kind"] == "authored":
        base = reference["position_steps"]
        observation = None
    else:
        observation = detections.get(reference["detection_id"])
        if observation is None or type(observation.get("final_position_steps")) is not int:
            raise ValueError(f"surface reference operand unavailable: {reference['detection_id']}")
        base = observation["final_position_steps"] + (reference.get("penetration_correction_steps") or 0)
    delta, speed = resolve_displacement(move["displacement"])
    return {"target_steps": base + delta, "speed_native": speed,
            "reference_steps": base, "authored": dict(move), "detection": observation,
            "physical_surface_verified": False, "shared_z": True,
            "inactive_mounted_tips_move": True}


def vessel_state(vessel: Mapping[str, Any], delta=Decimal(0)) -> dict[str, Any]:
    fill = vessel.get("fill_ul")
    unavailable = vessel.get("inaccessible_volume_ul")
    dead = vessel.get("dead_volume_ul")
    # Inaccessible is an explicitly supplied total, not silently added to dead.
    with localcontext() as ctx:
        ctx.prec = decimal_work_precision(vessel, delta)
        planned = None if fill is None else Decimal(str(fill)) + delta
        accessible = None if planned is None or unavailable is None else planned - Decimal(str(unavailable))
    return {"vessel_id": vessel["vessel_id"], "authored": dict(vessel),
            "planned_fill_ul": None if planned is None else format(planned, "f"),
            "planned_delta_ul": format(delta, "f"), "dead_volume_ul": dead,
            "inaccessible_volume_ul": unavailable,
            "planned_accessible_ul": None if accessible is None else format(accessible, "f"),
            "below_accessible_floor": None if accessible is None else accessible < 0,
            "actual_fill_ul": None, "basis": "authored_liquid_intent_not_measured"}


def height_at_volume(vessel: Mapping[str, Any], volume: Decimal) -> Decimal:
    if volume < 0:
        raise ValueError("negative planned fill; no extrapolation")
    if vessel.get("cross_section_model") == "constant" and vessel.get("area_mm2") is not None:
        area = Decimal(str(vessel["area_mm2"]))
        height = volume / area
        if height * area != volume:
            raise ValueError("height has no exact decimal representation")
        return height
    points = vessel.get("area_points") or []
    if vessel.get("cross_section_model") != "linear_area" or len(points) < 2:
        raise ValueError("cross-section model unavailable")
    if Decimal(str(points[0]["height_mm"])) != 0:
        raise ValueError("cross-section origin unavailable; no extrapolation")
    remaining = volume
    for left, right in zip(points, points[1:]):
        h0, h1, a0, a1 = (Decimal(str(v)) for v in
            (left["height_mm"], right["height_mm"], left["area_mm2"], right["area_mm2"]))
        span = h1 - h0
        if span <= 0 or min(a0, a1) < 0 or max(a0, a1) == 0:
            raise ValueError("cross-section heights/areas do not define a positive volume interval")
        capacity = (a0 + a1) * span / 2
        if remaining <= capacity:
            if remaining == 0:
                return h0
            slope = (a1 - a0) / span
            if slope == 0:
                depth = remaining / a0
            else:
                discriminant = a0 * a0 + 2 * slope * remaining
                root = discriminant.sqrt()
                if root * root != discriminant:
                    raise ValueError("surface inversion not exactly representable; no implicit rounding")
                depth = 2 * remaining / (a0 + root)
            if a0 * depth + slope * depth * depth / 2 != remaining:
                raise ValueError("surface inversion not exactly representable")
            return h0 + depth
        remaining -= capacity
    raise ValueError("fill beyond declared cross-section; no extrapolation")


def vessel_reference(reference: Mapping[str, Any]) -> dict[str, Any]:
    vessel = reference["vessel"]
    out = {"position_steps": reference["fallback_position_steps"], "geometry_used": False,
           "vessel_state": vessel_state(vessel), "height_mm": None, "observation": None}
    try:
        with localcontext() as ctx:
            ctx.prec = decimal_work_precision(reference) * 2
            ctx.traps[Inexact] = True
            level = reference["selected_level"]
            volume = 0 if level == "bottom" else vessel.get(
                "fill_ul" if level == "surface" else "inaccessible_volume_ul")
            if volume is None:
                raise ValueError("selected fill/accessibility unknown")
            height = Decimal(0) if level == "bottom" else height_at_volume(vessel, Decimal(str(volume)))
            cal = reference["calibration"]
            sign = -1 if cal["positive_steps_direction"] == "down" else 1
            value = Decimal(reference["bottom_head_steps"]) + (
                sign * height + Decimal(str(reference["tip_correction_mm"]))) * Decimal(str(cal["steps_per_mm"]))
            out.update(position_steps=exact_native(value, "vessel reference"), geometry_used=True, height_mm=format(height, "f"))
    except (ValueError, ArithmeticError) as exc:
        out["observation"] = str(exc)
    return out


def resolve_rack_transform(transform: Mapping[str, Any], shared_head=None, channels=()) -> dict[str, Any]:
    """Selected common head pose and all mounted tip footprints; no fit gate."""
    result = {"authored": dict(transform), "geometry_used": False, "observation": None,
              "mounted_tip_footprints": [], "physical_fit_verified": False, "shared_z": True}
    try:
        with localcontext() as ctx:
            ctx.prec = decimal_work_precision(transform) * 2
            ctx.traps[Inexact] = True
            axes = ("x", "y", "z")
            wells = transform["channel_wells"]
            matches = [w for w in wells if w["channel"] == transform["reference_channel"]]
            if len(matches) != 1:
                raise ValueError("unique reference-channel mapping unavailable")
            def well_position(w):
                return {a: Decimal(str(transform["origin_mm"][a])) +
                        w["row"] * Decimal(str(transform["row_pitch_mm"][a])) +
                        w["column"] * Decimal(str(transform["column_pitch_mm"][a])) for a in axes}
            target = well_position(matches[0])
            head = {a: target[a] - Decimal(str(transform["tool_offset_mm"][a])) -
                    Decimal(str(transform["reference_tip_offset_mm"][a])) for a in axes}
            native = {a: exact_native(Decimal(str(transform["native_origin_steps"][a])) +
                sum(head[b] * Decimal(str(transform[a + "_steps_per_mm"][b])) for b in axes),
                "rack " + a) for a in axes}
            result.update(geometry_used=True, head_mm={a: format(head[a], "f") for a in axes}, target_steps=native)
            for tip in (shared_head or {}).get("mounted_tips", []):
                footprint = tip.get("footprint")
                mapping = [w for w in wells if w["channel"] == tip["channel"]]
                row = {"channel": tip["channel"], "pump_active": tip["channel"] in channels,
                       "profile": tip["profile"], "well_mapping": mapping,
                       "tip_xyz_mm": None, "xy_bounds_mm": None, "well_offset_mm": None, "footprint": footprint,
                       "vessel_state": vessel_state(tip["vessel"]) if tip.get("vessel") else None}
                if footprint:
                    xyz = {a: head[a] + Decimal(str(transform["tool_offset_mm"][a])) +
                           Decimal(str(footprint["offset_mm"][a])) for a in axes}
                    row["tip_xyz_mm"] = {a: format(xyz[a], "f") for a in axes}
                    if footprint.get("radius_mm") is not None:
                        radius = Decimal(str(footprint["radius_mm"]))
                        if radius >= 0:
                            row["xy_bounds_mm"] = {a: [format(xyz[a] - radius, "f"), format(xyz[a] + radius, "f")] for a in ("x", "y")}
                    if len(mapping) == 1:
                        center = well_position(mapping[0])
                        row["well_offset_mm"] = {a: format(xyz[a] - center[a], "f") for a in axes}
                result["mounted_tip_footprints"].append(row)
    except (ValueError, ArithmeticError) as exc:
        result.update(geometry_used=False, observation=str(exc))
    return result


def mounted_trajectory(context, target_steps, transform=None, channels=()):
    """Predicted common-head tip coordinates, including inactive/unknown tips."""
    rows = []
    with localcontext() as ctx:
        ctx.prec = decimal_work_precision(context, target_steps, transform) * 2
        for tip in (context or {}).get('mounted_tips', []):
            position: dict[str, Any] = {a: None for a in ('x', 'y', 'z')}
            footprint = tip.get('footprint')
            if footprint and transform:
                offset = {a: Decimal(str(transform['tool_offset_mm'][a])) + Decimal(str(footprint['offset_mm'][a])) for a in position}
                for axis in position:
                    if target_steps.get(axis) is not None:
                        position[axis] = str(Decimal(target_steps[axis]) + sum(
                            offset[a] * Decimal(str(transform[axis + '_steps_per_mm'][a])) for a in position))
            elif tip.get('tip_offset_steps') is not None and target_steps.get('z') is not None:
                position['z'] = str(target_steps['z'] + tip['tip_offset_steps'])
            rows.append({'channel': tip['channel'], 'pump_active': tip['channel'] in channels,
                         'well_id': tip.get('well_id'), 'profile': tip['profile'],
                         'tip_reference_steps': position, 'footprint': footprint,
                         'coordinate_known': all(v is not None for v in position.values())})
    return {'mounted_tip_footprints': rows, 'head_target_steps': target_steps,
            'physical_fit_verified': False, 'shared_z': True, 'inactive_mounted_tips_move': True}


def shared_head_evidence(context: Mapping[str, Any] | None, channels: list[int]) -> dict[str, Any]:
    tips = (context or {}).get("mounted_tips", [])
    effective = [t["surface_steps"] - t["tip_offset_steps"] for t in tips
                 if t.get("surface_steps") is not None and t.get("tip_offset_steps") is not None]
    return {"shared_z": True, "pump_channels": list(channels),
            "inactive_mounted_tips_move": True, "authored_context": context,
            "unequal_authored_surface_references": len(set(effective)) > 1,
            "independent_channel_z": False, "physical_fit_verified": False,
            "geometry_limit": "One authored head trajectory; pump selection does not isolate mounted tips."}
