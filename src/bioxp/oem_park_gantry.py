"""Strict receipt acceptance for source-shaped ControlLib.parkGantry."""
from __future__ import annotations

import math
from typing import Any


class OemParkReceiptError(ValueError):
    pass


SOURCE_ANCHOR = "ControlLib.parkGantry:7071-7122"


def _mapping(value: Any, name: str, keys: set[str]) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise OemParkReceiptError(f"{name} must be an object")
    missing = sorted(keys - set(value))
    extra = sorted(set(value) - keys)
    if missing:
        raise OemParkReceiptError(f"{name} missing fields: {missing}")
    if extra:
        raise OemParkReceiptError(f"{name} has unexpected fields: {extra}")
    return value


def _bool(value: Any, name: str) -> bool:
    if type(value) is not bool:
        raise OemParkReceiptError(f"{name} must be an exact bool")
    return value


def _int(value: Any, name: str) -> int:
    if type(value) is not int:
        raise OemParkReceiptError(f"{name} must be an exact integer")
    return value


def _number(value: Any, name: str) -> float:
    if type(value) not in {int, float} or not math.isfinite(float(value)):
        raise OemParkReceiptError(f"{name} must be a finite number")
    return float(value)


def evaluate_park_gantry_receipt(receipt: dict[str, Any]) -> dict[str, Any]:
    row = _mapping(
        receipt,
        "receipt",
        {
            "current_location_before",
            "current_well_before",
            "tip_present_before",
            "tip_cleanup",
            "rehome",
            "preliminary_route_without_z_verified",
            "xy_home_residual_steps",
            "final_route_profile",
            "final_route_verified",
            "location_28_persisted",
        },
    )
    current_location = _int(row["current_location_before"], "current_location_before")
    _int(row["current_well_before"], "current_well_before")
    tip_present = _bool(row["tip_present_before"], "tip_present_before")
    rehome = _bool(row["rehome"], "rehome")
    preliminary = _bool(row["preliminary_route_without_z_verified"], "preliminary_route_without_z_verified")
    final_route = _bool(row["final_route_verified"], "final_route_verified")
    persisted = _bool(row["location_28_persisted"], "location_28_persisted")

    if current_location == 28:
        if row["tip_cleanup"] is not None or rehome or preliminary or row["xy_home_residual_steps"] is not None:
            raise OemParkReceiptError("already-parked OEM branch must not contain movement receipts")
        if row["final_route_profile"] is not None or final_route:
            raise OemParkReceiptError("already-parked OEM branch must not contain a final route receipt")
        production_pass = bool(persisted and not tip_present)
        return {
            "ok": production_pass,
            "outcome": "already_parked" if production_pass else "already_parked_with_unresolved_tip_state",
            "oem_effective_pass": True,
            "production_admission_pass": production_pass,
            "physical_effect_verified": False,
            "failures": [] if production_pass else ["tip_present_in_oem_early_return"],
            "source_anchor": SOURCE_ANCHOR,
        }

    failures: list[str] = []
    if tip_present:
        cleanup = _mapping(
            row["tip_cleanup"],
            "tip_cleanup",
            {
                "waste_route_verified",
                "ejection_postcondition_empty",
                "z_80000_verified",
                "x_79000_verified",
                "machine_tip_state_cleared",
            },
        )
        for field, failure in (
            ("waste_route_verified", "waste_route_not_verified"),
            ("ejection_postcondition_empty", "tip_ejection_not_verified_empty"),
            ("z_80000_verified", "z_clearance_not_verified"),
            ("x_79000_verified", "x_clearance_not_verified"),
            ("machine_tip_state_cleared", "machine_tip_state_not_cleared"),
        ):
            if not _bool(cleanup[field], f"tip_cleanup.{field}"):
                failures.append(failure)
    elif row["tip_cleanup"] is not None:
        raise OemParkReceiptError("no-tip branch cannot contain a tip_cleanup receipt")

    if rehome:
        if not preliminary:
            failures.append("preliminary_route_without_z_not_verified")
        residuals = row["xy_home_residual_steps"]
        if not isinstance(residuals, list) or len(residuals) != 2:
            raise OemParkReceiptError("xy_home_residual_steps must contain exactly two values for rehome")
        x_error = _number(residuals[0], "x_home_residual_steps")
        y_error = _number(residuals[1], "y_home_residual_steps")
        if abs(x_error) > 100:
            failures.append("x_home_residual_exceeded")
        if abs(y_error) > 100:
            failures.append("y_home_residual_exceeded")
    else:
        if preliminary or row["xy_home_residual_steps"] is not None:
            raise OemParkReceiptError("non-rehome branch cannot contain preliminary route or XY-home receipts")

    if type(row["final_route_profile"]) is not int or row["final_route_profile"] != 2:
        failures.append("final_route_profile_not_2")
    if not final_route:
        failures.append("final_route_not_verified")
    if not persisted:
        failures.append("location_28_not_persisted")
    ok = not failures
    return {
        "ok": ok,
        "outcome": "park_verified" if ok else "park_failed_closed",
        "production_admission_pass": ok,
        "physical_effect_verified": ok,
        "failures": failures,
        "source_anchor": SOURCE_ANCHOR,
        "final_location": 28 if ok else None,
    }
