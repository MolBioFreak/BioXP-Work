"""Liquid-surface detection helpers: search window, per-channel contact, level grouping.

Z steps increase downward. Every height here is an absolute Z step value for the
loaded tip (the tip length correction is applied once, to the station heights).
"""
from __future__ import annotations

from typing import Any, Iterable

from .tip_length import Z_STEPS_PER_MM


def search_window(z_high: int, z_low: int, tip_offset_steps: int, floor_clearance_steps: int) -> tuple[int, int]:
    """Start at the station's calibrated high; never search below its calibrated low."""
    start = int(z_high) + int(tip_offset_steps)
    floor = int(z_low) + int(tip_offset_steps) - int(floor_clearance_steps)
    if floor <= start:
        raise ValueError("liquid search floor is not below its start height; reduce the floor clearance")
    return start, floor


def contact_from_timestamps(*, start_steps: int, stop_steps: int, started_at: float, stopped_at: float,
                            stamp: float | None) -> tuple[int | None, str]:
    """Estimate where one channel touched during a shared constant-speed descent.

    The pump reports only when it sensed liquid; Z is read once after it stops.
    Linear interpolation over the descent is an estimate, reported as such.
    """
    if stamp is None:
        return None, "no_contact_time"
    if not (stopped_at > started_at) or not (started_at <= stamp <= stopped_at):
        return None, "contact_time_outside_descent"
    fraction = (stamp - started_at) / (stopped_at - started_at)
    return int(round(start_steps + (stop_steps - start_steps) * fraction)), "interpolated_descent"


def level_groups(contacts: dict[int, int], mode: str, tolerance_steps: int) -> list[tuple[list[int], int]]:
    """Channels to aspirate together and the surface they share, highest liquid first.

    The depth of a group is its lowest surface, so every tip in it is submerged.
    """
    if not contacts:
        return []
    ordered = sorted(contacts, key=lambda c: (contacts[c], c))
    if mode == "together":
        return [(sorted(ordered), max(contacts.values()))]
    groups: list[tuple[list[int], int]] = []
    current: list[int] = []
    first = 0
    for channel in ordered:
        if current and contacts[channel] - first > tolerance_steps:
            groups.append((sorted(current), max(contacts[c] for c in current)))
            current = []
        if not current:
            first = contacts[channel]
        current.append(channel)
    groups.append((sorted(current), max(contacts[c] for c in current)))
    return groups


def aspirate_target(surface_steps: int, submerge_steps: int, floor_steps: int) -> int:
    """Submerge below the found surface, never below the search floor."""
    return min(int(surface_steps) + int(submerge_steps), int(floor_steps))


def height_above_floor_mm(contact_steps: int | None, floor_steps: int) -> str | None:
    if contact_steps is None:
        return None
    return format((floor_steps - contact_steps) / Z_STEPS_PER_MM, ".3f")


def missing_channels(record: dict[str, Any], expected: Iterable[int]) -> list[int]:
    return [c for c in expected if not record["channels"].get(str(c), {}).get("found")]


def interpolate(table: list[list[float]], x: float, *, key: int = 0) -> float | None:
    """Piecewise-linear lookup in a measured [height_mm, volume_ul] table (either direction)."""
    rows = sorted(table, key=lambda r: r[key])
    if len(rows) < 2:
        return None
    other = 1 - key
    for lo, hi in zip(rows, rows[1:]):
        if lo[key] <= x <= hi[key]:
            span = hi[key] - lo[key]
            return lo[other] if span == 0 else lo[other] + (hi[other] - lo[other]) * (x - lo[key]) / span
    lo, hi = (rows[0], rows[1]) if x < rows[0][key] else (rows[-2], rows[-1])
    span = hi[key] - lo[key]
    return None if span == 0 else lo[other] + (hi[other] - lo[other]) * (x - lo[key]) / span


def volume_check(*, contact_steps: int | None, low_steps: int, check: dict[str, Any], channel: int) -> dict[str, Any]:
    """Compare a found surface with the expected volume. Reports; never moves."""
    expected = (check.get("expected_ul") or {}).get(str(channel))
    table, gap = check.get("table"), check.get("gap_mm")
    if contact_steps is None or expected is None:
        return {"checked": False, "reason": "no contact" if contact_steps is None else "no expected volume"}
    if not table or gap is None:
        return {"checked": False, "reason": "plate height-to-volume table or well-bottom gap not measured",
                "expected_ul": expected}
    found_mm = (low_steps - contact_steps) / Z_STEPS_PER_MM + float(gap)   # above well bottom
    expected_mm = interpolate(table, float(expected), key=1)
    found_ul = interpolate(table, found_mm, key=0)
    upper = interpolate(table, float(expected) * (1 + float(check.get("tolerance_fraction", 0.15))), key=1)
    if expected_mm is None or found_ul is None or upper is None:
        return {"checked": False, "reason": "plate table has fewer than two points", "expected_ul": expected}
    tolerance_mm = max(float(check.get("tolerance_mm", 1.0)), upper - expected_mm)
    deviation = found_mm - expected_mm
    return {"checked": True, "expected_ul": expected, "found_ul": round(found_ul, 2),
            "expected_height_mm": round(expected_mm, 3), "found_height_mm": round(found_mm, 3),
            "deviation_mm": round(deviation, 3), "tolerance_mm": round(tolerance_mm, 3),
            "within_tolerance": abs(deviation) <= tolerance_mm}
