"""ControlLib.zOffset:3387-3627, inside an already-owned finite child.

The bindings are trusted source operations, not an operator-selected opcode table.
No catch/finally is added to the source body: failure retains preceding effects.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Mapping

from .oem_calibration import CALIBRATION_PLATES, CalibrationExecutionError, _Sequence, calibration_samples
from ..oem_compat.position_table import well_id_from_label

Result = dict[str, Any]


@dataclass(frozen=True)
class FluidScanBindings:
    facts: Callable[[], Mapping[str, Any]]
    publish_plate: Callable[[int, int], Result]  # updatePlateLocation at scan entry
    load_tips: Callable[[], Result]  # source loadTips(T50, forcenewtip:true)
    move: Callable[[int, str | int, int], Result]  # scriptmoveTo, position flag 0 or 1
    publish: Callable[[int, int], Result]  # updateLocation, including source return quirk
    aspirate: Callable[[int], Result]  # masp(100, volume)
    dispense: Callable[[], Result]  # mdsa(100)
    detect: Callable[[int], Result]  # scrFluidDetection, not a one-well mock
    query_tips: Callable[[], Result]  # queryTipStatus(-1), returns tip_exists
    eject: Callable[[], Result]  # ejectAllTips(true,true)
    clear_tip_loaded: Callable[[], Result]


def z_offset(plate: str, bindings: FluidScanBindings, *, speed: int = 300,
             transfer_fluid: bool = True, skip_steps: int = 4) -> Result:
    """Run source prefill and A/B detection in order, returning rounded motor Z.

    The source's return move physically targets the saved location but publishes
    the scan location with the saved well. Both are exposed, never reconciled.
    Source tip-ejection loop has no retry cap; Stop remains with its owner.
    """
    samples = calibration_samples(plate, skip_steps)
    location, plate_id, columns, volume = CALIBRATION_PLATES[plate]
    seq = _Sequence("ControlLib.zOffset:3387-3627")
    seq.result.update(plate=plate, speed=speed, transfer_fluid=transfer_fluid,
                      skip_steps=skip_steps, samples=[], location=location)

    def step(name: str, fn: Callable[[], Result]) -> Result:
        return seq.step(name, fn)

    def move(target: int, well: str | int, flag: int) -> None:
        step("scriptmoveTo", lambda: bindings.move(target, well, flag))
        step("updateLocation", lambda: bindings.publish(target, well_id_from_label(well)))

    step("updatePlateLocation", lambda: bindings.publish_plate(location, plate_id))
    if transfer_fluid and location != 0:
        seq.step("loadTips", bindings.load_tips, allow_false=True)
        saved = bindings.facts()
        # Source integer division, including its (40+49)/50 = 1 branch.
        batches = (volume + 49) // 50
        aliquot = volume // batches
        for _ in range(batches):
            for column in range(0, columns, skip_steps):
                for row in ("A", "B"):
                    well = f"{row}{column + 1}"
                    move(16, 0, 0)
                    step("masp", lambda: bindings.aspirate(aliquot))
                    move(location, well, 1)
                    step("mdsa", bindings.dispense)
        step("scriptmoveTo.return", lambda: bindings.move(saved["current_location"], saved["current_well"], 1))
        # Source publishes 'val', not the return-to-tray destination.
        step("source_return_publication", lambda: bindings.publish(location, saved["current_well"]))
        step("ejectAllTips", bindings.eject)
        step("TipLoaded=false", bindings.clear_tip_loaded)

    heights: list[int] = []
    for well in samples:
        # The OEM ignores loadTips' Boolean here and proceeds to the move.
        seq.step("loadTips", bindings.load_tips, allow_false=True)
        saved = bindings.facts()
        move(location, well, 1)
        measured = step("scrFluidDetection", lambda: bindings.detect(speed))
        height = measured["source_return"]
        heights.append(height)
        seq.result["samples"].append({"well": well, "position_steps": height})
        step("scriptmoveTo.return", lambda: bindings.move(saved["current_location"], saved["current_well"], 1))
        step("source_return_publication", lambda: bindings.publish(location, saved["current_well"]))
        tips = step("queryTipStatus", bindings.query_tips)
        while tips["tip_exists"]:
            step("ejectAllTips", bindings.eject)
            tips = step("queryTipStatus", bindings.query_tips)
        step("TipLoaded=false", bindings.clear_tip_loaded)
    # C# integer cast truncates toward zero; .5 is added BEFORE truncation.
    seq.result["source_return"] = int(sum(heights) / len(heights) + .5)
    return seq.result
