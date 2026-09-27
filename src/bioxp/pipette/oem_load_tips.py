"""ControlLib.loadTips:9684-9917 (not newloadTips), under a finite owner.

The callback boundary deliberately keeps the source machine tray selection, camera
inspection and pipette query with their existing owners. No hardware transport is
created here. The OEM ignores many primitive return values but not exceptions.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Mapping

from .oem_calibration import _Sequence
from ..oem_compat.position_table import tip_group_well_ids, well_id_from_label

Result = dict[str, Any]


@dataclass(frozen=True)
class LoadTipsBindings:
    facts: Callable[[], Mapping[str, Any]]
    query: Callable[[], Result]  # TipExist, TipMissing, tipType from queryTipStatus(-1)
    light: Callable[[], Result]  # white RGB, LED2, z stall guard=10
    stall_default: Callable[[], Result]
    tray: Callable[[int], Mapping[str, Any]]  # getTipType/tipAvailable/getTipTray
    move: Callable[[int, int, int], Result]  # scriptmoveTo(location,column,row)
    publish: Callable[[int, int], Result]
    home: Callable[[], Result]  # MoveZHome(false), source_return lost steps
    lower: Callable[[int], Result]
    lift: Callable[[int], Result]
    eject: Callable[[bool], Result]  # ejectAllTips(checkMissingTip,true)
    tip_state: Callable[[Mapping[str, Any]], Result]
    remove: Callable[[int, int], Result]  # ClassTipTray.removeTip(group,-1)
    load_type: Callable[[int], Result]  # ClassPipetteCollection.loadTip (host metadata)
    inspect: Callable[[int, int], Result]  # checkTips(tray,group) source bool
    current_max: Callable[[], Result]
    log_missing: Callable[[], Result]
    error_event: Callable[[str], Result]
    unlock_door: Callable[[], Result]
    start_mode: Callable[[], int]
    overpress_checked_has_value: Callable[[], bool]
    camera_ready: Callable[[], bool]


def load_tips(tip_type: int, bindings: LoadTipsBindings, *, force_new_tip: bool = False) -> Result:
    """Exact old loadTips branch/retry structure; false checkTips is a return.

    Tip group 24 is the source sentinel. The query/ejection loops are retained
    with their original limits (including the source's unbounded outer search).
    """
    b = bindings
    seq = _Sequence("ControlLib.loadTips:9684-9917")
    def step(name, call):
        # ControlLib ignores primitive bools in this method. Exceptions still
        # unwind and retain the already-executed prefix.
        return seq.step(name, call, allow_false=True)
    status = step("queryTipStatus", b.query)
    step("setColor/led2On/stallGuard10", b.light)
    if status["tip_type"] == tip_type and not force_new_tip:
        seq.result["source_return"] = True
        return seq.result
    if status["tip_exists"] and status["tip_type"] != 201 and (
            force_new_tip or status["tip_type"] != tip_type):
        if force_new_tip:
            step("TipLoaded=true", lambda: b.tip_state({"tip_loaded": True}))
        step("scriptmoveTo.waste", lambda: b.move(6, 0, 0))
        step("updateLocation.waste", lambda: b.publish(6, 0))
        step("ejectAllTips", lambda: b.eject(True))
        if force_new_tip:
            step("TipLoaded=false", lambda: b.tip_state({"tip_loaded": False}))
    found = False
    selected_tray = 0
    selected_group = 0
    for index in range(4):
        selected_tray = index
        tray = b.tray(index)
        if tray["tip_type"] != tip_type or not tray["tip_available"]:
            continue
        found = True
        selected_group = tray["next_group"]
        if selected_group == 24:
            continue
        attempt = 0
        while True:
            selected_group = b.tray(index)["next_group"]
            # Source tipLocation enumerates A1,B1,A2,B2,...
            well = f"{'AB'[selected_group % 2]}{selected_group // 2 + 1}"
            well_id = well_id_from_label(well)
            row, column = divmod(well_id, 12)
            location = tray["location"]
            step("MoveZHome.before", b.home)
            step("scriptmoveTo.tray", lambda: b.move(location, column, row))
            # First Enum.Parse receives only the numeric substring, then the
            # second TryParse receives the full tipLocation label.
            step("updateLocation.numeric", lambda: b.publish(location, column + 1))
            step("updateLocation.parsed", lambda: b.publish(location, well_id))
            z_retries = 5
            missing_queries = 0
            while True:
                step("lowerPipette", lambda: b.lower(location))
                step("liftPipette", lambda: b.lift(location))
                status = step("queryTipStatus", b.query)
                inner = 0
                while status["tip_missing"]:
                    step("logErrorMessage", b.log_missing)
                    status = step("queryTipStatus", b.query)
                    missing_queries += 1
                    if status["tip_missing"] and missing_queries > 2:
                        missing_queries = 0
                        step("lowerPipette.retry", lambda: b.lower(location))
                        step("liftPipette.retry", lambda: b.lift(location))
                        status = step("queryTipStatus", b.query)
                    inner += 1
                    if inner > 3:
                        step("removeTip.missing", lambda: b.remove(index, selected_group))
                        break
                if status["tip_exists"] and status["tip_missing"]:
                    step("TipLoaded=true", lambda: b.tip_state({"tip_loaded": True}))
                    if b.start_mode() != 3:
                        step("scriptmoveTo.waste", lambda: b.move(6, column, row))
                        step("updateLocation.waste", lambda: b.publish(6, 0))
                    step("ejectAllTips.partial", lambda: b.eject(False))
                    step("TipLoaded=false", lambda: b.tip_state({"tip_loaded": False}))
                    if attempt > 1:
                        step("errorEvent", lambda: b.error_event("Load tip failed"))
                        raise RuntimeError("Load tip failed")
                lost = step("MoveZHome.after", b.home)["source_return"]
                if abs(lost) <= 250:
                    break
                if z_retries == 5:
                    z_retries = 0
                if z_retries >= 3:
                    step("errorEvent", lambda: b.error_event("Tips are not available"))
                    if not b.overpress_checked_has_value():
                        step("unlockDoor", b.unlock_door)
                z_retries += 1
                attempt += 1
                if z_retries >= 3:
                    break
            if not status["tip_missing"] or selected_group >= 23:
                break
        if not status["tip_missing"]:
            step("loadTip.metadata", lambda: b.load_type(tip_type))
            step("removeTip.loaded", lambda: b.remove(index, selected_group))
            break
    if status["tip_missing"]:
        step("errorEvent", lambda: b.error_event("Tips are not available"))
        raise RuntimeError("Tips are not available")
    step("setStallGuard.default", b.stall_default)
    if not found:
        step("errorEvent", lambda: b.error_event("The required tip is not loaded!"))
        raise RuntimeError("The required tip is not loaded!")
    step("TipDirty=false/TipLoaded=true/TipLocation=-1", lambda: b.tip_state(
        {"tip_dirty": False, "tip_loaded": True, "tip_location": -1}))
    inspected = True
    if b.camera_ready():
        inspected = step("checkTips", lambda: b.inspect(selected_tray, selected_group))["source_return"]
        if not inspected:
            step("scriptmoveTo.waste", lambda: b.move(6, 0, 0))
            step("updateLocation.waste", lambda: b.publish(6, 0))
            step("ejectAllTips.inspection", lambda: b.eject(False))
            step("TipLoaded=false", lambda: b.tip_state({"tip_loaded": False}))
    step("setZaxisCurrentmax", b.current_max)
    seq.result.update(source_return=inspected, selected_tray=selected_tray,
                      selected_group=selected_group,
                      selected_wells=(tip_group_well_ids(selected_group)
                                      if selected_group < 24 else None))
    return seq.result
