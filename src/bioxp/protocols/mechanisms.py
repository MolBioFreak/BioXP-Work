"""Native method mechanisms on existing device owners, never HTTP or a queue.

Time is monotonic elapsed time. Ordinary pause is reached at the next action
boundary; abort/Stop interrupts a wait without implicit thermal shutdown.
"""
from math import isfinite
from time import monotonic
from collections.abc import Mapping

from .models import ProtocolActionKind as K

KINDS = {K.WAIT, K.THERMAL_SETPOINT, K.THERMAL_HOLD, K.THERMAL_PROFILE,
         K.CHILLER_SETPOINT, K.SNAPSHOT, K.CAMERA_ILLUMINATION,
         K.TIMER_START, K.TIMER_WAIT, K.BARCODE_READ}


def validate_mechanism(kind, p):
    if kind not in KINDS:
        return
    fields = {
        K.WAIT: {"seconds"}, K.TIMER_START: {"timer_id", "seconds"}, K.TIMER_WAIT: {"timer_id"},
        K.THERMAL_SETPOINT: {"bank", "target_temp_c", "fan_speed", "cool_rate_c_s", "heat_rate_c_s"}, K.CHILLER_SETPOINT: {"bank", "target_temp_c"},
        K.THERMAL_HOLD: {"bank", "target_temp_c", "duration_s", "start", "tolerance_c", "timeout_s", "fan_speed", "cool_rate_c_s", "heat_rate_c_s"},
        K.THERMAL_PROFILE: {"segments", "repeat"}, K.SNAPSHOT: set(),
        K.CAMERA_ILLUMINATION: {"channel", "on"}, K.BARCODE_READ: {"mode"},
    }
    if set(p) - fields[kind]:
        raise ValueError(f"Unsupported {kind.value} fields: {sorted(set(p) - fields[kind])}")
    def number(key, nonnegative=False):
        v = p.get(key)
        if type(v) not in (int, float) or not isfinite(v) or (nonnegative and v < 0):
            raise ValueError(f"{kind.value}.{key} requires a finite {'nonnegative ' if nonnegative else ''}number")
    if kind == K.WAIT:
        number("seconds", True)
    elif kind in {K.TIMER_START, K.TIMER_WAIT}:
        if not isinstance(p.get("timer_id"), str) or not p["timer_id"]:
            raise ValueError("timer_id requires a nonempty string")
        if kind == K.TIMER_START:
            number("seconds", True)
    elif kind == K.BARCODE_READ:
        if p.get("mode") not in {"stationary", "job_id", "reagent_id"}:
            raise ValueError("barcode_read.mode requires stationary, job_id or reagent_id")
    elif kind in {K.THERMAL_SETPOINT, K.THERMAL_HOLD, K.CHILLER_SETPOINT}:
        number("target_temp_c")
        banks = {"rc", "oc"} if kind == K.CHILLER_SETPOINT else {"nest", "lid", "pedestal"}
        if p.get("bank") not in banks:
            raise ValueError(f"{kind.value}.bank requires one of {sorted(banks)}")
        if kind != K.CHILLER_SETPOINT:
            if "fan_speed" in p and (type(p["fan_speed"]) is not int or not 0 <= p["fan_speed"] <= 255):
                raise ValueError("fan_speed requires native integer 0..255")
            if "cool_rate_c_s" in p or "heat_rate_c_s" in p:
                if p["bank"] == "pedestal":
                    raise ValueError("thermal rates support nest/lid, not pedestal")
                number("cool_rate_c_s")
                number("heat_rate_c_s")
                if not -2 <= p["cool_rate_c_s"] <= 0 or not 0 <= p["heat_rate_c_s"] <= 2:
                    raise ValueError("thermal rates require cool -2..0 and heat 0..2 C/s")
        if kind == K.THERMAL_HOLD:
            number("duration_s", True)
            if p.get("start") not in {"dispatch", "attainment"}:
                raise ValueError("thermal_hold.start requires dispatch or attainment")
            if p["start"] == "attainment":
                number("tolerance_c", True)
                number("timeout_s", True)
    elif kind == K.THERMAL_PROFILE:
        if type(p.get("repeat")) is not int or p["repeat"] < 0:
            raise ValueError("thermal_profile.repeat requires a nonnegative integer")
        if not isinstance(p.get("segments"), (list, tuple)) or not p["segments"]:
            raise ValueError("thermal_profile.segments requires a nonempty array")
        for segment in p["segments"]:
            if not isinstance(segment, Mapping):
                raise ValueError("thermal_profile segment requires an object")
            validate_mechanism(K.THERMAL_HOLD, segment)
    elif kind == K.CAMERA_ILLUMINATION:
        if type(p.get("channel")) is not int or p["channel"] not in {1, 2, 3} or type(p.get("on")) is not bool:
            raise ValueError("camera_illumination requires channel 1/2/3 and boolean on")


def build_mechanism_handlers(*, get_tester, get_camera, get_executor, save_snapshot, read_source_barcode=None):
    def owner():
        if get_executor is None:
            raise RuntimeError("workflow executor unavailable")
        return get_executor()

    def wait(seconds):
        return owner().wait_elapsed(seconds)

    def event(action, name, detail):
        owner().mechanism_event(action, name, detail)

    def setpoint(p):
        tester = get_tester()
        settings = []
        if "fan_speed" in p:
            value = tester.thermal_set_fan(p["fan_speed"], verify=True)
            settings.append({"field": "fan_speed", "requested": p["fan_speed"], "result": value})
            if value.get("ok") is not True:
                return {"ok": False, "settings": settings, "setpoint_emitted": False}
        bank = tester.THERMAL_BANK_NEST if p["bank"] == "nest" else tester.THERMAL_BANK_LID
        if "cool_rate_c_s" in p:
            value = tester.thermal_set_rates(bank, p["cool_rate_c_s"], p["heat_rate_c_s"], verify=True)
            settings.append({"field": "rates", "requested": {k: p[k] for k in ("cool_rate_c_s", "heat_rate_c_s")}, "result": value})
            if value.get("ok") is not True:
                return {"ok": False, "settings": settings, "setpoint_emitted": False}
        value = (tester.thermal_set_ped_temp(p["target_temp_c"], verify=True) if p["bank"] == "pedestal"
                 else tester.thermal_set_target_temp(bank, p["target_temp_c"], verify=True))
        return {**value, "settings": settings, "setpoint_emitted": True}

    def read_temp(p):
        tester = get_tester()
        if p["bank"] == "pedestal":
            return tester.thermal_read_temp_axis(2)
        bank = tester.THERMAL_BANK_NEST if p["bank"] == "nest" else tester.THERMAL_BANK_LID
        return tester.thermal_read_temp_gp(bank)

    def hold(action, p, identity):
        started = monotonic()
        receipt = setpoint(p)
        result = {"ok": receipt.get("ok") is True, "identity": identity,
                  "setpoint": receipt, "temperature_reached": None, "dwell_complete": False}
        if not result["ok"]:
            return result
        event(action, "setpoint_accepted", {"identity": identity, "bank": p["bank"], "target_temp_c": p["target_temp_c"]})
        if p["start"] == "attainment":
            deadline = started + p["timeout_s"]
            while True:
                if not wait(0):
                    return {**result, "ok": False, "interrupted": True}
                reading = read_temp(p)
                result["last_temperature"] = reading
                temp = reading.get("temp_c")
                if reading.get("ok") is True and type(temp) in (int, float) and isfinite(temp) and abs(temp - p["target_temp_c"]) <= p["tolerance_c"]:
                    result["temperature_reached"] = True
                    event(action, "temperature_reached", {"identity": identity, "reading": reading})
                    break
                remaining = deadline - monotonic()
                if remaining <= 0:
                    return {**result, "ok": False, "error": "temperature_attainment_timeout", "temperature_reached": False}
                if not wait(min(0.25, remaining)):
                    return {**result, "ok": False, "interrupted": True}
            dwell = p["duration_s"]
        else:
            dwell = max(0, p["duration_s"] - (monotonic() - started))
        if not wait(dwell):
            return {**result, "ok": False, "interrupted": True}
        result["dwell_complete"] = True
        event(action, "dwell_complete", {"identity": identity, "start": p["start"], "duration_s": p["duration_s"]})
        return result

    def run(action, state):
        kind, p = action.kind, action.params
        validate_mechanism(kind, p)
        if kind == K.WAIT:
            completed = wait(p["seconds"])
            return {"ok": completed, "elapsed_wait_complete": completed, "interrupted": not completed}
        if kind in {K.TIMER_START, K.TIMER_WAIT}:
            return owner().method_timer(p["timer_id"], p.get("seconds"), start=kind == K.TIMER_START)
        if kind == K.THERMAL_SETPOINT:
            receipt = setpoint(p)
            return {**receipt, "setpoint_accepted": receipt.get("ok") is True, "temperature_reached": None}
        if kind == K.CHILLER_SETPOINT:
            tester = get_tester()
            tester.chiller_activate()
            bank = tester.CHILLER_BANK_RC if p["bank"] == "rc" else tester.CHILLER_BANK_OC
            receipt = tester.chiller_set_target_temp(bank, p["target_temp_c"], verify=True)
            return {**receipt, "setpoint_accepted": receipt.get("ok") is True, "temperature_reached": None}
        if kind == K.THERMAL_HOLD:
            return hold(action, p, action.action_id)
        if kind == K.THERMAL_PROFILE:
            children = []
            for repeat in range(p["repeat"]):
                for index, segment in enumerate(p["segments"]):
                    identity = f"{action.action_id}:repeat:{repeat}:segment:{index}"
                    result = hold(action, segment, identity)
                    children.append(result)
                    event(action, "profile_segment_result", result)
                    if result["ok"] is not True:
                        return {"ok": False, "profile_complete": False, "children": children, "failed_child": identity}
            return {"ok": True, "profile_complete": True, "children": children}
        if kind == K.BARCODE_READ and p["mode"] != "stationary":
            return read_source_barcode(action, state)
        camera = get_camera()
        if kind == K.CAMERA_ILLUMINATION:
            result = camera.command_illumination(channel=p["channel"], on=p["on"])
            if camera is not get_camera() or camera.generation != result["provider_generation"]:
                raise RuntimeError("camera owner changed during illumination")
            return result
        frame = camera.capture()
        if camera is not get_camera() or camera.generation != frame.provider_generation:
            raise RuntimeError("camera owner changed during capture")
        if kind == K.BARCODE_READ:
            from ..vision.oem_inspection import scan_barcode
            value = scan_barcode(frame.content)
            return {"ok": True, "mode": "stationary", "value": value, "decoded": bool(value),
                    "producer": "host-decoder:zbar", "frame_sequence": frame.sequence,
                    "provider_generation": frame.provider_generation}
        return {"ok": True, "photo_only": True, "artifact": save_snapshot(frame, action, state)}

    return dict.fromkeys(KINDS, run)
