"""OEM PrepareToRunJob body, hosted by the existing workflow prepare hook.

No worker, device connection, claim owner or inventory store lives here. Native
calls are finite existing operation names; the composing owner must join their
canonical children. A new logical constructor is never a historical repair.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from collections import Counter
from dataclasses import dataclass
from typing import Any, Callable, Mapping

from .protocols.runtime_state import ProtocolSourceModel, SourceTray, SourceWell

SOURCE = "BioXPMainWindow.PrepareToRunJob:1588-1808"
INSPECTION_ITEMS = {0: "StripInspection", 6: "CoverInspection", 7: "TroughInspection",
                    8: "StripWellHandleInspection", 9: "PurificationInspection",
                    10: "OutputPlateInspection", 17: "TroughInspectionEmpty"}
BARCODE_LETTERS = dict(zip((3, 5, 7, 15, 11, 13, 9, 31, 17, 19, 23, 25, 29, 27), "ABCDEFGHIJKLMN"))


def _json_copy(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(k): _json_copy(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_copy(v) for v in value]
    if value is None or type(value) in (str, bool, int, float):
        return value
    raise TypeError(f"Not captured JSON: {type(value).__name__}")


def _digest(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def construct_new_machine_source_model() -> ProtocolSourceModel:
    """Explicit NEW ClassMachineStatus logical projection (400–454).

    This is not ClassVirtualBioXP (whose constructor has all T50), not resetStatus,
    and not evidence that retained physical state is empty or freshly constructed.
    ClassWell's C# bool default is false; only the hotel sets empty=true.
    Physical-loaded/carried/allow-to-stop facts remain unknown.
    """
    def wells(count: int, capacity: float, empty: bool = False) -> list[SourceWell]:
        return [SourceWell(None, 0.0, capacity, empty, None) for _ in range(count)]
    return ProtocolSourceModel(
        trays={name: SourceTray(name, location, wells(96, capacity))
               for name, location, capacity in (("POOL_PLATE", 23, 200.0), ("OUTPUT_PLATE", 21, 200.0),
                                                ("REAGENT_PLATE", 3, 1081.0), ("TROUGH", 16, 200.0))},
        strips=[SourceTray(name, 11 + i, wells(8, 500.0))
                for i, name in enumerate(("STRIP_ONE", "STRIP_TWO", "STRIP_THREE", "STRIP_FOUR"))],
        tip_trays=[SourceTray(str(i), location, wells(96, 200.0, i == 4),
                              200 if i == 3 else 50, False)
                   for i, location in enumerate((7, 8, 9, 10, 15))],
    )


def _local(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _children(node: ET.Element) -> dict[str, ET.Element]:
    return {_local(child.tag): child for child in node}


def _xml_scalar(node: ET.Element) -> Any:
    value = (node.text or "").strip()
    kind = next((v.rsplit(":", 1)[-1] for k, v in node.attrib.items() if _local(k) == "type"), None)
    if kind == "boolean" or value in ("true", "false"):
        if value not in ("true", "false"):
            raise ValueError("Invalid source XML boolean")
        return value == "true"
    if kind in ("int", "long", "short"):
        return int(value)
    if kind in ("double", "float", "decimal"):
        return float(value)
    return value


def capture_preparation_settings(snapshot: Any, *, operation_parameters: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Read captured immutable bundle bytes, not paths/device or guessed defaults.

    The current host policy may export only a subset. Source properties absent
    from it retain immutable operation values or proven C# constructor values.
    Per-property origins and effective-policy digest make that distinction explicit.
    """
    # ClassBioXPSettings1649-1661 -> loadConfig1669 -> setter branches2558-2602
    # and2646-2672; do not mistake an omitted host projection for missing source.
    constructor_policy = {"DeckInspection": False, "InspectionLogOnly": True,
                          "ThermalFault": False, "CheckCamera": False,
                          "ScreenResolutionHigh": False}
    policy: dict[str, Any] = dict(constructor_policy)
    policy.update(_json_copy(snapshot.operation_parameters))
    supplied_policy = _json_copy(operation_parameters) if operation_parameters is not None else {}
    policy.update(supplied_policy)
    policy_origins = {key: ("captured_current_host_policy" if key in supplied_policy else
                           "immutable_operation_parameters" if key in snapshot.operation_parameters else
                           "ClassBioXPSettings_constructor1649-1661") for key in policy}
    config = snapshot.records["appdata/config.xml"]
    inspection = snapshot.records["appdata/InspectionSettings.xml"]
    offsets = snapshot.config_sections["offsets"]
    # Reuse the validated snapshot, not a second config/path reader.
    camera = next((row for row in snapshot.position_table if row["name"] == "CAMERA_OFFSET"), None)
    if camera is None:
        raise ValueError("source_artifact_missing:PositionTable/CAMERA_OFFSET")
    values = dict(policy)
    for name in ("DeckInspection", "InspectionLogOnly", "ScreenResolutionHigh", "ThermalFault"):
        if type(values.get(name)) is not bool:
            raise ValueError(f"source_setting_missing_or_invalid:{name}")
    for name, attr in (("CameraXOffset", "x"), ("CameraYOffset", "y"), ("CameraZOffset", "zLow")):
        values[name] = int(camera[attr])
    # These are explicit source constructor defaults, overridden by captured XML.
    for name, default in (("LowerHandlePosition", 239.0), ("UpperHandlePosition", 236.0)):
        values[name] = float(offsets.get(name, default))
    profile_name = "Settings3250" if values["ScreenResolutionHigh"] else "Settings3200"
    profiles = _children(ET.fromstring(inspection.raw_bytes))
    profile = profiles.get(profile_name)
    if profile is None:
        raise ValueError(f"source_artifact_missing:InspectionSettings/{profile_name}")
    cameras = {}
    for entry in profile:
        fields = _children(entry)
        key = (fields["Key"].text or "").strip()
        item = _children(fields["Value"])
        parameters_node = item["Parameters"]
        params = None if parameters_node.attrib.get("{http://www.w3.org/2001/XMLSchema-instance}nil") == "true" else {}
        if params is not None:
            for pair in parameters_node:
                parameter = _children(pair)
                params[(parameter["Key"].text or "").strip()] = _xml_scalar(parameter["Value"])
        cameras[key] = {"Exposure": float(item["Exposure"].text or ""),
                        "Gain": float(item["Gain"].text or ""),
                        **{name: _xml_scalar(item[name]) for name in ("LED1", "LED2", "LED3")},
                        "Parameters": params}
    values["InspectionSettings"] = cameras
    return {"settings": values, "source_identity": {
        "acquisition_id": snapshot.acquisition_id, "lock_sha256": snapshot.lock_sha256,
        "config_sha256": hashlib.sha256(config.raw_bytes).hexdigest(),
        "inspection_sha256": hashlib.sha256(inspection.raw_bytes).hexdigest(),
        "operation_parameters_sha256": _digest(policy), "inspection_profile": profile_name,
        "operation_parameter_origins": policy_origins,
        "captured_host_policy_sha256": _digest(supplied_policy),
    }}


def capture_preparation_input(*, settings: Mapping[str, Any], source_model: ProtocolSourceModel,
                              requirements: Mapping[str, Any], source_identity: Mapping[str, Any]) -> dict[str, Any]:
    """Detach actual selected inputs. No name, tray or missing-setting defaults."""
    if set(requirements) != {"JobName", "OutputPlateRequired", "TroughRequired"}:
        raise ValueError("Exact source script requirements required")
    if requirements["JobName"] is not None and type(requirements["JobName"]) is not str:
        raise ValueError("JobName must be source string or source null")
    for name in ("OutputPlateRequired", "TroughRequired"):
        if type(requirements[name]) is not bool:
            raise ValueError(f"{name} must be source bool")
    captured = {"source_settings": _json_copy(settings), "source_model": source_model.to_payload(),
                "requirements": _json_copy(requirements), "source_identity": _json_copy(source_identity),
                "source_anchor": SOURCE}
    captured["capture_sha256"] = _digest(captured)
    return captured


@dataclass(frozen=True)
class PreparationBindings:
    """Trusted composition only. Callbacks are never taken from prepared JSON."""
    execute_native: Callable[[str, Mapping[str, Any], Any], Mapping[str, Any]]
    execute_control: Callable[[str, Mapping[str, Any], Any], Mapping[str, Any]]
    inspect_image: Callable[[str, Mapping[str, Any], Any], Any]
    sleep: Callable[[float], None]


class PreparationChildFailure(RuntimeError):
    def __init__(self, operation: str, result: Any, evidence: list[dict[str, Any]]):
        super().__init__(f"Preparation native child failed: {operation}")
        self.operation = operation
        self.result = result
        self.preparation_evidence = copy.deepcopy(evidence)


class _PrepareBody:
    def __init__(self, captured: Mapping[str, Any], bindings: PreparationBindings, state: Any):
        self.settings = captured["source_settings"]
        self.requirements = captured["requirements"]
        self.bindings, self.state = bindings, state
        self.evidence: list[dict[str, Any]] = []
        self.inspections: list[dict[str, Any]] = []

    def native(self, operation: str, **arguments: Any) -> Mapping[str, Any]:
        return self.call(self.bindings.execute_native, operation, arguments)

    def control(self, operation: str, **arguments: Any) -> Mapping[str, Any]:
        return self.call(self.bindings.execute_control, operation, arguments)

    def call(self, callback: Callable, operation: str, arguments: Mapping[str, Any]) -> Mapping[str, Any]:
        try:
            result = callback(operation, arguments, self.state)
        except Exception as exc:
            self.evidence.append({"operation": operation, "arguments": dict(arguments),
                                  "exception_type": type(exc).__name__, "exception": str(exc)})
            # Preserve native exception identity/attributes for canonical finalization.
            try:
                setattr(exc, "preparation_evidence", copy.deepcopy(self.evidence))
            except (AttributeError, TypeError):
                pass
            raise
        self.evidence.append({"operation": operation, "arguments": dict(arguments), "result": copy.deepcopy(result)})
        if not isinstance(result, Mapping) or result.get("ok") is not True:
            raise PreparationChildFailure(operation, result, self.evidence)
        return result

    def inspect(self, operation: str, **arguments: Any) -> Any:
        value = self.bindings.inspect_image(operation, arguments, self.state)
        self.evidence.append({"inspection_method": operation, "arguments": dict(arguments), "value": copy.deepcopy(value)})
        return value

    def pause(self, seconds: float) -> None:
        self.bindings.sleep(seconds)

    def force_high(self) -> None:
        self.native("preparation_force_high_home")

    def rgb(self) -> None:
        self.native("pipette_color", r=255, g=255, b=255)

    def camera(self, item: int) -> Mapping[str, Any]:
        profile = self.settings["InspectionSettings"][INSPECTION_ITEMS[item]]
        exposure = profile["Exposure"]
        if exposure != 1000.0:
            self.control("preparation_camera_exposure", value=None if exposure == 0.0 else exposure)
        for i in (1, 2, 3):
            self.control("preparation_led", channel=i, on=profile[f"LED{i}"])
        return profile["Parameters"]

    def off(self) -> None:
        for i in (1, 2, 3):
            self.control("preparation_led", channel=i, on=False)

    def move(self, location: int, x: int, y: int, *, publish: int | None = None) -> None:
        self.native("pipette_waste", location=location, offset_x=x, offset_y=y, run_in_parallel=True)
        if publish is not None:
            self.location(publish)

    def location(self, location: int) -> None:
        self.native("pipette_location", destination=location, well=0)

    def z(self, z: int) -> None:
        self.native("pipette_move_z", value=z)

    def snapshot(self, name: str) -> None:
        self.native("pipette_snapshot", name=name)

    def match(self, template: str) -> Any:
        return self.inspect("matchPattern", template=template, method=5)

    def purification(self) -> str:
        self.force_high()
        self.rgb()
        params = self.camera(9)
        self.move(0, 2369 + self.settings["CameraXOffset"], self.settings["CameraYOffset"], publish=0)
        self.z(17395 + self.settings["CameraZOffset"])
        self.pause(.7)
        found = self.inspect("checkPurificationStation")
        if found and self.settings["ScreenResolutionHigh"]:
            self.native("pipette_move_xy", x=34000, y=500)
            self.location(0)
            self.pause(.7)
            found = self.match("purificationTray.jpg")[0] > params["threshold"]
        if found:
            self.snapshot("MS_plate")
            if not self.settings["InspectionLogOnly"]:
                # Literal early return bypasses AllLEDOff.
                return "POOL_PLATE_IN_MS"
        self.off()
        return "OK"

    def output(self) -> str:
        required = self.requirements["OutputPlateRequired"]
        if not self.settings["ScreenResolutionHigh"]:
            self.force_high()
            if not required:
                return "OK"
            self.rgb()
            params = self.camera(10)
            self.move(1, 1895 + self.settings["CameraXOffset"], -710 + self.settings["CameraYOffset"], publish=1)
            self.z(17395 + self.settings["CameraZOffset"])
            self.pause(.7)
            found = self.inspect("checkOutputPlate", threshold=params["threshold"], pixelCount=params["pixelCount"])
            status = "OK"
            if not found:
                self.snapshot("output_plate_missing")
                if not self.settings["InspectionLogOnly"]:
                    status = "OUTPUT_PLATE_MISLOCATE"
            self.off()
            return status
        if not required:
            return "OK"
        self.camera(6)
        self.move(1, 3435 + self.settings["CameraXOffset"], -2772 + self.settings["CameraYOffset"], publish=17)
        self.pause(.7)
        cover = self.match("cover.jpg")[0]
        self.camera(10)
        self.z(15000 + self.settings["CameraZOffset"])
        output = self.match("output.jpg")[0]
        foil = self.match("outputw_foil.jpg")[0]
        empty = self.match("output_empty.jpg")[0]
        maximum = max(cover, output, foil, empty)
        # Preserve the source precedence: output branch has no >.7 gate.
        selected = 1 if maximum == cover and maximum > .7 else 2 if maximum == output or (maximum == foil and maximum > .7) else 0
        if selected != 2:
            self.snapshot("recovery_PlateLOC_OC missing")
        self.off()
        return "OK" if selected == 2 else "OUTPUT_PLATE_MISLOCATE"

    def trough(self) -> str:
        self.force_high()
        if not self.requirements["TroughRequired"]:
            return "OK"
        self.rgb()
        high = self.settings["ScreenResolutionHigh"]
        params = self.camera(17 if high else 7)
        self.move(16, self.settings["CameraXOffset"], 7991 + self.settings["CameraYOffset"] - (10000 if high else 0))
        self.z(-7841 + self.settings["CameraZOffset"])
        if not high:
            self.move(16, self.settings["CameraXOffset"], 0)
        self.location(16)
        self.pause(1.0 if high else .5)
        if high:
            self.z(40000 + self.settings["CameraZOffset"])
            missing = self.match("EmptyTrough.jpg")[0] > params["threshold"]
        else:
            missing = not self.inspect("checkTrough", threshold=params["threshold"])
        status = "OK"
        if missing:
            self.snapshot("trough_missing")
            if not self.settings["InspectionLogOnly"]:
                status = "TROUGH_MISLOCATED"
        self.off()
        return status

    def handles(self) -> str:
        self.force_high()
        status = "OK"
        try:
            self.rgb()
            params = self.camera(8)
            equalize = params["equalize"]  # Source reads this even on high-res branch.
            self.z(300)
            self.move(11, -3222 + self.settings["CameraXOffset"], -3198 + self.settings["CameraYOffset"], publish=11)
            if self.settings["ScreenResolutionHigh"]:
                self.pause(1.0)
                lower = self.match("LowerHandle.jpg")
                good = lower[0] > .8 and abs(lower[1] - self.settings["LowerHandlePosition"]) < 20.0
                if not good:
                    status = "STRIP_HANDLE_MISLOCATED"
                    self.snapshot("lower_strip_variance_issue")
                self.move(11, -3222 + self.settings["CameraXOffset"], 17675 + self.settings["CameraYOffset"], publish=11)
                self.pause(1.0)
                upper = self.match("UpperHandle.jpg")
                if not (upper[0] > .8 and abs(upper[1] - self.settings["UpperHandlePosition"]) < 20.0 and good):
                    status = "STRIP_HANDLE_MISLOCATED"
                    self.snapshot("upper_strip_issue")
            else:
                self.pause(1.5)
                upper_adjust = 0
                while True:
                    reading = self.inspect("findStripHandle", adjustment=upper_adjust, equalize=equalize)
                    area, upper_center = reading[0], reading[1]
                    if 2500 < area < 4000 and upper_center != 0:
                        break
                    upper_adjust += 30
                    if upper_adjust > 91:
                        self.snapshot("upper_strip_handle_not_closed")
                        return "STRIP_HANDLE_MISLOCATED"
                self.move(11, -3222 + self.settings["CameraXOffset"], 17675 + self.settings["CameraYOffset"], publish=11)
                self.pause(1.5)
                lower_adjust = 0
                while True:
                    reading = self.inspect("findStripHandle", adjustment=lower_adjust, equalize=equalize)
                    area, lower_center = reading[0], reading[1]
                    if 2500 < area < 4000 and lower_center != 0:
                        break
                    lower_adjust -= 30
                    if lower_adjust < -91:
                        self.snapshot("lower_strip_handle_not_closed")
                        return "STRIP_HANDLE_MISLOCATED"
                average = math.trunc((lower_center + upper_adjust + upper_center - lower_adjust) / 2)
                if abs(lower_center - average) > 20:
                    status = "STRIP_HANDLE_MISLOCATED"
                    self.snapshot("lower_strip_variance_issue")
                    self.move(11, -3222 + self.settings["CameraXOffset"], -3198 + self.settings["CameraYOffset"], publish=11)
                    self.pause(1.0)
                    self.snapshot("upper_strip_variance_issue")
        except Exception as exc:
            # ControlLib has this catch; preserve it visibly, don't fabricate CV.
            self.evidence.append({"source_catch": "ControlLib.inspectStripHandle", "exception_type": type(exc).__name__,
                                  "exception": str(exc), "source_status_preserved": status})
        self.off()
        return status

    def strips(self) -> str:
        self.force_high()
        strips = self.state.source_model.strips
        if not any(t.strip_color is not None and t.strip_color != "X" for t in strips):
            return "OK"
        self.rgb()
        params = self.camera(0)
        self.move(11, -3198 + self.settings["CameraXOffset"], 4264 + self.settings["CameraYOffset"], publish=11)
        reads = [self.inspect("inspectStrip", threLow=params["threLow"], threHigh=params["threHigh"], equalize=params["equalize"]) for _ in range(5)]
        counts = Counter(reads)
        selected = max(reads, key=lambda value: counts[value])  # stable source GroupBy/OrderBy tie
        actual = [BARCODE_LETTERS.get((selected >> shift) & 255, "Empty") for shift in (24, 16, 8, 0)]
        missing = [t.strip_color for t in strips if t.strip_color is not None and t.strip_color != "X" and t.strip_color not in actual]
        status = "OK"
        if missing:
            status = "STRIP_MISLOCATED"
            self.snapshot("strip_well_missing")
        else:
            for strip in strips:
                if strip.strip_color is not None:
                    for i, color in enumerate(actual):
                        if strip.strip_color == color:
                            strip.location = 11 + i  # source last duplicate wins
        if self.settings["InspectionLogOnly"]:
            self.snapshot("strip_well_image_log")
            status = "OK"
        self.off()
        return status

    def run(self) -> dict[str, Any]:
        if not self.settings["DeckInspection"]:
            return {"ok": True, "source_return": "OK", "source_noop": "DeckInspection=false",
                    "delivery_attempted": False, "inspection_issues": [], "preparation_evidence": []}
        if len(self.state.source_model.strips) != 4:
            raise ValueError("source_preparation_requires_actual_four_strip_model")
        confirmed = self.native("confirm_gripper").get("source_return")
        if type(confirmed) is not bool:
            raise RuntimeError("source_authority_missing:confirmAxis_gripper_return")
        if not confirmed:
            self.native("home_gripper")
        for name, function in (("purification", self.purification), ("recovery", self.output),
                               ("trough", self.trough), ("handle", self.handles), ("wells", self.strips)):
            status = function()
            effective = "OK" if self.settings["InspectionLogOnly"] else status
            self.inspections.append({"stage": name, "source_status": status, "effective_status": effective})
        self.inspections.append({"stage": "tips", "source_status": "OK", "effective_status": "OK",
                                 "source_noop": "BioXPMainWindow:1736 literal OK; no tip inspection"})
        self.native("park_gantry")
        issues = [row["stage"] for row in self.inspections if row["effective_status"] != "OK"]
        return {"ok": True, "source_return": "OK", "delivery_attempted": True,
                "inspection_issues": issues, "inspection_decision_required": bool(issues),
                "thermal_fault_warning": self.settings["ThermalFault"],
                "inspections": self.inspections, "preparation_evidence": self.evidence,
                "source_model": self.state.source_model.to_payload(), "source_anchor": SOURCE}


def selected_diagnostic_job_trays(selected: Mapping[str, Any]) -> dict[str, Any]:
    """Validate actual authored no-liquid job metadata, without allocating stock.

    The caller has explicitly selected a NEW job load, not retained preparation.
    Empty well collections here are declarations, never measured empty trays.
    """
    if set(selected) != {"no_liquid", "tip_trays", "strips", "trays"} or selected["no_liquid"] is not True:
        raise ValueError("source_job_load_requires_declared_no_liquid_job")
    def tray(row: Mapping[str, Any]) -> SourceTray:
        if "wells" in row:
            raise ValueError("source_job_metadata_must_not_supply_prepared_wells")
        return SourceTray.from_payload({**_json_copy(row), "wells": []})
    tips = [tray(row) for row in selected["tip_trays"]]
    strips = [tray(row) for row in selected["strips"]]
    trays = {name: tray(row) for name, row in selected["trays"].items()}
    if len(tips) != 5 or any(type(t.tip_type) is not int or t.tip_type not in (50, 200, 201) for t in tips):
        raise ValueError("source_job_load_requires_selected_five_tray_metadata")
    if len(strips) != 4:
        raise ValueError("source_job_load_requires_selected_four_strip_metadata")
    if set(trays) != {"POOL_PLATE", "OUTPUT_PLATE", "REAGENT_PLATE"}:
        raise ValueError("source_job_load_requires_selected_plate_metadata")
    return {"tip_trays": tips, "strips": strips, "trays": trays}


def load_declared_diagnostic_job(state: Any, *, selected: Mapping[str, Any], execute_native: Callable) -> list[dict[str, Any]]:
    """Actual resetStatus -> selected-job update prefix for a no-liquid job.

    ClassWellCollection495-511 allocates NEW wells during reset NOW. This is
    not a MachineStatus constructor or backfill of missing historical stock.
    No tip-loaded, gripper/carry, history, pressure or stopping fields change.
    """
    job = selected_diagnostic_job_trays(selected)
    results = reset_loaded_job_tip_inventory(state, execute_native=execute_native,
        selected_trays=job["tip_trays"])
    # MachineStatus672-685 resets these three plates and four strips. Source
    # MainWindow3176-3180 then applies the explicitly selected job metadata.
    for name, tray in job["trays"].items():
        capacity = 1081.0 if name == "REAGENT_PLATE" else 200.0
        tray.wells = [SourceWell(None, 0.0, capacity, False, None) for _ in range(96)]
        state.source_model.trays[name] = tray
    for tray in job["strips"]:
        tray.wells = [SourceWell(None, 0.0, 500.0, False, None) for _ in range(8)]
    state.source_model.strips = job["strips"]
    state.record_event("source_job_loaded", detail={
        "source_anchor": "MainWindow3175-3180/MachineStatus655-689/ClassWellCollection495-511",
        "inventory_provenance": "declared_source_job_load_logical_stock_not_camera_measured",
        "selected_job_metadata": _json_copy(selected),
    })
    return results


def reset_loaded_job_tip_inventory(state: Any, *, execute_native: Callable,
                                   selected_trays: list[SourceTray] | None = None) -> list[dict[str, Any]]:
    """Explicit source *job-load caller* reset, NEVER automatic inspection work.

    MainWindow3175 -> MachineStatus655 -> TipTray469 loads all five trays,
    including the hotel. The composing job loader supplies its actual selected
    tip metadata after updateTipInfo; this function never guesses missing types.
    Each canonical reset joins before its corresponding logical wells change.
    """
    trays = state.source_model.tip_trays if selected_trays is None else selected_trays
    if len(trays) != 5 or any(t.tip_type is None for t in trays):
        raise ValueError("source_job_load_requires_selected_five_tray_metadata")
    results = []
    for i, tray in enumerate(trays):
        result = execute_native("pipette_tip_transition", {"tray_id": i, "well_ids": [], "transition": "reset"}, state)
        results.append(dict(result))
        if result.get("ok") is not True:
            raise PreparationChildFailure("pipette_tip_transition", result, results)
        tray.wells = [SourceWell(None, 0.0, 200.0, False, None) for _ in range(96)]
        tray.tray_empty = False
        if selected_trays is not None:
            if i < len(state.source_model.tip_trays):
                state.source_model.tip_trays[i] = tray
            else:
                state.source_model.tip_trays.append(tray)
    return results


def build_preparation_inspection_reader(*, snapshot: Any, settings: Mapping[str, Any],
                                       source_model: ProtocolSourceModel, capture_image: Callable) -> Callable:
    """Bind genuine CV to each fresh source image, using existing bundle assets.

    capture_image(method,state) is a trusted shared camera leaf returning bytes or
    an array; no connection, fallback image, template search or image cache here.
    Missing selected computational dependencies reject binding before any motion.
    """
    if not settings["DeckInspection"]:
        def unused_inspection(operation: str, arguments: Mapping[str, Any], state: Any) -> Any:
            raise RuntimeError("inspection_requested_outside_source_DeckInspection_branch")
        return unused_inspection
    from .vision import oem_inspection as cv
    from .vision import oem_strips
    if settings["DeckInspection"]:
        required = ["check_purification_station"]
        required.append("match_pattern" if settings["ScreenResolutionHigh"] else "find_strip_handle")
        if any(t.strip_color is not None and t.strip_color != "X" for t in source_model.strips):
            required.append("inspect_strip")
        missing = [name for name in required if not callable(getattr(oem_strips if name in {"find_strip_handle", "inspect_strip"} else cv, name, None))]
        if missing:
            raise RuntimeError("source_preparation_cv_unimplemented:" + ",".join(missing))
    def inspect_image(operation: str, arguments: Mapping[str, Any], state: Any) -> Any:
        image = capture_image(operation, state)
        if operation == "matchPattern":
            template = snapshot.records["appdata/" + arguments["template"]].raw_bytes
            return cv.match_pattern(image, template, arguments["method"])
        if operation == "checkPurificationStation":
            return cv.check_purification_station(image)
        if operation == "checkOutputPlate":
            return cv.check_output_plate(image, arguments["threshold"], arguments["pixelCount"])
        if operation == "checkTrough":
            return cv.check_trough(image, arguments["threshold"])
        if operation == "findStripHandle":
            return oem_strips.find_strip_handle(image, arguments["adjustment"], arguments["equalize"])
        if operation == "inspectStrip":
            return oem_strips.inspect_strip(image, arguments["threLow"], arguments["threHigh"], arguments["equalize"])
        raise ValueError("Unknown finite preparation inspection")
    return inspect_image


def build_prepare_handler(*, captured: Mapping[str, Any], execute_native: Callable, execute_control: Callable,
                          inspect_image: Callable, sleep: Callable) -> Callable:
    """Return the existing ProtocolExecutor prepare(state) callback; no runner.

    Inspection UI issues are returned separately from OEM's literal OK return.
    Parent must route issues into its existing review gate, never auto-ignore.
    """
    snapshot = _json_copy(captured)
    digest = snapshot.pop("capture_sha256")
    if digest != _digest(snapshot):
        raise ValueError("Preparation capture digest mismatch")
    bindings = PreparationBindings(execute_native, execute_control, inspect_image, sleep)
    def prepare(state: Any) -> dict[str, Any]:
        body = _PrepareBody(snapshot, bindings, state)
        try:
            result = body.run()
        except Exception as exc:
            try:
                setattr(exc, "preparation_evidence", copy.deepcopy(body.evidence))
            except (AttributeError, TypeError):
                pass
            raise
        result["capture_sha256"] = digest
        return result
    return prepare
