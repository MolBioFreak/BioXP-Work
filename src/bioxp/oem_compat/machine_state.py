from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

PSEUDO_Z_HOME_HIGH = 500
PSEUDO_Z_HOME_LOW = 65000
GRIPPER_SAFE_MOVE_HEIGHT = 18000
FAST_ACC_MIN_STEPS = 10000
OVERPRESS = 4030
BIO_SECURITY_COVER = "BIO_SECURITY_COVER"
UNKNOWN_TIP_VALUES = {None, "", "UNKNOWN", 201}


@dataclass(frozen=True)
class OemDefaultParameters:
    """Pure model of BioXPCommonLib.DefaultParameters relevant to path planning."""

    pseudo_z_home: int = PSEUDO_Z_HOME_LOW

    def force_to_high_home(self) -> "OemDefaultParameters":
        return OemDefaultParameters(pseudo_z_home=PSEUDO_Z_HOME_HIGH)

    def gantry_load(self, *, tiploaded: Any = None, plateloaded: Any = None) -> "OemDefaultParameters":
        # OEM: if tiploaded.HasValue && tiploaded != UNKNOWN => 500.
        if tiploaded not in UNKNOWN_TIP_VALUES:
            return OemDefaultParameters(pseudo_z_home=PSEUDO_Z_HOME_HIGH)
        # OEM: else if !plateloaded.HasValue => 65000.
        if plateloaded is None or plateloaded == "":
            return OemDefaultParameters(pseudo_z_home=PSEUDO_Z_HOME_LOW)
        # OEM: BIO_SECURITY_COVER preserves low home.
        if str(plateloaded).upper() == BIO_SECURITY_COVER:
            return OemDefaultParameters(pseudo_z_home=PSEUDO_Z_HOME_LOW)
        # OEM: all other known plate loads set high home.
        return OemDefaultParameters(pseudo_z_home=PSEUDO_Z_HOME_HIGH)

    def to_payload(self) -> dict[str, Any]:
        return {
            "schema_version": "bioxp.oem_default_parameters.v1",
            "pseudo_z_home": self.pseudo_z_home,
            "pseudo_z_home_high": PSEUDO_Z_HOME_HIGH,
            "pseudo_z_home_low": PSEUDO_Z_HOME_LOW,
            "gripper_safe_move_height": GRIPPER_SAFE_MOVE_HEIGHT,
            "fast_acc_min_steps": FAST_ACC_MIN_STEPS,
            "overpress": OVERPRESS,
            "source_anchor": "DefaultParameters.cs:47-84",
        }


@dataclass(frozen=True)
class OemMachineState:
    """Minimal OEM MachineStatus/control flags needed by scriptmoveTo path planning."""

    current_location_id: str | None = None
    current_well_id: str | None = None
    current_x: int = 0
    current_y: int = 0
    current_z: int = 0
    tip_loaded: bool = False
    tip_dirty: bool = False
    tip_location: int = -1
    clean_path: bool = False
    device_type: str = ""
    axis_confirmed: Mapping[str, bool] = field(default_factory=dict)
    default_parameters: OemDefaultParameters = field(default_factory=OemDefaultParameters)

    @staticmethod
    def _norm_axis(axis: str) -> str:
        text = str(axis).strip().lower()
        return {"g": "gripper", "grip": "gripper", "motorgrip": "gripper"}.get(text, text)

    def confirm_axis(self, axis: str) -> bool:
        return bool(dict(self.axis_confirmed).get(self._norm_axis(axis), False))

    def with_force_to_high_home(self) -> "OemMachineState":
        return self.with_default_parameters(self.default_parameters.force_to_high_home())

    def with_gantry_load(self, *, tiploaded: Any = None, plateloaded: Any = None) -> "OemMachineState":
        return self.with_default_parameters(self.default_parameters.gantry_load(tiploaded=tiploaded, plateloaded=plateloaded))

    def with_default_parameters(self, params: OemDefaultParameters) -> "OemMachineState":
        return OemMachineState(
            current_location_id=self.current_location_id,
            current_well_id=self.current_well_id,
            current_x=self.current_x,
            current_y=self.current_y,
            current_z=self.current_z,
            tip_loaded=self.tip_loaded,
            tip_dirty=self.tip_dirty,
            tip_location=self.tip_location,
            clean_path=self.clean_path,
            device_type=self.device_type,
            axis_confirmed=dict(self.axis_confirmed),
            default_parameters=params,
        )

    @classmethod
    def from_query(cls, **kwargs: Any) -> "OemMachineState":
        axis_confirmed = dict(kwargs.pop("axis_confirmed", {}) or {})
        gripper_confirmed = kwargs.pop("gripper_confirmed", None)
        if gripper_confirmed is not None:
            axis_confirmed["gripper"] = bool(gripper_confirmed)
        pseudo_z_home = kwargs.pop("pseudo_z_home", None)
        params = OemDefaultParameters(int(pseudo_z_home)) if pseudo_z_home is not None else OemDefaultParameters()
        return cls(axis_confirmed=axis_confirmed, default_parameters=params, **kwargs)

    def to_payload(self) -> dict[str, Any]:
        return {
            "schema_version": "bioxp.oem_machine_state.v1",
            "current_location_id": self.current_location_id,
            "current_well_id": self.current_well_id,
            "current_position": {"x": self.current_x, "y": self.current_y, "z": self.current_z},
            "tip_loaded": self.tip_loaded,
            "tip_dirty": self.tip_dirty,
            "tip_location": self.tip_location,
            "clean_path": self.clean_path,
            "device_type": self.device_type,
            "axis_confirmed": dict(self.axis_confirmed),
            "gripper_confirmed": self.confirm_axis("gripper"),
            "default_parameters": self.default_parameters.to_payload(),
            "source_anchors": ["DefaultParameters.cs:47-84", "ClassControlInterface.cs:3734-4014"],
        }
