"""OEM manual tray Set and consumed pipette operation settings; no motion.

ControlLib.cs:1370-1415,1471-1477. A Set reads controller Z once,
then writes one atomic paired revision; configuration binds at next startup.
"""
from __future__ import annotations

import re
from typing import Literal

from pydantic import BaseModel, ConfigDict, StrictBool, field_validator

from ..oem_calibration_settings import CalibrationSettingsPatch, CalibrationSettingsService
from ..runtime_state import OemRuntimeStateStore


class ManualTipTraySet(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    tray: Literal[1, 2, 3, 4]

    @field_validator("tray", mode="before")
    @classmethod
    def exact_integer(cls, value):
        if type(value) is not int:
            raise ValueError("tray must be an integer OEM tray number")
        return value


def _settings_patch_schema(schema: dict) -> None:
    schema["minProperties"] = 1
    for field in schema["properties"].values():
        field.pop("anyOf", None)
        field.pop("default", None)
        field["type"] = "boolean"


class PipetteOperationSettingsPatch(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, json_schema_extra=_settings_patch_schema)
    LogPressure: StrictBool | None = None
    CheckForStaticTipLoss: StrictBool | None = None

    def selected(self) -> dict[str, bool]:
        values = self.model_dump(exclude_unset=True)
        if not values or any(value is None for value in values.values()):
            raise ValueError("supply at least one boolean; omit unchanged settings")
        return values


def manual_tip_tray_set(request: ManualTipTraySet, service: CalibrationSettingsService, read_z) -> dict:
    """read_z uses the same provider primitive as sourcePosition/getZPosition."""
    measured = read_z()
    if type(measured) is not int:
        raise ValueError("current controller Z has no integer position")
    # Same signed Int32 representation as OEM positionStruct.zLow, not a travel policy.
    names = ("TECANRACK1", "TECANRACK2") if request.tray in (1, 2) else ("TECANRACK3", "TECANRACK4")
    patch = CalibrationSettingsPatch.model_validate({
        "positions": [{"name": name, "zLow": measured} for name in names],
    })
    result = service.save(patch)
    return {
        "schema_version": "bioxp.manual_tip_tray_set.v1", "tray": request.tray,
        "paired_positions": list(names), "measured_z_steps": measured,
        "committed_revision_id": result["committed_revision_id"],
        "saved_revision_id": result["saved_revision_id"],
        "active_revision_id": result["active_revision_id"],
        "pending_restart": result["pending_restart"],
        "application_semantics": result["application_semantics"],
        "saved_positions": [row for row in result["saved_positions"] if row["name"] in names],
        "active_positions": [row for row in result["active_positions"] if row["name"] in names],
        "motion_commanded": False,
    }


_PIPETTE_FLAGS = ("LogPressure", "CheckForStaticTipLoss")


def read_pipette_operation_settings(store: OemRuntimeStateStore) -> dict:
    current = store.operation_parameters_projection()
    return {
        "schema_version": "bioxp.pipette_operation_settings.v1",
        "runtime_values": {key: current[key] for key in _PIPETTE_FLAGS},
        "application_semantics": "committed Operation_parameters.xml projection; new source operations capture settings at construction; existing operations retain their captured settings",
    }


def save_pipette_operation_settings(store: OemRuntimeStateStore, patch: PipetteOperationSettingsPatch) -> dict:
    changes = PipetteOperationSettingsPatch.model_validate(patch.model_dump(exclude_unset=True)).selected()

    def transform(data: bytes) -> bytes:
        for key, value in changes.items():
            # Replace only the captured XML attribute value, preserving all other bytes.
            pattern = rb'(<'+key.encode()+rb'\s+'+key.encode()+rb'=")[^"]*("\s*/>)'
            data, count = re.subn(pattern, lambda match: match[1] + (b"True" if value else b"False") + match[2], data)
            if count != 1:
                raise ValueError(f"operation parameter {key} missing or ambiguous")
        return data

    transaction = store.update_operation_parameters(
        transform, writer="pipette.manual_settings", call_path="ControlLib.chkLogPressure_Click:1471-1477; source pipette inspection setting",
    )
    return {**read_pipette_operation_settings(store), "transaction_id": transaction.transaction_id}
