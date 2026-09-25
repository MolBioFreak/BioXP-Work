"""Typed PositionTable editing over the existing canonical SQLite owner.

OEM source: CommonLib/positionStruct.cs; ClassBioXPSettings.saveConfig
3932-3945, calXYZoffset, setCameraOffset/setCameraZOffset 6219-6246.
Values here are final table values, NOT raw measurement inputs to calXYZoffset
(which updates several related rows). No hardware, homes, or rebinds occur here.
"""
from __future__ import annotations

from dataclasses import replace
from datetime import datetime, timezone
from typing import Annotated, Literal
from uuid import uuid4

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .oem_machine_bundle import OemMachineSnapshot, _freeze, _thaw
from .oem_runtime_store import OEMRuntimeStore

# OEM positionStruct stores signed System.Int32, including inc_factor. These
# are representational bounds only, not invented travel/calibration limits.
OemInt32 = Annotated[int, Field(strict=True, ge=-2147483648, le=2147483647)]
PositionName = Literal[
    "LOC_MS", "LOC_OC", "LOC_TC", "LOC_RC", "LOC_BSCS", "LOC_BSC", "WASTE_BIN",
    "TECANRACK1", "TECANRACK2", "TECANRACK3", "TECANRACK4", "LOC_STRIP1", "LOC_STRIP2",
    "LOC_STRIP3", "LOC_STRIP4", "LOC_TIP_HOTEL", "LOC_TROUGH", "LOC_OC_COVER",
    "LOC_OC_COVER_STORAGE", "LOC_RC_COVER", "LOC_RC_COVER_STORAGE", "LOC_P_OC", "LOC_P_OC_PRESS",
    "LOC_P_TC", "LOC_P_TC_PRESS", "LOC_P_MS", "LOC_P_MS_PRESS", "LOC_P_RC_PRESS", "LOC_PARK",
    "LOC_GANTRY", "LOC_CHECK_POINT", "CAMERA_OFFSET", "UNKNOWN",
]


def _position_patch_schema(schema: dict) -> None:
    # Omission means unchanged; explicit null is not an OEM integer. Keep the
    # public schema aligned with the runtime validator so typed editors do not
    # seed rejected null defaults into otherwise valid partial updates.
    schema["minProperties"] = 2
    for name, field in schema.get("properties", {}).items():
        if name == "name":
            continue
        branches = field.pop("anyOf", ())
        field.update(next(branch for branch in branches if branch.get("type") != "null"))
        field.pop("default", None)


class PositionCalibrationPatch(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, json_schema_extra=_position_patch_schema)
    name: PositionName
    x: OemInt32 | None = None
    y: OemInt32 | None = None
    zLow: OemInt32 | None = None
    zDelta: OemInt32 | None = None
    inc_factor: OemInt32 | None = None

    @model_validator(mode="after")
    def nonempty_values(self):
        fields = self.model_fields_set - {"name"}
        if not fields or any(getattr(self, field) is None for field in fields):
            raise ValueError("supply at least one integer PositionTable field; omit unchanged fields")
        return self


class CalibrationSettingsPatch(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    positions: tuple[PositionCalibrationPatch, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def unique_names(self):
        if len({row.name for row in self.positions}) != len(self.positions):
            raise ValueError("duplicate PositionTable names")
        return self


class CalibrationRevision(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    schema_version: Literal["bioxp.machine_calibration.v1"] = "bioxp.machine_calibration.v1"
    revision_id: str
    baseline_lock_sha256: str
    saved_at: str
    positions: tuple[PositionCalibrationPatch, ...]


def _revision(snapshot: OemMachineSnapshot, payload: dict) -> CalibrationRevision:
    revision = CalibrationRevision.model_validate(payload)
    if revision.baseline_lock_sha256 != snapshot.lock_sha256:
        raise ValueError("calibration revision belongs to a different captured baseline")
    names = {row["name"] for row in snapshot.position_table}
    for row in revision.positions:
        if row.name not in names:
            raise ValueError(f"PositionTable row is absent from this machine: {row.name}")
    if len({row.name for row in revision.positions}) != len(revision.positions):
        raise ValueError("duplicate PositionTable names")
    return revision


def project_calibration(snapshot: OemMachineSnapshot, payload: dict | None) -> OemMachineSnapshot:
    """Pure prospective configuration. Does not bind or promote active state."""
    if payload is None:
        return snapshot
    revision = _revision(snapshot, payload)
    patches = {row.name: row.model_dump(exclude_unset=True) for row in revision.positions}
    rows = []
    for original in snapshot.position_table:
        if original["name"] not in patches:
            rows.append(original)
            continue
        row = _thaw(original)
        changes = patches[row["name"]]
        row.update(changes)
        row["zHigh"] = row["zLow"] - row["zDelta"]
        # Preserve original raw_attributes as captured evidence, not edited XML.
        row["source"] = f"user_authored_calibration:{revision.revision_id}"
        row["calibration_revision_id"] = revision.revision_id
        row["baseline_source"] = "serial_206_oem_machine_snapshot:appdata/config.xml"
        rows.append(_freeze(row))
    return replace(snapshot, position_table=tuple(rows), calibration_revision=_freeze(revision.model_dump(mode="json")))


def load_saved_calibration(snapshot: OemMachineSnapshot, store: OEMRuntimeStore) -> OemMachineSnapshot:
    """Call only at ordinary startup, before set_active_oem_machine_snapshot."""
    return project_calibration(snapshot, store.read_machine_calibration_revision(snapshot.lock_sha256))


class CalibrationSettingsService:
    def __init__(self, store: OEMRuntimeStore, active_snapshot: OemMachineSnapshot):
        self.store = store
        self.active_snapshot = active_snapshot

    def read(self) -> dict:
        snapshot = self.active_snapshot
        saved = self.store.read_machine_calibration_revision(snapshot.lock_sha256)
        prospective = project_calibration(snapshot, saved)
        active_id = None if snapshot.calibration_revision is None else snapshot.calibration_revision["revision_id"]
        saved_id = None if saved is None else saved["revision_id"]
        pending = saved_id != active_id
        from .oem_compat.position_table import load_bound_oem_position_table

        active_table = load_bound_oem_position_table(snapshot)
        saved_table = load_bound_oem_position_table(prospective)
        return {
            "schema_version": "bioxp.calibration_settings.v1",
            "baseline_lock_sha256": snapshot.lock_sha256,
            "saved_revision": saved,
            "active_revision_id": active_id,
            "saved_revision_id": saved_id,
            "pending_restart": pending,
            "application_status": "pending_restart" if pending else "bound_configuration",
            "application_semantics": "next ordinary process startup; no live refresh or hardware execution",
            "active_positions": _thaw(snapshot.position_table),
            "saved_positions": _thaw(prospective.position_table),
            "active_motion_positions": active_table.rows(),
            "saved_motion_positions": saved_table.rows(),
            "active_loader_adjustments": list(active_table.adjustment_ledger),
            "saved_loader_adjustments": list(saved_table.adjustment_ledger),
            "physical_calibration_verified": False,
            "motion_commanded": False,
        }

    def save(self, patch: CalibrationSettingsPatch) -> dict:
        # Validate even for callers using model_construct or non-HTTP callers.
        patch = CalibrationSettingsPatch.model_validate(patch.model_dump(exclude_unset=True))
        snapshot = self.active_snapshot

        def update(previous):
            prior = None if previous is None else _revision(snapshot, previous)
            merged = {} if prior is None else {
                row.name: row.model_dump(exclude_unset=True) for row in prior.positions
            }
            for row in patch.positions:
                merged.setdefault(row.name, {"name": row.name}).update(row.model_dump(exclude_unset=True))
            revision = CalibrationRevision(
                revision_id=uuid4().hex,
                baseline_lock_sha256=snapshot.lock_sha256,
                saved_at=datetime.now(timezone.utc).isoformat(),
                positions=tuple(PositionCalibrationPatch.model_validate(merged[name]) for name in sorted(merged)),
            ).model_dump(mode="json", exclude_unset=True)
            # Fill the schema default explicitly in the persisted contract.
            revision["schema_version"] = "bioxp.machine_calibration.v1"
            project_calibration(snapshot, revision)
            return revision

        committed = self.store.update_machine_calibration_revision(snapshot.lock_sha256, update)
        result = self.read()
        result["committed_revision_id"] = committed["revision_id"]
        return result
