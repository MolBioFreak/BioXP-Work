"""Typed PositionTable editing over the existing canonical SQLite owner.

OEM source: CommonLib/positionStruct.cs; ClassBioXPSettings.saveConfig
3932-3945, calXYZoffset, setCameraOffset/setCameraZOffset 6219-6246.
Values here are final table values, NOT raw measurement inputs to calXYZoffset
(which updates several related rows). The existing owner applies/restores the
separate calibration projection in-process; no hardware, homes, or reconnects.
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


class CalibrationDecisionRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    decision: Literal["accept", "restore"]


class CalibrationComparisonPosition(BaseModel):
    name: str
    x: int
    y: int
    zLow: int
    zDelta: int
    inc_factor: int


class CalibrationMeasurement(BaseModel):
    model_config = ConfigDict(extra="allow")
    plate: str
    measured_raw_z: int
    saved_revision_id: str


class CalibrationComparison(BaseModel):
    positions: list[CalibrationComparisonPosition]
    liquid_calibration: dict
    revision_id: str | None


def comparison_values(settings: dict) -> dict:
    return {"positions": [{key: row[key] for key in ("name", "x", "y", "zLow", "zDelta", "inc_factor")}
                          for row in settings["saved_positions"]],
            "liquid_calibration": settings["saved_liquid_calibration"],
            "revision_id": settings["saved_revision_id"]}


class CalibrationRunResponse(BaseModel):
    model_config = ConfigDict(extra="allow")
    schema_version: Literal["bioxp.calibration_run.v1"] = "bioxp.calibration_run.v1"
    run_id: str
    saved_revision_id: str | None
    active_revision_id: str | None
    body_completed: bool
    decision: Literal["accept", "restore"] | None = None
    decision_status: str
    error: str | None = None
    comparison_error: str | None = None
    finalization_error: str | None = None
    before: CalibrationComparison
    after: CalibrationComparison
    measurements: list[CalibrationMeasurement] = Field(default_factory=list)


class CalibrationRevision(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    schema_version: Literal["bioxp.machine_calibration.v1"] = "bioxp.machine_calibration.v1"
    revision_id: str
    baseline_lock_sha256: str
    saved_at: str
    positions: tuple[PositionCalibrationPatch, ...]
    liquid_calibration: dict[str, str | bool] | None = None


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
    snapshot = snapshot.calibration_baseline or snapshot
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
    sections = _thaw(snapshot.config_sections)
    if revision.liquid_calibration is not None:
        liquid = revision.liquid_calibration
        sections["calibration"].update({
            "m_Liquid_Cal": liquid["calibrated"],
            "m_Liquid_Cal_date": liquid["saved_at"],
            "m_liquid_cal_reversion": liquid["reference_revision"],
        })
    return replace(snapshot, position_table=tuple(rows), config_sections=_freeze(sections),
                   calibration_revision=_freeze(revision.model_dump(mode="json")),
                   calibration_baseline=snapshot)


def load_saved_calibration(snapshot: OemMachineSnapshot, store: OEMRuntimeStore) -> OemMachineSnapshot:
    """Call only at ordinary startup, before set_active_oem_machine_snapshot."""
    return project_calibration(snapshot, store.read_machine_calibration_revision(snapshot.lock_sha256))


class CalibrationSettingsService:
    def __init__(self, store: OEMRuntimeStore, active_snapshot: OemMachineSnapshot, *, publish=None):
        self.store = store
        self.active_snapshot = active_snapshot
        self._publish = publish

    def _apply(self, revision):
        projected = project_calibration(self.active_snapshot, revision)
        if self._publish is not None:
            self._publish(projected)
        self.active_snapshot = projected

    def read(self) -> dict:
        with self.store._lock:
            return self._read()

    def _read(self) -> dict:
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
            "active_liquid_calibration": _thaw(snapshot.config_sections["calibration"]),
            "saved_liquid_calibration": _thaw(prospective.config_sections["calibration"]),
            "pending_restart": pending,
            "application_status": "pending_restart" if pending else "bound_configuration",
            "application_semantics": "in-process calibration owner; no reconnect, home or restart",
            "active_positions": _thaw(snapshot.position_table),
            "saved_positions": _thaw(prospective.position_table),
            "active_motion_positions": active_table.rows(),
            "saved_motion_positions": saved_table.rows(),
            "active_loader_adjustments": list(active_table.adjustment_ledger),
            "saved_loader_adjustments": list(saved_table.adjustment_ledger),
            "physical_calibration_verified": False,
            "motion_commanded": False,
        }

    def save(self, patch: CalibrationSettingsPatch, *, liquid_reference_revision: str | None = None,
             run_id: str | None = None, measurement: dict | None = None) -> dict:
        with self.store._lock:
            return self._save(patch, liquid_reference_revision=liquid_reference_revision,
                              run_id=run_id, measurement=measurement)

    def _save(self, patch: CalibrationSettingsPatch, *, liquid_reference_revision: str | None = None,
              run_id: str | None = None, measurement: dict | None = None) -> dict:
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
            timestamp = datetime.now(timezone.utc).isoformat()
            liquid = (None if prior is None else prior.liquid_calibration)
            if liquid_reference_revision is not None:
                # OEM adjustZ sets FluidReference; each saveConfig writes the
                # liquid-calibrated flag, reference revision and current date.
                liquid = {"calibrated": True, "saved_at": timestamp,
                          "reference_revision": liquid_reference_revision}
            revision = CalibrationRevision(
                revision_id=uuid4().hex,
                baseline_lock_sha256=snapshot.lock_sha256,
                saved_at=timestamp,
                positions=tuple(PositionCalibrationPatch.model_validate(merged[name]) for name in sorted(merged)),
                liquid_calibration=liquid,
            ).model_dump(mode="json", exclude_unset=True)
            # Fill the schema default explicitly in the persisted contract.
            revision["schema_version"] = "bioxp.machine_calibration.v1"
            project_calibration(snapshot, revision)
            return revision

        def checkpoint(revision):
            if run_id is None:
                return
            run = self.store.read_calibration_run(run_id)
            if run is None:
                raise KeyError(run_id)
            projected = project_calibration(snapshot, revision)
            after = comparison_values({"saved_positions": _thaw(projected.position_table),
                "saved_liquid_calibration": _thaw(projected.config_sections["calibration"]),
                "saved_revision_id": revision["revision_id"]})
            run["after"] = after
            run["measurements"].append({**(measurement or {}), "saved_revision_id": revision["revision_id"],
                                        "pending_restart": False})
            run["saved_revision_id"] = revision["revision_id"]
            self.store.write_calibration_run(run)

        committed = self.store.update_machine_calibration_revision(snapshot.lock_sha256, update,
                                                                   after_update=checkpoint)
        self._apply(committed)
        result = self.read()
        result["committed_revision_id"] = committed["revision_id"]
        return result

    def begin_run(self, *, machine_calibrated: bool) -> dict:
        with self.store._lock:
            before = self.read()
            run = CalibrationRunResponse(run_id=uuid4().hex,
                saved_revision_id=before["saved_revision_id"],
                active_revision_id=before["active_revision_id"], body_completed=False,
                decision_status="running", before=CalibrationComparison.model_validate(comparison_values(before)),
                after=CalibrationComparison.model_validate(comparison_values(before))).model_dump()
            run["pre_run_revision"] = before["saved_revision"]
            run.update(machine_calibrated=machine_calibrated, started_at=datetime.now(timezone.utc).isoformat())
            self.store.write_calibration_run(run)
            return run

    def update_run(self, run_id: str, **updates) -> dict:
        with self.store._lock:
            run = self.store.read_calibration_run(run_id)
            if run is None:
                raise KeyError(run_id)
            run.update(updates)
            self.store.write_calibration_run(run)
            return run

    def read_run(self, run_id: str) -> dict:
        with self.store._lock:
            run = self.store.read_calibration_run(run_id)
            if run is None:
                raise KeyError(run_id)
            run.pop("pre_run_revision", None)
            current = self.read()
            # before/after remain the compared run, not subsequent settings edits.
            return {**run, "saved_revision_id": current["saved_revision_id"],
                    "active_revision_id": current["active_revision_id"],
                    "pending_restart": current["pending_restart"]}

    def decide_run(self, run_id: str, decision: str, *, save_history=None, restore=None) -> dict:
        decision = CalibrationDecisionRequest.model_validate({"decision": decision}).decision
        with self.store._lock:
            run = self.store.read_calibration_run(run_id)
            if run is None:
                raise KeyError(run_id)
            try:
                if not run["machine_calibrated"]:
                    # Source no-backup result is not fabricated operator consent.
                    return self.read_run(run_id)
                if decision == "restore":
                    # Explicit OEM policy: full pre-run revision, overwriting later edits.
                    (self.restore if restore is None else restore)(run["pre_run_revision"])
                else:
                    if save_history is not None:
                        save_history()
                    run["accepted_history"] = run["after"]
                run.update(decision=decision, decision_status="accepted" if decision == "accept" else "restored",
                           comparison_error=None, decided_at=datetime.now(timezone.utc).isoformat())
                self.store.write_calibration_run(run)
            except Exception as exc:
                self.update_run(run_id, comparison_error=str(exc))
            return self.read_run(run_id)

    def restore(self, previous: dict | None) -> dict:
        if previous is not None:
            _revision(self.active_snapshot, previous)
        with self.store._lock:
            self.store.restore_machine_calibration_revision(self.active_snapshot.lock_sha256, previous)
            self._apply(previous)
            return self.read()
